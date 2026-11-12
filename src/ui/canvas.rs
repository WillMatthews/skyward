//! Offscreen vector canvases: shapes drawn by the UI code into a 4× MSAA
//! texture (own egui-wgpu renderer per canvas), then shown in the main UI as
//! an image — so map polygons, coastlines and roads get GPU antialiasing
//! instead of jagged triangle edges.
//!
//! Per frame: the UI pushes [`Job`]s into [`Frame`] (and draws the canvas's
//! texture where it wants it); `main.rs` then calls [`Canvases::render`]
//! before the main egui pass.

use egui::{epaint, Color32, Pos2, Rect, Shape, TextureId, Vec2};
use std::collections::HashMap;

pub const MSAA: u32 = 4;
const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;

/// One canvas to draw this frame. Shapes are in canvas points (origin top-left).
pub struct Job {
    pub key: &'static str,
    pub size: Vec2,
    pub ppp: f32,
    pub clear: Color32,
    pub shapes: Vec<Shape>,
}

/// UI-side handle: known texture ids + this frame's jobs.
#[derive(Default)]
pub struct Frame {
    pub ids: HashMap<&'static str, TextureId>,
    pub jobs: Vec<Job>,
    /// Per canvas: (frames since redrawn, size) — see [`Frame::due`].
    ages: HashMap<&'static str, (u32, Vec2)>,
}

/// Browser: moving-map canvases are redrawn every this many frames (their
/// shapes and tessellation are a large share of the main thread's frame).
const WEB_REDRAW_EVERY: u32 = 3;

impl Frame {
    /// Should canvas `key` be redrawn this frame? Always natively; in the
    /// browser every few frames (and whenever it changes size), the texture
    /// showing the previous drawing in between.
    pub fn due(&mut self, key: &'static str, size: Vec2) -> bool {
        if !crate::platform::WEB || !self.ids.contains_key(key) {
            return true;
        }
        let a = self.ages.entry(key).or_insert((0, size));
        a.0 += 1;
        if a.0 >= WEB_REDRAW_EVERY || a.1 != size {
            *a = (0, size);
            return true;
        }
        false
    }

    /// Queue a canvas; returns its texture id once it exists (from the next frame on).
    pub fn draw(&mut self, key: &'static str, size: Vec2, ppp: f32, clear: Color32, shapes: Vec<Shape>) -> Option<TextureId> {
        self.jobs.retain(|j| j.key != key);
        self.jobs.push(Job { key, size, ppp, clear, shapes });
        self.ids.get(key).copied()
    }
}

/// Draw a canvas texture into `rect` (full texture).
pub fn image(p: &egui::Painter, id: TextureId, rect: Rect, tint: Color32) {
    p.image(id, rect, Rect::from_min_max(Pos2::ZERO, Pos2::new(1.0, 1.0)), tint);
}

/// Draw a canvas texture clipped to a circle (centre, radius) inside `rect`.
pub fn image_circle(p: &egui::Painter, id: TextureId, rect: Rect, c: Pos2, r: f32, tint: Color32) {
    let mut m = epaint::Mesh::with_texture(id);
    let uv = |q: Pos2| Pos2::new((q.x - rect.min.x) / rect.width(), (q.y - rect.min.y) / rect.height());
    m.vertices.push(epaint::Vertex { pos: c, uv: uv(c), color: tint });
    let n = 128;
    for i in 0..=n {
        let a = i as f32 / n as f32 * std::f32::consts::TAU;
        let q = Pos2::new(c.x + r * a.cos(), c.y + r * a.sin());
        m.vertices.push(epaint::Vertex { pos: q, uv: uv(q), color: tint });
        if i > 0 {
            m.add_triangle(0, i, i + 1);
        }
    }
    p.add(Shape::mesh(m));
}

struct Target {
    renderer: egui_wgpu::Renderer,
    size: [u32; 2],
    msaa: wgpu::TextureView,
    resolve: wgpu::TextureView,
    id: Option<TextureId>,
}

#[derive(Default)]
pub struct Canvases {
    targets: HashMap<&'static str, Target>,
}

fn tex(device: &wgpu::Device, size: [u32; 2], samples: u32, label: &str) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: samples,
            dimension: wgpu::TextureDimension::D2,
            format: FORMAT,
            usage: if samples > 1 { wgpu::TextureUsages::RENDER_ATTACHMENT } else { wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING },
            view_formats: &[],
        })
        .create_view(&Default::default())
}

impl Canvases {
    /// Render this frame's jobs. New / resized canvases are (re)registered
    /// with the main egui renderer; their ids go back into `frame.ids`.
    pub fn render(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, enc: &mut wgpu::CommandEncoder, main: &mut egui_wgpu::Renderer, frame: &mut Frame) -> Vec<wgpu::CommandBuffer> {
        let mut extra = Vec::new();
        for job in frame.jobs.drain(..) {
            let size = [((job.size.x * job.ppp).round() as u32).clamp(1, 8192), ((job.size.y * job.ppp).round() as u32).clamp(1, 8192)];
            let t = self.targets.entry(job.key).or_insert_with(|| {
                let mut renderer = egui_wgpu::Renderer::new(device, FORMAT, egui_wgpu::RendererOptions { msaa_samples: MSAA, dithering: false, ..Default::default() });
                // Shapes use the font atlas's white texel: a 1×1 white texture stands in.
                let white = epaint::ImageDelta::full(epaint::ImageData::Color(std::sync::Arc::new(epaint::ColorImage::filled([1, 1], Color32::WHITE))), epaint::textures::TextureOptions::LINEAR);
                renderer.update_texture(device, queue, TextureId::default(), &white);
                Target { renderer, size: [0, 0], msaa: tex(device, [1, 1], MSAA, "canvas msaa"), resolve: tex(device, [1, 1], 1, "canvas"), id: None }
            });
            if t.size != size {
                t.size = size;
                t.msaa = tex(device, size, MSAA, "canvas msaa");
                t.resolve = tex(device, size, 1, "canvas");
                match t.id {
                    Some(id) => main.update_egui_texture_from_wgpu_texture(device, &t.resolve, wgpu::FilterMode::Linear, id),
                    None => t.id = Some(main.register_native_texture(device, &t.resolve, wgpu::FilterMode::Linear)),
                }
            }
            if let Some(id) = t.id {
                frame.ids.insert(job.key, id);
            }
            let opts = epaint::TessellationOptions { feathering: false, ..Default::default() };
            let mut tess = epaint::Tessellator::new(job.ppp, opts, [1, 1], Vec::new());
            let clip = Rect::from_min_size(Pos2::ZERO, job.size);
            let prims = tess.tessellate_shapes(job.shapes.into_iter().map(|shape| epaint::ClippedShape { clip_rect: clip, shape, transform_after_tessellation: egui::emath::TSTransform::IDENTITY }).collect());
            let sd = egui_wgpu::ScreenDescriptor { size_in_pixels: size, pixels_per_point: job.ppp };
            extra.extend(t.renderer.update_buffers(device, queue, enc, &prims, &sd));
            let c = job.clear.to_normalized_gamma_f32();
            let mut pass = enc
                .begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("canvas"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: &t.msaa,
                        depth_slice: None,
                        resolve_target: Some(&t.resolve),
                        ops: wgpu::Operations {
                            load: wgpu::LoadOp::Clear(wgpu::Color { r: c[0] as f64, g: c[1] as f64, b: c[2] as f64, a: c[3] as f64 }),
                            store: wgpu::StoreOp::Discard,
                        },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                })
                .forget_lifetime();
            t.renderer.render(&mut pass, &prims, &sd);
        }
        extra
    }
}
