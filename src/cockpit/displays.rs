//! Glass cockpit displays rendered into offscreen textures and mapped onto
//! the cockpit's screen meshes (`display` chunks of the imported model, see
//! `tools/fgimport`): PFD, ND, engine / warning display, system display,
//! standby (ISIS / IESS) and MCDU pages. The 2D instrument code
//! (`ui::glass`) draws them into a private egui context; the shapes are
//! tessellated and rendered (own egui-wgpu renderer, feathered antialiasing)
//! into an `Rgba8Unorm` texture that is sampled through an sRGB view with a
//! full mip chain (box-filtered in linear light), so the screens stay sharp
//! close up and do not shimmer from the far seat. Works on WebGPU.
//!
//! Each display is redrawn at ≥ 30 Hz (wall clock), a few per frame at most.

use crate::gfx_aircraft::{AircraftFrame, DisplaySet};
use crate::sim::Sim;
use crate::ui::map::Tiles;
use crate::ui::{self, canvas, glass, pal, polar};
use crate::world::World;
use egui::{Align2, Color32, Pos2, Rect, Shape, Stroke, Vec2, epaint};
use std::sync::Arc;
use web_time::Instant;

const FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
const SRGB: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;
/// Redraw interval per display (s): 30 Hz.
const PERIOD: f32 = 1.0 / 30.0;
/// Most displays redrawn in one frame (the rest wait for the next frame).
const MAX_PER_FRAME: usize = 3;
const SCREEN_BG: Color32 = Color32::from_rgb(6, 8, 11);

struct Target {
    kind: String,
    /// Own egui renderer: its vertex buffers are written with the queue, so
    /// several displays drawn in one encoder cannot share one.
    renderer: egui_wgpu::Renderer,
    size: [u32; 2],
    /// Level 0 as `Rgba8Unorm` (egui writes gamma-encoded colour).
    level0: wgpu::TextureView,
    /// Per level ≥ 1: (render view, bind group sampling the level above), sRGB.
    mips: Vec<(wgpu::TextureView, wgpu::BindGroup)>,
    /// Whole texture, sRGB (what the cockpit samples).
    view: wgpu::TextureView,
    last: Option<Instant>,
}

struct MipGen {
    pipe: wgpu::RenderPipeline,
    bgl: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
}

/// Offscreen glass-display renderer (one per app).
#[derive(Default)]
pub struct Displays {
    ctx: Option<egui::Context>,
    mip: Option<MipGen>,
    targets: Vec<Target>,
    /// The model the targets were made for.
    key: usize,
    generation: u64,
    set: Option<Arc<DisplaySet>>,
    /// Last frame: displays redrawn and CPU time spent (ms).
    pub stats: (usize, f32),
}

/// Texture size for a display of physical `aspect` (width / height).
pub fn texture_size(kind: &str, aspect: f32) -> [u32; 2] {
    let h = match kind {
        "isis" | "mcdu" => 448.0,
        _ => 704.0,
    };
    let a = aspect.clamp(0.3, 3.0);
    // Keep the long side at the nominal size.
    let (w, h) = if a >= 1.0 { (h, h / a) } else { (h * a, h) };
    [((w / 8.0).round() * 8.0).max(64.0) as u32, ((h / 8.0).round() * 8.0).max(64.0) as u32]
}

fn mip_levels(size: [u32; 2]) -> u32 {
    32 - size[0].max(size[1]).max(1).leading_zeros()
}

impl Displays {
    /// Redraw the displays that are due and attach their textures to `af`.
    #[allow(clippy::too_many_arguments)]
    pub fn update(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, enc: &mut wgpu::CommandEncoder, sim: &Sim, world: &World, tiles: &Tiles, af: &mut AircraftFrame) {
        let t0 = Instant::now();
        self.stats = (0, 0.0);
        let key = Arc::as_ptr(&af.asset) as usize;
        if key != self.key {
            self.key = key;
            self.targets.clear();
            self.set = None;
            // A fresh context re-sends the whole font atlas to the new renderers.
            self.ctx = None;
            self.mip_gen(device);
            let mg = self.mip.as_ref().unwrap();
            for (kind, aspect) in af.asset.displays() {
                self.targets.push(make_target(device, mg, &kind, texture_size(&kind, aspect)));
            }
            if !self.targets.is_empty() {
                self.generation += 1;
                let views = self.targets.iter().map(|t| (t.kind.clone(), t.view.clone())).collect();
                self.set = Some(Arc::new(DisplaySet { generation: self.generation, views, brightness: 1.0 }));
            }
        }
        if self.targets.is_empty() {
            return;
        }
        let now = Instant::now();
        // Oldest first; never-drawn first of all.
        let mut due: Vec<usize> = (0..self.targets.len()).filter(|&i| self.targets[i].last.is_none_or(|l| now.duration_since(l).as_secs_f32() >= PERIOD * 0.9)).collect();
        due.sort_by_key(|&i| self.targets[i].last);
        // SKYWARD_DISPLAYS_ALL=1: redraw every display every frame (worst-case timing).
        if std::env::var_os("SKYWARD_DISPLAYS_ALL").is_some() {
            due = (0..self.targets.len()).collect();
        }
        due.truncate(MAX_PER_FRAME.max(self.targets.iter().filter(|t| t.last.is_none()).count()).max(if std::env::var_os("SKYWARD_DISPLAYS_ALL").is_some() {
            usize::MAX
        } else {
            0
        }));
        if self.ctx.is_none() {
            let ctx = egui::Context::default();
            ui::install_fonts(&ctx);
            self.ctx = Some(ctx);
        }
        let route: Vec<(String, glam::Vec2)> = sim.bot.as_ref().map(|b| b.route()).unwrap_or_default();
        for i in due {
            let size = self.targets[i].size;
            let kind = self.targets[i].kind.clone();
            let ctx = self.ctx.as_ref().unwrap();
            let rect = Rect::from_min_size(Pos2::ZERO, Vec2::new(size[0] as f32, size[1] as f32));
            let raw = egui::RawInput { screen_rect: Some(rect), ..Default::default() };
            let full = ctx.run_ui(raw, |u| {
                let p = u.ctx().layer_painter(egui::LayerId::background());
                let p = p.with_clip_rect(rect);
                draw(&p, &kind, rect, sim, world, tiles, &route);
            });
            let prims = ctx.tessellate(full.shapes, full.pixels_per_point);
            // The font atlas changes go to every display's renderer.
            for t in &mut self.targets {
                for (id, deltas) in &full.textures_delta.set {
                    for d in deltas {
                        t.renderer.update_texture(device, queue, *id, d);
                    }
                }
            }
            let r = &mut self.targets[i].renderer;
            let sd = egui_wgpu::ScreenDescriptor { size_in_pixels: size, pixels_per_point: full.pixels_per_point };
            let cmds = r.update_buffers(device, queue, enc, &prims, &sd);
            if !cmds.is_empty() {
                queue.submit(cmds);
            }
            let t = &mut self.targets[i];
            let r = &mut t.renderer;
            {
                let mut pass = enc
                    .begin_render_pass(&wgpu::RenderPassDescriptor {
                        label: Some("cockpit display"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &t.level0,
                            depth_slice: None,
                            resolve_target: None,
                            ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color { r: 0.0018, g: 0.0024, b: 0.0034, a: 1.0 }), store: wgpu::StoreOp::Store },
                        })],
                        depth_stencil_attachment: None,
                        timestamp_writes: None,
                        occlusion_query_set: None,
                        multiview_mask: None,
                    })
                    .forget_lifetime();
                r.render(&mut pass, &prims, &sd);
            }
            for t in &mut self.targets {
                for id in &full.textures_delta.free {
                    t.renderer.free_texture(id);
                }
            }
            let t = &mut self.targets[i];
            // Mip chain (linear-light box filter through the sRGB views).
            let mg = self.mip.as_ref().unwrap();
            for (view, bg) in &t.mips {
                let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("cockpit display mip"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_pipeline(&mg.pipe);
                pass.set_bind_group(0, bg, &[]);
                pass.draw(0..3, 0..1);
            }
            t.last = Some(now);
            self.stats.0 += 1;
        }
        if let Some(set) = &self.set {
            let lum = brightness(crate::cockpit::daylight(sim.sun_dir().y));
            if (set.brightness - lum).abs() > 1e-4 {
                self.set = Some(Arc::new(DisplaySet { generation: set.generation, views: set.views.clone(), brightness: lum }));
            }
        }
        af.displays = self.set.clone();
        self.stats.1 = t0.elapsed().as_secs_f32() * 1000.0;
        if std::env::var_os("SKYWARD_DISPLAY_STATS").is_some() {
            eprintln!("cockpit displays: {} redrawn in {:.2} ms CPU", self.stats.0, self.stats.1);
        }
    }

    fn mip_gen(&mut self, device: &wgpu::Device) {
        if self.mip.is_some() {
            return;
        }
        self.mip = Some({
            let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("display mips"), source: wgpu::ShaderSource::Wgsl(MIP_WGSL.into()) });
            let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("display mips"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Float { filterable: true },
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });
            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("display mips"), bind_group_layouts: &[Some(&bgl)], immediate_size: 0 });
            let pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("display mips"),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some("vs"), compilation_options: Default::default(), buffers: &[] },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some("fs"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: SRGB, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            });
            let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("display mips"),
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                ..Default::default()
            });
            MipGen { pipe, bgl, sampler }
        });
    }
}

/// Screen luminance (scene radiance units) for the ambient daylight: the
/// auto-brightness of real display units — full by day, well dimmed at night.
pub fn brightness(daylight: f32) -> f32 {
    // 1 radiance unit ≈ 30 000 cd/m²: ~35 cd/m² at night; by day brighter than
    // a real unit's ~600 cd/m² so the screens read against the sunlit view.
    let d = daylight.clamp(0.0, 1.0);
    0.0012 * (1.0 - d) + 0.8 * d * d + 0.02 * d
}

const MIP_WGSL: &str = r#"
@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var smp: sampler;
struct VO { @builtin(position) p: vec4<f32>, @location(0) uv: vec2<f32> };
@vertex fn vs(@builtin(vertex_index) i: u32) -> VO {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: VO;
    o.p = vec4<f32>(uv * vec2<f32>(2.0, -2.0) + vec2<f32>(-1.0, 1.0), 0.0, 1.0);
    o.uv = uv;
    return o;
}
@fragment fn fs(v: VO) -> @location(0) vec4<f32> {
    // Bilinear at the corner of 2×2 source texels = their box average.
    return textureSampleLevel(src, smp, v.uv, 0.0);
}
"#;

fn make_target(device: &wgpu::Device, mg: &MipGen, kind: &str, size: [u32; 2]) -> Target {
    let levels = mip_levels(size);
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("cockpit display"),
        size: wgpu::Extent3d { width: size[0], height: size[1], depth_or_array_layers: 1 },
        mip_level_count: levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: FORMAT,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[SRGB],
    });
    let level =
        |fmt: wgpu::TextureFormat, l: u32| tex.create_view(&wgpu::TextureViewDescriptor { format: Some(fmt), base_mip_level: l, mip_level_count: Some(1), ..Default::default() });
    let level0 = level(FORMAT, 0);
    let mips = (1..levels)
        .map(|l| {
            let src = level(SRGB, l - 1);
            let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("display mip"),
                layout: &mg.bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&src) },
                    wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&mg.sampler) },
                ],
            });
            (level(SRGB, l), bg)
        })
        .collect();
    let view = tex.create_view(&wgpu::TextureViewDescriptor { format: Some(SRGB), ..Default::default() });
    let renderer = egui_wgpu::Renderer::new(device, FORMAT, egui_wgpu::RendererOptions { msaa_samples: 1, dithering: false, ..Default::default() });
    Target { kind: kind.to_string(), renderer, size, level0, mips, view, last: None }
}

// ---------------------------------------------------------------- pages

/// Draw display `kind` filling `r` (texture pixels).
pub fn draw(p: &egui::Painter, kind: &str, r: Rect, sim: &Sim, world: &World, tiles: &Tiles, route: &[(String, glam::Vec2)]) {
    p.rect_filled(r, 0.0, SCREEN_BG);
    // No power on its bus: the screen is dark (systems/).
    if !sim.ac.sys.display_on(kind) {
        p.rect_filled(r, 0.0, Color32::BLACK);
        return;
    }
    // The 2D displays draw their own bezel (7 px): push it just outside the texture.
    let bare = |q: Rect| q.expand(7.0);
    let portrait = r.height() > r.width() * 1.15;
    // Portrait units (E-Jet style): the main format in a square on top and a
    // second format below.
    let (top, bottom) = if portrait {
        let t = Rect::from_min_size(r.min, Vec2::splat(r.width()));
        (t, Some(Rect::from_min_max(Pos2::new(r.min.x, t.max.y), r.max)))
    } else {
        (r, None)
    };
    match kind {
        "pfd" | "isis" => {
            glass::pfd_unit(p, bare(top), sim, world, kind == "isis");
            if let Some(b) = bottom {
                hsi(p, b, sim);
            }
        }
        "nd" => {
            let mut cv = canvas::Frame::default();
            glass::nd(p, &mut cv, 1.0, bare(top), sim, world, tiles, route);
            if let Some(b) = bottom {
                status_strip(p, b, sim, world);
            }
        }
        "ewd" => glass::ewd(p, bare(r), sim),
        "sd" => system_display(p, r, sim, world),
        "mcdu" => mcdu(p, r, sim, world),
        _ => {}
    }
}

fn fi(s: f32) -> egui::FontId {
    ui::f_inst(s)
}
fn fd(s: f32) -> egui::FontId {
    ui::f_digits_b(s)
}

/// Compass rose with heading bug and lubber line (lower half of a portrait PFD).
fn hsi(p: &egui::Painter, r: Rect, sim: &Sim) {
    let u = r.width() / 100.0;
    let c = Pos2::new(r.center().x, r.min.y + r.height() * 0.62);
    let rad = (r.height() * 0.52).min(r.width() * 0.4);
    // Heading from the heading reference (systems/): red HDG flag when it has failed.
    let ad = sim.ac.sys.air_data(&sim.ac);
    if ad.has(crate::systems::flag::HDG) {
        let rr = p.text(c, Align2::CENTER_CENTER, "HDG", fi(6.0 * u), pal::RED);
        p.rect_stroke(rr.expand(1.0 * u), 0.0, Stroke::new(1.4, pal::RED), egui::StrokeKind::Outside);
        return;
    }
    let hdg = ad.heading;
    p.circle_filled(c, rad, Color32::from_rgb(14, 17, 22));
    p.circle_stroke(c, rad, Stroke::new(1.6, pal::WHITE));
    for d in 0..72 {
        let deg = d as f32 * 5.0;
        let a = deg.to_radians() - hdg;
        let len = if d % 2 == 0 { 3.0 * u } else { 1.6 * u };
        p.line_segment([polar(c, rad, a), polar(c, rad - len, a)], Stroke::new(1.4, pal::WHITE));
        if d % 6 == 0 {
            let s = match d / 6 {
                0 => "N".to_string(),
                3 => "E".to_string(),
                6 => "S".to_string(),
                9 => "W".to_string(),
                k => format!("{}", k * 3),
            };
            p.text(polar(c, rad - 7.0 * u, a), Align2::CENTER_CENTER, s, fi(4.4 * u), pal::WHITE);
        }
    }
    if let Some(h) = sim.ap.hdg {
        let a = h - hdg;
        p.add(Shape::convex_polygon(vec![polar(c, rad + 0.2 * u, a - 0.05), polar(c, rad + 0.2 * u, a + 0.05), polar(c, rad - 2.4 * u, a)], pal::CYAN, Stroke::NONE));
    }
    p.add(Shape::convex_polygon(vec![polar(c, rad + 3.4 * u, -0.05), polar(c, rad + 3.4 * u, 0.05), polar(c, rad + 0.6 * u, 0.0)], pal::WHITE, Stroke::NONE));
    // Aircraft symbol.
    let w = pal::WHITE;
    p.line_segment([c + Vec2::new(0.0, -5.0 * u), c + Vec2::new(0.0, 6.0 * u)], Stroke::new(2.0, w));
    p.line_segment([c + Vec2::new(-5.0 * u, -0.5 * u), c + Vec2::new(5.0 * u, -0.5 * u)], Stroke::new(2.0, w));
    p.line_segment([c + Vec2::new(-2.2 * u, 5.0 * u), c + Vec2::new(2.2 * u, 5.0 * u)], Stroke::new(2.0, w));
    let v = glam::Vec2::new(sim.ac.vel.x, sim.ac.vel.z);
    p.text(Pos2::new(r.min.x + 3.0 * u, r.min.y + 4.0 * u), Align2::LEFT_TOP, format!("GS {:.0}", v.length() / crate::flight::KT), fd(4.6 * u), pal::GREEN);
    p.text(Pos2::new(r.max.x - 3.0 * u, r.min.y + 4.0 * u), Align2::RIGHT_TOP, format!("HDG {:03.0}", hdg.to_degrees().rem_euclid(360.0)), fd(4.6 * u), pal::GREEN);
}

/// Bottom strip of a portrait MFD: temperatures, clock, gross weight.
fn status_strip(p: &egui::Painter, r: Rect, sim: &Sim, world: &World) {
    let u = r.width() / 100.0;
    let ac = &sim.ac;
    let rows = [
        ("SAT", format!("{:+.0}°C", ac.oat())),
        ("QNH", format!("{:.0}", world.wx.qnh_hpa)),
        ("TIME", format!("{:02}:{:02}", sim.hours.rem_euclid(24.0) as u32, ((sim.hours.rem_euclid(1.0)) * 60.0) as u32)),
        ("GW", format!("{:.1} T", sim.def().mass / 1000.0)),
    ];
    let y0 = r.min.y + 6.0 * u;
    let dy = (r.height() - 8.0 * u) / rows.len() as f32;
    for (k, (l, v)) in rows.iter().enumerate() {
        let y = y0 + dy * (k as f32 + 0.5);
        p.text(Pos2::new(r.min.x + 6.0 * u, y), Align2::LEFT_CENTER, *l, fi(4.4 * u), pal::CYAN);
        p.text(Pos2::new(r.max.x - 6.0 * u, y), Align2::RIGHT_CENTER, v, fd(5.0 * u), pal::GREEN);
    }
}

/// Which system page the SD shows: the one with a failure (ECAM auto paging), else ENGINE.
fn sd_page(sim: &Sim) -> &'static str {
    let sys = &sim.ac.sys;
    if !sys.init {
        return "ENGINE";
    }
    let alerts = crate::systems::alerts::alerts(sys, sim.def(), &sim.ac, &sim.ctl);
    for a in &alerts {
        let t = a.title.as_str();
        if t.contains("HYD") {
            return "HYD";
        }
        if t.starts_with("ELEC") || t.contains("GEN") || t.starts_with("IDG") {
            return "ELEC";
        }
        if t.starts_with("FUEL") {
            return "FUEL";
        }
    }
    "ENGINE"
}

/// SD pages for hydraulics, electrics and fuel (systems/).
fn sd_system_page(p: &egui::Painter, r: Rect, sim: &Sim, page: &str) {
    let u = r.width() / 100.0;
    let sys = &sim.ac.sys;
    let sd = &sys.sd;
    let col = |ok: bool| if ok { pal::GREEN } else { pal::AMBER };
    match page {
        "HYD" => {
            let n = sd.hyd.len().max(1);
            for (k, h) in sd.hyd.iter().enumerate() {
                let x = r.min.x + r.width() * (k as f32 + 0.5) / n as f32;
                let ok = sys.hyd[k] > 0.5;
                p.text(Pos2::new(x, r.min.y + 20.0 * u), Align2::CENTER_CENTER, h.name, fi(7.0 * u), col(ok));
                p.text(Pos2::new(x, r.min.y + 32.0 * u), Align2::CENTER_CENTER, format!("{:.0}", (sys.hyd_psi(k) / 50.0).round() * 50.0), fd(6.0 * u), col(ok));
                p.line_segment([Pos2::new(x, r.min.y + 37.0 * u), Pos2::new(x, r.min.y + 58.0 * u)], Stroke::new(2.0, col(ok)));
                let src = if !h.pumps.is_empty() {
                    "ENG"
                } else if h.rat {
                    "ELEC / RAT"
                } else {
                    "ELEC"
                };
                p.text(Pos2::new(x, r.min.y + 63.0 * u), Align2::CENTER_CENTER, src, fi(3.6 * u), pal::WHITE);
            }
            if sd.ptu {
                p.text(Pos2::new(r.center().x, r.min.y + 45.0 * u), Align2::CENTER_CENTER, "PTU", fi(3.8 * u), pal::WHITE);
            }
        }
        "ELEC" => {
            let soc = sys.soc;
            p.text(Pos2::new(r.center().x, r.min.y + 18.0 * u), Align2::CENTER_CENTER, "BAT", fi(4.4 * u), pal::WHITE);
            p.text(Pos2::new(r.center().x, r.min.y + 25.0 * u), Align2::CENTER_CENTER, format!("{:.1} V   {:+.0} A", sys.volts, sys.amps), fd(4.4 * u), col(soc > 0.3));
            for i in 0..sys.n_eng {
                let x = r.min.x + r.width() * (i as f32 + 0.5) / sys.n_eng.max(1) as f32;
                let on = sys.gen_on[i];
                p.text(Pos2::new(x, r.min.y + 42.0 * u), Align2::CENTER_CENTER, format!("GEN {}", i + 1), fi(4.4 * u), col(on));
                p.text(Pos2::new(x, r.min.y + 49.0 * u), Align2::CENTER_CENTER, if on { "115 V 400 HZ" } else { "FAULT" }, fd(3.6 * u), col(on));
            }
            if sd.apu {
                p.text(
                    Pos2::new(r.center().x, r.min.y + 60.0 * u),
                    Align2::CENTER_CENTER,
                    format!("APU GEN  {}", if sys.apu > 0.95 { "AVAIL" } else { "OFF" }),
                    fi(3.8 * u),
                    col(sys.apu > 0.95),
                );
            }
            if sys.rat_out {
                p.text(Pos2::new(r.center().x, r.min.y + 67.0 * u), Align2::CENTER_CENTER, "EMER GEN (RAT)", fi(3.8 * u), pal::GREEN);
            }
        }
        _ => {
            let nt = sd.tanks.len().min(3).max(1);
            for (k, t) in sd.tanks.iter().enumerate().take(3) {
                let x = r.min.x + r.width() * (k as f32 + 0.5) / nt as f32;
                p.text(Pos2::new(x, r.min.y + 24.0 * u), Align2::CENTER_CENTER, t.0, fi(4.4 * u), pal::WHITE);
                let low = sys.tank[k] < 0.1 * sys.tank_cap[k];
                p.text(Pos2::new(x, r.min.y + 32.0 * u), Align2::CENTER_CENTER, format!("{:.0}", (sys.tank[k] / 10.0).round() * 10.0), fd(5.6 * u), col(!low));
            }
            p.text(Pos2::new(r.center().x, r.min.y + 46.0 * u), Align2::CENTER_CENTER, format!("FOB {:.0} KG", sys.fuel_total()), fd(4.6 * u), pal::GREEN);
            if sys.sw.crossfeed {
                p.text(Pos2::new(r.center().x, r.min.y + 54.0 * u), Align2::CENTER_CENTER, "X FEED OPEN", fi(3.8 * u), pal::GREEN);
            }
        }
    }
}

/// Airbus-style system display: fuel / engines secondary page and the status strip.
fn system_display(p: &egui::Painter, r: Rect, sim: &Sim, world: &World) {
    let u = r.width() / 100.0;
    let ac = &sim.ac;
    let def = sim.def();
    let n = def.engine_count.clamp(1, 4);
    let page = sd_page(sim);
    p.text(Pos2::new(r.center().x, r.min.y + 5.0 * u), Align2::CENTER_CENTER, page, fi(5.0 * u), pal::WHITE);
    if page != "ENGINE" {
        sd_system_page(p, r, sim, page);
    }
    p.line_segment([Pos2::new(r.center().x - 9.0 * u, r.min.y + 8.0 * u), Pos2::new(r.center().x + 9.0 * u, r.min.y + 8.0 * u)], Stroke::new(1.2, pal::WHITE));
    let rows: [(&str, &str); 4] = [("F.USED", "KG"), ("OIL", "PSI"), ("N2", "%"), ("VIB N1", "")];
    let rows = if page == "ENGINE" { &rows[..] } else { &rows[..0] };
    let col_x = |i: usize| r.min.x + r.width() * (0.22 + 0.56 * if n > 1 { i as f32 / (n - 1) as f32 } else { 0.5 });
    for (k, (l, unit)) in rows.iter().enumerate() {
        let y = r.min.y + (20.0 + 13.0 * k as f32) * u;
        p.text(Pos2::new(r.center().x, y), Align2::CENTER_CENTER, *l, fi(3.8 * u), pal::WHITE);
        p.text(Pos2::new(r.center().x, y + 4.4 * u), Align2::CENTER_CENTER, *unit, fi(3.2 * u), pal::CYAN);
        for i in 0..n {
            let v = match k {
                0 => format!("{:.0}", (sim.time / 3600.0 * ac.fuel_flow(i)).max(0.0) / 10.0 * 10.0),
                1 => format!("{:.0}", if ac.engine_running(i) { 30.0 + 25.0 * ac.n1(i) } else { 0.0 }),
                2 => format!("{:.1}", (ac.n1(i) * 100.0 * 0.9 + 10.0).min(105.0) * if ac.engine_running(i) { 1.0 } else { 0.0 }),
                _ => format!("{:.1}", if ac.engine_running(i) { 0.3 + 0.4 * ac.n1(i) } else { 0.0 }),
            };
            p.text(Pos2::new(col_x(i), y + 2.0 * u), Align2::CENTER_CENTER, v, fd(5.2 * u), pal::GREEN);
        }
    }
    // Status strip.
    let y = r.max.y - 16.0 * u;
    p.line_segment([Pos2::new(r.min.x + 2.0 * u, y), Pos2::new(r.max.x - 2.0 * u, y)], Stroke::new(1.2, pal::WHITE));
    let tat = ac.oat() + ac.tas() * ac.tas() / 2010.0;
    let items = [
        (format!("TAT {:+.0} °C", tat), format!("SAT {:+.0} °C", ac.oat())),
        (format!("{:02} H {:02}", sim.hours.rem_euclid(24.0) as u32, ((sim.hours.rem_euclid(1.0)) * 60.0) as u32), String::new()),
        (format!("GW {:.0} KG", def.mass + ac.sys.fx.mass_delta), format!("QNH {:.0}", world.wx.qnh_hpa)),
    ];
    for (k, (a, b)) in items.iter().enumerate() {
        let x = r.min.x + r.width() * (0.17 + 0.33 * k as f32);
        p.text(Pos2::new(x, y + 5.0 * u), Align2::CENTER_CENTER, a, fd(3.8 * u), pal::GREEN);
        p.text(Pos2::new(x, y + 10.5 * u), Align2::CENTER_CENTER, b, fd(3.8 * u), pal::GREEN);
    }
}

/// MCDU progress page.
fn mcdu(p: &egui::Painter, r: Rect, sim: &Sim, world: &World) {
    let u = r.width() / 100.0;
    let ac = &sim.ac;
    let line = |row: f32| r.min.y + (6.0 + row * 6.6) * u;
    let alt = ac.pos.y * 3.280_84;
    p.text(Pos2::new(r.center().x, line(0.0)), Align2::CENTER_CENTER, "PROG", fi(5.4 * u), pal::WHITE);
    let crz = ((alt / 1000.0).ceil() * 10.0).max(50.0);
    let rows: [(&str, String, Color32); 5] = [
        ("CRZ", format!("FL{crz:03.0}"), pal::CYAN),
        ("GS", format!("{:.0} KT", glam::Vec2::new(ac.vel.x, ac.vel.z).length() / crate::flight::KT), pal::GREEN),
        ("ALT", format!("{:.0} FT", (alt / 10.0).round() * 10.0), pal::GREEN),
        ("WIND", format!("{:03.0}°/{:.0}", world.wx.wind_from, world.wx.wind_kt), pal::GREEN),
        ("QNH", format!("{:.0}", world.wx.qnh_hpa), pal::CYAN),
    ];
    for (k, (l, v, c)) in rows.iter().enumerate() {
        let y = line(1.6 + k as f32 * 2.0);
        p.text(Pos2::new(r.min.x + 4.0 * u, y), Align2::LEFT_CENTER, *l, fi(4.0 * u), pal::WHITE);
        p.text(Pos2::new(r.min.x + 4.0 * u, y + 6.0 * u), Align2::LEFT_CENTER, v, fd(5.6 * u), *c);
    }
    let _ = epaint::Shadow::NONE;
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sizes_follow_the_screen_aspect() {
        let s = texture_size("pfd", 1.0);
        assert_eq!(s[0], s[1]);
        let p = texture_size("pfd", 0.716);
        assert!(p[1] > p[0] && (p[0] as f32 / p[1] as f32 - 0.716).abs() < 0.02, "{p:?}");
        assert!(texture_size("mcdu", 1.06)[0] < 704);
        assert_eq!(mip_levels([704, 704]), 10);
        assert_eq!(mip_levels([504, 704]), 10);
    }

    #[test]
    fn brightness_dims_at_night() {
        assert!(brightness(0.0) < 0.01 * brightness(1.0));
        assert!(brightness(1.0) > 0.4);
    }
}
