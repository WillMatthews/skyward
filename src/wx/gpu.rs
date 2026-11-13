//! GPU side of the weather drama: what the renderer needs from the weather
//! each frame ([`WxFrame`], built by [`prepare`] from the world and the sim),
//! the lightning channels (bright ribbons with a glow, depth-tested against
//! the scene and hidden by the cloud in front of them) and the windscreen
//! pass (rain drops refracting the view, wiped clean by the blades, misting,
//! frost creeping in from the frame).
//!
//! Order in gfx.rs: clouds (which add the flash light inside the cloud and
//! in the air, `CloudRenderer::set_flashes`) → lightning channels → fog,
//! effects, rain → windscreen (cockpit view) → light halos → bloom.

use super::glass::{GlassState, Panel};
use super::storm::{FlashKind, Seg};
use crate::clouds::{FlashLight, LentInput, StormInput};
use glam::{Mat4, Vec2, Vec3};
use std::collections::HashMap;
use std::sync::Arc;

/// Lightning channel luminance (cd/m², a return stroke's peak) and the
/// flash's luminous intensity (cd) as it lights the cloud around it.
pub const CHANNEL_CDM2: f32 = 1.5e7;
pub const FLASH_CD: f32 = 2.5e9;
/// Real channel core width (m): drawn at least ~1.5 px wide, dimmed to keep its energy.
const CORE_W: f32 = 3.0;

/// A flash's channel to draw this frame.
#[derive(Clone, Debug)]
pub struct BoltDraw {
    pub kind: FlashKind,
    /// 0..~2 (return-stroke brightness now).
    pub bright: f32,
    pub segs: Arc<Vec<Seg>>,
}

/// The windscreen for the glass pass (eye-relative world).
#[derive(Clone, Debug)]
pub struct GlassDraw {
    pub panels: Vec<Panel>,
    /// Wiper pivot (pane uv, m), arm length (m), park angle and sweep sign per pane.
    pub wipers: Vec<(Vec2, f32, f32, f32)>,
    pub state: GlassState,
    pub time: f32,
}

/// Everything weather-related the renderer needs this frame.
#[derive(Clone, Debug, Default)]
pub struct WxFrame {
    pub storms: StormInput,
    pub lent: LentInput,
    pub bolts: Vec<BoltDraw>,
    /// Flashes lighting the clouds (world space).
    pub lights: [FlashLight; 2],
    /// Scene radiance from the lit cloud base overhead (scene units, before pre-exposure).
    pub ambient: f32,
    pub glass: Option<GlassDraw>,
}

/// Channel shapes by flash key (they are deterministic, compute once).
fn channel_cached(f: &super::storm::Flash) -> Arc<Vec<Seg>> {
    use std::sync::{Mutex, OnceLock};
    static CACHE: OnceLock<Mutex<HashMap<u64, (f32, Arc<Vec<Seg>>)>>> = OnceLock::new();
    let c = CACHE.get_or_init(Default::default);
    let mut m = c.lock().unwrap_or_else(|e| e.into_inner());
    m.retain(|_, (t0, _)| (f.t0 - *t0).abs() < 30.0);
    m.entry(f.key).or_insert_with(|| (f.t0, Arc::new(super::storm::channel(f)))).1.clone()
}

/// Build this frame's weather inputs (storm cells, lenticulars, flashes, the
/// windscreen) and append the wiper arms to the dynamic mesh.
pub fn prepare(
    world: &crate::world::World,
    sim: &crate::sim::Sim,
    eye: Vec3,
    cockpit: bool,
    af: Option<&crate::gfx_aircraft::AircraftFrame>,
    dynm: &mut crate::scene::MeshData,
) -> WxFrame {
    let w = &world.wxs;
    let t = w.time;
    let mut out = WxFrame::default();
    if w.storms.active() {
        let spec = w.storms.spec;
        // Rain drifts with the low-level wind relative to the storm (which
        // moves with the wind aloft) as it falls at ~8 m/s: slanted shafts.
        let shear = (world.wind.surface - world.wind.aloft) * 0.05;
        out.storms = StormInput { cells: w.gpu_cells(Vec2::new(eye.x, eye.z)), version: w.version, shaft_bottom: world.terrain_base - 20.0, shear, dry: spec.dry };
        // Re-bake when the set of nearby cells changes.
        out.storms.version = out.storms.version.wrapping_mul(1_000_003) ^ out.storms.cells.iter().fold(0u64, |h, c| h.wrapping_mul(31).wrapping_add(c.id as u64));
    }
    if let Some((grid, origin, size)) = w.waves.lenticular_grid() {
        out.lent = LentInput {
            grid: grid.to_vec(),
            res: super::mountain::RES as u32,
            origin: world.from_field(origin),
            size,
            alt: world.terrain_base + w.waves.spec.lent_alt,
            thick: 220.0,
            version: w.wave_version,
        };
    }
    // Lightning: channels and the light they cast.
    let mut lights: Vec<(f32, FlashLight)> = Vec::new();
    let mut ambient = 0.0;
    for f in &w.flashes {
        let b = f.brightness(t);
        if b <= 0.002 {
            continue;
        }
        let segs = channel_cached(f);
        let centre = match f.kind {
            FlashKind::Ground => f.origin.lerp(f.strike, 0.35),
            FlashKind::Cloud => f.origin,
        };
        let i = FLASH_CD * crate::atmosphere::LUX * b * f.energy;
        let d = centre.distance(eye);
        ambient += i / (std::f32::consts::PI * (d * d + 1.0e6)) * 0.3;
        lights.push((b, FlashLight { pos: centre, intensity: i }));
        out.bolts.push(BoltDraw { kind: f.kind, bright: b, segs });
    }
    lights.sort_by(|a, b| b.0.total_cmp(&a.0));
    for (k, l) in lights.into_iter().take(2).enumerate() {
        out.lights[k] = l.1;
    }
    out.ambient = ambient;
    // The windscreen (cockpit view) and the wipers (any view).
    if let Some(af) = af {
        let def = sim.def();
        let panels = super::glass::panels(&af.asset);
        let (pos, rot) = sim.render_pose();
        let airliner = def.category == crate::flight::Category::Airliner && panels.len() == 2;
        if airliner {
            let xf = super::glass::body_xf(pos, rot, eye);
            super::glass::wiper_mesh(&panels, sim.wxc.glass.wipers.angle(), dynm, xf);
        }
        if cockpit && !panels.is_empty() && sim.wxc.glass.visible() {
            let wipers = panels
                .iter()
                .map(|p| {
                    let (pivot, len) = p.wiper();
                    let inboard = if p.origin.x >= 0.0 { -1.0 } else { 1.0 } * p.u.x.signum();
                    let a0 = if inboard > 0.0 { 0.06 } else { std::f32::consts::PI - 0.06 };
                    (pivot, if airliner { len } else { 0.0 }, a0, inboard.signum())
                })
                .collect();
            out.glass = Some(GlassDraw { panels: super::glass::world_panels(&panels, pos, rot, eye), wipers, state: sim.wxc.glass, time: sim.time });
        }
    }
    out
}

// ------------------------------------------------------------------ renderer

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct BoltU {
    view_proj: [[f32; 4]; 4],
    /// w, h, pre-exposure, 0
    screen: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct BoltVert {
    pos: [f32; 3],
    /// across the ribbon -1..1, radiance (pre-exposed), glow (1) / core (0)
    k: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct GlassU {
    inv_view_proj: [[f32; 4]; 4],
    /// w, h, 1/w, 1/h
    screen: [f32; 4],
    /// time, rain 0..1, airspeed m/s, wet time s
    state: [f32; 4],
    /// fog, frost, ice, focal length (px)
    state2: [f32; 4],
    /// wiper phase, running, stroke s, since stop s
    wiper: [f32; 4],
    /// panes: origin (xyz, valid), normal, u, v, (lo, hi), (pivot uv, arm, park angle | sign)
    panes: [[[f32; 4]; 6]; 2],
}

pub struct WxGpu {
    bolt_pipe: wgpu::RenderPipeline,
    bolt_ubuf: wgpu::Buffer,
    bolt_vbuf: Option<(wgpu::Buffer, usize)>,
    glass_pipe: wgpu::RenderPipeline,
    blit_pipe: wgpu::RenderPipeline,
    glass_ubuf: wgpu::Buffer,
    glass_tex: Option<(wgpu::TextureView, u32, u32)>,
    samp: wgpu::Sampler,
}

const HDR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;

impl WxGpu {
    pub fn new(device: &wgpu::Device) -> WxGpu {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("wx"), source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/wx.wgsl").into()) });
        let add = wgpu::BlendState {
            color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
            alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
        };
        let over = wgpu::BlendState {
            color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::SrcAlpha, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add },
            alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
        };
        let pipe = |label: &str, vs: &str, fs: &str, blend: Option<wgpu::BlendState>, buffers: &[Option<wgpu::VertexBufferLayout>]| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: None,
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: HDR, blend, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3];
        let vl = wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<BoltVert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS };
        let ubuf = |label: &str, size: usize| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: size as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        WxGpu {
            bolt_pipe: pipe("lightning", "vs_bolt", "fs_bolt", Some(add), &[Some(vl)]),
            bolt_ubuf: ubuf("lightning u", std::mem::size_of::<BoltU>()),
            bolt_vbuf: None,
            glass_pipe: pipe("windscreen", "vs_full", "fs_glass", None, &[]),
            blit_pipe: pipe("windscreen blit", "vs_full", "fs_blit", Some(over), &[]),
            glass_ubuf: ubuf("windscreen u", std::mem::size_of::<GlassU>()),
            glass_tex: None,
            samp: device.create_sampler(&wgpu::SamplerDescriptor {
                label: Some("wx"),
                mag_filter: wgpu::FilterMode::Linear,
                min_filter: wgpu::FilterMode::Linear,
                address_mode_u: wgpu::AddressMode::ClampToEdge,
                address_mode_v: wgpu::AddressMode::ClampToEdge,
                ..Default::default()
            }),
        }
    }

    /// Lightning channels into `hdr` (after the clouds' composite). `vp` =
    /// eye-relative view-projection, `px_rad` = pixels per radian.
    #[allow(clippy::too_many_arguments)]
    pub fn bolts(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        f: &WxFrame,
        eye: Vec3,
        vp: Mat4,
        px_rad: f32,
        pre: f32,
        hdr: &wgpu::TextureView,
        dist: &wgpu::TextureView,
        cloud: Option<(&wgpu::TextureView, &wgpu::TextureView)>,
        size: (u32, u32),
    ) {
        if f.bolts.is_empty() {
            return;
        }
        let Some((ccol, cdep)) = cloud else { return };
        let mut verts: Vec<BoltVert> = Vec::new();
        let lum = CHANNEL_CDM2 * crate::atmosphere::LUX * pre;
        for b in &f.bolts {
            let hidden = if b.kind == FlashKind::Cloud { 0.35 } else { 1.0 };
            for s in b.segs.iter() {
                let (a, c) = (s.a - eye, s.b - eye);
                let mid = (a + c) * 0.5;
                let d = mid.length().max(1.0);
                let dir = (c - a).normalize_or_zero();
                let side = dir.cross(mid / d).normalize_or_zero();
                if side == Vec3::ZERO {
                    continue;
                }
                // Core: at least ~1.6 px wide; radiance scaled to keep the real channel's energy.
                let px_w = d / px_rad;
                let core_w = CORE_W.max(1.6 * px_w);
                let core_l = lum * b.bright * s.bright * hidden * (CORE_W / core_w).min(1.0);
                // Glow (the hot air / light scattered in rain around it): ~40 m, dimmer.
                let glow_w = (45.0 * (0.4 + 0.6 * s.bright)).max(5.0 * px_w);
                let glow_l = core_l * 0.03 * (core_w / glow_w).sqrt();
                for (w, l, g) in [(glow_w, glow_l, 1.0), (core_w, core_l, 0.0)] {
                    let h = side * w * 0.5;
                    let q = [a - h, a + h, c + h, a - h, c + h, c - h];
                    let k = [-1.0, 1.0, 1.0, -1.0, 1.0, -1.0];
                    for i in 0..6 {
                        verts.push(BoltVert { pos: q[i].into(), k: [k[i], l.min(6.0e4), g] });
                    }
                }
            }
        }
        if verts.is_empty() {
            return;
        }
        let bytes: &[u8] = bytemuck::cast_slice(&verts);
        if self.bolt_vbuf.as_ref().is_none_or(|(_, n)| *n < bytes.len()) {
            let n = bytes.len().next_power_of_two();
            self.bolt_vbuf = Some((
                device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("lightning vb"),
                    size: n as u64,
                    usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                    mapped_at_creation: false,
                }),
                n,
            ));
        }
        let vb = &self.bolt_vbuf.as_ref().unwrap().0;
        queue.write_buffer(vb, 0, bytes);
        queue.write_buffer(&self.bolt_ubuf, 0, bytemuck::bytes_of(&BoltU { view_proj: vp.to_cols_array_2d(), screen: [size.0 as f32, size.1 as f32, pre, 0.0] }));
        use wgpu::BindingResource as R;
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("lightning"),
            layout: &self.bolt_pipe.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: self.bolt_ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(dist) },
                wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(ccol) },
                wgpu::BindGroupEntry { binding: 3, resource: R::TextureView(cdep) },
            ],
        });
        let mut pass = begin(encoder, "lightning", hdr, wgpu::LoadOp::Load);
        pass.set_pipeline(&self.bolt_pipe);
        pass.set_bind_group(0, &bg, &[]);
        pass.set_vertex_buffer(0, vb.slice(..bytes.len() as u64));
        pass.draw(0..verts.len() as u32, 0..1);
    }

    /// The windscreen pass (cockpit view): reads `hdr`, writes the glass
    /// into an own target, then lays it over `hdr`.
    #[allow(clippy::too_many_arguments)]
    pub fn glass(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        f: &WxFrame,
        inv_vp: Mat4,
        focal: f32,
        hdr: &wgpu::TextureView,
        dist: &wgpu::TextureView,
        size: (u32, u32),
    ) {
        let Some(g) = &f.glass else { return };
        if g.panels.is_empty() {
            return;
        }
        if self.glass_tex.as_ref().is_none_or(|t| t.1 != size.0 || t.2 != size.1) {
            let tex = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("windscreen"),
                size: wgpu::Extent3d { width: size.0.max(1), height: size.1.max(1), depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: HDR,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            });
            self.glass_tex = Some((tex.create_view(&Default::default()), size.0, size.1));
        }
        let s = &g.state;
        let mut panes = [[[0.0f32; 4]; 6]; 2];
        for (i, p) in g.panels.iter().take(2).enumerate() {
            let (pivot, arm, a0, sign) = g.wipers.get(i).copied().unwrap_or((Vec2::ZERO, 0.0, 0.0, 1.0));
            panes[i] = [
                [p.origin.x, p.origin.y, p.origin.z, 1.0],
                [p.normal.x, p.normal.y, p.normal.z, 0.0],
                [p.u.x, p.u.y, p.u.z, 0.0],
                [p.v.x, p.v.y, p.v.z, 0.0],
                [p.lo.x, p.lo.y, p.hi.x, p.hi.y],
                [pivot.x, pivot.y, arm, a0 * sign.signum()],
            ];
        }
        let u = GlassU {
            inv_view_proj: inv_vp.to_cols_array_2d(),
            screen: [size.0 as f32, size.1 as f32, 1.0 / size.0 as f32, 1.0 / size.1 as f32],
            state: [g.time, s.rain, s.airspeed, s.wet_time],
            state2: [s.fog, s.frost, s.ice, focal],
            wiper: [s.wipers.phase, if s.wipers.running { 1.0 } else { 0.0 }, s.wipers.stroke.max(0.3), s.wipers.since_stop],
            panes,
        };
        queue.write_buffer(&self.glass_ubuf, 0, bytemuck::bytes_of(&u));
        use wgpu::BindingResource as R;
        let gt = &self.glass_tex.as_ref().unwrap().0;
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("windscreen"),
            layout: &self.glass_pipe.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 5, resource: self.glass_ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 6, resource: R::TextureView(hdr) },
                wgpu::BindGroupEntry { binding: 7, resource: R::TextureView(dist) },
                wgpu::BindGroupEntry { binding: 8, resource: R::Sampler(&self.samp) },
            ],
        });
        {
            let mut pass = begin(encoder, "windscreen", gt, wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT));
            pass.set_pipeline(&self.glass_pipe);
            pass.set_bind_group(0, &bg, &[]);
            pass.draw(0..3, 0..1);
        }
        let bg2 = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("windscreen blit"),
            layout: &self.blit_pipe.get_bind_group_layout(0),
            entries: &[wgpu::BindGroupEntry { binding: 9, resource: R::TextureView(gt) }],
        });
        let mut pass = begin(encoder, "windscreen blit", hdr, wgpu::LoadOp::Load);
        pass.set_pipeline(&self.blit_pipe);
        pass.set_bind_group(0, &bg2, &[]);
        pass.draw(0..3, 0..1);
    }
}

fn begin<'e>(encoder: &'e mut wgpu::CommandEncoder, label: &str, view: &wgpu::TextureView, load: wgpu::LoadOp<wgpu::Color>) -> wgpu::RenderPass<'e> {
    encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
        label: Some(label),
        color_attachments: &[Some(wgpu::RenderPassColorAttachment { view, depth_slice: None, resolve_target: None, ops: wgpu::Operations { load, store: wgpu::StoreOp::Store } })],
        depth_stencil_attachment: None,
        timestamp_writes: None,
        occlusion_query_set: None,
        multiview_mask: None,
    })
}
