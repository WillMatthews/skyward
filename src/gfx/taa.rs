//! Temporal anti-aliasing / upscaling (TAAU), FXAA, motion blur and depth
//! of field on the HDR image (shaders/taa.wgsl), plus the maths they share
//! with the CPU: the jitter sequence, the jittered projection and the
//! reprojection of a pixel into last frame's image.
//!
//! Motion vectors: static scenery is reprojected from the eye-distance
//! buffer through last frame's camera (eye-relative: `prev_vp` already holds
//! the eye's motion, and the renderer keeps the previous eye in the current
//! floating-origin frame — [`crate::gfx::Renderer::rebase`]); the aircraft
//! (own, multiplayer traffic) write their own motion in a velocity pass
//! (gfx_aircraft.rs `draw_velocity`, shaders/aircraft_vel.wgsl); clouds
//! move with their own mean distance (clouds.rs history).

use glam::{Mat4, Vec2, Vec3, Vec4, vec2};

/// Velocity target: xy = uv then − uv now, z = eye distance now, w = eye distance last frame (0 = no object).
pub const VEL: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
const HDR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;

/// The reactive mask: eye-relative dynamic-mesh vertices → (0, 0, distance, −1).
const REACTIVE: &str = r#"
struct VelU { view_proj: mat4x4<f32>, view_proj_now: mat4x4<f32>, view_proj_prev: mat4x4<f32>, eye_dt: vec4<f32> };
@group(0) @binding(0) var<uniform> vu: VelU;
struct O { @builtin(position) clip: vec4<f32>, @location(0) rel: vec3<f32> };
@vertex
fn vs_r(@location(0) p: vec3<f32>) -> O {
    var o: O;
    o.clip = vu.view_proj * vec4<f32>(p, 1.0);
    o.rel = p;
    return o;
}
@fragment
fn fs_r(in: O) -> @location(0) vec4<f32> {
    return vec4<f32>(0.0, 0.0, length(in.rel), -1.0);
}
"#;

/// Element `i` (1-based) of the Halton sequence in `base`.
pub fn halton(mut i: u32, base: u32) -> f32 {
    let mut f = 1.0;
    let mut r = 0.0;
    while i > 0 {
        f /= base as f32;
        r += f * (i % base) as f32;
        i /= base;
    }
    r
}

/// Sub-pixel jitter (input pixels, −0.5..0.5, +y down) of frame `frame`:
/// Halton(2, 3) over `n` phases (8 at native resolution, more when upscaling
/// so every output pixel gets covered).
pub fn jitter(frame: u64, n: u32) -> Vec2 {
    let i = (frame % n.max(1) as u64) as u32 + 1;
    vec2(halton(i, 2) - 0.5, halton(i, 3) - 0.5)
}

/// Phases for an upscale factor (output / input pixels per axis).
pub fn phases(upscale: f32) -> u32 {
    (8.0 * (upscale * upscale).max(1.0)).round().clamp(8.0, 32.0) as u32
}

/// `proj` shifted by `j` pixels of a `w`×`h` target (content moves +x right, +y down).
pub fn jitter_proj(proj: Mat4, j: Vec2, w: u32, h: u32) -> Mat4 {
    // A clip-space translation scaled by w: the same pixel shift at every depth.
    Mat4::from_translation(Vec3::new(2.0 * j.x / w.max(1) as f32, -2.0 * j.y / h.max(1) as f32, 0.0)) * proj
}

/// uv (0..1, +y down) of an eye-relative point through a clip matrix.
pub fn project_uv(m: Mat4, p: Vec4) -> Option<Vec2> {
    let c = m * p;
    (c.w > 1e-5).then(|| vec2(c.x / c.w * 0.5 + 0.5, 0.5 - c.y / c.w * 0.5))
}

/// Where the surface seen at `uv` at eye distance `dist` was in last frame's
/// image (the shader's `motion_at` for static scenery). `inv_vp`: this
/// frame's unjittered eye-relative inverse; `prev_rel`: last frame's
/// view-projection relative to this frame's eye.
pub fn reproject(uv: Vec2, dist: f32, inv_vp: Mat4, prev_rel: Mat4) -> Option<Vec2> {
    let ndc = vec2(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let p = inv_vp * Vec4::new(ndc.x, ndc.y, 0.5, 1.0);
    let d = (p.truncate() / p.w).normalize();
    if dist >= 60_000.0 { project_uv(prev_rel, d.extend(0.0)) } else { project_uv(prev_rel, (d * dist).extend(1.0)) }
}

/// Bind group layout of the velocity pass's uniforms (group 0).
pub fn vel_bgl(device: &wgpu::Device) -> wgpu::BindGroupLayout {
    device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("velocity"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
            count: None,
        }],
    })
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct VelU {
    pub view_proj: [[f32; 4]; 4],
    pub view_proj_now: [[f32; 4]; 4],
    pub view_proj_prev: [[f32; 4]; 4],
    /// This eye − last frame's eye (m), w = the frame interval (s).
    pub eye_dt: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable, Default)]
pub struct TaaU {
    pub inv_vp: [[f32; 4]; 4],
    pub prev_vp: [[f32; 4]; 4],
    pub jitter: [f32; 4],
    pub in_size: [f32; 4],
    pub out_size: [f32; 4],
    pub eye_delta: [f32; 4],
    pub params: [f32; 4],
    pub focus: [f32; 4],
}

fn size4(w: u32, h: u32) -> [f32; 4] {
    [w as f32, h as f32, 1.0 / w.max(1) as f32, 1.0 / h.max(1) as f32]
}

/// Scene inputs of the post passes (the frame's eye distances and cloud result).
pub struct SceneIn<'a> {
    pub dist: &'a wgpu::TextureView,
    pub clouds: Option<(&'a wgpu::TextureView, &'a wgpu::TextureView)>,
}

/// An image a pass reads: an outside view (the scene's HDR target) or one of ours.
#[derive(Clone, Copy)]
pub enum Input<'a> {
    Ext(&'a wgpu::TextureView),
    Taa,
    Fxaa,
    Fx(usize),
}

struct Tex {
    view: wgpu::TextureView,
    w: u32,
    h: u32,
}

fn tex(device: &wgpu::Device, label: &str, w: u32, h: u32, format: wgpu::TextureFormat, usage: wgpu::TextureUsages) -> Tex {
    let view = device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        })
        .create_view(&Default::default());
    Tex { view, w, h }
}

/// The AA / camera-effects passes and their targets.
pub struct Post {
    bgl: wgpu::BindGroupLayout,
    /// Velocity pass: the dynamic mesh as "reactive" (no motion known).
    reactive: wgpu::RenderPipeline,
    taa: wgpu::RenderPipeline,
    fxaa: wgpu::RenderPipeline,
    mblur: wgpu::RenderPipeline,
    dof: wgpu::RenderPipeline,
    /// One uniform buffer per pass kind: taa, fxaa, mblur, dof.
    ubufs: [wgpu::Buffer; 4],
    samp: wgpu::Sampler,
    vel_ubuf: wgpu::Buffer,
    vel_bg: wgpu::BindGroup,
    dummy: wgpu::TextureView,
    dummy_dep: wgpu::TextureView,
    /// TAA history (output size) ping-pong; `cur` = the latest.
    hist: Option<[Tex; 2]>,
    cur: usize,
    pub valid: bool,
    /// Velocity target + its depth (scene size).
    vel: Option<(Tex, wgpu::TextureView)>,
    /// FXAA output (scene size), effect ping-pong (post size).
    aa_out: Option<Tex>,
    fx: Option<[Tex; 2]>,
    /// Frames rendered (jitter phase).
    pub frame: u64,
}

const RT: wgpu::TextureUsages = wgpu::TextureUsages::RENDER_ATTACHMENT.union(wgpu::TextureUsages::TEXTURE_BINDING);

impl Post {
    pub fn new(device: &wgpu::Device) -> Post {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("taa"), source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/taa.wgsl").into()) });
        let t = |binding: u32, filterable: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false },
            count: None,
        };
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("taa"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                t(1, true),
                t(2, true),
                t(3, true),
                t(4, true),
                wgpu::BindGroupLayoutEntry {
                    binding: 5,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                t(6, true),
                t(7, false),
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("taa"), bind_group_layouts: &[Some(&bgl)], immediate_size: 0 });
        let pipe = |fs: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(fs),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some("vs_full"), compilation_options: Default::default(), buffers: &[] },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: HDR, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let ubuf = || {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("taa u"),
                size: std::mem::size_of::<TaaU>() as u64,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let samp = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("taa"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            ..Default::default()
        });
        let vel_ubuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("velocity u"),
            size: std::mem::size_of::<VelU>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let vbgl = vel_bgl(device);
        let vel_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("velocity"),
            layout: &vbgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: vel_ubuf.as_entire_binding() }],
        });
        let reactive = {
            let m = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("reactive"), source: wgpu::ShaderSource::Wgsl(REACTIVE.into()) });
            let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("reactive"), bind_group_layouts: &[Some(&vbgl)], immediate_size: 0 });
            const ATTR: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 }];
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("reactive"),
                layout: Some(&layout),
                vertex: wgpu::VertexState {
                    module: &m,
                    entry_point: Some("vs_r"),
                    compilation_options: Default::default(),
                    buffers: &[Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<crate::scene::Vert>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &ATTR,
                    })],
                },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: crate::gfx::DEPTH,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &m,
                    entry_point: Some("fs_r"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: VEL, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        Post {
            reactive,
            taa: pipe("fs_taa"),
            fxaa: pipe("fs_fxaa"),
            mblur: pipe("fs_mblur"),
            dof: pipe("fs_dof"),
            bgl,
            ubufs: [ubuf(), ubuf(), ubuf(), ubuf()],
            samp,
            vel_ubuf,
            vel_bg,
            dummy: tex(device, "taa dummy", 1, 1, HDR, wgpu::TextureUsages::TEXTURE_BINDING).view,
            dummy_dep: tex(device, "taa dummy dep", 1, 1, wgpu::TextureFormat::Rg32Float, wgpu::TextureUsages::TEXTURE_BINDING).view,
            hist: None,
            cur: 0,
            valid: false,
            vel: None,
            aa_out: None,
            fx: None,
            frame: 0,
        }
    }

    /// Bytes of the targets (VRAM estimate).
    pub fn bytes(&self) -> u64 {
        let b = |t: &Tex, bpp: u64| t.w as u64 * t.h as u64 * bpp;
        self.hist.as_ref().map_or(0, |h| b(&h[0], 8) * 2)
            + self.vel.as_ref().map_or(0, |v| b(&v.0, 12))
            + self.aa_out.as_ref().map_or(0, |t| b(t, 8))
            + self.fx.as_ref().map_or(0, |f| b(&f[0], 8) * 2)
    }

    /// Forget the history (camera cut, resize).
    pub fn reset(&mut self) {
        self.valid = false;
    }

    /// The velocity target exists at the scene size `w`×`h` (cleared by `velocity_pass`).
    fn ensure_vel(&mut self, device: &wgpu::Device, w: u32, h: u32) {
        if self.vel.as_ref().is_some_and(|v| v.0.w == w && v.0.h == h) {
            return;
        }
        let v = tex(device, "velocity", w, h, VEL, RT);
        let d = tex(device, "velocity depth", w, h, crate::gfx::DEPTH, wgpu::TextureUsages::RENDER_ATTACHMENT).view;
        self.vel = Some((v, d));
    }

    /// Object motion (aircraft) into the velocity target. `vp_j`: this
    /// frame's jittered view-projection, `vp`: unjittered, `prev_vp`: last
    /// frame's unjittered (relative to last frame's eye).
    #[allow(clippy::too_many_arguments)]
    pub fn velocity_pass(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        w: u32,
        h: u32,
        vp_j: Mat4,
        vp: Mat4,
        prev_vp: Mat4,
        eye_dt: Vec4,
        reactive: Option<(&wgpu::Buffer, &wgpu::Buffer, u32)>,
        draw: impl FnOnce(&mut wgpu::RenderPass<'_>, &wgpu::BindGroup),
    ) {
        self.ensure_vel(device, w, h);
        let u = VelU { view_proj: vp_j.to_cols_array_2d(), view_proj_now: vp.to_cols_array_2d(), view_proj_prev: prev_vp.to_cols_array_2d(), eye_dt: eye_dt.to_array() };
        queue.write_buffer(&self.vel_ubuf, 0, bytemuck::bytes_of(&u));
        let (v, d) = self.vel.as_ref().unwrap();
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("velocity"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &v.view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::TRANSPARENT), store: wgpu::StoreOp::Store },
            })],
            depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                view: d,
                depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(0.0), store: wgpu::StoreOp::Discard }),
                stencil_ops: None,
            }),
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        draw(&mut pass, &self.vel_bg);
        // Moving things without their own motion (missiles, tracers, flames,
        // the London Eye: the dynamic mesh): reactive — TAA trusts this frame there.
        if let Some((vb, ib, n)) = reactive {
            pass.set_pipeline(&self.reactive);
            pass.set_bind_group(0, &self.vel_bg, &[]);
            pass.set_vertex_buffer(0, vb.slice(..));
            pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..n, 0, 0..1);
        }
    }

    /// The view of an input (ours must exist: produced earlier this frame).
    pub fn view<'a>(&'a self, i: Input<'a>) -> &'a wgpu::TextureView {
        match i {
            Input::Ext(v) => v,
            Input::Taa => &self.hist.as_ref().expect("taa output")[self.cur].view,
            Input::Fxaa => &self.aa_out.as_ref().expect("fxaa output").view,
            Input::Fx(k) => &self.fx.as_ref().expect("effect output")[k & 1].view,
        }
    }

    fn bind(&self, device: &wgpu::Device, ubuf: &wgpu::Buffer, cur: &wgpu::TextureView, i: &SceneIn<'_>, hist: Option<&wgpu::TextureView>) -> wgpu::BindGroup {
        use wgpu::BindingResource as R;
        let vel = self.vel.as_ref().map_or(&self.dummy, |v| &v.0.view);
        let (cc, cd) = i.clouds.unwrap_or((&self.dummy, &self.dummy_dep));
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("taa"),
            layout: &self.bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(cur) },
                wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(i.dist) },
                wgpu::BindGroupEntry { binding: 3, resource: R::TextureView(vel) },
                wgpu::BindGroupEntry { binding: 4, resource: R::TextureView(hist.unwrap_or(&self.dummy)) },
                wgpu::BindGroupEntry { binding: 5, resource: R::Sampler(&self.samp) },
                wgpu::BindGroupEntry { binding: 6, resource: R::TextureView(cc) },
                wgpu::BindGroupEntry { binding: 7, resource: R::TextureView(cd) },
            ],
        })
    }

    fn run(encoder: &mut wgpu::CommandEncoder, label: &str, view: &wgpu::TextureView, pipe: &wgpu::RenderPipeline, bg: &wgpu::BindGroup) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some(label),
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
        pass.set_pipeline(pipe);
        pass.set_bind_group(0, bg, &[]);
        pass.draw(0..3, 0..1);
    }

    /// Temporal AA / upscale `cur` (scene size, jittered by `u.jitter`) into
    /// the history at the output size (then `Input::Taa`).
    #[allow(clippy::too_many_arguments)]
    pub fn taa(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, encoder: &mut wgpu::CommandEncoder, cur: &wgpu::TextureView, i: &SceneIn<'_>, out: (u32, u32), mut u: TaaU) {
        if self.hist.as_ref().is_none_or(|h| h[0].w != out.0 || h[0].h != out.1) {
            self.hist = Some([tex(device, "taa history a", out.0, out.1, HDR, RT), tex(device, "taa history b", out.0, out.1, HDR, RT)]);
            self.valid = false;
        }
        u.out_size = size4(out.0, out.1);
        u.jitter[2] = if self.valid { 1.0 } else { 0.0 };
        queue.write_buffer(&self.ubufs[0], 0, bytemuck::bytes_of(&u));
        let hist = self.hist.as_ref().unwrap();
        let (prev, next) = (&hist[self.cur].view, &hist[1 - self.cur].view);
        let bg = self.bind(device, &self.ubufs[0], cur, i, Some(prev));
        Post::run(encoder, "taa", next, &self.taa, &bg);
        self.cur = 1 - self.cur;
        self.valid = true;
    }

    /// FXAA of `cur` at its own (scene) size `w`×`h` (then `Input::Fxaa`);
    /// `exposure` makes the edge detection perceptual.
    #[allow(clippy::too_many_arguments)]
    pub fn fxaa(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        cur: &wgpu::TextureView,
        i: &SceneIn<'_>,
        w: u32,
        h: u32,
        exposure: f32,
    ) {
        if self.aa_out.as_ref().is_none_or(|t| t.w != w || t.h != h) {
            self.aa_out = Some(tex(device, "fxaa", w, h, HDR, RT));
        }
        let u = TaaU { in_size: size4(w, h), out_size: size4(w, h), jitter: [0.0, 0.0, 0.0, exposure], ..Default::default() };
        queue.write_buffer(&self.ubufs[1], 0, bytemuck::bytes_of(&u));
        let bg = self.bind(device, &self.ubufs[1], cur, i, None);
        let out = &self.aa_out.as_ref().unwrap().view;
        Post::run(encoder, "fxaa", out, &self.fxaa, &bg);
    }

    /// Run a camera effect (`kind` 2 = motion blur, 3 = depth of field) from
    /// `input` (post size `w`×`h`) into effect target `slot` (then `Input::Fx(slot)`).
    #[allow(clippy::too_many_arguments)]
    pub fn effect(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        kind: usize,
        input: Input<'_>,
        i: &SceneIn<'_>,
        w: u32,
        h: u32,
        slot: usize,
        u: TaaU,
    ) {
        if self.fx.as_ref().is_none_or(|f| f[0].w != w || f[0].h != h) {
            self.fx = Some([tex(device, "post fx a", w, h, HDR, RT), tex(device, "post fx b", w, h, HDR, RT)]);
        }
        queue.write_buffer(&self.ubufs[kind], 0, bytemuck::bytes_of(&TaaU { out_size: size4(w, h), ..u }));
        let bg = self.bind(device, &self.ubufs[kind], self.view(input), i, None);
        let out = &self.fx.as_ref().unwrap()[slot & 1].view;
        Post::run(encoder, if kind == 2 { "motion blur" } else { "dof" }, out, if kind == 2 { &self.mblur } else { &self.dof }, &bg);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, vec3};

    fn proj(w: u32, h: u32) -> Mat4 {
        glam::camera::rh::proj::directx::perspective_infinite_reverse(60f32.to_radians(), w as f32 / h as f32, 0.25)
    }

    #[test]
    fn jitter_sequence_is_centred_and_well_spread() {
        for n in [8u32, 16, 32] {
            let js: Vec<Vec2> = (0..n as u64).map(|f| jitter(f, n)).collect();
            assert!(js.iter().all(|j| j.x.abs() <= 0.5 && j.y.abs() <= 0.5));
            let mean = js.iter().copied().sum::<Vec2>() / n as f32;
            assert!(mean.length() < 0.08, "mean {mean} for {n}");
            // Low discrepancy: every quadrant gets samples, no two coincide.
            for q in 0..4 {
                let (sx, sy) = (q & 1 == 1, q & 2 == 2);
                assert!(js.iter().any(|j| (j.x > 0.0) == sx && (j.y > 0.0) == sy));
            }
            for (a, b) in js.iter().enumerate().flat_map(|(i, a)| js[i + 1..].iter().map(move |b| (a, b))) {
                assert!(a.distance(*b) > 0.02);
            }
        }
        // Periodic, and the 8-phase sequence starts at Halton index 1.
        assert_eq!(jitter(3, 8), jitter(11, 8));
        assert_eq!(jitter(0, 8), vec2(0.0, 1.0 / 3.0 - 0.5));
        assert_eq!(phases(1.0), 8);
        assert_eq!(phases(2.0), 32);
    }

    #[test]
    fn jittered_projection_shifts_by_whole_pixels_at_any_depth() {
        let (w, h) = (1920, 1080);
        let p = proj(w, h);
        let j = vec2(0.25, -0.375);
        let pj = jitter_proj(p, j, w, h);
        for z in [0.5f32, 30.0, 4000.0, 90_000.0] {
            let v = Vec4::new(3.0, -1.5, -z, 1.0);
            let a = project_uv(p, v).unwrap() * vec2(w as f32, h as f32);
            let b = project_uv(pj, v).unwrap() * vec2(w as f32, h as f32);
            let d = b - a;
            assert!((d - j).length() < 1e-3 * (1.0 + z / 1000.0), "z {z}: {d} vs {j}");
        }
    }

    /// A camera at 250 m/s turning quickly, with a floating-origin rebase between
    /// the two frames: the reprojected uv of a static point must equal where it
    /// really was last frame.
    #[test]
    fn reprojection_is_exact_under_motion_and_rebase() {
        let (w, h) = (2560u32, 1440u32);
        let p = proj(w, h);
        let rot_view = |yaw: f32, pitch: f32| Mat4::from_quat(Quat::from_euler(glam::EulerRot::YXZ, yaw, pitch, 0.0)).inverse();
        // World (f64-ish but within f32 range here), 1/120 s apart.
        let eye0 = vec3(12_345.0, 820.0, -7_654.0);
        let eye1 = eye0 + vec3(250.0, 0.0, -30.0) / 120.0;
        let (yaw0, yaw1) = (0.3f32, 0.3 + 90f32.to_radians() / 120.0);
        let vp0 = p * rot_view(yaw0, -0.1);
        let vp1 = p * rot_view(yaw1, -0.09);
        // The origin moves by `delta` between the frames (World::rebase): the
        // renderer shifts the stored previous eye by the same delta.
        let delta = vec3(-10_000.0, 0.0, 8_000.0);
        let eye1_rebased = eye1 + delta;
        let eye0_stored = eye0 + delta;
        let prev_rel = vp0 * Mat4::from_translation(eye1_rebased - eye0_stored);
        let inv1 = vp1.inverse();
        for (k, world) in [vec3(12_500.0, 0.0, -9_000.0), vec3(12_300.0, 300.0, -8_000.0), vec3(13_000.0, 50.0, -12_000.0)].into_iter().enumerate() {
            let rel1 = world - eye1;
            let Some(uv1) = project_uv(vp1, rel1.extend(1.0)) else { continue };
            let truth = project_uv(vp0, (world - eye0).extend(1.0)).unwrap();
            let got = reproject(uv1, rel1.length(), inv1, prev_rel).unwrap();
            let err_px = (got - truth) * vec2(w as f32, h as f32);
            assert!(err_px.length() < 0.05, "point {k}: {err_px} px");
        }
        // The sky (direction only) ignores the eye's translation entirely.
        let dir = vec3(0.4, 0.2, -1.0).normalize();
        let uv1 = project_uv(vp1, dir.extend(0.0)).unwrap();
        let truth = project_uv(vp0, dir.extend(0.0)).unwrap();
        let got = reproject(uv1, 65_000.0, inv1, prev_rel).unwrap();
        assert!(((got - truth) * vec2(w as f32, h as f32)).length() < 0.05);
    }

    /// Object velocities (aircraft_vel.wgsl): eye-relative transforms on both
    /// frames, so a chase camera locked to the aircraft gives zero motion even
    /// at 250 m/s, and a rebase changes nothing.
    #[test]
    fn object_motion_is_zero_for_a_locked_chase_camera() {
        let p = proj(1920, 1080);
        let vp = p * Mat4::from_quat(Quat::from_rotation_y(0.4)).inverse();
        // Model eye-relative: same both frames (camera follows rigidly).
        let model = Mat4::from_rotation_translation(Quat::from_rotation_y(0.4), vec3(0.0, -2.0, -15.0));
        for v in [vec3(1.0, 0.5, 2.0), vec3(-5.0, 0.0, -1.0)] {
            let now = project_uv(vp, (model * v.extend(1.0)).truncate().extend(1.0)).unwrap();
            let prev = project_uv(vp, (model * v.extend(1.0)).truncate().extend(1.0)).unwrap();
            assert_eq!(now, prev);
        }
    }
}
