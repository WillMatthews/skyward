//! GPU side of the effects: one storage buffer of particles drawn as
//! instanced quads, one vertex buffer of ribbons / meshes, and the heat-haze
//! pass (a copy of the HDR scene, re-sampled with an animated offset). All
//! draw into the HDR target after the clouds and fog, reading the scene's
//! eye-distance buffer for soft edges and the clouds' half-resolution result
//! (transmittance + mean depth) so effects behind a cloud are hidden by it
//! (gfx.rs calls [`VfxRenderer::render`]).
//!
//! Before upload the particles get their lighting terms ([`light_particles`]):
//! the optical depth of the other smoke toward the key light (sun / moon) and
//! up to the sky, and the fires' irradiance through the smoke between them —
//! a coarse density grid marched once per particle on the CPU.

use super::{GpuParticle, VfxFrame, VfxVert};
use glam::Vec3;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct VfxU {
    off: [f32; 4],
    screen: [f32; 4],
    fires: [[f32; 4]; 4],
    misc: [f32; 4],
    key: [f32; 4],
    vol: [[f32; 4]; super::VOL_N],
}

struct Buf {
    buf: wgpu::Buffer,
    cap: u64,
}

impl Buf {
    fn new(device: &wgpu::Device, label: &str, usage: wgpu::BufferUsages, cap: u64) -> Buf {
        let buf = device.create_buffer(&wgpu::BufferDescriptor { label: Some(label), size: cap.max(256), usage: usage | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false });
        Buf { buf, cap: cap.max(256) }
    }
    /// Upload, growing the buffer when needed.
    fn put(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, label: &str, usage: wgpu::BufferUsages, data: &[u8]) {
        let need = data.len() as u64;
        if need > self.cap {
            *self = Buf::new(device, label, usage, need.next_power_of_two());
        }
        if need > 0 {
            queue.write_buffer(&self.buf, 0, data);
        }
    }
}

/// GPU timing of the effects passes (env `SKYWARD_VFX_TIMING=1`): each frame's
/// effects are recorded `reps` times between two timestamps (the image is
/// then wrong; for measurements only) and the mean printed every 30 frames.
struct Timing {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    read: wgpu::Buffer,
    period: f32,
    reps: u32,
    sum: f64,
    n: u32,
    /// A measurement was recorded last frame (read back at the next one).
    pending: bool,
    /// CPU time of the lighting pass (ms), summed.
    cpu: f64,
}

pub struct VfxRenderer {
    bgl: wgpu::BindGroupLayout,
    blit_bgl: wgpu::BindGroupLayout,
    particles: wgpu::RenderPipeline,
    mesh: wgpu::RenderPipeline,
    haze: wgpu::RenderPipeline,
    vol: wgpu::RenderPipeline,
    blit: wgpu::RenderPipeline,
    ubuf: wgpu::Buffer,
    pbuf: Buf,
    vbuf: Buf,
    ibuf: Buf,
    hvbuf: Buf,
    hibuf: Buf,
    copy: Option<(wgpu::TextureView, u32, u32)>,
    dummy: wgpu::TextureView,
    dummy_col: wgpu::TextureView,
    dummy_dep: wgpu::TextureView,
    /// Lit particles (scratch, reused).
    lit: Vec<GpuParticle>,
    grid: Grid,
    timing: Option<Timing>,
    /// Particles / ribbon vertices drawn last frame (stats).
    pub drawn: (usize, usize),
}

fn premul() -> wgpu::BlendState {
    wgpu::BlendState {
        color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add },
        alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
    }
}

fn tex1(device: &wgpu::Device, label: &str, format: wgpu::TextureFormat) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

impl VfxRenderer {
    /// `globals_bgl` = the scene's group 0 (its bind group is passed to `render`).
    pub fn new(device: &wgpu::Device, globals_bgl: &wgpu::BindGroupLayout, immediate_size: u32, hdr: wgpu::TextureFormat) -> VfxRenderer {
        let src = format!("{}\n{}\n{}", include_str!("../shaders/atmos_core.wgsl"), include_str!("../shaders/common.wgsl"), include_str!("../shaders/vfx.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("vfx"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let blit_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("vfx blit"), source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/vfx_blit.wgsl").into()) });
        let vf = wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT;
        let tex = |binding: u32, visibility: wgpu::ShaderStages| wgpu::BindGroupLayoutEntry {
            binding,
            visibility,
            ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: false }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false },
            count: None,
        };
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vfx"),
            entries: &[
                wgpu::BindGroupLayoutEntry { binding: 0, visibility: vf, ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None }, count: None },
                wgpu::BindGroupLayoutEntry { binding: 1, visibility: wgpu::ShaderStages::VERTEX, ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None }, count: None },
                tex(2, wgpu::ShaderStages::FRAGMENT),
                tex(3, wgpu::ShaderStages::FRAGMENT),
                // Clouds (half resolution): colour (a = transmittance), depth.
                tex(4, vf),
                tex(5, vf),
            ],
        });
        let blit_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor { label: Some("vfx blit"), entries: &[tex(0, wgpu::ShaderStages::FRAGMENT)] });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("vfx"), bind_group_layouts: &[Some(globals_bgl), Some(&bgl)], immediate_size });
        let blit_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("vfx blit"), bind_group_layouts: &[Some(&blit_bgl)], immediate_size: 0 });
        const MESH_ATTRS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0 => Float32x4, 1 => Float32x4, 2 => Float32x4];
        let mesh_layout = [Some(wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<VfxVert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &MESH_ATTRS })];
        let pipe = |label: &str, vs: &str, fs: &str, buffers: &[Option<wgpu::VertexBufferLayout>], blend: Option<wgpu::BlendState>| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState { module: &module, entry_point: Some(fs), compilation_options: Default::default(), targets: &[Some(wgpu::ColorTargetState { format: hdr, blend, write_mask: wgpu::ColorWrites::ALL })] }),
                multiview_mask: None,
                cache: None,
            })
        };
        let particles = pipe("vfx particles", "vs_particle", "fs_particle", &[], Some(premul()));
        let mesh = pipe("vfx mesh", "vs_mesh", "fs_mesh", &mesh_layout, Some(premul()));
        let haze = pipe("vfx haze", "vs_mesh", "fs_haze", &mesh_layout, Some(premul()));
        let vol = pipe("vfx volumes", "vs_vol", "fs_vol", &[], Some(premul()));
        let blit = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("vfx blit"),
            layout: Some(&blit_layout),
            vertex: wgpu::VertexState { module: &blit_mod, entry_point: Some("vs_blit"), compilation_options: Default::default(), buffers: &[] },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState { module: &blit_mod, entry_point: Some("fs_blit"), compilation_options: Default::default(), targets: &[Some(wgpu::ColorTargetState { format: hdr, blend: None, write_mask: wgpu::ColorWrites::ALL })] }),
            multiview_mask: None,
            cache: None,
        });
        let ubuf = device.create_buffer(&wgpu::BufferDescriptor { label: Some("vfx u"), size: std::mem::size_of::<VfxU>() as u64, usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false });
        let st = wgpu::BufferUsages::STORAGE;
        let vx = wgpu::BufferUsages::VERTEX;
        let ix = wgpu::BufferUsages::INDEX;
        let dummy = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("vfx dummy"),
                size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: hdr,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let timing = (std::env::var_os("SKYWARD_VFX_TIMING").is_some() && device.features().contains(wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS)).then(|| Timing {
            set: device.create_query_set(&wgpu::QuerySetDescriptor { label: Some("vfx timing"), ty: wgpu::QueryType::Timestamp, count: 2 }),
            resolve: device.create_buffer(&wgpu::BufferDescriptor { label: Some("vfx timing"), size: 16, usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC, mapped_at_creation: false }),
            read: device.create_buffer(&wgpu::BufferDescriptor { label: Some("vfx timing read"), size: 16, usage: wgpu::BufferUsages::MAP_READ | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false }),
            period: 1.0,
            reps: std::env::var("SKYWARD_VFX_TIMING").ok().and_then(|s| s.parse().ok()).unwrap_or(20u32).max(1),
            sum: 0.0,
            n: 0,
            pending: false,
            cpu: 0.0,
        });
        VfxRenderer {
            bgl,
            blit_bgl,
            particles,
            mesh,
            haze,
            vol,
            blit,
            ubuf,
            pbuf: Buf::new(device, "vfx particles", st, 64 * 1024),
            vbuf: Buf::new(device, "vfx verts", vx, 64 * 1024),
            ibuf: Buf::new(device, "vfx idx", ix, 32 * 1024),
            hvbuf: Buf::new(device, "vfx haze verts", vx, 8 * 1024),
            hibuf: Buf::new(device, "vfx haze idx", ix, 4 * 1024),
            copy: None,
            dummy,
            dummy_col: tex1(device, "vfx no clouds", wgpu::TextureFormat::Rgba16Float),
            dummy_dep: tex1(device, "vfx no cloud depth", wgpu::TextureFormat::Rg32Float),
            lit: Vec::new(),
            grid: Grid::default(),
            timing,
            drawn: (0, 0),
        }
    }

    /// Draw this frame's effects into `hdr` (after the clouds / fog).
    /// `clouds`: the clouds' latest result (colour, depth) at half resolution;
    /// `key`: direction toward the key light (sun, or the moon at night).
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        f: &VfxFrame,
        eye: Vec3,
        time: f32,
        globals: &wgpu::BindGroup,
        hdr: &wgpu::TextureView,
        dist: &wgpu::TextureView,
        clouds: Option<(&wgpu::TextureView, &wgpu::TextureView)>,
        key: Vec3,
        hdr_format: wgpu::TextureFormat,
        w: u32,
        h: u32,
    ) {
        if self.timing.as_ref().is_some_and(|t| t.pending) {
            self.collect_timing(device);
        }
        self.drawn = (f.particles.len(), f.mesh.len());
        if f.is_empty() {
            return;
        }
        let off = f.origin - eye;
        let mut fires = f.fires;
        for fl in &mut fires {
            fl[0] += off.x;
            fl[1] += off.y;
            fl[2] += off.z;
        }
        let key = key.normalize_or(Vec3::Y);
        let u = VfxU {
            off: [off.x, off.y, off.z, time],
            screen: [w as f32, h as f32, 1.0 / w as f32, 1.0 / h as f32],
            fires,
            misc: [1.0, if clouds.is_some() { 1.0 } else { 0.0 }, 0.0, 0.0],
            key: [key.x, key.y, key.z, 0.0],
            vol: f.vol,
        };
        queue.write_buffer(&self.ubuf, 0, bytemuck::bytes_of(&u));
        // Lighting through the smoke (CPU, coarse grid).
        self.lit.clear();
        self.lit.extend_from_slice(&f.particles);
        // (Timed only when measuring: `Instant` is unavailable on wasm32.)
        let t0 = self.timing.as_ref().map(|_| std::time::Instant::now());
        light_particles(&mut self.grid, &mut self.lit, key, &f.fires);
        if let (Some(t), Some(t0)) = (&mut self.timing, t0) {
            t.cpu += t0.elapsed().as_secs_f64() * 1e3;
        }
        let st = wgpu::BufferUsages::STORAGE;
        let vx = wgpu::BufferUsages::VERTEX;
        let ix = wgpu::BufferUsages::INDEX;
        self.pbuf.put(device, queue, "vfx particles", st, bytemuck::cast_slice::<GpuParticle, u8>(&self.lit));
        self.vbuf.put(device, queue, "vfx verts", vx, bytemuck::cast_slice::<VfxVert, u8>(&f.mesh));
        self.ibuf.put(device, queue, "vfx idx", ix, bytemuck::cast_slice::<u32, u8>(&f.mesh_idx));
        self.hvbuf.put(device, queue, "vfx haze verts", vx, bytemuck::cast_slice::<VfxVert, u8>(&f.haze));
        self.hibuf.put(device, queue, "vfx haze idx", ix, bytemuck::cast_slice::<u32, u8>(&f.haze_idx));
        // Heat haze needs a copy of the scene to refract.
        let haze = !f.haze_idx.is_empty();
        if let Some(t) = &self.timing {
            encoder.write_timestamp(&t.set, 0);
        }
        let reps = self.timing.as_ref().map_or(1, |t| t.reps);
        if haze && self.copy.as_ref().is_none_or(|c| c.1 != w || c.2 != h) {
            let t = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("vfx scene copy"),
                size: wgpu::Extent3d { width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: hdr_format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::RENDER_ATTACHMENT,
                view_formats: &[],
            });
            self.copy = Some((t.create_view(&Default::default()), w, h));
        }
        let scene = if haze { &self.copy.as_ref().unwrap().0 } else { &self.dummy };
        let (ccol, cdep) = clouds.unwrap_or((&self.dummy_col, &self.dummy_dep));
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vfx"),
            layout: &self.bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: self.ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: self.pbuf.buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::TextureView(dist) },
                wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::TextureView(scene) },
                wgpu::BindGroupEntry { binding: 4, resource: wgpu::BindingResource::TextureView(ccol) },
                wgpu::BindGroupEntry { binding: 5, resource: wgpu::BindingResource::TextureView(cdep) },
            ],
        });
        for _ in 0..reps {
            if haze {
                let copy = &self.copy.as_ref().unwrap().0;
                let bgc = device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("vfx blit"), layout: &self.blit_bgl, entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(hdr) }] });
                let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some("vfx copy"),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment { view: copy, depth_slice: None, resolve_target: None, ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store } })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                });
                pass.set_pipeline(&self.blit);
                pass.set_bind_group(0, &bgc, &[]);
                pass.draw(0..3, 0..1);
            }
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("vfx"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment { view: hdr, depth_slice: None, resolve_target: None, ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store } })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, globals, &[]);
            pass.set_bind_group(1, &bg, &[]);
            if haze {
                pass.set_pipeline(&self.haze);
                pass.set_vertex_buffer(0, self.hvbuf.buf.slice(..));
                pass.set_index_buffer(self.hibuf.buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..f.haze_idx.len() as u32, 0, 0..1);
            }
            if !f.mesh_idx.is_empty() {
                pass.set_pipeline(&self.mesh);
                pass.set_vertex_buffer(0, self.vbuf.buf.slice(..));
                pass.set_index_buffer(self.ibuf.buf.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..f.mesh_idx.len() as u32, 0, 0..1);
            }
            if f.vol_boxes != 0 {
                // Condensation volumes: a box each (36 vertices), ray-marched.
                pass.set_pipeline(&self.vol);
                for i in 0..3u32 {
                    if f.vol_boxes & (1 << i) != 0 {
                        pass.draw(0..36, i..i + 1);
                    }
                }
            }
            if !self.lit.is_empty() {
                pass.set_pipeline(&self.particles);
                pass.draw(0..6, 0..self.lit.len() as u32);
            }
        }
        if let Some(t) = &mut self.timing {
            encoder.write_timestamp(&t.set, 1);
            encoder.resolve_query_set(&t.set, 0..2, &t.resolve, 0);
            encoder.copy_buffer_to_buffer(&t.resolve, 0, &t.read, 0, 16);
            t.period = queue.get_timestamp_period();
            t.pending = true;
        }
    }

    /// With `SKYWARD_VFX_TIMING`: read back the last frame's effects time
    /// (after the frame was submitted; blocks on the GPU).
    fn collect_timing(&mut self, device: &wgpu::Device) -> Option<f32> {
        let t = self.timing.as_mut()?;
        t.pending = false;
        let slice = t.read.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        let _ = device.poll(wgpu::PollType::wait_indefinitely());
        let ts: Vec<u64> = bytemuck::cast_slice::<u8, u64>(&slice.get_mapped_range().ok()?).to_vec();
        t.read.unmap();
        let ms = ts[1].saturating_sub(ts[0]) as f64 * t.period as f64 * 1e-6 / t.reps as f64;
        t.sum += ms;
        t.n += 1;
        if t.n == 8 {
            eprintln!("vfx gpu ms: {:.3} ({} particles, {} mesh verts; lighting cpu {:.2} ms)", t.sum / 8.0, self.drawn.0, self.drawn.1, t.cpu / 8.0);
            t.sum = 0.0;
            t.cpu = 0.0;
            t.n = 0;
        }
        Some(ms as f32)
    }
}

// ---------------------------------------------------------------- lighting through the smoke

/// Coarse extinction grid over the absorbing particles (smoke, dust, steam,
/// soot), with the optical depth of every cell toward the key light and up
/// to the sky (swept slice by slice: O(cells)).
#[derive(Default)]
struct Grid {
    lo: Vec3,
    cell: f32,
    n: [usize; 3],
    k: Vec<f32>,
    /// Optical depth toward the key light / straight up.
    tk: Vec<f32>,
    tu: Vec<f32>,
}

/// Cells in the grid (at most).
const GN: usize = 36 * 36 * 36;
const GN_AXIS: usize = 160;

impl Grid {
    fn at(&self, x: usize, y: usize, z: usize) -> usize {
        (z * self.n[1] + y) * self.n[0] + x
    }
    fn idx(&self, p: Vec3) -> Option<usize> {
        let q = (p - self.lo) / self.cell;
        if q.x < 0.0 || q.y < 0.0 || q.z < 0.0 {
            return None;
        }
        let (x, y, z) = (q.x as usize, q.y as usize, q.z as usize);
        (x < self.n[0] && y < self.n[1] && z < self.n[2]).then(|| self.at(x, y, z))
    }
    /// Trilinear sample of a cell field at `p` (cell centres; 0 outside).
    fn sample(&self, v: &[f32], p: Vec3) -> f32 {
        let q = (p - self.lo) / self.cell - 0.5;
        let b = q.floor();
        let f = q - b;
        let mut acc = 0.0;
        for c in 0..8 {
            let o = [(c & 1) as i64, ((c >> 1) & 1) as i64, ((c >> 2) & 1) as i64];
            let i = [b.x as i64 + o[0], b.y as i64 + o[1], b.z as i64 + o[2]];
            if i.iter().zip(self.n.iter()).any(|(&i, &n)| i < 0 || i >= n as i64) {
                continue;
            }
            let w = (if o[0] == 1 { f.x } else { 1.0 - f.x }) * (if o[1] == 1 { f.y } else { 1.0 - f.y }) * (if o[2] == 1 { f.z } else { 1.0 - f.z });
            acc += w * v[self.at(i[0] as usize, i[1] as usize, i[2] as usize)];
        }
        acc
    }
    /// Optical depth from `p` along `d` (unit) for at most `len` m, `steps` samples.
    fn tau(&self, p: Vec3, d: Vec3, len: f32, skip: f32, steps: usize) -> f32 {
        let step = (len - skip).max(0.0) / steps as f32;
        if step <= 0.0 {
            return 0.0;
        }
        let mut tau = 0.0;
        for s in 0..steps {
            if let Some(i) = self.idx(p + d * (skip + (s as f32 + 0.5) * step)) {
                tau += self.k[i] * step;
            }
        }
        tau
    }
    /// Sweep the optical depths toward the light `l` (unit) and up.
    fn sweep(&mut self, l: Vec3) {
        let [nx, ny, nz] = self.n;
        self.tk.clear();
        self.tk.resize(self.k.len(), 0.0);
        self.tu.clear();
        self.tu.resize(self.k.len(), 0.0);
        // Up: running sums down each column (half of the cell itself).
        for z in 0..nz {
            for x in 0..nx {
                let mut acc = 0.0;
                for y in (0..ny).rev() {
                    let i = self.at(x, y, z);
                    self.tu[i] = acc + 0.5 * self.k[i] * self.cell;
                    acc += self.k[i] * self.cell;
                }
            }
        }
        // Toward the light: slices along its dominant axis, from the lit side;
        // each cell continues the depth of the point one slice toward the light.
        let la = [l.x.abs(), l.y.abs(), l.z.abs()];
        let a = if la[0] >= la[1] && la[0] >= la[2] { 0 } else if la[1] >= la[2] { 1 } else { 2 };
        let (b, c) = ((a + 1) % 3, (a + 2) % 3);
        let lv = [l.x, l.y, l.z];
        let toward = lv[a].signum() as i64;
        let step_len = self.cell / la[a].max(1e-3);
        // Lateral shift (in cells) of the point one slice toward the light.
        let db = lv[b] / la[a].max(1e-3);
        let dc = lv[c] / la[a].max(1e-3);
        let n = self.n;
        let idx3 = |ia: usize, ib: usize, ic: usize| {
            let mut p = [0usize; 3];
            p[a] = ia;
            p[b] = ib;
            p[c] = ic;
            (p[2] * n[1] + p[1]) * n[0] + p[0]
        };
        let order: Vec<usize> = if toward > 0 { (0..n[a]).rev().collect() } else { (0..n[a]).collect() };
        let mut prev: Option<usize> = None;
        for &ia in &order {
            for ib in 0..n[b] {
                for ic in 0..n[c] {
                    let i = idx3(ia, ib, ic);
                    let own = self.k[i] * step_len;
                    let up = match prev {
                        None => 0.0,
                        Some(pa) => {
                            // Bilinear in the previous slice.
                            let fb = ib as f32 + db;
                            let fc = ic as f32 + dc;
                            let (b0, c0) = (fb.floor(), fc.floor());
                            let (wb, wc) = (fb - b0, fc - c0);
                            let mut acc = 0.0;
                            for (ob, oc, w) in [(0, 0, (1.0 - wb) * (1.0 - wc)), (1, 0, wb * (1.0 - wc)), (0, 1, (1.0 - wb) * wc), (1, 1, wb * wc)] {
                                let (jb, jc) = (b0 as i64 + ob, c0 as i64 + oc);
                                if jb >= 0 && jc >= 0 && (jb as usize) < n[b] && (jc as usize) < n[c] {
                                    let j = idx3(pa, jb as usize, jc as usize);
                                    acc += w * (self.tk[j] + 0.5 * self.k[j] * step_len);
                                }
                            }
                            acc
                        }
                    };
                    self.tk[i] = up + 0.5 * own;
                }
            }
            prev = Some(ia);
        }
    }
}

/// Soot fraction of a flame puff at temperature `t` (K) (== `vfx.wgsl`).
fn soot(t: f32) -> f32 {
    let x = ((1350.0 - t) / 400.0).clamp(0.0, 1.0);
    x * x * (3.0 - 2.0 * x)
}

/// Scattering kinds that absorb / shadow (== `vfx::Kind`): smoke, fire (soot), dust, steam.
fn absorbs(kind: f32) -> bool {
    matches!(kind as u32, 0 | 1 | 2 | 5)
}

/// Fill `light` = (key-light transmittance, fire irradiance (lux), -, sky
/// visibility) for the scattering particles.
fn light_particles(g: &mut Grid, ps: &mut [GpuParticle], key: Vec3, fires: &[[f32; 4]; 4]) {
    let mut lo = Vec3::splat(f32::MAX);
    let mut hi = Vec3::splat(f32::MIN);
    let mut any = 0usize;
    for p in ps.iter().filter(|p| absorbs(p.vel[3])) {
        let c = Vec3::new(p.pos[0], p.pos[1], p.pos[2]);
        lo = lo.min(c - p.pos[3]);
        hi = hi.max(c + p.pos[3]);
        any += 1;
    }
    let fire_list: Vec<(Vec3, f32)> = fires.iter().filter(|f| f[3] > 0.0).map(|f| (Vec3::new(f[0], f[1], f[2]), f[3])).collect();
    if any == 0 {
        for p in ps.iter_mut() {
            p.light[0] = 1.0;
            p.light[3] = 1.0;
        }
        return;
    }
    let ext = (hi - lo).max(Vec3::splat(1.0));
    // (Resolution follows the particle count: a few cells per puff.)
    let cells = (any * 3).clamp(4096, GN) as f32;
    let cell = (ext.x * ext.y * ext.z / cells).cbrt().max(ext.max_element() / GN_AXIS as f32).max(0.5);
    g.lo = lo;
    g.cell = cell;
    g.n = [((ext.x / cell) as usize + 1).min(GN_AXIS), ((ext.y / cell) as usize + 1).min(GN_AXIS), ((ext.z / cell) as usize + 1).min(GN_AXIS)];
    let total = g.n[0] * g.n[1] * g.n[2];
    g.k.clear();
    g.k.resize(total, 0.0);
    let vol = cell * cell * cell;
    for p in ps.iter().filter(|p| absorbs(p.vel[3])) {
        let c = Vec3::new(p.pos[0], p.pos[1], p.pos[2]);
        let r = p.pos[3].max(0.05);
        // Extinction of the puff: its opacity over its diameter.
        let a = p.col[3].clamp(0.0, 0.98) * if p.vel[3] as u32 == 1 { soot(p.light[2]) } else { 1.0 };
        if a < 0.01 {
            continue;
        }
        let sigma = -(1.0 - a).ln() / (1.6 * r);
        let mass = sigma * 0.5 * r * r * r * 4.18879;
        // Splat over the cells around it.
        let span = if r * 0.7 > cell { 1 } else { 0 };
        let mut idx: [usize; 27] = [0; 27];
        let mut m = 0;
        for dz in -span..=span {
            for dy in -span..=span {
                for dx in -span..=span {
                    if let Some(i) = g.idx(c + Vec3::new(dx as f32, dy as f32, dz as f32) * (r * 0.5).min(cell)) {
                        idx[m] = i;
                        m += 1;
                    }
                }
            }
        }
        if m > 0 {
            let k = mass / (vol * m as f32);
            for &i in &idx[..m] {
                g.k[i] += k;
            }
        }
    }
    g.sweep(key);
    // The brightest fire is seen through the smoke (a short march); the others unoccluded.
    let main_fire = fire_list.iter().enumerate().max_by(|a, b| a.1 .1.total_cmp(&b.1 .1)).map(|f| f.0);
    for p in ps.iter_mut() {
        let kind = p.vel[3] as u32;
        if !matches!(kind, 0 | 1 | 2 | 3 | 5) {
            continue;
        }
        let c = Vec3::new(p.pos[0], p.pos[1], p.pos[2]);
        p.light[0] = (-g.sample(&g.tk, c)).exp();
        p.light[3] = (-g.sample(&g.tu, c)).exp();
        let mut e = 0.0;
        for (fi, (fp, cd)) in fire_list.iter().enumerate() {
            let v = *fp - c;
            let d2 = v.length_squared().max(25.0);
            let d = d2.sqrt();
            if d > 3000.0 {
                continue;
            }
            let tf = if Some(fi) == main_fire && d < 600.0 { g.tau(c, v / d, (d - 2.0).max(0.0), p.pos[3] * 0.6, 8) } else { 0.0 };
            // A tall flame seen from above shows its smoky top, not its side:
            // less light goes straight up than out to the sides.
            let up = (-v.y / d).max(0.0);
            e += cd / d2 * (-tf).exp() * (1.0 - 0.75 * up * up);
        }
        p.light[1] = e;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn puff(p: Vec3, r: f32, a: f32) -> GpuParticle {
        GpuParticle { pos: [p.x, p.y, p.z, r], col: [0.1, 0.1, 0.1, a], vel: [0.0, 0.0, 0.0, 0.0], extra: [0.5, 0.0, 0.0, 1.0], light: [1.0, 0.0, 0.0, 1.0] }
    }

    #[test]
    fn smoke_shadows_itself_toward_the_light() {
        // A dense column; the light from +x: the far (-x) side is darker, the top sees the sky.
        let mut ps = Vec::new();
        for y in 0..20 {
            for x in -3..=3 {
                for z in -3..=3 {
                    ps.push(puff(Vec3::new(x as f32 * 4.0, y as f32 * 4.0, z as f32 * 4.0), 3.0, 0.6));
                }
            }
        }
        let mut g = Grid::default();
        light_particles(&mut g, &mut ps, Vec3::X, &[[0.0; 4]; 4]);
        let at = |x: i32, y: i32| ps.iter().find(|p| p.pos[0] == x as f32 * 4.0 && p.pos[1] == y as f32 * 4.0 && p.pos[2] == 0.0).unwrap().light;
        assert!(at(3, 10)[0] > 3.0 * at(-3, 10)[0], "{:?} {:?}", at(3, 10), at(-3, 10));
        assert!(at(0, 19)[3] > 3.0 * at(0, 0)[3]);
        assert!(ps.iter().all(|p| p.light.iter().all(|v| v.is_finite() && *v >= 0.0)));
    }

    #[test]
    fn sweep_matches_a_march() {
        let mut ps = Vec::new();
        for k in 0..400 {
            let f = k as f32;
            ps.push(puff(Vec3::new((f * 7.3) % 40.0, (f * 3.1) % 30.0, (f * 5.7) % 35.0), 2.5, 0.5));
        }
        let mut g = Grid::default();
        let l = Vec3::new(0.4, 0.8, -0.45).normalize();
        light_particles(&mut g, &mut ps, l, &[[0.0; 4]; 4]);
        let mut err = 0.0;
        for p in ps.iter() {
            let c = Vec3::new(p.pos[0], p.pos[1], p.pos[2]);
            let m = g.tau(c, l, 200.0, 0.0, 400);
            err += (g.sample(&g.tk, c) - m).abs() / (m + 0.5);
        }
        assert!(err / (ps.len() as f32) < 0.35, "{}", err / ps.len() as f32);
    }
}
