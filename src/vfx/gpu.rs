//! GPU side of the effects: one storage buffer of particles drawn as
//! instanced quads, one vertex buffer of ribbons / meshes, and the heat-haze
//! pass (a copy of the HDR scene, re-sampled with an animated offset). All
//! draw into the HDR target after the clouds and fog, reading the scene's
//! eye-distance buffer for soft edges (gfx.rs calls [`VfxRenderer::render`]).

use super::{GpuParticle, VfxFrame, VfxVert};
use glam::Vec3;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct VfxU {
    off: [f32; 4],
    screen: [f32; 4],
    fires: [[f32; 4]; 4],
    misc: [f32; 4],
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

pub struct VfxRenderer {
    bgl: wgpu::BindGroupLayout,
    blit_bgl: wgpu::BindGroupLayout,
    particles: wgpu::RenderPipeline,
    mesh: wgpu::RenderPipeline,
    haze: wgpu::RenderPipeline,
    blit: wgpu::RenderPipeline,
    ubuf: wgpu::Buffer,
    pbuf: Buf,
    vbuf: Buf,
    ibuf: Buf,
    hvbuf: Buf,
    hibuf: Buf,
    copy: Option<(wgpu::TextureView, u32, u32)>,
    dummy: wgpu::TextureView,
    /// Particles / ribbon vertices drawn last frame (stats).
    pub drawn: (usize, usize),
}

fn premul() -> wgpu::BlendState {
    wgpu::BlendState {
        color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add },
        alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
    }
}

impl VfxRenderer {
    /// `globals_bgl` = the scene's group 0 (its bind group is passed to `render`).
    pub fn new(device: &wgpu::Device, globals_bgl: &wgpu::BindGroupLayout, immediate_size: u32, hdr: wgpu::TextureFormat) -> VfxRenderer {
        let src = format!("{}\n{}\n{}", include_str!("../shaders/atmos_core.wgsl"), include_str!("../shaders/common.wgsl"), include_str!("../shaders/vfx.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("vfx"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let blit_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("vfx blit"), source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/vfx_blit.wgsl").into()) });
        let vf = wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT;
        let tex = |binding: u32| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: wgpu::ShaderStages::FRAGMENT,
            ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: false }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false },
            count: None,
        };
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("vfx"),
            entries: &[
                wgpu::BindGroupLayoutEntry { binding: 0, visibility: vf, ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None }, count: None },
                wgpu::BindGroupLayoutEntry { binding: 1, visibility: wgpu::ShaderStages::VERTEX, ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None }, count: None },
                tex(2),
                tex(3),
            ],
        });
        let blit_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor { label: Some("vfx blit"), entries: &[tex(0)] });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("vfx"), bind_group_layouts: &[Some(globals_bgl), Some(&bgl)], immediate_size });
        let blit_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("vfx blit"), bind_group_layouts: &[Some(&blit_bgl)], immediate_size: 0 });
        const MESH_ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x4, 1 => Float32x4];
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
        VfxRenderer {
            bgl,
            blit_bgl,
            particles,
            mesh,
            haze,
            blit,
            ubuf,
            pbuf: Buf::new(device, "vfx particles", st, 64 * 1024),
            vbuf: Buf::new(device, "vfx verts", vx, 64 * 1024),
            ibuf: Buf::new(device, "vfx idx", ix, 32 * 1024),
            hvbuf: Buf::new(device, "vfx haze verts", vx, 8 * 1024),
            hibuf: Buf::new(device, "vfx haze idx", ix, 4 * 1024),
            copy: None,
            dummy,
            drawn: (0, 0),
        }
    }

    /// Draw this frame's effects into `hdr` (after the clouds / fog).
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
        hdr_format: wgpu::TextureFormat,
        w: u32,
        h: u32,
    ) {
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
        let u = VfxU { off: [off.x, off.y, off.z, time], screen: [w as f32, h as f32, 1.0 / w as f32, 1.0 / h as f32], fires, misc: [1.0, 0.0, 0.0, 0.0] };
        queue.write_buffer(&self.ubuf, 0, bytemuck::bytes_of(&u));
        let st = wgpu::BufferUsages::STORAGE;
        let vx = wgpu::BufferUsages::VERTEX;
        let ix = wgpu::BufferUsages::INDEX;
        self.pbuf.put(device, queue, "vfx particles", st, bytemuck::cast_slice::<GpuParticle, u8>(&f.particles));
        self.vbuf.put(device, queue, "vfx verts", vx, bytemuck::cast_slice::<VfxVert, u8>(&f.mesh));
        self.ibuf.put(device, queue, "vfx idx", ix, bytemuck::cast_slice::<u32, u8>(&f.mesh_idx));
        self.hvbuf.put(device, queue, "vfx haze verts", vx, bytemuck::cast_slice::<VfxVert, u8>(&f.haze));
        self.hibuf.put(device, queue, "vfx haze idx", ix, bytemuck::cast_slice::<u32, u8>(&f.haze_idx));
        // Heat haze needs a copy of the scene to refract.
        let haze = !f.haze_idx.is_empty();
        if haze {
            if self.copy.as_ref().is_none_or(|c| c.1 != w || c.2 != h) {
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
            let copy = &self.copy.as_ref().unwrap().0;
            let bg = device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("vfx blit"), layout: &self.blit_bgl, entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(hdr) }] });
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("vfx copy"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment { view: copy, depth_slice: None, resolve_target: None, ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store } })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(&self.blit);
            pass.set_bind_group(0, &bg, &[]);
            pass.draw(0..3, 0..1);
        }
        let scene = if haze { &self.copy.as_ref().unwrap().0 } else { &self.dummy };
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("vfx"),
            layout: &self.bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: self.ubuf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 1, resource: self.pbuf.buf.as_entire_binding() },
                wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::TextureView(dist) },
                wgpu::BindGroupEntry { binding: 3, resource: wgpu::BindingResource::TextureView(scene) },
            ],
        });
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
        if !f.particles.is_empty() {
            pass.set_pipeline(&self.particles);
            pass.draw(0..6, 0..f.particles.len() as u32);
        }
    }
}
