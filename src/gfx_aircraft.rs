//! Renderer for the imported, textured aircraft models
//! ([`crate::aircraft_asset`]). Hooked into [`crate::gfx::Renderer`] at three
//! points: the shadow cascades, the planar water reflection and the main HDR
//! pass. The fragment shader is appended to the scene shader so the aircraft
//! gets exactly the scene's lighting, cascaded shadows, aerial perspective,
//! fog and cloud shadows (and is excluded from reprojected SSAO like the
//! old vertex-coloured aircraft).
//!
//! Per frame the app hands over an [`AircraftFrame`] (in `FrameView`): the
//! model plus one eye-relative transform per visible chunk.

use crate::aircraft_asset::AircraftAsset;
use glam::{Mat4, Vec4};
use std::sync::Arc;
use wgpu::util::DeviceExt;

/// One chunk to draw this frame.
#[derive(Clone, Copy, Debug)]
pub struct ChunkDraw {
    pub chunk: u32,
    /// Chunk node → eye-relative world.
    pub model: Mat4,
    /// Animated emission factor (× the chunk's emission colour).
    pub emit: f32,
}

/// The aircraft for one frame.
#[derive(Clone)]
pub struct AircraftFrame {
    pub asset: Arc<AircraftAsset>,
    pub draws: Vec<ChunkDraw>,
}

impl AircraftFrame {
    /// Re-express the draw transforms relative to an eye moved by `-d`
    /// (i.e. add `d` to every eye-relative position), e.g. for thumbnails.
    pub fn shift(&mut self, d: glam::Vec3) {
        for dr in &mut self.draws {
            dr.model = Mat4::from_translation(d) * dr.model;
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Inst {
    model: [[f32; 4]; 4],
    color: [f32; 4],
    emis: [f32; 4],
    params: [f32; 4],
}

struct GpuModel {
    key: usize,
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    tex_bgs: Vec<wgpu::BindGroup>,
}

/// Draw list prepared for this frame: (chunk index, instance index).
#[derive(Default)]
struct Prepared {
    opaque: Vec<(u32, u32)>,
    shadow: Vec<(u32, u32)>,
}

pub struct AircraftRenderer {
    main_cull: wgpu::RenderPipeline,
    main_twosided: wgpu::RenderPipeline,
    refl: wgpu::RenderPipeline,
    shadow: wgpu::RenderPipeline,
    inst_bgl: wgpu::BindGroupLayout,
    tex_bgl: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    white_bg: wgpu::BindGroup,
    inst_buf: wgpu::Buffer,
    inst_cap: usize,
    inst_bg: wgpu::BindGroup,
    model: Option<GpuModel>,
    asset: Option<Arc<AircraftAsset>>,
    prepared: Prepared,
}

fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2];
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<crate::aircraft_asset::Vertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

fn make_inst(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, cap: usize) -> (wgpu::Buffer, wgpu::BindGroup) {
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("aircraft instances"),
        size: (cap * std::mem::size_of::<Inst>()) as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("aircraft instances"),
        layout: bgl,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }],
    });
    (buf, bg)
}

impl AircraftRenderer {
    /// `globals_bgl` / `shadow_bgl`: the scene's group-0 layouts; `immediate_size`:
    /// the scene pipelines' immediate-data size (the shared shader declares it).
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, globals_bgl: &wgpu::BindGroupLayout, shadow_bgl: &wgpu::BindGroupLayout, immediate_size: u32) -> Self {
        let src = format!(
            "{}\n{}\n{}\n{}",
            include_str!("shaders/atmos_core.wgsl"),
            include_str!("shaders/common.wgsl"),
            include_str!("shaders/scene.wgsl"),
            include_str!("shaders/aircraft.wgsl")
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("aircraft"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let shadow_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("aircraft shadow"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/aircraft_shadow.wgsl").into()),
        });
        let inst_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("aircraft instances"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            }],
        });
        let tex_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("aircraft texture"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: true }, view_dimension: wgpu::TextureViewDimension::D2, multisampled: false },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry { binding: 1, visibility: wgpu::ShaderStages::FRAGMENT, ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering), count: None },
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("aircraft"),
            bind_group_layouts: &[Some(globals_bgl), Some(&inst_bgl), Some(&tex_bgl)],
            immediate_size,
        });
        let shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("aircraft shadow"),
            bind_group_layouts: &[Some(shadow_bgl), Some(&inst_bgl)],
            immediate_size: 0,
        });
        let pipe = |label: &str, cull: Option<wgpu::Face>, samples: u32| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some("vs_aircraft"), compilation_options: Default::default(), buffers: &[Some(vertex_layout())] },
                primitive: wgpu::PrimitiveState { cull_mode: cull, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: crate::gfx::DEPTH,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: samples, mask: !0, alpha_to_coverage_enabled: samples > 1 },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some("fs_aircraft"),
                    compilation_options: Default::default(),
                    targets: &[
                        Some(wgpu::ColorTargetState { format: crate::gfx::HDR, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                        Some(wgpu::ColorTargetState { format: crate::gfx::DIST, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                    ],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let main_cull = pipe("aircraft", Some(wgpu::Face::Back), crate::gfx::MSAA);
        let main_twosided = pipe("aircraft 2-sided", None, crate::gfx::MSAA);
        // The mirrored reflection flips the winding: draw both sides there.
        let refl = pipe("aircraft refl", None, 1);
        let shadow = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("aircraft shadow"),
            layout: Some(&shadow_layout),
            vertex: wgpu::VertexState {
                module: &shadow_mod,
                entry_point: Some("vs_ac_shadow"),
                compilation_options: Default::default(),
                buffers: &[Some(wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<crate::aircraft_asset::Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 }],
                })],
            },
            primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: crate::gfx::DEPTH,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::LessEqual),
                stencil: Default::default(),
                bias: wgpu::DepthBiasState { constant: 1, slope_scale: 1.5, clamp: 0.0 },
            }),
            multisample: Default::default(),
            fragment: None,
            multiview_mask: None,
            cache: None,
        });
        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("aircraft"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        let white = upload_texture(device, queue, 1, 1, &[255, 255, 255, 255]);
        let white_bg = tex_bg(device, &tex_bgl, &white, &sampler);
        let inst_cap = 256;
        let (inst_buf, inst_bg) = make_inst(device, &inst_bgl, inst_cap);
        AircraftRenderer {
            main_cull,
            main_twosided,
            refl,
            shadow,
            inst_bgl,
            tex_bgl,
            sampler,
            white_bg,
            inst_buf,
            inst_cap,
            inst_bg,
            model: None,
            asset: None,
            prepared: Prepared::default(),
        }
    }

    /// Upload the model (when it changed) and this frame's instances.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: Option<&AircraftFrame>) {
        self.prepared.opaque.clear();
        self.prepared.shadow.clear();
        let Some(f) = frame else { return };
        let key = Arc::as_ptr(&f.asset) as usize;
        if self.model.as_ref().map(|m| m.key) != Some(key) {
            let a = &f.asset;
            if a.vertices.is_empty() || a.indices.is_empty() {
                self.model = None;
                return;
            }
            let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("aircraft vb"),
                contents: bytemuck::cast_slice(&a.vertices),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("aircraft ib"),
                contents: bytemuck::cast_slice(&a.indices),
                usage: wgpu::BufferUsages::INDEX,
            });
            let tex_bgs = a
                .textures
                .iter()
                .map(|t| {
                    let v = upload_texture(device, queue, t.width, t.height, &t.rgba);
                    tex_bg(device, &self.tex_bgl, &v, &self.sampler)
                })
                .collect();
            self.model = Some(GpuModel { key, vb, ib, tex_bgs });
            self.asset = Some(f.asset.clone());
        }
        let a = &f.asset;
        if f.draws.len() > self.inst_cap {
            self.inst_cap = f.draws.len().next_power_of_two();
            (self.inst_buf, self.inst_bg) = make_inst(device, &self.inst_bgl, self.inst_cap);
        }
        let mut insts = Vec::with_capacity(f.draws.len());
        // Opaque first, then cut-outs, then glass (alpha-to-coverage needs no sorting,
        // but drawing glass last lets the depth test skip what it hides).
        let mut order: Vec<(u8, usize)> = f
            .draws
            .iter()
            .enumerate()
            .map(|(i, d)| {
                let m = &a.materials[a.chunks[d.chunk as usize].material];
                (if m.translucent { 2 } else if m.alpha_tex { 1 } else { 0 }, i)
            })
            .collect();
        order.sort_by_key(|o| o.0);
        for (mode, i) in order {
            let d = &f.draws[i];
            let c = &a.chunks[d.chunk as usize];
            let m = &a.materials[c.material];
            // AC3D diffuse colours are display (sRGB-ish) values.
            let lin = |x: f32| x.clamp(0.0, 1.0).powf(2.2);
            let textured = m.tex.is_some();
            let col = if textured {
                // Texture × material: the material colour is a (usually white) tint.
                Vec4::new(m.col.x.clamp(0.0, 1.0), m.col.y.clamp(0.0, 1.0), m.col.z.clamp(0.0, 1.0), m.col.w)
            } else {
                Vec4::new(lin(m.col.x), lin(m.col.y), lin(m.col.z), m.col.w)
            };
            let mut emis = m.emis * 0.02;
            if let Some((ec, _)) = &c.emit {
                let k = if c.interior { 0.02 } else { 3.0 };
                emis += *ec * d.emit * k;
            }
            // Shininess (0..128) → roughness; specular level → f0.
            let rough = (2.0 / (m.shininess.max(0.0) + 2.0)).sqrt().clamp(0.12, 0.9);
            let spec = (m.spec.x + m.spec.y + m.spec.z) / 3.0;
            let f0 = if mode == 2 { 0.06 } else { 0.03 + 0.05 * spec.clamp(0.0, 1.0) };
            insts.push(Inst {
                model: d.model.to_cols_array_2d(),
                color: col.to_array(),
                emis: [emis.x, emis.y, emis.z, if c.interior { 1.0 } else { 0.0 }],
                params: [f0, rough, mode as f32, if textured { 1.0 } else { 0.0 }],
            });
            let ii = (insts.len() - 1) as u32;
            self.prepared.opaque.push((d.chunk, ii));
            if mode != 2 && !c.interior {
                self.prepared.shadow.push((d.chunk, ii));
            }
        }
        if !insts.is_empty() {
            queue.write_buffer(&self.inst_buf, 0, bytemuck::cast_slice(&insts));
        }
    }

    fn draw_list(&self, pass: &mut wgpu::RenderPass<'_>, list: &[(u32, u32)], refl: bool) {
        let (Some(gm), Some(a)) = (&self.model, &self.asset) else { return };
        pass.set_bind_group(1, &self.inst_bg, &[]);
        pass.set_vertex_buffer(0, gm.vb.slice(..));
        pass.set_index_buffer(gm.ib.slice(..), wgpu::IndexFormat::Uint32);
        let mut cur_pipe = 255u8;
        for &(ci, ii) in list {
            let c = &a.chunks[ci as usize];
            let m = &a.materials[c.material];
            let p = if refl { 2 } else if m.two_sided || m.translucent { 1 } else { 0 };
            if p != cur_pipe {
                pass.set_pipeline(match p {
                    0 => &self.main_cull,
                    1 => &self.main_twosided,
                    _ => &self.refl,
                });
                cur_pipe = p;
            }
            let bg = m.tex.and_then(|t| gm.tex_bgs.get(t)).unwrap_or(&self.white_bg);
            pass.set_bind_group(2, bg, &[]);
            pass.draw_indexed(c.first..c.first + c.count, c.base_vertex as i32, ii..ii + 1);
        }
    }

    /// Main HDR pass (group 0 = the scene globals, already bound).
    pub fn draw_main(&self, pass: &mut wgpu::RenderPass<'_>) {
        self.draw_list(pass, &self.prepared.opaque, false);
    }

    /// Planar water reflection (group 0 = the reflection globals, already bound).
    pub fn draw_refl(&self, pass: &mut wgpu::RenderPass<'_>) {
        let list: Vec<(u32, u32)> = self
            .prepared
            .opaque
            .iter()
            .copied()
            .filter(|&(ci, _)| self.asset.as_ref().map(|a| !a.chunks[ci as usize].interior).unwrap_or(false))
            .collect();
        self.draw_list(pass, &list, true);
    }

    /// One shadow cascade (group 0 = the cascade's uniform, already bound).
    pub fn draw_shadow(&self, pass: &mut wgpu::RenderPass<'_>) {
        let (Some(gm), Some(a)) = (&self.model, &self.asset) else { return };
        if self.prepared.shadow.is_empty() {
            return;
        }
        pass.set_pipeline(&self.shadow);
        pass.set_bind_group(1, &self.inst_bg, &[]);
        pass.set_vertex_buffer(0, gm.vb.slice(..));
        pass.set_index_buffer(gm.ib.slice(..), wgpu::IndexFormat::Uint32);
        for &(ci, ii) in &self.prepared.shadow {
            let c = &a.chunks[ci as usize];
            pass.draw_indexed(c.first..c.first + c.count, c.base_vertex as i32, ii..ii + 1);
        }
    }
}

fn tex_bg(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, view: &wgpu::TextureView, samp: &wgpu::Sampler) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("aircraft texture"),
        layout: bgl,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(view) }, wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(samp) }],
    })
}

/// sRGB RGBA8 texture with a CPU-built mip chain (box filter in linear light).
fn upload_texture(device: &wgpu::Device, queue: &wgpu::Queue, w: u32, h: u32, rgba: &[u8]) -> wgpu::TextureView {
    let levels = 32 - w.max(h).max(1).leading_zeros();
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("aircraft texture"),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let to_lin: Vec<f32> = (0..256).map(|i| (i as f32 / 255.0).powf(2.2)).collect();
    let mut cur: Vec<f32> = rgba.iter().enumerate().map(|(i, &b)| if i % 4 == 3 { b as f32 / 255.0 } else { to_lin[b as usize] }).collect();
    let (mut cw, mut ch) = (w, h);
    for level in 0..levels {
        let bytes: Vec<u8> = cur
            .iter()
            .enumerate()
            .map(|(i, &v)| if i % 4 == 3 { (v * 255.0 + 0.5) as u8 } else { (v.max(0.0).powf(1.0 / 2.2) * 255.0 + 0.5).min(255.0) as u8 })
            .collect();
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &tex, mip_level: level, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            &bytes,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(cw * 4), rows_per_image: Some(ch) },
            wgpu::Extent3d { width: cw, height: ch, depth_or_array_layers: 1 },
        );
        if level + 1 == levels {
            break;
        }
        let (nw, nh) = ((cw / 2).max(1), (ch / 2).max(1));
        let mut next = vec![0f32; (nw * nh * 4) as usize];
        for y in 0..nh {
            for x in 0..nw {
                let mut acc = [0f32; 4];
                let mut n = 0.0;
                for dy in 0..2u32 {
                    for dx in 0..2u32 {
                        let sx = (x * 2 + dx).min(cw - 1);
                        let sy = (y * 2 + dy).min(ch - 1);
                        let o = ((sy * cw + sx) * 4) as usize;
                        for k in 0..4 {
                            acc[k] += cur[o + k];
                        }
                        n += 1.0;
                    }
                }
                let o = ((y * nw + x) * 4) as usize;
                for k in 0..4 {
                    next[o + k] = acc[k] / n;
                }
            }
        }
        cur = next;
        cw = nw;
        ch = nh;
    }
    tex.create_view(&Default::default())
}
