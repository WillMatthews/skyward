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
    /// Texture-coordinate transform (`aircraft_asset::Pose::uv`).
    pub uv: [f32; 6],
}

/// The aircraft for one frame.
#[derive(Clone)]
pub struct AircraftFrame {
    pub asset: Arc<AircraftAsset>,
    pub draws: Vec<ChunkDraw>,
    /// Structural parts of the vertices (wing flex, debris), models.rs.
    pub parts: Option<Arc<crate::models::PartMap>>,
    /// Wing bending, vibration and the loose parts' transforms this frame.
    pub dynamics: Option<Box<AcDynamics>>,
    /// Glass cockpit display textures (`cockpit::displays`), when rendered.
    pub displays: Option<Arc<DisplaySet>>,
}

/// The glass displays' textures for the screens of the current model.
pub struct DisplaySet {
    /// Changes whenever the views are recreated (bind groups are rebuilt).
    pub generation: u64,
    /// (display kind, view sampled as sRGB with a full mip chain).
    pub views: Vec<(String, wgpu::TextureView)>,
    /// Screen luminance (scene radiance units): bright by day, dimmed at night.
    pub brightness: f32,
}

/// Per-frame structural animation (eye-relative), shader `AcDyn`.
#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
pub struct AcDynamics {
    /// Body up axis (eye-relative world), w = time (s).
    pub up: [f32; 4],
    /// Tip deflection left / right (m, + up), bend-twist (rad per m of tip deflection), unused.
    pub flex: [f32; 4],
    /// Vibration: amplitude (m), frequency (Hz), unused ×2.
    pub vib: [f32; 4],
    /// Per part: 1 = bends with the wing (still attached), 0 = rigid.
    pub part_flex: [[f32; 4]; 4],
    /// Per part: intact eye-relative world → this part's eye-relative world
    /// (identity while attached).
    pub parts: [[[f32; 4]; 4]; 16],
}

impl Default for AcDynamics {
    fn default() -> Self {
        let mut d: AcDynamics = bytemuck::Zeroable::zeroed();
        d.up = [0.0, 1.0, 0.0, 0.0];
        for k in 0..16 {
            d.parts[k] = Mat4::IDENTITY.to_cols_array_2d();
            d.part_flex[k / 4][k % 4] = 1.0;
        }
        d
    }
}

impl AircraftFrame {
    /// Re-express the draw transforms relative to an eye moved by `-d`
    /// (i.e. add `d` to every eye-relative position), e.g. for thumbnails.
    pub fn shift(&mut self, d: glam::Vec3) {
        for dr in &mut self.draws {
            dr.model = Mat4::from_translation(d) * dr.model;
        }
        if let Some(dy) = self.dynamics.as_mut() {
            for m in &mut dy.parts {
                let x = Mat4::from_cols_array_2d(m);
                *m = (Mat4::from_translation(d) * x * Mat4::from_translation(-d)).to_cols_array_2d();
            }
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
    /// UV transform: 2×2 (column-major) …
    uvm: [f32; 4],
    /// … + offset (xy), z unused, w = display luminance.
    uvo: [f32; 4],
}

struct GpuModel {
    key: (usize, bool),
    /// Per-vertex structural attributes (part, span station, chord offset).
    pv: wgpu::Buffer,
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    /// Per chunk (first index, count) in `ib` (the parts copy re-tessellates the wings).
    ranges: Vec<(u32, u32)>,
    tex_bgs: Vec<wgpu::BindGroup>,
    asset: Arc<AircraftAsset>,
    /// Frames since an aircraft last used it (evicted when unused for a while).
    idle: u32,
}

/// Draw list prepared for this frame: (chunk index, instance index).
#[derive(Default)]
struct Prepared {
    opaque: Vec<(u32, u32)>,
    shadow: Vec<(u32, u32)>,
}

/// One aircraft's per-frame GPU state: its instances, structural dynamics
/// and which (shared) model it draws.
struct Slot {
    inst_buf: wgpu::Buffer,
    inst_cap: usize,
    inst_bg: wgpu::BindGroup,
    dyn_buf: wgpu::Buffer,
    /// Index into `AircraftRenderer::models`.
    model: Option<usize>,
    prepared: Prepared,
    /// Per prepared draw: the display bind group to use (index into `disp_bgs`).
    disp_of: Vec<Option<usize>>,
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
    /// Uploaded models, shared by every aircraft of the same type.
    models: Vec<GpuModel>,
    /// The player's aircraft.
    main: Slot,
    /// Other aircraft (multiplayer traffic), one slot each, reused frame to frame.
    traffic: Vec<Slot>,
    n_traffic: usize,
    /// Display sampler (clamped, trilinear, anisotropic).
    disp_sampler: wgpu::Sampler,
    /// (generation, per display kind: bind group).
    disp_bgs: Option<(u64, Vec<(String, wgpu::BindGroup)>)>,
}

/// Scene radiance of fully lit cockpit backlighting / annunciators (× the
/// model's emission colour × the painted albedo): ~15 cd/m² on white
/// markings — readable at night, invisible by day.
pub const INTERIOR_EMIT: f32 = 0.0003;

fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2];
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<crate::aircraft_asset::Vertex>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

fn part_layout(location: u32) -> wgpu::VertexBufferLayout<'static> {
    const A3: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x4, offset: 0, shader_location: 3 }];
    const A1: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x4, offset: 0, shader_location: 1 }];
    wgpu::VertexBufferLayout { array_stride: 16, step_mode: wgpu::VertexStepMode::Vertex, attributes: if location == 3 { &A3 } else { &A1 } }
}

fn make_inst(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, cap: usize, dyn_buf: &wgpu::Buffer) -> (wgpu::Buffer, wgpu::BindGroup) {
    let buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("aircraft instances"),
        size: (cap * std::mem::size_of::<Inst>()) as u64,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("aircraft instances"),
        layout: bgl,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() }, wgpu::BindGroupEntry { binding: 1, resource: dyn_buf.as_entire_binding() }],
    });
    (buf, bg)
}

impl AircraftRenderer {
    /// `globals_bgl` / `shadow_bgl`: the scene's group-0 layouts; `immediate_size`:
    /// the scene pipelines' immediate-data size (the shared shader declares it).
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, globals_bgl: &wgpu::BindGroupLayout, shadow_bgl: &wgpu::BindGroupLayout, immediate_size: u32) -> Self {
        let src = format!(
            "{}\n{}\n{}\n{}\n{}",
            include_str!("shaders/atmos_core.wgsl"),
            include_str!("shaders/common.wgsl"),
            include_str!("shaders/scene.wgsl"),
            include_str!("shaders/aircraft_dyn.wgsl"),
            include_str!("shaders/aircraft.wgsl")
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("aircraft"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let shadow_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("aircraft shadow"),
            source: wgpu::ShaderSource::Wgsl(format!("{}\n{}", include_str!("shaders/aircraft_dyn.wgsl"), include_str!("shaders/aircraft_shadow.wgsl")).into()),
        });
        let inst_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("aircraft instances"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
            ],
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
                vertex: wgpu::VertexState { module: &module, entry_point: Some("vs_aircraft"), compilation_options: Default::default(), buffers: &[Some(vertex_layout()), Some(part_layout(3))] },
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
                buffers: &[
                    Some(wgpu::VertexBufferLayout {
                        array_stride: std::mem::size_of::<crate::aircraft_asset::Vertex>() as u64,
                        step_mode: wgpu::VertexStepMode::Vertex,
                        attributes: &[wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 }],
                    }),
                    Some(part_layout(1)),
                ],
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
        let disp_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("cockpit display"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        let white = upload_texture(device, queue, 1, 1, &[255, 255, 255, 255], &[]);
        let white_bg = tex_bg(device, &tex_bgl, &white, &sampler);
        let main = new_slot(device, &inst_bgl);
        AircraftRenderer {
            main_cull,
            main_twosided,
            refl,
            shadow,
            inst_bgl,
            tex_bgl,
            sampler,
            white_bg,
            models: Vec::new(),
            main,
            traffic: Vec::new(),
            n_traffic: 0,
            disp_sampler,
            disp_bgs: None,
        }
    }

    /// Upload the model (when it changed) and this frame's instances.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frame: Option<&AircraftFrame>) {
        for m in &mut self.models {
            m.idle = m.idle.saturating_add(1);
        }
        // Display textures → bind groups (rebuilt when the views change).
        if let Some(ds) = frame.and_then(|f| f.displays.as_ref()) {
            if self.disp_bgs.as_ref().map(|d| d.0) != Some(ds.generation) {
                let bgs = ds.views.iter().map(|(k, v)| (k.clone(), tex_bg(device, &self.tex_bgl, v, &self.disp_sampler))).collect();
                self.disp_bgs = Some((ds.generation, bgs));
            }
        }
        let ph = Slot::placeholder(&self.main);
        let mut main = std::mem::replace(&mut self.main, ph);
        self.prepare_slot(device, queue, &mut main, frame, true);
        self.main = main;
    }

    /// Other aircraft this frame (multiplayer traffic; eye-relative frames
    /// like the player's). Models are shared with the player's and between
    /// aircraft of the same type.
    pub fn prepare_traffic(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, frames: &[AircraftFrame]) {
        while self.traffic.len() < frames.len() {
            self.traffic.push(new_slot(device, &self.inst_bgl));
        }
        let mut slots = std::mem::take(&mut self.traffic);
        for (slot, f) in slots.iter_mut().zip(frames) {
            self.prepare_slot(device, queue, slot, Some(f), false);
        }
        self.traffic = slots;
        self.n_traffic = frames.len();
        // Free models no aircraft has used for ~10 s (not the player's).
        let keep = self.main.model;
        let mut i = 0;
        while i < self.models.len() {
            if self.models[i].idle > 600 && Some(i) != keep && !self.traffic[..self.n_traffic].iter().any(|s| s.model == Some(i)) {
                self.models.swap_remove(i);
                let last = self.models.len();
                for s in std::iter::once(&mut self.main).chain(self.traffic.iter_mut()) {
                    if s.model == Some(i) {
                        s.model = None;
                    } else if s.model == Some(last) {
                        s.model = Some(i);
                    }
                }
            } else {
                i += 1;
            }
        }
    }

    /// The model for a frame (uploaded once per asset / parts variant).
    fn model_for(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, f: &AircraftFrame) -> Option<usize> {
        let key = (Arc::as_ptr(&f.asset) as usize, f.parts.is_some());
        if let Some(i) = self.models.iter().position(|m| m.key == key) {
            self.models[i].idle = 0;
            return Some(i);
        }
        let a = &f.asset;
        if a.vertices.is_empty() || a.indices.is_empty() {
            return None;
        }
        // With structural parts: the seam-split copy of the geometry.
        let split = f.parts.as_ref().filter(|p| p.vert.len() == p.vertices.len() && p.ranges.len() == a.chunks.len());
        let (verts, idx) = match split {
            Some(p) => (&p.vertices[..], &p.indices[..]),
            None => (&a.vertices[..], &a.indices[..]),
        };
        let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("aircraft vb"), contents: bytemuck::cast_slice(verts), usage: wgpu::BufferUsages::VERTEX });
        let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("aircraft ib"), contents: bytemuck::cast_slice(idx), usage: wgpu::BufferUsages::INDEX });
        // Textures: reuse another variant's bind groups of the same asset.
        let tex_bgs = match self.models.iter().find(|m| m.key.0 == key.0) {
            Some(other) => other.tex_bgs.clone(),
            None => a
                .textures
                .iter()
                .map(|t| {
                    let v = upload_texture(device, queue, t.width, t.height, &t.rgba, &t.mips);
                    tex_bg(device, &self.tex_bgl, &v, &self.sampler)
                })
                .collect(),
        };
        let pv_data: Vec<[f32; 4]> = match split {
            Some(p) => p.vert.clone(),
            None => vec![[0.0, 0.0, 0.0, 1.0]; a.vertices.len()],
        };
        let pv = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("aircraft parts"), contents: bytemuck::cast_slice(&pv_data), usage: wgpu::BufferUsages::VERTEX });
        let ranges = match split {
            Some(p) => p.ranges.clone(),
            None => a.chunks.iter().map(|c| (c.first, c.count)).collect(),
        };
        self.models.push(GpuModel { key, pv, vb, ib, ranges, tex_bgs, asset: a.clone(), idle: 0 });
        Some(self.models.len() - 1)
    }

    fn prepare_slot(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, slot: &mut Slot, frame: Option<&AircraftFrame>, displays: bool) {
        slot.prepared.opaque.clear();
        slot.prepared.shadow.clear();
        slot.disp_of.clear();
        let Some(f) = frame else { return };
        slot.model = self.model_for(device, queue, f);
        if slot.model.is_none() {
            return;
        }
        let disp_lum = if displays { f.displays.as_ref().map(|d| d.brightness) } else { None };
        let a = &f.asset;
        if f.draws.len() > slot.inst_cap {
            slot.inst_cap = f.draws.len().next_power_of_two();
            (slot.inst_buf, slot.inst_bg) = make_inst(device, &self.inst_bgl, slot.inst_cap, &slot.dyn_buf);
        }
        let dynamics = f.dynamics.as_deref().copied().unwrap_or_default();
        queue.write_buffer(&slot.dyn_buf, 0, bytemuck::bytes_of(&dynamics));
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
            // AC3D material emission: modellers use it as "self-lit" (lenses, dials),
            // far too strong as a radiance inside a dark cockpit.
            let mut emis = m.emis * if c.interior { INTERIOR_EMIT } else { 0.02 };
            if let Some((ec, _)) = &c.emit {
                // Interior: instrument backlighting / annunciators, a few tens of
                // cd/m² (1 radiance unit ≈ 30 000 cd/m²); exterior: light lenses.
                let (k, e) = if c.interior || c.backlight { (INTERIOR_EMIT, d.emit.min(1.0)) } else { (3.0, d.emit) };
                emis += *ec * e * k;
            }
            // Shininess (0..128) → roughness; specular level → f0.
            let rough = (2.0 / (m.shininess.max(0.0) + 2.0)).sqrt().clamp(0.12, 0.9);
            let spec = (m.spec.x + m.spec.y + m.spec.z) / 3.0;
            let f0 = if mode == 2 { 0.06 } else { 0.03 + 0.05 * spec.clamp(0.0, 1.0) };
            let mut uv = d.uv;
            // A glass display: its texture window → the whole display texture.
            let disp = c.display.as_ref().filter(|_| displays).and_then(|ds| {
                let i = self.disp_bgs.as_ref()?.1.iter().position(|(k, _)| *k == ds.kind)?;
                let (w, h) = ((ds.uv[2] - ds.uv[0]).max(1e-4), (ds.uv[3] - ds.uv[1]).max(1e-4));
                uv = [uv[0] / w, uv[1] / h, uv[2] / w, uv[3] / h, (uv[4] - ds.uv[0]) / w, (uv[5] - ds.uv[1]) / h];
                Some(i)
            });
            slot.disp_of.push(disp);
            let (mode, params) = match (disp, disp_lum) {
                // Mode 3: emissive screen behind smooth cover glass.
                (Some(_), Some(lum)) => (3, [0.04, 0.12, 3.0, lum]),
                _ => (mode, [f0, rough, mode as f32, if textured { 1.0 } else { 0.0 }]),
            };
            insts.push(Inst {
                model: d.model.to_cols_array_2d(),
                color: col.to_array(),
                emis: [emis.x, emis.y, emis.z, if c.interior || c.backlight { 1.0 } else { 0.0 }],
                params,
                uvm: [uv[0], uv[1], uv[2], uv[3]],
                uvo: [uv[4], uv[5], 0.0, 0.0],
            });
            let ii = (insts.len() - 1) as u32;
            slot.prepared.opaque.push((d.chunk, ii));
            if mode != 2 && mode != 3 && !c.interior {
                slot.prepared.shadow.push((d.chunk, ii));
            }
        }
        if !insts.is_empty() {
            queue.write_buffer(&slot.inst_buf, 0, bytemuck::cast_slice(&insts));
        }
    }

    fn slots(&self) -> impl Iterator<Item = &Slot> {
        std::iter::once(&self.main).chain(self.traffic[..self.n_traffic].iter())
    }

    fn draw_list(&self, pass: &mut wgpu::RenderPass<'_>, slot: &Slot, list: &[(u32, u32)], refl: bool) {
        let Some(gm) = slot.model.and_then(|i| self.models.get(i)) else { return };
        let a = &gm.asset;
        pass.set_bind_group(1, &slot.inst_bg, &[]);
        pass.set_vertex_buffer(0, gm.vb.slice(..));
        pass.set_vertex_buffer(1, gm.pv.slice(..));
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
            let disp = slot.disp_of.get(ii as usize).copied().flatten().and_then(|k| self.disp_bgs.as_ref().map(|d| &d.1[k].1));
            let bg = disp.or_else(|| m.tex.and_then(|t| gm.tex_bgs.get(t))).unwrap_or(&self.white_bg);
            pass.set_bind_group(2, bg, &[]);
            let (first, count) = gm.ranges[ci as usize];
            pass.draw_indexed(first..first + count, c.base_vertex as i32, ii..ii + 1);
        }
    }

    /// Main HDR pass (group 0 = the scene globals, already bound).
    pub fn draw_main(&self, pass: &mut wgpu::RenderPass<'_>) {
        for s in self.slots() {
            self.draw_list(pass, s, &s.prepared.opaque, false);
        }
    }

    /// Planar water reflection (group 0 = the reflection globals, already bound).
    pub fn draw_refl(&self, pass: &mut wgpu::RenderPass<'_>) {
        for s in self.slots() {
            let Some(a) = s.model.and_then(|i| self.models.get(i)).map(|m| &m.asset) else { continue };
            let list: Vec<(u32, u32)> = s.prepared.opaque.iter().copied().filter(|&(ci, _)| !a.chunks[ci as usize].interior).collect();
            self.draw_list(pass, s, &list, true);
        }
    }

    /// One shadow cascade (group 0 = the cascade's uniform, already bound).
    pub fn draw_shadow(&self, pass: &mut wgpu::RenderPass<'_>) {
        let mut set = false;
        for s in self.slots() {
            let Some(gm) = s.model.and_then(|i| self.models.get(i)) else { continue };
            if s.prepared.shadow.is_empty() {
                continue;
            }
            if !set {
                pass.set_pipeline(&self.shadow);
                set = true;
            }
            let a = &gm.asset;
            pass.set_bind_group(1, &s.inst_bg, &[]);
            pass.set_vertex_buffer(0, gm.vb.slice(..));
            pass.set_vertex_buffer(1, gm.pv.slice(..));
            pass.set_index_buffer(gm.ib.slice(..), wgpu::IndexFormat::Uint32);
            for &(ci, ii) in &s.prepared.shadow {
                let c = &a.chunks[ci as usize];
                let (first, count) = gm.ranges[ci as usize];
                pass.draw_indexed(first..first + count, c.base_vertex as i32, ii..ii + 1);
            }
        }
    }
}

fn new_slot(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout) -> Slot {
    let inst_cap = 256;
    let dyn_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("aircraft dynamics"),
        contents: bytemuck::bytes_of(&AcDynamics::default()),
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
    });
    let (inst_buf, inst_bg) = make_inst(device, bgl, inst_cap, &dyn_buf);
    Slot { inst_buf, inst_cap, inst_bg, dyn_buf, model: None, prepared: Prepared::default(), disp_of: Vec::new() }
}

impl Slot {
    /// A stand-in while `main` is borrowed out (shares its buffers; never drawn).
    fn placeholder(s: &Slot) -> Slot {
        Slot { inst_buf: s.inst_buf.clone(), inst_cap: s.inst_cap, inst_bg: s.inst_bg.clone(), dyn_buf: s.dyn_buf.clone(), model: None, prepared: Prepared::default(), disp_of: Vec::new() }
    }
}

fn tex_bg(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, view: &wgpu::TextureView, samp: &wgpu::Sampler) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("aircraft texture"),
        layout: bgl,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(view) }, wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(samp) }],
    })
}

/// Mip levels 1.. of an sRGB RGBA8 image (box filter in linear light), as
/// sRGB RGBA8. (The browser build runs this in a worker: platform::web.)
pub fn mip_chain(w: u32, h: u32, rgba: &[u8]) -> Vec<Vec<u8>> {
    let levels = 32 - w.max(h).max(1).leading_zeros();
    let to_lin: Vec<f32> = (0..256).map(|i| (i as f32 / 255.0).powf(2.2)).collect();
    let mut cur: Vec<f32> = rgba.iter().enumerate().map(|(i, &b)| if i % 4 == 3 { b as f32 / 255.0 } else { to_lin[b as usize] }).collect();
    let (mut cw, mut ch) = (w, h);
    let mut out = Vec::new();
    for _ in 1..levels {
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
        out.push(next.iter().enumerate().map(|(i, &v)| if i % 4 == 3 { (v * 255.0 + 0.5) as u8 } else { (v.max(0.0).powf(1.0 / 2.2) * 255.0 + 0.5).min(255.0) as u8 }).collect());
        cur = next;
        cw = nw;
        ch = nh;
    }
    out
}

/// sRGB RGBA8 texture with a CPU-built mip chain ([`mip_chain`]; `mips`:
/// levels 1.. already built, else empty).
fn upload_texture(device: &wgpu::Device, queue: &wgpu::Queue, w: u32, h: u32, rgba: &[u8], mips: &[Vec<u8>]) -> wgpu::TextureView {
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
    let built;
    let mips = if mips.len() + 1 == levels as usize {
        mips
    } else {
        built = mip_chain(w, h, rgba);
        &built
    };
    let (mut cw, mut ch) = (w, h);
    for level in 0..levels {
        let bytes = if level == 0 { rgba } else { &mips[level as usize - 1][..] };
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &tex, mip_level: level, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            bytes,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(cw * 4), rows_per_image: Some(ch) },
            wgpu::Extent3d { width: cw, height: ch, depth_or_array_layers: 1 },
        );
        cw = (cw / 2).max(1);
        ch = (ch / 2).max(1);
    }
    tex.create_view(&Default::default())
}
