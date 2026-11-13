//! Airport renderer: the baked static meshes per tile (markings drawn with
//! the scene's draped-layer pipeline, objects with the solid one — both
//! bound by `gfx.rs`), and the instanced parked aircraft (textured LOD 0,
//! clustered LOD 1 / 2), ground vehicles and windsocks through the scene's
//! own lighting (`shaders/airport.wgsl` is appended to the scene shader).

use super::{Frame, fleet, mesh, types};
use crate::osm::{Proj, TileId};
use crate::scene::{MeshData, Vert};
use glam::{Mat4, Vec3};
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;

struct Mesh {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    n: u32,
}

fn upload<T: bytemuck::Pod>(device: &wgpu::Device, label: &str, verts: &[T], idx: &[u32]) -> Option<Mesh> {
    if idx.is_empty() {
        return None;
    }
    Some(Mesh {
        vb: device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some(label), contents: bytemuck::cast_slice(verts), usage: wgpu::BufferUsages::VERTEX }),
        ib: device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some(label), contents: bytemuck::cast_slice(idx), usage: wgpu::BufferUsages::INDEX }),
        n: idx.len() as u32,
    })
}

/// One tile's baked statics on the GPU.
struct Static {
    id: TileId,
    ptr: usize,
    marks: Option<Mesh>,
    solids: Option<Mesh>,
    min_y: f32,
    max_y: f32,
    bytes: u64,
}

struct GpuFleet {
    /// Textures and LOD 0 uploaded (LOD 1 / 2 are drawn meanwhile).
    ready: bool,
    lod0: Option<Mesh>,
    groups: Vec<(u32, u32, Option<usize>)>,
    tex_bgs: Vec<wgpu::BindGroup>,
    lod1: Option<Mesh>,
    lod2: Option<Mesh>,
    rest: Mat4,
}

/// Instance ranges prepared for this frame.
#[derive(Default)]
struct Prepared {
    /// (type, lod, first instance, count).
    parked: Vec<(u16, u8, u32, u32)>,
    /// (vehicle kind, first, count).
    vehicles: Vec<(u16, u32, u32)>,
    /// Instance index of the identity transform (the windsock mesh).
    identity: u32,
    /// (static index, eye-relative transform, in the main view, distance).
    statics: Vec<(usize, [f32; 4], bool, f32)>,
}

pub struct AirportRenderer {
    vc_pipe: wgpu::RenderPipeline,
    tex_pipe: wgpu::RenderPipeline,
    shadow_pipe: wgpu::RenderPipeline,
    tex_bgl: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    white_bg: wgpu::BindGroup,
    statics: Vec<Static>,
    keys: HashMap<(String, TileId), usize>,
    seen_version: u64,
    fleet: HashMap<u16, GpuFleet>,
    vehicles: Vec<Option<Mesh>>,
    inst_buf: wgpu::Buffer,
    inst_cap: usize,
    dynamic: Option<Mesh>,
    prepared: Prepared,
    /// GPU bytes of the statics (budget / stats).
    pub bytes: u64,
}

/// Bytes of static meshes uploaded per frame at most.
const SYNC_BUDGET: u64 = 6 << 20;

fn scene_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32x2, 4 => Float32];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<Vert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS }
}

fn parked_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x2, 3 => Unorm8x4, 4 => Unorm8x4];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<fleet::PVert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS }
}

fn shadow_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 }];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<Vert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS }
}

fn instance_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 4] = wgpu::vertex_attr_array![5 => Float32x4, 6 => Float32x4, 7 => Float32x4, 8 => Float32x4];
    wgpu::VertexBufferLayout { array_stride: 64, step_mode: wgpu::VertexStepMode::Instance, attributes: &ATTRS }
}

fn make_inst(device: &wgpu::Device, cap: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("airport instances"),
        size: (cap * 64) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

impl AirportRenderer {
    /// `globals_bgl` / `shadow_bgl`: the scene's group-0 layouts.
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, globals_bgl: &wgpu::BindGroupLayout, shadow_bgl: &wgpu::BindGroupLayout) -> Self {
        let src = format!(
            "{}\n{}\n{}\n{}",
            include_str!("../shaders/atmos_core.wgsl"),
            include_str!("../shaders/common.wgsl"),
            include_str!("../shaders/scene.wgsl"),
            include_str!("../shaders/airport.wgsl")
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("airport"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let shadow_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("airport shadow"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/airport_shadow.wgsl").into()),
        });
        let tex_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("airport texture"),
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
        let vc_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("airport vc"), bind_group_layouts: &[Some(globals_bgl)], immediate_size: 0 });
        let tex_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("airport tex"),
            bind_group_layouts: &[Some(globals_bgl), Some(&tex_bgl)],
            immediate_size: 0,
        });
        let shadow_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("airport shadow"), bind_group_layouts: &[Some(shadow_bgl)], immediate_size: 0 });
        let main = |label: &str, layout: &wgpu::PipelineLayout, vs: &str, fs: &str, vbl: wgpu::VertexBufferLayout<'static>| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers: &[Some(vbl), Some(instance_layout())] },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: crate::gfx::DEPTH,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: crate::gfx::MSAA, mask: !0, alpha_to_coverage_enabled: true },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
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
        let vc_pipe = main("airport vc", &vc_layout, "vs_inst", "fs_main", scene_vertex_layout());
        let tex_pipe = main("airport parked", &tex_layout, "vs_parked", "fs_parked", parked_vertex_layout());
        let shadow_pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("airport shadow"),
            layout: Some(&shadow_layout),
            vertex: wgpu::VertexState {
                module: &shadow_mod,
                entry_point: Some("vs_ap_shadow"),
                compilation_options: Default::default(),
                buffers: &[Some(shadow_vertex_layout()), Some(instance_layout())],
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
            label: Some("airport"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        let white = upload_texture(device, queue, 1, 1, &[255, 255, 255, 255], &[]);
        let white_bg = tex_bg(device, &tex_bgl, &white, &sampler);
        let vehicles = mesh::VEHICLES.iter().map(|v| {
            let m = mesh::vehicle(*v, 0);
            upload(device, "vehicle", &m.verts, &m.idx)
        });
        let inst_cap = 1024;
        AirportRenderer {
            vc_pipe,
            tex_pipe,
            shadow_pipe,
            tex_bgl,
            sampler,
            white_bg,
            statics: Vec::new(),
            keys: HashMap::new(),
            seen_version: u64::MAX,
            fleet: HashMap::new(),
            vehicles: vehicles.collect(),
            inst_buf: make_inst(device, inst_cap),
            inst_cap,
            dynamic: None,
            prepared: Prepared::default(),
            bytes: 0,
        }
    }

    /// Mirror the scene's bakes on the GPU (when they changed).
    pub fn sync(&mut self, device: &wgpu::Device, scene: &super::AirportScene) {
        if scene.version == self.seen_version {
            return;
        }
        let (mut spent, mut pending) = (0u64, false);
        let mut keep: HashMap<(String, TileId), usize> = HashMap::new();
        let mut statics: Vec<Static> = Vec::new();
        let mut old: Vec<Option<Static>> = std::mem::take(&mut self.statics).into_iter().map(Some).collect();
        for (site, b) in scene.bakes() {
            let key = (site.to_string(), b.id);
            let ptr = Arc::as_ptr(b) as usize;
            let reuse = self.keys.get(&key).and_then(|&i| old[i].take_if(|s| s.ptr == ptr));
            let s = match reuse {
                Some(s) => s,
                // Upload budget per frame: the rest next frame (keeps streaming smooth).
                None if spent > SYNC_BUDGET => {
                    pending = true;
                    continue;
                }
                None => {
                    spent += ((b.marks.verts.len() + b.solids.verts.len()) * std::mem::size_of::<Vert>()) as u64;
                    let marks = upload(device, "airport marks", &b.marks.verts, &b.marks.idx);
                    let solids = upload(device, "airport objects", &b.solids.verts, &b.solids.idx);
                    let bytes = ((b.marks.verts.len() + b.solids.verts.len()) * std::mem::size_of::<Vert>() + (b.marks.idx.len() + b.solids.idx.len()) * 4) as u64;
                    Static { id: b.id, ptr, marks, solids, min_y: b.min_y, max_y: b.max_y, bytes }
                }
            };
            keep.insert(key, statics.len());
            statics.push(s);
        }
        self.bytes = statics.iter().map(|s| s.bytes).sum();
        self.statics = statics;
        self.keys = keep;
        if !pending {
            self.seen_version = scene.version;
        }
    }

    /// Upload this frame's instances; place the statics relative to the eye.
    /// `visible(lo, hi)`: the main view's frustum test (eye-relative box).
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, proj: &Proj, eye: Vec3, frame: Option<&Frame>, visible: &dyn Fn(Vec3, Vec3) -> bool) {
        let p = &mut self.prepared;
        p.parked.clear();
        p.vehicles.clear();
        p.statics.clear();
        let (ex, ey, ez) = (eye.x as f64, eye.y as f64, eye.z as f64);
        for (i, s) in self.statics.iter().enumerate() {
            let m = Proj::tile_frame(s.id).map_to(proj);
            let xf = [(m.sx - 1.0) as f32, (m.tx - ex) as f32, (m.tz - ez) as f32, -ey as f32];
            let tf = Proj::tile_frame(s.id);
            let (lo, hi) = s.id.rect(&tf);
            let (a, b) = (m.apply(lo), m.apply(hi));
            // Objects overhang their tile a little (bridges, signs).
            let lo3 = Vec3::new(a.x.min(b.x) - 60.0 - eye.x, s.min_y - eye.y, a.y.min(b.y) - 60.0 - eye.z);
            let hi3 = Vec3::new(a.x.max(b.x) + 60.0 - eye.x, s.max_y + 2.0 - eye.y, a.y.max(b.y) + 60.0 - eye.z);
            let d = Vec3::ZERO.clamp(lo3, hi3).length();
            p.statics.push((i, xf, visible(lo3, hi3), d));
        }
        let Some(f) = frame else { return };
        // Load / upload the parked types in view — spread over frames (a
        // budget of texture bytes per frame) so streaming stays smooth.
        let mut need: Vec<u16> = f.parked.iter().map(|i| i.model).collect();
        need.sort_unstable();
        need.dedup();
        let mut budget: i64 = 8 << 20;
        for ty in need {
            if self.fleet.get(&ty).is_some_and(|g| g.ready) || budget <= 0 {
                continue;
            }
            let Some(m) = fleet::get(ty) else { continue };
            let g = self.fleet.entry(ty).or_insert_with(|| GpuFleet {
                lod0: None,
                groups: m.groups.iter().map(|g| (g.first, g.count, g.tex)).collect(),
                tex_bgs: Vec::new(),
                lod1: upload(device, "parked lod1", &m.lod1.verts, &m.lod1.idx),
                lod2: upload(device, "parked lod2", &m.lod2.verts, &m.lod2.idx),
                rest: m.rest,
                ready: false,
            });
            while g.tex_bgs.len() < m.textures.len() && budget > 0 {
                let t = &m.textures[g.tex_bgs.len()];
                budget -= (t.rgba.len() * 4 / 3) as i64;
                g.tex_bgs.push(tex_bg(device, &self.tex_bgl, &upload_texture(device, queue, t.w, t.h, &t.rgba, &t.mips), &self.sampler));
            }
            if g.tex_bgs.len() == m.textures.len() && budget > 0 {
                budget -= (m.verts.len() * std::mem::size_of::<fleet::PVert>()) as i64;
                g.lod0 = upload(device, "parked lod0", &m.verts, &m.idx);
                g.ready = true;
            }
        }
        // Instance data grouped by (type, lod) and by vehicle kind.
        let mut data: Vec<[f32; 16]> = Vec::with_capacity(f.parked.len() + f.vehicles.len() + 1);
        let mut parked = f.parked.clone();
        parked.retain(|i| self.fleet.contains_key(&i.model));
        for i in &mut parked {
            if i.lod == 0 && !self.fleet[&i.model].ready {
                i.lod = 1;
            }
        }
        parked.sort_by_key(|i| (i.model, i.lod));
        let mut k = 0;
        while k < parked.len() {
            let (ty, lod) = (parked[k].model, parked[k].lod);
            let first = data.len() as u32;
            let rest = self.fleet[&ty].rest;
            while k < parked.len() && parked[k].model == ty && parked[k].lod == lod {
                data.push((parked[k].xf * rest).to_cols_array());
                k += 1;
            }
            p.parked.push((ty, lod, first, data.len() as u32 - first));
        }
        let mut veh = f.vehicles.clone();
        veh.sort_by_key(|i| i.model);
        let mut k = 0;
        while k < veh.len() {
            let kind = veh[k].model;
            let first = data.len() as u32;
            while k < veh.len() && veh[k].model == kind {
                data.push(veh[k].xf.to_cols_array());
                k += 1;
            }
            p.vehicles.push((kind, first, data.len() as u32 - first));
        }
        p.identity = data.len() as u32;
        data.push(Mat4::IDENTITY.to_cols_array());
        if data.len() > self.inst_cap {
            self.inst_cap = data.len().next_power_of_two();
            self.inst_buf = make_inst(device, self.inst_cap);
        }
        queue.write_buffer(&self.inst_buf, 0, bytemuck::cast_slice(&data));
        self.dynamic = upload(device, "windsocks", &f.dynamic.verts, &f.dynamic.idx);
    }

    /// Ground markings (pipeline: the scene's draped-layer pipeline, bound by
    /// the caller). `resident(id)`: the tile's own scenery is drawn (the
    /// markings lie on its terrain).
    pub fn draw_marks(&self, pass: &mut wgpu::RenderPass<'_>, xfs: &crate::gfx::Xforms, resident: &dyn Fn(TileId) -> bool) {
        for &(i, xf, vis, d) in &self.prepared.statics {
            let s = &self.statics[i];
            let Some(m) = &s.marks else { continue };
            if !vis || d > 9000.0 || !resident(s.id) {
                continue;
            }
            xfs.set(pass, &xf);
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, 0..1);
        }
    }

    /// Static objects (pipeline: solid / shadow / reflection, bound by the
    /// caller). `max_dist`: shadow cascades / the reflection draw what is
    /// within their range whether or not it is in the main view.
    pub fn draw_objects(&self, pass: &mut wgpu::RenderPass<'_>, xfs: &crate::gfx::Xforms, resident: &dyn Fn(TileId) -> bool, main_view: bool, max_dist: f32) {
        for &(i, xf, vis, d) in &self.prepared.statics {
            let s = &self.statics[i];
            let Some(m) = &s.solids else { continue };
            if (main_view && !vis) || d > max_dist || !resident(s.id) {
                continue;
            }
            xfs.set(pass, &xf);
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, 0..1);
        }
    }

    /// Parked aircraft, vehicles and windsocks in the main pass (group 0 bound).
    pub fn draw_main(&self, pass: &mut wgpu::RenderPass<'_>) {
        let p = &self.prepared;
        pass.set_pipeline(&self.vc_pipe);
        pass.set_vertex_buffer(1, self.inst_buf.slice(..));
        for &(ty, lod, first, n) in &p.parked {
            let Some(f) = self.fleet.get(&ty) else { continue };
            let m = match lod {
                0 => continue,
                1 => &f.lod1,
                _ => &f.lod2,
            };
            if let Some(m) = m {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
        for &(kind, first, n) in &p.vehicles {
            if let Some(Some(m)) = self.vehicles.get(kind as usize) {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
        if let Some(m) = &self.dynamic {
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, p.identity..p.identity + 1);
        }
        // Textured LOD 0, one draw per texture of each type.
        pass.set_pipeline(&self.tex_pipe);
        for &(ty, lod, first, n) in &p.parked {
            if lod != 0 {
                continue;
            }
            let Some(f) = self.fleet.get(&ty) else { continue };
            let Some(m) = &f.lod0 else { continue };
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            for &(g0, gn, tex) in &f.groups {
                let bg = tex.and_then(|t| f.tex_bgs.get(t)).unwrap_or(&self.white_bg);
                pass.set_bind_group(1, bg, &[]);
                pass.draw_indexed(g0..g0 + gn, 0, first..first + n);
            }
        }
    }

    /// Shadow cascade (group 0 = the cascade uniform, bound): LOD 1 / 2 aircraft, vehicles, windsocks.
    pub fn draw_shadow(&self, pass: &mut wgpu::RenderPass<'_>, cascade: usize) {
        let p = &self.prepared;
        if p.parked.is_empty() && p.vehicles.is_empty() {
            return;
        }
        pass.set_pipeline(&self.shadow_pipe);
        pass.set_vertex_buffer(1, self.inst_buf.slice(..));
        for &(ty, lod, first, n) in &p.parked {
            let Some(f) = self.fleet.get(&ty) else { continue };
            if lod == 2 && cascade < 2 {
                continue;
            }
            let m = if lod == 2 { &f.lod2 } else { &f.lod1 };
            if let Some(m) = m {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
        if cascade < 3 {
            for &(kind, first, n) in &p.vehicles {
                if let Some(Some(m)) = self.vehicles.get(kind as usize) {
                    pass.set_vertex_buffer(0, m.vb.slice(..));
                    pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                    pass.draw_indexed(0..m.n, 0, first..first + n);
                }
            }
        }
        if let (Some(m), true) = (&self.dynamic, cascade < 2) {
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, p.identity..p.identity + 1);
        }
    }

    /// Number of parked aircraft / vehicles drawn this frame.
    pub fn counts(&self) -> (u32, u32) {
        (self.prepared.parked.iter().map(|p| p.3).sum(), self.prepared.vehicles.iter().map(|v| v.2).sum())
    }
}

/// Fleet entry (for type stats).
pub fn type_name(ty: u16) -> &'static str {
    types::get(ty).id
}

/// Static mesh helper for tests.
pub fn mesh_bytes(m: &MeshData) -> usize {
    m.verts.len() * std::mem::size_of::<Vert>() + m.idx.len() * 4
}

fn tex_bg(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, view: &wgpu::TextureView, samp: &wgpu::Sampler) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("parked texture"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(view) },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(samp) },
        ],
    })
}

/// sRGB RGBA8 texture with its (precomputed) mip chain.
fn upload_texture(device: &wgpu::Device, queue: &wgpu::Queue, w: u32, h: u32, rgba: &[u8], mips: &[Vec<u8>]) -> wgpu::TextureView {
    let levels = 1 + mips.len() as u32;
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("parked texture"),
        size: wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
        mip_level_count: levels,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba8UnormSrgb,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    let (mut cw, mut ch) = (w, h);
    for (level, bytes) in std::iter::once(rgba).chain(mips.iter().map(|m| &m[..])).enumerate() {
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &tex, mip_level: level as u32, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            bytes,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(cw * 4), rows_per_image: Some(ch) },
            wgpu::Extent3d { width: cw, height: ch, depth_or_array_layers: 1 },
        );
        cw = (cw / 2).max(1);
        ch = (ch / 2).max(1);
    }
    tex.create_view(&Default::default())
}
