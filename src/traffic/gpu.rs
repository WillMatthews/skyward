//! Traffic renderer: one instance buffer per frame (eye-relative,
//! extrapolated to the frame's time on the CPU), drawn per (kind, LOD) with
//! the scene's lighting (`shaders/traffic.wgsl` is appended to the scene
//! shader), in the shadow cascades near the eye, and as head / tail / brake
//! light sprites in the lights pass (lights.wgsl `vs_vehicle_lights`).

use super::{Snapshot, VehInst, mesh};
use crate::scene::Vert;
use glam::Vec3;
use wgpu::util::DeviceExt;

struct Mesh {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    n: u32,
}

/// Vertex layout of the instances (locations 5..9; == traffic.wgsl, lights.wgsl).
pub fn instance_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![5 => Float32x3, 6 => Float32x2, 7 => Float32, 8 => Uint32, 9 => Uint32];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<VehInst>() as u64, step_mode: wgpu::VertexStepMode::Instance, attributes: &ATTRS }
}

fn scene_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32x2, 4 => Float32];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<Vert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS }
}

fn shadow_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 }];
    wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<Vert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &ATTRS }
}

/// Shadows in the first three cascades (to ~1.1 km; gfx.rs SPLITS), cast by the box LOD.
const SHADOW_CASCADES: usize = 3;

pub struct TrafficRenderer {
    main: wgpu::RenderPipeline,
    /// Velocity pass (gfx/taa.rs: TAA / motion blur) and the vehicles' world velocities.
    vel_pipe: wgpu::RenderPipeline,
    vel: wgpu::Buffer,
    vel_staging: Vec<[f32; 4]>,
    shadow: wgpu::RenderPipeline,
    /// [kind][lod].
    meshes: Vec<[Option<Mesh>; mesh::LODS]>,
    inst: wgpu::Buffer,
    cap: usize,
    /// Instances this frame (all LODs; the light sprites draw them all).
    n: u32,
    ranges: Vec<(u8, u8, u32, u32)>,
    staging: Vec<VehInst>,
    pub stats: super::Stats,
    /// CPU time of the last `prepare` (ms).
    pub prepare_ms: f32,
    /// GPU cost measurement: draw everything this many times (SKYWARD_TRAFFIC_GPU_REPEAT).
    repeat: u32,
}

fn make_vel(device: &wgpu::Device, cap: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("traffic velocities"),
        size: (cap * 16) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

fn make_inst(device: &wgpu::Device, cap: usize) -> wgpu::Buffer {
    device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("traffic instances"),
        size: (cap * std::mem::size_of::<VehInst>()) as u64,
        usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    })
}

impl TrafficRenderer {
    /// `globals_bgl` / `shadow_bgl`: the scene's group-0 layouts.
    pub fn new(device: &wgpu::Device, globals_bgl: &wgpu::BindGroupLayout, shadow_bgl: &wgpu::BindGroupLayout) -> Self {
        let src = format!(
            "{}\n{}\n{}\n{}",
            include_str!("../shaders/atmos_core.wgsl"),
            include_str!("../shaders/common.wgsl"),
            include_str!("../shaders/scene.wgsl"),
            include_str!("../shaders/traffic.wgsl")
        );
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("traffic"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let shadow_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("traffic shadow"),
            source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/traffic_shadow.wgsl").into()),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("traffic"), bind_group_layouts: &[Some(globals_bgl)], immediate_size: 0 });
        let shadow_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("traffic shadow"), bind_group_layouts: &[Some(shadow_bgl)], immediate_size: 0 });
        let main = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("traffic"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &module,
                entry_point: Some("vs_vehicle"),
                compilation_options: Default::default(),
                buffers: &[Some(scene_vertex_layout()), Some(instance_layout())],
            },
            primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: crate::gfx::DEPTH,
                depth_write_enabled: Some(true),
                depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                stencil: Default::default(),
                bias: Default::default(),
            }),
            multisample: wgpu::MultisampleState { count: crate::gfx::msaa(), mask: !0, alpha_to_coverage_enabled: crate::gfx::msaa() > 1 },
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[
                    Some(wgpu::ColorTargetState { format: crate::gfx::HDR, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                    Some(wgpu::ColorTargetState { format: crate::gfx::DIST, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                ],
            }),
            multiview_mask: None,
            cache: None,
        });
        let vel_pipe = {
            let vel_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some("traffic velocity"),
                source: wgpu::ShaderSource::Wgsl(include_str!("../shaders/traffic_vel.wgsl").into()),
            });
            let vbgl = crate::gfx::taa::vel_bgl(device);
            let vlayout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("traffic velocity"), bind_group_layouts: &[Some(&vbgl)], immediate_size: 0 });
            const VATTR: [wgpu::VertexAttribute; 1] = [wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x4, offset: 0, shader_location: 10 }];
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some("traffic velocity"),
                layout: Some(&vlayout),
                vertex: wgpu::VertexState {
                    module: &vel_mod,
                    entry_point: Some("vs_vel"),
                    compilation_options: Default::default(),
                    buffers: &[
                        Some(shadow_vertex_layout()),
                        Some(instance_layout()),
                        Some(wgpu::VertexBufferLayout { array_stride: 16, step_mode: wgpu::VertexStepMode::Instance, attributes: &VATTR }),
                    ],
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
                    module: &vel_mod,
                    entry_point: Some("fs_vel"),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: crate::gfx::taa::VEL, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let shadow = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("traffic shadow"),
            layout: Some(&shadow_layout),
            vertex: wgpu::VertexState {
                module: &shadow_mod,
                entry_point: Some("vs_veh_shadow"),
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
        let up = |m: &crate::scene::MeshData| {
            (!m.idx.is_empty()).then(|| Mesh {
                vb: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("vehicle"),
                    contents: bytemuck::cast_slice(&m.verts),
                    usage: wgpu::BufferUsages::VERTEX,
                }),
                ib: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some("vehicle"),
                    contents: bytemuck::cast_slice(&m.idx),
                    usage: wgpu::BufferUsages::INDEX,
                }),
                n: m.idx.len() as u32,
            })
        };
        let meshes = mesh::all().iter().map(|l| [up(&l[0]), up(&l[1]), up(&l[2])]).collect();
        let cap = 4096;
        let repeat = std::env::var("SKYWARD_TRAFFIC_GPU_REPEAT").ok().and_then(|v| v.parse().ok()).unwrap_or(1u32).clamp(1, 50);
        TrafficRenderer {
            main,
            shadow,
            meshes,
            inst: make_inst(device, cap),
            cap,
            n: 0,
            ranges: Vec::new(),
            staging: Vec::new(),
            vel_pipe,
            vel: make_vel(device, cap),
            vel_staging: Vec::new(),
            stats: Default::default(),
            prepare_ms: 0.0,
            repeat,
        }
    }

    /// Upload this frame's vehicles: positions extrapolated from the
    /// snapshot to `time` and made relative to `eye`. None: no traffic.
    pub fn prepare(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, snap: Option<&Snapshot>, time: f64, eye: Vec3) {
        let t0 = web_time::Instant::now();
        self.n = 0;
        self.ranges.clear();
        let Some(s) = snap else {
            self.stats = Default::default();
            return;
        };
        let dt = (time - s.time).clamp(0.0, 0.25) as f32;
        self.staging.clear();
        self.staging.extend(s.inst.iter().zip(&s.speed).map(|(i, &v)| {
            let d = v * dt;
            VehInst { pos: [i.pos[0] - eye.x + i.dx * d, i.pos[1] - eye.y + i.grade * d, i.pos[2] - eye.z + i.dz * d], ..*i }
        }));
        if self.staging.is_empty() {
            self.stats = s.stats;
            return;
        }
        if self.staging.len() > self.cap {
            self.cap = self.staging.len().next_power_of_two();
            self.inst = make_inst(device, self.cap);
            self.vel = make_vel(device, self.cap);
        }
        queue.write_buffer(&self.inst, 0, bytemuck::cast_slice(&self.staging));
        // World velocities (velocity pass).
        self.vel_staging.clear();
        self.vel_staging.extend(s.inst.iter().zip(&s.speed).map(|(i, &v)| [i.dx * v, i.grade * v, i.dz * v, 0.0]));
        queue.write_buffer(&self.vel, 0, bytemuck::cast_slice(&self.vel_staging));
        self.n = self.staging.len() as u32;
        self.ranges.extend_from_slice(&s.ranges);
        self.stats = s.stats;
        self.prepare_ms = t0.elapsed().as_secs_f32() * 1000.0;
    }

    /// The instance buffer and count for the light sprites.
    pub fn lights(&self) -> Option<(&wgpu::Buffer, u32, u32)> {
        (self.n > 0).then_some((&self.inst, self.n, self.repeat))
    }

    /// Vehicles in the main pass (group 0 bound).
    pub fn draw_main(&self, pass: &mut wgpu::RenderPass<'_>) {
        if self.n == 0 {
            return;
        }
        pass.set_pipeline(&self.main);
        pass.set_vertex_buffer(1, self.inst.slice(..));
        for &(k, lod, first, n) in self.ranges.iter().cycle().take(self.ranges.len() * self.repeat as usize) {
            if lod as usize >= mesh::LODS {
                continue;
            }
            if let Some(m) = &self.meshes[k as usize][lod as usize] {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
    }

    /// Velocity pass (group 0 = gfx/taa.rs velocity uniforms): each vehicle's
    /// screen motion since last frame, by the box LOD's outline at range.
    pub fn draw_velocity(&self, pass: &mut wgpu::RenderPass<'_>, vel_bg: &wgpu::BindGroup) {
        if self.n == 0 {
            return;
        }
        pass.set_pipeline(&self.vel_pipe);
        pass.set_bind_group(0, vel_bg, &[]);
        pass.set_vertex_buffer(1, self.inst.slice(..));
        pass.set_vertex_buffer(2, self.vel.slice(..));
        for &(k, lod, first, n) in &self.ranges {
            if lod as usize >= mesh::LODS {
                continue;
            }
            if let Some(m) = &self.meshes[k as usize][lod as usize] {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
    }

    /// Shadow cascade `cascade` (group 0 = the cascade uniform, bound).
    pub fn draw_shadow(&self, pass: &mut wgpu::RenderPass<'_>, cascade: usize) {
        if self.n == 0 || cascade >= SHADOW_CASCADES {
            return;
        }
        pass.set_pipeline(&self.shadow);
        pass.set_vertex_buffer(1, self.inst.slice(..));
        for &(k, lod, first, n) in self.ranges.iter().cycle().take(self.ranges.len() * self.repeat as usize) {
            if lod as usize >= mesh::LODS || (cascade == 2 && lod == 0) {
                continue;
            }
            // (The box LOD casts every shadow: cheap, the same outline.)
            if let Some(m) = &self.meshes[k as usize][mesh::LODS - 1] {
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, first..first + n);
            }
        }
    }
}
