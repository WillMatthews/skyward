//! wgpu renderer: physically based atmosphere (sky, aerial perspective,
//! ambient, exposure), volumetric clouds with cloud shadows, 4 blended shadow
//! cascades, planar water reflection, MSAA HDR scene, bloom and AgX tonemap.
//! Works with a window surface or headless.

use crate::atmosphere::{self, AtmosFrame, Atmosphere};
use crate::clouds::{CloudField, CloudFrame, CloudRenderer};
use crate::osm::{Proj, TileId, Xform2};
use crate::scene::{self, MeshData, TreeInst, Vert};
use crate::world::TileMeshes;
use glam::{vec3, Mat4, Vec2, Vec3, Vec4};
use std::collections::HashMap;
use wgpu::util::DeviceExt;

pub const HDR: wgpu::TextureFormat = wgpu::TextureFormat::Rgba16Float;
pub const DEPTH: wgpu::TextureFormat = wgpu::TextureFormat::Depth32Float;
/// Eye distance (m) written by every scene pass (clouds depth-test against it).
pub const DIST: wgpu::TextureFormat = wgpu::TextureFormat::R16Float;
pub const MSAA: u32 = 4;
pub const SHADOW_RES: u32 = 4096;
const CASCADES: usize = 4;
const SPLITS: [f32; 5] = [0.3, 45.0, 220.0, 1100.0, 6000.0];
/// Fraction of each cascade over which it blends into the next.
const CASCADE_BLEND: f32 = 0.15;
const BLOOM_LEVELS: usize = 6;
const NEAR: f32 = 0.25;
/// Bytes of immediate data: the per-tile transform (osm::Xform2::gpu).
const IMMEDIATE_SIZE: u32 = 16;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct Globals {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    shadow: [[[f32; 4]; 4]; CASCADES],
    cam_pos: [f32; 4],
    sun_dir: [f32; 4],
    sun_e: [f32; 4],
    splits: [f32; 4],
    shadow_info: [f32; 4],
    screen: [f32; 4],
    cloud: [f32; 4],
    texel: [f32; 4],
    prev_view_proj: [[f32; 4]; 4],
    /// Weather: wetness, falling rain, falling snow, snow cover.
    wx: [f32; 4],
    /// Surface wind x, z (m/s), hoar frost, gust (m/s).
    wx2: [f32; 4],
    /// Season: leaf cover, autumn colour, spring freshness, grass dormancy.
    season: [f32; 4],
    /// Season: summer dryness, year phase, season strength, drizzle flag.
    season2: [f32; 4],
    /// Pattern space of the current frame (World::pattern_frame).
    pattern: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct AoU {
    inv_view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    params: [f32; 4],
    size: [f32; 4],
}

/// SSAO world radius (m) and strength.
const AO_RADIUS: f32 = 5.0;
const AO_STRENGTH: f32 = 1.4;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct PostU {
    params: [f32; 4],
    texel: [f32; 4],
}

const UNIFORM_STRIDE: u64 = 256;

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct FogU {
    inv_view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    fog: [f32; 4],
    sun: [f32; 4],
    screen: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct RainU {
    view_proj: [[f32; 4]; 4],
    eye: [f32; 4],
    cam_vel: [f32; 4],
    params: [f32; 4],
    sun: [f32; 4],
    screen: [f32; 4],
}

/// Rain / snow particles per nested layer (must match `PER_LAYER` in rain.wgsl).
const RAIN_PER_LAYER: u32 = 9000;

/// Ground fog + precipitation particles (weather passes after the clouds).
struct WeatherPasses {
    fog_pipe: wgpu::RenderPipeline,
    fog_buf: wgpu::Buffer,
    rain_pipe: wgpu::RenderPipeline,
    rain_buf: wgpu::Buffer,
    /// Last (eye, time) and the smoothed camera velocity, for rain streaks.
    last_eye: Option<(Vec3, f32)>,
    cam_vel: Vec3,
}

fn premul_blend() -> wgpu::BlendState {
    wgpu::BlendState {
        color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::SrcAlpha, operation: wgpu::BlendOperation::Add },
        alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
    }
}

impl WeatherPasses {
    fn new(device: &wgpu::Device) -> Self {
        let core = include_str!("shaders/atmos_core.wgsl");
        let fog_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("fog"),
            source: wgpu::ShaderSource::Wgsl(format!("{core}\n{}", include_str!("shaders/fog.wgsl")).into()),
        });
        let rain_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rain"),
            source: wgpu::ShaderSource::Wgsl(format!("{core}\n{}", include_str!("shaders/rain.wgsl")).into()),
        });
        let pipe = |label: &str, module: &wgpu::ShaderModule, vs: &str, fs: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: None,
                vertex: wgpu::VertexState { module, entry_point: Some(vs), compilation_options: Default::default(), buffers: &[] },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format: HDR, blend: Some(premul_blend()), write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let ubuf = |label: &str, size: usize| {
            device.create_buffer(&wgpu::BufferDescriptor { label: Some(label), size: size as u64, usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST, mapped_at_creation: false })
        };
        WeatherPasses {
            fog_pipe: pipe("fog", &fog_mod, "vs_full", "fs_fog"),
            fog_buf: ubuf("fog u", std::mem::size_of::<FogU>()),
            rain_pipe: pipe("rain", &rain_mod, "vs_rain", "fs_rain"),
            rain_buf: ubuf("rain u", std::mem::size_of::<RainU>()),
            last_eye: None,
            cam_vel: Vec3::ZERO,
        }
    }
}

/// Camera + lighting for one frame.
pub struct FrameView {
    pub eye: Vec3,
    pub view: Mat4,
    pub fov_y: f32,
    pub time: f32,
    pub sun_dir: Vec3,
    /// Manual exposure multiplier on top of the metered exposure.
    pub exposure: f32,
}

/// Trees are drawn up to this distance (m) from the eye; detailed within TREE_LOD0.
const TREE_DIST: f32 = 4200.0;
const TREE_LOD0: f32 = 1100.0;
const TREE_REFL_DIST: f32 = 1500.0;
const BUILDING_REFL_DIST: f32 = 6000.0;
/// Half-size of the base ground quad that follows the camera (m).
const GROUND_HALF: f32 = 60_000.0;

/// View-frustum planes extracted from a clip matrix (Gribb–Hartmann, depth 0..1).
struct Frustum([Vec4; 6]);

impl Frustum {
    fn new(m: Mat4) -> Self {
        let r = |i| m.row(i);
        Frustum([r(3) + r(0), r(3) - r(0), r(3) + r(1), r(3) - r(1), r(2), r(3) - r(2)])
    }
    fn aabb(&self, lo: Vec3, hi: Vec3) -> bool {
        self.0.iter().all(|p| {
            let v = vec3(if p.x > 0.0 { hi.x } else { lo.x }, if p.y > 0.0 { hi.y } else { lo.y }, if p.z > 0.0 { hi.z } else { lo.z });
            p.truncate().dot(v) + p.w >= 0.0
        })
    }
}

/// One streamed map tile on the GPU.
struct GpuTile {
    id: TileId,
    /// Bounds in the current frame.
    lo: Vec2,
    hi: Vec2,
    /// Tile frame → current frame (exact, f64).
    map: Xform2,
    /// Tile frame → eye-relative, this frame (immediate data for its draws).
    xf: [f32; 4],
    max_h: f32,
    flat: Option<GpuMesh>,
    buildings: Option<GpuMesh>,
    /// Instance buffer: deciduous [0, split), conifers [split, n).
    trees: Option<(wgpu::Buffer, u32, u32)>,
    bytes: u64,
}

impl GpuTile {
    fn dist(&self, p: Vec2) -> f32 {
        (self.lo - p).max(p - self.hi).max(Vec2::ZERO).length()
    }
    fn flat_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x, -0.5, self.lo.y), vec3(self.hi.x, 0.5, self.hi.y))
    }
    fn solid_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x, 0.0, self.lo.y), vec3(self.hi.x, self.max_h + 1.0, self.hi.y))
    }
    fn tree_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x - 10.0, 0.0, self.lo.y - 10.0), vec3(self.hi.x + 10.0, 26.0, self.hi.y + 10.0))
    }
}

/// A tile whose buffers exist but are still being filled.
struct PendingTile {
    tile: GpuTile,
    data: TileMeshes,
    part: usize,
    offset: usize,
}

/// Shared tree meshes: [deciduous, conifer] x [lod0, lod1].
struct TreeMeshes {
    m: [[GpuMesh; 2]; 2],
}

/// Optional GPU pass timing (env `SKYWARD_GPU_TIMING=1`, needs timestamp
/// queries): prints a per-pass breakdown averaged over 120 frames.
struct Profiler {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    read: wgpu::Buffer,
    period: f32,
    n: u32,
    sums: Vec<f64>,
    frames: u32,
}

const PROF_MARKS: [&str; 8] = ["start", "atmos+cloudmaps", "shadows", "reflection", "main", "ao", "clouds", "post"];

impl Profiler {
    fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Option<Self> {
        if std::env::var("SKYWARD_GPU_TIMING").is_err() || !device.features().contains(wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS) {
            return None;
        }
        let n = PROF_MARKS.len() as u32;
        let set = device.create_query_set(&wgpu::QuerySetDescriptor { label: Some("prof"), ty: wgpu::QueryType::Timestamp, count: n });
        let size = 8 * n as u64;
        let resolve = device.create_buffer(&wgpu::BufferDescriptor { label: Some("prof"), size, usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC, mapped_at_creation: false });
        let read = device.create_buffer(&wgpu::BufferDescriptor { label: Some("prof read"), size, usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ, mapped_at_creation: false });
        Some(Profiler { set, resolve, read, period: queue.get_timestamp_period(), n, sums: vec![0.0; n as usize], frames: 0 })
    }
    fn mark(&self, encoder: &mut wgpu::CommandEncoder, i: usize) {
        encoder.write_timestamp(&self.set, i as u32);
    }
    fn finish(&self, encoder: &mut wgpu::CommandEncoder) {
        encoder.resolve_query_set(&self.set, 0..self.n, &self.resolve, 0);
        encoder.copy_buffer_to_buffer(&self.resolve, 0, &self.read, 0, 8 * self.n as u64);
    }
}

/// Per-frame render statistics.
#[derive(Default, Clone, Copy, Debug)]
pub struct RenderStats {
    pub tiles: usize,
    pub tiles_drawn: usize,
    pub tree_instances: usize,
    pub gpu_bytes: u64,
}

struct GpuMesh {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    n: u32,
}

impl GpuMesh {
    fn bytes(&self) -> u64 {
        self.vb.size() + self.ib.size()
    }
}

fn upload(device: &wgpu::Device, label: &str, m: &MeshData) -> Option<GpuMesh> {
    if m.idx.is_empty() {
        return None;
    }
    Some(GpuMesh {
        vb: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytemuck::cast_slice(&m.verts),
            usage: wgpu::BufferUsages::VERTEX,
        }),
        ib: device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some(label),
            contents: bytemuck::cast_slice(&m.idx),
            usage: wgpu::BufferUsages::INDEX,
        }),
        n: m.idx.len() as u32,
    })
}

struct DynMesh {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    vcap: usize,
    icap: usize,
    n: u32,
}

impl DynMesh {
    fn new(device: &wgpu::Device, vcap: usize, icap: usize) -> Self {
        DynMesh {
            vb: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("dyn vb"),
                size: (vcap * std::mem::size_of::<Vert>()) as u64,
                usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            ib: device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("dyn ib"),
                size: (icap * 4) as u64,
                usage: wgpu::BufferUsages::INDEX | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            }),
            vcap,
            icap,
            n: 0,
        }
    }
    fn set(&mut self, device: &wgpu::Device, queue: &wgpu::Queue, m: &MeshData) {
        if m.verts.len() > self.vcap || m.idx.len() > self.icap {
            *self = DynMesh::new(device, m.verts.len().next_power_of_two(), m.idx.len().next_power_of_two());
        }
        if !m.idx.is_empty() {
            queue.write_buffer(&self.vb, 0, bytemuck::cast_slice(&m.verts));
            queue.write_buffer(&self.ib, 0, bytemuck::cast_slice(&m.idx));
        }
        self.n = m.idx.len() as u32;
    }
}

struct Targets {
    width: u32,
    height: u32,
    hdr_msaa: wgpu::TextureView,
    hdr: wgpu::TextureView,
    dist_msaa: wgpu::TextureView,
    dist: wgpu::TextureView,
    depth: wgpu::TextureView,
    refl: wgpu::TextureView,
    refl_dist: wgpu::TextureView,
    refl_depth: wgpu::TextureView,
    refl_size: (u32, u32),
    ao_raw: wgpu::TextureView,
    ao: wgpu::TextureView,
    ao_size: (u32, u32),
    ao_bg: wgpu::BindGroup,
    ao_blur_bg: wgpu::BindGroup,
    bloom: Vec<(wgpu::TextureView, u32, u32)>,
    main_bg: wgpu::BindGroup,
    refl_bg: wgpu::BindGroup,
    /// bind groups for bloom down passes (src = previous level), up passes, tonemap
    down_bgs: Vec<wgpu::BindGroup>,
    up_bgs: Vec<wgpu::BindGroup>,
    tonemap_bg: wgpu::BindGroup,
}

pub struct Renderer {
    globals_bgl: wgpu::BindGroupLayout,
    post_bgl: wgpu::BindGroupLayout,
    sky_main: wgpu::RenderPipeline,
    flat_main: wgpu::RenderPipeline,
    solid_main: wgpu::RenderPipeline,
    sky_refl: wgpu::RenderPipeline,
    solid_refl: wgpu::RenderPipeline,
    tree_main: wgpu::RenderPipeline,
    tree_refl: wgpu::RenderPipeline,
    shadow_pipe: wgpu::RenderPipeline,
    shadow_tree: wgpu::RenderPipeline,
    bloom_down: wgpu::RenderPipeline,
    bloom_up: wgpu::RenderPipeline,
    tonemap: wgpu::RenderPipeline,
    globals_buf: wgpu::Buffer,
    refl_globals_buf: wgpu::Buffer,
    shadow_buf: wgpu::Buffer,
    shadow_bg: wgpu::BindGroup,
    shadow_layers: Vec<wgpu::TextureView>,
    shadow_array: wgpu::TextureView,
    shadow_samp: wgpu::Sampler,
    lin_samp: wgpu::Sampler,
    post_buf: wgpu::Buffer,
    dummy_refl: wgpu::TextureView,
    dummy_ao: wgpu::TextureView,
    ground: DynMesh,
    tiles: HashMap<TileId, GpuTile>,
    pending: std::collections::VecDeque<PendingTile>,
    runways: Option<GpuMesh>,
    tree_meshes: TreeMeshes,
    dynamic: DynMesh,
    targets: Option<Targets>,
    atmos: Atmosphere,
    clouds: CloudRenderer,
    /// Last frame's camera (eye, forward) to detect cuts.
    last_cam: Option<(Vec3, Vec3)>,
    ao_pipe: wgpu::RenderPipeline,
    ao_blur_pipe: wgpu::RenderPipeline,
    ao_buf: wgpu::Buffer,
    prof: Option<Profiler>,
    /// Last frame's view-projection, once an AO result for it exists.
    /// Last frame's rotation-only view-projection and eye (AO reprojection).
    prev_vp: Option<(Mat4, Vec3)>,
    /// Current floating-origin frame and pattern space (set_frame).
    proj: Proj,
    pattern: [f32; 4],
    /// Shadow-snapping anchor (the start point) in the current frame (m, f64).
    anchor: (f64, f64),
    pub output_format: wgpu::TextureFormat,
    pub stats: RenderStats,
    /// Weather / season inputs (see `set_weather`).
    wxr: crate::weather::WxRender,
    cloud_version: u64,
    wpass: WeatherPasses,
}

fn vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 5] = wgpu::vertex_attr_array![
        0 => Float32x3, 1 => Float32x3, 2 => Float32x3, 3 => Float32x2, 4 => Float32
    ];
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vert>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

fn tree_instance_layout() -> wgpu::VertexBufferLayout<'static> {
    const ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![5 => Float32x4, 6 => Float32x4];
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<TreeInst>() as u64,
        step_mode: wgpu::VertexStepMode::Instance,
        attributes: &ATTRS,
    }
}

fn shadow_vertex_layout() -> wgpu::VertexBufferLayout<'static> {
    // Position + uv (the tree shadow shader reads the conifer flag in uv.x).
    const ATTRS: [wgpu::VertexAttribute; 2] = [
        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 0, shader_location: 0 },
        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 36, shader_location: 3 },
    ];
    wgpu::VertexBufferLayout {
        array_stride: std::mem::size_of::<Vert>() as u64,
        step_mode: wgpu::VertexStepMode::Vertex,
        attributes: &ATTRS,
    }
}

fn tex(device: &wgpu::Device, label: &str, w: u32, h: u32, format: wgpu::TextureFormat, samples: u32, usage: wgpu::TextureUsages) -> wgpu::TextureView {
    device
        .create_texture(&wgpu::TextureDescriptor {
            label: Some(label),
            size: wgpu::Extent3d { width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: samples,
            dimension: wgpu::TextureDimension::D2,
            format,
            usage,
            view_formats: &[],
        })
        .create_view(&Default::default())
}

fn bgl_tex(binding: u32, dim: wgpu::TextureViewDimension) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: true }, view_dimension: dim, multisampled: false },
        count: None,
    }
}

fn bgl_storage(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
        count: None,
    }
}

/// Haze (boundary-layer aerosol multiplier) for a weather.
fn haze_for(cover: f32) -> f32 {
    1.6 + 3.0 * cover
}

impl Renderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, output_format: wgpu::TextureFormat) -> Self {
        let core = include_str!("shaders/atmos_core.wgsl");
        let common = include_str!("shaders/common.wgsl");
        let scene_src = format!("{core}\n{common}\n{}", include_str!("shaders/scene.wgsl"));
        let scene_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("scene"),
            source: wgpu::ShaderSource::Wgsl(scene_src.into()),
        });
        let shadow_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("shadow"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/shadow.wgsl").into()),
        });
        let post_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("post"),
            source: wgpu::ShaderSource::Wgsl(include_str!("shaders/post.wgsl").into()),
        });

        let vf = wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT;
        use wgpu::TextureViewDimension as Dim;
        let globals_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("globals"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: vf,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Depth,
                        view_dimension: wgpu::TextureViewDimension::D2Array,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Comparison),
                    count: None,
                },
                bgl_tex(3, Dim::D2),
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                bgl_tex(5, Dim::D2),
                bgl_tex(6, Dim::D2),
                bgl_tex(7, Dim::D3),
                bgl_tex(8, Dim::D2),
                bgl_storage(9),
                bgl_tex(10, Dim::D2),
            ],
        });
        let shadow_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("shadow"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: wgpu::BufferSize::new(80),
                },
                count: None,
            }],
        });
        let post_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("post"),
            entries: &[
                bgl_tex(0, Dim::D2),
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                bgl_tex(2, Dim::D2),
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: true,
                        min_binding_size: wgpu::BufferSize::new(32),
                    },
                    count: None,
                },
                bgl_storage(4),
            ],
        });

        let scene_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene"),
            bind_group_layouts: &[Some(&globals_bgl)],
            immediate_size: IMMEDIATE_SIZE,
        });
        let shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow"),
            bind_group_layouts: &[Some(&shadow_bgl)],
            immediate_size: IMMEDIATE_SIZE,
        });
        let post_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("post"),
            bind_group_layouts: &[Some(&post_bgl)],
            immediate_size: 0,
        });

        let scene_pipe = |label: &str,
                          vs: &str,
                          fs: &str,
                          with_vb: bool,
                          compare: wgpu::CompareFunction,
                          write: bool,
                          samples: u32| {
            let vbl = [Some(vertex_layout())];
            let tbl = [Some(vertex_layout()), Some(tree_instance_layout())];
            let instanced = vs == "vs_tree";
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&scene_layout),
                vertex: wgpu::VertexState {
                    module: &scene_mod,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers: if instanced { &tbl } else if with_vb { &vbl } else { &[] },
                },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH,
                    depth_write_enabled: Some(write),
                    depth_compare: Some(compare),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState {
                    count: samples,
                    mask: !0,
                    alpha_to_coverage_enabled: samples > 1 && with_vb && write && compare != wgpu::CompareFunction::Always,
                },
                fragment: Some(wgpu::FragmentState {
                    module: &scene_mod,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[
                        Some(wgpu::ColorTargetState { format: HDR, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                        Some(wgpu::ColorTargetState { format: DIST, blend: None, write_mask: wgpu::ColorWrites::ALL }),
                    ],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        use wgpu::CompareFunction as C;
        // Reverse-Z: larger depth = nearer.
        let sky_main = scene_pipe("sky", "vs_sky", "fs_sky", false, C::Always, false, MSAA);
        let flat_main = scene_pipe("flat", "vs_main", "fs_main", true, C::Always, true, MSAA);
        let solid_main = scene_pipe("solid", "vs_main", "fs_main", true, C::GreaterEqual, true, MSAA);
        let sky_refl = scene_pipe("sky refl", "vs_sky", "fs_sky", false, C::Always, false, 1);
        let solid_refl = scene_pipe("solid refl", "vs_main", "fs_main", true, C::GreaterEqual, true, 1);
        let tree_main = scene_pipe("tree", "vs_tree", "fs_main", true, C::GreaterEqual, true, MSAA);
        let tree_refl = scene_pipe("tree refl", "vs_tree", "fs_main", true, C::GreaterEqual, true, 1);

        let shadow_variant = |label: &str, vs: &str, buffers: &[Option<wgpu::VertexBufferLayout>]| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&shadow_layout),
                vertex: wgpu::VertexState {
                    module: &shadow_mod,
                    entry_point: Some(vs),
                    compilation_options: Default::default(),
                    buffers,
                },
                primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH,
                    depth_write_enabled: Some(true),
                    depth_compare: Some(C::LessEqual),
                    stencil: Default::default(),
                    bias: wgpu::DepthBiasState { constant: 1, slope_scale: 1.5, clamp: 0.0 },
                }),
                multisample: Default::default(),
                fragment: None,
                multiview_mask: None,
                cache: None,
            })
        };
        let shadow_tree = shadow_variant("shadow tree", "vs_shadow_tree", &[Some(shadow_vertex_layout()), Some(tree_instance_layout())]);
        let shadow_pipe = shadow_variant("shadow", "vs_shadow", &[Some(shadow_vertex_layout())]);

        let post_pipe = |label: &str, fs: &str, format: wgpu::TextureFormat, blend: Option<wgpu::BlendState>| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&post_layout),
                vertex: wgpu::VertexState {
                    module: &post_mod,
                    entry_point: Some("vs_full"),
                    compilation_options: Default::default(),
                    buffers: &[],
                },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &post_mod,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format, blend, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let additive = wgpu::BlendState {
            color: wgpu::BlendComponent {
                src_factor: wgpu::BlendFactor::One,
                dst_factor: wgpu::BlendFactor::One,
                operation: wgpu::BlendOperation::Add,
            },
            alpha: wgpu::BlendComponent::OVER,
        };
        let bloom_down = post_pipe("bloom down", "fs_down", HDR, None);
        let bloom_up = post_pipe("bloom up", "fs_up", HDR, Some(additive));
        let tonemap = post_pipe("tonemap", "fs_tonemap", output_format, None);

        let ubuf = |label: &str, size: u64| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let globals_buf = ubuf("globals", std::mem::size_of::<Globals>() as u64);
        let refl_globals_buf = ubuf("refl globals", std::mem::size_of::<Globals>() as u64);
        let shadow_buf = ubuf("shadow u", UNIFORM_STRIDE * CASCADES as u64);
        let shadow_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("shadow"),
            layout: &shadow_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &shadow_buf,
                    offset: 0,
                    size: wgpu::BufferSize::new(80),
                }),
            }],
        });
        let post_buf = ubuf("post u", UNIFORM_STRIDE * (2 * BLOOM_LEVELS as u64 + 2));

        let shadow_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("shadow map"),
            size: wgpu::Extent3d { width: SHADOW_RES, height: SHADOW_RES, depth_or_array_layers: CASCADES as u32 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: DEPTH,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let shadow_array = shadow_tex.create_view(&wgpu::TextureViewDescriptor {
            dimension: Some(wgpu::TextureViewDimension::D2Array),
            ..Default::default()
        });
        let shadow_layers = (0..CASCADES as u32)
            .map(|i| {
                shadow_tex.create_view(&wgpu::TextureViewDescriptor {
                    dimension: Some(wgpu::TextureViewDimension::D2),
                    base_array_layer: i,
                    array_layer_count: Some(1),
                    ..Default::default()
                })
            })
            .collect();
        let shadow_samp = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("shadow"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            compare: Some(C::LessEqual),
            ..Default::default()
        });
        let lin_samp = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("linear"),
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            ..Default::default()
        });
        let dummy_refl = tex(device, "dummy", 1, 1, HDR, 1, wgpu::TextureUsages::TEXTURE_BINDING);
        // 1x1 white "AO" until SSAO is enabled.
        let dummy_ao = {
            let t = device.create_texture(&wgpu::TextureDescriptor {
                label: Some("ao white"),
                size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: wgpu::TextureFormat::Rgba8Unorm,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            });
            queue.write_texture(
                wgpu::TexelCopyTextureInfo { texture: &t, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
                &[255u8; 4],
                wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(4), rows_per_image: Some(1) },
                wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
            );
            t.create_view(&Default::default())
        };
        let tm = |c: bool, l: u32| upload(device, "tree", &scene::tree_mesh(c, l)).expect("tree mesh");
        let tree_meshes = TreeMeshes { m: [[tm(false, 0), tm(false, 1)], [tm(true, 0), tm(true, 1)]] };
        let atmos = Atmosphere::new(device);
        let ao_mod = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("ao"), source: wgpu::ShaderSource::Wgsl(include_str!("shaders/ao.wgsl").into()) });
        let ao_pipeline = |fs: &str, format: wgpu::TextureFormat| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(fs),
                layout: None,
                vertex: wgpu::VertexState { module: &ao_mod, entry_point: Some("vs_full"), compilation_options: Default::default(), buffers: &[] },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &ao_mod,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format, blend: None, write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let ao_pipe = ao_pipeline("fs_ao", wgpu::TextureFormat::Rg16Float);
        let ao_blur_pipe = ao_pipeline("fs_blur", wgpu::TextureFormat::R8Unorm);
        let ao_buf = ubuf("ao u", std::mem::size_of::<AoU>() as u64);
        let mut clouds = CloudRenderer::new(device, queue, atmos.cloud_views());
        clouds.set_field(&CloudField::new(7, crate::clouds::Weather::Scattered));

        Renderer {
            globals_bgl,
            post_bgl,
            sky_main,
            flat_main,
            solid_main,
            sky_refl,
            solid_refl,
            tree_main,
            tree_refl,
            shadow_pipe,
            shadow_tree,
            bloom_down,
            bloom_up,
            tonemap,
            globals_buf,
            refl_globals_buf,
            shadow_buf,
            shadow_bg,
            shadow_layers,
            shadow_array,
            shadow_samp,
            lin_samp,
            post_buf,
            dummy_refl,
            dummy_ao,
            ground: DynMesh::new(device, 4, 6),
            tiles: HashMap::new(),
            pending: Default::default(),
            runways: None,
            tree_meshes,
            dynamic: DynMesh::new(device, 1 << 16, 1 << 17),
            targets: None,
            atmos,
            clouds,
            last_cam: None,
            ao_pipe,
            ao_blur_pipe,
            ao_buf,
            prof: Profiler::new(device, queue),
            prev_vp: None,
            proj: Proj::new(0.0, 0.0),
            pattern: [1.0, 0.0, 0.0, 0.0],
            anchor: (0.0, 0.0),
            output_format,
            stats: RenderStats::default(),
            wxr: crate::weather::WxRender { season: crate::weather::Season::at(crate::sim::DEFAULT_DAY, 51.5), ..Default::default() },
            cloud_version: 0,
            wpass: WeatherPasses::new(device),
        }
    }

    /// Queue a streamed tile for upload: its (empty) GPU buffers are created
    /// now, the data is copied in by [`Renderer::pump_uploads`] a few MB per
    /// frame, and the tile becomes visible once complete.
    pub fn queue_tile(&mut self, device: &wgpu::Device, t: TileMeshes) {
        let buf = |label: &str, size: usize, usage: wgpu::BufferUsages| {
            device.create_buffer(&wgpu::BufferDescriptor {
                label: Some(label),
                size: (size as u64).max(4),
                usage: usage | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            })
        };
        let vsz = std::mem::size_of::<Vert>();
        let mesh = |label: &str, m: &MeshData| {
            (!m.idx.is_empty()).then(|| GpuMesh {
                vb: buf(label, m.verts.len() * vsz, wgpu::BufferUsages::VERTEX),
                ib: buf(label, m.idx.len() * 4, wgpu::BufferUsages::INDEX),
                n: m.idx.len() as u32,
            })
        };
        let flat = mesh("tile flat", &t.flat);
        let buildings = mesh("tile buildings", &t.buildings);
        let trees = (!t.trees.is_empty()).then(|| {
            let b = buf("tile trees", t.trees.len() * std::mem::size_of::<TreeInst>(), wgpu::BufferUsages::VERTEX);
            (b, (t.trees.len() - t.conifers) as u32, t.trees.len() as u32)
        });
        let bytes = flat.as_ref().map_or(0, |m| m.bytes()) + buildings.as_ref().map_or(0, |m| m.bytes()) + trees.as_ref().map_or(0, |t| t.0.size());
        let (lo, hi) = t.id.rect(&self.proj);
        let map = Proj::tile_frame(t.id).map_to(&self.proj);
        let gt = GpuTile { id: t.id, lo, hi, map, xf: map.gpu(), max_h: t.max_h, flat, buildings, trees, bytes };
        self.pending.retain(|p| p.data.id != t.id);
        self.pending.push_back(PendingTile { tile: gt, data: t, part: 0, offset: 0 });
    }

    /// Copy queued tile data to the GPU, at most `budget` bytes (None = all).
    /// Returns the bytes written.
    pub fn pump_uploads(&mut self, queue: &wgpu::Queue, budget: Option<u64>) -> u64 {
        const CHUNK: usize = 1 << 20;
        let mut spent = 0u64;
        while let Some(p) = self.pending.front_mut() {
            // Parts: flat verts, flat idx, building verts, building idx, trees.
            let (buf, bytes): (Option<&wgpu::Buffer>, &[u8]) = match p.part {
                0 => (p.tile.flat.as_ref().map(|m| &m.vb), bytemuck::cast_slice(&p.data.flat.verts)),
                1 => (p.tile.flat.as_ref().map(|m| &m.ib), bytemuck::cast_slice(&p.data.flat.idx)),
                2 => (p.tile.buildings.as_ref().map(|m| &m.vb), bytemuck::cast_slice(&p.data.buildings.verts)),
                3 => (p.tile.buildings.as_ref().map(|m| &m.ib), bytemuck::cast_slice(&p.data.buildings.idx)),
                4 => (p.tile.trees.as_ref().map(|t| &t.0), bytemuck::cast_slice(&p.data.trees)),
                _ => {
                    let p = self.pending.pop_front().unwrap();
                    self.tiles.insert(p.data.id, p.tile);
                    continue;
                }
            };
            let Some(buf) = buf else {
                p.part += 1;
                p.offset = 0;
                continue;
            };
            if p.offset >= bytes.len() {
                p.part += 1;
                p.offset = 0;
                continue;
            }
            if let Some(lim) = budget {
                if spent >= lim {
                    break;
                }
            }
            // Chunks stay 4-byte aligned (all element sizes are multiples of 4).
            let n = (bytes.len() - p.offset).min(CHUNK);
            queue.write_buffer(buf, p.offset as u64, &bytes[p.offset..p.offset + n]);
            p.offset += n;
            spent += n as u64;
        }
        self.stats.gpu_bytes = self.gpu_bytes();
        spent
    }

    /// Tiles still waiting for (part of) their upload.
    pub fn pending_tiles(&self) -> impl Iterator<Item = TileId> + '_ {
        self.pending.iter().map(|p| p.data.id)
    }

    pub fn remove_tile(&mut self, id: TileId) {
        self.tiles.remove(&id);
        self.pending.retain(|p| p.data.id != id);
        self.stats.gpu_bytes = self.gpu_bytes();
    }

    /// Replace the global (merged) runway mesh.
    pub fn set_runways(&mut self, device: &wgpu::Device, m: &MeshData) {
        self.runways = upload(device, "runways", m);
    }

    /// Use this cloud field (weather) for the volumetric clouds.
    pub fn set_cloud_field(&mut self, field: &CloudField) {
        self.clouds.set_field(field);
    }

    /// Weather, season and cloud drift for the coming frames (call every
    /// frame; the cloud field is re-uploaded only when its version changes).
    pub fn set_weather(&mut self, r: &crate::weather::WxRender, field: &CloudField) {
        if r.cloud_version != self.cloud_version {
            self.cloud_version = r.cloud_version;
            self.clouds.set_field(field);
        }
        self.clouds.set_drift(r.drift);
        self.clouds.set_cirrus_drift(r.cirrus_drift);
        self.wxr = *r;
    }

    /// The current floating-origin frame and its pattern space
    /// (`World::pattern_frame`). Call before rendering / queueing tiles.
    pub fn set_frame(&mut self, proj: Proj, pattern: [f32; 4], home: Proj) {
        self.pattern = pattern;
        // Shadow texel snapping is anchored at the start point: ground-fixed, so
        // it neither crawls with the camera nor jumps when the origin moves.
        let h = home.map_to(&proj);
        self.anchor = (h.tx, h.tz);
        if proj == self.proj {
            return;
        }
        self.proj = proj;
        let place = |t: &mut GpuTile| {
            (t.lo, t.hi) = t.id.rect(&proj);
            t.map = Proj::tile_frame(t.id).map_to(&proj);
        };
        self.tiles.values_mut().for_each(place);
        self.pending.iter_mut().for_each(|p| place(&mut p.tile));
    }

    /// The origin moved by `delta` (world points p → p + delta): keep the
    /// temporal history (cloud reprojection, last camera) valid.
    pub fn rebase(&mut self, delta: Vec3) {
        if let Some((_, e)) = &mut self.prev_vp {
            *e += delta;
        }
        if let Some((e, _)) = &mut self.last_cam {
            *e += delta;
        }
        self.clouds.rebase(delta);
    }

    /// Forget temporal history (camera cut / teleport).
    pub fn reset_history(&mut self) {
        self.clouds.reset_history();
    }

    /// With `SKYWARD_GPU_TIMING=1`: read back the last frame's pass timings
    /// (call after the frame was submitted; blocks on the GPU).
    pub fn collect_timings(&mut self, device: &wgpu::Device) {
        let Some(p) = &mut self.prof else { return };
        let slice = p.read.slice(..);
        slice.map_async(wgpu::MapMode::Read, |_| {});
        let _ = device.poll(wgpu::PollType::wait_indefinitely());
        let t: Vec<u64> = bytemuck::cast_slice::<u8, u64>(&slice.get_mapped_range().expect("prof map")).to_vec();
        p.read.unmap();
        for i in 1..t.len() {
            p.sums[i] += t[i].saturating_sub(t[i - 1]) as f64 * p.period as f64 * 1e-6;
        }
        p.sums[0] += t[t.len() - 1].saturating_sub(t[0]) as f64 * p.period as f64 * 1e-6;
        p.frames += 1;
        if p.frames == 120 {
            let f = p.frames as f64;
            let parts: Vec<String> = (1..t.len()).map(|i| format!("{} {:.2}", PROF_MARKS[i], p.sums[i] / f)).collect();
            eprintln!("gpu ms: total {:.2} | {}", p.sums[0] / f, parts.join(", "));
            p.sums.iter_mut().for_each(|s| *s = 0.0);
            p.frames = 0;
        }
    }

    pub fn gpu_bytes(&self) -> u64 {
        self.tiles.values().map(|t| t.bytes).sum::<u64>()
            + self.pending.iter().map(|p| p.tile.bytes).sum::<u64>()
            + self.runways.as_ref().map_or(0, |m| m.bytes())
    }

    fn ensure_targets(&mut self, device: &wgpu::Device, w: u32, h: u32) {
        if let Some(t) = &self.targets {
            if t.width == w && t.height == h {
                return;
            }
        }
        let ra = wgpu::TextureUsages::RENDER_ATTACHMENT;
        let rt = ra | wgpu::TextureUsages::TEXTURE_BINDING;
        let hdr_msaa = tex(device, "hdr msaa", w, h, HDR, MSAA, ra);
        let hdr = tex(device, "hdr", w, h, HDR, 1, rt);
        let dist_msaa = tex(device, "dist msaa", w, h, DIST, MSAA, ra);
        let dist = tex(device, "dist", w, h, DIST, 1, rt);
        let depth = tex(device, "depth", w, h, DEPTH, MSAA, ra);
        let (rw, rh) = ((w / 2).max(1), (h / 2).max(1));
        let refl = tex(device, "refl", rw, rh, HDR, 1, rt);
        let refl_dist = tex(device, "refl dist", rw, rh, DIST, 1, rt);
        let refl_depth = tex(device, "refl depth", rw, rh, DEPTH, 1, ra);
        let (aw, ah) = ((w / 2).max(1), (h / 2).max(1));
        let ao_raw = tex(device, "ao raw", aw, ah, wgpu::TextureFormat::Rg16Float, 1, rt);
        let ao = tex(device, "ao", aw, ah, wgpu::TextureFormat::R8Unorm, 1, rt);
        let ao_bg_for = |pipe: &wgpu::RenderPipeline, src: Option<&wgpu::TextureView>| {
            let e = match src {
                Some(v) => vec![wgpu::BindGroupEntry { binding: 2, resource: wgpu::BindingResource::TextureView(v) }],
                None => vec![
                    wgpu::BindGroupEntry { binding: 0, resource: self.ao_buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::TextureView(&dist) },
                ],
            };
            device.create_bind_group(&wgpu::BindGroupDescriptor { label: Some("ao"), layout: &pipe.get_bind_group_layout(0), entries: &e })
        };
        let ao_bg = ao_bg_for(&self.ao_pipe, None);
        let ao_blur_bg = ao_bg_for(&self.ao_blur_pipe, Some(&ao_raw));
        let mut bloom = Vec::new();
        let (mut bw, mut bh) = (w, h);
        for i in 0..BLOOM_LEVELS {
            bw = (bw / 2).max(1);
            bh = (bh / 2).max(1);
            bloom.push((tex(device, &format!("bloom{i}"), bw, bh, HDR, 1, rt), bw, bh));
        }
        use wgpu::BindingResource as R;
        let globals_bg = |buf: &wgpu::Buffer, refl_view: &wgpu::TextureView, ap: &wgpu::TextureView, ao_view: &wgpu::TextureView| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("globals"),
                layout: &self.globals_bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: buf.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(&self.shadow_array) },
                    wgpu::BindGroupEntry { binding: 2, resource: R::Sampler(&self.shadow_samp) },
                    wgpu::BindGroupEntry { binding: 3, resource: R::TextureView(refl_view) },
                    wgpu::BindGroupEntry { binding: 4, resource: R::Sampler(&self.lin_samp) },
                    wgpu::BindGroupEntry { binding: 5, resource: R::TextureView(&self.atmos.trans) },
                    wgpu::BindGroupEntry { binding: 6, resource: R::TextureView(&self.atmos.sky) },
                    wgpu::BindGroupEntry { binding: 7, resource: R::TextureView(ap) },
                    wgpu::BindGroupEntry { binding: 8, resource: R::TextureView(self.clouds.shadow_view()) },
                    wgpu::BindGroupEntry { binding: 9, resource: self.atmos.out.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 10, resource: R::TextureView(ao_view) },
                ],
            })
        };
        let main_bg = globals_bg(&self.globals_buf, &refl, &self.atmos.ap_main, &ao);
        let refl_bg = globals_bg(&self.refl_globals_buf, &self.dummy_refl, &self.atmos.ap_refl, &self.dummy_ao);
        let post_bg = |src: &wgpu::TextureView, bloom_src: &wgpu::TextureView| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("post"),
                layout: &self.post_bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: R::TextureView(src) },
                    wgpu::BindGroupEntry { binding: 1, resource: R::Sampler(&self.lin_samp) },
                    wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(bloom_src) },
                    wgpu::BindGroupEntry {
                        binding: 3,
                        resource: R::Buffer(wgpu::BufferBinding { buffer: &self.post_buf, offset: 0, size: wgpu::BufferSize::new(32) }),
                    },
                    wgpu::BindGroupEntry { binding: 4, resource: self.atmos.out.as_entire_binding() },
                ],
            })
        };
        let mut down_bgs = Vec::new();
        for i in 0..BLOOM_LEVELS {
            let src = if i == 0 { &hdr } else { &bloom[i - 1].0 };
            down_bgs.push(post_bg(src, &self.dummy_refl));
        }
        let mut up_bgs = Vec::new();
        for i in (1..BLOOM_LEVELS).rev() {
            up_bgs.push(post_bg(&bloom[i].0, &self.dummy_refl));
        }
        let tonemap_bg = post_bg(&hdr, &bloom[0].0);
        self.clouds.reset_history();
        self.prev_vp = None;
        self.targets = Some(Targets {
            width: w,
            height: h,
            hdr_msaa,
            hdr,
            dist_msaa,
            dist,
            depth,
            refl,
            refl_dist,
            refl_depth,
            refl_size: (rw, rh),
            ao_raw,
            ao,
            ao_size: (aw, ah),
            ao_bg,
            ao_blur_bg,
            bloom,
            main_bg,
            refl_bg,
            down_bgs,
            up_bgs,
            tonemap_bg,
        });
    }

    /// Render one frame into `out` (format must equal `output_format`).
    #[allow(clippy::too_many_arguments)]
    pub fn render(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        encoder: &mut wgpu::CommandEncoder,
        out: &wgpu::TextureView,
        width: u32,
        height: u32,
        fv: &FrameView,
        // Dynamic geometry (the aircraft), already relative to `fv.eye`.
        dynamic: &MeshData,
    ) {
        self.ensure_targets(device, width, height);
        self.dynamic.set(device, queue, dynamic);
        // Base ground follows the camera (snapped so the procedural pattern is stable).
        {
            let c = (Vec2::new(fv.eye.x, fv.eye.z) / 1000.0).round() * 1000.0;
            let g = GROUND_HALF;
            let mut m = MeshData { mat: scene::mat::GROUND, ..Default::default() };
            m.flat_quad([c + Vec2::new(-g, -g), c + Vec2::new(g, -g), c + Vec2::new(g, g), c + Vec2::new(-g, g)], 0.0, scene::LAND);
            self.ground.set(device, queue, &m);
        }
        // Camera cuts drop the clouds' temporal history.
        let fwd = -fv.view.inverse().z_axis.truncate();
        if let Some((e, f)) = self.last_cam {
            if e.distance(fv.eye) > 300.0 || f.dot(fwd) < 0.8 {
                self.clouds.reset_history();
                self.prev_vp = None;
            }
        }
        self.last_cam = Some((fv.eye, fwd));

        // Each tile's placement relative to the eye (f64: small, exact numbers on the GPU).
        let (ex, ey, ez) = (fv.eye.x as f64, fv.eye.y as f64, fv.eye.z as f64);
        for t in self.tiles.values_mut() {
            let m = t.map;
            t.xf = [(m.sx - 1.0) as f32, (m.tx - ex) as f32, (m.tz - ez) as f32, -ey as f32];
        }
        let aspect = width as f32 / height.max(1) as f32;
        let proj = glam::camera::rh::proj::directx::perspective_infinite_reverse(fv.fov_y, aspect, NEAR);
        let vp = proj * fv.view;
        // Eye-relative (rotation-only) view-projection: screen rays are
        // reconstructed with it. Using the full matrix and subtracting the eye
        // cancels large world coordinates in f32 and makes the sky / clouds
        // judder against the (vertex-transformed, stable) ground far from the
        // world origin.
        let view_rot = Mat4::from_cols(fv.view.x_axis, fv.view.y_axis, fv.view.z_axis, Vec4::W);
        let vp_rot = proj * view_rot;
        let sun = fv.sun_dir.normalize();
        let field = self.clouds.field();
        let cover = field.sky_cover();
        let wx = self.wxr;
        let haze = if wx.haze > 0.0 { wx.haze } else { haze_for(cover) };
        let fog = (wx.fog_ext, wx.fog_top * 0.001);
        // Thick decks (nimbostratus) let much less light through.
        let deck_trans = 1.0 - 0.75 * ((field.deck_density() - 0.3) / 0.7).clamp(0.0, 1.0);
        let cloud_base = field.base();

        // Cascades: each one also covers the blend band before its near split.
        let mut shadow_mats = [[[0f32; 4]; 4]; CASCADES];
        let mut texel = [0f32; 4];
        for i in 0..CASCADES {
            let near = if i == 0 { SPLITS[0] } else { SPLITS[i] - (SPLITS[i] - SPLITS[i - 1]) * CASCADE_BLEND };
            let anchor = glam::DVec3::new(self.anchor.0 - ex, -ey, self.anchor.1 - ez);
            let (m, radius) = cascade_matrix(view_rot, fv.fov_y, aspect, near, SPLITS[i + 1], sun, anchor);
            shadow_mats[i] = m.to_cols_array_2d();
            texel[i] = radius * 2.0 / SHADOW_RES as f32;
            queue.write_buffer(&self.shadow_buf, UNIFORM_STRIDE * i as u64, bytemuck::cast_slice(&m.to_cols_array()));
            let leaf = self.wxr.season.leaf;
            queue.write_buffer(&self.shadow_buf, UNIFORM_STRIDE * i as u64 + 64, bytemuck::cast_slice(&[leaf, 0.0f32, 0.0, 0.0]));
        }

        let t = self.targets.as_ref().unwrap();
        // Pattern space of the eye (f64, wrapped to the pattern period).
        let pat = {
            let wrapp = |v: f64| (v - crate::world::NOISE_PERIOD * (v / crate::world::NOISE_PERIOD).round()) as f32;
            let [sx, ox, oz, _] = self.pattern;
            [sx, wrapp(fv.eye.x as f64 * sx as f64 + ox as f64), wrapp(fv.eye.z as f64 + oz as f64), 0.0]
        };
        // Last frame's camera, relative to this frame's eye.
        let prev_rel = self.prev_vp.map(|(m, e)| m * Mat4::from_translation(fv.eye - e)).unwrap_or(vp_rot);
        let globals = Globals {
            view_proj: vp_rot.to_cols_array_2d(),
            inv_view_proj: vp_rot.inverse().to_cols_array_2d(),
            shadow: shadow_mats,
            cam_pos: [fv.eye.x, fv.eye.y, fv.eye.z, fv.time],
            sun_dir: [sun.x, sun.y, sun.z, cover],
            sun_e: [atmosphere::SUN_E, atmosphere::SUN_E, atmosphere::SUN_E, haze],
            splits: [SPLITS[1], SPLITS[2], SPLITS[3], SPLITS[4]],
            shadow_info: [SHADOW_RES as f32, 0.0, CASCADE_BLEND, if self.prev_vp.is_some() { 1.0 } else { 0.0 }],
            screen: [width as f32, height as f32, 1.0 / width as f32, 1.0 / height as f32],
            cloud: self.clouds.shadow_params(),
            texel,
            prev_view_proj: prev_rel.to_cols_array_2d(),
            wx: [wx.wet, wx.rain, wx.snow, wx.snow_cover],
            wx2: [wx.wind.x, wx.wind.y, wx.frost, wx.gust],
            season: [wx.season.leaf, wx.season.autumn, wx.season.spring, wx.season.dormant],
            season2: [wx.season.dry, wx.season.year, wx.season.strength, if wx.drizzle { 1.0 } else { 0.0 }],
            pattern: pat,
        };
        let px_per_m = height as f32 * 0.5 / (fv.fov_y * 0.5).tan();
        let aou = AoU {
            inv_view_proj: vp_rot.inverse().to_cols_array_2d(),
            eye: [fv.eye.x, fv.eye.y, fv.eye.z, 0.0],
            params: [AO_RADIUS, px_per_m, AO_STRENGTH, 0.0],
            size: [width as f32, height as f32, (width / 2) as f32, (height / 2) as f32],
        };
        queue.write_buffer(&self.ao_buf, 0, bytemuck::bytes_of(&aou));
        queue.write_buffer(&self.globals_buf, 0, bytemuck::bytes_of(&globals));
        let mirror = Mat4::from_scale(vec3(1.0, -1.0, 1.0));
        let rvp = proj * fv.view * mirror;
        let rvp_rot = vp_rot * mirror;
        let reye = fv.eye * vec3(1.0, -1.0, 1.0);
        let (rw, rh) = t.refl_size;
        // Mirrored pass, eye-relative: world p = rel + eye, mirrored and seen from the eye.
        let rvp_rel = proj * view_rot * Mat4::from_translation(vec3(0.0, -2.0 * fv.eye.y, 0.0)) * mirror;
        let rglobals = Globals {
            view_proj: rvp_rel.to_cols_array_2d(),
            pattern: [pat[0], pat[1], pat[2], 2.0 * fv.eye.y],
            inv_view_proj: rvp_rot.inverse().to_cols_array_2d(),
            cam_pos: [reye.x, reye.y, reye.z, fv.time],
            shadow_info: [SHADOW_RES as f32, 1.0, CASCADE_BLEND, 0.0],
            screen: [rw as f32, rh as f32, 1.0 / rw as f32, 1.0 / rh as f32],
            ..globals
        };
        queue.write_buffer(&self.refl_globals_buf, 0, bytemuck::bytes_of(&rglobals));

        // Post uniforms: [0..L) down, [L..2L) up, 2L tonemap.
        for i in 0..BLOOM_LEVELS {
            let (sw, sh) = if i == 0 { (width, height) } else { (t.bloom[i - 1].1, t.bloom[i - 1].2) };
            let u = PostU { params: [0.0, 0.0, if i == 0 { 1.0 } else { 0.0 }, 0.0], texel: [1.0 / sw as f32, 1.0 / sh as f32, 0.0, 0.0] };
            queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * i as u64, bytemuck::bytes_of(&u));
        }
        for (k, i) in (1..BLOOM_LEVELS).rev().enumerate() {
            let (sw, sh) = (t.bloom[i].1, t.bloom[i].2);
            let u = PostU { params: [0.0; 4], texel: [1.0 / sw as f32, 1.0 / sh as f32, 0.0, 0.0] };
            queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * (BLOOM_LEVELS + k) as u64, bytemuck::bytes_of(&u));
        }
        // Tonemap: manual exposure, bloom strength, AgX look power, saturation.
        let tone = PostU { params: [fv.exposure, 0.04, 1.22, 1.18], texel: [0.0; 4] };
        queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * (2 * BLOOM_LEVELS) as u64, bytemuck::bytes_of(&tone));

        if let Some(p) = &self.prof {
            p.mark(encoder, 0);
        }
        // Atmosphere LUTs / froxels / ambient for this frame.
        self.atmos.update(
            queue,
            encoder,
            &AtmosFrame {
                sun,
                haze,
                cover,
                eye: fv.eye,
                inv_view_proj: vp_rot.inverse(),
                refl_eye: reye,
                refl_inv_view_proj: rvp_rot.inverse(),
                cloud_base,
                fog_ext: fog.0,
                fog_top: fog.1 * 1000.0,
                deck_trans,
            },
        );
        // Clouds: lit by the sun as it arrives at the middle of the layer.
        let sun_cloud = atmosphere::transmittance(cloud_base + 600.0, sun.y, haze, fog) * atmosphere::SUN_E;
        let cloud_frame = |view_proj: Mat4, rot_view_proj: Mat4, eye: Vec3, w: u32, h: u32| CloudFrame {
            view_proj,
            inv_view_proj: view_proj.inverse(),
            rot_view_proj,
            eye,
            sun_dir: sun,
            sun_col: sun_cloud / std::f32::consts::PI,
            sky_intensity: 1.0,
            time: fv.time,
            width: w,
            height: h,
            fog: [atmosphere::SUN_E, haze],
            ground_fog: [fog.0, fog.1],
        };
        let main_cf = cloud_frame(vp, vp_rot, fv.eye, width, height);
        let refl_cf = cloud_frame(rvp, rvp_rot, reye, rw, rh);
        self.clouds.prepare(device, queue, encoder, &main_cf);
        let prof = self.prof.as_ref();
        if let Some(p) = prof {
            p.mark(encoder, 1);
        }

        let eye2 = Vec2::new(fv.eye.x, fv.eye.z);
        let main_f = Frustum::new(vp);
        let refl_f = Frustum::new(rvp);
        // (Cascade matrices are eye-relative; culling boxes are world-space.)
        let shadow_f: Vec<Frustum> = shadow_mats.iter().map(|m| Frustum::new(Mat4::from_cols_array_2d(m) * Mat4::from_translation(-fv.eye))).collect();
        // Far-field (low zoom) tiles first: the ground pass is painter's-ordered.
        let mut tiles: Vec<(&TileId, &GpuTile)> = self.tiles.iter().collect();
        tiles.sort_by_key(|(id, _)| id.z);
        let tiles: Vec<&GpuTile> = tiles.into_iter().map(|(_, t)| t).collect();
        let dynamic = (self.dynamic.n > 0).then_some((&self.dynamic.vb, &self.dynamic.ib, self.dynamic.n));
        let tm = &self.tree_meshes;
        let mut stats = RenderStats { tiles: self.tiles.keys().filter(|id| !id.is_far()).count(), gpu_bytes: self.stats.gpu_bytes, ..Default::default() };

        // Draw helpers. Every draw sets its transform (immediate data): the
        // tile's frame → current frame, or zero (identity) for everything else.
        // World-space meshes (ground quad, runways) → eye-relative; the aircraft already is.
        let world_xf: [f32; 4] = [0.0, -fv.eye.x, -fv.eye.z, -fv.eye.y];
        const DYN: [f32; 4] = [0.0; 4];
        fn place(pass: &mut wgpu::RenderPass<'_>, xf: &[f32; 4]) {
            pass.set_immediates(0, bytemuck::cast_slice(xf));
        }
        fn draw_mesh(pass: &mut wgpu::RenderPass<'_>, m: &GpuMesh) {
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, 0..1);
        }
        // Instanced trees for one tile: (mesh lod per kind).
        fn draw_trees(pass: &mut wgpu::RenderPass<'_>, t: &GpuTile, tm: &TreeMeshes, lod: usize) -> usize {
            let Some((buf, split, n)) = &t.trees else { return 0 };
            place(pass, &t.xf);
            pass.set_vertex_buffer(1, buf.slice(..));
            for (kind, range) in [(0usize, 0..*split), (1, *split..*n)] {
                if range.is_empty() {
                    continue;
                }
                let m = &tm.m[kind][lod];
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, range);
            }
            *n as usize
        }

        // 1. Shadow cascades.
        for i in 0..CASCADES {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("shadow"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.shadow_layers[i],
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(1.0), store: wgpu::StoreOp::Store }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            let f = &shadow_f[i];
            pass.set_bind_group(0, &self.shadow_bg, &[(UNIFORM_STRIDE * i as u64) as u32]);
            pass.set_pipeline(&self.shadow_pipe);
            for t in &tiles {
                if let Some(m) = &t.buildings {
                    let (lo, hi) = t.solid_box();
                    if f.aabb(lo, hi) {
                        place(&mut pass, &t.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            pass.set_pipeline(&self.shadow_tree);
            for t in &tiles {
                let (lo, hi) = t.tree_box();
                let d = t.dist(eye2);
                if d < SPLITS[i + 1] * 1.1 && d < TREE_DIST && f.aabb(lo, hi) {
                    draw_trees(&mut pass, t, tm, if i == 0 { 0 } else { 1 });
                }
            }
        }

        if let Some(p) = prof {
            p.mark(encoder, 2);
        }
        // 2. Planar reflection (mirror about y = 0) at half resolution.
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("reflection"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: &t.refl,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &t.refl_dist,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &t.refl_depth,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(0.0), store: wgpu::StoreOp::Discard }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &t.refl_bg, &[]);
            pass.set_pipeline(&self.sky_refl);
            pass.draw(0..3, 0..1);
            pass.set_pipeline(&self.solid_refl);
            for tile in &tiles {
                if let Some(m) = &tile.buildings {
                    let (lo, hi) = tile.solid_box();
                    if tile.dist(eye2) < BUILDING_REFL_DIST && refl_f.aabb(lo, hi) {
                        place(&mut pass, &tile.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            pass.set_pipeline(&self.tree_refl);
            for tile in &tiles {
                let (lo, hi) = tile.tree_box();
                if tile.dist(eye2) < TREE_REFL_DIST && refl_f.aabb(lo, hi) {
                    draw_trees(&mut pass, tile, tm, 1);
                }
            }
        }
        self.clouds.render(device, queue, encoder, &refl_cf, &t.refl, &t.refl_dist, 1);
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 3);
        }

        // 3. Main HDR pass.
        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: &t.hdr_msaa,
                        depth_slice: None,
                        resolve_target: Some(&t.hdr),
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Discard },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &t.dist_msaa,
                        depth_slice: None,
                        resolve_target: Some(&t.dist),
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Discard },
                    }),
                ],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &t.depth,
                    depth_ops: Some(wgpu::Operations { load: wgpu::LoadOp::Clear(0.0), store: wgpu::StoreOp::Discard }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_bind_group(0, &t.main_bg, &[]);
            pass.set_pipeline(&self.sky_main);
            pass.draw(0..3, 0..1);
            // Ground layers: painter's order (base ground, tiles, runways), no depth test.
            pass.set_pipeline(&self.flat_main);
            place(&mut pass, &world_xf);
            pass.set_vertex_buffer(0, self.ground.vb.slice(..));
            pass.set_index_buffer(self.ground.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.ground.n, 0, 0..1);
            for tile in &tiles {
                if let Some(m) = &tile.flat {
                    let (lo, hi) = tile.flat_box();
                    if main_f.aabb(lo, hi) {
                        place(&mut pass, &tile.xf);
                        draw_mesh(&mut pass, m);
                        stats.tiles_drawn += 1;
                    }
                }
            }
            if let Some(m) = &self.runways {
                place(&mut pass, &world_xf);
                draw_mesh(&mut pass, m);
            }
            pass.set_pipeline(&self.solid_main);
            for tile in &tiles {
                if let Some(m) = &tile.buildings {
                    let (lo, hi) = tile.solid_box();
                    if main_f.aabb(lo, hi) {
                        place(&mut pass, &tile.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            pass.set_pipeline(&self.tree_main);
            for tile in &tiles {
                let d = tile.dist(eye2);
                let (lo, hi) = tile.tree_box();
                if d < TREE_DIST && main_f.aabb(lo, hi) {
                    stats.tree_instances += draw_trees(&mut pass, tile, tm, if d < TREE_LOD0 { 0 } else { 1 });
                }
            }
        }
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 4);
        }
        // SSAO from this frame's distances, used (reprojected) next frame.
        for (view, pipe, bg) in [(&t.ao_raw, &self.ao_pipe, &t.ao_bg), (&t.ao, &self.ao_blur_pipe, &t.ao_blur_bg)] {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("ao"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::WHITE), store: wgpu::StoreOp::Store },
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
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 5);
        }
        self.clouds.render(device, queue, encoder, &main_cf, &t.hdr, &t.dist, 0);
        // Weather: ground fog / mist (exact per-pixel optical depth), then
        // rain / snow particles in front of everything.
        {
            let wp = &mut self.wpass;
            // Camera velocity (for the rain streaks' motion blur).
            if let Some((e, t0)) = wp.last_eye {
                let dt = fv.time - t0;
                if dt > 1e-4 && dt < 0.5 {
                    let v = (fv.eye - e) / dt;
                    wp.cam_vel = if v.is_finite() && v.length() < 600.0 { wp.cam_vel.lerp(v, 0.5) } else { wp.cam_vel };
                } else if dt.abs() > 0.5 {
                    wp.cam_vel = Vec3::ZERO;
                }
            }
            if wp.last_eye.is_none_or(|(_, t0)| t0 != fv.time) {
                wp.last_eye = Some((fv.eye, fv.time));
            }
            fn fullscreen_pass<'e>(encoder: &'e mut wgpu::CommandEncoder, label: &str, hdr: &wgpu::TextureView) -> wgpu::RenderPass<'e> {
                encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                    label: Some(label),
                    color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                        view: hdr,
                        depth_slice: None,
                        resolve_target: None,
                        ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
                    })],
                    depth_stencil_attachment: None,
                    timestamp_writes: None,
                    occlusion_query_set: None,
                    multiview_mask: None,
                })
            }
            if wx.fog_ext > 0.0 && wx.fog_top > 1.0 {
                let u = FogU {
                    inv_view_proj: vp_rot.inverse().to_cols_array_2d(),
                    eye: [fv.eye.x, fv.eye.y, fv.eye.z, wx.fog_ext],
                    fog: [wx.fog_top, cover, atmosphere::SUN_E, haze],
                    sun: [sun.x, sun.y, sun.z, width as f32],
                    screen: [width as f32, height as f32, 1.0 / width as f32, 1.0 / height as f32],
                };
                queue.write_buffer(&wp.fog_buf, 0, bytemuck::bytes_of(&u));
                use wgpu::BindingResource as R;
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("fog"),
                    layout: &wp.fog_pipe.get_bind_group_layout(0),
                    entries: &[
                        wgpu::BindGroupEntry { binding: 0, resource: wp.fog_buf.as_entire_binding() },
                        wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(&t.dist) },
                        wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(&self.atmos.trans) },
                        wgpu::BindGroupEntry { binding: 3, resource: R::Sampler(&self.lin_samp) },
                        wgpu::BindGroupEntry { binding: 4, resource: self.atmos.out.as_entire_binding() },
                    ],
                });
                let mut pass = fullscreen_pass(encoder, "fog", &t.hdr);
                pass.set_pipeline(&wp.fog_pipe);
                pass.set_bind_group(0, &bg, &[]);
                pass.draw(0..3, 0..1);
            }
            let precip = wx.rain.max(wx.snow);
            if precip > 0.0 {
                let snow = wx.snow > wx.rain;
                let focal = height as f32 * 0.5 / (fv.fov_y * 0.5).tan();
                let u = RainU {
                    view_proj: vp_rot.to_cols_array_2d(),
                    eye: [fv.eye.x, fv.eye.y, fv.eye.z, fv.time],
                    cam_vel: [wp.cam_vel.x, wp.cam_vel.y, wp.cam_vel.z, 1.0 / 60.0],
                    params: [precip, if snow { 1.0 } else { 0.0 }, wx.wind.x, wx.wind.y],
                    sun: [sun.x, sun.y, sun.z, 1.0 - 0.9 * cover],
                    screen: [width as f32, height as f32, focal, if wx.drizzle { 1.0 } else { 0.0 }],
                };
                queue.write_buffer(&wp.rain_buf, 0, bytemuck::bytes_of(&u));
                use wgpu::BindingResource as R;
                let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
                    label: Some("rain"),
                    layout: &wp.rain_pipe.get_bind_group_layout(0),
                    entries: &[
                        wgpu::BindGroupEntry { binding: 0, resource: wp.rain_buf.as_entire_binding() },
                        wgpu::BindGroupEntry { binding: 1, resource: R::TextureView(&t.dist) },
                        wgpu::BindGroupEntry { binding: 2, resource: self.atmos.out.as_entire_binding() },
                    ],
                });
                let mut pass = fullscreen_pass(encoder, "rain", &t.hdr);
                pass.set_pipeline(&wp.rain_pipe);
                pass.set_bind_group(0, &bg, &[]);
                pass.draw(0..6, 0..3 * RAIN_PER_LAYER);
            }
        }
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 6);
        }

        // 4. Bloom chain.
        let fullscreen = |encoder: &mut wgpu::CommandEncoder,
                          label: &str,
                          view: &wgpu::TextureView,
                          load: wgpu::LoadOp<wgpu::Color>,
                          pipe: &wgpu::RenderPipeline,
                          bg: &wgpu::BindGroup,
                          slot: usize| {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some(label),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view,
                    depth_slice: None,
                    resolve_target: None,
                    ops: wgpu::Operations { load, store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
                multiview_mask: None,
            });
            pass.set_pipeline(pipe);
            pass.set_bind_group(0, bg, &[(UNIFORM_STRIDE * slot as u64) as u32]);
            pass.draw(0..3, 0..1);
        };
        let clear = wgpu::LoadOp::Clear(wgpu::Color::BLACK);
        for i in 0..BLOOM_LEVELS {
            fullscreen(encoder, "bloom down", &t.bloom[i].0, clear, &self.bloom_down, &t.down_bgs[i], i);
        }
        for (k, i) in (1..BLOOM_LEVELS).rev().enumerate() {
            fullscreen(encoder, "bloom up", &t.bloom[i - 1].0, wgpu::LoadOp::Load, &self.bloom_up, &t.up_bgs[k], BLOOM_LEVELS + k);
        }
        // 5. Tonemap to output.
        fullscreen(encoder, "tonemap", out, clear, &self.tonemap, &t.tonemap_bg, 2 * BLOOM_LEVELS);
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 7);
            p.finish(encoder);
        }
        drop(tiles);
        self.prev_vp = Some((vp_rot, fv.eye));
        self.stats = stats;
    }
}

/// Fit a stable, texel-snapped orthographic light matrix around a slice of
/// the camera frustum. Returns the matrix and the covered radius (m).
/// Light matrix for one cascade of `view` (eye-relative: rotation only).
/// `anchor` (the world origin, eye-relative) fixes the texel snapping to the
/// world, so shadows don't crawl as the camera moves.
fn cascade_matrix(view: Mat4, fov_y: f32, aspect: f32, near: f32, far: f32, sun: Vec3, anchor: glam::DVec3) -> (Mat4, f32) {
    let inv_view = view.inverse();
    let th = (fov_y * 0.5).tan();
    let mut corners = Vec::with_capacity(8);
    for d in [near, far] {
        for (sx, sy) in [(-1.0, -1.0), (1.0, -1.0), (1.0, 1.0), (-1.0, 1.0)] {
            corners.push(inv_view.transform_point3(vec3(sx * th * aspect * d, sy * th * d, -d)));
        }
    }
    let center = corners.iter().copied().sum::<Vec3>() / 8.0;
    let mut radius = corners.iter().map(|c| c.distance(center)).fold(0.0, f32::max);
    radius = (radius * 16.0).ceil() / 16.0;
    let back = radius + 5000.0;
    let up = if sun.y.abs() > 0.99 { Vec3::Z } else { Vec3::Y };
    let light_view = glam::camera::rh::view::look_at_mat4(center + sun * back, center, up);
    let mut proj = glam::camera::rh::proj::directx::orthographic(-radius, radius, -radius, radius, 0.0, back + radius + 1500.0);
    // Snap to texels so the shadow doesn't shimmer as the camera moves.
    let m = proj * light_view;
    // (In f64: the anchor may be far away in light space.)
    let origin = m.as_dmat4() * anchor.extend(1.0);
    let half = SHADOW_RES as f64 * 0.5;
    let o2 = glam::DVec2::new(origin.x, origin.y);
    let off = (o2 * half).round() / half - o2;
    proj.w_axis.x += off.x as f32;
    proj.w_axis.y += off.y as f32;
    (proj * light_view, radius)
}

// ---------------------------------------------------------------- GPU context

pub struct Gpu {
    pub instance: wgpu::Instance,
    pub adapter: wgpu::Adapter,
    pub device: wgpu::Device,
    pub queue: wgpu::Queue,
}

impl Gpu {
    pub fn new(surface: Option<&wgpu::Surface<'_>>, instance: wgpu::Instance) -> Result<Gpu, String> {
        let adapter = pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            force_fallback_adapter: false,
            compatible_surface: surface,
            apply_limit_buckets: false,
        }))
        .map_err(|e| format!("no GPU adapter: {e}"))?;
        let (device, queue) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("skyward"),
            // Timestamp queries only for the optional profiler.
            // Immediates carry the per-tile transform (floating origin).
            required_features: wgpu::Features::IMMEDIATES
                | adapter.features() & (wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS),
            required_limits: wgpu::Limits {
                max_texture_dimension_2d: 8192,
                max_buffer_size: 1 << 30,
                max_immediate_size: IMMEDIATE_SIZE,
                ..wgpu::Limits::default()
            },
            ..Default::default()
        }))
        .map_err(|e| format!("request_device: {e}"))?;
        Ok(Gpu { instance, adapter, device, queue })
    }

    pub fn instance() -> wgpu::Instance {
        wgpu::Instance::new(wgpu::InstanceDescriptor {
            backends: wgpu::Backends::VULKAN,
            ..wgpu::InstanceDescriptor::new_without_display_handle()
        })
    }

    pub fn adapter_name(&self) -> String {
        let i = self.adapter.get_info();
        format!("{} ({:?})", i.name, i.backend)
    }
}

/// Read back an Rgba8 texture into tightly packed RGBA bytes.
pub fn read_texture(gpu: &Gpu, texture: &wgpu::Texture, w: u32, h: u32) -> Vec<u8> {
    let bpr = (w * 4).div_ceil(256) * 256;
    let buf = gpu.device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("readback"),
        size: (bpr * h) as u64,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });
    let mut enc = gpu.device.create_command_encoder(&Default::default());
    enc.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo { texture, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
        wgpu::TexelCopyBufferInfo {
            buffer: &buf,
            layout: wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(bpr), rows_per_image: Some(h) },
        },
        wgpu::Extent3d { width: w, height: h, depth_or_array_layers: 1 },
    );
    gpu.queue.submit([enc.finish()]);
    let slice = buf.slice(..);
    slice.map_async(wgpu::MapMode::Read, |_| {});
    let _ = gpu.device.poll(wgpu::PollType::wait_indefinitely());
    let data = slice.get_mapped_range().expect("map readback");
    let mut out = Vec::with_capacity((w * h * 4) as usize);
    for row in 0..h {
        let s = (row * bpr) as usize;
        out.extend_from_slice(&data[s..s + (w * 4) as usize]);
    }
    out
}
