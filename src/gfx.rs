//! wgpu renderer: physically based atmosphere (sky, aerial perspective,
//! ambient, exposure), volumetric clouds with cloud shadows, up to 4 blended
//! shadow cascades, planar water reflection, HDR scene (TAA / TAAU, MSAA or
//! FXAA: gfx/taa.rs), motion blur, depth of field, bloom, lens effects and
//! AgX tonemap. Every quality knob comes from [`quality::Quality`]
//! ([`Renderer::set_quality`]). Works with a window surface or headless.

pub mod hdr_ui;
pub mod perf;
pub mod quality;
pub mod taa;

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
/// Scene sample count (MSAA 4× or 1: TAA / FXAA / off), fixed at startup
/// ([`configure`]) — every scene pipeline (here, aircraft, airport, lights)
/// is built for it.
static SAMPLES: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(1);
/// Texture sampling (aircraft / airport): anisotropy, minimum mip level.
static TEX_ANISO: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(8);
static TEX_LOD: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);

/// The scene's MSAA sample count (pipelines must match it).
pub fn msaa() -> u32 {
    SAMPLES.load(std::sync::atomic::Ordering::Relaxed)
}

/// (anisotropy clamp, minimum mip level) for material texture samplers.
pub fn texture_sampling() -> (u16, f32) {
    (TEX_ANISO.load(std::sync::atomic::Ordering::Relaxed) as u16, f32::from_bits(TEX_LOD.load(std::sync::atomic::Ordering::Relaxed)))
}

/// Settings fixed when the renderer is built (call before [`Renderer::new`]):
/// MSAA sample count and texture sampling.
pub fn configure(q: &quality::Quality) {
    use std::sync::atomic::Ordering::Relaxed;
    SAMPLES.store(q.aa.samples(), Relaxed);
    TEX_ANISO.store(q.aniso.clamp(1, 16) as u32, Relaxed);
    TEX_LOD.store([1.0f32, 0.5, 0.0][q.textures.min(2) as usize].to_bits(), Relaxed);
}

/// Largest shadow cascade resolution (lower in the browser: GPU budget, memory).
pub const SHADOW_RES: u32 = if cfg!(target_arch = "wasm32") { 2048 } else { 4096 };
const CASCADES: usize = 4;
/// Cascade far distances (m) by the number of cascades drawn (2, 3, 4),
/// before the shadow-distance multiplier.
const SPLITS_BY_COUNT: [[f32; 5]; 3] = [[0.3, 200.0, 4000.0, 4000.0, 4000.0], [0.3, 80.0, 600.0, 6000.0, 6000.0], [0.3, 45.0, 220.0, 1100.0, 6000.0]];
/// Fraction of each cascade over which it blends into the next.
const CASCADE_BLEND: f32 = 0.15;
const BLOOM_LEVELS: usize = 6;
const NEAR: f32 = 0.25;
/// Bytes of immediate data (push constants) the pipelines declare: none —
/// WebGPU has none, the per-draw transform is an [`Xforms`] uniform.
const IMMEDIATE_SIZE: u32 = 0;
/// Bind group of the per-draw transform (common.wgsl / shadow.wgsl `xf`).
pub const XF_GROUP: u32 = 1;
/// Dynamic-offset stride (WebGPU's minimum uniform offset alignment).
const XF_STRIDE: u64 = 256;
/// Transforms per frame (draws whose transform differs from the previous one's).
const XF_CAP: u64 = 8192;

/// Per-draw transforms (the floating origin's `xf`: a tile's frame →
/// eye-relative, osm::Xform2::gpu) as one uniform buffer bound with a
/// dynamic offset per draw. Draws record their transform with [`Xforms::set`]
/// while the passes are encoded; [`Xforms::flush`] uploads them before the
/// frame is submitted (queue writes are ordered before later submissions).
pub struct Xforms {
    buf: wgpu::Buffer,
    pub bgl: wgpu::BindGroupLayout,
    bg: wgpu::BindGroup,
    st: std::sync::Mutex<XfState>,
}

#[derive(Default)]
struct XfState {
    data: Vec<u8>,
    n: u64,
    last: Option<[f32; 4]>,
    warned: bool,
}

impl Xforms {
    pub fn new(device: &wgpu::Device) -> Xforms {
        let buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("xforms"),
            size: XF_STRIDE * XF_CAP,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("xform"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 15,
                visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: true, min_binding_size: wgpu::BufferSize::new(16) },
                count: None,
            }],
        });
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("xform"),
            layout: &bgl,
            entries: &[wgpu::BindGroupEntry { binding: 15, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: &buf, offset: 0, size: wgpu::BufferSize::new(16) }) }],
        });
        Xforms { buf, bgl, bg, st: Default::default() }
    }

    /// Start a frame's transforms.
    pub fn reset(&self) {
        let mut s = self.st.lock().unwrap();
        s.n = 0;
        s.last = None;
    }

    /// Use transform `xf` for the following draws of `pass`.
    pub fn set(&self, pass: &mut wgpu::RenderPass<'_>, xf: &[f32; 4]) {
        let mut s = self.st.lock().unwrap();
        if s.last != Some(*xf) {
            if s.n < XF_CAP {
                s.n += 1;
            } else if !s.warned {
                s.warned = true;
                eprintln!("gfx: more than {XF_CAP} transforms in a frame");
            }
            let at = ((s.n - 1) * XF_STRIDE) as usize;
            if s.data.len() < at + XF_STRIDE as usize {
                s.data.resize(at + XF_STRIDE as usize, 0);
            }
            s.data[at..at + 16].copy_from_slice(bytemuck::cast_slice(xf));
            s.last = Some(*xf);
        }
        pass.set_bind_group(XF_GROUP, &self.bg, &[((s.n - 1) * XF_STRIDE) as u32]);
    }

    /// Upload this frame's transforms (before the frame is submitted).
    pub fn flush(&self, queue: &wgpu::Queue) {
        let s = self.st.lock().unwrap();
        if s.n > 0 {
            queue.write_buffer(&self.buf, 0, &s.data[..(s.n * XF_STRIDE) as usize]);
        }
    }
}

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
    // --- night & lights (lights.rs). `sun_dir` / `sun_e` above are the *key*
    // light: the sun by day, the moon at night.
    /// The real sun direction; w = night factor 0 (day) .. 1 (night).
    sun_true: [f32; 4],
    /// Moon direction; w = illuminated fraction.
    moon: [f32; 4],
    /// x sky-LUT scale (SUN_E), y full-moon disc radiance, z earthshine radiance, w pixels per radian.
    sky: [f32; 4],
    /// Columns of the equatorial → world rotation (stars).
    stars0: [f32; 4],
    stars1: [f32; 4],
    stars2: [f32; 4],
    /// x street lights on, y office windows lit, z home windows lit, w local hours.
    city: [f32; 4],
    /// Ground light map: eye-relative centre x, z, 1 / size, enabled.
    lmap: [f32; 4],
    /// x airport lights level, y aerodrome beacons on, z ground fog extinction (/km), w fog top (m).
    misc: [f32; 4],
    /// x traffic density, y number of aircraft spot / point lights, z moon phase angle, w unused.
    misc2: [f32; 4],
    /// Aircraft spot / point lights (see lights::gpu::pack_spots).
    alights: [[f32; 4]; 3 * crate::lights::place::AC_SPOTS],
    /// x pre-exposure (radiances written to the HDR targets are x this), y its inverse.
    pre: [f32; 4],
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
    /// Tonemap: x sharpen (CAS), y lens flare, z lens dirt, w chromatic aberration.
    fx: [f32; 4],
    /// Tonemap: x film grain, y frame time (s), z brightness, w display gamma.
    fx2: [f32; 4],
    /// Tonemap: x HDR output on, y paper white / 80 nits, z peak / paper white, w unused.
    hdr: [f32; 4],
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
    /// Ground light map (street lights glowing in the fog).
    lmap: [f32; 4],
    /// x samples along the ray, y samples for the street-light glow.
    quality: [f32; 4],
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
    /// Date, time and place (moon, stars, city life).
    pub clock: crate::atmosphere::SkyClock,
    /// The aircraft's lights this frame (eye-relative).
    pub acl: crate::lights::AircraftLights,
    /// The textured aircraft model for this frame (gfx_aircraft.rs).
    pub aircraft: Option<crate::gfx_aircraft::AircraftFrame>,
    /// Particles, trails, vapour and heat haze (vfx/).
    pub vfx: Option<std::sync::Arc<crate::vfx::VfxFrame>>,
}

/// Trees are drawn up to this distance (m) from the eye; detailed within TREE_LOD0.
const TREE_DIST: f32 = 4200.0;
const TREE_LOD0: f32 = 1100.0;
const TREE_REFL_DIST: f32 = 1500.0;
const BUILDING_REFL_DIST: f32 = 6000.0;
/// Half-size of the base ground quad that follows the camera (m): a backstop
/// below the terrain (terrain.rs horizon tiles cover the view to ~95 km).
const GROUND_HALF: f32 = 60_000.0;
const GROUND_Y: f32 = -40.0;

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
    min_y: f32,
    /// Base / draped index ranges of `flat`, bucketed (terrain.rs).
    ranges: crate::terrain::FlatRanges,
    /// Terrain surface (tile frame) and water bodies (bounds, level).
    grid: std::sync::Arc<crate::terrain::Grid>,
    water: Vec<(Vec2, Vec2, f32)>,
    flat: Option<GpuMesh>,
    buildings: Option<GpuMesh>,
    /// Instance buffer: deciduous [0, split), conifers [split, n).
    trees: Option<(wgpu::Buffer, u32, u32)>,
    /// Street lights, traffic, skyglow sources (lights.rs).
    lights: Option<crate::lights::gpu::GpuTileLights>,
    bytes: u64,
}

impl GpuTile {
    fn dist(&self, p: Vec2) -> f32 {
        (self.lo - p).max(p - self.hi).max(Vec2::ZERO).length()
    }
    fn flat_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x, self.min_y - 0.5, self.lo.y), vec3(self.hi.x, self.grid.max + 0.5, self.hi.y))
    }
    fn solid_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x, self.min_y, self.lo.y), vec3(self.hi.x, self.max_h + 1.0, self.hi.y))
    }
    fn tree_box(&self) -> (Vec3, Vec3) {
        (vec3(self.lo.x - 10.0, self.grid.min - 1.0, self.lo.y - 10.0), vec3(self.hi.x + 10.0, self.grid.max + 26.0, self.hi.y + 10.0))
    }
    /// Ground height under a current-frame point inside the tile.
    fn ground(&self, p: Vec2) -> f32 {
        self.grid.height(self.map.inverse(p))
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

/// Scene-resolution targets.
struct Targets {
    width: u32,
    height: u32,
    /// Reflection resolution divisor (1, 2, 4).
    refl_div: u32,
    /// Multisampled colour / distance (MSAA only; resolved into `hdr` / `dist`).
    hdr_msaa: Option<wgpu::TextureView>,
    hdr: wgpu::TextureView,
    dist_msaa: Option<wgpu::TextureView>,
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
    main_bg: wgpu::BindGroup,
    refl_bg: wgpu::BindGroup,
    /// Size of all of the above (VRAM estimate).
    bytes: u64,
}

/// Bloom chain at the post-processing size (the AA output's).
struct Bloom {
    width: u32,
    height: u32,
    levels: Vec<(wgpu::TextureView, u32, u32)>,
    /// Down passes 1.. (src = previous level), up passes.
    down_bgs: Vec<wgpu::BindGroup>,
    up_bgs: Vec<wgpu::BindGroup>,
}

pub struct Renderer {
    /// Per-draw transforms, shared with the light renderer.
    xforms: std::sync::Arc<Xforms>,
    globals_bgl: wgpu::BindGroupLayout,
    post_bgl: wgpu::BindGroupLayout,
    sky_main: wgpu::RenderPipeline,
    /// Terrain base surfaces (depth write) and the draped layers on them
    /// (pulled toward the eye, no depth write): terrain.rs.
    flat_base: wgpu::RenderPipeline,
    flat_over: wgpu::RenderPipeline,
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
    /// With an HDR output: the tonemap for an sRGB target too (thumbnails, [`Renderer::ldr`]).
    tonemap_ldr: Option<wgpu::RenderPipeline>,
    /// Render the next frames into an `Rgba8UnormSrgb` target (HDR output only).
    pub ldr: bool,
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
    aircraft: crate::gfx_aircraft::AircraftRenderer,
    /// Airport detail: markings, objects, parked aircraft, vehicles (airport/gfx.rs).
    pub airport: crate::airport::gfx::AirportRenderer,
    /// This frame's airport instances (set before `render`).
    airport_frame: Option<crate::airport::Frame>,
    /// Other aircraft this frame (multiplayer, net/): models and light sprites, eye-relative.
    traffic: Vec<crate::gfx_aircraft::AircraftFrame>,
    traffic_lights: Vec<crate::lights::LightInst>,
    targets: Option<Targets>,
    bloom: Option<Bloom>,
    /// Quality knobs and display effects (set_video).
    q: quality::Quality,
    video: quality::VideoSettings,
    /// TAA / FXAA / motion blur / depth of field.
    post: taa::Post,
    /// Current shadow-map resolution (q.shadow_res once applied).
    shadow_res: u32,
    /// Dynamic-resolution scale (None = the quality's render scale), and the
    /// largest scale allowed (the browser's pixel budget).
    dyn_scale: Option<f32>,
    scale_cap: f32,
    /// Last frame's pre-exposure (the TAA history is rescaled when it changes).
    prev_pre: f32,
    /// This frame's scene and output sizes.
    pub scene_size: (u32, u32),
    pub out_size: (u32, u32),
    atmos: Atmosphere,
    clouds: CloudRenderer,
    /// Last frame's camera (eye, forward) to detect cuts.
    last_cam: Option<(Vec3, Vec3)>,
    ao_pipe: wgpu::RenderPipeline,
    ao_blur_pipe: wgpu::RenderPipeline,
    ao_buf: wgpu::Buffer,
    prof: Option<perf::GpuProfiler>,
    /// Last frame's view-projection, once an AO result for it exists.
    /// Last frame's rotation-only view-projection and eye (AO reprojection).
    prev_vp: Option<(Mat4, Vec3)>,
    /// The same for TAA / motion blur / velocities: kept when only the render
    /// targets are re-created (dynamic resolution), dropped on camera cuts.
    cam_prev: Option<(Mat4, Vec3)>,
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
    /// Night lighting: light sprites, light map, stars, beams (lights.rs).
    lights: crate::lights::LightRenderer,
    /// Effects: particles, trails, vapour, heat haze (vfx/gpu.rs).
    vfx: crate::vfx::gpu::VfxRenderer,
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

/// The shadow cascades (`res`² × CASCADES depth array): per-layer views, array view.
fn shadow_maps(device: &wgpu::Device, res: u32) -> (Vec<wgpu::TextureView>, wgpu::TextureView) {
    let shadow_tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("shadow map"),
        size: wgpu::Extent3d { width: res, height: res, depth_or_array_layers: CASCADES as u32 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: DEPTH,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
        view_formats: &[],
    });
    let array = shadow_tex.create_view(&wgpu::TextureViewDescriptor { dimension: Some(wgpu::TextureViewDimension::D2Array), ..Default::default() });
    let layers = (0..CASCADES as u32)
        .map(|i| shadow_tex.create_view(&wgpu::TextureViewDescriptor { dimension: Some(wgpu::TextureViewDimension::D2), base_array_layer: i, array_layer_count: Some(1), ..Default::default() }))
        .collect();
    (layers, array)
}

fn bgl_tex(binding: u32, dim: wgpu::TextureViewDimension) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable: true }, view_dimension: dim, multisampled: false },
        count: None,
    }
}

fn bgl_storage(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT,
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
        let xforms = std::sync::Arc::new(Xforms::new(device));
        let lights = crate::lights::LightRenderer::new(device, queue, &format!("{core}\n{common}"), xforms.clone());
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
                    // (Vertex too: the effects light particles per vertex, vfx.wgsl.)
                    visibility: vf,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                bgl_tex(5, Dim::D2),
                bgl_tex(6, Dim::D2),
                bgl_tex(7, Dim::D3),
                bgl_tex(8, Dim::D2),
                bgl_storage(9),
                bgl_tex(10, Dim::D2),
                bgl_tex(11, Dim::D2),
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
                        min_binding_size: wgpu::BufferSize::new(std::mem::size_of::<PostU>() as u64),
                    },
                    count: None,
                },
                bgl_storage(4),
            ],
        });

        let scene_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("scene"),
            bind_group_layouts: &[Some(&globals_bgl), Some(&xforms.bgl)],
            immediate_size: IMMEDIATE_SIZE,
        });
        // The sky draws no tile geometry (no transform group to bind).
        let sky_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("sky"),
            bind_group_layouts: &[Some(&globals_bgl)],
            immediate_size: IMMEDIATE_SIZE,
        });
        let shadow_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("shadow"),
            bind_group_layouts: &[Some(&shadow_bgl), Some(&xforms.bgl)],
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
                layout: Some(if vs == "vs_sky" { &sky_layout } else { &scene_layout }),
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
        let sky_main = scene_pipe("sky", "vs_sky", "fs_sky", false, C::Always, false, msaa());
        let flat_base = scene_pipe("flat base", "vs_main", "fs_main", true, C::GreaterEqual, true, msaa());
        let flat_over = scene_pipe("flat over", "vs_over", "fs_main", true, C::GreaterEqual, false, msaa());
        let solid_main = scene_pipe("solid", "vs_main", "fs_main", true, C::GreaterEqual, true, msaa());
        let sky_refl = scene_pipe("sky refl", "vs_sky", "fs_sky", false, C::Always, false, 1);
        let solid_refl = scene_pipe("solid refl", "vs_main", "fs_main", true, C::GreaterEqual, true, 1);
        let tree_main = scene_pipe("tree", "vs_tree", "fs_main", true, C::GreaterEqual, true, msaa());
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
        let tonemap_ldr = (output_format != wgpu::TextureFormat::Rgba8UnormSrgb).then(|| post_pipe("tonemap ldr", "fs_tonemap", wgpu::TextureFormat::Rgba8UnormSrgb, None));

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

        let (shadow_layers, shadow_array) = shadow_maps(device, SHADOW_RES);
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
        let vfx = crate::vfx::gpu::VfxRenderer::new(device, &globals_bgl, IMMEDIATE_SIZE, HDR);
        let mut clouds = CloudRenderer::new(device, queue, atmos.cloud_views());
        clouds.set_field(&CloudField::new(7, crate::clouds::Weather::Scattered));

        Renderer {
            xforms,
            aircraft: crate::gfx_aircraft::AircraftRenderer::new(device, queue, &globals_bgl, &shadow_bgl, IMMEDIATE_SIZE),
            airport: crate::airport::gfx::AirportRenderer::new(device, queue, &globals_bgl, &shadow_bgl),
            airport_frame: None,
            traffic: Vec::new(),
            traffic_lights: Vec::new(),
            globals_bgl,
            post_bgl,
            sky_main,
            flat_base,
            flat_over,
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
            tonemap_ldr,
            ldr: false,
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
            bloom: None,
            q: quality::Quality::default(),
            video: quality::VideoSettings::default(),
            post: taa::Post::new(device),
            shadow_res: SHADOW_RES,
            dyn_scale: None,
            scale_cap: 2.0,
            prev_pre: 1.0,
            scene_size: (1, 1),
            out_size: (1, 1),
            atmos,
            clouds,
            last_cam: None,
            ao_pipe,
            ao_blur_pipe,
            ao_buf,
            prof: perf::GpuProfiler::new(device, queue),
            prev_vp: None,
            cam_prev: None,
            proj: Proj::new(0.0, 0.0),
            pattern: [1.0, 0.0, 0.0, 0.0],
            anchor: (0.0, 0.0),
            output_format,
            stats: RenderStats::default(),
            wxr: crate::weather::WxRender { season: crate::weather::Season::at(crate::sim::DEFAULT_DAY, 51.5), ..Default::default() },
            cloud_version: 0,
            wpass: WeatherPasses::new(device),
            vfx,
            lights,
        }
    }

    /// Replace the airport lighting (world frame, `lights::place::airport_lights`).
    /// Airport instances for the next `render` (airport/: parked aircraft, vehicles, windsocks).
    pub fn set_airport_frame(&mut self, f: crate::airport::Frame) {
        self.airport_frame = Some(f);
    }

    /// Other aircraft for the next `render` (multiplayer traffic: real models,
    /// eye-relative like `FrameView::aircraft`, and their light sprites).
    pub fn set_traffic(&mut self, frames: Vec<crate::gfx_aircraft::AircraftFrame>, lights: Vec<crate::lights::LightInst>) {
        self.traffic = frames;
        self.traffic_lights = lights;
    }

    pub fn set_airport_lights(&mut self, device: &wgpu::Device, l: &[crate::lights::LightInst]) {
        self.lights.set_airport(device, l);
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
        let tile_rect = t.id.rect(&Proj::tile_frame(t.id));
        let lights = crate::lights::gpu::GpuTileLights::new(device, &t.lights, tile_rect);
        let bytes = flat.as_ref().map_or(0, |m| m.bytes()) + buildings.as_ref().map_or(0, |m| m.bytes()) + trees.as_ref().map_or(0, |t| t.0.size()) + lights.as_ref().map_or(0, |l| l.bytes);
        let (lo, hi) = t.id.rect(&self.proj);
        let map = Proj::tile_frame(t.id).map_to(&self.proj);
        let gt = GpuTile {
            id: t.id,
            lo,
            hi,
            map,
            xf: map.gpu(),
            max_h: t.max_h,
            min_y: t.min_y,
            ranges: t.ranges.clone(),
            grid: t.grid.clone(),
            water: t.water.clone(),
            flat,
            buildings,
            trees,
            lights,
            bytes,
        };
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
        if let Some((_, e)) = &mut self.cam_prev {
            *e += delta;
        }
        if let Some((_, e)) = &mut self.prev_vp {
            *e += delta;
        }
        if let Some((e, _)) = &mut self.last_cam {
            *e += delta;
        }
        self.clouds.rebase(delta);
        self.lights.rebase();
    }

    /// Forget temporal history (camera cut / teleport).
    pub fn reset_history(&mut self) {
        self.clouds.reset_history();
        self.cam_prev = None;
        self.post.reset();
        self.aircraft.reset_motion();
    }

    /// GPU pass timings (timestamp queries, where supported): read back the
    /// finished frames' times without waiting (call after each submit).
    /// `SKYWARD_GPU_TIMING=1` also prints 120-frame averages.
    pub fn collect_timings(&mut self, device: &wgpu::Device) {
        self.collect_timings_wait(device, false);
    }

    /// As `collect_timings`; `wait`: block until this frame's times are in.
    pub fn collect_timings_wait(&mut self, device: &wgpu::Device, wait: bool) {
        if let Some(p) = &mut self.prof {
            p.collect(device, wait);
        }
    }

    /// Ground height at a current-frame point (finest resident tile).
    fn ground_at(&self, p: Vec2) -> Option<f32> {
        [crate::osm::ZOOM, crate::osm::FAR_ZOOM].iter().find_map(|&z| self.tiles.get(&TileId::at_world_z(&self.proj, p, z)).map(|t| t.ground(p)))
    }

    /// Height of the planar-reflection mirror: the surface of the water body
    /// nearest the eye (within 3 km), else the ground under the eye.
    fn reflection_plane(&self, eye: Vec2) -> (f32, bool) {
        let mut best: Option<(f32, f32)> = None;
        let mut under = None;
        for t in self.tiles.values().filter(|t| !t.id.is_far()) {
            if t.dist(eye) > 3000.0 {
                continue;
            }
            if t.dist(eye) == 0.0 {
                under = Some(t.ground(eye));
            }
            for (lo, hi, h) in &t.water {
                let (a, b) = (t.map.apply(*lo), t.map.apply(*hi));
                let d = (a.min(b) - eye).max(eye - a.max(b)).max(Vec2::ZERO).length();
                if d < 3000.0 && best.is_none_or(|x| d < x.0) {
                    best = Some((d, *h));
                }
            }
        }
        match best {
            Some(b) => (b.1, true),
            None => (under.unwrap_or(0.0), false),
        }
    }

    pub fn gpu_bytes(&self) -> u64 {
        self.tiles.values().map(|t| t.bytes).sum::<u64>()
            + self.pending.iter().map(|p| p.tile.bytes).sum::<u64>()
            + self.runways.as_ref().map_or(0, |m| m.bytes())
            + self.airport.bytes
    }

    fn ensure_targets(&mut self, device: &wgpu::Device, w: u32, h: u32) {
        let refl_div = [4, 2, 1][self.q.reflections.min(2) as usize];
        if let Some(t) = &self.targets {
            if t.width == w && t.height == h && t.refl_div == refl_div {
                return;
            }
        }
        let samples = msaa();
        let ra = wgpu::TextureUsages::RENDER_ATTACHMENT;
        let rt = ra | wgpu::TextureUsages::TEXTURE_BINDING;
        let ms = samples > 1;
        let hdr_msaa = ms.then(|| tex(device, "hdr msaa", w, h, HDR, samples, ra));
        let hdr = tex(device, "hdr", w, h, HDR, 1, rt);
        let dist_msaa = ms.then(|| tex(device, "dist msaa", w, h, DIST, samples, ra));
        let dist = tex(device, "dist", w, h, DIST, 1, rt);
        let depth = tex(device, "depth", w, h, DEPTH, samples, ra);
        let (rw, rh) = ((w / refl_div).max(1), (h / refl_div).max(1));
        let refl = tex(device, "refl", rw, rh, HDR, 1, rt);
        let refl_dist = tex(device, "refl dist", rw, rh, DIST, 1, rt);
        let refl_depth = tex(device, "refl depth", rw, rh, DEPTH, 1, ra);
        let (aw, ah) = ((w / 2).max(1), (h / 2).max(1));
        let ao_raw = tex(device, "ao raw", aw, ah, wgpu::TextureFormat::Rg16Float, 1, rt);
        let ao = tex(device, "ao", aw, ah, wgpu::TextureFormat::R8Unorm, 1, rt);
        let px = w as u64 * h as u64;
        let bytes = px * (8 + 2 + 4) * samples as u64 + if ms { px * 10 } else { 0 } + (rw * rh) as u64 * 14 + (aw * ah) as u64 * 5;
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
                    wgpu::BindGroupEntry { binding: 11, resource: R::TextureView(self.lights.map_view()) },
                ],
            })
        };
        let main_bg = globals_bg(&self.globals_buf, &refl, &self.atmos.ap_main, &ao);
        let refl_bg = globals_bg(&self.refl_globals_buf, &self.dummy_refl, &self.atmos.ap_refl, &self.dummy_ao);
        self.clouds.reset_history();
        self.prev_vp = None;
        self.lights.bind(
            device,
            &crate::lights::gpu::Shared {
                globals: &self.globals_buf,
                refl_globals: &self.refl_globals_buf,
                samp: &self.lin_samp,
                trans: &self.atmos.trans,
                ap_main: &self.atmos.ap_main,
                ap_refl: &self.atmos.ap_refl,
                atmos_out: &self.atmos.out,
                dist: &dist,
            },
        );
        self.targets = Some(Targets {
            width: w,
            height: h,
            refl_div,
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
            main_bg,
            refl_bg,
            bytes,
        });
    }

    fn post_bg(&self, device: &wgpu::Device, src: &wgpu::TextureView, bloom_src: &wgpu::TextureView) -> wgpu::BindGroup {
        use wgpu::BindingResource as R;
        device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("post"),
            layout: &self.post_bgl,
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: R::TextureView(src) },
                wgpu::BindGroupEntry { binding: 1, resource: R::Sampler(&self.lin_samp) },
                wgpu::BindGroupEntry { binding: 2, resource: R::TextureView(bloom_src) },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: R::Buffer(wgpu::BufferBinding { buffer: &self.post_buf, offset: 0, size: wgpu::BufferSize::new(std::mem::size_of::<PostU>() as u64) }),
                },
                wgpu::BindGroupEntry { binding: 4, resource: self.atmos.out.as_entire_binding() },
            ],
        })
    }

    /// The bloom chain for a post-processing image of `w`×`h`.
    fn ensure_bloom(&mut self, device: &wgpu::Device, w: u32, h: u32) {
        if self.bloom.as_ref().is_some_and(|b| b.width == w && b.height == h) {
            return;
        }
        let rt = wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING;
        let mut levels = Vec::new();
        let (mut bw, mut bh) = (w, h);
        for i in 0..BLOOM_LEVELS {
            bw = (bw / 2).max(1);
            bh = (bh / 2).max(1);
            levels.push((tex(device, &format!("bloom{i}"), bw, bh, HDR, 1, rt), bw, bh));
        }
        let down_bgs = (1..BLOOM_LEVELS).map(|i| self.post_bg(device, &levels[i - 1].0, &self.dummy_refl)).collect();
        let up_bgs = (1..BLOOM_LEVELS).rev().map(|i| self.post_bg(device, &levels[i].0, &self.dummy_refl)).collect();
        self.bloom = Some(Bloom { width: w, height: h, levels, down_bgs, up_bgs });
    }

    /// Video settings for the coming frames (quality knobs apply live,
    /// except what [`quality::Quality::restart_diff`] lists).
    pub fn set_video(&mut self, v: &quality::VideoSettings) {
        if self.video.q.aa != v.q.aa || (self.video.q.render_scale - v.q.render_scale).abs() > 1e-4 {
            self.post.reset();
        }
        self.video = v.clone();
        self.q = v.q;
    }

    /// Dynamic resolution: the scale to render at (None = the quality's).
    pub fn set_dynamic_scale(&mut self, s: Option<f32>) {
        self.dyn_scale = s;
    }

    /// The largest render scale allowed (the browser's pixel budget).
    pub fn set_scale_cap(&mut self, cap: f32) {
        self.scale_cap = cap.clamp(0.25, 2.0);
    }

    /// The render scale in effect.
    pub fn render_scale(&self) -> f32 {
        self.dyn_scale.unwrap_or(self.q.render_scale).min(self.scale_cap).clamp(0.25, 2.0)
    }

    /// GPU timings (smoothed, total first) when timestamp queries are available.
    pub fn gpu_times(&self) -> Option<Vec<(&'static str, f32)>> {
        self.prof.as_ref().map(|p| perf::MARKS.iter().enumerate().map(|(i, k)| (if i == 0 { "total" } else { *k }, p.smooth[i])).collect())
    }

    /// The last GPU frame time (ms), when measured.
    pub fn gpu_frame_ms(&self) -> Option<f32> {
        self.prof.as_ref().map(|p| p.last[0]).filter(|t| *t > 0.0)
    }

    /// Benchmark mode: keep every frame's GPU times (see `bench_medians`).
    pub fn bench_start(&mut self) {
        if let Some(p) = self.prof.as_mut() {
            p.bench = Some(Vec::new());
        }
    }

    /// Per-pass medians since `bench_start` (labels, total first).
    pub fn bench_medians(&self) -> Option<Vec<(&'static str, f32)>> {
        let m = self.prof.as_ref()?.bench_medians()?;
        Some(perf::MARKS.iter().zip(m).enumerate().map(|(i, (k, v))| (if i == 0 { "total" } else { *k }, v)).collect())
    }

    /// Estimated GPU memory (bytes): scenery buffers, render targets, shadow
    /// maps, post-processing and the fixed LUTs / cloud volumes.
    pub fn vram_estimate(&self) -> u64 {
        let targets = self.targets.as_ref().map_or(0, |t| t.bytes);
        let bloom = self.bloom.as_ref().map_or(0, |b| b.levels.iter().map(|l| l.1 as u64 * l.2 as u64 * 8).sum());
        let shadows = self.shadow_res as u64 * self.shadow_res as u64 * 4 * CASCADES as u64;
        // Cloud noise volumes, weather / shadow maps, atmosphere LUTs, light map (measured once).
        const FIXED: u64 = 96 << 20;
        self.gpu_bytes() + targets + bloom + shadows + self.post.bytes() + FIXED
    }

    /// Render one frame into `out` (format must equal `output_format`) of
    /// `width`×`height` pixels. The scene renders at the render scale
    /// (quality / dynamic resolution) and is upscaled by the AA pass.
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
        let (out_w, out_h) = (width.max(1), height.max(1));
        let q = self.q;
        // A new shadow-map resolution: new maps (the scene bind groups follow).
        if q.shadow_res != self.shadow_res {
            (self.shadow_layers, self.shadow_array) = shadow_maps(device, q.shadow_res);
            self.shadow_res = q.shadow_res;
            self.targets = None;
        }
        let scale = self.render_scale();
        let (width, height) = (((out_w as f32 * scale).round() as u32).max(1), ((out_h as f32 * scale).round() as u32).max(1));
        self.scene_size = (width, height);
        self.out_size = (out_w, out_h);
        self.ensure_targets(device, width, height);
        // Post-processing runs at the output size after TAA(U), else at the scene's.
        if q.aa == quality::AaMode::Taa {
            self.ensure_bloom(device, out_w, out_h);
        } else {
            self.ensure_bloom(device, width, height);
        }
        if let Some(p) = self.prof.as_mut() {
            p.begin();
        }
        self.clouds.set_quality(q.clouds);
        self.vfx.quality = q.vfx;
        // Moving landmark parts (London Eye), eye-relative like the aircraft.
        let lm = crate::landmarks::animated(&self.proj, fv.eye, fv.time, &|p| self.ground_at(p).unwrap_or(0.0));
        if lm.idx.is_empty() {
            self.dynamic.set(device, queue, dynamic);
        } else {
            let mut all = MeshData::default();
            all.append(dynamic, Mat4::IDENTITY);
            all.append(&lm, Mat4::IDENTITY);
            self.dynamic.set(device, queue, &all);
        }
        self.aircraft.prepare(device, queue, fv.aircraft.as_ref());
        let traffic = std::mem::take(&mut self.traffic);
        self.aircraft.prepare_traffic(device, queue, &traffic);
        if self.traffic_lights.is_empty() {
            self.lights.set_aircraft(queue, &fv.acl.sprites[..fv.acl.n_sprites]);
        } else {
            let mut all = fv.acl.sprites[..fv.acl.n_sprites].to_vec();
            all.append(&mut self.traffic_lights);
            self.lights.set_aircraft(queue, &all);
        }
        self.lights.set_coverage(queue, self.tiles.keys().filter(|id| !id.is_far()).copied());
        // Base ground follows the camera (snapped so the procedural pattern is stable).
        {
            let c = (Vec2::new(fv.eye.x, fv.eye.z) / 1000.0).round() * 1000.0;
            let g = GROUND_HALF;
            let mut m = MeshData { mat: scene::mat::GROUND, ..Default::default() };
            m.flat_quad([c + Vec2::new(-g, -g), c + Vec2::new(g, -g), c + Vec2::new(g, g), c + Vec2::new(-g, g)], GROUND_Y, scene::LAND);
            self.ground.set(device, queue, &m);
        }
        // Camera cuts drop the clouds' temporal history.
        let fwd = -fv.view.inverse().z_axis.truncate();
        if let Some((e, f)) = self.last_cam {
            if e.distance(fv.eye) > 300.0 || f.dot(fwd) < 0.8 {
                self.clouds.reset_history();
                self.prev_vp = None;
                self.cam_prev = None;
                self.post.reset();
                self.aircraft.reset_motion();
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
        // TAA: the scene is drawn with a sub-pixel jitter (Halton); culling,
        // clouds, reprojection and the previous frame stay unjittered.
        let taa_on = q.aa == quality::AaMode::Taa;
        let jit = if taa_on { taa::jitter(self.post.frame, taa::phases(out_w as f32 / width as f32)) } else { Vec2::ZERO };
        let proj_j = taa::jitter_proj(proj, jit, width, height);
        let vp = proj * fv.view;
        // Eye-relative (rotation-only) view-projection: screen rays are
        // reconstructed with it. Using the full matrix and subtracting the eye
        // cancels large world coordinates in f32 and makes the sky / clouds
        // judder against the (vertex-transformed, stable) ground far from the
        // world origin.
        let view_rot = Mat4::from_cols(fv.view.x_axis, fv.view.y_axis, fv.view.z_axis, Vec4::W);
        let vp_rot = proj * view_rot;
        let vp_rot_j = proj_j * view_rot;
        let sun = fv.sun_dir.normalize();
        let field = self.clouds.field();
        let cover = field.sky_cover();
        let wx = self.wxr;
        let haze = if wx.haze > 0.0 { wx.haze } else { haze_for(cover) };
        let fog = (wx.fog_ext, wx.fog_top * 0.001);
        // Night: moon, stars, the city's rhythm. The *key* light (direct light,
        // shadows, cloud lighting) is the sun, or the moon once the sun is down.
        let night = crate::lights::sky::NightState::new(&fv.clock, sun, cover);
        let smooth = |a: f32, b: f32, x: f32| {
            let t = ((x - a) / (b - a)).clamp(0.0, 1.0);
            t * t * (3.0 - 2.0 * t)
        };
        let (key, key_e) = if sun.y > -0.02 {
            (sun, Vec3::splat(atmosphere::SUN_E))
        } else {
            // Moonlight: a touch cooler than sunlight (the Purkinje shift in the tonemapper does the rest).
            (night.moon_dir, vec3(0.9, 1.0, 1.12) * night.moon_e * smooth(-0.02, -0.07, sun.y))
        };
        let night_k = 1.0 - smooth(-0.10, 0.06, sun.y);
        let lut_scale = atmosphere::lut_scale(sun.y);
        let pre = atmosphere::pre_exposure(sun.y);
        // Planar reflection plane: the nearest water body's surface, else the
        // ground under the eye (puddles).
        let (plane_h, water_plane) = self.reflection_plane(Vec2::new(fv.eye.x, fv.eye.z));
        // Skyglow from the far tiles' built-up areas (cached in the light renderer).
        let glow_src: Vec<(Vec2, f32)> = self.tiles.values().filter_map(|t| t.lights.as_ref().map(|l| (t, l))).flat_map(|(t, l)| l.glow.iter().map(move |(p, lm)| (t.map.apply(*p), *lm))).collect();
        let (glow, city_up) = self.lights.skyglow(&glow_src, fv.eye, haze);
        let street = night.street;
        if std::env::var_os("SKYWARD_NIGHT_DEBUG").is_some() {
            eprintln!(
                "night: sun {:.3} moon {:?} e {:.3e} frac {:.2} street {:.2} office {:.2} homes {:.2} | glow src {} up {:.3e} zen {:.3e} hor {:?}",
                sun.y, night.moon_dir, night.moon_e, night.moon_frac, night.street, night.office, night.homes, glow_src.len(), city_up, glow[7 * 16], &glow[0..16].iter().map(|g| format!("{g:.1e}")).collect::<Vec<_>>()
            );
            for l in &fv.acl.sprites[..fv.acl.n_sprites] {
                eprintln!("  ac sprite kind {} cd {} pos {:?} dist {:.0}", l.kind(), l.cd, l.pos, Vec3::from(l.pos).length());
            }
            for sp in &fv.acl.spots[..fv.acl.n_spots] {
                eprintln!("  ac spot {:?}", sp);
            }
        }
        // Visibility (km) for the airport lighting level.
        let vis_km = 3.0 / (atmosphere::transmittance_ext0(haze) + wx.fog_ext).max(1e-3);
        let low_vis = smooth(5.0, 1.5, vis_km);
        // Runway lights: full by day / in low visibility, a low step on a clear night.
        let airport_level = 1.0 + (0.12 + 0.88 * low_vis - 1.0) * night_k;
        let beacon = (1.0 - smooth(0.0, 0.1, sun.y)).max(low_vis);
        let moon_frac = night.moon_frac;
        let moon_full = atmosphere::moon_lux(0.0, 384_400.0) * atmosphere::LUX / crate::lights::sky::MOON_SOLID_ANGLE;
        let moon_disc = night.moon_e / crate::lights::sky::MOON_SOLID_ANGLE / crate::lights::sky::moon_disc_norm(night.moon_alpha);
        let earthshine = moon_full * 2.5e-4 * (1.0 - moon_frac).powf(1.5);
        // Thick decks (nimbostratus) let much less light through.
        let deck_trans = 1.0 - 0.75 * ((field.deck_density() - 0.3) / 0.7).clamp(0.0, 1.0);
        let cloud_base = field.base();

        // Cascades: each one also covers the blend band before its near split.
        // (Fewer cascades: the last ones are left clear, the shadow fades out
        // at the end of the last drawn one.)
        let n_casc = q.cascades.clamp(2, CASCADES as u8) as usize;
        let splits: [f32; 5] = {
            let b = SPLITS_BY_COUNT[n_casc - 2];
            [b[0], b[1] * q.shadow_dist, b[2] * q.shadow_dist, b[3] * q.shadow_dist, b[4] * q.shadow_dist]
        };
        let shadow_res = self.shadow_res;
        let mut shadow_mats = [[[0f32; 4]; 4]; CASCADES];
        let mut texel = [0f32; 4];
        for i in 0..n_casc {
            let near = if i == 0 { splits[0] } else { splits[i] - (splits[i] - splits[i - 1]) * CASCADE_BLEND };
            let anchor = glam::DVec3::new(self.anchor.0 - ex, -ey, self.anchor.1 - ez);
            let (m, radius) = cascade_matrix(view_rot, fv.fov_y, aspect, near, splits[i + 1], key, anchor, shadow_res);
            shadow_mats[i] = m.to_cols_array_2d();
            texel[i] = radius * 2.0 / shadow_res as f32;
            queue.write_buffer(&self.shadow_buf, UNIFORM_STRIDE * i as u64, bytemuck::cast_slice(&m.to_cols_array()));
            let leaf = self.wxr.season.leaf;
            queue.write_buffer(&self.shadow_buf, UNIFORM_STRIDE * i as u64 + 64, bytemuck::cast_slice(&[leaf, 0.0f32, 0.0, 0.0]));
        }

        let star_cols = crate::lights::gpu::star_cols(night.star_rot);
        let (spots, n_spots) = crate::lights::gpu::pack_spots(&fv.acl.spots[..fv.acl.n_spots]);
        // Ground light map centre, snapped to its texels (stable pools).
        let lmap_c = {
            let texel = crate::lights::gpu::MAP_SIZE / crate::lights::gpu::MAP_RES as f32;
            (Vec2::new(fv.eye.x, fv.eye.z) / texel).round() * texel
        };
        let t = self.targets.as_ref().unwrap();
        // Pattern space of the eye (f64, wrapped to the pattern period).
        let pat = {
            let wrapp = |v: f64| (v - crate::world::NOISE_PERIOD * (v / crate::world::NOISE_PERIOD).round()) as f32;
            let [sx, ox, oz, _] = self.pattern;
            [sx, wrapp(fv.eye.x as f64 * sx as f64 + ox as f64), wrapp(fv.eye.z as f64 + oz as f64), 0.0]
        };
        // Last frame's camera, relative to this frame's eye.
        let prev_rel = self.prev_vp.map(|(m, e)| m * Mat4::from_translation(fv.eye - e)).unwrap_or(vp_rot);
        let ao_on = q.ao > 0;
        let globals = Globals {
            view_proj: vp_rot_j.to_cols_array_2d(),
            inv_view_proj: vp_rot_j.inverse().to_cols_array_2d(),
            shadow: shadow_mats,
            cam_pos: [fv.eye.x, fv.eye.y, fv.eye.z, fv.time],
            sun_dir: [key.x, key.y, key.z, cover],
            sun_e: [key_e.x * pre, key_e.y * pre, key_e.z * pre, haze],
            splits: [splits[1], splits[2], splits[3], splits[4]],
            shadow_info: [shadow_res as f32, 0.0, CASCADE_BLEND, if self.prev_vp.is_some() && ao_on { 1.0 } else { 0.0 }],
            screen: [width as f32, height as f32, 1.0 / width as f32, 1.0 / height as f32],
            cloud: self.clouds.shadow_params(),
            texel,
            prev_view_proj: prev_rel.to_cols_array_2d(),
            wx: [wx.wet, wx.rain, wx.snow, wx.snow_cover],
            wx2: [wx.wind.x, wx.wind.y, wx.frost, wx.gust],
            season: [wx.season.leaf, wx.season.autumn, wx.season.spring, wx.season.dormant],
            season2: [wx.season.dry, wx.season.year, wx.season.strength, if wx.drizzle { 1.0 } else { 0.0 }],
            pattern: pat,
            sun_true: [sun.x, sun.y, sun.z, night_k],
            moon: [night.moon_dir.x, night.moon_dir.y, night.moon_dir.z, moon_frac],
            sky: [atmosphere::SUN_E / lut_scale * pre, moon_disc, earthshine, height as f32 * 0.5 / (fv.fov_y * 0.5).tan()],
            stars0: star_cols[0],
            stars1: star_cols[1],
            stars2: star_cols[2],
            city: [street, night.office, night.homes, night.hours],
            lmap: [lmap_c.x - fv.eye.x, lmap_c.y - fv.eye.z, 1.0 / crate::lights::gpu::MAP_SIZE, if street > 0.0 { 1.0 } else { 0.0 }],
            misc: [airport_level, beacon, wx.fog_ext, wx.fog_top],
            misc2: [night.traffic, n_spots as f32, night.moon_alpha, if q.soft_shadows { 1.0 } else { 0.0 }],
            alights: spots,
            // w: 1 = no MSAA — alpha-tested / translucent surfaces dither (TAA resolves it).
            pre: [pre, 1.0 / pre, plane_h, if msaa() > 1 { 0.0 } else { 1.0 + (self.post.frame % 64) as f32 }],
        };
        let px_per_m = height as f32 * 0.5 / (fv.fov_y * 0.5).tan();
        let aou = AoU {
            inv_view_proj: vp_rot_j.inverse().to_cols_array_2d(),
            // w: frame index (high quality rotates the samples per frame; TAA averages).
            eye: [fv.eye.x, fv.eye.y, fv.eye.z, if q.ao >= 3 { (self.post.frame % 64) as f32 } else { 0.0 }],
            params: [AO_RADIUS, px_per_m, AO_STRENGTH, [6.0, 6.0, 12.0, 16.0][q.ao.min(3) as usize]],
            size: [width as f32, height as f32, (width / 2) as f32, (height / 2) as f32],
        };
        queue.write_buffer(&self.ao_buf, 0, bytemuck::bytes_of(&aou));
        queue.write_buffer(&self.globals_buf, 0, bytemuck::bytes_of(&globals));
        // Mirror about the plane y = plane_h.
        let mirror = Mat4::from_scale(vec3(1.0, -1.0, 1.0));
        let mirror_h = Mat4::from_translation(vec3(0.0, plane_h, 0.0)) * mirror * Mat4::from_translation(vec3(0.0, -plane_h, 0.0));
        let rvp = proj * fv.view * mirror_h;
        let rvp_rot = vp_rot * mirror;
        let reye = vec3(fv.eye.x, 2.0 * plane_h - fv.eye.y, fv.eye.z);
        let eh = fv.eye.y - plane_h;
        let (rw, rh) = t.refl_size;
        // Mirrored pass, eye-relative: world p = rel + eye, mirrored and seen from the eye.
        let rvp_rel = proj * view_rot * Mat4::from_translation(vec3(0.0, -2.0 * eh, 0.0)) * mirror;
        let rglobals = Globals {
            view_proj: rvp_rel.to_cols_array_2d(),
            pattern: [pat[0], pat[1], pat[2], 2.0 * eh],
            inv_view_proj: rvp_rot.inverse().to_cols_array_2d(),
            cam_pos: [reye.x, reye.y, reye.z, fv.time],
            shadow_info: [shadow_res as f32, 1.0, CASCADE_BLEND, 0.0],
            screen: [rw as f32, rh as f32, 1.0 / rw as f32, 1.0 / rh as f32],
            sky: [globals.sky[0], globals.sky[1], globals.sky[2], rh as f32 * 0.5 / (fv.fov_y * 0.5).tan()],
            // (Aircraft lights are eye-relative to the real eye: not in the mirror.)
            misc2: [globals.misc2[0], 0.0, globals.misc2[2], globals.misc2[3]],
            pre: [globals.pre[0], globals.pre[1], globals.pre[2], 0.0],
            ..globals
        };
        queue.write_buffer(&self.refl_globals_buf, 0, bytemuck::bytes_of(&rglobals));

        if let Some(p) = self.prof.as_ref() {
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
                moon: night.moon_dir,
                moon_e: night.moon_e,
                airglow: crate::lights::sky::AIRGLOW,
                // Under cloud, light bounces between the city and the deck.
                city_up: city_up * street * (1.0 + 2.0 * cover * cover),
                // The eye adapts to the lit city in view (≈ lit streets' illuminance x coverage).
                art_meter: city_up * street * std::f32::consts::PI * 12.0,
                glow: glow.map(|g| g * street),
                glow_tint: crate::lights::sky::GLOW_TINT,
                lut_scale,
                pre,
            },
        );
        // Clouds: lit by the sun as it arrives at the middle of the layer.
        let sun_cloud = atmosphere::transmittance(cloud_base + 600.0, key.y, haze, fog) * key_e * pre;
        let cloud_frame = |view_proj: Mat4, rot_view_proj: Mat4, eye: Vec3, w: u32, h: u32| CloudFrame {
            view_proj,
            inv_view_proj: view_proj.inverse(),
            rot_view_proj,
            eye,
            sun_dir: key,
            sun_col: sun_cloud / std::f32::consts::PI,
            // (Read as the key light's irradiance relative to `fog[0]`.)
            sky_intensity: key_e.y * lut_scale / atmosphere::SUN_E,
            time: fv.time,
            width: w,
            height: h,
            fog: [atmosphere::SUN_E / lut_scale * pre, haze],
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
        // Draw distances (quality).
        let tree_dist = TREE_DIST * q.tree_dist;
        let tree_lod0 = TREE_LOD0 * q.tree_dist;
        let building_dist = crate::world::FAR_RADIUS * q.building_dist;
        {
            let af = self.airport_frame.take();
            let eye = fv.eye;
            self.airport.prepare(device, queue, &self.proj, eye, af.as_ref(), &|lo, hi| main_f.aabb(lo + eye, hi + eye));
        }
        // Airport markings / objects stand on a detailed tile's terrain: drawn while it is resident.
        let resident = |id: TileId| self.tiles.contains_key(&id);
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
        // Light sources per tile (lights.rs), culled per pass.
        let tile_draws: Vec<crate::lights::gpu::TileDraw> = tiles
            .iter()
            .filter_map(|t| {
                let l = t.lights.as_ref()?;
                let (lo, hi) = (vec3(t.lo.x, 0.0, t.lo.y), vec3(t.hi.x, t.max_h + 15.0, t.hi.y));
                Some(crate::lights::gpu::TileDraw { lights: l, xf: t.xf, dist: t.dist(eye2), visible: main_f.aabb(lo, hi), visible_refl: refl_f.aabb(lo, hi) })
            })
            .collect();
        let xfs = self.xforms.clone();
        xfs.reset();
        let place = |pass: &mut wgpu::RenderPass<'_>, xf: &[f32; 4]| xfs.set(pass, xf);
        fn draw_mesh(pass: &mut wgpu::RenderPass<'_>, m: &GpuMesh) {
            pass.set_vertex_buffer(0, m.vb.slice(..));
            pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..m.n, 0, 0..1);
        }
        // Instanced trees for one tile: (mesh lod per kind), the first
        // `density` of each kind's instances (they are in random order).
        fn draw_trees(pass: &mut wgpu::RenderPass<'_>, xfs: &Xforms, t: &GpuTile, tm: &TreeMeshes, lod: usize, density: f32) -> usize {
            let Some((buf, split, n)) = &t.trees else { return 0 };
            xfs.set(pass, &t.xf);
            pass.set_vertex_buffer(1, buf.slice(..));
            let part = |a: u32, b: u32| a..a + ((b - a) as f32 * density).ceil().min((b - a) as f32) as u32;
            let mut drawn = 0;
            for (kind, range) in [(0usize, part(0, *split)), (1, part(*split, *n))] {
                if range.is_empty() {
                    continue;
                }
                drawn += range.len();
                let m = &tm.m[kind][lod];
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..m.n, 0, range);
            }
            drawn
        }

        // 1. Shadow cascades (cleared only when the key light is a moon too
        // low or too thin to cast visible shadows).
        let cast_shadows = key == sun || (night.moon_dir.y > 0.02 && night.moon_frac > 0.2);
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
            if !cast_shadows || i >= n_casc {
                continue;
            }
            let f = &shadow_f[i];
            pass.set_bind_group(0, &self.shadow_bg, &[(UNIFORM_STRIDE * i as u64) as u32]);
            pass.set_pipeline(&self.shadow_pipe);
            // Terrain casts too (ridges shading valleys); its base surfaces only.
            for t in &tiles {
                let Some(m) = &t.flat else { continue };
                if t.grid.max - t.grid.min < 8.0 {
                    continue; // flat land: nothing to cast
                }
                let (lo, hi) = t.flat_box();
                if !f.aabb(lo, hi) {
                    continue;
                }
                let (base, _) = ground_runs(t, &self.tiles);
                place(&mut pass, &t.xf);
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                for r in &base {
                    pass.draw_indexed(r.0..r.1, 0, 0..1);
                }
            }
            for t in &tiles {
                if let Some(m) = &t.buildings {
                    let (lo, hi) = t.solid_box();
                    if f.aabb(lo, hi) {
                        place(&mut pass, &t.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            self.airport.draw_objects(&mut pass, &xfs, &resident, false, splits[i + 1] * 1.2);
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            self.airport.draw_shadow(&mut pass, i);
            self.aircraft.draw_shadow(&mut pass);
            pass.set_pipeline(&self.shadow_tree);
            for t in &tiles {
                let (lo, hi) = t.tree_box();
                let d = t.dist(eye2);
                if d < splits[i + 1] * 1.1 && d < tree_dist && f.aabb(lo, hi) {
                    draw_trees(&mut pass, &xfs, t, tm, if i == 0 { 0 } else { 1 }, q.tree_density);
                }
            }
        }

        if street > 0.0 {
            self.lights.draw_map(encoder, &tile_draws, world_xf);
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
            // Hills and mountains mirrored in the water (the mirror pass
            // discards everything below the water level).
            if water_plane {
                for tile in &tiles {
                    let Some(m) = &tile.flat else { continue };
                    if tile.grid.max < plane_h + 3.0 {
                        continue;
                    }
                    let (lo, hi) = tile.flat_box();
                    if !refl_f.aabb(lo, hi) {
                        continue;
                    }
                    let (base, _) = ground_runs(tile, &self.tiles);
                    place(&mut pass, &tile.xf);
                    pass.set_vertex_buffer(0, m.vb.slice(..));
                    pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                    for r in &base {
                        pass.draw_indexed(r.0..r.1, 0, 0..1);
                    }
                }
            }
            for tile in &tiles {
                if let Some(m) = &tile.buildings {
                    let (lo, hi) = tile.solid_box();
                    if tile.dist(eye2) < BUILDING_REFL_DIST && refl_f.aabb(lo, hi) {
                        place(&mut pass, &tile.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            if water_plane {
                self.airport.draw_objects(&mut pass, &xfs, &resident, false, 2500.0);
            }
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            self.aircraft.draw_refl(&mut pass);
            pass.set_pipeline(&self.tree_refl);
            for tile in &tiles {
                let (lo, hi) = tile.tree_box();
                if tile.dist(eye2) < TREE_REFL_DIST * q.tree_dist && refl_f.aabb(lo, hi) {
                    draw_trees(&mut pass, &xfs, tile, tm, 1, q.tree_density);
                }
            }
            self.lights.draw_sprites(&mut pass, &tile_draws, world_xf, true, street > 0.0 || night_k > 0.0);
        }
        self.clouds.render(device, queue, encoder, &refl_cf, &t.refl, &t.refl_dist, 1);
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 3);
        }

        // 3. Main HDR pass.
        {
            // MSAA: draw multisampled, resolve; else straight into the targets.
            let att = |ms: &Option<wgpu::TextureView>, single: &wgpu::TextureView| -> (wgpu::TextureView, Option<wgpu::TextureView>, wgpu::StoreOp) {
                match ms {
                    Some(m) => (m.clone(), Some(single.clone()), wgpu::StoreOp::Discard),
                    None => (single.clone(), None, wgpu::StoreOp::Store),
                }
            };
            let (hv, hr, hs) = att(&t.hdr_msaa, &t.hdr);
            let (dv, dr, ds) = att(&t.dist_msaa, &t.dist);
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("main"),
                color_attachments: &[
                    Some(wgpu::RenderPassColorAttachment {
                        view: &hv,
                        depth_slice: None,
                        resolve_target: hr.as_ref(),
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: hs },
                    }),
                    Some(wgpu::RenderPassColorAttachment {
                        view: &dv,
                        depth_slice: None,
                        resolve_target: dr.as_ref(),
                        ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: ds },
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
            // Ground: every tile's terrain surface (depth write), coarser
            // levels only where no finer tile is resident, then the draped
            // layers in painter's order (no depth write) and the runways.
            pass.set_pipeline(&self.flat_base);
            place(&mut pass, &world_xf);
            pass.set_vertex_buffer(0, self.ground.vb.slice(..));
            pass.set_index_buffer(self.ground.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(0..self.ground.n, 0, 0..1);
            let visible: Vec<(&GpuTile, &GpuMesh, Vec<(u32, u32)>, Vec<(u32, u32)>)> = tiles
                .iter()
                .filter_map(|tile| {
                    let m = tile.flat.as_ref()?;
                    let (lo, hi) = tile.flat_box();
                    if !main_f.aabb(lo, hi) {
                        return None;
                    }
                    let (b, o) = ground_runs(tile, &self.tiles);
                    Some((*tile, m, b, o))
                })
                .collect();
            for (tile, m, base, _) in &visible {
                place(&mut pass, &tile.xf);
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                for r in base {
                    pass.draw_indexed(r.0..r.1, 0, 0..1);
                }
                stats.tiles_drawn += 1;
            }
            pass.set_pipeline(&self.flat_over);
            for (tile, m, _, over) in &visible {
                place(&mut pass, &tile.xf);
                pass.set_vertex_buffer(0, m.vb.slice(..));
                pass.set_index_buffer(m.ib.slice(..), wgpu::IndexFormat::Uint32);
                for r in over {
                    pass.draw_indexed(r.0..r.1, 0, 0..1);
                }
            }
            self.airport.draw_marks(&mut pass, &xfs, &resident);
            if let Some(m) = &self.runways {
                place(&mut pass, &world_xf);
                draw_mesh(&mut pass, m);
            }
            pass.set_pipeline(&self.solid_main);
            for tile in &tiles {
                if let Some(m) = &tile.buildings {
                    let (lo, hi) = tile.solid_box();
                    if main_f.aabb(lo, hi) && (!tile.id.is_far() || tile.dist(eye2) < building_dist) {
                        place(&mut pass, &tile.xf);
                        draw_mesh(&mut pass, m);
                    }
                }
            }
            self.airport.draw_objects(&mut pass, &xfs, &resident, true, 20_000.0);
            if let Some((vb, ib, n)) = dynamic {
                place(&mut pass, &DYN);
                pass.set_vertex_buffer(0, vb.slice(..));
                pass.set_index_buffer(ib.slice(..), wgpu::IndexFormat::Uint32);
                pass.draw_indexed(0..n, 0, 0..1);
            }
            self.airport.draw_main(&mut pass);
            self.aircraft.draw_main(&mut pass);
            pass.set_pipeline(&self.tree_main);
            for tile in &tiles {
                let d = tile.dist(eye2);
                let (lo, hi) = tile.tree_box();
                if d < tree_dist && main_f.aabb(lo, hi) {
                    stats.tree_instances += draw_trees(&mut pass, &xfs, tile, tm, if d < tree_lod0 { 0 } else { 1 }, q.tree_density);
                }
            }
            // Lights, traffic and stars (additive, depth-tested).
            self.lights.draw_sprites(&mut pass, &tile_draws, world_xf, false, street > 0.0 || night_k > 0.0);
        }
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 4);
        }
        // Object motion for TAA / motion blur: the aircraft's velocity pass.
        if taa_on || q.motion_blur > 0 {
            let prev = self.cam_prev.map(|(m, _)| m).unwrap_or(vp_rot);
            let ac = &self.aircraft;
            self.post.velocity_pass(device, queue, encoder, width, height, vp_rot_j, vp_rot, prev, |pass, bg| ac.draw_velocity(pass, bg));
        }
        // SSAO from this frame's distances, used (reprojected) next frame.
        for (view, pipe, bg) in [(&t.ao_raw, &self.ao_pipe, &t.ao_bg), (&t.ao, &self.ao_blur_pipe, &t.ao_blur_bg)].into_iter().filter(|_| ao_on) {
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
                    inv_view_proj: vp_rot_j.inverse().to_cols_array_2d(),
                    eye: [fv.eye.x, fv.eye.y, fv.eye.z, wx.fog_ext],
                    fog: [wx.fog_top, cover, key_e.y * pre, haze],
                    sun: [key.x, key.y, key.z, width as f32],
                    screen: [width as f32, height as f32, 1.0 / width as f32, 1.0 / height as f32],
                    // (w: pre-exposure when the map is on.)
                    lmap: [globals.lmap[0], globals.lmap[1], globals.lmap[2], globals.lmap[3] * pre],
                    quality: [[4.0, 6.0, 8.0, 12.0][q.fog.min(3) as usize], [4.0, 6.0, 8.0, 12.0][q.fog.min(3) as usize], 0.0, 0.0],
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
                        wgpu::BindGroupEntry { binding: 5, resource: R::TextureView(self.lights.map_view()) },
                    ],
                });
                let mut pass = fullscreen_pass(encoder, "fog", &t.hdr);
                pass.set_pipeline(&wp.fog_pipe);
                pass.set_bind_group(0, &bg, &[]);
                pass.draw(0..3, 0..1);
            }
            // Effects (fire, smoke, contrails, vapour, haze) over the scene and clouds.
            if let Some(vf) = fv.vfx.as_deref() {
                self.vfx.render(device, queue, encoder, vf, fv.eye, fv.time, &t.main_bg, &t.hdr, &t.dist, self.clouds.history(0), key, HDR, width, height);
            }
            let precip = wx.rain.max(wx.snow);
            if precip > 0.0 {
                let snow = wx.snow > wx.rain;
                let focal = height as f32 * 0.5 / (fv.fov_y * 0.5).tan();
                let u = RainU {
                    view_proj: vp_rot_j.to_cols_array_2d(),
                    eye: [fv.eye.x, fv.eye.y, fv.eye.z, fv.time],
                    cam_vel: [wp.cam_vel.x, wp.cam_vel.y, wp.cam_vel.z, 1.0 / 60.0],
                    params: [precip, if snow { 1.0 } else { 0.0 }, wx.wind.x, wx.wind.y],
                    sun: [key.x, key.y, key.z, (1.0 - 0.9 * cover) * (key_e.y / atmosphere::SUN_E).min(1.0)],
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
        // Halos around the lights (haze, fog, rain) and the aircraft's light beams.
        {
            let murky = haze > 2.5 || wx.fog_ext > 0.0 || wx.rain > 0.0 || wx.snow > 0.0;
            let halos = murky && street > 0.0;
            let beams = n_spots > 0 && night_k > 0.0;
            self.lights.draw_post(encoder, &t.hdr, &tile_draws, world_xf, halos, beams);
        }
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 6);
        }

        // 4. Anti-aliasing (TAA / TAAU at the output size, or FXAA), then
        // the camera effects, all on the HDR image.
        let pre_ratio = if self.prev_pre > 0.0 { pre / self.prev_pre } else { 1.0 };
        self.prev_pre = pre;
        let scene_in = taa::SceneIn { dist: &t.dist, clouds: self.clouds.history(0) };
        let prev_eye = self.cam_prev.map(|(_, e)| e).unwrap_or(fv.eye);
        let cam_rel = self.cam_prev.map(|(m, e)| m * Mat4::from_translation(fv.eye - e)).unwrap_or(vp_rot);
        let tu = taa::TaaU {
            inv_vp: vp_rot.inverse().to_cols_array_2d(),
            prev_vp: cam_rel.to_cols_array_2d(),
            jitter: [jit.x, jit.y, if self.cam_prev.is_some() { 1.0 } else { 0.0 }, pre_ratio],
            in_size: [width as f32, height as f32, 1.0 / width as f32, 1.0 / height as f32],
            eye_delta: [fv.eye.x - prev_eye.x, fv.eye.y - prev_eye.y, fv.eye.z - prev_eye.z, if scene_in.clouds.is_some() { 1.0 } else { 0.0 }],
            // Blend: 1/16 of this frame when still (converges, ~16 jitter phases), up to 1/5 when moving fast.
            params: [0.0625, 0.2, 0.0, 0.0],
            ..Default::default()
        };
        let (mut src, post_w, post_h) = match q.aa {
            quality::AaMode::Taa => {
                if self.cam_prev.is_none() {
                    self.post.reset();
                }
                self.post.taa(device, queue, encoder, &t.hdr, &scene_in, (out_w, out_h), tu);
                (taa::Input::Taa, out_w, out_h)
            }
            quality::AaMode::Fxaa => {
                self.post.fxaa(device, queue, encoder, &t.hdr, &scene_in, width, height, 1.0);
                (taa::Input::Fxaa, width, height)
            }
            _ => (taa::Input::Ext(&t.hdr), width, height),
        };
        let mut slot = 0;
        if q.motion_blur > 0 && self.cam_prev.is_some() {
            // Shutter: half the frame interval (180°), at most 32 px.
            let u = taa::TaaU { params: [0.0, 0.0, 0.5, 0.0], focus: [0.0, 32.0 * post_w as f32 / 2560.0, if q.motion_blur >= 2 { 1.0 } else { 0.0 }, self.post.frame as f32], ..tu };
            self.post.effect(device, queue, encoder, 2, src, &scene_in, post_w, post_h, slot, u);
            src = taa::Input::Fx(slot);
            slot += 1;
        }
        if q.dof > 0 {
            // External: focus on the aircraft; cockpit: auto-focus on the centre of the view.
            let ac_dist = fv.aircraft.as_ref().and_then(|a| a.draws.first()).map(|d| d.model.w_axis.truncate().length());
            let cockpit = ac_dist.is_some_and(|d| d < 4.0);
            let on = if cockpit { q.dof & 1 != 0 } else { q.dof & 2 != 0 };
            if on {
                let focus = if cockpit { 0.0 } else { ac_dist.unwrap_or(0.0) };
                // Aperture: blur (px) per unit of normalised defocus.
                // (Subtle: the scenery at infinity blurs ~4 px at 1440p.)
                let aperture = if cockpit { 4.5 } else { 3.5 } * post_h as f32 / 1440.0;
                let u = taa::TaaU { params: [0.0, 0.0, aperture, 0.0], focus: [focus, 8.0 * post_h as f32 / 1440.0, 0.0, 0.0], ..tu };
                self.post.effect(device, queue, encoder, 3, src, &scene_in, post_w, post_h, slot, u);
                src = taa::Input::Fx(slot);
            }
        }
        if let Some(p) = self.prof.as_ref() {
            p.mark(encoder, 7);
        }

        // 5. Bloom chain (at the post-processing size).
        let bloom = self.bloom.as_ref().unwrap();
        // Post uniforms: [0..L) down, [L..2L) up, 2L tonemap.
        for i in 0..BLOOM_LEVELS {
            let (sw, sh) = if i == 0 { (post_w, post_h) } else { (bloom.levels[i - 1].1, bloom.levels[i - 1].2) };
            let u = PostU { params: [0.0, 0.0, if i == 0 { 1.0 } else { 0.0 }, 0.0], texel: [1.0 / sw as f32, 1.0 / sh as f32, 0.0, 0.0], ..bytemuck::Zeroable::zeroed() };
            queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * i as u64, bytemuck::bytes_of(&u));
        }
        for (k, i) in (1..BLOOM_LEVELS).rev().enumerate() {
            let (sw, sh) = (bloom.levels[i].1, bloom.levels[i].2);
            let u = PostU { texel: [1.0 / sw as f32, 1.0 / sh as f32, 0.0, 0.0], ..bytemuck::Zeroable::zeroed() };
            queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * (BLOOM_LEVELS + k) as u64, bytemuck::bytes_of(&u));
        }
        // Tonemap: exposure (× the player's EV), bloom strength, AgX look
        // power, saturation; lens / film effects; display brightness, gamma, HDR.
        // (A little more bloom at night: glare around the lights.)
        let v = &self.video;
        let ldr = self.ldr && self.tonemap_ldr.is_some();
        let hdr_out = self.output_format == wgpu::TextureFormat::Rgba16Float && v.hdr && !ldr;
        let sharpen = if matches!(q.aa, quality::AaMode::Taa) { q.sharpen } else { 0.0 };
        let tone = PostU {
            params: [fv.exposure * 2f32.powf(v.exposure_ev), (0.04 + 0.035 * night_k) * q.bloom, 1.22, 1.18],
            texel: [1.0 / post_w as f32, 1.0 / post_h as f32, 1.0 / pre, 0.0],
            fx: [sharpen, if q.lens_flare { 1.0 } else { 0.0 }, if q.lens_dirt { 1.0 } else { 0.0 }, q.chroma as f32],
            fx2: [q.grain, fv.time, v.brightness, v.gamma],
            hdr: [if hdr_out { 1.0 } else { 0.0 }, v.paper_white / 80.0, 1000.0 / v.paper_white, 0.0],
        };
        queue.write_buffer(&self.post_buf, UNIFORM_STRIDE * (2 * BLOOM_LEVELS) as u64, bytemuck::bytes_of(&tone));
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
        let src_view = self.post.view(src);
        let bloom_on = q.bloom > 0.0 || q.lens_flare || q.lens_dirt;
        if bloom_on {
            let down0 = self.post_bg(device, src_view, &self.dummy_refl);
            for i in 0..BLOOM_LEVELS {
                let bg = if i == 0 { &down0 } else { &bloom.down_bgs[i - 1] };
                fullscreen(encoder, "bloom down", &bloom.levels[i].0, clear, &self.bloom_down, bg, i);
            }
            for (k, i) in (1..BLOOM_LEVELS).rev().enumerate() {
                fullscreen(encoder, "bloom up", &bloom.levels[i - 1].0, wgpu::LoadOp::Load, &self.bloom_up, &bloom.up_bgs[k], BLOOM_LEVELS + k);
            }
        }
        // 6. Tonemap to output.
        let tonemap_bg = self.post_bg(device, src_view, if bloom_on { &bloom.levels[0].0 } else { &self.dummy_refl });
        let tonemap = if ldr { self.tonemap_ldr.as_ref().unwrap() } else { &self.tonemap };
        fullscreen(encoder, "tonemap", out, clear, tonemap, &tonemap_bg, 2 * BLOOM_LEVELS);
        if let Some(p) = self.prof.as_mut() {
            p.mark(encoder, 8);
            p.finish(encoder);
        }
        self.post.frame += 1;
        drop(tiles);
        xfs.flush(queue);
        self.prev_vp = Some((vp_rot, fv.eye));
        self.cam_prev = Some((vp_rot, fv.eye));
        self.stats = stats;
    }
}

/// Index ranges of a tile's ground mesh to draw: (base, draped), with the
/// buckets a finer resident tile covers left out (merged into runs).
fn ground_runs(t: &GpuTile, tiles: &HashMap<TileId, GpuTile>) -> (Vec<(u32, u32)>, Vec<(u32, u32)>) {
    let r = &t.ranges;
    let b = r.b.max(1);
    if r.base.len() < 2 || r.over.len() < 2 {
        return (Vec::new(), Vec::new());
    }
    let child_z = match crate::terrain::Level::of(t.id) {
        crate::terrain::Level::Near => None,
        crate::terrain::Level::Far => Some(crate::osm::ZOOM),
        crate::terrain::Level::Horizon => Some(crate::osm::FAR_ZOOM),
    };
    let covered = |k: u32| {
        child_z.is_some_and(|z| {
            let (i, j) = (k % b, k / b);
            tiles.contains_key(&TileId { z, x: t.id.x * b + i, y: t.id.y * b + j })
        })
    };
    let (mut base, mut over) = (Vec::new(), Vec::new());
    let mut k = 0;
    let n = b * b;
    while k < n {
        if covered(k) {
            k += 1;
            continue;
        }
        let s = k;
        while k < n && !covered(k) {
            k += 1;
        }
        let (b0, b1) = (r.base[s as usize], r.base[k as usize]);
        let (o0, o1) = (r.over[s as usize], r.over[k as usize]);
        if b1 > b0 {
            base.push((b0, b1));
        }
        if o1 > o0 {
            over.push((o0, o1));
        }
    }
    (base, over)
}

/// Fit a stable, texel-snapped orthographic light matrix around a slice of
/// the camera frustum. Returns the matrix and the covered radius (m).
/// Light matrix for one cascade of `view` (eye-relative: rotation only).
/// `anchor` (the world origin, eye-relative) fixes the texel snapping to the
/// world, so shadows don't crawl as the camera moves.
#[allow(clippy::too_many_arguments)]
fn cascade_matrix(view: Mat4, fov_y: f32, aspect: f32, near: f32, far: f32, sun: Vec3, anchor: glam::DVec3, res: u32) -> (Mat4, f32) {
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
    let half = res as f64 * 0.5;
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
    #[cfg(not(target_arch = "wasm32"))]
    pub fn new(surface: Option<&wgpu::Surface<'_>>, instance: wgpu::Instance) -> Result<Gpu, String> {
        pollster::block_on(Self::new_async(surface, instance))
    }

    pub async fn new_async(surface: Option<&wgpu::Surface<'_>>, instance: wgpu::Instance) -> Result<Gpu, String> {
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: surface,
                apply_limit_buckets: false,
            })
            .await
            .map_err(|e| format!("no GPU adapter: {e}"))?;
        // What the renderer needs beyond WebGPU's defaults, as far as the
        // adapter goes (browsers expose only what is asked for).
        let a = adapter.limits();
        let (device, queue) = adapter
            .request_device(&wgpu::DeviceDescriptor {
                label: Some("skyward"),
                // Timestamp queries only for the optional profiler.
                required_features: adapter.features() & (wgpu::Features::TIMESTAMP_QUERY | wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS),
                required_limits: wgpu::Limits {
                    max_texture_dimension_2d: a.max_texture_dimension_2d.min(8192),
                    max_buffer_size: a.max_buffer_size.min(1 << 30),
                    max_storage_buffer_binding_size: a.max_storage_buffer_binding_size.min(1 << 30),
                    ..wgpu::Limits::default()
                },
                ..Default::default()
            })
            .await
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
