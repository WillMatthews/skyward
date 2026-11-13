//! GPU side of the night lighting: light sprites (main + water-reflection
//! passes), haze / fog halos, moving traffic, the ground light map, the star
//! field and the aircraft's light beams. `gfx.rs` owns the frame and calls
//! the hooks below.

use super::place::{TileLights, TrafficSeg, AC_SPOTS};
use super::{kind, LightInst};
use crate::gfx::{DEPTH, DIST, HDR, MSAA};
use glam::{Mat3, Vec2, Vec3};
use wgpu::util::DeviceExt;

/// Ground light map: texels and the area it covers (m) around the eye.
pub const MAP_RES: u32 = 2048;
pub const MAP_SIZE: f32 = 8192.0;
/// Coverage grid of detailed tiles (z14 x, y modulo 128).
const COVER_N: u32 = 128;

/// One tile's light buffers.
pub struct GpuTileLights {
    lights: Option<(wgpu::Buffer, u32)>,
    /// Lights that also splat into the ground light map (prefix of `lights`).
    n_map: u32,
    traffic: Option<(wgpu::Buffer, u32)>,
    /// Far tiles: skyglow sources (tile-frame cell centres, lumen).
    pub glow: Vec<(Vec2, f32)>,
    pub bytes: u64,
}

impl GpuTileLights {
    /// Upload a tile's lights (map-splatting lights first).
    pub fn new(device: &wgpu::Device, t: &TileLights, rect: (Vec2, Vec2)) -> Option<GpuTileLights> {
        let mut l: Vec<LightInst> = Vec::with_capacity(t.lights.len());
        let map_kind = |x: &LightInst| matches!(x.kind(), kind::STREET | kind::FLOOD);
        l.extend(t.lights.iter().filter(|x| map_kind(x)));
        let n_map = l.len() as u32;
        l.extend(t.lights.iter().filter(|x| !map_kind(x)));
        let buf = |label: &str, bytes: &[u8]| device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some(label), contents: bytes, usage: wgpu::BufferUsages::VERTEX });
        let lights = (!l.is_empty()).then(|| (buf("tile lights", bytemuck::cast_slice(&l)), l.len() as u32));
        let traffic = (!t.traffic.is_empty()).then(|| (buf("tile traffic", bytemuck::cast_slice(&t.traffic)), t.traffic.len() as u32));
        let mut glow = Vec::new();
        if !t.glow.is_empty() {
            let n = super::place::GLOW_N;
            let (lo, hi) = rect;
            for (i, lm) in t.glow.iter().enumerate() {
                if *lm > 0.0 {
                    let c = Vec2::new((i % n) as f32 + 0.5, (i / n) as f32 + 0.5) / n as f32;
                    glow.push((lo + (hi - lo) * c, *lm));
                }
            }
        }
        if lights.is_none() && traffic.is_none() && glow.is_empty() {
            return None;
        }
        let bytes = lights.as_ref().map_or(0, |b| b.0.size()) + traffic.as_ref().map_or(0, |b| b.0.size());
        Some(GpuTileLights { lights, n_map, traffic, glow, bytes })
    }
}

/// A tile as the light passes see it this frame.
pub struct TileDraw<'a> {
    pub lights: &'a GpuTileLights,
    /// Tile frame → eye-relative transform (immediate data).
    pub xf: [f32; 4],
    /// Distance from the eye (m, horizontal) and whether it is in the view.
    pub dist: f32,
    pub visible: bool,
    pub visible_refl: bool,
}

#[repr(C)]
#[derive(Clone, Copy, bytemuck::Pod, bytemuck::Zeroable)]
struct StarInst {
    dir_e: [f32; 4],
    col: [f32; 4],
}

/// Approximate colour of a star from its B-V index (blackbody via Ballesteros' formula).
fn star_colour(bv: f32) -> [f32; 3] {
    let bv = bv.clamp(-0.4, 2.0);
    let t = 4600.0 * (1.0 / (0.92 * bv + 1.7) + 1.0 / (0.92 * bv + 0.62));
    // Planckian locus → linear sRGB (rough fit, normalised to the brightest channel).
    let t = t / 100.0;
    let r = if t <= 66.0 { 1.0 } else { (1.292936 * (t - 60.0).powf(-0.1332047)).min(1.0) };
    let g = if t <= 66.0 { (0.3900816 * t.ln() - 0.6318414).clamp(0.0, 1.0) } else { (1.1298909 * (t - 60.0).powf(-0.0755148)).min(1.0) };
    let b = if t >= 66.0 { 1.0 } else if t <= 19.0 { 0.0 } else { (0.5432068 * (t - 10.0).ln() - 1.1962541).clamp(0.0, 1.0) };
    // (sRGB-encoded) → linear, and less saturated: eyes see star colours weakly.
    let lin = |c: f32| c.powf(2.2);
    let (r, g, b) = (lin(r), lin(g), lin(b));
    let m = r.max(g).max(b);
    let w = 0.55;
    [r / m * w + (1.0 - w), g / m * w + (1.0 - w), b / m * w + (1.0 - w)]
}

fn load_stars() -> Vec<StarInst> {
    const DATA: &[u8] = include_bytes!("stars.bin");
    DATA.chunks_exact(6)
        .map(|c| {
            let ra = u16::from_le_bytes([c[0], c[1]]) as f32 / 65535.0 * std::f32::consts::TAU;
            let dec = i16::from_le_bytes([c[2], c[3]]) as f32 / 32767.0 * std::f32::consts::FRAC_PI_2;
            let mag = c[4] as f32 / 20.0 - 2.0;
            let bv = (c[5] as i8) as f32 / 50.0;
            // Illuminance of a star: 2.54e-6 lux at magnitude 0.
            let e = 2.54e-6 * 10f32.powf(-0.4 * mag) * crate::atmosphere::LUX;
            let colr = star_colour(bv);
            let seed = (ra * 13.7 + dec * 7.3).fract().abs();
            StarInst { dir_e: [dec.cos() * ra.cos(), dec.cos() * ra.sin(), dec.sin(), e], col: [colr[0], colr[1], colr[2], seed] }
        })
        .collect()
}

pub struct LightRenderer {
    bgl: wgpu::BindGroupLayout,
    /// Per-draw transforms (gfx.rs).
    xforms: std::sync::Arc<crate::gfx::Xforms>,
    sprite_main: wgpu::RenderPipeline,
    sprite_refl: wgpu::RenderPipeline,
    traffic_main: wgpu::RenderPipeline,
    traffic_refl: wgpu::RenderPipeline,
    star_main: wgpu::RenderPipeline,
    halo: wgpu::RenderPipeline,
    beams: wgpu::RenderPipeline,
    map_pipe: wgpu::RenderPipeline,
    map_view: wgpu::TextureView,
    cover_tex: wgpu::Texture,
    cover_view: wgpu::TextureView,
    cover: Vec<u8>,
    stars: (wgpu::Buffer, u32),
    airport: Option<(wgpu::Buffer, u32)>,
    airport_n_map: u32,
    dyn_buf: wgpu::Buffer,
    dyn_n: u32,
    dummy_dist: wgpu::TextureView,
    /// Bind groups for the main and the reflection pass (with a dist view for halos / beams).
    bg_main: Option<wgpu::BindGroup>,
    bg_refl: Option<wgpu::BindGroup>,
    /// After the main pass: reads the resolved eye-distance image.
    bg_post: Option<wgpu::BindGroup>,
    /// Skyglow: computed on a worker thread (~10 ms each), the latest
    /// result is used meanwhile.
    glow: GlowWorker,
}

type GlowTable = ([f32; crate::atmosphere::GLOW_AZ * crate::atmosphere::GLOW_EL], f32);

struct GlowJob {
    sources: Vec<(Vec2, f32)>,
    eye: Vec3,
    haze: f32,
    key: usize,
}

struct GlowWorker {
    tx: std::sync::mpsc::Sender<GlowJob>,
    /// Browser (no threads): jobs run between frames (platform::spawn) and answer here.
    #[cfg(target_arch = "wasm32")]
    done: std::sync::mpsc::Sender<(Vec3, usize, GlowTable)>,
    rx: std::sync::mpsc::Receiver<(Vec3, usize, GlowTable)>,
    pending: bool,
    /// (eye, sources key, table) of the latest result.
    latest: Option<(Vec3, usize, GlowTable)>,
    /// Recompute even if the eye hasn't moved (origin shift).
    stale: bool,
}

impl GlowWorker {
    fn new() -> GlowWorker {
        let (tx, jobs) = std::sync::mpsc::channel::<GlowJob>();
        let (done, rx) = std::sync::mpsc::channel();
        #[cfg(target_arch = "wasm32")]
        drop(jobs);
        #[cfg(not(target_arch = "wasm32"))]
        std::thread::Builder::new()
            .name("skyglow".into())
            .spawn(move || {
                while let Ok(mut j) = jobs.recv() {
                    // Only the newest request matters.
                    while let Ok(n) = jobs.try_recv() {
                        j = n;
                    }
                    let t = super::sky::skyglow(&j.sources, j.eye, j.haze);
                    if done.send((j.eye, j.key, t)).is_err() {
                        break;
                    }
                }
            })
            .expect("skyglow thread");
        GlowWorker {
            tx,
            #[cfg(target_arch = "wasm32")]
            done,
            rx,
            pending: false,
            latest: None,
            stale: false,
        }
    }
}

/// Resources of the renderer the light passes read.
pub struct Shared<'a> {
    pub globals: &'a wgpu::Buffer,
    pub refl_globals: &'a wgpu::Buffer,
    pub samp: &'a wgpu::Sampler,
    pub trans: &'a wgpu::TextureView,
    pub ap_main: &'a wgpu::TextureView,
    pub ap_refl: &'a wgpu::TextureView,
    pub atmos_out: &'a wgpu::Buffer,
    pub dist: &'a wgpu::TextureView,
}

impl LightRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue, scene_header: &str, xforms: std::sync::Arc<crate::gfx::Xforms>) -> Self {
        let src = format!("{scene_header}\n{}", include_str!("../shaders/lights.wgsl"));
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("lights"), source: wgpu::ShaderSource::Wgsl(src.into()) });
        let vf = wgpu::ShaderStages::VERTEX | wgpu::ShaderStages::FRAGMENT;
        let tex = |binding: u32, dim: wgpu::TextureViewDimension, filterable: bool| wgpu::BindGroupLayoutEntry {
            binding,
            visibility: vf,
            ty: wgpu::BindingType::Texture { sample_type: wgpu::TextureSampleType::Float { filterable }, view_dimension: dim, multisampled: false },
            count: None,
        };
        use wgpu::TextureViewDimension as D;
        let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("lights"),
            entries: &[
                wgpu::BindGroupLayoutEntry { binding: 0, visibility: vf, ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None }, count: None },
                wgpu::BindGroupLayoutEntry { binding: 4, visibility: vf, ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering), count: None },
                tex(5, D::D2, true),
                tex(7, D::D3, true),
                wgpu::BindGroupLayoutEntry {
                    binding: 9,
                    visibility: vf,
                    ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Storage { read_only: true }, has_dynamic_offset: false, min_binding_size: None },
                    count: None,
                },
                tex(12, D::D2, false),
                tex(13, D::D2, false),
            ],
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("lights"), bind_group_layouts: &[Some(&bgl), Some(&xforms.bgl)], immediate_size: 0 });
        let add = wgpu::BlendState {
            color: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
            alpha: wgpu::BlendComponent { src_factor: wgpu::BlendFactor::Zero, dst_factor: wgpu::BlendFactor::One, operation: wgpu::BlendOperation::Add },
        };
        const LIGHT_ATTRS: [wgpu::VertexAttribute; 6] = wgpu::vertex_attr_array![0 => Float32x3, 1 => Float32, 2 => Uint32, 3 => Uint32, 4 => Uint32, 5 => Uint32];
        const TRAFFIC_ATTRS: [wgpu::VertexAttribute; 3] = wgpu::vertex_attr_array![0 => Float32x4, 1 => Float32x4, 2 => Float32x4];
        const STAR_ATTRS: [wgpu::VertexAttribute; 2] = wgpu::vertex_attr_array![0 => Float32x4, 1 => Float32x4];
        let inst = |stride: usize, attrs: &'static [wgpu::VertexAttribute]| wgpu::VertexBufferLayout { array_stride: stride as u64, step_mode: wgpu::VertexStepMode::Instance, attributes: attrs };
        let light_vb = [Some(inst(std::mem::size_of::<LightInst>(), &LIGHT_ATTRS))];
        let traffic_vb = [Some(inst(std::mem::size_of::<TrafficSeg>(), &TRAFFIC_ATTRS))];
        let star_vb = [Some(inst(std::mem::size_of::<StarInst>(), &STAR_ATTRS))];
        // Scene-pass variant: HDR + DIST targets (dist untouched), depth-tested, no depth write.
        let scene_pipe = |label: &str, vs: &str, vb: &[Option<wgpu::VertexBufferLayout>], samples: u32| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers: vb },
                primitive: Default::default(),
                depth_stencil: Some(wgpu::DepthStencilState {
                    format: DEPTH,
                    depth_write_enabled: Some(false),
                    depth_compare: Some(wgpu::CompareFunction::GreaterEqual),
                    stencil: Default::default(),
                    bias: Default::default(),
                }),
                multisample: wgpu::MultisampleState { count: samples, mask: !0, alpha_to_coverage_enabled: false },
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some("fs_sprite"),
                    compilation_options: Default::default(),
                    targets: &[
                        Some(wgpu::ColorTargetState { format: HDR, blend: Some(add), write_mask: wgpu::ColorWrites::ALL }),
                        Some(wgpu::ColorTargetState { format: DIST, blend: None, write_mask: wgpu::ColorWrites::empty() }),
                    ],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let full_pipe = |label: &str, vs: &str, fs: &str, vb: &[Option<wgpu::VertexBufferLayout>], format: wgpu::TextureFormat| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
                layout: Some(&layout),
                vertex: wgpu::VertexState { module: &module, entry_point: Some(vs), compilation_options: Default::default(), buffers: vb },
                primitive: Default::default(),
                depth_stencil: None,
                multisample: Default::default(),
                fragment: Some(wgpu::FragmentState {
                    module: &module,
                    entry_point: Some(fs),
                    compilation_options: Default::default(),
                    targets: &[Some(wgpu::ColorTargetState { format, blend: Some(add), write_mask: wgpu::ColorWrites::ALL })],
                }),
                multiview_mask: None,
                cache: None,
            })
        };
        let sprite_main = scene_pipe("light sprites", "vs_sprite", &light_vb, MSAA);
        let sprite_refl = scene_pipe("light sprites refl", "vs_sprite", &light_vb, 1);
        let traffic_main = scene_pipe("traffic", "vs_traffic", &traffic_vb, MSAA);
        let traffic_refl = scene_pipe("traffic refl", "vs_traffic", &traffic_vb, 1);
        let star_main = scene_pipe("stars", "vs_star", &star_vb, MSAA);
        let halo = full_pipe("light halos", "vs_halo", "fs_halo", &light_vb, HDR);
        let beams = full_pipe("light beams", "vs_full", "fs_beams", &[], HDR);
        let map_pipe = full_pipe("light map", "vs_map", "fs_map", &light_vb, HDR);
        let map_view = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("light map"),
                size: wgpu::Extent3d { width: MAP_RES, height: MAP_RES, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: HDR,
                usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let cover_tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("light cover"),
            size: wgpu::Extent3d { width: COVER_N, height: COVER_N, depth_or_array_layers: 1 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::R8Unorm,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let cover_view = cover_tex.create_view(&Default::default());
        let dummy_dist = device
            .create_texture(&wgpu::TextureDescriptor {
                label: Some("dummy dist"),
                size: wgpu::Extent3d { width: 1, height: 1, depth_or_array_layers: 1 },
                mip_level_count: 1,
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format: DIST,
                usage: wgpu::TextureUsages::TEXTURE_BINDING,
                view_formats: &[],
            })
            .create_view(&Default::default());
        let stars = load_stars();
        let stars_buf = device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("stars"), contents: bytemuck::cast_slice(&stars), usage: wgpu::BufferUsages::VERTEX });
        let dyn_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("aircraft lights"),
            size: (std::mem::size_of::<LightInst>() * super::place::DYN_SPRITES) as u64,
            usage: wgpu::BufferUsages::VERTEX | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let r = LightRenderer {
            xforms,
            bgl,
            sprite_main,
            sprite_refl,
            traffic_main,
            traffic_refl,
            star_main,
            halo,
            beams,
            map_pipe,
            map_view,
            cover_tex,
            cover_view,
            cover: vec![0; (COVER_N * COVER_N) as usize],
            stars: (stars_buf, stars.len() as u32),
            airport: None,
            airport_n_map: 0,
            dyn_buf,
            dyn_n: 0,
            dummy_dist,
            bg_main: None,
            bg_refl: None,
            bg_post: None,
            glow: GlowWorker::new(),
        };
        r.upload_cover(queue);
        r
    }

    /// The ground light map (sampled by every lit surface).
    pub fn map_view(&self) -> &wgpu::TextureView {
        &self.map_view
    }

    /// (Re)build the bind groups (after the render targets change).
    pub fn bind(&mut self, device: &wgpu::Device, s: &Shared) {
        use wgpu::BindingResource as R;
        let mk = |globals: &wgpu::Buffer, ap: &wgpu::TextureView, dist: &wgpu::TextureView| {
            device.create_bind_group(&wgpu::BindGroupDescriptor {
                label: Some("lights"),
                layout: &self.bgl,
                entries: &[
                    wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 4, resource: R::Sampler(s.samp) },
                    wgpu::BindGroupEntry { binding: 5, resource: R::TextureView(s.trans) },
                    wgpu::BindGroupEntry { binding: 7, resource: R::TextureView(ap) },
                    wgpu::BindGroupEntry { binding: 9, resource: s.atmos_out.as_entire_binding() },
                    wgpu::BindGroupEntry { binding: 12, resource: R::TextureView(&self.cover_view) },
                    wgpu::BindGroupEntry { binding: 13, resource: R::TextureView(dist) },
                ],
            })
        };
        // (The main pass resolves into `dist`: it can't be bound there too.)
        self.bg_main = Some(mk(s.globals, s.ap_main, &self.dummy_dist));
        self.bg_refl = Some(mk(s.refl_globals, s.ap_refl, &self.dummy_dist));
        self.bg_post = Some(mk(s.globals, s.ap_main, s.dist));
    }

    fn upload_cover(&self, queue: &wgpu::Queue) {
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &self.cover_tex, mip_level: 0, origin: wgpu::Origin3d::ZERO, aspect: wgpu::TextureAspect::All },
            &self.cover,
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(COVER_N), rows_per_image: Some(COVER_N) },
            wgpu::Extent3d { width: COVER_N, height: COVER_N, depth_or_array_layers: 1 },
        );
    }

    /// Which detailed (z14) tiles are on the GPU: the aggregated far-city
    /// lights are hidden there.
    pub fn set_coverage(&mut self, queue: &wgpu::Queue, near: impl Iterator<Item = crate::osm::TileId>) {
        let mut c = vec![0u8; (COVER_N * COVER_N) as usize];
        for id in near {
            c[((id.y % COVER_N) * COVER_N + (id.x % COVER_N)) as usize] = 255;
        }
        if c != self.cover {
            self.cover = c;
            self.upload_cover(queue);
        }
    }

    /// Replace the airport lights (world frame).
    pub fn set_airport(&mut self, device: &wgpu::Device, lights: &[LightInst]) {
        // Apron floodlights also light the ground (light map): they go first.
        let map_kind = |x: &LightInst| matches!(x.kind(), kind::STREET | kind::FLOOD);
        let mut l: Vec<LightInst> = lights.iter().filter(|x| map_kind(x)).copied().collect();
        let n_map = l.len() as u32;
        l.extend(lights.iter().filter(|x| !map_kind(x)));
        self.airport = (!l.is_empty()).then(|| {
            (device.create_buffer_init(&wgpu::util::BufferInitDescriptor { label: Some("airport lights"), contents: bytemuck::cast_slice(&l), usage: wgpu::BufferUsages::VERTEX }), l.len() as u32)
        });
        self.airport_n_map = n_map;
    }

    /// This frame's aircraft light sprites (eye-relative).
    pub fn set_aircraft(&mut self, queue: &wgpu::Queue, sprites: &[LightInst]) {
        self.dyn_n = sprites.len().min(super::place::DYN_SPRITES) as u32;
        if self.dyn_n > 0 {
            queue.write_buffer(&self.dyn_buf, 0, bytemuck::cast_slice(&sprites[..self.dyn_n as usize]));
        }
    }

    /// Skyglow table + city upward radiance for the eye, from the far tiles'
    /// sources (cached; recomputed when the eye moves or the sources change).
    pub fn skyglow(&mut self, sources: &[(Vec2, f32)], eye: Vec3, haze: f32) -> ([f32; crate::atmosphere::GLOW_AZ * crate::atmosphere::GLOW_EL], f32) {
        let w = &mut self.glow;
        while let Ok(d) = w.rx.try_recv() {
            w.pending = false;
            w.latest = Some(d);
        }
        let key = sources.len() + (sources.iter().map(|s| s.1).sum::<f32>() * 1e-3) as usize;
        let fresh = |l: &(Vec3, usize, GlowTable)| {
            Vec2::new(l.0.x, l.0.z).distance(Vec2::new(eye.x, eye.z)) < 250.0 && (l.0.y - eye.y).abs() < 80.0 + 0.1 * eye.y && l.1 == key
        };
        match &w.latest {
            // First time: compute right away (a few ms, once).
            None => {
                let t = super::sky::skyglow(sources, eye, haze);
                w.latest = Some((eye, key, t));
            }
            Some(l) if (w.stale || !fresh(l)) && !w.pending => {
                w.stale = false;
                #[cfg(not(target_arch = "wasm32"))]
                {
                    w.pending = w.tx.send(GlowJob { sources: sources.to_vec(), eye, haze, key }).is_ok();
                }
                // Browser: in a tile worker (tiles_web.rs), else between frames here.
                #[cfg(target_arch = "wasm32")]
                {
                    let (done, sources) = (w.done.clone(), sources.to_vec());
                    let local = move |done: std::sync::mpsc::Sender<(Vec3, usize, GlowTable)>, sources: Vec<(Vec2, f32)>| {
                        crate::platform::spawn("skyglow", move || {
                            let _ = done.send((eye, key, super::sky::skyglow(&sources, eye, haze)));
                        })
                    };
                    let payload = crate::platform::wire::encode(&(sources.clone(), (eye, haze)));
                    let (d2, s2) = (done.clone(), sources.clone());
                    let sent = crate::platform::web::offload(
                        "skyglow",
                        payload,
                        Box::new(move |r| match r.and_then(|b| crate::platform::wire::decode::<GlowTable>(&b).ok()) {
                            Some(t) => {
                                let _ = d2.send((eye, key, t));
                            }
                            None => {
                                local(d2, s2);
                            }
                        }),
                    );
                    w.pending = sent || local(done, sources);
                }
            }
            _ => {}
        }
        w.latest.as_ref().map(|l| l.2).unwrap_or(([0.0; crate::atmosphere::GLOW_AZ * crate::atmosphere::GLOW_EL], 0.0))
    }

    /// The origin moved: refresh the skyglow (the old table stays in use meanwhile).
    pub fn rebase(&mut self) {
        self.glow.stale = true;
    }

    /// Splat street lights / floods into the ground light map (before the main pass).
    pub fn draw_map(&self, encoder: &mut wgpu::CommandEncoder, tiles: &[TileDraw], world_xf: [f32; 4]) {
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("light map"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: &self.map_view,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations { load: wgpu::LoadOp::Clear(wgpu::Color::BLACK), store: wgpu::StoreOp::Store },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        let Some(bg) = &self.bg_main else { return };
        pass.set_pipeline(&self.map_pipe);
        pass.set_bind_group(0, bg, &[]);
        for t in tiles {
            let Some((buf, _)) = &t.lights.lights else { continue };
            if t.lights.n_map == 0 || t.dist > MAP_SIZE * 0.5 + 200.0 {
                continue;
            }
            self.xforms.set(&mut pass, &t.xf);
            pass.set_vertex_buffer(0, buf.slice(..));
            pass.draw(0..6, 0..t.lights.n_map);
        }
        if let (Some((buf, _)), n) = (&self.airport, self.airport_n_map) {
            if n > 0 {
                self.xforms.set(&mut pass, &world_xf);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..6, 0..n);
            }
        }
    }

    /// Light sprites + traffic (+ stars in the main pass), inside a scene pass.
    /// `city`: the city's lights are on (dusk .. dawn) — else only the airport's
    /// and the aircraft's are drawn.
    pub fn draw_sprites(&self, pass: &mut wgpu::RenderPass<'_>, tiles: &[TileDraw], world_xf: [f32; 4], refl: bool, city: bool) {
        let Some(bg) = (if refl { &self.bg_refl } else { &self.bg_main }) else { return };
        pass.set_bind_group(0, bg, &[]);
        if !refl {
            pass.set_pipeline(&self.star_main);
            self.xforms.set(pass, &[0.0f32; 4]);
            pass.set_vertex_buffer(0, self.stars.0.slice(..));
            pass.draw(0..6, 0..self.stars.1);
        }
        pass.set_pipeline(if refl { &self.sprite_refl } else { &self.sprite_main });
        for t in tiles {
            if !city || !(if refl { t.visible_refl } else { t.visible }) {
                continue;
            }
            if let Some((buf, n)) = &t.lights.lights {
                self.xforms.set(pass, &t.xf);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..6, 0..*n);
            }
        }
        if let Some((buf, n)) = &self.airport {
            self.xforms.set(pass, &world_xf);
            pass.set_vertex_buffer(0, buf.slice(..));
            pass.draw(0..6, 0..*n);
        }
        if self.dyn_n > 0 {
            self.xforms.set(pass, &[0.0f32; 4]);
            pass.set_vertex_buffer(0, self.dyn_buf.slice(..));
            pass.draw(0..6, 0..self.dyn_n);
        }
        if !city {
            return;
        }
        pass.set_pipeline(if refl { &self.traffic_refl } else { &self.traffic_main });
        for t in tiles {
            if !(if refl { t.visible_refl } else { t.visible }) || t.dist > 6000.0 {
                continue;
            }
            if let Some((buf, n)) = &t.lights.traffic {
                self.xforms.set(pass, &t.xf);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..6 * 24, 0..*n);
            }
        }
    }

    /// Haze / fog halos around the lights and the aircraft's light beams,
    /// over the resolved HDR image (after clouds and fog).
    pub fn draw_post(&self, encoder: &mut wgpu::CommandEncoder, hdr: &wgpu::TextureView, tiles: &[TileDraw], world_xf: [f32; 4], halos: bool, beams: bool) {
        let Some(bg) = &self.bg_post else { return };
        if !halos && !beams {
            return;
        }
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("light halos"),
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
        });
        pass.set_bind_group(0, bg, &[]);
        if halos {
            pass.set_pipeline(&self.halo);
            for t in tiles {
                if !t.visible || t.dist > 6000.0 {
                    continue;
                }
                if let Some((buf, n)) = &t.lights.lights {
                    self.xforms.set(&mut pass, &t.xf);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..6, 0..*n);
                }
            }
            if let Some((buf, n)) = &self.airport {
                self.xforms.set(&mut pass, &world_xf);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..6, 0..*n);
            }
            if self.dyn_n > 0 {
                self.xforms.set(&mut pass, &[0.0f32; 4]);
                pass.set_vertex_buffer(0, self.dyn_buf.slice(..));
                pass.draw(0..6, 0..self.dyn_n);
            }
        }
        if beams {
            pass.set_pipeline(&self.beams);
            self.xforms.set(&mut pass, &[0.0f32; 4]);
            pass.draw(0..3, 0..1);
        }
    }
}

/// Analytic aircraft lights packed for `Globals::alights` (eye-relative):
/// per light [pos, range], [dir, cos outer], [rgb cd, cos inner].
pub fn pack_spots(spots: &[super::place::SpotLight]) -> ([[f32; 4]; 3 * AC_SPOTS], usize) {
    let mut out = [[0.0f32; 4]; 3 * AC_SPOTS];
    let n = spots.len().min(AC_SPOTS);
    for (i, s) in spots.iter().take(n).enumerate() {
        out[i * 3] = [s.pos.x, s.pos.y, s.pos.z, s.range];
        out[i * 3 + 1] = [s.dir.x, s.dir.y, s.dir.z, s.cos_outer];
        out[i * 3 + 2] = [s.cd.x, s.cd.y, s.cd.z, s.cos_inner];
    }
    (out, n)
}

/// Columns of the equatorial → world rotation as vec4s.
pub fn star_cols(m: Mat3) -> [[f32; 4]; 3] {
    [m.x_axis.extend(0.0).into(), m.y_axis.extend(0.0).into(), m.z_axis.extend(0.0).into()]
}

