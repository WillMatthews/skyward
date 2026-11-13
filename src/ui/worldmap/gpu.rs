//! GPU drawing of the world-map tiles inside the map's MSAA canvas (an egui
//! paint callback): every tile's fills and stroke meshes are uploaded once,
//! and a frame only writes a few bytes per tile (where it is, how big, how
//! faded) plus the zoom's colours and line widths — panning and zooming
//! cost the CPU nothing per vertex, at any detail.
//!
//! Vertex = tile-unit position + offset direction + class; the vertex
//! shader places it (`origin + pos · k + off · half-width[class][pass]`).
//! Passes per tile: fills (with the tile's own background), road casings,
//! lines, taxiway centrelines (drawn as instances 0 / 1 / 2: the instance
//! index picks the pass).

use super::decode::{L_TAXIWAY, MapTile, N_FILLS, N_LINES};
use crate::osm::TileId;
use egui::Vec2;
use std::collections::HashMap;
use std::ops::Range;
use std::sync::Arc;
use wgpu::util::DeviceExt;

/// Class slots: fills 0..16, the tile background 16, lines 17..32.
pub const C_BG: u32 = N_FILLS as u32;
pub const C_LINE0: u32 = C_BG + 1;
const SLOTS: usize = 32;
/// Per-tile uniform stride (min uniform offset alignment).
const STRIDE: u64 = 256;
/// GPU geometry budget (bytes) before unused tiles are dropped.
const GPU_CAP: usize = 192 << 20;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq, bytemuck::Pod, bytemuck::Zeroable)]
pub struct GVert {
    pub pos: [f32; 2],
    pub off: [f32; 2],
    pub cls: u32,
}

/// One tile's GPU geometry: vertices, indices, the index ranges of the
/// fills (with the background) and of the lines, and the taxiways' range.
#[derive(Debug, Default, PartialEq)]
pub struct Geometry {
    pub verts: Vec<GVert>,
    pub idx: Vec<u32>,
    pub fills: Range<u32>,
    pub lines: Range<u32>,
    pub taxi: Range<u32>,
}

/// Flatten a decoded tile into one vertex / index list.
pub fn geometry(t: &MapTile) -> Geometry {
    let mut g = Geometry::default();
    // Background quad (expanded by its "width" so neighbours overlap a hair).
    let corners = [([0.0, 0.0], [-1.0, -1.0]), ([1.0, 0.0], [1.0, -1.0]), ([1.0, 1.0], [1.0, 1.0]), ([0.0, 1.0], [-1.0, 1.0])];
    g.verts.extend(corners.iter().map(|(p, o)| GVert { pos: *p, off: *o, cls: C_BG }));
    g.idx.extend([0, 1, 2, 0, 2, 3]);
    for (c, v, i) in &t.fills {
        let b = g.verts.len() as u32;
        g.verts.extend(v.iter().map(|p| GVert { pos: *p, off: [0.0, 0.0], cls: *c as u32 }));
        g.idx.extend(i.iter().map(|x| b + x));
    }
    g.fills = 0..g.idx.len() as u32;
    let l0 = g.idx.len() as u32;
    for (c, s) in &t.lines {
        let b = g.verts.len() as u32;
        let i0 = g.idx.len() as u32;
        g.verts.extend(s.base.iter().zip(&s.off).map(|(p, o)| GVert { pos: *p, off: *o, cls: C_LINE0 + *c as u32 }));
        g.idx.extend(s.idx.iter().map(|x| b + x));
        if *c == L_TAXIWAY {
            g.taxi = i0..g.idx.len() as u32;
        }
    }
    g.lines = l0..g.idx.len() as u32;
    g
}

/// The zoom's look: half-widths [casing, line, centre] and colours per pass.
#[derive(Clone, Copy)]
pub struct Look {
    pub hw: [[f32; 4]; SLOTS],
    pub col: [[[f32; 4]; SLOTS]; 3],
}

impl Default for Look {
    fn default() -> Self {
        Look { hw: [[0.0; 4]; SLOTS], col: [[[0.0; 4]; SLOTS]; 3] }
    }
}

pub fn rgba(c: egui::Color32) -> [f32; 4] {
    // Premultiplied, gamma space: what egui's own shapes write into the canvas.
    [c.r() as f32 / 255.0, c.g() as f32 / 255.0, c.b() as f32 / 255.0, c.a() as f32 / 255.0]
}

/// One tile this frame.
pub struct Item {
    pub id: TileId,
    pub tile: Arc<MapTile>,
    pub origin: egui::Pos2,
    pub k: f32,
    pub alpha: f32,
}

/// The paint callback: this frame's tiles and look.
pub struct MapPaint {
    pub size: Vec2,
    pub items: Vec<Item>,
    pub look: Look,
    pub centre: bool,
}

struct TileGpu {
    vb: wgpu::Buffer,
    ib: wgpu::Buffer,
    fills: Range<u32>,
    lines: Range<u32>,
    taxi: Range<u32>,
    bytes: usize,
    /// Which decoded tile it came from (a reloaded tile re-uploads).
    src: usize,
    used: u64,
}

struct Res {
    pipe: wgpu::RenderPipeline,
    bgl: wgpu::BindGroupLayout,
    globals: wgpu::Buffer,
    tiles_buf: wgpu::Buffer,
    cap: u64,
    bind: wgpu::BindGroup,
    gpu: HashMap<TileId, TileGpu>,
    bytes: usize,
    frame: u64,
    /// This frame's uniform slot per item (paint order).
    order: Vec<TileId>,
}

const SHADER: &str = r#"
struct Globals { screen: vec4<f32>, hw: array<vec4<f32>, 32>, col: array<vec4<f32>, 96> };
struct Tile { o: vec2<f32>, k: f32, a: f32 };
@group(0) @binding(0) var<uniform> g: Globals;
@group(0) @binding(1) var<uniform> t: Tile;
struct VOut { @builtin(position) pos: vec4<f32>, @location(0) col: vec4<f32> };
@vertex fn vs(@location(0) p: vec2<f32>, @location(1) off: vec2<f32>, @location(2) cls: u32, @builtin(instance_index) ps: u32) -> VOut {
    let w = g.hw[cls][ps];
    let q = t.o + p * t.k + off * w;
    var o: VOut;
    o.pos = vec4<f32>(q.x / g.screen.x * 2.0 - 1.0, 1.0 - q.y / g.screen.y * 2.0, 0.0, 1.0);
    o.col = g.col[ps * 32u + cls] * t.a;
    return o;
}
@fragment fn fs(i: VOut) -> @location(0) vec4<f32> { return i.col; }
"#;

fn globals_bytes(size: Vec2, look: &Look) -> Vec<u8> {
    let mut v: Vec<f32> = vec![size.x.max(1.0), size.y.max(1.0), 0.0, 0.0];
    for h in &look.hw {
        v.extend_from_slice(h);
    }
    for pass in &look.col {
        for c in pass {
            v.extend_from_slice(c);
        }
    }
    bytemuck::cast_slice(&v).to_vec()
}

fn make_bind(device: &wgpu::Device, bgl: &wgpu::BindGroupLayout, globals: &wgpu::Buffer, tiles: &wgpu::Buffer) -> wgpu::BindGroup {
    device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("worldmap"),
        layout: bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: globals.as_entire_binding() },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding { buffer: tiles, offset: 0, size: wgpu::BufferSize::new(16) }) },
        ],
    })
}

fn create(device: &wgpu::Device) -> Res {
    let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("worldmap"), source: wgpu::ShaderSource::Wgsl(SHADER.into()) });
    let bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("worldmap"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: false, min_binding_size: None },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer { ty: wgpu::BufferBindingType::Uniform, has_dynamic_offset: true, min_binding_size: wgpu::BufferSize::new(16) },
                count: None,
            },
        ],
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor { label: Some("worldmap"), bind_group_layouts: &[Some(&bgl)], immediate_size: 0 });
    let attrs = wgpu::vertex_attr_array![0 => Float32x2, 1 => Float32x2, 2 => Uint32];
    let pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("worldmap"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &module,
            entry_point: Some("vs"),
            compilation_options: Default::default(),
            buffers: &[Some(wgpu::VertexBufferLayout { array_stride: std::mem::size_of::<GVert>() as u64, step_mode: wgpu::VertexStepMode::Vertex, attributes: &attrs })],
        },
        primitive: wgpu::PrimitiveState { cull_mode: None, ..Default::default() },
        depth_stencil: None,
        multisample: wgpu::MultisampleState { count: crate::ui::canvas::MSAA, mask: !0, alpha_to_coverage_enabled: false },
        fragment: Some(wgpu::FragmentState {
            module: &module,
            entry_point: Some("fs"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format: crate::ui::canvas::FORMAT,
                blend: Some(wgpu::BlendState::PREMULTIPLIED_ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        multiview_mask: None,
        cache: None,
    });
    let globals = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("worldmap globals"),
        size: (16 + SLOTS * 16 + 3 * SLOTS * 16) as u64,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let cap = 64;
    let tiles_buf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("worldmap tiles"),
        size: cap * STRIDE,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let bind = make_bind(device, &bgl, &globals, &tiles_buf);
    Res { pipe, bgl, globals, tiles_buf, cap, bind, gpu: HashMap::new(), bytes: 0, frame: 0, order: Vec::new() }
}

impl egui_wgpu::CallbackTrait for MapPaint {
    fn prepare(
        &self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        _sd: &egui_wgpu::ScreenDescriptor,
        _enc: &mut wgpu::CommandEncoder,
        res: &mut egui_wgpu::CallbackResources,
    ) -> Vec<wgpu::CommandBuffer> {
        if res.get::<Res>().is_none() {
            res.insert(create(device));
        }
        let Some(r) = res.get_mut::<Res>() else { return Vec::new() };
        r.frame += 1;
        queue.write_buffer(&r.globals, 0, &globals_bytes(self.size, &self.look));
        // Upload new tiles.
        for it in &self.items {
            let src = Arc::as_ptr(&it.tile) as usize;
            if r.gpu.get(&it.id).is_some_and(|g| g.src == src) {
                continue;
            }
            let g = geometry(&it.tile);
            let vb = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("worldmap vb"),
                contents: bytemuck::cast_slice(&g.verts),
                usage: wgpu::BufferUsages::VERTEX,
            });
            let ib = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("worldmap ib"),
                contents: bytemuck::cast_slice(&g.idx),
                usage: wgpu::BufferUsages::INDEX,
            });
            let bytes = g.verts.len() * std::mem::size_of::<GVert>() + g.idx.len() * 4;
            r.bytes += bytes;
            if let Some(old) = r.gpu.insert(it.id, TileGpu { vb, ib, fills: g.fills, lines: g.lines, taxi: g.taxi, bytes, src, used: r.frame }) {
                r.bytes -= old.bytes;
            }
        }
        // Per-tile placement.
        let n = self.items.len() as u64;
        if n > r.cap {
            r.cap = n.next_power_of_two();
            r.tiles_buf = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("worldmap tiles"),
                size: r.cap * STRIDE,
                usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
                mapped_at_creation: false,
            });
            r.bind = make_bind(device, &r.bgl, &r.globals, &r.tiles_buf);
        }
        let mut data = vec![0u8; (n * STRIDE) as usize];
        r.order.clear();
        for (i, it) in self.items.iter().enumerate() {
            let u = [it.origin.x, it.origin.y, it.k, it.alpha];
            data[i * STRIDE as usize..i * STRIDE as usize + 16].copy_from_slice(bytemuck::cast_slice(&u));
            r.order.push(it.id);
            if let Some(g) = r.gpu.get_mut(&it.id) {
                g.used = r.frame;
            }
        }
        if !data.is_empty() {
            queue.write_buffer(&r.tiles_buf, 0, &data);
        }
        // Drop geometry not drawn lately once over budget.
        if r.bytes > GPU_CAP {
            let frame = r.frame;
            let mut old: Vec<(u64, TileId, usize)> = r.gpu.iter().filter(|(_, g)| g.used + 2 < frame).map(|(k, g)| (g.used, *k, g.bytes)).collect();
            old.sort_by_key(|o| (o.0, o.1));
            for (_, k, b) in old {
                if r.bytes <= GPU_CAP {
                    break;
                }
                r.gpu.remove(&k);
                r.bytes -= b;
            }
        }
        Vec::new()
    }

    fn paint(&self, _info: egui::PaintCallbackInfo, pass: &mut wgpu::RenderPass<'static>, res: &egui_wgpu::CallbackResources) {
        let Some(r) = res.get::<Res>() else { return };
        pass.set_pipeline(&r.pipe);
        for (i, id) in r.order.iter().enumerate() {
            let Some(g) = r.gpu.get(id) else { continue };
            pass.set_bind_group(0, &r.bind, &[(i as u64 * STRIDE) as u32]);
            pass.set_vertex_buffer(0, g.vb.slice(..));
            pass.set_index_buffer(g.ib.slice(..), wgpu::IndexFormat::Uint32);
            pass.draw_indexed(g.fills.clone(), 0, 1..2);
            if !g.lines.is_empty() {
                pass.draw_indexed(g.lines.clone(), 0, 0..1);
                pass.draw_indexed(g.lines.clone(), 0, 1..2);
                if self.centre && !g.taxi.is_empty() {
                    pass.draw_indexed(g.taxi.clone(), 0, 2..3);
                }
            }
        }
    }
}

/// Line slot of a line class.
pub fn line_slot(class: u8) -> usize {
    C_LINE0 as usize + class as usize
}

const _: () = assert!(C_LINE0 as usize + N_LINES <= SLOTS);

#[cfg(test)]
mod tests {
    use super::super::decode::{F_WATER, L_MOTORWAY, Strokes};
    use super::*;

    #[test]
    fn geometry_ranges() {
        let mut s = Strokes::default();
        s.add(&[[0.1, 0.1], [0.9, 0.1]]);
        let mut tx = Strokes::default();
        tx.add(&[[0.2, 0.5], [0.8, 0.5], [0.8, 0.9]]);
        let t = MapTile { fills: vec![(F_WATER, vec![[0.0, 0.0], [1.0, 0.0], [0.0, 1.0]], vec![0, 1, 2])], lines: vec![(L_MOTORWAY, s), (L_TAXIWAY, tx)], labels: Vec::new() };
        let g = geometry(&t);
        // background quad + water triangle, then the two line classes
        assert_eq!(g.fills, 0..9);
        assert_eq!(g.lines, 9..9 + 6 + 18);
        assert_eq!(g.taxi, 15..33);
        assert_eq!(g.verts[0].cls, C_BG);
        assert_eq!(g.verts[4].cls, F_WATER as u32);
        assert_eq!(g.verts[7].cls, line_slot(L_MOTORWAY) as u32);
        assert!(g.idx.iter().all(|&i| (i as usize) < g.verts.len()));
    }
}
