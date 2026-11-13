//! Performance: GPU pass timing with timestamp queries (non-blocking ring
//! readback), the dynamic-resolution controller, the frame-rate limiter and
//! the in-game performance overlay (Ctrl+F3).

use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};

/// Timestamp marks written by the renderer, in order. Pass `i`'s time is
/// mark `i` − mark `i − 1`.
pub const MARKS: [&str; 9] = ["start", "atmosphere", "shadows", "reflection", "scene", "ao", "clouds+fx", "aa", "post"];

struct Readback {
    buf: wgpu::Buffer,
    /// Mapped and ready to read.
    ready: Arc<AtomicBool>,
    /// A copy into it was submitted and its map is pending.
    busy: bool,
}

/// GPU pass timings (needs `TIMESTAMP_QUERY_INSIDE_ENCODERS`: native only).
pub struct GpuProfiler {
    set: wgpu::QuerySet,
    resolve: wgpu::Buffer,
    ring: Vec<Readback>,
    period: f64,
    /// The readback the current frame copies into (None = this frame isn't timed).
    slot: Option<usize>,
    /// Latest per-pass times (ms), total first, and a smoothed copy.
    pub last: Vec<f32>,
    pub smooth: Vec<f32>,
    /// Every frame's times while benchmarking (`bench` = Some).
    pub bench: Option<Vec<Vec<f32>>>,
    /// `SKYWARD_GPU_TIMING=1`: print 120-frame averages.
    print: bool,
    sums: Vec<f64>,
    frames: u32,
}

impl GpuProfiler {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Option<GpuProfiler> {
        if !device.features().contains(wgpu::Features::TIMESTAMP_QUERY_INSIDE_ENCODERS) {
            return None;
        }
        let n = MARKS.len() as u32;
        let set = device.create_query_set(&wgpu::QuerySetDescriptor { label: Some("prof"), ty: wgpu::QueryType::Timestamp, count: n });
        let size = 8 * n as u64;
        let resolve = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("prof"),
            size,
            usage: wgpu::BufferUsages::QUERY_RESOLVE | wgpu::BufferUsages::COPY_SRC,
            mapped_at_creation: false,
        });
        let ring = (0..4)
            .map(|_| Readback {
                buf: device.create_buffer(&wgpu::BufferDescriptor {
                    label: Some("prof read"),
                    size,
                    usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                    mapped_at_creation: false,
                }),
                ready: Arc::new(AtomicBool::new(false)),
                busy: false,
            })
            .collect();
        Some(GpuProfiler {
            set,
            resolve,
            ring,
            period: queue.get_timestamp_period() as f64,
            slot: None,
            last: vec![0.0; MARKS.len()],
            smooth: vec![0.0; MARKS.len()],
            bench: None,
            print: std::env::var_os("SKYWARD_GPU_TIMING").is_some(),
            sums: vec![0.0; MARKS.len()],
            frames: 0,
        })
    }

    /// Start a frame: pick a free readback (none free → this frame isn't timed).
    pub fn begin(&mut self) {
        self.slot = self.ring.iter().position(|r| !r.busy);
    }

    pub fn mark(&self, encoder: &mut wgpu::CommandEncoder, i: usize) {
        if self.slot.is_some() {
            encoder.write_timestamp(&self.set, i as u32);
        }
    }

    /// After the last mark: resolve and copy into this frame's readback.
    pub fn finish(&mut self, encoder: &mut wgpu::CommandEncoder) {
        let Some(s) = self.slot else { return };
        let n = MARKS.len() as u32;
        encoder.resolve_query_set(&self.set, 0..n, &self.resolve, 0);
        encoder.copy_buffer_to_buffer(&self.resolve, 0, &self.ring[s].buf, 0, 8 * n as u64);
    }

    /// After the frame was submitted: request the map of this frame's
    /// readback, then read whatever earlier frames have finished. `wait`:
    /// block until this frame's times are in (benchmarks).
    pub fn collect(&mut self, device: &wgpu::Device, wait: bool) {
        if let Some(s) = self.slot.take() {
            let r = &mut self.ring[s];
            r.busy = true;
            r.ready.store(false, Ordering::Release);
            let flag = r.ready.clone();
            r.buf.slice(..).map_async(wgpu::MapMode::Read, move |res| flag.store(res.is_ok(), Ordering::Release));
        }
        let _ = device.poll(if wait { wgpu::PollType::wait_indefinitely() } else { wgpu::PollType::Poll });
        for i in 0..self.ring.len() {
            if !(self.ring[i].busy && self.ring[i].ready.load(Ordering::Acquire)) {
                continue;
            }
            let t: Vec<u64> = {
                let r = &self.ring[i];
                let data = r.buf.slice(..).get_mapped_range().expect("prof map");
                bytemuck::cast_slice::<u8, u64>(&data).to_vec()
            };
            self.ring[i].buf.unmap();
            self.ring[i].busy = false;
            self.record(&t);
        }
    }

    fn record(&mut self, t: &[u64]) {
        let ms = |a: u64, b: u64| (b.saturating_sub(a) as f64 * self.period * 1e-6) as f32;
        let n = t.len();
        let mut v = vec![0.0; n];
        v[0] = ms(t[0], t[n - 1]);
        for i in 1..n {
            v[i] = ms(t[i - 1], t[i]);
        }
        // (Garbage when a frame skipped marks: ignore implausible totals.)
        if !(0.0..1000.0).contains(&v[0]) {
            return;
        }
        for (s, x) in self.smooth.iter_mut().zip(&v) {
            *s = if *s == 0.0 { *x } else { *s * 0.9 + x * 0.1 };
        }
        if let Some(b) = self.bench.as_mut() {
            b.push(v.clone());
        }
        if self.print {
            for (s, x) in self.sums.iter_mut().zip(&v) {
                *s += *x as f64;
            }
            self.frames += 1;
            if self.frames == 120 {
                let f = self.frames as f64;
                let parts: Vec<String> = (1..n).map(|i| format!("{} {:.2}", MARKS[i], self.sums[i] / f)).collect();
                eprintln!("gpu ms: total {:.2} | {}", self.sums[0] / f, parts.join(", "));
                self.sums.iter_mut().for_each(|s| *s = 0.0);
                self.frames = 0;
            }
        }
        self.last = v;
    }

    /// Per-pass medians of the benchmark frames (total first).
    pub fn bench_medians(&self) -> Option<Vec<f32>> {
        let b = self.bench.as_ref().filter(|b| !b.is_empty())?;
        Some((0..MARKS.len()).map(|i| median(b.iter().map(|f| f[i]).collect())).collect())
    }
}

pub fn median(mut v: Vec<f32>) -> f32 {
    if v.is_empty() {
        return 0.0;
    }
    v.sort_by(|a, b| a.total_cmp(b));
    v[v.len() / 2]
}

// ---------------------------------------------------------------- dynamic resolution

/// Picks the render scale that holds a frame-time target: measured GPU (or
/// whole-frame) time over half-second windows; drops fast (GPU cost ∝ pixels
/// ∝ scale²), climbs back slowly, in 5 % steps (each change reallocates targets).
#[derive(Clone, Debug)]
pub struct DynRes {
    pub scale: f32,
    window: Vec<f32>,
    window_t: f32,
    since_change: f32,
}

impl DynRes {
    pub fn new(scale: f32) -> DynRes {
        DynRes { scale, window: Vec::new(), window_t: 0.0, since_change: 0.0 }
    }

    /// One frame: `ms` its GPU time (or frame time), `dt` its wall duration,
    /// `target_hz` the frame-rate goal, `max` the player's render scale.
    pub fn update(&mut self, ms: f32, dt: f32, target_hz: f32, max: f32) -> f32 {
        const MIN: f32 = 0.5;
        self.window.push(ms);
        self.window_t += dt;
        self.since_change += dt;
        self.scale = self.scale.min(max);
        if self.window_t < 0.5 || self.window.len() < 8 {
            return self.scale;
        }
        let mut w = std::mem::take(&mut self.window);
        self.window_t = 0.0;
        w.sort_by(|a, b| a.total_cmp(b));
        let p90 = w[(w.len() * 9 / 10).min(w.len() - 1)];
        let budget = 1000.0 / target_hz.max(1.0) * 0.9;
        let q = |s: f32| ((s / 0.05).round() * 0.05).clamp(MIN.min(max), max);
        let next = if p90 > budget {
            q((self.scale * (budget / p90).sqrt()).min(self.scale - 0.05))
        } else if p90 < budget * 0.72 && self.since_change > 1.5 {
            q(self.scale + 0.05)
        } else {
            self.scale
        };
        if next != self.scale {
            self.scale = next;
            self.since_change = 0.0;
        }
        self.scale
    }
}

// ---------------------------------------------------------------- frame limiter

/// Sleeps (then spins the last half millisecond) to hold a frame rate.
#[derive(Default)]
pub struct Limiter {
    next: Option<web_time::Instant>,
}

impl Limiter {
    /// Call once per frame before starting it; `hz` None = unlimited.
    pub fn wait(&mut self, hz: Option<f32>) {
        let Some(hz) = hz.filter(|h| *h > 1.0) else {
            self.next = None;
            return;
        };
        if crate::platform::WEB {
            return; // the browser paces frames (requestAnimationFrame)
        }
        let period = web_time::Duration::from_secs_f64(1.0 / hz as f64);
        let now = web_time::Instant::now();
        let Some(next) = self.next else {
            self.next = Some(now + period);
            return;
        };
        if next > now {
            let left = next - now;
            if left > web_time::Duration::from_micros(600) {
                std::thread::sleep(left - web_time::Duration::from_micros(500));
            }
            while web_time::Instant::now() < next {
                std::hint::spin_loop();
            }
        }
        // Next deadline; after a long frame don't try to catch up.
        let now = web_time::Instant::now();
        self.next = Some(if now > next + period { now + period } else { next + period });
    }
}

// ---------------------------------------------------------------- overlay

/// What the overlay shows (filled by main.rs and the renderer).
#[derive(Default)]
pub struct Stats {
    /// Recent frame times (ms), newest last.
    pub frames: VecDeque<f32>,
    /// CPU: (label, ms).
    pub cpu: Vec<(&'static str, f32)>,
    /// GPU: smoothed (label, ms), total first; empty when unsupported.
    pub gpu: Vec<(&'static str, f32)>,
    pub vram_mb: f32,
    pub scene: (u32, u32),
    pub output: (u32, u32),
    pub preset: String,
    pub aa: String,
    pub present: String,
}

impl Stats {
    pub fn push_frame(&mut self, ms: f32) {
        self.frames.push_back(ms);
        while self.frames.len() > 240 {
            self.frames.pop_front();
        }
    }
}

/// Draw the overlay (top-left, under any HUD).
pub fn overlay(ctx: &egui::Context, s: &Stats) {
    use crate::ui::{f_mono, f_semi, pal};
    use egui::{Align2, Color32, Pos2, Rect, Stroke, Vec2};
    let p = ctx.layer_painter(egui::LayerId::new(egui::Order::Tooltip, egui::Id::NULL.with("perf_overlay")));
    let sr = ctx.content_rect();
    let w = 330.0;
    let lines = 7 + s.cpu.len() + s.gpu.len();
    let h = 100.0 + lines as f32 * 15.0;
    // Right edge, below the moving map (the HUD panels are top-left).
    let r = Rect::from_min_size(Pos2::new(sr.max.x - w - 12.0, (sr.center().y - h * 0.5).max(sr.min.y + 12.0)), Vec2::new(w, h));
    p.rect_filled(r, 8, Color32::from_rgba_premultiplied(8, 10, 14, 220));
    let mut y = r.min.y + 10.0;
    let x = r.min.x + 12.0;
    let avg = if s.frames.is_empty() { 0.0 } else { s.frames.iter().sum::<f32>() / s.frames.len() as f32 };
    let worst = s.frames.iter().copied().fold(0.0, f32::max);
    let fps = if avg > 0.0 { 1000.0 / avg } else { 0.0 };
    let col = if fps >= 115.0 {
        pal::GREEN
    } else if fps >= 55.0 {
        pal::AMBER
    } else {
        pal::RED
    };
    p.text(Pos2::new(x, y), Align2::LEFT_TOP, format!("{fps:.0} FPS"), f_semi(18.0), col);
    p.text(Pos2::new(r.max.x - 12.0, y + 3.0), Align2::RIGHT_TOP, format!("{avg:.2} ms  max {worst:.1}"), f_mono(12.0), pal::TEXT2);
    y += 26.0;
    // Frame-time graph: 0..33 ms, lines at 60 and 120 fps.
    let g = Rect::from_min_size(Pos2::new(x, y), Vec2::new(w - 24.0, 44.0));
    p.rect_filled(g, 3, Color32::from_rgba_premultiplied(0, 0, 0, 120));
    let ms_y = |ms: f32| g.max.y - (ms / 33.3).clamp(0.0, 1.0) * g.height();
    for (ms, c) in [(16.67, pal::AMBER), (8.33, pal::GREEN)] {
        p.line_segment([Pos2::new(g.min.x, ms_y(ms)), Pos2::new(g.max.x, ms_y(ms))], Stroke::new(1.0, c.gamma_multiply(0.45)));
    }
    let n = s.frames.len();
    if n > 1 {
        let pts: Vec<Pos2> = s.frames.iter().enumerate().map(|(i, ms)| Pos2::new(g.min.x + g.width() * (i as f32 + 240.0 - n as f32) / 239.0, ms_y(*ms))).collect();
        p.add(egui::Shape::line(pts, Stroke::new(1.2, pal::ACCENT)));
    }
    y += 52.0;
    let row = |y: &mut f32, k: &str, v: String, c: Color32| {
        p.text(Pos2::new(x, *y), Align2::LEFT_TOP, k, f_mono(11.5), pal::TEXT2);
        p.text(Pos2::new(r.max.x - 12.0, *y), Align2::RIGHT_TOP, v, f_mono(11.5), c);
        *y += 15.0;
    };
    row(&mut y, "preset / AA", format!("{} · {}", s.preset, s.aa), pal::TEXT);
    let pct = if s.output.0 > 0 { 100.0 * s.scene.0 as f32 / s.output.0 as f32 } else { 100.0 };
    row(&mut y, "resolution", format!("{}×{} → {}×{} ({pct:.0} %)", s.scene.0, s.scene.1, s.output.0, s.output.1), pal::TEXT);
    row(&mut y, "present", s.present.clone(), pal::TEXT);
    row(&mut y, "VRAM (est.)", format!("{:.0} MB", s.vram_mb), pal::TEXT);
    y += 4.0;
    for (k, v) in &s.cpu {
        row(&mut y, &format!("cpu {k}"), format!("{v:.2} ms"), pal::TEXT);
    }
    y += 4.0;
    if s.gpu.is_empty() {
        row(&mut y, "gpu", "timestamps unavailable".into(), pal::TEXT3);
    }
    let total = s.gpu.first().map_or(1.0, |g| g.1.max(0.01));
    for (i, (k, v)) in s.gpu.iter().enumerate() {
        if i > 0 {
            // A bar: the pass's share of the GPU frame.
            let bw = (w - 24.0) * 0.32 * (v / total).clamp(0.0, 1.0);
            p.rect_filled(Rect::from_min_size(Pos2::new(x + 120.0, y + 3.0), Vec2::new(bw, 8.0)), 2, pal::ACCENT.gamma_multiply(0.6));
        }
        row(&mut y, &format!("gpu {k}"), format!("{v:.2} ms"), if i == 0 { pal::TEXT } else { pal::TEXT2 });
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A GPU whose frame cost is proportional to the pixel count: the
    /// controller settles under the budget without oscillating, and recovers
    /// when the load drops.
    #[test]
    fn dynres_converges_and_recovers() {
        let mut d = DynRes::new(1.0);
        let cost = |s: f32, load: f32| load * s * s; // ms
        let target = 120.0; // budget 7.5 ms
        let mut scales = Vec::new();
        for _ in 0..1200 {
            let s = d.scale;
            scales.push(d.update(cost(s, 14.0), 1.0 / 120.0, target, 1.0));
        }
        let last = *scales.last().unwrap();
        assert!(cost(last, 14.0) <= 7.5, "settled at {last}: {} ms", cost(last, 14.0));
        assert!(last >= 0.6, "dropped too far: {last}");
        // Stable at the end: no changes in the last 4 s.
        assert!(scales[scales.len() - 480..].iter().all(|s| *s == last));
        // Lighter scene: climbs back to the player's scale.
        for _ in 0..2400 {
            let s = d.scale;
            d.update(cost(s, 4.0), 1.0 / 120.0, target, 1.0);
        }
        assert_eq!(d.scale, 1.0);
        // Never above the player's choice, never below 50 %.
        for _ in 0..600 {
            let s = d.scale;
            d.update(cost(s, 500.0), 1.0 / 120.0, target, 0.8);
        }
        assert!((d.scale - 0.5).abs() < 1e-6);
    }

    #[test]
    fn medians() {
        assert_eq!(median(vec![3.0, 1.0, 2.0]), 2.0);
        assert_eq!(median(vec![]), 0.0);
    }
}
