//! Settings › Video: the quality preset, display options and every
//! rendering knob (gfx/quality.rs), grouped, each with a one-line
//! explanation and its GPU cost. Changes apply live (the renderer reads the
//! settings every frame) except the few marked "restart".
//!
//! The settings shell calls [`video_tab`] inside its scrolling page.

use super::menus::segmented;
use super::settings::{DETAIL_NAMES, Settings};
use super::{f_mono, f_semi, f_ui, pal, sp};
use crate::gfx::quality::{self as gq, AaMode, FrameCap, Knob, Present, Preset, RestartReason, VideoEnv, WindowMode};
use egui::{Align2, Color32, Pos2, Rect, RichText, Sense, Ui, UiBuilder, Vec2};

/// Row height and label column width.
const ROW: f32 = 54.0;
const LABEL_W: f32 = 230.0;
const COST_W: f32 = 74.0;

struct Layout {
    x: f32,
    w: f32,
    y: f32,
}

impl Layout {
    fn ctrl(&self) -> Rect {
        Rect::from_min_size(Pos2::new(self.x + LABEL_W, self.y + 6.0), Vec2::new(self.w - LABEL_W - COST_W, 34.0))
    }
}

/// Draw the Video tab at `origin` (`width` wide) for `s` (writes
/// `s.video`, and keeps the legacy `s.detail` / `s.vsync` in step).
/// Returns the height used.
pub fn video_tab(ui: &mut Ui, origin: Pos2, width: f32, s: &mut Settings, env: &VideoEnv) -> f32 {
    let p = ui.painter().clone();
    let mut l = Layout { x: origin.x, w: width - super::scroll::GUTTER, y: origin.y };
    let v0 = s.video.clone();
    let px = (env.output.0.max(1) * env.output.1.max(1)) as f32;
    let running = env.running.unwrap_or(s.video.q);
    let restart = running.restart_diff(&s.video.q);

    let label = |p: &egui::Painter, l: &Layout, k: &str, hint: &str| {
        p.text(Pos2::new(l.x, l.y + 7.0), Align2::LEFT_TOP, k, f_semi(14.0), pal::TEXT);
        if !hint.is_empty() {
            p.text(Pos2::new(l.x, l.y + 27.0), Align2::LEFT_TOP, hint, f_ui(11.5), pal::TEXT3);
        }
    };
    let cost = |p: &egui::Painter, l: &Layout, k: Knob, q: &gq::Quality| {
        let ms = gq::cost_ms(k, q, px);
        p.text(Pos2::new(l.x + l.w, l.y + 23.0), Align2::RIGHT_CENTER, gq::cost_label(ms), f_mono(11.5), if ms >= 1.0 { pal::AMBER } else { pal::TEXT3 });
    };
    let note = |p: &egui::Painter, l: &Layout, text: &str, col: Color32| {
        p.text(Pos2::new(l.x + l.w, l.y + 23.0), Align2::RIGHT_CENTER, text, f_mono(11.0), col);
    };
    let header = |p: &egui::Painter, l: &mut Layout, title: &str| {
        l.y += 10.0;
        p.text(Pos2::new(l.x, l.y), Align2::LEFT_TOP, title.to_uppercase(), f_semi(12.0), pal::ACCENT);
        p.line_segment([Pos2::new(l.x, l.y + 20.0), Pos2::new(l.x + l.w, l.y + 20.0)], egui::Stroke::new(1.0, Color32::from_white_alpha(14)));
        l.y += 28.0;
    };

    // ---- preset
    {
        let v = &mut s.video;
        label(
            &p,
            &l,
            "Quality preset",
            &match env.detected {
                Some(d) => format!("Recommended for this GPU: {}", d.name()),
                None => "Sets every option below".into(),
            },
        );
        let names: Vec<&str> = Preset::ALL.iter().map(|p| p.name()).collect();
        let cur = Preset::ALL.iter().position(|x| *x == v.preset).unwrap_or(2);
        let r = l.ctrl();
        if let Some(i) = segmented(ui, Rect::from_min_size(r.min, Vec2::new(r.width() + COST_W - 132.0, r.height())), "vpreset", &names, cur)
            && Preset::ALL[i] != Preset::Custom
        {
            v.apply_preset(Preset::ALL[i]);
        }
        let rb = Rect::from_min_size(Pos2::new(l.x + l.w - 122.0, r.min.y), Vec2::new(122.0, 34.0));
        if super::menus::button(ui, rb, "vreset", "Reset to preset", super::menus::Btn::Ghost, true).clicked() {
            v.reset_to_preset();
        }
        l.y += ROW;
    }

    // ---- display
    header(&p, &mut l, "Display");
    let web = crate::platform::WEB;
    if !web {
        let v = &mut s.video;
        label(&p, &l, "Window mode", "Fullscreen uses a video mode");
        let names: Vec<&str> = WindowMode::ALL.iter().map(|m| m.name()).collect();
        if let Some(i) = segmented(ui, l.ctrl(), "vwin", &names, WindowMode::ALL.iter().position(|m| *m == v.window).unwrap_or(0)) {
            v.window = WindowMode::ALL[i];
        }
        l.y += ROW;
        if env.monitors.len() > 1 {
            label(&p, &l, "Monitor", "Where fullscreen / borderless goes");
            let cur = v.monitor.as_ref().and_then(|m| env.monitors.iter().position(|x| x == m)).unwrap_or(0);
            let names: Vec<&str> = env.monitors.iter().map(|m| m.as_str()).collect();
            if let Some(i) = segmented(ui, l.ctrl(), "vmon", &names, cur) {
                v.monitor = Some(env.monitors[i].clone());
            }
            l.y += ROW;
        }
        if v.window == WindowMode::Exclusive && !env.modes.is_empty() {
            label(&p, &l, "Resolution", "Exclusive fullscreen video mode");
            let cur = v.mode.and_then(|m| env.modes.iter().position(|x| *x == m)).unwrap_or(0);
            let text = |m: &(u32, u32, u32)| format!("{} × {}  ·  {:.0} Hz", m.0, m.1, m.2 as f32 / 1000.0);
            if let Some(i) = stepper(ui, l.ctrl(), "vmode", &text(&env.modes[cur.min(env.modes.len() - 1)]), cur, env.modes.len()) {
                v.mode = Some(env.modes[i]);
            }
            l.y += ROW;
        }
    }
    {
        let v = &mut s.video;
        label(&p, &l, "Vertical sync", "Fast sync: low latency, no tearing");
        let names: Vec<&str> = Present::ALL.iter().map(|m| m.name()).collect();
        if let Some(i) = segmented(ui, l.ctrl(), "vsync", &names, Present::ALL.iter().position(|m| *m == v.present).unwrap_or(0)) {
            v.present = Present::ALL[i];
        }
        if !env.present_modes.is_empty() && !env.present_modes.contains(&v.present.wgpu()) {
            note(&p, &l, "fallback", pal::AMBER);
        }
        l.y += ROW;
        label(&p, &l, "Frame-rate limit", "Dynamic resolution aims for it");
        let names: Vec<String> = FrameCap::CHOICES.iter().map(|c| c.name()).collect();
        let refs: Vec<&str> = names.iter().map(|s| s.as_str()).collect();
        if let Some(i) = segmented(ui, l.ctrl(), "vcap", &refs, FrameCap::CHOICES.iter().position(|c| *c == v.frame_cap).unwrap_or(0)) {
            v.frame_cap = FrameCap::CHOICES[i];
        }
        l.y += ROW;
        if env.hdr_supported {
            label(&p, &l, "HDR output", "HDR displays (experimental)");
            let mut on = v.hdr;
            if switch(ui, l.ctrl(), "vhdr", &mut on) {
                v.hdr = on;
            }
            if v.hdr != env.running_hdr {
                note(&p, &l, "restart", pal::AMBER);
            }
            l.y += ROW;
            if v.hdr {
                label(&p, &l, "Paper white", "White level in nits");
                slider(ui, l.ctrl(), &mut v.paper_white, 80.0..=400.0, 10.0, |x| format!("{x:.0} nits"));
                l.y += ROW;
            }
        }
    }
    {
        let q = &mut s.video.q;
        label(&p, &l, "Render scale", "< 100 %: TAA upscales");
        let mut pct = q.render_scale * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 50.0..=200.0, 5.0, |x| format!("{x:.0} %")) {
            q.render_scale = pct / 100.0;
        }
        cost(&p, &l, Knob::RenderScale, q);
        l.y += ROW;
        label(&p, &l, "Dynamic resolution", "Holds the frame rate");
        switch(ui, l.ctrl(), "vdyn", &mut q.dynamic_res);
        if let (true, Some(sc)) = (q.dynamic_res, env.dyn_scale) {
            note(&p, &l, &format!("now {:.0} %", sc * 100.0), pal::TEXT3);
        }
        l.y += ROW;
    }

    // ---- image quality
    header(&p, &mut l, "Image quality");
    {
        let q = &mut s.video.q;
        label(&p, &l, "Anti-aliasing", "TAA: best; FXAA: cheapest");
        let names: Vec<&str> = AaMode::ALL.iter().map(|m| m.name()).collect();
        if let Some(i) = segmented(ui, l.ctrl(), "vaa", &names, AaMode::ALL.iter().position(|m| *m == q.aa).unwrap_or(3)) {
            q.aa = AaMode::ALL[i];
        }
        if restart.contains(&RestartReason::Msaa) {
            note(&p, &l, "restart", pal::AMBER);
        } else {
            cost(&p, &l, Knob::Aa, q);
        }
        l.y += ROW;
        if q.aa == AaMode::Taa {
            label(&p, &l, "Sharpening", "Restores detail after TAA");
            let mut pct = q.sharpen * 100.0;
            if slider(ui, l.ctrl(), &mut pct, 0.0..=100.0, 5.0, |x| format!("{x:.0} %")) {
                q.sharpen = pct / 100.0;
            }
            l.y += ROW;
        }
        label(&p, &l, "Anisotropic filtering", "Sharp textures at grazing angles");
        let opts = [1u8, 2, 4, 8, 16];
        if let Some(i) = segmented(ui, l.ctrl(), "vaniso", &["1×", "2×", "4×", "8×", "16×"], opts.iter().position(|a| *a == q.aniso).unwrap_or(3)) {
            q.aniso = opts[i];
        }
        if restart.contains(&RestartReason::Textures) && running.aniso != q.aniso {
            note(&p, &l, "restart", pal::AMBER);
        } else {
            cost(&p, &l, Knob::Aniso, q);
        }
        l.y += ROW;
        label(&p, &l, "Texture detail", "Aircraft and airport textures");
        if let Some(i) = segmented(ui, l.ctrl(), "vtex", &["Low", "Medium", "High"], q.textures as usize) {
            q.textures = i as u8;
        }
        if running.textures != q.textures {
            note(&p, &l, "restart", pal::AMBER);
        }
        l.y += ROW;
    }

    // ---- lighting & shadows
    header(&p, &mut l, "Lighting & shadows");
    {
        let q = &mut s.video.q;
        label(&p, &l, "Shadow resolution", "Sharpness of shadows");
        let opts = [1024u32, 2048, 4096];
        let names: &[&str] = if web { &["1024", "2048"] } else { &["1024", "2048", "4096"] };
        if let Some(i) = segmented(ui, l.ctrl(), "vshres", names, opts.iter().position(|a| *a == q.shadow_res).unwrap_or(2).min(names.len() - 1)) {
            q.shadow_res = opts[i];
        }
        cost(&p, &l, Knob::Shadows, q);
        l.y += ROW;
        label(&p, &l, "Shadow cascades", "More: crisper near and far");
        if let Some(i) = segmented(ui, l.ctrl(), "vcasc", &["2", "3", "4"], (q.cascades as usize).clamp(2, 4) - 2) {
            q.cascades = i as u8 + 2;
        }
        cost(&p, &l, Knob::Cascades, q);
        l.y += ROW;
        label(&p, &l, "Shadow distance", "How far out shadows are drawn");
        let mut pct = q.shadow_dist * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 50.0..=150.0, 5.0, |x| format!("{x:.0} %")) {
            q.shadow_dist = pct / 100.0;
        }
        cost(&p, &l, Knob::ShadowDist, q);
        l.y += ROW;
        label(&p, &l, "Soft shadows", "Filtered shadow edges");
        switch(ui, l.ctrl(), "vsoft", &mut q.soft_shadows);
        cost(&p, &l, Knob::SoftShadows, q);
        l.y += ROW;
        label(&p, &l, "Ambient occlusion", "Contact shading");
        if let Some(i) = segmented(ui, l.ctrl(), "vao", &["Off", "Low", "Medium", "High"], q.ao as usize) {
            q.ao = i as u8;
        }
        cost(&p, &l, Knob::Ao, q);
        l.y += ROW;
        label(&p, &l, "Clouds", "Volumetric cloud detail");
        if let Some(i) = segmented(ui, l.ctrl(), "vclouds", &["Low", "Medium", "High", "Ultra"], q.clouds as usize) {
            q.clouds = i as u8;
        }
        cost(&p, &l, Knob::Clouds, q);
        l.y += ROW;
        label(&p, &l, "Atmosphere & fog", "Ground fog and mist detail");
        if let Some(i) = segmented(ui, l.ctrl(), "vfog", &["Low", "Medium", "High", "Ultra"], q.fog as usize) {
            q.fog = i as u8;
        }
        cost(&p, &l, Knob::Fog, q);
        l.y += ROW;
        label(&p, &l, "Water reflections", "Mirrored scene resolution");
        if let Some(i) = segmented(ui, l.ctrl(), "vrefl", &["Quarter", "Half", "Full"], q.reflections as usize) {
            q.reflections = i as u8;
        }
        cost(&p, &l, Knob::Reflections, q);
        l.y += ROW;
    }

    // ---- world detail
    header(&p, &mut l, "World detail");
    {
        let q = &mut s.video.q;
        label(&p, &l, "Scenery detail", "Detailed scenery radius");
        if let Some(i) = segmented(ui, l.ctrl(), "vdetail", &DETAIL_NAMES, q.detail as usize) {
            q.detail = i as u8;
        }
        cost(&p, &l, Knob::Detail, q);
        l.y += ROW;
        label(&p, &l, "Tree density", "Share of the trees drawn");
        let mut pct = q.tree_density * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 25.0..=100.0, 5.0, |x| format!("{x:.0} %")) {
            q.tree_density = pct / 100.0;
        }
        cost(&p, &l, Knob::Trees, q);
        l.y += ROW;
        label(&p, &l, "Tree distance", "How far out trees are drawn");
        let mut pct = q.tree_dist * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 50.0..=150.0, 5.0, |x| format!("{x:.0} %")) {
            q.tree_dist = pct / 100.0;
        }
        l.y += ROW;
        label(&p, &l, "Building distance", "Far-field buildings");
        let mut pct = q.building_dist * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 50.0..=100.0, 5.0, |x| format!("{x:.0} %")) {
            q.building_dist = pct / 100.0;
        }
        cost(&p, &l, Knob::Buildings, q);
        l.y += ROW;
    }

    // ---- effects
    header(&p, &mut l, "Effects");
    {
        let q = &mut s.video.q;
        label(&p, &l, "Effects quality", "Vapour, haze, smoke");
        if let Some(i) = segmented(ui, l.ctrl(), "vvfx", &["Low", "Medium", "High"], q.vfx as usize) {
            q.vfx = i as u8;
        }
        cost(&p, &l, Knob::Vfx, q);
        l.y += ROW;
        label(&p, &l, "Bloom", "Glow around bright lights");
        let mut pct = q.bloom * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 0.0..=150.0, 5.0, |x| if x <= 0.0 { "Off".into() } else { format!("{x:.0} %") }) {
            q.bloom = pct / 100.0;
        }
        cost(&p, &l, Knob::Bloom, q);
        l.y += ROW;
        label(&p, &l, "Lens flare", "Ghosts and a halo from the sun");
        switch(ui, l.ctrl(), "vflare", &mut q.lens_flare);
        cost(&p, &l, Knob::LensFlare, q);
        l.y += ROW;
        label(&p, &l, "Lens dirt", "Smudges lit by bright light");
        switch(ui, l.ctrl(), "vdirt", &mut q.lens_dirt);
        l.y += ROW;
        label(&p, &l, "Chromatic aberration", "Colour fringes toward the edges");
        if let Some(i) = segmented(ui, l.ctrl(), "vca", &["Off", "Subtle"], q.chroma as usize) {
            q.chroma = i as u8;
        }
        l.y += ROW;
        label(&p, &l, "Motion blur", "Camera: aircraft stays sharp");
        if let Some(i) = segmented(ui, l.ctrl(), "vmb", &["Off", "Camera", "Camera + objects"], q.motion_blur as usize) {
            q.motion_blur = i as u8;
        }
        cost(&p, &l, Knob::MotionBlur, q);
        l.y += ROW;
        label(&p, &l, "Depth of field", "Focus: panel / your aircraft");
        if let Some(i) = segmented(ui, l.ctrl(), "vdof", &["Off", "Cockpit", "External", "Both"], q.dof as usize) {
            q.dof = i as u8;
        }
        cost(&p, &l, Knob::Dof, q);
        l.y += ROW;
        label(&p, &l, "Film grain", "");
        let mut pct = q.grain * 100.0;
        if slider(ui, l.ctrl(), &mut pct, 0.0..=100.0, 5.0, |x| if x <= 0.0 { "Off".into() } else { format!("{x:.0} %") }) {
            q.grain = pct / 100.0;
        }
        l.y += ROW;
    }

    // ---- camera
    header(&p, &mut l, "Camera");
    {
        let v = &mut s.video;
        label(&p, &l, "Exposure", "On top of auto exposure");
        slider(ui, l.ctrl(), &mut v.exposure_ev, -2.0..=2.0, 0.1, |x| format!("{x:+.1} EV"));
        l.y += ROW;
        label(&p, &l, "Brightness", "Lifts or deepens the shadows");
        let mut pct = v.brightness * 100.0;
        if slider(ui, l.ctrl(), &mut pct, -50.0..=50.0, 5.0, |x| format!("{x:+.0}")) {
            v.brightness = pct / 100.0;
        }
        l.y += ROW;
        label(&p, &l, "Gamma", "2.2 = standard");
        slider(ui, l.ctrl(), &mut v.gamma, 1.8..=2.6, 0.05, |x| format!("{x:.2}"));
        l.y += ROW;
        label(&p, &l, "Field of view", "External, vertical");
        slider(ui, l.ctrl(), &mut v.fov_external, 30.0..=100.0, 1.0, |x| format!("{x:.0}°"));
        l.y += ROW;
        label(&p, &l, "Cockpit field of view", "Pilot's seat, vertical");
        slider(ui, l.ctrl(), &mut v.fov_cockpit, 40.0..=110.0, 1.0, |x| format!("{x:.0}°"));
        l.y += ROW;
        label(&p, &l, "Performance overlay", "FPS, GPU passes · Ctrl+F3");
        switch(ui, l.ctrl(), "vperf", &mut v.perf_overlay);
        l.y += ROW;
    }
    if !env.adapter.is_empty() {
        p.text(Pos2::new(l.x, l.y + 6.0), Align2::LEFT_TOP, format!("GPU: {}", env.adapter), f_ui(12.0), pal::TEXT3);
        l.y += 26.0;
    }

    // A knob changed: Custom unless it matches a preset; keep the legacy keys in step.
    if s.video.q != v0.q {
        s.video.knob_changed();
    }
    s.detail = s.video.q.detail;
    s.vsync = s.video.present == Present::Fifo;
    l.y - origin.y
}

/// An on/off switch filling the left of `r`; returns true when toggled.
fn switch(ui: &mut Ui, r: Rect, id: &str, v: &mut bool) -> bool {
    let sw = Rect::from_min_size(Pos2::new(r.min.x, r.center().y - 11.0), Vec2::new(40.0, 22.0));
    let resp = ui.interact(sw.expand(4.0), ui.id().with(("vswitch", id)), Sense::click());
    let p = ui.painter();
    p.rect_filled(sw, 11.0, if *v { pal::ACCENT } else { Color32::from_gray(70) });
    p.circle_filled(Pos2::new(if *v { sw.max.x - 11.0 } else { sw.min.x + 11.0 }, sw.center().y), 8.5, Color32::WHITE);
    p.text(Pos2::new(sw.max.x + 12.0, sw.center().y), Align2::LEFT_CENTER, if *v { "On" } else { "Off" }, f_ui(13.0), pal::TEXT2);
    if resp.hovered() {
        ui.ctx().set_cursor_icon(egui::CursorIcon::PointingHand);
    }
    if resp.clicked() {
        *v = !*v;
        return true;
    }
    false
}

/// A slider with its value text, snapped to `step`; returns true when changed.
fn slider(ui: &mut Ui, r: Rect, v: &mut f32, range: std::ops::RangeInclusive<f32>, step: f32, fmt: impl Fn(f32) -> String) -> bool {
    let before = *v;
    ui.scope_builder(UiBuilder::new().max_rect(r).layout(egui::Layout::left_to_right(egui::Align::Center)), |ui| {
        ui.spacing_mut().slider_width = r.width() - 90.0;
        ui.add(egui::Slider::new(v, range).step_by(step as f64).show_value(false));
        ui.label(RichText::new(fmt(*v)).font(f_mono(12.0)).color(pal::TEXT2));
    });
    (*v - before).abs() > 1e-6
}

/// "‹ value ›" stepper over `n` choices; returns the new index.
fn stepper(ui: &mut Ui, r: Rect, id: &str, text: &str, cur: usize, n: usize) -> Option<usize> {
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, Color32::from_rgb(9, 11, 15));
    p.text(r.center(), Align2::CENTER_CENTER, text, f_semi(13.0), pal::TEXT);
    let mut out = None;
    for (k, (glyph, dir)) in [("‹", -1i32), ("›", 1)].into_iter().enumerate() {
        let b = if k == 0 { Rect::from_min_size(r.min, Vec2::new(36.0, r.height())) } else { Rect::from_min_size(Pos2::new(r.max.x - 36.0, r.min.y), Vec2::new(36.0, r.height())) };
        let resp = ui.interact(b, ui.id().with((id, k)), Sense::click());
        p.text(b.center(), Align2::CENTER_CENTER, glyph, f_semi(18.0), if resp.hovered() { pal::TEXT } else { pal::TEXT2 });
        if resp.clicked() {
            out = Some(((cur as i32 + dir).rem_euclid(n.max(1) as i32)) as usize);
        }
    }
    out
}
