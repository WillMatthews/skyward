//! Settings › Video: the quality preset and every rendering knob
//! (gfx/quality.rs), as rows of the settings catalogue (ui/settings_schema.rs
//! splices [`items`] into its Video category, so search, the "changed"
//! dots, Reset category and Apply / Revert work on them), plus what only
//! the app knows: monitors and video modes, the GPU's recommended preset,
//! each knob's measured GPU cost, the dynamic-resolution scale now.
//!
//! Contract with the settings shell (ui/settings_ui.rs): [`video_tab`] draws
//! the category into `ui` (a vertical layout inside the shell's scroll area)
//! and returns true when something changed. Changes apply live (the
//! renderer reads the settings every frame); the rows marked "Restart" and
//! switching to / from MSAA take effect at the next start.

use super::settings::{DETAIL_NAMES, Settings};
use super::settings_schema::{Cat, Item, Kind, Val};
use super::{UiState, alpha, f_inst_b, f_mono, f_semi, f_ui, pal, sp};
use crate::gfx::quality::{AaMode, FrameCap, Knob, Present, Preset, RestartReason, WindowMode};
use egui::{Align2, Pos2, Rect, Sense, Ui, Vec2};

const VIDEO: &[Cat] = &[Cat::Video];

/// After a knob changed: Custom unless the knobs match a preset; keep the
/// legacy `detail` / `vsync` mirrors in step.
fn after(s: &mut Settings) {
    s.video.knob_changed();
    sync(s);
}

fn sync(s: &mut Settings) {
    s.detail = s.video.q.detail;
    s.vsync = s.video.present == Present::Fifo;
}

fn pct(v: f32) -> String {
    format!("{:.0} %", v * 100.0)
}
fn pct_off(v: f32) -> String {
    if v <= 0.0 { "Off".into() } else { format!("{:.0} %", v * 100.0) }
}
fn ev(v: f32) -> String {
    format!("{v:+.1} EV")
}
fn signed(v: f32) -> String {
    format!("{:+.0}", v * 100.0)
}
fn two(v: f32) -> String {
    format!("{v:.2}")
}
fn nits(v: f32) -> String {
    format!("{v:.0} nits")
}

fn item(
    id: &'static str,
    group: &'static str,
    label: &'static str,
    desc: &'static str,
    keywords: &'static str,
    kind: Kind,
    get: fn(&Settings) -> Val,
    set: fn(&mut Settings, Val),
) -> Item {
    Item { id, cats: VIDEO, group, label, desc, keywords, kind, get, set, restart: false, pending: None, native_only: false, in_flight_only: false }
}

/// A u8 level knob (`$field` on `Quality`) as a choice.
macro_rules! level {
    ($id:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, $opts:expr, $field:ident) => {
        item(
            $id,
            $group,
            $label,
            $desc,
            $kw,
            Kind::Choice($opts),
            |s| Val::I(s.video.q.$field as usize),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.q.$field = i as u8;
                    after(s);
                }
            },
        )
    };
}

/// An f32 knob on `Quality` as a slider.
macro_rules! knob_f {
    ($id:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, $min:expr, $max:expr, $step:expr, $fmt:expr, $field:ident) => {
        item(
            $id,
            $group,
            $label,
            $desc,
            $kw,
            Kind::Slider { min: $min, max: $max, step: $step, fmt: $fmt },
            |s| Val::F(s.video.q.$field),
            |s, v| {
                if let Val::F(f) = v {
                    s.video.q.$field = f.clamp($min, $max);
                    after(s);
                }
            },
        )
    };
}

/// A bool knob on `Quality`.
macro_rules! knob_b {
    ($id:expr, $group:expr, $label:expr, $desc:expr, $kw:expr, $field:ident) => {
        item(
            $id,
            $group,
            $label,
            $desc,
            $kw,
            Kind::Toggle,
            |s| Val::B(s.video.q.$field),
            |s, v| {
                if let Val::B(b) = v {
                    s.video.q.$field = b;
                    after(s);
                }
            },
        )
    };
}

const PRESETS: [&str; 5] = ["Low", "Medium", "High", "Ultra", "Custom"];
const ANISO: [u8; 5] = [1, 2, 4, 8, 16];
const SHADOW_RES: [u32; 3] = [1024, 2048, 4096];

/// The Video rows of the settings catalogue, in display order.
pub fn items() -> Vec<Item> {
    let web = crate::platform::WEB;
    let mut v = vec![
        // ---------------------------------------------------------- preset
        item(
            "video_preset",
            "Quality",
            "Quality preset",
            "Sets every option below; Custom once one is changed",
            "graphics quality preset low medium high ultra custom auto detect",
            Kind::Choice(&PRESETS),
            |s| Val::I(Preset::ALL.iter().position(|p| *p == s.video.preset).unwrap_or(2)),
            |s, v| {
                if let Val::I(i) = v
                    && i < 4
                {
                    s.video.apply_preset(Preset::FIXED[i]);
                    sync(s);
                }
            },
        ),
        // ---------------------------------------------------------- display
        Item {
            native_only: true,
            ..item(
                "video_window",
                "Display",
                "Window mode",
                "Fullscreen uses the video mode below",
                "window fullscreen borderless exclusive windowed monitor",
                Kind::Choice(&["Windowed", "Borderless", "Fullscreen"]),
                |s| Val::I(WindowMode::ALL.iter().position(|m| *m == s.video.window).unwrap_or(0)),
                |s, v| {
                    if let Val::I(i) = v {
                        s.video.window = WindowMode::ALL[i.min(2)];
                    }
                },
            )
        },
        item(
            "vsync",
            "Display",
            "Vertical sync",
            "Fast sync: low latency without tearing; Off may tear",
            "vsync tearing latency present fifo mailbox immediate fps",
            Kind::Choice(&["VSync", "Fast sync", "Off"]),
            |s| Val::I(Present::ALL.iter().position(|m| *m == s.video.present).unwrap_or(0)),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.present = Present::ALL[i.min(2)];
                    sync(s);
                }
            },
        ),
        item(
            "video_frame_cap",
            "Display",
            "Frame-rate limit",
            "Dynamic resolution aims for it",
            "fps frame rate limit cap 30 60 120 144 refresh",
            Kind::Choice(&["Unlimited", "30", "60", "120", "144", "Display"]),
            |s| Val::I(FrameCap::CHOICES.iter().position(|c| *c == s.video.frame_cap).unwrap_or(0)),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.frame_cap = FrameCap::CHOICES[i.min(5)];
                }
            },
        ),
        Item {
            native_only: true,
            restart: true,
            ..item(
                "video_hdr",
                "Display",
                "HDR output",
                "HDR displays (experimental)",
                "hdr high dynamic range nits scrgb",
                Kind::Toggle,
                |s| Val::B(s.video.hdr),
                |s, v| {
                    if let Val::B(b) = v {
                        s.video.hdr = b;
                    }
                },
            )
        },
        Item {
            native_only: true,
            ..item(
                "video_paper_white",
                "Display",
                "HDR paper white",
                "Brightness of white in HDR",
                "hdr nits brightness paper white",
                Kind::Slider { min: 80.0, max: 400.0, step: 10.0, fmt: nits },
                |s| Val::F(s.video.paper_white),
                |s, v| {
                    if let Val::F(f) = v {
                        s.video.paper_white = f.clamp(80.0, 400.0);
                    }
                },
            )
        },
        knob_f!(
            "video_render_scale",
            "Display",
            "Render scale",
            "Below 100 %: TAA upscales; above: supersampling",
            "resolution scale upscaling taau supersampling dlss fsr",
            0.5,
            2.0,
            0.05,
            pct,
            render_scale
        ),
        knob_b!("video_dynamic_res", "Display", "Dynamic resolution", "Lowers the render scale to hold the frame rate", "dynamic resolution scaling fps target", dynamic_res),
        // ---------------------------------------------------------- image quality
        item(
            "video_aa",
            "Image quality",
            "Anti-aliasing",
            "TAA: smooth edges, wires and foliage; FXAA: cheapest",
            "anti-aliasing antialiasing taa msaa fxaa jaggies shimmer",
            Kind::Choice(&["Off", "FXAA", "MSAA 4×", "TAA"]),
            |s| Val::I(AaMode::ALL.iter().position(|m| *m == s.video.q.aa).unwrap_or(3)),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.q.aa = AaMode::ALL[i.min(3)];
                    after(s);
                }
            },
        ),
        knob_f!("video_sharpen", "Image quality", "Sharpening", "Restores detail after TAA", "sharpen sharpness cas blur taa", 0.0, 1.0, 0.05, pct, sharpen),
        Item {
            restart: true,
            ..item(
                "video_aniso",
                "Image quality",
                "Anisotropic filtering",
                "Sharp textures at grazing angles",
                "anisotropic filtering texture af",
                Kind::Choice(&["1×", "2×", "4×", "8×", "16×"]),
                |s| Val::I(ANISO.iter().position(|a| *a == s.video.q.aniso).unwrap_or(3)),
                |s, v| {
                    if let Val::I(i) = v {
                        s.video.q.aniso = ANISO[i.min(4)];
                        after(s);
                    }
                },
            )
        },
        Item {
            restart: true,
            ..level!("video_textures", "Image quality", "Texture detail", "Aircraft and airport textures", "texture quality resolution mip", &["Low", "Medium", "High"], textures)
        },
        // ---------------------------------------------------------- lighting & shadows
        item(
            "video_shadow_res",
            "Lighting & shadows",
            "Shadow resolution",
            "Sharpness of sun and moon shadows",
            "shadow quality resolution map",
            Kind::Choice(if web { &["1024", "2048"] } else { &["1024", "2048", "4096"] }),
            |s| Val::I(SHADOW_RES.iter().position(|r| *r == s.video.q.shadow_res).unwrap_or(2)),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.q.shadow_res = SHADOW_RES[i.min(2)];
                    after(s);
                }
            },
        ),
        item(
            "video_cascades",
            "Lighting & shadows",
            "Shadow cascades",
            "More: crisper shadows near and far",
            "shadow cascades csm",
            Kind::Choice(&["2", "3", "4"]),
            |s| Val::I((s.video.q.cascades as usize).clamp(2, 4) - 2),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.q.cascades = i.min(2) as u8 + 2;
                    after(s);
                }
            },
        ),
        knob_f!("video_shadow_dist", "Lighting & shadows", "Shadow distance", "How far out shadows are drawn", "shadow distance range", 0.5, 1.5, 0.05, pct, shadow_dist),
        knob_b!("video_soft_shadows", "Lighting & shadows", "Soft shadows", "Filtered shadow edges", "soft shadows pcf filtering", soft_shadows),
        level!(
            "video_ao",
            "Lighting & shadows",
            "Ambient occlusion",
            "Contact shading in corners and under objects",
            "ambient occlusion ssao ao",
            &["Off", "Low", "Medium", "High"],
            ao
        ),
        level!("video_clouds", "Lighting & shadows", "Clouds", "Volumetric cloud detail", "clouds volumetric quality raymarch", &["Low", "Medium", "High", "Ultra"], clouds),
        level!("video_fog", "Lighting & shadows", "Atmosphere & fog", "Ground fog and mist detail", "fog mist atmosphere haze", &["Low", "Medium", "High", "Ultra"], fog),
        level!(
            "video_reflections",
            "Lighting & shadows",
            "Water reflections",
            "Resolution of the mirrored scene",
            "reflections water mirror",
            &["Quarter", "Half", "Full"],
            reflections
        ),
        // ---------------------------------------------------------- world detail
        item(
            "detail",
            "World detail",
            "Scenery detail",
            "How far detailed buildings and roads stream in",
            "detail distance radius tiles buildings lod",
            Kind::Choice(&DETAIL_NAMES),
            |s| Val::I(s.video.q.detail as usize),
            |s, v| {
                if let Val::I(i) = v {
                    s.video.q.detail = i.min(3) as u8;
                    after(s);
                }
            },
        ),
        knob_f!("video_tree_density", "World detail", "Tree density", "Share of the trees drawn", "trees forest vegetation density", 0.25, 1.0, 0.05, pct, tree_density),
        knob_f!("video_tree_dist", "World detail", "Tree distance", "How far out trees are drawn", "trees distance lod", 0.5, 1.5, 0.05, pct, tree_dist),
        knob_f!("video_building_dist", "World detail", "Building distance", "Far-field buildings", "buildings distance far lod", 0.5, 1.0, 0.05, pct, building_dist),
        // ---------------------------------------------------------- effects
        level!("video_vfx", "Effects", "Effects quality", "Vapour, heat haze and smoke detail", "effects vfx particles vapour haze smoke", &["Low", "Medium", "High"], vfx),
        knob_f!("video_bloom", "Effects", "Bloom", "Glow around bright lights", "bloom glow glare", 0.0, 1.5, 0.05, pct_off, bloom),
        knob_b!("video_lens_flare", "Effects", "Lens flare", "Ghosts and a halo from the sun", "lens flare sun ghosts", lens_flare),
        knob_b!("video_lens_dirt", "Effects", "Lens dirt", "Smudges lit by bright light", "lens dirt smudges", lens_dirt),
        level!("video_chroma", "Effects", "Chromatic aberration", "Colour fringes toward the edges", "chromatic aberration fringe", &["Off", "Subtle"], chroma),
        level!("video_motion_blur", "Effects", "Motion blur", "Camera only keeps your aircraft sharp", "motion blur", &["Off", "Camera", "Camera + objects"], motion_blur),
        level!(
            "video_dof",
            "Effects",
            "Depth of field",
            "Focus on the panel / on your aircraft",
            "depth of field dof bokeh focus blur",
            &["Off", "Cockpit", "External", "Both"],
            dof
        ),
        knob_f!("video_grain", "Effects", "Film grain", "Cinematic noise over the image", "film grain noise", 0.0, 1.0, 0.05, pct_off, grain),
        // ---------------------------------------------------------- image
        item(
            "video_exposure",
            "Image",
            "Exposure",
            "On top of the automatic exposure",
            "exposure brightness ev",
            Kind::Slider { min: -2.0, max: 2.0, step: 0.1, fmt: ev },
            |s| Val::F(s.video.exposure_ev),
            |s, v| {
                if let Val::F(f) = v {
                    s.video.exposure_ev = f.clamp(-2.0, 2.0);
                }
            },
        ),
        item(
            "video_brightness",
            "Image",
            "Brightness",
            "Lifts or deepens the shadows",
            "brightness black level",
            Kind::Slider { min: -0.5, max: 0.5, step: 0.05, fmt: signed },
            |s| Val::F(s.video.brightness),
            |s, v| {
                if let Val::F(f) = v {
                    s.video.brightness = f.clamp(-0.5, 0.5);
                }
            },
        ),
        item(
            "video_gamma",
            "Image",
            "Gamma",
            "Display gamma (2.2 = standard)",
            "gamma display",
            Kind::Slider { min: 1.8, max: 2.6, step: 0.05, fmt: two },
            |s| Val::F(s.video.gamma),
            |s, v| {
                if let Val::F(f) = v {
                    s.video.gamma = f.clamp(1.8, 2.6);
                }
            },
        ),
        item(
            "video_perf_overlay",
            "Image",
            "Performance overlay",
            "FPS and GPU time per pass · Ctrl+F3",
            "performance overlay fps frame time gpu timing stats",
            Kind::Toggle,
            |s| Val::B(s.video.perf_overlay),
            |s, v| {
                if let Val::B(b) = v {
                    s.video.perf_overlay = b;
                }
            },
        ),
    ];
    if web {
        v.retain(|i| !i.native_only);
    }
    v
}

/// The GPU cost hint of a row, if it has one.
fn knob_of(id: &str) -> Option<Knob> {
    Some(match id {
        "video_render_scale" => Knob::RenderScale,
        "video_aa" => Knob::Aa,
        "video_aniso" => Knob::Aniso,
        "video_shadow_res" => Knob::Shadows,
        "video_cascades" => Knob::Cascades,
        "video_shadow_dist" => Knob::ShadowDist,
        "video_soft_shadows" => Knob::SoftShadows,
        "video_ao" => Knob::Ao,
        "video_clouds" => Knob::Clouds,
        "video_fog" => Knob::Fog,
        "video_reflections" => Knob::Reflections,
        "detail" => Knob::Detail,
        "video_tree_density" => Knob::Trees,
        "video_building_dist" => Knob::Buildings,
        "video_vfx" => Knob::Vfx,
        "video_bloom" => Knob::Bloom,
        "video_lens_flare" => Knob::LensFlare,
        "video_motion_blur" => Knob::MotionBlur,
        "video_dof" => Knob::Dof,
        _ => return None,
    })
}

/// Draw the Video category; true when something changed.
pub fn video_tab(ui: &mut Ui, st: &mut UiState) -> bool {
    let before = st.settings.clone();
    let env = st.video_env.clone();
    let px = (env.output.0.max(1) * env.output.1.max(1)) as f32;
    let flying = matches!(st.back_to, super::Screen::Pause | super::Screen::Flying);
    let rows = super::settings_schema::rows(Cat::Video, flying);
    let mut group = "";
    for it in rows {
        if it.group != group {
            group = it.group;
            ui.add_space(6.0);
            ui.label(egui::RichText::new(group.to_uppercase()).font(f_inst_b(11.0)).color(pal::ACCENT));
            ui.add_space(2.0);
        }
        // (Rows that only matter in some states.)
        let v = &st.settings.video;
        if (it.id == "video_paper_white" && !(v.hdr && env.hdr_supported)) || (it.id == "video_hdr" && !env.hdr_supported) || (it.id == "video_sharpen" && v.q.aa != AaMode::Taa) {
            continue;
        }
        let w = ui.available_width() - super::scroll::GUTTER;
        let top = ui.cursor().min.y;
        let x0 = ui.cursor().min.x;
        super::settings_ui::setting_row(ui, st, it);
        let h = if it.desc.is_empty() { 46.0 } else { 58.0 };
        let r = Rect::from_min_size(Pos2::new(x0, top), Vec2::new(w, h));
        note(ui, st, it.id, r, px, &env);
        ui.add_space(4.0);
        // Monitor and video mode (only the app knows them) after the window mode.
        if it.id == "video_window" {
            display_rows(ui, st, &env);
        }
    }
    if !env.adapter.is_empty() {
        ui.add_space(8.0);
        let rec = env.detected.map(|p| format!(" · recommended preset: {}", p.name())).unwrap_or_default();
        ui.label(egui::RichText::new(format!("GPU: {}{rec}", env.adapter)).font(f_ui(12.0)).color(pal::TEXT3));
    }
    if st.settings != before {
        sync(&mut st.settings);
        st.settings_dirty = true;
        return true;
    }
    false
}

/// Right of a row's label: its GPU cost, or a status (restart, fallback, live scale).
fn note(ui: &Ui, st: &UiState, id: &str, r: Rect, px: f32, env: &crate::gfx::quality::VideoEnv) {
    let v = &st.settings.video;
    let cw = (r.width() * 0.46).clamp(180.0, 430.0).min(r.width() - 170.0);
    let at = Pos2::new(r.max.x - 12.0 - cw - 14.0, r.min.y + 19.0);
    let running = env.running.unwrap_or(v.q);
    let (text, col) = match id {
        "video_aa" if running.restart_diff(&v.q).contains(&RestartReason::Msaa) => ("restart".to_string(), pal::AMBER),
        "vsync" if !env.present_modes.is_empty() && !env.present_modes.contains(&v.present.wgpu()) => ("fallback".to_string(), pal::AMBER),
        "video_dynamic_res" if v.q.dynamic_res && env.dyn_scale.is_some() => (format!("now {:.0} %", env.dyn_scale.unwrap_or(1.0) * 100.0), pal::TEXT3),
        _ => match knob_of(id) {
            Some(k) => {
                let ms = crate::gfx::quality::cost_ms(k, &v.q, px);
                (crate::gfx::quality::cost_label(ms), if ms >= 1.0 { pal::AMBER } else { pal::TEXT3 })
            }
            None => return,
        },
    };
    ui.painter().text(at, Align2::RIGHT_CENTER, text, f_mono(11.5), col);
}

/// Monitor and exclusive-fullscreen mode rows (native, when there is a choice).
fn display_rows(ui: &mut Ui, st: &mut UiState, env: &crate::gfx::quality::VideoEnv) {
    if crate::platform::WEB {
        return;
    }
    if env.monitors.len() > 1 {
        let cur = st.settings.video.monitor.as_ref().and_then(|m| env.monitors.iter().position(|x| x == m)).unwrap_or(0);
        if let Some(i) = custom_row(ui, "video_monitor", "Monitor", "Where fullscreen and borderless go", &env.monitors[cur], cur, env.monitors.len()) {
            st.settings.video.monitor = Some(env.monitors[i].clone());
        }
    }
    if st.settings.video.window == WindowMode::Exclusive && !env.modes.is_empty() {
        let cur = st.settings.video.mode.and_then(|m| env.modes.iter().position(|x| *x == m)).unwrap_or(0).min(env.modes.len() - 1);
        let m = env.modes[cur];
        let text = format!("{} × {}  ·  {:.0} Hz", m.0, m.1, m.2 as f32 / 1000.0);
        if let Some(i) = custom_row(ui, "video_mode", "Resolution", "Exclusive fullscreen video mode", &text, cur, env.modes.len()) {
            st.settings.video.mode = Some(env.modes[i]);
        }
    }
}

/// A row in the shell's style with a "‹ value ›" stepper; returns the new index.
fn custom_row(ui: &mut Ui, id: &str, label: &str, desc: &str, value: &str, cur: usize, n: usize) -> Option<usize> {
    let w = ui.available_width() - super::scroll::GUTTER;
    let (r, resp) = ui.allocate_exact_size(Vec2::new(w, 58.0), Sense::hover());
    let p = ui.painter().clone();
    p.rect_filled(r, sp::R_CTRL, if resp.hovered() { pal::RAISED } else { alpha(pal::RAISED, 0.45) });
    p.text(Pos2::new(r.min.x + 22.0, r.min.y + 19.0), Align2::LEFT_CENTER, label, f_semi(14.0), pal::TEXT);
    p.text(Pos2::new(r.min.x + 22.0, r.min.y + 32.0), Align2::LEFT_TOP, desc, f_ui(12.0), pal::TEXT3);
    let cw = (w * 0.46).clamp(180.0, 430.0).min(w - 170.0);
    let cr = Rect::from_min_size(Pos2::new(r.max.x - 12.0 - cw, r.center().y - 17.0), Vec2::new(cw, 34.0));
    p.rect_filled(cr, sp::R_CTRL, egui::Color32::from_rgb(9, 11, 15));
    p.text(cr.center(), Align2::CENTER_CENTER, value, f_semi(13.0), pal::TEXT);
    let mut out = None;
    for (k, (glyph, dir)) in [("‹", -1i32), ("›", 1)].into_iter().enumerate() {
        let b = if k == 0 {
            Rect::from_min_size(cr.min, Vec2::new(36.0, cr.height()))
        } else {
            Rect::from_min_size(Pos2::new(cr.max.x - 36.0, cr.min.y), Vec2::new(36.0, cr.height()))
        };
        let bresp = ui.interact(b, ui.id().with((id, k)), Sense::click());
        p.text(b.center(), Align2::CENTER_CENTER, glyph, f_semi(18.0), if bresp.hovered() { pal::TEXT } else { pal::TEXT2 });
        if bresp.clicked() {
            out = Some(((cur as i32 + dir).rem_euclid(n.max(1) as i32)) as usize);
        }
    }
    ui.add_space(4.0);
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every row reads what it writes, ids are unique, and changing a knob
    /// through its row makes the preset Custom while keeping the mirrors.
    #[test]
    fn rows_round_trip() {
        let rows = items();
        let mut ids: Vec<&str> = rows.iter().map(|i| i.id).collect();
        ids.sort();
        ids.dedup();
        assert_eq!(ids.len(), rows.len());
        let mut s = Settings::default();
        for it in &rows {
            let v = (it.get)(&s);
            (it.set)(&mut s, v.clone());
            assert_eq!((it.get)(&s), v, "{}", it.id);
        }
        assert_eq!(s.video.preset, Preset::High);
        let ao = rows.iter().find(|i| i.id == "video_ao").unwrap();
        (ao.set)(&mut s, Val::I(0));
        assert_eq!(s.video.preset, Preset::Custom);
        let preset = rows.iter().find(|i| i.id == "video_preset").unwrap();
        (preset.set)(&mut s, Val::I(0));
        assert_eq!((s.video.preset, s.detail), (Preset::Low, 0));
        let vsync = rows.iter().find(|i| i.id == "vsync").unwrap();
        (vsync.set)(&mut s, Val::I(1));
        assert!(!s.vsync);
    }
}
