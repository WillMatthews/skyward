//! Video settings: the quality presets (Low → Ultra, Custom) and every knob
//! they drive, display options (window mode, present mode, frame cap, HDR,
//! FOV), their JSON form inside `settings.json` (backwards-compatible: files
//! from before this existed migrate from the old `detail` / `vsync` keys),
//! adapter auto-detection and the GPU cost hints the Video tab shows.
//!
//! The renderer reads a [`Quality`] each frame ([`crate::gfx::Renderer::set_quality`]);
//! almost everything applies live. What needs a restart is listed in
//! [`Quality::restart_diff`].

use serde_json::{Value, json};

/// A quality preset. `Custom` = any knob moved away from a preset's value.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Preset {
    Low,
    Medium,
    #[default]
    High,
    Ultra,
    Custom,
}

impl Preset {
    pub const ALL: [Preset; 5] = [Preset::Low, Preset::Medium, Preset::High, Preset::Ultra, Preset::Custom];
    pub const FIXED: [Preset; 4] = [Preset::Low, Preset::Medium, Preset::High, Preset::Ultra];
    pub fn name(self) -> &'static str {
        match self {
            Preset::Low => "Low",
            Preset::Medium => "Medium",
            Preset::High => "High",
            Preset::Ultra => "Ultra",
            Preset::Custom => "Custom",
        }
    }
    pub fn from_name(s: &str) -> Option<Preset> {
        Preset::ALL.into_iter().find(|p| p.name().eq_ignore_ascii_case(s))
    }
    /// The knobs of a fixed preset (`Custom` → High's).
    pub fn quality(self) -> Quality {
        Quality::preset(self)
    }
}

/// Anti-aliasing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AaMode {
    Off,
    /// Fast approximate AA (one post pass): the cheap option, default on the web's Low.
    Fxaa,
    /// 4× multisampling: geometry edges only (no shader / foliage aliasing), costly.
    Msaa4,
    /// Temporal AA: jittered frames accumulated with motion vectors; also the
    /// temporal upscaler (TAAU) when the render scale is below 100 %.
    Taa,
}

impl AaMode {
    pub const ALL: [AaMode; 4] = [AaMode::Off, AaMode::Fxaa, AaMode::Msaa4, AaMode::Taa];
    pub fn name(self) -> &'static str {
        match self {
            AaMode::Off => "Off",
            AaMode::Fxaa => "FXAA",
            AaMode::Msaa4 => "MSAA 4×",
            AaMode::Taa => "TAA",
        }
    }
    fn key(self) -> &'static str {
        match self {
            AaMode::Off => "off",
            AaMode::Fxaa => "fxaa",
            AaMode::Msaa4 => "msaa4",
            AaMode::Taa => "taa",
        }
    }
    fn from_key(s: &str) -> Option<AaMode> {
        AaMode::ALL.into_iter().find(|a| a.key() == s)
    }
    /// Scene sample count (pipelines are built for it: changing it needs a restart).
    pub fn samples(self) -> u32 {
        if self == AaMode::Msaa4 { 4 } else { 1 }
    }
}

/// Swapchain presentation.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum Present {
    /// VSync (FIFO): no tearing; the frame rate locks to divisors of the refresh.
    #[default]
    Fifo,
    /// Mailbox: no tearing, lowest-latency vsync (newest frame wins).
    Mailbox,
    /// Immediate: no vsync, may tear.
    Immediate,
}

impl Present {
    pub const ALL: [Present; 3] = [Present::Fifo, Present::Mailbox, Present::Immediate];
    pub fn name(self) -> &'static str {
        match self {
            Present::Fifo => "VSync",
            Present::Mailbox => "Fast sync",
            Present::Immediate => "Off",
        }
    }
    fn key(self) -> &'static str {
        match self {
            Present::Fifo => "fifo",
            Present::Mailbox => "mailbox",
            Present::Immediate => "immediate",
        }
    }
    fn from_key(s: &str) -> Option<Present> {
        Present::ALL.into_iter().find(|a| a.key() == s)
    }
    pub fn wgpu(self) -> wgpu::PresentMode {
        match self {
            Present::Fifo => wgpu::PresentMode::Fifo,
            Present::Mailbox => wgpu::PresentMode::Mailbox,
            Present::Immediate => wgpu::PresentMode::Immediate,
        }
    }
    /// The mode to configure given what the surface supports (falls back
    /// Immediate → Mailbox → FIFO, which is always there).
    pub fn pick(self, supported: &[wgpu::PresentMode]) -> wgpu::PresentMode {
        let order: &[Present] = match self {
            Present::Fifo => &[Present::Fifo],
            Present::Mailbox => &[Present::Mailbox, Present::Immediate, Present::Fifo],
            Present::Immediate => &[Present::Immediate, Present::Mailbox, Present::Fifo],
        };
        order.iter().map(|p| p.wgpu()).find(|m| supported.contains(m)).unwrap_or(wgpu::PresentMode::Fifo)
    }
}

/// Window mode (native only).
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum WindowMode {
    #[default]
    Windowed,
    /// Borderless window covering the monitor (instant alt-tab).
    Borderless,
    /// Exclusive fullscreen at a chosen video mode.
    Exclusive,
}

impl WindowMode {
    pub const ALL: [WindowMode; 3] = [WindowMode::Windowed, WindowMode::Borderless, WindowMode::Exclusive];
    pub fn name(self) -> &'static str {
        match self {
            WindowMode::Windowed => "Windowed",
            WindowMode::Borderless => "Borderless",
            WindowMode::Exclusive => "Fullscreen",
        }
    }
    fn key(self) -> &'static str {
        match self {
            WindowMode::Windowed => "windowed",
            WindowMode::Borderless => "borderless",
            WindowMode::Exclusive => "exclusive",
        }
    }
    fn from_key(s: &str) -> Option<WindowMode> {
        WindowMode::ALL.into_iter().find(|a| a.key() == s)
    }
}

/// Frame-rate limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub enum FrameCap {
    #[default]
    Unlimited,
    Fps(u32),
    /// The monitor's refresh rate.
    Display,
}

impl FrameCap {
    pub const CHOICES: [FrameCap; 6] = [FrameCap::Unlimited, FrameCap::Fps(30), FrameCap::Fps(60), FrameCap::Fps(120), FrameCap::Fps(144), FrameCap::Display];
    pub fn name(self) -> String {
        match self {
            FrameCap::Unlimited => "Unlimited".into(),
            FrameCap::Fps(n) => format!("{n}"),
            FrameCap::Display => "Display".into(),
        }
    }
    fn to_json(self) -> Value {
        match self {
            FrameCap::Unlimited => json!(0),
            FrameCap::Fps(n) => json!(n),
            FrameCap::Display => json!("display"),
        }
    }
    fn from_json(v: &Value) -> FrameCap {
        if v.as_str() == Some("display") {
            return FrameCap::Display;
        }
        match v.as_u64() {
            Some(n) if (10..=1000).contains(&n) => FrameCap::Fps(n as u32),
            _ => FrameCap::Unlimited,
        }
    }
    /// Target frame rate (Hz) given the display's refresh rate, None = unlimited.
    pub fn hz(self, display_hz: Option<f32>) -> Option<f32> {
        match self {
            FrameCap::Unlimited => None,
            FrameCap::Fps(n) => Some(n as f32),
            FrameCap::Display => display_hz.or(Some(60.0)),
        }
    }
}

/// Every preset-driven rendering knob (what "Reset to preset" restores).
/// Levels are 0 = lowest; see the doc of each field.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Quality {
    pub aa: AaMode,
    /// TAA / upscaler sharpening (contrast-adaptive), 0..1.
    pub sharpen: f32,
    /// Scene resolution relative to the output, 0.5..2.0 (TAAU below 1, supersampling above).
    pub render_scale: f32,
    /// Lower the render scale (down to 50 %) when the GPU can't hold the frame-rate target.
    pub dynamic_res: bool,
    /// Shadow cascade resolution (px): 1024, 2048, 4096.
    pub shadow_res: u32,
    /// Shadow cascades drawn: 2..4.
    pub cascades: u8,
    /// Shadow distance multiplier (cascade splits), 0.5..1.5.
    pub shadow_dist: f32,
    /// 5×5 tent-filtered PCF (else a single bilinear tap: hard edges).
    pub soft_shadows: bool,
    /// Ambient occlusion: 0 off, 1 low (6 samples), 2 medium (12), 3 high (16, temporally rotated).
    pub ao: u8,
    /// Volumetric clouds: 0 low .. 3 ultra (ray-march step size).
    pub clouds: u8,
    /// Ground fog / mist integration: 0 low .. 3 ultra.
    pub fog: u8,
    /// Planar water reflection: 0 quarter res, 1 half res (default), 2 full res.
    pub reflections: u8,
    /// Scenery detail (tile streaming radius): 0 low .. 3 ultra.
    pub detail: u8,
    /// Trees drawn: fraction of the instances (0.25..1).
    pub tree_density: f32,
    /// Tree draw-distance multiplier (0.5..1.5).
    pub tree_dist: f32,
    /// Building draw-distance multiplier for the far (z10) tiles (0.5..1.5).
    pub building_dist: f32,
    /// Anisotropic filtering: 1, 2, 4, 8, 16 (aircraft / airport textures).
    pub aniso: u8,
    /// Texture detail: 0 low (mip bias +1), 1 medium (+0.5), 2 high (0).
    pub textures: u8,
    /// Particle / vapour / heat-haze effects: 0 low (no haze distortion, coarse vapour) .. 2 high.
    pub vfx: u8,
    /// Bloom strength multiplier (0 = off .. 1.5).
    pub bloom: f32,
    /// Sun / bright-light ghosts and halo.
    pub lens_flare: bool,
    /// Lens dirt lit by bloom.
    pub lens_dirt: bool,
    /// Chromatic aberration: 0 off, 1 subtle.
    pub chroma: u8,
    /// Motion blur: 0 off, 1 camera, 2 camera + objects.
    pub motion_blur: u8,
    /// Depth of field: 0 off, 1 cockpit (far scenery soft), 2 external (focus on the aircraft), 3 both.
    pub dof: u8,
    /// Film grain amount (0 = off .. 1).
    pub grain: f32,
}

impl Default for Quality {
    fn default() -> Self {
        Quality::preset(Preset::High)
    }
}

/// Fields that need a restart to change (pipelines / samplers are built for them).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RestartReason {
    /// Switching to / from MSAA (scene sample count).
    Msaa,
    /// Anisotropic filtering / texture detail (samplers).
    Textures,
    /// HDR output (swapchain format).
    Hdr,
}

impl Quality {
    pub fn preset(p: Preset) -> Quality {
        let web = crate::platform::WEB;
        let q = Quality {
            aa: AaMode::Taa,
            sharpen: 0.35,
            render_scale: 1.0,
            dynamic_res: false,
            shadow_res: 4096,
            cascades: 4,
            shadow_dist: 1.0,
            soft_shadows: true,
            ao: 2,
            clouds: 2,
            fog: 2,
            reflections: 1,
            detail: 2,
            tree_density: 1.0,
            tree_dist: 1.0,
            building_dist: 1.0,
            aniso: 8,
            textures: 2,
            vfx: 2,
            bloom: 1.0,
            lens_flare: false,
            lens_dirt: false,
            chroma: 0,
            motion_blur: 0,
            dof: 0,
            grain: 0.0,
        };
        let q = match p {
            Preset::Low => Quality {
                aa: AaMode::Fxaa,
                sharpen: 0.0,
                render_scale: 0.75,
                dynamic_res: true,
                shadow_res: 1024,
                cascades: 2,
                shadow_dist: 0.6,
                soft_shadows: false,
                ao: 0,
                clouds: 0,
                fog: 0,
                reflections: 0,
                detail: 0,
                tree_density: 0.5,
                tree_dist: 0.6,
                building_dist: 0.6,
                aniso: 2,
                textures: 1,
                vfx: 0,
                ..q
            },
            Preset::Medium => Quality {
                render_scale: 0.85,
                dynamic_res: true,
                shadow_res: 2048,
                cascades: 3,
                shadow_dist: 0.8,
                ao: 1,
                clouds: 1,
                fog: 1,
                reflections: 0,
                detail: 1,
                tree_density: 0.75,
                tree_dist: 0.8,
                building_dist: 0.8,
                aniso: 4,
                vfx: 1,
                ..q
            },
            Preset::High | Preset::Custom => q,
            Preset::Ultra => Quality { sharpen: 0.3, shadow_dist: 1.15, ao: 3, clouds: 3, fog: 3, detail: 3, tree_dist: 1.2, aniso: 16, lens_flare: true, ..q },
        };
        // The browser: 2048 shadows at most (memory, budget).
        if web { Quality { shadow_res: q.shadow_res.min(2048), ..q } } else { q }
    }

    /// The fixed preset these knobs equal, else Custom.
    pub fn matching_preset(&self) -> Preset {
        Preset::FIXED.into_iter().find(|p| Quality::preset(*p) == *self).unwrap_or(Preset::Custom)
    }

    /// What changed between `self` (running) and `new` that only applies after a restart.
    pub fn restart_diff(&self, new: &Quality) -> Vec<RestartReason> {
        let mut v = Vec::new();
        if self.aa.samples() != new.aa.samples() {
            v.push(RestartReason::Msaa);
        }
        if self.aniso != new.aniso || self.textures != new.textures {
            v.push(RestartReason::Textures);
        }
        v
    }

    /// Clamp everything into range (hand-edited files).
    pub fn sanitized(mut self) -> Quality {
        self.sharpen = self.sharpen.clamp(0.0, 1.0);
        self.render_scale = self.render_scale.clamp(0.5, 2.0);
        self.shadow_res = match self.shadow_res {
            0..=1535 => 1024,
            1536..=3071 => 2048,
            _ => 4096,
        };
        if crate::platform::WEB {
            self.shadow_res = self.shadow_res.min(2048);
        }
        self.cascades = self.cascades.clamp(2, 4);
        self.shadow_dist = self.shadow_dist.clamp(0.5, 1.5);
        self.ao = self.ao.min(3);
        self.clouds = self.clouds.min(3);
        self.fog = self.fog.min(3);
        self.reflections = self.reflections.min(2);
        self.detail = self.detail.min(3);
        self.tree_density = self.tree_density.clamp(0.25, 1.0);
        self.tree_dist = self.tree_dist.clamp(0.5, 1.5);
        self.building_dist = self.building_dist.clamp(0.5, 1.5);
        self.aniso = [1u8, 2, 4, 8, 16].into_iter().rev().find(|a| *a <= self.aniso.max(1)).unwrap_or(1);
        self.textures = self.textures.min(2);
        self.vfx = self.vfx.min(2);
        self.bloom = self.bloom.clamp(0.0, 1.5);
        self.chroma = self.chroma.min(1);
        self.motion_blur = self.motion_blur.min(2);
        self.dof = self.dof.min(3);
        self.grain = self.grain.clamp(0.0, 1.0);
        self
    }

    /// The knobs with `k=v,k=v` overrides (the JSON keys; captures and
    /// benchmarks: `SKYWARD_QUALITY="clouds=0,ao=3,aa=fxaa"`).
    pub fn with_overrides(self, spec: &str) -> Quality {
        let mut m = serde_json::Map::new();
        for kv in spec.split(',') {
            let Some((k, v)) = kv.split_once('=') else { continue };
            let v = v.trim();
            let val = match v {
                "true" | "on" => json!(true),
                "false" | "off" if k.trim() != "aa" => json!(false),
                _ => v.parse::<u64>().map(|n| json!(n)).or_else(|_| v.parse::<f64>().map(|f| json!(f))).unwrap_or_else(|_| json!(v)),
            };
            m.insert(k.trim().to_string(), val);
        }
        Quality::from_json(&Value::Object(m), self)
    }

    fn to_json(self) -> Value {
        json!({
            "aa": self.aa.key(),
            "sharpen": self.sharpen,
            "render_scale": self.render_scale,
            "dynamic_res": self.dynamic_res,
            "shadow_res": self.shadow_res,
            "cascades": self.cascades,
            "shadow_dist": self.shadow_dist,
            "soft_shadows": self.soft_shadows,
            "ao": self.ao,
            "clouds": self.clouds,
            "fog": self.fog,
            "reflections": self.reflections,
            "detail": self.detail,
            "tree_density": self.tree_density,
            "tree_dist": self.tree_dist,
            "building_dist": self.building_dist,
            "aniso": self.aniso,
            "textures": self.textures,
            "vfx": self.vfx,
            "bloom": self.bloom,
            "lens_flare": self.lens_flare,
            "lens_dirt": self.lens_dirt,
            "chroma": self.chroma,
            "motion_blur": self.motion_blur,
            "dof": self.dof,
            "grain": self.grain,
        })
    }

    /// Read the knobs present in `v` over `base` (missing keys keep `base`'s).
    fn from_json(v: &Value, base: Quality) -> Quality {
        let f = |k: &str, d: f32| v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(d);
        let u = |k: &str, d: u8| v.get(k).and_then(|x| x.as_u64()).map(|x| x.min(255) as u8).unwrap_or(d);
        let b = |k: &str, d: bool| v.get(k).and_then(|x| x.as_bool()).unwrap_or(d);
        Quality {
            aa: v.get("aa").and_then(|x| x.as_str()).and_then(AaMode::from_key).unwrap_or(base.aa),
            sharpen: f("sharpen", base.sharpen),
            render_scale: f("render_scale", base.render_scale),
            dynamic_res: b("dynamic_res", base.dynamic_res),
            shadow_res: v.get("shadow_res").and_then(|x| x.as_u64()).map(|x| x as u32).unwrap_or(base.shadow_res),
            cascades: u("cascades", base.cascades),
            shadow_dist: f("shadow_dist", base.shadow_dist),
            soft_shadows: b("soft_shadows", base.soft_shadows),
            ao: u("ao", base.ao),
            clouds: u("clouds", base.clouds),
            fog: u("fog", base.fog),
            reflections: u("reflections", base.reflections),
            detail: u("detail", base.detail),
            tree_density: f("tree_density", base.tree_density),
            tree_dist: f("tree_dist", base.tree_dist),
            building_dist: f("building_dist", base.building_dist),
            aniso: u("aniso", base.aniso),
            textures: u("textures", base.textures),
            vfx: u("vfx", base.vfx),
            bloom: f("bloom", base.bloom),
            lens_flare: b("lens_flare", base.lens_flare),
            lens_dirt: b("lens_dirt", base.lens_dirt),
            chroma: u("chroma", base.chroma),
            motion_blur: u("motion_blur", base.motion_blur),
            dof: u("dof", base.dof),
            grain: f("grain", base.grain),
        }
        .sanitized()
    }
}

/// The Video settings: a preset + its knobs, and the display options.
#[derive(Clone, Debug, PartialEq)]
pub struct VideoSettings {
    pub preset: Preset,
    pub q: Quality,
    pub window: WindowMode,
    /// Monitor name (None = the current one).
    pub monitor: Option<String>,
    /// Exclusive fullscreen mode: width, height, refresh (mHz); None = the monitor's native.
    pub mode: Option<(u32, u32, u32)>,
    pub present: Present,
    pub frame_cap: FrameCap,
    /// HDR output (scRGB swapchain) where the surface supports it.
    pub hdr: bool,
    /// HDR paper white (nits): the brightness of a white UI / diffuse white.
    pub paper_white: f32,
    /// Exposure compensation (EV, −2..+2).
    pub exposure_ev: f32,
    /// Brightness (black-level lift, −0.5..+0.5) and display gamma (1.8..2.6).
    pub brightness: f32,
    pub gamma: f32,
    /// Vertical field of view (degrees): chase / external cameras, and the cockpit.
    pub fov_external: f32,
    pub fov_cockpit: f32,
    /// Performance overlay (Ctrl+F3).
    pub perf_overlay: bool,
    /// The adapter auto-detect has picked a preset (first run).
    pub detected: bool,
}

pub const FOV_EXTERNAL: f32 = 60.0;
pub const FOV_COCKPIT: f32 = 72.0;
pub const GAMMA: f32 = 2.2;

impl Default for VideoSettings {
    fn default() -> Self {
        // (The browser starts light; the first run auto-detects either way.)
        let preset = if crate::platform::WEB { Preset::Low } else { Preset::High };
        VideoSettings {
            preset,
            q: Quality::preset(preset),
            window: WindowMode::Windowed,
            monitor: None,
            mode: None,
            present: Present::Fifo,
            frame_cap: FrameCap::Unlimited,
            hdr: false,
            paper_white: 200.0,
            exposure_ev: 0.0,
            brightness: 0.0,
            gamma: GAMMA,
            fov_external: FOV_EXTERNAL,
            fov_cockpit: FOV_COCKPIT,
            perf_overlay: false,
            detected: false,
        }
    }
}

impl VideoSettings {
    /// Pick a preset: all its knobs.
    pub fn apply_preset(&mut self, p: Preset) {
        self.preset = p;
        if p != Preset::Custom {
            self.q = Quality::preset(p);
        }
    }

    /// After a knob changed: Custom unless the knobs still equal a preset.
    pub fn knob_changed(&mut self) {
        self.q = self.q.sanitized();
        self.preset = self.q.matching_preset();
    }

    /// Reset the knobs to the selected preset's (Custom → High).
    pub fn reset_to_preset(&mut self) {
        let p = if self.preset == Preset::Custom { Preset::High } else { self.preset };
        self.apply_preset(p);
    }

    pub fn to_json(&self) -> Value {
        json!({
            "preset": self.preset.name(),
            "quality": self.q.to_json(),
            "window": self.window.key(),
            "monitor": self.monitor,
            "mode": self.mode.map(|(w, h, r)| json!([w, h, r])),
            "present": self.present.key(),
            "frame_cap": self.frame_cap.to_json(),
            "hdr": self.hdr,
            "paper_white": self.paper_white,
            "exposure_ev": self.exposure_ev,
            "brightness": self.brightness,
            "gamma": self.gamma,
            "fov_external": self.fov_external,
            "fov_cockpit": self.fov_cockpit,
            "perf_overlay": self.perf_overlay,
            "detected": self.detected,
        })
    }

    /// From a whole settings file (`root`): its `"video"` object, or — files
    /// written before the Video tab existed — the old `detail` (scenery
    /// level → the matching preset) and `vsync` keys.
    pub fn from_settings_json(root: &Value) -> VideoSettings {
        let d = VideoSettings::default();
        let Some(v) = root.get("video").filter(|v| v.is_object()) else {
            // Migration: the old scenery-detail level picks the preset; the
            // player chose it, so no auto-detect over it.
            let mut out = VideoSettings { detected: root.is_object(), ..d };
            if let Some(level) = root.get("detail").and_then(|x| x.as_f64()) {
                out.apply_preset(Preset::FIXED[(level as usize).min(3)]);
            }
            if root.get("vsync").and_then(|x| x.as_bool()) == Some(false) {
                out.present = Present::Mailbox;
            }
            return out;
        };
        let f = |k: &str, def: f32| v.get(k).and_then(|x| x.as_f64()).map(|x| x as f32).unwrap_or(def);
        let preset = v.get("preset").and_then(|x| x.as_str()).and_then(Preset::from_name).unwrap_or(d.preset);
        // Knobs over the preset's (keys added later default to the preset).
        let base = Quality::preset(preset);
        let q = match v.get("quality") {
            Some(qv) => Quality::from_json(qv, base),
            None => base,
        };
        let mode = v.get("mode").and_then(|m| m.as_array()).and_then(|a| Some((a.first()?.as_u64()? as u32, a.get(1)?.as_u64()? as u32, a.get(2)?.as_u64()? as u32)));
        VideoSettings {
            preset: if preset == Preset::Custom { Preset::Custom } else { q.matching_preset() },
            q,
            window: v.get("window").and_then(|x| x.as_str()).and_then(WindowMode::from_key).unwrap_or_default(),
            monitor: v.get("monitor").and_then(|x| x.as_str()).map(String::from),
            mode,
            present: v.get("present").and_then(|x| x.as_str()).and_then(Present::from_key).unwrap_or_default(),
            frame_cap: v.get("frame_cap").map(FrameCap::from_json).unwrap_or_default(),
            hdr: v.get("hdr").and_then(|x| x.as_bool()).unwrap_or(false),
            paper_white: f("paper_white", d.paper_white).clamp(80.0, 500.0),
            exposure_ev: f("exposure_ev", 0.0).clamp(-2.0, 2.0),
            brightness: f("brightness", 0.0).clamp(-0.5, 0.5),
            gamma: f("gamma", GAMMA).clamp(1.8, 2.6),
            fov_external: f("fov_external", FOV_EXTERNAL).clamp(30.0, 100.0),
            fov_cockpit: f("fov_cockpit", FOV_COCKPIT).clamp(40.0, 110.0),
            perf_overlay: v.get("perf_overlay").and_then(|x| x.as_bool()).unwrap_or(false),
            detected: v.get("detected").and_then(|x| x.as_bool()).unwrap_or(true),
        }
    }
}

// ---------------------------------------------------------------- auto-detect

/// The preset for an adapter: what holds ≥ 60 fps (≥ 120 on high-end GPUs) at the output size.
pub fn detect(info: &wgpu::AdapterInfo, web: bool) -> Preset {
    use wgpu::DeviceType as D;
    let name = info.name.to_ascii_lowercase();
    if web {
        // Browsers: the GPU type is often hidden; Medium only for a reported discrete GPU.
        return if info.device_type == D::DiscreteGpu { Preset::Medium } else { Preset::Low };
    }
    match info.device_type {
        D::Cpu | D::VirtualGpu => Preset::Low,
        D::IntegratedGpu => {
            // Apple M-series Pro / Max and the Radeon 780M / 890M class iGPUs hold Medium.
            if ["m1 max", "m2 max", "m3 max", "m4 max", "m1 pro", "m2 pro", "m3 pro", "m4 pro", "780m", "880m", "890m", "8060s", "arc 140v"].iter().any(|k| name.contains(k)) {
                Preset::Medium
            } else {
                Preset::Low
            }
        }
        D::DiscreteGpu | D::Other => {
            let ultra = ["rtx 4080", "rtx 4090", "rtx 5070 ti", "rtx 5080", "rtx 5090", "rx 7900", "rx 9070", "rtx 6000", "rtx a6000"];
            let high = ["rtx 30", "rtx 40", "rtx 50", "rtx 20", "rx 6", "rx 7", "arc a7", "arc b5", "arc b7", "titan"];
            // (Laptop parts run at a fraction of the desktop power: one step down.)
            let laptop = name.contains("laptop") || name.contains("mobile") || name.contains(" max-q");
            if ultra.iter().any(|k| name.contains(k)) && !laptop {
                Preset::Ultra
            } else if high.iter().any(|k| name.contains(k)) {
                Preset::High
            } else {
                Preset::Medium
            }
        }
    }
}

// ---------------------------------------------------------------- cost hints

/// A knob of the Video tab (for the cost hints).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Knob {
    Aa,
    RenderScale,
    Shadows,
    Cascades,
    ShadowDist,
    SoftShadows,
    Ao,
    Clouds,
    Fog,
    Reflections,
    Detail,
    Trees,
    Buildings,
    Aniso,
    Vfx,
    Bloom,
    LensFlare,
    MotionBlur,
    Dof,
    Grain,
}

/// Approximate GPU cost (ms) of a knob's current setting over its cheapest,
/// at `pixels` output pixels, measured on the reference GPU (RTX 5090 laptop,
/// 2560×1440, London; see README › Performance). Scene-resolution-bound costs
/// scale with the pixel count; geometry costs don't.
pub fn cost_ms(k: Knob, q: &Quality, pixels: f32) -> f32 {
    let px = pixels / (2560.0 * 1440.0);
    let sp = px * q.render_scale * q.render_scale;
    let lvl = |v: u8, t: [f32; 4]| t[(v as usize).min(3)];
    match k {
        Knob::Aa => match q.aa {
            AaMode::Off => 0.0,
            AaMode::Fxaa => 0.07 * sp,
            // (MSAA: the scene pass at 4 samples, ≈ +1.5 ms there, + the resolve.)
            AaMode::Msaa4 => 1.4 * sp,
            AaMode::Taa => 0.28 * px,
        },
        // Everything drawn at the scene resolution (≈ 3.5 ms of a London frame at 100 %).
        Knob::RenderScale => 3.5 * (q.render_scale * q.render_scale - 0.25) * px,
        Knob::Shadows => match q.shadow_res {
            1024 => 0.0,
            2048 => 0.15,
            _ => 0.4,
        },
        Knob::Cascades => 0.2 * (q.cascades as f32 - 2.0),
        // (Past 100 % the far cascade takes in far more of the city.)
        Knob::ShadowDist => 0.6 * (q.shadow_dist - 0.5) + 25.0 * (q.shadow_dist - 1.0).max(0.0).powi(2),
        Knob::SoftShadows => {
            if q.soft_shadows {
                0.2 * sp
            } else {
                0.0
            }
        }
        Knob::Ao => lvl(q.ao, [0.0, 0.07, 0.1, 0.12]) * sp,
        // (Under a cloudy sky; clear skies cost little at any setting.)
        Knob::Clouds => lvl(q.clouds, [0.0, 0.6, 1.2, 1.9]) * sp,
        Knob::Fog => lvl(q.fog, [0.0, 0.02, 0.04, 0.08]) * sp,
        Knob::Reflections => lvl(q.reflections, [0.0, 0.3, 2.5, 0.0]) * sp,
        Knob::Detail => lvl(q.detail, [0.0, 0.3, 0.7, 1.2]),
        Knob::Trees => 0.8 * (q.tree_density * q.tree_dist - 0.125),
        Knob::Buildings => 0.4 * (q.building_dist - 0.5),
        Knob::Aniso => [0.0, 0.0, 0.02, 0.05, 0.1][match q.aniso {
            1 => 0,
            2 => 1,
            4 => 2,
            8 => 3,
            _ => 4,
        }],
        Knob::Vfx => lvl(q.vfx, [0.0, 0.3, 0.6, 0.6]),
        Knob::Bloom => {
            if q.bloom > 0.0 {
                0.15 * px
            } else {
                0.0
            }
        }
        Knob::LensFlare => {
            if q.lens_flare {
                0.05 * px
            } else {
                0.0
            }
        }
        Knob::MotionBlur => lvl(q.motion_blur, [0.0, 0.35, 0.4, 0.4]) * px,
        Knob::Dof => {
            if q.dof > 0 {
                0.45 * px
            } else {
                0.0
            }
        }
        Knob::Grain => 0.0,
    }
}

/// "~0.6 ms" / "< 0.1 ms" / "free".
pub fn cost_label(ms: f32) -> String {
    if ms <= 0.005 {
        "free".into()
    } else if ms < 0.1 {
        "< 0.1 ms".into()
    } else {
        format!("~{ms:.1} ms")
    }
}

// ---------------------------------------------------------------- environment

/// What the Video tab needs from the app (refreshed by main.rs).
#[derive(Clone, Debug, Default)]
pub struct VideoEnv {
    pub adapter: String,
    /// Monitors: names.
    pub monitors: Vec<String>,
    /// Exclusive-fullscreen modes of the chosen monitor: (w, h, refresh mHz).
    pub modes: Vec<(u32, u32, u32)>,
    pub present_modes: Vec<wgpu::PresentMode>,
    pub hdr_supported: bool,
    /// The output size (px) for the cost hints.
    pub output: (u32, u32),
    /// The running quality (restart-required knobs compare against it).
    pub running: Option<Quality>,
    /// The running HDR state.
    pub running_hdr: bool,
    /// The current dynamic-resolution scale (when on).
    pub dyn_scale: Option<f32>,
    /// The auto-detected preset for this adapter.
    pub detected: Option<Preset>,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn presets_are_ordered_and_distinct() {
        let qs: Vec<Quality> = Preset::FIXED.iter().map(|p| p.quality()).collect();
        for (i, a) in qs.iter().enumerate() {
            for b in &qs[i + 1..] {
                assert_ne!(a, b);
            }
        }
        // Every knob is monotone non-decreasing Low → Ultra (more quality, more cost).
        for w in qs.windows(2) {
            let (a, b) = (&w[0], &w[1]);
            assert!(a.render_scale <= b.render_scale && a.shadow_res <= b.shadow_res && a.cascades <= b.cascades);
            assert!(a.shadow_dist <= b.shadow_dist && a.ao <= b.ao && a.clouds <= b.clouds && a.fog <= b.fog);
            assert!(a.reflections <= b.reflections && a.detail <= b.detail && a.tree_density <= b.tree_density);
            assert!(a.tree_dist <= b.tree_dist && a.building_dist <= b.building_dist && a.aniso <= b.aniso && a.vfx <= b.vfx);
            let cost = |q: &Quality| {
                [Knob::Aa, Knob::RenderScale, Knob::Shadows, Knob::Cascades, Knob::Ao, Knob::Clouds, Knob::Reflections, Knob::Detail, Knob::Trees]
                    .iter()
                    .map(|k| cost_ms(*k, q, 2560.0 * 1440.0))
                    .sum::<f32>()
            };
            assert!(cost(a) < cost(b), "{} vs {}", cost(a), cost(b));
        }
        // Effects that change the look by taste are off by default.
        for q in &qs {
            assert_eq!((q.motion_blur, q.dof, q.grain, q.chroma), (0, 0, 0.0, 0));
        }
        // None of the presets switches to MSAA (no restart between presets).
        assert!(qs.iter().all(|q| q.aa.samples() == 1));
    }

    #[test]
    fn preset_mapping_and_custom() {
        let mut v = VideoSettings::default();
        assert_eq!(v.preset, Preset::High);
        v.apply_preset(Preset::Low);
        assert_eq!(v.q.aa, AaMode::Fxaa);
        assert_eq!(v.q.shadow_res, 1024);
        assert_eq!(v.q.matching_preset(), Preset::Low);
        // Moving a knob → Custom; moving it back → the preset again.
        v.q.ao = 3;
        v.knob_changed();
        assert_eq!(v.preset, Preset::Custom);
        v.q.ao = 0;
        v.knob_changed();
        assert_eq!(v.preset, Preset::Low);
        // Reset from Custom goes to High.
        v.q.cascades = 3;
        v.knob_changed();
        v.reset_to_preset();
        assert_eq!(v.preset, Preset::High);
        assert_eq!(v.q, Quality::preset(Preset::High));
    }

    #[test]
    fn json_round_trip() {
        let mut v = VideoSettings {
            window: WindowMode::Borderless,
            monitor: Some("DP-1".into()),
            mode: Some((2560, 1440, 165_000)),
            present: Present::Mailbox,
            frame_cap: FrameCap::Fps(144),
            hdr: true,
            paper_white: 250.0,
            exposure_ev: 0.5,
            fov_cockpit: 80.0,
            perf_overlay: true,
            detected: true,
            ..VideoSettings::default()
        };
        v.apply_preset(Preset::Ultra);
        v.q.motion_blur = 2;
        v.knob_changed();
        let back = VideoSettings::from_settings_json(&json!({ "video": v.to_json() }));
        assert_eq!(back, v);
        assert_eq!(back.preset, Preset::Custom);
        let cap = VideoSettings { frame_cap: FrameCap::Display, ..VideoSettings::default() };
        assert_eq!(VideoSettings::from_settings_json(&json!({ "video": cap.to_json() })).frame_cap, FrameCap::Display);
    }

    #[test]
    fn migrates_old_settings_files() {
        // A file from before the Video tab: scenery detail 1, vsync off.
        let old = json!({ "ui_scale": 1.0, "hud": "Full", "vsync": false, "detail": 1 });
        let v = VideoSettings::from_settings_json(&old);
        assert_eq!(v.preset, Preset::Medium);
        assert_eq!(v.q, Quality::preset(Preset::Medium));
        assert_eq!(v.present, Present::Mailbox);
        assert!(v.detected, "an existing file is the player's choice: no auto-detect over it");
        // Defaults: detail 2 = High, vsync on.
        let v = VideoSettings::from_settings_json(&json!({ "detail": 2, "vsync": true }));
        assert_eq!((v.preset, v.present), (Preset::High, Present::Fifo));
        // No file at all: auto-detect pending.
        assert!(!VideoSettings::from_settings_json(&Value::Null).detected);
        // A partial / older "video" object: missing knobs come from its preset, junk is clamped.
        let v = VideoSettings::from_settings_json(&json!({ "video": { "preset": "Low", "quality": { "render_scale": 9.0, "aniso": 5, "shadow_res": 3000 } } }));
        assert_eq!(v.q.render_scale, 2.0);
        assert_eq!(v.q.aniso, 4);
        assert_eq!(v.q.shadow_res, 2048);
        assert_eq!(v.q.cascades, Quality::preset(Preset::Low).cascades);
        assert_eq!(v.preset, Preset::Custom);
    }

    #[test]
    fn overrides() {
        let q = Quality::preset(Preset::High).with_overrides("clouds=0, ao=3,aa=fxaa,lens_flare=on,render_scale=0.7,bogus=1");
        assert_eq!((q.clouds, q.ao, q.aa, q.lens_flare, q.render_scale), (0, 3, AaMode::Fxaa, true, 0.7));
        assert_eq!(q.cascades, Quality::preset(Preset::High).cascades);
        assert_eq!(Quality::default().with_overrides("aa=off").aa, AaMode::Off);
    }

    #[test]
    fn restart_only_for_msaa_and_samplers() {
        let a = Quality::preset(Preset::High);
        assert!(a.restart_diff(&Quality::preset(Preset::Low)).contains(&RestartReason::Textures));
        assert!(!a.restart_diff(&Quality { ao: 0, clouds: 0, render_scale: 0.5, ..a }).contains(&RestartReason::Msaa));
        assert_eq!(a.restart_diff(&Quality { aa: AaMode::Msaa4, ..a }), vec![RestartReason::Msaa]);
        assert!(a.restart_diff(&Quality { aa: AaMode::Fxaa, ..a }).is_empty());
    }

    #[test]
    fn present_mode_fallbacks() {
        use wgpu::PresentMode as P;
        assert_eq!(Present::Immediate.pick(&[P::Fifo, P::Mailbox]), P::Mailbox);
        assert_eq!(Present::Mailbox.pick(&[P::Fifo]), P::Fifo);
        assert_eq!(Present::Fifo.pick(&[P::Fifo, P::Immediate]), P::Fifo);
        assert_eq!(FrameCap::Display.hz(Some(165.0)), Some(165.0));
        assert_eq!(FrameCap::Unlimited.hz(Some(165.0)), None);
    }

    #[test]
    fn auto_detect() {
        let info = |name: &str, t: wgpu::DeviceType| wgpu::AdapterInfo { name: name.into(), ..wgpu::AdapterInfo::new(t, wgpu::Backend::Vulkan) };
        use wgpu::DeviceType as D;
        assert_eq!(detect(&info("NVIDIA GeForce RTX 5090", D::DiscreteGpu), false), Preset::Ultra);
        // Laptop parts: one step down (the 5090 laptop holds ≥ 120 fps at 1440p on High).
        assert_eq!(detect(&info("NVIDIA GeForce RTX 5090 Laptop GPU", D::DiscreteGpu), false), Preset::High);
        assert_eq!(detect(&info("NVIDIA GeForce RTX 3060", D::DiscreteGpu), false), Preset::High);
        assert_eq!(detect(&info("NVIDIA GeForce GTX 1050", D::DiscreteGpu), false), Preset::Medium);
        assert_eq!(detect(&info("Intel(R) Graphics (RPL-S)", D::IntegratedGpu), false), Preset::Low);
        assert_eq!(detect(&info("llvmpipe (LLVM 17.0.6, 256 bits)", D::Cpu), false), Preset::Low);
        assert_eq!(detect(&info("", D::Other), true), Preset::Low);
        assert_eq!(detect(&info("", D::DiscreteGpu), true), Preset::Medium);
    }
}
