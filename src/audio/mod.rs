//! Sound: recorded FlightGear engine / airframe / impact samples (see
//! `assets/audio/CREDITS.md`) layered with procedural, physically-informed
//! synthesis, driven continuously by the simulation, mixed positionally for
//! the camera (propagation delay, Doppler, Mach cone, sonic booms), played
//! through `cpal`.
//!
//! * [`samples`] decodes the embedded Ogg Vorbis samples off the audio
//!   thread; `soundsets` / `sampled` turn them into per-engine layered loop
//!   banks (FlightGear sound-config tables, equal-power cross-fades,
//!   close / distant variants, start-up / shutdown one-shots) with the
//!   procedural voices of `engines` underneath as detail.
//! * `crash` layers impacts, tearing, sliding, debris, fire and explosions
//!   by energy and surface — also an event API (`Event::Impact`, `Scrape`,
//!   `Separation`, `Fire`, `Explosion`) for debris physics.
//! * [`capture::Director`] (game thread) turns the sim into a [`params::Frame`]
//!   of continuous state plus one-shot [`params::Event`]s (gear locks,
//!   touchdowns, callouts, GPWS, chimes...).
//! * [`synth::Synth`] (audio thread, or the offline renderer) synthesises
//!   and mixes everything — see the module docs there and in `engines.rs`.
//! * The two meet in a mutex that the audio callback only ever `try_lock`s
//!   (it never blocks; on contention it keeps the previous frame).
//!
//! No audio device (headless / CI / no permission) is not an error: the
//! game runs silently and retries the device every few seconds.

pub mod capture;
mod crash;
pub mod dsp;
mod engines;
mod fx;
mod output;
pub mod params;
pub mod radio;
pub mod render;
mod rotorcraft;
mod sampled;
pub mod samples;
mod soundsets;
pub mod synth;
mod traffic;
#[cfg(test)]
mod scenarios;
#[cfg(test)]
mod tests;
mod voice;

pub use synth::Settings;

use output::Shared;
use params::{Event, Frame};
use std::sync::{Arc, Mutex};

pub struct Audio {
    shared: Arc<Mutex<Shared>>,
    out: Option<output::Output>,
    director: capture::Director,
    events: Vec<Event>,
    settings: Settings,
    retry_in: f32,
    status: String,
    clock: f32,
    stats_at: f32,
    /// Multiplayer: other aircraft, and the spectator's ear (overrides the sim's).
    traffic: [params::TrafficFrame; params::MAX_TRAFFIC],
    listener: Option<Listener>,
}

/// A listener placed by the caller (multiplayer spectator camera).
#[derive(Clone, Copy, Debug)]
pub struct Listener {
    pub pos: glam::Vec3,
    pub fwd: glam::Vec3,
    pub up: glam::Vec3,
}

impl Audio {
    /// Open the default output device. Never fails: without a device the
    /// game is silent (and the device is retried every few seconds).
    pub fn new(muted: bool) -> Audio {
        let settings = Settings { muted, ..Settings::default() };
        let shared = Arc::new(Mutex::new(Shared {
            frame: Frame::default(),
            seq: 0,
            events: Vec::with_capacity(256),
            settings,
            cpu_ns: 0,
            cpu_audio_s: 0.0,
        }));
        let mut a = Audio {
            shared,
            out: None,
            director: capture::Director::default(),
            events: Vec::with_capacity(64),
            settings,
            retry_in: 0.0,
            status: String::new(),
            clock: 0.0,
            stats_at: 10.0,
            traffic: Default::default(),
            listener: None,
        };
        a.try_open();
        a
    }

    fn try_open(&mut self) {
        match output::Output::open(self.shared.clone()) {
            Ok(o) => {
                self.status = format!("{} · {} Hz", o.name, o.sample_rate);
                eprintln!("audio: {}", self.status);
                self.out = Some(o);
            }
            Err(e) => {
                if self.status.is_empty() || !self.status.starts_with("No audio") {
                    eprintln!("audio: no output ({e}) — running silent");
                }
                self.status = format!("No audio device ({e})");
                self.out = None;
                self.retry_in = 5.0;
            }
        }
    }

    /// Device / state line for the menu.
    pub fn status(&self) -> &str {
        &self.status
    }

    pub fn set_settings(&mut self, s: Settings) {
        self.settings = s;
    }

    /// Other aircraft for the next `update` (multiplayer); `listener`: the
    /// spectator camera (no own aircraft) or None.
    pub fn set_traffic(&mut self, t: [params::TrafficFrame; params::MAX_TRAFFIC], listener: Option<Listener>) {
        self.traffic = t;
        self.listener = listener;
    }

    /// A UI click (menu).
    pub fn click(&mut self) {
        self.events.push(Event::UiClick);
    }

    /// Call once per video frame (after the frame's camera was computed).
    /// `sim` = None while loading. `menu` = the menu is open.
    pub fn update(&mut self, sim: Option<&crate::sim::Sim>, world: &crate::world::World, dt: f32, menu: bool) {
        self.clock += dt;
        // Device lost or never found: retry now and then.
        if self.out.as_ref().is_some_and(|o| o.failed()) {
            eprintln!("audio: output stream failed — reopening");
            self.out = None;
            self.retry_in = 1.0;
        }
        if self.out.is_none() {
            self.retry_in -= dt;
            if self.retry_in <= 0.0 {
                self.try_open();
            }
        }
        let mut frame = match (sim, self.listener) {
            (Some(_), Some(l)) => {
                // Spectating: the world from the spectator camera, no own aircraft.
                let right = l.fwd.cross(l.up).normalize_or(glam::Vec3::X);
                Frame { active: false, spectator: true, menu, warp: 1.0, ear_pos: l.pos, ear_right: right, ear_up: l.up, ear_fwd: l.fwd, ear_agl: 100.0, day: 1.0, ..Frame::default() }
            }
            (Some(s), None) => self.director.capture(s, world, dt, menu, &mut self.events),
            (None, _) => Frame { active: false, warp: 1.0, ..Frame::default() },
        };
        frame.traffic = self.traffic;
        if let Ok(mut sh) = self.shared.lock() {
            sh.frame = frame;
            sh.seq = sh.seq.wrapping_add(1);
            sh.settings = self.settings;
            if self.out.is_some() {
                sh.events.append(&mut self.events);
                // A stalled callback must not grow this without bound.
                let n = sh.events.len();
                if n > 256 {
                    sh.events.drain(..n - 256);
                }
            } else {
                self.events.clear();
            }
            if std::env::var_os("SKYWARD_AUDIO_STATS").is_some() && self.clock > self.stats_at && sh.cpu_audio_s > 0.0 {
                self.stats_at = self.clock + 10.0;
                eprintln!(
                    "audio: {:.3} ms CPU per 10 ms of audio ({:.1} % of one core)",
                    sh.cpu_ns as f64 / 1e6 / (sh.cpu_audio_s * 100.0),
                    sh.cpu_ns as f64 / 1e9 / sh.cpu_audio_s * 100.0
                );
            }
        }
    }
}

/// The "Sound" panel shown next to the menu. Returns true if the pointer
/// clicked anywhere this frame (for the UI click sound).
pub fn menu_window(ctx: &egui::Context, s: &mut Settings, status: &str) -> bool {
    egui::Window::new("Sound")
        .anchor(egui::Align2::RIGHT_BOTTOM, [-16.0, -16.0])
        .collapsible(true)
        .resizable(false)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.checkbox(&mut s.muted, "Mute (N)");
            });
            let row = |ui: &mut egui::Ui, v: &mut f32, label: &str, max: f32| {
                ui.add(egui::Slider::new(v, 0.0..=max).text(label).show_value(false));
            };
            row(ui, &mut s.master, "Master", 1.0);
            row(ui, &mut s.engines, "Engines", 1.5);
            row(ui, &mut s.environment, "Wind, ground & weather", 1.5);
            row(ui, &mut s.systems, "Cockpit systems", 1.5);
            row(ui, &mut s.voice, "Callouts", 1.5);
            row(ui, &mut s.ui, "Interface", 1.5);
            ui.label(egui::RichText::new(status).size(11.0).color(egui::Color32::GRAY));
        });
    ctx.input(|i| i.pointer.any_click())
}
