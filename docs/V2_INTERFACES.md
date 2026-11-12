# Skyward v2 — interface contract (read before coding)

Repo: `/home/wam/coding/fs98`. v1 is working (see `README.md`, `ROADMAP.md`).
v2 is being built in parallel by several agents; **each owns specific files
only**. Do not edit files you do not own, and do not run cargo inside the repo
while others are working (develop in your own scratch crate and copy your file(s)
in at the end, as v1 did). Never open a window or display anything on screen —
the user is working on this machine; render offscreen and inspect PNGs.

| Track | Owner of | Summary |
|---|---|---|
| A flight | `src/flight.rs`, `src/autopilot.rs` | v2 aircraft dynamics + autopilot |
| B models | `src/models.rs` (may split into `src/models/*.rs` + `src/models.rs` facade) | v2 aircraft meshes |
| C clouds | `src/clouds.rs`, `src/shaders/clouds.wgsl` | volumetric clouds + thermals |
| D integration (orchestrator) | everything else | tile streaming, renderer, HUD, sim glue |

## Conventions (unchanged from v1)
- World: metres, +X east, -Z north, +Y up, flat ground at y = 0 (terrain may come later — always ask the environment for ground height).
- Body: +Z nose, +Y up, -X right wing, +X left wing, origin = CG. Heading 0 = north, 90 = east. `rot: Quat` body→world.
- Physics fixed step dt = 1/120 s.
- Vertex/mesh type: `scene::MeshData` / `scene::Vert` (pos, nrm, col, uv, mat). Materials in `scene::mat` (now also `EMISSIVE = 13` with intensity in `uv.x`, and `METAL = 14`). Vertex colours from models are LINEAR (the app sRGB-encodes them on load — keep doing linear).

## Aircraft roster (ids are the contract)
| id | name | class | real-world analogue (original design, no real branding/markings/registrations) |
|---|---|---|---|
| kestrel | Kestrel C1 | Trainer | (v1) Cessna-172 class |
| swift | Swift S2 | Glider | (v1) 15 m sailplane |
| peregrine | Peregrine Mk IX | Warbird | Spitfire Mk IX class: elliptical wing, ~1,500 hp V12, 4-blade prop, taildragger |
| lancer | Lancer F2 | FastJet | F-16 / Gripen class single-engine fighter with afterburner, FBW |
| aerolink | Aerolink 72 | Airliner (turboprop) | ATR 72 class: high wing, twin 6-blade turboprops, T-tail |
| meridian190 | Meridian 190 | Airliner (jet) | E190 class: low wing, twin underwing turbofans |
| meridian320 | Meridian 320 | Airliner (jet) | A320 class: low wing, twin underwing turbofans, wingtip fences/sharklets |

### Shared geometry (body frame, metres) — A's contact points and B's meshes MUST agree
| id | span | length | gear | wheel contact points (x, y, z) | engine / prop pivots | cockpit eye |
|---|---|---|---|---|---|---|
| peregrine | 11.2 | 9.1 | taildragger, retractable mains, fixed tailwheel | mains (±0.90, -1.75, 0.90); tail (0, -0.55, -5.40) → rests ~10.8° nose-up | prop pivot (0, 0.10, 4.60), 4 blades, radius 1.60 | (0, 0.75, -0.40) |
| lancer | 9.5 | 15.0 | tricycle, retractable | nose (0, -1.90, 4.80); mains (±1.30, -1.90, -0.60) | nozzle exit (0, 0.0, -7.2) | (0, 0.95, 4.20) |
| aerolink | 27.0 | 27.2 | tricycle, retractable | nose (0, -2.00, 10.50); mains (±2.00, -2.00, -0.90) | props (±4.10, 0.90, 4.10), 6 blades, radius 1.95 | (0.55, 1.05, 11.60) |
| meridian190 | 28.7 | 36.2 | tricycle, retractable | nose (0, -2.40, 13.00); mains (±2.70, -2.40, -1.50) | fan faces (±5.00, -1.20, 3.80) | (0.55, 1.15, 15.40) |
| meridian320 | 35.8 | 37.6 | tricycle, retractable | nose (0, -2.60, 13.30); mains (±3.80, -2.60, -1.40) | fan faces (±5.75, -1.60, 4.20) | (0.55, 1.25, 15.80) |

(Cockpit eye x>0 = left seat. Kestrel eye (0.32, 0.62, 0.55), Swift (0, 0.42, 1.55).)

## Track A — flight API additions (all v1 API must keep working)
```rust
pub enum Category { Trainer, Glider, Warbird, FastJet, Airliner }
pub enum EngineKind { None, Piston, Turboprop, Jet }
// new AircraftDef fields (pub): category, engine_kind, engine_count: usize,
//   retractable_gear: bool, has_spoilers: bool, has_reverser: bool, has_afterburner: bool,
//   taildragger: bool, eye: Vec3 (cockpit eye, table above), vr_kt, vref_kt, vle_kt (gear limit),
//   ceiling_m.  catalogue() returns all 7 in the roster order above.
// Controls new fields: gear_down: bool (default true), spoilers: f32 0..1, reverse: bool,
//   failed_engines: u8 bitmask (for engine-failure practice).
// Throttle 0..1; for has_afterburner, throttle > 0.9 is the reheat zone.
// Aircraft new methods: engine_count(), engine_rpm_i(i) (props) / n1(i) 0..~1.05 (jets),
//   egt(i) degC, fuel_flow(i) kg/h (optional), gear_pos() 0 = up .. 1 = down,
//   mach(), afterburner() 0..1.  Keep engine_rpm() = engine 0.
// Surfaces gains: gear (0 = down/extended .. 1 = stowed — NOTE the inverse of gear_pos, it is
//   the *retraction* amount used as part deflection), spoilers 0..1, reverser 0..1,
//   afterburner 0..1, prop_angles: [f32; 4], and keeps prop_angle (= engine 0).
```
- Gear-up touchdown → crash "Gear-up landing" (belly). Gear transit ~6 s (light) to ~10 s (airliner).
- Taildragger: real 3-point ground handling, torque + P-factor + slipstream swirl → swings left on take-off roll unless right rudder; tail rises with forward stick at speed; nose-over risk with heavy braking.
- Jets: spool-up lag (airliner ~5–8 s idle→TOGA, fighter faster), thrust lapse with Mach/altitude, Mach drag rise; air density with altitude (ISA) for everyone now.
- Airliners: realistic weights (ATR 72 ~ 21 t landing, E190 ~ 43 t, A320 ~ 64 t), Vref ~ 113/124/135 kt, flaps detents in degrees for the type (A320-like 0/1/2/3/FULL may be shown as degrees), spoilers (air + ground), reversers (ground only).
- Fast jet: FBW-ish (rate command / g-limit ~9 g, AoA limit) so it is flyable on keyboard; afterburner; supersonic capable.
- Asymmetric thrust with failed engines (yaw + roll) for multi-engine.
- `src/autopilot.rs`: `pub struct Autopilot { pub engaged: bool, pub hdg: Option<f32 /*rad*/>, pub alt: Option<f32 /*m*/>, pub ias: Option<f32 /*m/s*/>, pub vs: Option<f32 /*m/s*/> }` with
  `fn engage_hold_current(&mut self, ac: &Aircraft)`, `fn update(&mut self, def, ac, ctl: &mut Controls, dt) -> bool /*still engaged*/` (it writes elevator/aileron/rudder/throttle/trim; the app calls it after reading the keyboard; app disengages AP on big pilot input). Autothrottle only when `ias` is Some and aircraft has an engine.
- Tests for every new aircraft (stall, Vr/takeoff distance, climb, cruise/max speed, landing, gear-up crash, spoilers/reversers effect, engine-out controllability, Peregrine swing, Lancer supersonic + g-limit, autopilot captures & holds alt/hdg/speed).

## Track B — models API v2
```rust
pub enum PartKind {
    AileronLeft, AileronRight, Elevator, Rudder, FlapLeft, FlapRight, Airbrakes,
    Spoilers,              // deflection 0..1 = surfaces.spoilers (may be several parts)
    Propeller(u8),         // engine index; axis = thrust axis; app passes prop_angles[i]; uv.x on PROP verts: 0 blade, 1 disc (as v1)
    GearNose, GearMain, GearTail, GearDoors, // deflection = surfaces.gear (0 down .. 1 stowed); multiple parts per kind allowed
    Reverser,              // deflection = surfaces.reverser
    Afterburner,           // shown only when afterburner > 0.02; app scales EMISSIVE uv.x by the afterburner level
    Strobe, Beacon,        // EMISSIVE; app blinks them
}
pub struct AircraftModel { pub body: MeshData, pub parts: Vec<Part>, pub eye: Vec3 }
pub fn build(id: &str) -> AircraftModel;   // all 7 ids
```
Part transform unchanged: `translate(pivot) * rotation(axis, deflection * max_angle)` with mesh in part-local coords (origin at pivot). Gear legs + wheels must fold into wells and doors close when deflection = 1. Navigation lights: EMISSIVE (red +X tip, green -X tip, white tail). Liveries: original fictional (e.g. "Skyward Air" style) for airliners, original military camouflage without real national insignia for Peregrine/Lancer.

## Track C — clouds API
```rust
pub enum Weather { Clear, Few, Scattered, Broken, Overcast }
pub struct CloudField { /* deterministic world-space field */ }
impl CloudField {
    pub fn new(seed: u32, weather: Weather) -> Self;
    pub fn updraft(&self, p: glam::Vec3) -> f32;        // m/s, consistent with where the cumulus are drawn
    pub fn thermals_near(&self, xz: glam::Vec2, radius: f32) -> Vec<(glam::Vec2, f32 /*radius*/, f32 /*strength*/)>;
    pub fn base(&self) -> f32;                          // cloud base (m)
}
pub struct CloudFrame { pub view_proj: Mat4, pub inv_view_proj: Mat4, pub eye: Vec3, pub sun_dir: Vec3,
                        pub sun_col: Vec3, pub sky_intensity: f32, pub time: f32, pub width: u32, pub height: u32,
                        pub fog: [f32; 2] /* density, height falloff — see common.wgsl apply_fog */ }
pub struct CloudRenderer { .. }
impl CloudRenderer {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Self;
    pub fn set_field(&mut self, field: &CloudField);
    /// Composite clouds in place over `hdr` (Rgba16Float, RENDER_ATTACHMENT|TEXTURE_BINDING, resolved),
    /// using `dist` (R16Float, same size: distance from eye in metres to the opaque surface; sky = 65000).
    /// `slot` distinguishes independent targets (0 = main view, 1 = half-res water reflection) so
    /// temporal history is kept per target.
    pub fn render(&mut self, device, queue, encoder, frame: &CloudFrame, hdr: &wgpu::TextureView, dist: &wgpu::TextureView, slot: usize);
    /// Top-down cloud-shadow transmittance (0 = full shadow .. 1 = clear) around the eye, updated in render(slot 0).
    pub fn shadow_view(&self) -> &wgpu::TextureView;   // R16Float or R8Unorm, filterable
    pub fn shadow_params(&self) -> [f32; 4];             // origin_x, origin_z, inv_size, unused  → uv = (xz - origin) * inv_size + 0.5
}
```
- Sky/lighting must match the app: copy `sky_color` and `apply_fog` from `src/shaders/common.wgsl`; sun radiance (3.3, 3.05, 2.75), sky intensity 1.15, sun elevation ~38°.
- Look: real cumulus (base ~1100 m, tops 1.5–3 km by weather), Beer–Lambert + powder + HG dual-lobe phase + multi-scattering approximation, dark flat-ish bases, bright silver linings, aerial perspective; optional thin cirrus layer high up. Flying through a cloud must look right (camera inside the layer).
- Budget: ≤ 1.5 ms main view at 1920×1080 on the RTX 5090 (reduced-res raymarch + temporal + depth-aware upsample is fine), stable without ghosting when the camera moves at 250 m/s or rotates fast.
