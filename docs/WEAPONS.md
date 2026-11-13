# Weapons & air combat (Track W)

`src/weapons/` — guns, missiles, countermeasures, sensors, damage, ejection
and AI opponents. Only fighters / warbirds are armed (`guns::loadout`):
the Spitfire IX (`peregrine`) and the F-16C (`lancer`), plus generic fits
for any future `Warbird` / `FastJet` type. Air-to-air only.

## Module map

| File | What |
|---|---|
| `mod.rs` | `Combat`: units, rounds, missiles, decoys, chutes; the fixed-step `step`, hit resolution, kill credit, `apply` for remote events |
| `ballistics.rs` | Round data (Hispano, .303, PGU-28, .50), Mach-dependent drag, fly-out tables |
| `guns.rs` | Loadouts, gun installations, rate / spin-up / ammunition, harmonisation, dispersion |
| `missiles.rs` | AIM-9M / AIM-120C: motor, drag, g limits, PN guidance, IR / radar seekers, flares / chaff / notch, fuze, fragments |
| `sensors.rs` | Fire-control radar (search / STT, notch), RWR, the AIM-9 seeker on the rail |
| `hitbox.rs` | Per-part hit volumes from the model's part boxes and wing planform; segment intersection |
| `bdmg.rs` | Battle damage → `damage.rs` (weakened spars, separations) and `systems` failures; fires, glycol, pilot |
| `eject.rs` | ACES II seat and Spitfire bail-out, parachute descent, jettisoned canopy |
| `ai.rs` | BFM tactics + lift-vector control of the real flight model; difficulty presets |
| `setup.rs` | Dogfight quick start (1v1 / 2v2), respawns, score |
| `render.rs` | Other aircraft's model frames, missiles, chutes, seats, holes, tracer / flare particles |
| `events.rs` | `CombatEvent` + versioned binary encoding |

Glue elsewhere (all small): `sim.rs` (`Sim::combat`, keys, stepping,
camera, respawn, `combat_moment` for captures), `gfx.rs` (`FrameView::others`
drawn with extra `AircraftRenderer`s), `vfx/weapons.rs`, `audio/weapons.rs`
(+ `params::Event` variants), `ui/weapons.rs` (HUD / gunsight / RWR / feed),
`ui/menus.rs` (Dogfight card), `damage.rs` (`Damage::weaken`), `input/*`
(keys, `SHARED` bindings), `main.rs` (CLI).

## Multiplayer (wired: `src/net/combat.rs`, `skyward-net` `CombatMsg`)

**Model: client-detected, server-arbitrated, target-applied.**
* Each client runs its own `Combat` with `Authority::Client`. Every other
  player is an `external` unit (pose written each frame from the interpolated
  network state; no physics, drawn by MP's traffic renderer), so rounds,
  seekers and fuzes see them.
* The **shooter's** client detects its own hits (`resolves(shooter)` is true
  only for the local player) and, instead of applying them, sends one
  `CombatMsg::Hit` claim per target / part / weapon per frame (target body
  frame, damage units, HE). Others' rounds / missiles are regenerated
  locally as visuals (`Fire { seed }` → identical dispersion cone).
* The **server** (`Server::combat_check`) relays combat messages only if
  combat is on, the sender isn't spectating, a hit's target exists and is
  flying, the target's last known position is within the weapon's reach of
  the shooter's (3 km guns, 80 km missiles), the damage is bounded
  (`MAX_HIT_DAMAGE`) and the sender is within its rate limit (150 / s, burst
  400). It keeps kills / deaths per player and logs them. No echo to the sender.
* The **target's** client applies validated hits to its own aircraft
  (`Combat::apply(Hit)` → `bdmg` → structure / systems), then reports
  `Damage` (others show its holes / smoke / fire) and, when it goes down,
  `Kill { killer }` (credit from its own last hit) and `Eject`.

| `CombatMsg` | Sent by | Server | Receivers |
|---|---|---|---|
| `Fire { gun, rounds, seed }` | shooter, batched 0.1 s | combat on, rate limit | visual rounds, tracers, sound |
| `Launch { id, kind, target }` | shooter | combat on | visual missile from the shooter's pose (guided locally) |
| `MissileEnd { id }` | shooter | relay | burst effect |
| `Countermeasure { flare }` | owner | combat on | decoy (fools *this* client's own missiles too) |
| `Hit { target, weapon, part, pos, normal, damage, he }` | shooter | **validated** | target applies; others show the strike |
| `Damage { hp, lost, failures, fire, leak }` | target | relay | battle-damage state of that player |
| `Kill { killer, weapon }` | victim | relay + score | kill feed |
| `Eject` | pilot | relay | seat / parachute |

Wire: `Msg::Combat { from, msg }` (reliable, type 70); `PROTOCOL` = 2.

**Server combat flag:** `WorldSync::combat` (default on) — `skyward-server
--combat on|off`, TOML `combat = true|false`, console `combat on|off`
(broadcast at once). It shows in the server browser's details ("COMBAT") and
conditions summary; clients set `Rules::enabled` from it every frame.

Limits: no lag compensation beyond the server's range check (claims are
judged against the latest states); a missile's guidance on other clients is a
local re-simulation (corrected only by its `MissileEnd`); the dogfight AI is
single-player only.

## Numbers (published → modelled, see the tests)
* Hispano Mk II: 880 m/s, 130 g; 400 m in 0.53 s with 1.24 m drop; the Mk V
  reference round reaches 1,000 yd in 1.59 s (published ≈ 1.66 s).
* M61A1: 6,000 rpm after ≈ 0.5 s spin-up, 511 rounds; Spitfire 120 rpg cannon /
  350 rpg .303, 600 / 1,150 rpm.
* AIM-9M burnout Mach 3.0 fired at Mach 0.8 (published "Mach 2.5+"); hits a
  6 g breaking target at 2.5 km rear quarter and 6 km head-on, misses a 14 km
  tail chase and inside minimum range; one flare decoys ≈ 35 % of shots, four
  ≈ 85 %, a non-IRCCM seeker 100 %.
* AIM-120C: 30 km head-on shot with datalink, active inside 15 km, hits through a
  6 g terminal break; a 40 km tail chase is outside the envelope.
* ACES II: zero-delay low-altitude mode, main canopy ≈ 1.6 s after firing, ≈ 6–7 m/s
  descent; mode 3 holds the main above 14,000 ft.
