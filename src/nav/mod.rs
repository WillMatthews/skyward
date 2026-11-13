//! Navigation: navaid database ([`db`]), radios with reception and Morse
//! idents ([`radio`], [`morse`]), the flight plan / FMS-lite ([`fms`]) and
//! the per-flight state the sim owns ([`state`]). Geometry in [`geo`].
//!
//! The autopilot's NAV / LOC / VOR / APPR / VNAV modes (autopilot.rs) couple
//! to [`state::NavState::ap_input`]; the cockpit instruments, PFD / ND /
//! MCDU and the radio panel read the receivers' outputs.

pub mod db;
pub mod fms;
pub mod geo;
pub mod morse;
pub mod procedures;
pub mod radio;
pub mod state;
#[cfg(test)]
mod tests;

pub use state::NavState;
