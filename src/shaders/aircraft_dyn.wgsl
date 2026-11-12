// Structural animation of the aircraft (gfx_aircraft.rs `AcDynamics`):
// first-mode wing bending (+ bend-twist), ground vibration, and per-part
// transforms for pieces that have broken off.
struct AcDyn {
    up: vec4<f32>,                  // body up (eye-relative), w = time
    flex: vec4<f32>,                // tip deflection L, R (m), twist per m, -
    vib: vec4<f32>,                 // amplitude (m), frequency (Hz)
    part_flex: array<vec4<f32>, 4>, // 1 = part bends with the wing
    parts: array<mat4x4<f32>, 16>,  // intact → part (eye-relative)
};

// Displacement (eye-relative world) of a vertex with structural attributes `a`.
fn ac_flex(a: vec4<f32>) -> vec3<f32> {
    let k = min(u32(a.x + 0.5), 15u);
    let pf = ac_dyn.part_flex[k / 4u][k % 4u];
    let s = abs(a.y);
    // Uniformly loaded cantilever: w(s) = s^2 (6 - 4 s + s^2) / 3, w(1) = 1.
    let w = s * s * (6.0 - 4.0 * s + s * s) / 3.0;
    let tip = select(ac_dyn.flex.y, ac_dyn.flex.x, a.y >= 0.0);
    var d = (tip * w + ac_dyn.flex.z * tip * s * a.z) * pf;
    // Vibration (rolling on the ground): wings flap more than the fuselage.
    let t = ac_dyn.up.w;
    d += ac_dyn.vib.x * sin(t * 6.2831853 * ac_dyn.vib.y + a.z * 0.9 + a.y * 2.0) * (0.25 + 1.5 * s) * a.w;
    return ac_dyn.up.xyz * d;
}
