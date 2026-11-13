// Airport instances in the shadow cascades (airport/gfx.rs): eye-relative
// instance matrices, the cascade's eye-relative light matrix (shadow.wgsl).
struct ShadowU {
    light_vp: mat4x4<f32>,
    season: vec4<f32>,
};
@group(0) @binding(0) var<uniform> su: ShadowU;

@vertex
fn vs_ap_shadow(@location(0) pos: vec3<f32>, @location(5) m0: vec4<f32>, @location(6) m1: vec4<f32>, @location(7) m2: vec4<f32>, @location(8) m3: vec4<f32>) -> @builtin(position) vec4<f32> {
    let m = mat4x4<f32>(m0, m1, m2, m3);
    return su.light_vp * (m * vec4<f32>(pos, 1.0));
}
