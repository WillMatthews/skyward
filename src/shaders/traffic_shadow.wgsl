// Road traffic in the shadow cascades (traffic/gpu.rs): eye-relative
// instances, the cascade's eye-relative light matrix (shadow.wgsl).
struct ShadowU {
    light_vp: mat4x4<f32>,
    season: vec4<f32>,
};
@group(0) @binding(0) var<uniform> su: ShadowU;

@vertex
fn vs_veh_shadow(@location(0) pos: vec3<f32>, @location(5) ipos: vec3<f32>, @location(6) dir: vec2<f32>, @location(7) grade: f32) -> @builtin(position) vec4<f32> {
    let h = vec3<f32>(dir.x, 0.0, dir.y);
    let fwd = normalize(h + vec3<f32>(0.0, clamp(grade, -0.3, 0.3), 0.0));
    let left = normalize(cross(vec3<f32>(0.0, 1.0, 0.0), fwd));
    let up = cross(fwd, left);
    return su.light_vp * vec4<f32>(ipos + left * pos.x + up * pos.y + fwd * pos.z, 1.0);
}
