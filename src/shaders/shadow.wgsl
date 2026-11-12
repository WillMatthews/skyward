struct ShadowU {
    light_vp: mat4x4<f32>,
    season: vec4<f32>, // x = deciduous leaf cover (bare winter trees cast thin shadows)
};
@group(0) @binding(0) var<uniform> su: ShadowU;
// Eye-relative placement (see common.wgsl tile_pos); light_vp is eye-relative too.
@group(1) @binding(15) var<uniform> xf: vec4<f32>;

fn tile_pos(p: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(p.x * (1.0 + xf.x) + xf.y, p.y + xf.w, p.z + xf.z);
}

@vertex
fn vs_shadow(@location(0) pos: vec3<f32>) -> @builtin(position) vec4<f32> {
    return su.light_vp * vec4<f32>(tile_pos(pos), 1.0);
}

@vertex
fn vs_shadow_tree(@location(0) pos: vec3<f32>, @location(3) uv: vec2<f32>, @location(5) pos_h: vec4<f32>, @location(6) tint_rot: vec4<f32>) -> @builtin(position) vec4<f32> {
    let c = cos(tint_rot.w);
    let s = sin(tint_rot.w);
    let wk = 0.8 + 0.45 * fract(sin(dot(pos_h.xz, vec2<f32>(12.9898, 78.233))) * 43758.5453);
    // Leafless deciduous crowns (== tree_leaf in scene.wgsl) shrink to a thin core.
    let seed = fract(pos_h.x * 0.137 + pos_h.z * 0.071);
    let k = seed * 0.75;
    let leaf = select(smoothstep(k, k + 0.3, su.season.x * 1.05), 1.0, uv.x > 0.5);
    let shrink = mix(0.4, 1.0, leaf);
    let lp = pos * vec3<f32>(pos_h.w * wk * shrink, pos_h.w, pos_h.w * wk * shrink);
    let p = tile_pos(pos_h.xyz) + vec3<f32>(c * lp.x - s * lp.z, lp.y, s * lp.x + c * lp.z);
    return su.light_vp * vec4<f32>(p, 1.0);
}
