// Aircraft velocity pass (gfx_aircraft.rs `draw_velocity`, for TAA / motion
// blur): each aircraft fragment writes its screen motion since last frame
// (uv then − uv now, unjittered), its eye distance now and its eye distance
// last frame (relative to last frame's eye). aircraft_dyn.wgsl is prepended.
// Everything is eye-relative, so floating-origin shifts don't matter.

struct VelU {
    view_proj: mat4x4<f32>,      // this frame, jittered (rasterisation; matches the main pass)
    view_proj_now: mat4x4<f32>,  // this frame, unjittered
    view_proj_prev: mat4x4<f32>, // last frame's, unjittered, relative to last frame's eye
};
struct AcInst {
    model: mat4x4<f32>,
    color: vec4<f32>,
    emis: vec4<f32>,
    params: vec4<f32>,
    uvm: vec4<f32>,
    uvo: vec4<f32>,
};
@group(0) @binding(0) var<uniform> vu: VelU;
@group(1) @binding(0) var<storage, read> ac_inst: array<AcInst>;
@group(1) @binding(1) var<storage, read> ac_dyn: AcDyn;
// Last frame's node → eye-relative transform per instance (= this frame's when new).
@group(2) @binding(0) var<storage, read> ac_prev: array<mat4x4<f32>>;

struct VelOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) now: vec4<f32>,
    @location(1) prev: vec4<f32>,
    @location(2) rel: vec3<f32>,
    @location(3) prel: vec3<f32>,
};

@vertex
fn vs_vel(@location(0) pos: vec3<f32>, @location(1) part: vec4<f32>, @builtin(instance_index) ii: u32) -> VelOut {
    let pm = ac_dyn.parts[min(u32(part.x + 0.5), 15u)];
    let fl = ac_flex(part);
    let p = (pm * vec4<f32>((ac_inst[ii].model * vec4<f32>(pos, 1.0)).xyz + fl, 1.0)).xyz;
    let q = (pm * vec4<f32>((ac_prev[ii] * vec4<f32>(pos, 1.0)).xyz + fl, 1.0)).xyz;
    var o: VelOut;
    o.clip = vu.view_proj * vec4<f32>(p, 1.0);
    o.now = vu.view_proj_now * vec4<f32>(p, 1.0);
    o.prev = vu.view_proj_prev * vec4<f32>(q, 1.0);
    o.rel = p;
    o.prel = q;
    return o;
}

@fragment
fn fs_vel(in: VelOut) -> @location(0) vec4<f32> {
    let a = in.now.xy / in.now.w;
    let b = in.prev.xy / max(in.prev.w, 1e-5);
    let mv = vec2<f32>(b.x - a.x, a.y - b.y) * 0.5;
    return vec4<f32>(mv, length(in.rel), length(in.prel));
}
