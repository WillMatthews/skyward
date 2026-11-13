// Road-traffic velocity pass (traffic/gpu.rs `draw_velocity`, for TAA /
// motion blur): each vehicle fragment writes its screen motion since last
// frame (uv then − uv now, unjittered), its eye distance now and last frame.
// A vehicle was `vel × dt` back along the road last frame; eye-relative
// positions shift by the eye's own motion (`eye_dt.xyz`).

struct VelU {
    view_proj: mat4x4<f32>,      // this frame, jittered (matches the main pass)
    view_proj_now: mat4x4<f32>,  // this frame, unjittered
    view_proj_prev: mat4x4<f32>, // last frame's, unjittered, relative to last frame's eye
    eye_dt: vec4<f32>,           // this eye − last frame's eye (m), w = frame interval (s)
};
@group(0) @binding(0) var<uniform> vu: VelU;

struct TInst {
    @location(5) pos: vec3<f32>,
    @location(6) dir: vec2<f32>,
    @location(7) grade: f32,
    @location(8) col: u32,
    @location(9) info: u32,
    @location(10) vel: vec4<f32>, // world velocity (m/s)
};

struct VelOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) now: vec4<f32>,
    @location(1) prev: vec4<f32>,
    @location(2) rel: vec3<f32>,
    @location(3) prel: vec3<f32>,
};

@vertex
fn vs_vel(@location(0) vpos: vec3<f32>, i: TInst) -> VelOut {
    // (== traffic.wgsl veh_frame)
    let h = vec3<f32>(i.dir.x, 0.0, i.dir.y);
    let fwd = normalize(h + vec3<f32>(0.0, clamp(i.grade, -0.3, 0.3), 0.0));
    let left = normalize(cross(vec3<f32>(0.0, 1.0, 0.0), fwd));
    let up = cross(fwd, left);
    let p = i.pos + left * vpos.x + up * vpos.y + fwd * vpos.z;
    let q = p + vu.eye_dt.xyz - i.vel.xyz * vu.eye_dt.w;
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
    return vec4<f32>(vec2<f32>(b.x - a.x, a.y - b.y) * 0.5, length(in.rel), length(in.prel));
}
