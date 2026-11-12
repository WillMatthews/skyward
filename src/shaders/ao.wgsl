// Screen-space ambient occlusion from the resolved eye-distance buffer
// (half resolution), then a depth-aware blur. The main pass samples last
// frame's result (reprojected) to occlude only the sky / ambient light.

struct AoU {
    inv_view_proj: mat4x4<f32>,
    eye: vec4<f32>,      // xyz eye, w frame index
    params: vec4<f32>,   // x radius (m), y px per (m at 1 m distance), z strength, w unused
    size: vec4<f32>,     // full w, h, ao w, h
};

@group(0) @binding(0) var<uniform> A: AoU;
@group(0) @binding(1) var dist_tex: texture_2d<f32>;
@group(0) @binding(2) var ao_src: texture_2d<f32>;

struct FsOut {
    @builtin(position) pos: vec4<f32>,
};

@vertex
fn vs_full(@builtin(vertex_index) i: u32) -> FsOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: FsOut;
    o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    return o;
}

fn ray(px: vec2<f32>) -> vec3<f32> {
    let uv = px / A.size.xy;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let p = A.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    return normalize(p.xyz / p.w); // eye-relative inverse
}

fn dist_at(px: vec2<f32>) -> f32 {
    let d = vec2<i32>(textureDimensions(dist_tex));
    return textureLoad(dist_tex, clamp(vec2<i32>(px), vec2<i32>(0), d - 1), 0).x;
}

// Eye-relative position of a full-res pixel.
fn pos_at(px: vec2<f32>) -> vec3<f32> {
    return ray(px) * dist_at(px);
}

@fragment
fn fs_ao(in: FsOut) -> @location(0) vec4<f32> {
    let px = (floor(in.pos.xy) + 0.5) * 2.0;
    let d = dist_at(px);
    if (d > 20000.0) {
        return vec4<f32>(1.0);
    }
    let p = ray(px) * d;
    // Normal from the smaller of the forward / backward differences.
    let s = 2.0;
    let pr = pos_at(px + vec2<f32>(s, 0.0)) - p;
    let pl = p - pos_at(px - vec2<f32>(s, 0.0));
    let pd = pos_at(px + vec2<f32>(0.0, s)) - p;
    let pu = p - pos_at(px - vec2<f32>(0.0, s));
    let dx = select(pl, pr, dot(pr, pr) < dot(pl, pl));
    let dy = select(pu, pd, dot(pd, pd) < dot(pu, pu));
    var n = normalize(cross(dy, dx));
    if (dot(n, -p) < 0.0) {
        n = -n;
    }
    let R = A.params.x;
    let rpx = clamp(R * A.params.y / d, 1.5, 90.0);
    if (rpx < 1.6) {
        return vec4<f32>(1.0);
    }
    // Interleaved gradient noise rotates the sample spiral per pixel.
    let ign = fract(52.9829189 * fract(dot(in.pos.xy, vec2<f32>(0.06711056, 0.00583715))));
    let N = 12;
    var occ = 0.0;
    for (var i = 0; i < N; i++) {
        let t = (f32(i) + 0.5) / f32(N);
        let a = (f32(i) * 2.39996 + ign * 6.2832);
        let r = rpx * sqrt(t) * (0.7 + 0.6 * fract(ign + f32(i) * 0.618));
        let q = pos_at(px + vec2<f32>(cos(a), sin(a)) * r);
        let v = q - p;
        let vv = dot(v, v);
        let fall = 1.0 - smoothstep(R * R * 0.5, R * R, vv);
        occ += max(dot(v, n) - 0.02 * d * 0.01 - 0.05, 0.0) / (vv + 0.05 * R * R) * fall;
    }
    let ao = clamp(1.0 - A.params.z * occ * R / f32(N) * 2.0, 0.0, 1.0);
    return vec4<f32>(ao, d, 0.0, 1.0);
}

@fragment
fn fs_blur(in: FsOut) -> @location(0) vec4<f32> {
    let c = vec2<i32>(in.pos.xy);
    let dims = vec2<i32>(textureDimensions(ao_src));
    let cd = textureLoad(ao_src, c, 0).y;
    var acc = 0.0;
    var wsum = 0.0;
    for (var y = -2; y <= 2; y++) {
        for (var x = -2; x <= 2; x++) {
            let s = textureLoad(ao_src, clamp(c + vec2<i32>(x, y), vec2<i32>(0), dims - 1), 0);
            let w = exp(-abs(s.y - cd) / max(cd * 0.02, 0.05)) * (1.0 - 0.1 * f32(abs(x) + abs(y)));
            acc += s.x * w;
            wsum += w;
        }
    }
    return vec4<f32>(acc / max(wsum, 1e-4), 0.0, 0.0, 1.0);
}
