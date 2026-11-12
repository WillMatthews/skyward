// Shared by every scene shader. Prepended (after atmos_core.wgsl) at load time.

struct Globals {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>, // eye-relative (rotation-only) inverse: d = normalize(p.xyz / p.w)
    shadow: array<mat4x4<f32>, 4>,
    cam_pos: vec4<f32>,   // xyz, w = time (s)
    sun_dir: vec4<f32>,   // xyz toward the sun, w = cloud cover 0..1
    sun_e: vec4<f32>,     // top-of-atmosphere sun irradiance rgb, w = haze
    splits: vec4<f32>,    // cascade far distances (m)
    shadow_info: vec4<f32>, // x map size, y = 1 in the reflection pass, z cascade blend fraction, w ao enabled
    screen: vec4<f32>,    // w, h, 1/w, 1/h of the current target
    cloud: vec4<f32>,     // cloud shadow map: origin x, z, 1/size, cloud base (m)
    texel: vec4<f32>,     // world size (m) of one shadow texel per cascade
    prev_view_proj: mat4x4<f32>, // last frame's camera (AO reprojection)
    wx: vec4<f32>,        // wetness, falling rain, falling snow, snow cover (0..1)
    wx2: vec4<f32>,       // surface wind x, z (m/s), hoar frost, gust (m/s)
    season: vec4<f32>,    // leaf cover, autumn colour, spring freshness, grass dormancy
    season2: vec4<f32>,   // summer dryness, year phase 0..1, season strength, drizzle
    pattern: vec4<f32>,   // pattern space of the eye: x scale, x, z (mod NP); w = y of the real eye
                          // above this pass's eye (2 eye.y in the mirrored reflection pass, else 0)
};

@group(0) @binding(0) var<uniform> g: Globals;
@group(0) @binding(1) var shadow_map: texture_depth_2d_array;
@group(0) @binding(2) var shadow_samp: sampler_comparison;
@group(0) @binding(3) var refl_tex: texture_2d<f32>;
@group(0) @binding(4) var lin_samp: sampler;
@group(0) @binding(5) var trans_tex: texture_2d<f32>;
@group(0) @binding(6) var sky_tex: texture_2d<f32>;
@group(0) @binding(7) var ap_tex: texture_3d<f32>;
@group(0) @binding(8) var cloud_shadow_tex: texture_2d<f32>;
struct AtmosOut {
    sh: array<vec4<f32>, 4>,
    sun_eye: vec4<f32>,
    expo: vec4<f32>,
};
@group(0) @binding(9) var<storage, read> atmos: AtmosOut;
@group(0) @binding(10) var ao_tex: texture_2d<f32>;

fn hash12(p: vec2<f32>) -> f32 {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

fn hash22(p: vec2<f32>) -> vec2<f32> {
    var p3 = fract(vec3<f32>(p.x, p.y, p.x) * vec3<f32>(0.1031, 0.1030, 0.0973));
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.xx + p3.yz) * p3.zy);
}

fn noise2(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    return mix(
        mix(hash12(i), hash12(i + vec2<f32>(1.0, 0.0)), u.x),
        mix(hash12(i + vec2<f32>(0.0, 1.0)), hash12(i + vec2<f32>(1.0, 1.0)), u.x),
        u.y
    );
}

fn fbm2(p: vec2<f32>) -> f32 {
    var a = 0.5;
    var s = 0.0;
    var q = p;
    for (var i = 0; i < 5; i++) {
        s += a * noise2(q);
        q = q * 2.03 + vec2<f32>(17.1, 3.7);
        a *= 0.5;
    }
    return s;
}

// ---------------------------------------------------------------- floating origin

// Rendering is eye-relative: every vertex is placed relative to the camera
// before the (rotation-only) view-projection, so nothing large is ever
// cancelled in f32 — no shake of the aircraft or the ground anywhere.
// Per-draw transform (immediate data): x' = (1 + xf.x) x + xf.y,
// y' = y + xf.w, z' = z + xf.z — a tile's frame → eye-relative (computed in
// f64 on the CPU), world → eye-relative (-eye), or zero for meshes that are
// already eye-relative (the aircraft).
var<immediate> xf: vec4<f32>;

fn tile_pos(p: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(p.x * (1.0 + xf.x) + xf.y, p.y + xf.w, p.z + xf.z);
}

// World-space procedural patterns live in "pattern space": fixed to the
// ground whatever the floating origin is (the CPU passes the frame's offset
// modulo NP, so values stay small and precise anywhere on Earth). Every
// pattern built from the helpers below is exactly NP-periodic, so the
// wrapped offset never shows.
const NP: f32 = 16384.0;
const TAU: f32 = 6.28318530718;

// Pattern space of an eye-relative position.
fn pat(rel: vec3<f32>) -> vec2<f32> {
    return vec2<f32>(rel.x * g.pattern.x + g.pattern.y, rel.z + g.pattern.z);
}

// Cells per period for ~f cycles per metre.
fn nper(f: f32) -> f32 {
    return max(round(NP * f), 1.0);
}

// Integer-valued cell coordinates wrapped into [0, per).
fn wrapc(c: vec2<f32>, per: f32) -> vec2<f32> {
    return c - per * floor((c + 0.5) / per);
}

// Value noise on a lattice that repeats every `per` cells.
fn pnoise_c(p: vec2<f32>, per: f32) -> f32 {
    let i = floor(p);
    let f = p - i;
    let u = f * f * (3.0 - 2.0 * f);
    let a = wrapc(i, per);
    let b = wrapc(i + 1.0, per);
    return mix(
        mix(hash12(a), hash12(vec2<f32>(b.x, a.y)), u.x),
        mix(hash12(vec2<f32>(a.x, b.y)), hash12(b), u.x),
        u.y
    );
}

// Value noise at pattern-space `np`, ~f cycles per metre, `off` in cells.
fn pn(np: vec2<f32>, f: f32, off: vec2<f32>) -> f32 {
    let per = nper(f);
    return pnoise_c(np * (per / NP) + off, per);
}

// ... fading to its mean below a pixel (fw = cells per pixel, as noise_aa).
fn pna(np: vec2<f32>, f: f32, off: vec2<f32>, fw: f32) -> f32 {
    return mix(pn(np, f, off), 0.5, smoothstep(0.35, 1.0, fw));
}

fn pfbm(np: vec2<f32>, f: f32, off: vec2<f32>) -> f32 {
    var per = nper(f);
    var q = np * (per / NP) + off;
    var a = 0.5;
    var s = 0.0;
    for (var i = 0; i < 5; i++) {
        s += a * pnoise_c(q, per);
        q = q * 2.0 + vec2<f32>(17.1, 3.7);
        per *= 2.0;
        a *= 0.5;
    }
    return s;
}

// Noise that fades to its mean once its features get smaller than a pixel
// (prevents shimmer on distant ground).
fn noise_aa(p: vec2<f32>, fw: f32) -> f32 {
    return mix(noise2(p), 0.5, smoothstep(0.35, 1.0, fw));
}

fn srgb_to_lin(c: vec3<f32>) -> vec3<f32> {
    return pow(max(c, vec3<f32>(0.0)), vec3<f32>(2.2));
}

fn is_refl() -> bool {
    return g.shadow_info.y > 0.5;
}

// ---------------------------------------------------------------- atmosphere

fn eye_r() -> f32 {
    return RG + max(abs(g.cam_pos.y) * 0.001, 0.002);
}

fn transmittance(h_m: f32, mu: f32) -> vec3<f32> {
    let r = RG + max(h_m, 0.0) * 0.001;
    if (hits_ground(r, mu)) {
        return vec3<f32>(0.0);
    }
    return textureSampleLevel(trans_tex, lin_samp, trans_uv(r, mu), 0.0).rgb;
}

// Sky radiance seen from the eye in direction d (includes the far planet ground).
fn sky_radiance(d: vec3<f32>) -> vec3<f32> {
    return g.sun_e.rgb * textureSampleLevel(sky_tex, lin_samp, skyview_uv(eye_r(), d, g.sun_dir.xyz), 0.0).rgb;
}

// What a mirror sees of the sky in direction d: the clear sky-view LUT
// blended toward the grey cloud deck by the cloud cover (the LUT has no clouds).
fn sky_reflection(d: vec3<f32>) -> vec3<f32> {
    let clear = sky_radiance(d);
    let cover = g.sun_dir.w;
    if (cover <= 0.0) {
        return clear;
    }
    let deck = vec3<f32>(luma(max(atmos.sh[0].rgb + atmos.sh[2].rgb, vec3<f32>(0.0)))) / PI * 1.1;
    return mix(clear, deck, cover * cover * smoothstep(-0.05, 0.15, d.y));
}

// Irradiance from the sky (L1 SH, includes cloud cover and ground bounce).
fn sky_irradiance(n: vec3<f32>) -> vec3<f32> {
    return max(atmos.sh[0].rgb + atmos.sh[1].rgb * n.x + atmos.sh[2].rgb * n.y + atmos.sh[3].rgb * n.z, vec3<f32>(0.0));
}

fn cloud_shadow(wp: vec3<f32>) -> f32 {
    let sun = g.sun_dir.xyz;
    let xz = wp.xz - sun.xz * (wp.y / max(sun.y, 0.05));
    let uv = (xz - g.cloud.xy) * g.cloud.z + 0.5;
    var s = textureSampleLevel(cloud_shadow_tex, lin_samp, uv, 0.0).r;
    s = mix(s, 1.0, smoothstep(0.42, 0.5, max(abs(uv.x - 0.5), abs(uv.y - 0.5))));
    return mix(s, 1.0, smoothstep(g.cloud.w - 100.0, g.cloud.w + 300.0, wp.y));
}

// Direct sun irradiance arriving at a point (atmospheric transmittance + cloud shadow).
fn sun_irradiance(wp: vec3<f32>) -> vec3<f32> {
    return g.sun_e.rgb * transmittance(wp.y, g.sun_dir.y) * cloud_shadow(wp);
}

// Aerial perspective: froxel lookup by screen position + distance.
fn apply_fog_px(col: vec3<f32>, px: vec2<f32>, dist: f32) -> vec3<f32> {
    let uv = px * g.screen.zw;
    let s = ap_slice_of(dist * 0.001);
    let ap = textureSampleLevel(ap_tex, lin_samp, vec3<f32>(uv, s), 0.0);
    // Fade in over the first froxel slice (the volume starts ~15 m out).
    let k = smoothstep(0.0, 1.0 / AP_N, s);
    return col * mix(1.0, ap.a, k) + ap.rgb * g.sun_e.rgb * k;
}

// ---------------------------------------------------------------- shadows

fn cascade_sample(idx: i32, wpos: vec3<f32>, n: vec3<f32>) -> f32 {
    let ndl = clamp(dot(n, g.sun_dir.xyz), 0.0, 1.0);
    let tx = g.texel[idx];
    // Normal offset scaled to the texel size (more at grazing light).
    let off = n * tx * (1.2 + 1.8 * (1.0 - ndl)) + g.sun_dir.xyz * tx * 0.6;
    let p = g.shadow[idx] * vec4<f32>(wpos + off, 1.0);
    let uv = vec2<f32>(p.x * 0.5 + 0.5, -p.y * 0.5 + 0.5);
    if (any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0)) || p.z > 1.0) {
        return 1.0;
    }
    let ts = 1.0 / g.shadow_info.x;
    // 16-tap bilinear PCF over a 5x5 texel footprint with tent weights (Castano).
    let base = uv * g.shadow_info.x - 0.5;
    let bf = floor(base);
    let f = base - bf;
    let c = (bf + 0.5) * ts;
    let z = p.z - 0.00002;
    let uw0 = 4.0 - 3.0 * f.x;
    let uw1 = 7.0;
    let uw2 = 1.0 + 3.0 * f.x;
    let u0 = (3.0 - 2.0 * f.x) / uw0 - 2.0;
    let u1 = (3.0 + f.x) / uw1;
    let u2 = f.x / uw2 + 2.0;
    let vw0 = 4.0 - 3.0 * f.y;
    let vw1 = 7.0;
    let vw2 = 1.0 + 3.0 * f.y;
    let v0 = (3.0 - 2.0 * f.y) / vw0 - 2.0;
    let v1 = (3.0 + f.y) / vw1;
    let v2 = f.y / vw2 + 2.0;
    var s = 0.0;
    s += uw0 * vw0 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u0, v0) * ts, idx, z);
    s += uw1 * vw0 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u1, v0) * ts, idx, z);
    s += uw2 * vw0 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u2, v0) * ts, idx, z);
    s += uw0 * vw1 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u0, v1) * ts, idx, z);
    s += uw1 * vw1 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u1, v1) * ts, idx, z);
    s += uw2 * vw1 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u2, v1) * ts, idx, z);
    s += uw0 * vw2 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u0, v2) * ts, idx, z);
    s += uw1 * vw2 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u1, v2) * ts, idx, z);
    s += uw2 * vw2 * textureSampleCompareLevel(shadow_map, shadow_samp, c + vec2<f32>(u2, v2) * ts, idx, z);
    return s / 144.0;
}

fn shadow_factor(wpos: vec3<f32>, n: vec3<f32>, view_dist: f32) -> f32 {
    if (is_refl()) {
        return 1.0;
    }
    if (view_dist > g.splits.w) {
        return 1.0;
    }
    var idx = 0;
    if (view_dist > g.splits.x) { idx = 1; }
    if (view_dist > g.splits.y) { idx = 2; }
    if (view_dist > g.splits.z) { idx = 3; }
    var s = cascade_sample(idx, wpos, n);
    // Blend into the next cascade over the last part of this one (no seams).
    let far = g.splits[idx];
    let near = select(0.0, g.splits[max(idx - 1, 0)], idx > 0);
    let bstart = far - (far - near) * g.shadow_info.z;
    if (view_dist > bstart) {
        let t = smoothstep(bstart, far, view_dist);
        var s2 = 1.0;
        if (idx < 3) {
            s2 = cascade_sample(idx + 1, wpos, n);
        }
        s = mix(s, s2, t);
    }
    return s;
}

// Last frame's SSAO, reprojected to this point.
fn ssao(wp: vec3<f32>) -> f32 {
    if (g.shadow_info.w < 0.5 || is_refl()) {
        return 1.0;
    }
    let c = g.prev_view_proj * vec4<f32>(wp, 1.0);
    let uv = vec2<f32>(c.x / c.w * 0.5 + 0.5, 0.5 - c.y / c.w * 0.5);
    if (c.w <= 0.0 || any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0))) {
        return 1.0;
    }
    return textureSampleLevel(ao_tex, lin_samp, uv, 0.0).r;
}
