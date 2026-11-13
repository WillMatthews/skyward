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
    // Night & lights. NB: sun_dir / sun_e above are the *key* light (the sun,
    // or the moon at night); the real sun is sun_true.
    sun_true: vec4<f32>,  // xyz toward the sun, w = night factor 0 (day) .. 1 (night)
    moon: vec4<f32>,      // xyz toward the moon, w = illuminated fraction
    sky: vec4<f32>,       // x sky-LUT scale, y moon disc radiance, z earthshine radiance, w pixels per radian
    stars0: vec4<f32>,    // columns of the equatorial -> world rotation
    stars1: vec4<f32>,
    stars2: vec4<f32>,
    city: vec4<f32>,      // x street lights on, y office windows lit, z home windows lit, w local hours
    lmap: vec4<f32>,      // ground light map: eye-relative centre x, z, 1 / size, enabled
    misc: vec4<f32>,      // x airport lights level, y beacons on, z ground fog extinction (/km), w fog top (m)
    misc2: vec4<f32>,     // x traffic density, y aircraft lights count, z moon phase angle, w simulated traffic radius (m, 0 = none)
    alights: array<vec4<f32>, 12>, // aircraft lights: [pos, range], [dir, cos outer], [rgb cd, cos inner]
    pre: vec4<f32>,       // x pre-exposure: every radiance written to the HDR targets is x this
                          // (sun_e, sky.x and the SH ambient already include it), y 1 / x,
                          // z mirror plane height, w = 0 with MSAA, else 1 + frame index (alpha dither)
    gq: vec4<f32>,        // quality (gfx/quality.rs): x soft shadows (1 = PCF)
};

// Absolute radiance (scene units) → what the HDR targets hold.
fn pre_exposed(c: vec3<f32>) -> vec3<f32> {
    return c * g.pre.x;
}

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
@group(0) @binding(11) var light_map: texture_2d<f32>;

// Scene irradiance per lux (and scene radiance per cd/m²): sun_e (2.5) ≈ 128 klx.
const LUX: f32 = 1.953125e-5;

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
// Per-draw transform (gfx.rs Xforms: a uniform with a dynamic offset per
// draw — WebGPU has no push constants): x' = (1 + xf.x) x + xf.y,
// y' = y + xf.w, z' = z + xf.z — a tile's frame → eye-relative (computed in
// f64 on the CPU), world → eye-relative (-eye), or zero for meshes that are
// already eye-relative (the aircraft).
@group(1) @binding(15) var<uniform> xf: vec4<f32>;

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

// Without MSAA (TAA / FXAA) there is no alpha-to-coverage: partly
// transparent surfaces (foliage twigs, prop discs) keep a fraction `a` of
// their pixels instead — interleaved gradient noise shifted every frame, so
// TAA resolves the dither into the blend. True = discard this fragment.
fn alpha_dither(a: f32, px: vec2<f32>) -> bool {
    if (g.pre.w < 0.5 || a >= 0.999) {
        return false;
    }
    let q = px + vec2<f32>(5.588238, 3.236068) * g.pre.w;
    let n = fract(52.9829189 * fract(dot(q, vec2<f32>(0.06711056, 0.00583715))));
    return a <= n;
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
    return g.sky.x * textureSampleLevel(sky_tex, lin_samp, skyview_uv(eye_r(), d, g.sun_dir.xyz), 0.0).rgb;
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
    return col * mix(1.0, ap.a, k) + ap.rgb * g.sky.x * k;
}

// ---------------------------------------------------------------- artificial light

// Street-light irradiance on a surface (ground light map, lights.rs): the
// map holds the irradiance on the ground; walls facing a lit street see part
// of it, fading up the facade above the lamps; roofs see none.
fn street_light(rel: vec3<f32>, n: vec3<f32>) -> vec3<f32> {
    if (g.lmap.w < 0.5) {
        return vec3<f32>(0.0);
    }
    let y = rel.y + g.cam_pos.y;
    // Walls sample the street in front of them.
    let off = n.xz * (3.5 * (1.0 - abs(n.y)));
    let uv = (rel.xz + off - g.lmap.xy) * g.lmap.z + 0.5;
    if (any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0))) {
        return vec3<f32>(0.0);
    }
    let e = textureSampleLevel(light_map, lin_samp, uv, 0.0).rgb;
    let edge = 1.0 - smoothstep(0.42, 0.5, max(abs(uv.x - 0.5), abs(uv.y - 0.5)));
    // Up-facing surfaces near the ground get it all; walls ~half, fading with height.
    let up = clamp(n.y, 0.0, 1.0);
    let ground = up * (1.0 - smoothstep(2.0, 9.0, y));
    let wall = (1.0 - up) * 0.55 * exp(-max(y - 3.0, 0.0) / 7.0) * step(-0.2, n.y);
    return e * (ground + wall) * edge * g.pre.x;
}

// Irradiance-weighted direction + colour of the aircraft's lights at a point
// (eye-relative): spot cones / point lights, inverse-square, no shadows.
struct ALight {
    l: vec3<f32>,   // unit vector toward the light
    e: vec3<f32>,   // irradiance on a surface facing it (scene units)
};

fn aircraft_light(i: i32, rel: vec3<f32>) -> ALight {
    var o: ALight;
    let lp = g.alights[i * 3].xyz;
    let range = g.alights[i * 3].w;
    let ld = g.alights[i * 3 + 1].xyz;
    let cos_o = g.alights[i * 3 + 1].w;
    let cd = g.alights[i * 3 + 2].rgb;
    let cos_i = g.alights[i * 3 + 2].w;
    let v = lp - rel;
    let d2 = max(dot(v, v), 0.04);
    let d = sqrt(d2);
    o.l = v / d;
    var cone = 1.0;
    if (cos_o > -0.99) {
        cone = smoothstep(cos_o, cos_i, dot(-o.l, ld));
    }
    let fade = 1.0 - smoothstep(range * 0.7, range, d);
    o.e = cd * (LUX * cone * fade / d2) * g.pre.x;
    return o;
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
    if (g.gq.x < 0.5) {
        // Hard shadows (low quality): one bilinear comparison (2x2 PCF).
        return textureSampleCompareLevel(shadow_map, shadow_samp, uv, idx, p.z - 0.00002);
    }
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
