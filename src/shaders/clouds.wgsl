// Volumetric cumulus clouds (Track C). atmos_core.wgsl is prepended: the
// clouds are lit by the same physically based sun/sky and hazed by the same
// aerial-perspective volume as the scene (atmosphere.rs).

struct CloudU {
    view_proj: mat4x4<f32>,      // eye-relative (rotation only)
    inv_view_proj: mat4x4<f32>,  // eye-relative: ray = normalize(p.xyz / p.w)
    prev_view_proj: mat4x4<f32>, // last frame's, relative to this frame's eye
    eye: vec4<f32>,      // xyz, w = time (s)
    sun_dir: vec4<f32>,  // xyz toward the sun, w = sky intensity
    sun_col: vec4<f32>,  // rgb sun radiance, w = temporal blend (1 = no history)
    fog: vec4<f32>,      // x density, y height falloff, z frame jitter 0..1, w slot
    screen: vec4<f32>,   // full w, h, trace w, h
    wmap: vec4<f32>,     // origin x, origin z, m per texel, texels
    shadow: vec4<f32>,   // origin x, origin z, 1 / size (m), m per texel
    shape: vec4<f32>,    // base, layer top, sigma, stratus thickness
    cells: vec4<f32>,    // cell size, probability, r_min, r_max
    cells2: vec4<f32>,   // h_min, h_max, stratus coverage, cirrus amount
    seed: vec4<u32>,     // seed, frame, cumulus visible, 0
    layers: vec4<f32>,   // stratus base, mid-layer coverage param, mid base, mid thickness
    drift: vec4<f32>,    // field drift x, z (m: clouds move with the wind), lowest base, 0
    gfog: vec4<f32>,     // ground fog extinction (/km), fog top (km), cirrus extra drift x, z (m)
};

@group(0) @binding(0) var<uniform> U: CloudU;
@group(0) @binding(1) var shape_tex: texture_3d<f32>;
@group(0) @binding(2) var detail_tex: texture_3d<f32>;
@group(0) @binding(3) var weather_tex: texture_2d<f32>;
@group(0) @binding(4) var dist_tex: texture_2d<f32>;
@group(0) @binding(5) var trace_col: texture_2d<f32>;
@group(0) @binding(6) var trace_dep: texture_2d<f32>;
@group(0) @binding(7) var hist_col: texture_2d<f32>;
@group(0) @binding(8) var hist_dep: texture_2d<f32>;
@group(0) @binding(9) var samp_rep: sampler;
@group(0) @binding(10) var samp_clamp: sampler;
@group(0) @binding(12) var shadow_tex: texture_2d<f32>;
@group(0) @binding(13) var trans_tex: texture_2d<f32>;
@group(0) @binding(14) var sky_tex: texture_2d<f32>;
@group(0) @binding(15) var ap_tex: texture_3d<f32>;
struct AtmosOut {
    sh: array<vec4<f32>, 4>,
    sun_eye: vec4<f32>,
    expo: vec4<f32>,
};
@group(0) @binding(16) var<storage, read> atmos: AtmosOut;

const MAX_T: f32 = 42000.0;
const FAR_DEPTH: f32 = 60000.0;
const SHAPE_SCALE: f32 = 1.0 / 1500.0;   // metres -> shape-noise uvw
const DETAIL_SCALE: f32 = 1.0 / 180.0;   // metres -> detail-noise uvw

// ------------------------------------------------------------------ atmosphere

// U.fog: x = top-of-atmosphere sun irradiance, y = haze, z = jitter, w = slot.
fn sun_e() -> f32 {
    return U.fog.x;
}

fn sky_color(d: vec3<f32>) -> vec3<f32> {
    let r = RG + max(abs(U.eye.y) * 0.001, 0.002);
    return sun_e() * textureSampleLevel(sky_tex, samp_clamp, skyview_uv(r, d, U.sun_dir.xyz), 0.0).rgb;
}

fn sky_irr(n: vec3<f32>) -> vec3<f32> {
    return max(atmos.sh[0].rgb + atmos.sh[1].rgb * n.x + atmos.sh[2].rgb * n.y + atmos.sh[3].rgb * n.z, vec3<f32>(0.0));
}

fn sun_trans(h_m: f32, mu: f32) -> vec3<f32> {
    let r = RG + max(h_m, 0.0) * 0.001;
    if (hits_ground(r, mu)) {
        return vec3<f32>(0.0);
    }
    return textureSampleLevel(trans_tex, samp_clamp, trans_uv(r, mu), 0.0).rgb;
}

// Aerial perspective from the eye to `dist` metres along the pixel at `uv`:
// rgb = in-scattered radiance, a = transmittance.
fn ap_at(uv: vec2<f32>, dist: f32) -> vec4<f32> {
    let a = textureSampleLevel(ap_tex, samp_clamp, vec3<f32>(uv, ap_slice_of(dist * 0.001)), 0.0);
    return vec4<f32>(a.rgb * sun_e(), a.a);
}

// ------------------------------------------------------------------ hashing (== clouds.rs)

fn lowbias32(x0: u32) -> u32 {
    var x = x0;
    x ^= x >> 16u;
    x *= 0x7feb352du;
    x ^= x >> 15u;
    x *= 0x846ca68bu;
    x ^= x >> 16u;
    return x;
}

fn cell_hash(seed: u32, ix: i32, iz: i32, k: u32) -> u32 {
    return lowbias32(lowbias32(lowbias32(seed ^ bitcast<u32>(ix)) + bitcast<u32>(iz)) + k * 0x9e3779b9u);
}

fn u01(h: u32) -> f32 {
    return f32(h >> 8u) * (1.0 / 16777216.0);
}

fn bump(r: f32) -> f32 {
    let x = max(1.0 - r * r, 0.0);
    return x * x;
}

fn remap(x: f32, lo: f32, hi: f32, nlo: f32, nhi: f32) -> f32 {
    return nlo + (x - lo) * (nhi - nlo) / (hi - lo);
}

// ------------------------------------------------------------------ fullscreen

struct FsOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) uv: vec2<f32>,
};

@vertex
fn vs_full(@builtin(vertex_index) i: u32) -> FsOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: FsOut;
    o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    o.uv = vec2<f32>(uv.x, 1.0 - uv.y);
    return o;
}

// ------------------------------------------------------------------ 3D noise generation

fn pcg3d(v0: vec3<u32>) -> vec3<u32> {
    var v = v0 * 1664525u + 1013904223u;
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v ^= v >> vec3<u32>(16u);
    v.x += v.y * v.z;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    return v;
}

fn rand3(c: vec3<i32>, period: i32, salt: u32) -> vec3<f32> {
    let w = ((c % period) + period) % period;
    return vec3<f32>(pcg3d(vec3<u32>(w) + vec3<u32>(salt, salt * 3u + 11u, salt * 7u + 23u))) * (1.0 / 4294967296.0);
}

// Tileable Worley, inverted (1 at feature points).
fn worley(p: vec3<f32>, period: i32, salt: u32) -> f32 {
    let id = vec3<i32>(floor(p));
    let f = fract(p);
    var md = 9.0;
    for (var z = -1; z <= 1; z++) {
        for (var y = -1; y <= 1; y++) {
            for (var x = -1; x <= 1; x++) {
                let o = vec3<i32>(x, y, z);
                let d = vec3<f32>(o) + rand3(id + o, period, salt) - f;
                md = min(md, dot(d, d));
            }
        }
    }
    return 1.0 - clamp(sqrt(md), 0.0, 1.0);
}

// Tileable gradient noise, roughly -1..1.
fn gnoise(p: vec3<f32>, period: i32, salt: u32) -> f32 {
    let i = vec3<i32>(floor(p));
    let f = fract(p);
    let u = f * f * f * (f * (f * 6.0 - 15.0) + 10.0);
    var v: array<f32, 8>;
    for (var k = 0; k < 8; k++) {
        let o = vec3<i32>(k & 1, (k >> 1) & 1, (k >> 2) & 1);
        let g = normalize(rand3(i + o, period, salt) * 2.0 - 1.0 + vec3<f32>(1e-4));
        v[k] = dot(g, f - vec3<f32>(o));
    }
    let x0 = mix(v[0], v[1], u.x);
    let x1 = mix(v[2], v[3], u.x);
    let x2 = mix(v[4], v[5], u.x);
    let x3 = mix(v[6], v[7], u.x);
    return mix(mix(x0, x1, u.y), mix(x2, x3, u.y), u.z) * 1.6;
}

fn worley_fbm(p: vec3<f32>, f: i32, salt: u32) -> f32 {
    return worley(p * f32(f), f, salt) * 0.625 + worley(p * f32(f * 2), f * 2, salt + 1u) * 0.25 + worley(p * f32(f * 4), f * 4, salt + 2u) * 0.125;
}

@group(0) @binding(11) var noise_out: texture_storage_3d<rgba8unorm, write>;

@compute @workgroup_size(4, 4, 4)
fn cs_shape_noise(@builtin(global_invocation_id) id: vec3<u32>) {
    let n = textureDimensions(noise_out).x;
    let p = (vec3<f32>(id) + 0.5) / f32(n);
    var pf = 0.0;
    var amp = 1.0;
    var fq = 4;
    for (var o = 0; o < 4; o++) {
        pf += gnoise(p * f32(fq), fq, 100u + u32(o)) * amp;
        amp *= 0.5;
        fq *= 2;
    }
    let perlin = clamp(pf * 0.42 + 0.5, 0.0, 1.0);
    let w4 = worley_fbm(p, 4, 10u);
    let pw = clamp(remap(perlin, w4 - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    let w8 = worley_fbm(p, 8, 20u);
    let w16 = worley_fbm(p, 16, 30u);
    // Normalise to use the full 0..1 range of the texture.
    let wn = clamp((vec3<f32>(w4, w8, w16) - 0.22) / 0.56, vec3<f32>(0.0), vec3<f32>(1.0));
    textureStore(noise_out, id, vec4<f32>(clamp((pw - 0.45) / 0.47, 0.0, 1.0), wn));
}

@compute @workgroup_size(4, 4, 4)
fn cs_detail_noise(@builtin(global_invocation_id) id: vec3<u32>) {
    let n = textureDimensions(noise_out).x;
    let p = (vec3<f32>(id) + 0.5) / f32(n);
    let a = worley_fbm(p, 2, 40u);
    let b = worley_fbm(p, 4, 50u);
    let c = worley(p * 8.0, 8, 60u);
    let v = clamp((vec3<f32>(a, b, c) - vec3<f32>(0.24, 0.22, 0.1)) / vec3<f32>(0.56, 0.56, 0.8), vec3<f32>(0.0), vec3<f32>(1.0));
    textureStore(noise_out, id, vec4<f32>(v, 1.0));
}

// ------------------------------------------------------------------ weather map

fn value_noise2(p: vec2<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let ii = vec2<i32>(i);
    let s = U.seed.x;
    let a = u01(cell_hash(s, ii.x, ii.y, 101u));
    let b = u01(cell_hash(s, ii.x + 1, ii.y, 101u));
    let c = u01(cell_hash(s, ii.x, ii.y + 1, 101u));
    let d = u01(cell_hash(s, ii.x + 1, ii.y + 1, 101u));
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

fn fbm2(p0: vec2<f32>) -> f32 {
    var p = p0;
    var s = 0.0;
    var a = 0.5;
    for (var i = 0; i < 5; i++) {
        s += value_noise2(p) * a;
        p = p * 2.03 + vec2<f32>(17.3, -9.1);
        a *= 0.5;
    }
    return s / 0.96875;
}

// RGBA: cumulus coverage, cumulus height (km), stratus coverage, mid-layer
// coverage. Baked in field space (world xz - drift).
@fragment
fn fs_weather(in: FsOut) -> @location(0) vec4<f32> {
    let res = U.wmap.w;
    let xz = U.wmap.xy + (floor(in.pos.xy) + 0.5 - res * 0.5) * U.wmap.z;
    let cell = U.cells.x;
    let seed = U.seed.x;
    let ci = vec2<i32>(floor(xz / cell));
    var cov = 0.0;
    var hsum = 0.0;
    var wsum = 0.0;
    for (var dz = -1; dz <= 1; dz++) {
        for (var dx = -1; dx <= 1; dx++) {
            let ix = ci.x + dx;
            let iz = ci.y + dz;
            if (u01(cell_hash(seed, ix, iz, 0u)) >= U.cells.y) {
                continue;
            }
            let jx = 0.2 + 0.6 * u01(cell_hash(seed, ix, iz, 1u));
            let jz = 0.2 + 0.6 * u01(cell_hash(seed, ix, iz, 2u));
            let size = u01(cell_hash(seed, ix, iz, 3u));
            let hv = u01(cell_hash(seed, ix, iz, 4u));
            let center = vec2<f32>((f32(ix) + jx) * cell, (f32(iz) + jz) * cell);
            let radius = U.cells.z + (U.cells.w - U.cells.z) * size;
            let height = U.cells2.x + (U.cells2.y - U.cells2.x) * (size * 0.7 + hv * 0.3);
            // Main body over the thermal + 4 turrets (== CloudField::cell).
            for (var j = 0u; j < 5u; j++) {
                var bc = center;
                var br = radius * 0.8;
                var bh = height;
                if (j > 0u) {
                    let k = j - 1u;
                    let a = u01(cell_hash(seed, ix, iz, 5u + k)) * 6.28318530718;
                    let d = radius * (0.25 + 0.45 * u01(cell_hash(seed, ix, iz, 9u + k)));
                    br = radius * (0.35 + 0.3 * u01(cell_hash(seed, ix, iz, 13u + k)));
                    bh = height * (0.35 + 0.6 * u01(cell_hash(seed, ix, iz, 17u + k)));
                    bc = center + vec2<f32>(cos(a), sin(a)) * d;
                }
                let b = bump(distance(xz, bc) / br);
                cov = max(cov, b);
                let w = b * b * b;
                hsum += w * bh;
                wsum += w;
            }
        }
    }
    let h = select(0.5 * (U.cells2.x + U.cells2.y), hsum / max(wsum, 1e-6), wsum > 1e-6);
    if (U.seed.z == 0u) {
        cov = 0.0;
    }
    var strato = 0.0;
    if (U.cells2.z > 0.0) {
        let n = fbm2(xz / 6500.0);
        strato = clamp((n + U.cells2.z - 0.92) * 3.2, 0.0, 1.0);
    }
    var mid = 0.0;
    if (U.layers.y > 0.0) {
        let n = fbm2(xz / 9000.0 + vec2<f32>(37.0, -11.0));
        mid = clamp((n + U.layers.y - 0.92) * 3.2, 0.0, 1.0);
    }
    // Fade toward the map border (circular, so no square edge is visible).
    let r = length(xz - U.wmap.xy) / (U.wmap.z * res * 0.5);
    let fade = 1.0 - smoothstep(0.80, 0.98, r);
    return vec4<f32>(cov * fade, h / 1000.0, strato * fade, mid * fade);
}

// Weather at world xz (the map is in field space: subtract the wind drift).
fn weather(xz: vec2<f32>) -> vec4<f32> {
    let uv = (xz - U.drift.xy - U.wmap.xy) / (U.wmap.z * U.wmap.w) + 0.5;
    var w = textureSampleLevel(weather_tex, samp_clamp, uv, 0.0);
    // A full overcast deck continues past the map to the horizon.
    let r = length(uv - 0.5) * 2.0;
    let deck = clamp((U.cells2.z - 0.85) * 6.6, 0.0, 1.0);
    if (deck > 0.0) {
        w.z = mix(w.z, deck, smoothstep(0.78, 0.98, r));
    }
    let mdeck = clamp((U.layers.y - 0.85) * 6.6, 0.0, 1.0);
    if (mdeck > 0.0) {
        w.w = mix(w.w, mdeck, smoothstep(0.78, 0.98, r));
    }
    return w;
}

// ------------------------------------------------------------------ density

// Noise offset: the field drifts with the wind; a slow extra drift keeps the
// shapes evolving.
fn wind_shape() -> vec3<f32> {
    let t = U.eye.w;
    return vec3<f32>(U.drift.x, 0.0, U.drift.y) + vec3<f32>(t * 0.9, -t * 0.5, t * 0.6);
}

// Cloud density (0..~1) at p. `w` = weather at p.xz. `lod` 0 = full detail,
// 1 = base shape only (light / shadow rays).
fn density(p: vec3<f32>, w: vec4<f32>, lod: i32) -> f32 {
    let base = U.shape.x;
    let y = p.y - base;
    if (p.y < U.drift.z - 40.0 || p.y > U.shape.y) {
        return 0.0;
    }
    // Cumulus envelope: flat base, dome whose height follows the footprint.
    var env = 0.0;
    var hc = 1.0;
    if (w.x > 0.003 && y > -40.0) {
        let H = max(w.y * 1000.0, 50.0);
        hc = y / H;
        // Rounded dome on top, near-vertical sides, flat base with tucked-in edges.
        // Widest a little above the base, base edges tucked in, rounded top.
        let tuck = (1.0 - smoothstep(0.0, 0.22, hc)) * 0.2;
        let dome = sqrt(w.x) - pow(hc, 1.6) - tuck;
        env = clamp(dome * 1.25, 0.0, 1.0);
    }
    // Stratocumulus / stratus deck.
    var es = 0.0;
    if (w.z > 0.003) {
        let hs = (p.y - U.layers.x) / U.shape.w;
        es = w.z * clamp(hs * 5.0 + 0.2, 0.0, 1.0) * clamp((1.0 - hs) * 3.0, 0.0, 1.0);
    }
    // Mid-level layer (altocumulus / altostratus).
    if (w.w > 0.003) {
        let hm = (p.y - U.layers.z) / U.layers.w;
        es = max(es, w.w * clamp(hm * 5.0 + 0.2, 0.0, 1.0) * clamp((1.0 - hm) * 3.0, 0.0, 1.0));
    }
    let e = max(env, es);
    if (e <= 0.002) {
        return 0.0;
    }
    let q = p - wind_shape();
    let n = textureSampleLevel(shape_tex, samp_rep, q * SHAPE_SCALE, 0.0);
    let low = n.y * 0.625 + n.z * 0.25 + n.w * 0.125;
    let bn = clamp(remap(n.x, low - 1.0, 1.0, 0.0, 1.0), 0.0, 1.0);
    // Erode the envelope with the billowy noise.
    var d = clamp(remap(bn, 1.0 - e, 1.0, 0.0, 1.0), 0.0, 1.0);
    var base_y = y + 10.0 + (n.y - 0.5) * 40.0;
    if (lod == 0 && d > 0.0) {
        let dq = p - wind_shape() * 2.0 + vec3<f32>(0.0, U.eye.w * 1.5, 0.0);
        let dn = textureSampleLevel(detail_tex, samp_rep, dq * DETAIL_SCALE, 0.0);
        let df = dn.x * 0.625 + dn.y * 0.25 + dn.z * 0.125;
        base_y += (df - 0.5) * 30.0;
        // Slightly wispy right at the base, billowy everywhere else.
        let dm = mix(1.0 - df, df, clamp(hc * 12.0, 0.0, 1.0));
        d = clamp(remap(d, dm * 0.45, 1.0, 0.0, 1.0), 0.0, 1.0);
    }
    // Flat but slightly lumpy, ragged cumulus base.
    if (env > es) {
        d *= clamp(base_y / 24.0, 0.0, 1.0);
    }
    // Steep edge: real cumulus go from clear to opaque within metres.
    return smoothstep(0.0, 0.4, d) * (0.5 + 0.5 * d);
}

// Is p in a stratiform layer rather than a cumulus? (for the ambient term)
fn env_is_deck(p: vec3<f32>, w: vec4<f32>) -> bool {
    let H = max(w.y * 1000.0, 50.0);
    let in_cu = w.x > 0.05 && p.y >= U.shape.x - 40.0 && p.y < U.shape.x + H;
    return !in_cu && (w.z > 0.003 || w.w > 0.003);
}

// Height fraction 0..1 within the stratus / mid deck containing p.
fn deck_frac(p: vec3<f32>) -> f32 {
    let fs = (p.y - U.layers.x) / U.shape.w;
    if (fs >= -0.1 && fs <= 1.1) {
        return clamp(fs, 0.0, 1.0);
    }
    return clamp((p.y - U.layers.z) / U.layers.w, 0.0, 1.0);
}

// ------------------------------------------------------------------ god rays

fn cloud_shadow_at(p: vec3<f32>) -> f32 {
    let sun = U.sun_dir.xyz;
    let xz = p.xz - sun.xz * (p.y / max(sun.y, 0.05));
    let uv = (xz - U.shadow.xy) * U.shadow.z + 0.5;
    let e = max(abs(uv.x - 0.5), abs(uv.y - 0.5));
    let s = textureSampleLevel(shadow_tex, samp_clamp, uv, 0.0).x;
    return mix(s, 1.0, smoothstep(0.4, 0.5, e));
}

struct GodRays {
    lost: vec3<f32>,    // sunlit in-scatter missing over the whole ray (cloud-shadowed air)
    lost_c: vec3<f32>,  // ... up to the cloud depth
};

// Single-scattered sunlight that the aerial perspective assumed but which the
// cloud shadows block (crepuscular rays / shadowed haze).
fn god_rays(ro: vec3<f32>, rd: vec3<f32>, tmax: f32, tc: f32, mu: f32) -> GodRays {
    var o: GodRays;
    o.lost = vec3<f32>(0.0);
    o.lost_c = vec3<f32>(0.0);
    let sun = U.sun_dir.xyz;
    if (sun.y < 0.0) {
        return o;
    }
    // Only the air below the cloud base can be in cloud shadow.
    var t1 = tmax;
    let cb = U.drift.z;
    if (rd.y > 1e-4) {
        t1 = min(t1, (cb - ro.y) / rd.y);
    } else if (ro.y > cb) {
        return o;
    }
    var t0 = 0.0;
    if (ro.y > cb && rd.y < -1e-4) {
        t0 = (cb - ro.y) / rd.y;
    }
    if (t1 <= t0) {
        return o;
    }
    let n = 20;
    let dt = (t1 - t0) / f32(n);
    let pr = rayleigh_phase(mu);
    let pm = mie_phase(mu, 0.78);
    var od = vec3<f32>(0.0);
    for (var i = 0; i < n; i++) {
        let t = t0 + (f32(i) + 0.5) * dt;
        let p = ro + rd * t;
        let m = medium(max(p.y, 0.0) * 0.001, U.fog.y, U.gfog.xy);
        let sc = (m.ray * pr + vec3<f32>(m.mie * pm)) * sun_trans(p.y, sun.y) * sun_e();
        let l = exp(-od) * sc * (1.0 - cloud_shadow_at(p)) * dt * 0.001;
        o.lost += l;
        if (t < tc) {
            o.lost_c += l;
        }
        od += m.ext * dt * 0.001;
    }
    return o;
}

// ------------------------------------------------------------------ cirrus

const CIRRUS_ALT: f32 = 8500.0;

// Cirrus opacity at a point on the cirrus plane: domain-warped streaks.
fn cirrus(q: vec3<f32>) -> f32 {
    let ang = 0.6;
    let rot = mat2x2<f32>(cos(ang), sin(ang), -sin(ang), cos(ang));
    let p = rot * q.xz;
    // Gentle warp so the streaks curve and hook a little.
    let warp = textureSampleLevel(shape_tex, samp_rep, vec3<f32>(p.x / 50000.0, 0.31, p.y / 50000.0), 0.0);
    let pw = p + vec2<f32>(0.0, (warp.y - 0.5) * 2500.0);
    // Patches, elongated along the streak direction.
    let big = textureSampleLevel(shape_tex, samp_rep, vec3<f32>(pw.x / 45000.0, 0.73, pw.y / 15000.0), 0.0);
    let cov = clamp(remap(big.x * 0.5 + big.y * 0.5, 1.0 - U.cells2.w * 0.8, 1.0, 0.0, 1.0), 0.0, 1.0);
    if (cov <= 0.0) {
        return 0.0;
    }
    // Fibres: strongly anisotropic noise along the streak direction.
    let f1 = textureSampleLevel(detail_tex, samp_rep, vec3<f32>(pw.x / 14000.0, 0.17, pw.y / 420.0), 0.0);
    let f2 = textureSampleLevel(detail_tex, samp_rep, vec3<f32>(pw.x / 5000.0, 0.57, pw.y / 150.0), 0.0);
    let fib = smoothstep(0.25, 0.85, f1.x * 0.7 + f2.y * 0.3);
    return cov * fib * 0.55;
}

// ------------------------------------------------------------------ lighting

fn hg(c: f32, g: f32) -> f32 {
    let g2 = g * g;
    return (1.0 - g2) / (4.0 * PI * pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5));
}

fn phase(c: f32, k: f32) -> f32 {
    return mix(hg(c, 0.80 * k), hg(c, -0.25 * k), 0.28);
}

// Optical depth toward the sun (sigma included).
fn light_od(p: vec3<f32>, nsteps: i32) -> f32 {
    let sun = U.sun_dir.xyz;
    var steps = array<f32, 6>(22.0, 45.0, 90.0, 170.0, 300.0, 520.0);
    var od = 0.0;
    var tl = 0.0;
    for (var j = 0; j < nsteps; j++) {
        var l = steps[j];
        if (j == nsteps - 1) {
            l = 1147.0 - tl; // last step covers the rest of the 1.1 km reach
        }
        let q = p + sun * (tl + l * 0.5);
        tl += l;
        if (q.y > U.shape.y) {
            break;
        }
        od += density(q, weather(q.xz), select(1, 0, j == 0)) * l;
    }
    return od * U.shape.z;
}

// Per-pixel ray offset: hashed white noise (IGN shows row structure on
// near-horizontal rays through a thin layer).
fn pix_noise(p: vec2<i32>) -> f32 {
    return u01(lowbias32(bitcast<u32>(p.x) * 1973u + bitcast<u32>(p.y) * 9277u + 26699u));
}

fn step_at(t: f32) -> f32 {
    return clamp(t * 0.009, 12.0, 320.0);
}

fn ray_dir(uv: vec2<f32>) -> vec3<f32> {
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let p = U.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    // inv_view_proj is eye-relative (rotation only).
    return normalize(p.xyz / p.w);
}

struct TraceOut {
    @location(0) col: vec4<f32>, // premultiplied in-scatter, a = transmittance
    @location(1) dep: vec2<f32>, // x = march limit (surface), y = mean cloud depth
};

@fragment
fn fs_trace(in: FsOut) -> TraceOut {
    let px = vec2<i32>(in.pos.xy);
    let uv = (vec2<f32>(px) + 0.5) / U.screen.zw;
    let rd = ray_dir(uv);
    let ro = U.eye.xyz;
    let sun = U.sun_dir.xyz;

    // Scene distance over the 2x2 footprint: checkerboard min / max so the
    // upsample always finds a matching near and far sample at silhouettes.
    let dims = vec2<i32>(textureDimensions(dist_tex));
    let fp = px * 2;
    let d0 = textureLoad(dist_tex, min(fp, dims - 1), 0).x;
    let d1 = textureLoad(dist_tex, min(fp + vec2<i32>(1, 0), dims - 1), 0).x;
    let d2 = textureLoad(dist_tex, min(fp + vec2<i32>(0, 1), dims - 1), 0).x;
    let d3 = textureLoad(dist_tex, min(fp + vec2<i32>(1, 1), dims - 1), 0).x;
    let checker = ((px.x + px.y) & 1) == 0;
    var limit = select(min(min(d0, d1), min(d2, d3)), max(max(d0, d1), max(d2, d3)), checker);
    limit = min(limit, MAX_T);

    var o: TraceOut;
    var T = 1.0;
    var L = vec3<f32>(0.0);
    var tsum = 0.0;
    var wsum = 0.0;

    let ylo = U.drift.z - 40.0;
    let yhi = U.shape.y;
    var t0 = 0.0;
    var t1 = limit;
    if (abs(rd.y) > 1e-6) {
        let ta = (ylo - ro.y) / rd.y;
        let tb = (yhi - ro.y) / rd.y;
        t0 = max(min(ta, tb), 0.0);
        t1 = min(max(ta, tb), limit);
    } else if (ro.y < ylo || ro.y > yhi) {
        t1 = -1.0;
    }

    let mu = dot(rd, sun);
    let sky_up = sky_color(vec3<f32>(0.0, 1.0, 0.0));
    let sky_hz = sky_color(normalize(vec3<f32>(sun.x, 0.1, sun.z)));
    let amb_top = (sky_up * 0.6 + sky_hz * 0.4) * 0.45;
    // Light reflected up from the sunlit ground.
    let amb_bot = sky_irr(vec3<f32>(0.0, -1.0, 0.0)) / PI * 1.2;
    let powder_amt = 0.4 * clamp(0.6 - 0.6 * mu, 0.0, 1.0);
    var phases = array<f32, 4>(phase(mu, 1.0), phase(mu, 0.5), phase(mu, 0.25), phase(mu, 0.12));
    // sun_col is radiance-like (scene: albedo * sun_col * ndl), irradiance = pi * sun_col.
    let E = U.sun_col.xyz * PI * 1.2;

    if (t1 > t0) {
        let jit = fract(pix_noise(px) + U.fog.z);
        // Secondary targets (water reflection) march more coarsely.
        let qstep = select(1.0, 1.7, U.fog.w > 0.5);
        var t = t0 + step_at(t0) * qstep * jit;
        // Cheap coarse march (base shape only) until a cloud is hit, then step
        // back and march finely with full detail + lighting until the ray has
        // been out of cloud for a while.
        var fine = false;
        var empty = 0;
        // Two decks + cumulus can need more steps than one layer.
        let max_it = select(220, 300, U.cells2.z + U.layers.y > 0.0);
        for (var i = 0; i < max_it; i++) {
            if (t >= t1 || T < 0.008) {
                break;
            }
            let base_dt = step_at(t) * qstep;
            let p = ro + rd * t;
            let w = weather(p.xz);
            if (!fine) {
                if (w.x + w.z + w.w < 0.003) {
                    t += base_dt * 2.5;
                    continue;
                }
                if (density(p, w, 1) > 0.0) {
                    fine = true;
                    empty = 0;
                    t = max(t - base_dt * 1.5, t0);
                    continue;
                }
                t += base_dt * 1.5;
                continue;
            }
            // Fine steps at the cloud surface, growing once the ray is deep
            // inside (what lies behind contributes little); stratiform decks
            // are smooth inside and take longer steps.
            var dt = base_dt * (0.35 + 0.9 * (1.0 - T));
            let deck_only = w.x < 0.05 && w.z + w.w > 0.5;
            if (deck_only) {
                dt *= 2.4;
            }
            let d = density(p, w, 0);
            if (d > 0.002) {
                empty = 0;
                let sig = d * U.shape.z;
                // Smooth decks need fewer light samples than billowy cumulus.
                let od = light_od(p, select(6, 4, deck_only));
                // Single scattering + multiple-scattering octaves (Wrenninge):
                // each octave is dimmer, less attenuated and more isotropic.
                let single = phases[0] * exp(-od);
                var ms = 0.0;
                var a = 0.65;
                var b = 0.55;
                for (var k = 1; k < 4; k++) {
                    ms += a * phases[k] * exp(-od * b);
                    a *= 0.6;
                    b *= 0.55;
                }
                // Deep diffusion: light keeps leaking into thick cloud
                // (transport optical depth ~ (1 - g) * od).
                ms += 0.16 * phases[3] * exp(-od * 0.06);
                // Powder: multiple scattering builds up with depth into the cloud.
                let powder = 1.0 - powder_amt * exp(-od * 2.0 - d * 3.0);
                var sunL = single + ms * powder;
                // Layers above this one (beyond the light march's reach) diffuse
                // the sunlight: transmission ~ 1 / (1 + 0.75 (1 - g) tau).
                var upper = 1.0;
                if (w.w > 0.003 && p.y < U.layers.z) {
                    upper /= 1.0 + 0.12 * w.w * U.layers.w * U.shape.z * 0.5;
                }
                if (w.z > 0.003 && p.y < U.layers.x) {
                    upper /= 1.0 + 0.12 * w.z * U.shape.w * U.shape.z * 0.5;
                }
                sunL *= upper;
                // Height inside the layer for the ambient term.
                // Height inside whichever layer this is.
                var hf = clamp((p.y - U.shape.x) / max(w.y * 1000.0, U.shape.w), 0.0, 1.0);
                let in_deck = env_is_deck(p, w);
                if (in_deck) {
                    hf = deck_frac(p);
                }
                // Sky-light occlusion from a short probe straight up.
                let pu = p + vec3<f32>(0.0, 80.0, 0.0);
                let occ = density(pu, w, 1) * 160.0;
                let sky_vis = exp(-occ * U.shape.z * 0.12);
                var amb = mix(amb_bot, amb_top * (0.35 + 0.65 * sky_vis), smoothstep(-0.05, 0.7, hf));
                if (in_deck) {
                    // Inside a deck the light is diffuse and grey, fading with
                    // depth below the top (diffusion through optical depth tau).
                    let tau = (1.0 - hf) * U.shape.w * U.shape.z * 0.5;
                    let diff = 1.0 / (1.0 + 0.12 * tau);
                    amb = mix(amb_bot, vec3<f32>(luma(amb_top)) * 1.3, 0.3 + 0.7 * hf) * diff;
                }
                let S = E * sunL + amb * sqrt(upper);
                let tr = exp(-sig * dt);
                let wgt = T * (1.0 - tr);
                L += S * wgt;
                tsum += t * wgt;
                wsum += wgt;
                T *= tr;
            } else {
                empty += 1;
                if (empty > 10) {
                    fine = false;
                }
            }
            t += dt;
        }
        if (T < 0.008) {
            T = 0.0;
        }
    }

    // Aerial perspective at the transmittance-weighted cloud depth.
    var depth = FAR_DEPTH;
    if (wsum > 1e-4) {
        depth = tsum / wsum;
        let ap = ap_at(uv, depth);
        L = L * ap.a + (1.0 - T) * ap.rgb;
    }

    // A full overcast deck beyond the march range: an opaque, hazed ceiling
    // out to the horizon (no bright sky gap under the deck), or seen from
    // above, a sea of cloud to the horizon.
    for (var k = 0; k < 2; k++) {
        let dk = select(clamp((U.cells2.z - 0.85) * 6.6, 0.0, 1.0), clamp((U.layers.y - 0.85) * 6.6, 0.0, 1.0), k == 1);
        let db = select(U.layers.x, U.layers.z, k == 1);
        let dth = select(U.shape.w, U.layers.w, k == 1);
        if (dk <= 0.0 || limit < MAX_T) {
            continue;
        }
        var td = -1.0;
        var dc = vec3<f32>(0.0);
        if (rd.y > 0.0 && ro.y < db) {
            td = (db + 60.0 - ro.y) / rd.y;
            dc = amb_bot * 1.6 + E * 0.01;
        } else if (rd.y < 0.0 && ro.y > db + dth) {
            td = (db + dth * 0.8 - ro.y) / rd.y;
            dc = E * (0.12 + 0.5 * max(sun.y, 0.0)) * phases[2] * 4.0 + amb_top * 1.2;
        }
        let a = dk * smoothstep(0.6 * MAX_T, 0.95 * MAX_T, td);
        if (td > 0.0 && a > 0.0) {
            let ap = ap_at(uv, td);
            let cl = dc * ap.a + ap.rgb;
            L += T * cl * a;
            T *= 1.0 - a;
        }
    }

    // Cirrus: a thin, fibrous high sheet, only where the sky is visible.
    if (U.cells2.w > 0.0 && rd.y > 0.0 && T > 0.01 && limit >= MAX_T) {
        let tc = (CIRRUS_ALT - ro.y) / rd.y;
        if (tc > 0.0) {
            // The jet stream carries cirrus faster than the low clouds.
            // The cirrus runs faster than the cumulus (gfog.zw: its extra wind drift).
            let c = cirrus(ro + rd * tc - vec3<f32>(U.drift.x + U.gfog.z, 0.0, U.drift.y + U.gfog.w));
            let alpha = c * smoothstep(0.0, 0.06, rd.y);
            let ccol = E * (0.012 + 0.6 * hg(mu, 0.75)) * 0.5 + amb_top * 1.4;
            let ap = ap_at(uv, tc);
            let cl = ccol * ap.a + ap.rgb;
            L += T * cl * alpha;
            T *= 1.0 - alpha;
        }
    }

    // Crepuscular rays: the scene's haze assumes full sun everywhere; remove
    // the sunlit part of the haze in-scatter where the air is in cloud shadow.
    // In front of a cloud only the haze up to the cloud counts (and never
    // more than the haze that was actually added to it).
    if (U.fog.w < 0.5) {
        let gr = god_rays(ro, rd, min(limit, 30000.0), depth, mu);
        let total = ap_at(uv, min(limit, 60000.0)).rgb;
        let front = ap_at(uv, depth).rgb;
        L -= T * min(gr.lost, total * 0.9) + (1.0 - T) * min(gr.lost_c, front * 0.8);
        L = max(L, vec3<f32>(0.0));
    }

    o.col = vec4<f32>(L, T);
    o.dep = vec2<f32>(limit, depth);
    return o;
}

// ------------------------------------------------------------------ temporal

@fragment
fn fs_temporal(in: FsOut) -> TraceOut {
    let px = vec2<i32>(in.pos.xy);
    let dims = vec2<i32>(textureDimensions(trace_col));
    let cur = textureLoad(trace_col, px, 0);
    let cd = textureLoad(trace_dep, px, 0).xy;
    var o: TraceOut;
    if (U.sun_col.w >= 0.999) {
        o.col = cur;
        o.dep = cd;
        return o;
    }
    // Neighbourhood statistics for variance clipping.
    var m1 = vec4<f32>(0.0);
    var m2 = vec4<f32>(0.0);
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let c = textureLoad(trace_col, clamp(px + vec2<i32>(x, y), vec2<i32>(0), dims - 1), 0);
            m1 += c;
            m2 += c * c;
        }
    }
    m1 /= 9.0;
    m2 /= 9.0;
    let sd = sqrt(max(m2 - m1 * m1, vec4<f32>(0.0)));

    let uv = (vec2<f32>(px) + 0.5) / U.screen.zw;
    let rd = ray_dir(uv);
    // prev_view_proj is relative to this frame's eye.
    let clip = U.prev_view_proj * vec4<f32>(rd * cd.y, 1.0);
    var valid = clip.w > 1e-3;
    let pndc = clip.xy / clip.w;
    let puv = vec2<f32>(pndc.x * 0.5 + 0.5, 0.5 - pndc.y * 0.5);
    valid = valid && all(puv > vec2<f32>(0.0)) && all(puv < vec2<f32>(1.0));
    var res = cur;
    var dep = cd;
    if (valid) {
        let hp = vec2<i32>(puv * U.screen.zw);
        let hd = textureLoad(hist_dep, clamp(hp, vec2<i32>(0), dims - 1), 0).xy;
        // Reject history whose march limit was a different surface.
        let same = abs(hd.x - cd.x) < 0.08 * max(cd.x, 1.0) + 2.0;
        if (same) {
            // Clip loosely when the history is static (lets the jitter converge),
            // tightly when it moved (no ghosting).
            let motion = length((puv - uv) * U.screen.zw);
            let gamma = mix(3.0, 1.0, clamp(motion * 0.5, 0.0, 1.0));
            var h = textureSampleLevel(hist_col, samp_clamp, puv, 0.0);
            h = clamp(h, m1 - sd * gamma, m1 + sd * gamma);
            res = mix(h, cur, U.sun_col.w);
            dep = vec2<f32>(cd.x, mix(hd.y, cd.y, 0.5));
        }
    }
    o.col = res;
    o.dep = dep;
    return o;
}

// ------------------------------------------------------------------ composite

@fragment
fn fs_composite(in: FsOut) -> @location(0) vec4<f32> {
    let px = vec2<i32>(in.pos.xy);
    let d = min(textureLoad(dist_tex, px, 0).x, MAX_T);
    let hdims = vec2<i32>(textureDimensions(hist_col));
    let hp = (vec2<f32>(px) + 0.5) * 0.5 - 0.5;
    let b = vec2<i32>(floor(hp));
    let f = hp - floor(hp);
    var acc = vec4<f32>(0.0);
    var wsum = 0.0;
    for (var k = 0; k < 4; k++) {
        let o = vec2<i32>(k & 1, k >> 1);
        let tc = clamp(b + o, vec2<i32>(0), hdims - 1);
        let c = textureLoad(hist_col, tc, 0);
        let l = textureLoad(hist_dep, tc, 0).x;
        let bw = select(1.0 - f.x, f.x, o.x == 1) * select(1.0 - f.y, f.y, o.y == 1);
        let rel = abs(l - d) / max(d, 1.0);
        let dw = 1.0 / (1.0 + rel * rel * 4000.0);
        let w = bw * dw + 1e-5;
        acc += c * w;
        wsum += w;
    }
    let c = acc / wsum;
    return vec4<f32>(c.rgb, clamp(c.a, 0.0, 1.0));
}

// ------------------------------------------------------------------ cloud shadow map

@fragment
fn fs_shadow(in: FsOut) -> @location(0) vec4<f32> {
    let res = 1.0 / (U.shadow.z * U.shadow.w);
    let xz = U.shadow.xy + (floor(in.pos.xy) + 0.5 - res * 0.5) * U.shadow.w;
    let sun = U.sun_dir.xyz;
    if (sun.y < 0.02) {
        return vec4<f32>(1.0);
    }
    let g = vec3<f32>(xz.x, 0.0, xz.y);
    let ta = (U.drift.z - 40.0) / sun.y;
    let tb = U.shape.y / sun.y;
    let n = 24;
    let dt = (tb - ta) / f32(n);
    var od = 0.0;
    for (var i = 0; i < n; i++) {
        let p = g + sun * (ta + (f32(i) + 0.5) * dt);
        let w = weather(p.xz);
        if (w.x + w.z + w.w < 0.003) {
            continue;
        }
        od += density(p, w, 1) * dt;
    }
    // Forward scattering keeps cloud shadows from going fully black.
    let tr = exp(-od * U.shape.z * 0.6);
    return vec4<f32>(mix(0.06, 1.0, tr));
}
