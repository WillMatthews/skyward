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
    seed: vec4<u32>,     // seed, frame, cumulus visible, quality 0..3 (step size)
    layers: vec4<f32>,   // stratus base, mid-layer coverage param, mid base, mid thickness
    drift: vec4<f32>,    // field drift x, z (m: clouds move with the wind), lowest base, 0
    gfog: vec4<f32>,     // ground fog extinction (/km), fog top (km), cirrus extra drift x, z (m)
    storm: vec4<f32>,    // storm cells, rain-shaft bottom (world y), shaft slant x, z (m per m of fall)
    storm2: vec4<f32>,   // under-base dryness, storm top (world y), 0, 0
    lent: vec4<f32>,     // lenticular grid origin x, z (world), size (m), lens level (world y)
    lent2: vec4<f32>,    // on, half thickness (m), 0, 0
    flash: array<vec4<f32>, 2>, // lightning: eye-relative position, intensity
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
// Thunderstorm cells (wx/storm.rs, == clouds.rs StormCellsU): per cell
// (centre x, z, tower radius, base), (top, overshoot, intensity, anvil length),
// (anvil dir x, z, rain offset x, z), (rain radius, rain 0..1, shaft bottom, anvil base).
const MAX_CELLS: u32 = 24u;
struct StormCells {
    c: array<vec4<f32>, 96>,
};
@group(0) @binding(17) var<uniform> SC: StormCells;
@group(0) @binding(18) var storm_a_tex: texture_2d<f32>;
@group(0) @binding(19) var storm_b_tex: texture_2d<f32>;
@group(0) @binding(20) var lent_tex: texture_2d<f32>;

// Storms / lenticulars present (clouds.rs builds copies of the trace /
// shadow passes with these false: that code compiled out).
const WX: bool = true;
const WXL: bool = true;
fn wx_storm() -> bool {
    return WX && U.storm.x > 0.5;
}
fn wx_lent() -> bool {
    return WXL && U.lent2.x > 0.5;
}
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

struct WeatherOut {
    @location(0) w: vec4<f32>,
    // Storms: (occupancy, low-level cells + 1 (2), rain 0..1) and (tower
    // cover, high-level cells + 1 (2), raining cell + 1).
    @location(1) sa: vec4<f32>,
    @location(2) sb: vec4<f32>,
};

// Thunderstorm geometry shared by the bake, the density and the lighting
// (wx/storm.rs Cell: the same cells the CPU uses for winds, turbulence and
// radar; the tower leans downshear like `Cell::tower_axis`).
const STORM_TILT: f32 = 0.09;

fn rot2(v: vec2<f32>, a: f32) -> vec2<f32> {
    let c = cos(a);
    let s = sin(a);
    return vec2<f32>(c * v.x - s * v.y, s * v.x + c * v.y);
}

// The flanking line of towering cumulus trails from the rear right flank.
fn flank_dir(adir: vec2<f32>) -> vec2<f32> {
    return rot2(adir, 2.3);
}

fn seg_dist(p: vec2<f32>, a: vec2<f32>, b: vec2<f32>) -> f32 {
    let ab = b - a;
    let t = clamp(dot(p - a, ab) / max(dot(ab, ab), 1e-3), 0.0, 1.0);
    return length(p - a - ab * t);
}

// The storm maps at field xz: which (up to four) cells can have cloud or rain
// over this texel, with a conservative footprint (the density is analytic per
// cell: no texel seams as long as every cell reaching a texel is listed).
fn storm_bake(xz: vec2<f32>) -> array<vec4<f32>, 2> {
    // Two lists: the cells that matter low down (towers, flanking lines,
    // rain) and high up (towers, anvils): long anvils overlap a lot.
    var occ = vec4<f32>(0.0);
    var ids = vec4<f32>(0.0);
    var occ_all = 0.0;
    var rain_id = 0.0;
    var tower = 0.0;
    var rain = 0.0;
    let n = u32(U.storm.x);
    for (var i = 0u; i < min(n, MAX_CELLS); i++) {
        let c0 = SC.c[i * 4u];
        let c1 = SC.c[i * 4u + 1u];
        let c2 = SC.c[i * 4u + 2u];
        let c3 = SC.c[i * 4u + 3u];
        let c = c0.xy;
        let R = c0.z;
        let H = max(c1.x - c0.w, 500.0);
        let adir = c2.xy;
        let lean = adir * STORM_TILT * H;
        let rc = c + c2.zw;
        if (length(xz - c) > R * 7.0 + c1.w + 4000.0 + length(U.storm.zw) * c0.w) {
            continue;
        }
        // Tower (leaning, merged with the precipitation core) and its base clouds.
        let dt = min(seg_dist(xz, c, c + lean), seg_dist(xz, c, rc));
        let imp_t = 1.0 - dt / (R * 2.2);
        var imp = imp_t;
        tower = max(tower, bump(dt / (R * 1.15)));
        // Flanking line.
        if (c1.z > 0.5) {
            let fl = flank_dir(adir);
            imp = max(imp, 1.0 - seg_dist(xz, c + fl * R * 0.9, c + fl * R * 5.8) / (R * 1.6));
        }
        // Rain shafts (slanted by the shear down to the ground).
        let fall = max(c0.w - U.storm.y, 0.0);
        let dr = seg_dist(xz, rc, rc + U.storm.zw * fall);
        imp = max(imp, 1.0 - dr / (c3.x * 1.8));
        let rv = c3.y * (1.0 - smoothstep(1.0, 1.4, dr / c3.x));
        if (rv > rain) {
            rain = rv;
            rain_id = f32(i + 1u);
        }
        var imp_h = imp_t;
        // Anvil: a plume from the tower top spreading downwind.
        if (c1.w > 0.0) {
            let a = xz - c - lean;
            let s = dot(a, adir);
            let nn = abs(a.x * adir.y - a.y * adir.x);
            let W = R * 1.35 + max(s, 0.0) * 0.5;
            let along = max(-s - R * 1.7, s - c1.w);
            imp_h = max(imp_h, min(1.0 - nn / (W * 1.5), 1.0 - along / (R * 1.5)));
        }
        occ_all = max(occ_all, max(imp, imp_h));
        // Insert into the two most important of each list.
        for (var l = 0; l < 2; l++) {
            var v = clamp(select(imp, imp_h, l == 1), 0.0, 1.0);
            var id = f32(i + 1u);
            for (var k = l * 2; k < l * 2 + 2; k++) {
                if (v > occ[k]) {
                    let ov = occ[k];
                    let oi = ids[k];
                    occ[k] = v;
                    ids[k] = id;
                    v = ov;
                    id = oi;
                }
            }
        }
    }
    return array<vec4<f32>, 2>(vec4<f32>(occ_all, ids.x, ids.y, rain), vec4<f32>(tower, ids.z, ids.w, rain_id));
}

// RGBA: cumulus coverage, cumulus height (km), stratus coverage, mid-layer
// coverage. Baked in field space (world xz - drift). Plus the storm maps.
@fragment
fn fs_weather(in: FsOut) -> WeatherOut {
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
    var o: WeatherOut;
    o.w = vec4<f32>(cov * fade, h / 1000.0, strato * fade, mid * fade);
    o.sa = vec4<f32>(0.0);
    o.sb = vec4<f32>(0.0);
    if (wx_storm()) {
        // The storm maps cover STORM_SPAN x the weather map (CBs are seen from far away).
        let sxz = U.wmap.xy + (floor(in.pos.xy) + 0.5 - res * 0.5) * U.wmap.z * STORM_SPAN;
        let st = storm_bake(sxz);
        let rs = length(sxz - U.wmap.xy) / (U.wmap.z * STORM_SPAN * res * 0.5);
        let sfade = 1.0 - smoothstep(0.85, 0.98, rs);
        o.sa = vec4<f32>(st[0].x * sfade, st[0].y, st[0].z, st[0].w * sfade);
        o.sb = st[1];
    }
    return o;
}

// Storm maps at world xz (field space like the weather map).
const STORM_SPAN: f32 = 5.0;
fn storm_uv(xz: vec2<f32>) -> vec2<f32> {
    return (xz - U.drift.xy - U.wmap.xy) / (U.wmap.z * STORM_SPAN * U.wmap.w) + 0.5;
}
fn storm_a(xz: vec2<f32>) -> vec4<f32> {
    return textureSampleLevel(storm_a_tex, samp_clamp, storm_uv(xz), 0.0);
}
fn storm_b(xz: vec2<f32>) -> vec4<f32> {
    return textureSampleLevel(storm_b_tex, samp_clamp, storm_uv(xz), 0.0);
}

// Lenticular cover at world xz (fixed to the ground: no drift).
fn lent_at(xz: vec2<f32>) -> f32 {
    if (!wx_lent()) {
        return 0.0;
    }
    let uv = (xz - U.lent.xy) / U.lent.z;
    if (any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0))) {
        return 0.0;
    }
    return textureSampleLevel(lent_tex, samp_clamp, uv, 0.0).x;
}

// The (up to four) storm cells that can reach world xz: indices + 1 (0 =
// none; the low-level list first, then the high-level one).
fn storm_ids(xz: vec2<f32>) -> vec4<u32> {
    let uv = storm_uv(xz);
    if (any(uv < vec2<f32>(0.0)) || any(uv >= vec2<f32>(1.0))) {
        return vec4<u32>(0u);
    }
    let tc = vec2<i32>(uv * U.wmap.w);
    let t = textureLoad(storm_a_tex, tc, 0);
    let b = textureLoad(storm_b_tex, tc, 0);
    var o = vec4<u32>(u32(t.y + 0.5), u32(t.z + 0.5), u32(b.y + 0.5), u32(b.z + 0.5));
    // (The two lists can share a cell.)
    if (o.z == o.x || o.z == o.y) {
        o.z = 0u;
    }
    if (o.w == o.x || o.w == o.y) {
        o.w = 0u;
    }
    return o;
}

// What a storm sample needs from the noise (one set, shared by both cells).
struct StormNoise {
    big: vec4<f32>,  // turret-scale worley (x unused, y ~1.7 km, z ~0.9 km, w ~0.45 km cells)
    fine: f32,       // cauliflower billows on the turrets (~250 m)
    fib: f32,        // fibrous ice texture streaming along the anvil
    cut: f32,        // the field noise as a fraction of R (> 0 bulges out)
    bounds: bool,    // noise-free pass: also bound the field over any noise
};

fn storm_noise_neutral() -> StormNoise {
    var nz: StormNoise;
    nz.big = vec4<f32>(0.5);
    nz.fine = 0.5;
    nz.fib = 0.5;
    nz.cut = 0.0;
    nz.bounds = true;
    return nz;
}

// One cell's cloud at p (field xz `f`), as a signed distance-like field in
// metres (> 0 inside) with the edge softness (m), the density scale and the
// anvil share; plus the cloud overhead (m) and the height fraction in the cell
// for the lighting. `hi` / `lo` bound the field over any noise (evaluated
// with neutral noise: lets most samples skip the noise lookups).
struct StormCell {
    field: f32,
    edge: f32,
    dens: f32,
    anvil: f32,
    over: f32,
    h: f32,
    hi: f32,
    lo: f32,
};

fn storm_none() -> StormCell {
    var g: StormCell;
    g.field = -1e4;
    g.edge = 100.0;
    g.dens = 1.0;
    g.anvil = 0.0;
    g.over = 0.0;
    g.h = 0.0;
    g.hi = -1e4;
    g.lo = -1e4;
    return g;
}

// A part of the cell (field, its bounds over any noise, edge): union.
fn storm_part(o: ptr<function, StormCell>, fld: f32, hi: f32, lo: f32, edge: f32) {
    (*o).hi = max((*o).hi, hi);
    (*o).lo = max((*o).lo, lo);
    if (fld > (*o).field) {
        (*o).field = fld;
        (*o).edge = edge;
    }
}

// Can cell i have cloud at height y? (cheap reject before the full field)
fn storm_cell_near(i: u32, y: f32) -> bool {
    let c0 = SC.c[i * 4u];
    let c1 = SC.c[i * 4u + 1u];
    return y > c0.w - 700.0 && y < c1.x + c1.y + 600.0;
}

fn storm_cell(i: u32, p: vec3<f32>, f: vec2<f32>, nz: StormNoise) -> StormCell {
    let c0 = SC.c[i * 4u];
    let c1 = SC.c[i * 4u + 1u];
    let c2 = SC.c[i * 4u + 2u];
    let c3 = SC.c[i * 4u + 3u];
    let c = c0.xy;
    let R = c0.z;
    let base = c0.w;
    let top = c1.x;
    let H = max(top - base, 500.0);
    let y = p.y;
    let h = (y - base) / H;
    let hc = clamp(h, 0.0, 1.0);
    let adir = c2.xy;
    let inten = c1.z;
    let rc = c + c2.zw;
    var o = storm_none();
    o.h = h;
    // ---- tower: a continuous leaning column of stacked turrets, merged with
    // the precipitation core downshear; widest at mid levels, narrowing into
    // the anvil, softer and ragged at the base, crisp cauliflower higher up.
    let lean = adir * STORM_TILT * clamp(y - base, 0.0, H);
    let axis = c + lean;
    let dc = seg_dist(f, axis, mix(axis, rc, 1.0 - 0.7 * hc));
    let rq = dc / R;
    if (rq < 2.2) {
        let prof = 0.84 + 0.2 * sin(PI * clamp(hc * 1.15, 0.0, 1.0)) - 0.12 * smoothstep(0.72, 1.0, hc);
        let amp = mix(0.8, 1.35, hc);
        // Sides, flat (slightly lumpy) base, cauliflower dome where it tops out.
        let r0 = R * prof - dc;
        let b0 = (y - base) * 1.5;
        let c0t = (top - y) - H * 0.16 * rq * rq;
        let ft = min(min(r0 + R * nz.cut * amp, b0 + (nz.big.y - 0.5) * 90.0), c0t + R * nz.cut * 1.3);
        let ar = R * 0.5;
        storm_part(&o, ft, min(min(r0 + ar, b0 + 45.0), c0t + ar), min(min(r0 - ar, b0 - 45.0), c0t - ar), mix(130.0, 30.0, smoothstep(0.0, 0.5, h)));
    }
    // Overshooting top: a hard dome punching through the anvil.
    if (c1.y > 1.0 && y > top - c1.y) {
        let rh = R * 0.62;
        let e = length(vec2<f32>(length(f - c - adir * STORM_TILT * H) / rh, (y - top) / c1.y));
        let fo = (1.0 - e) * min(rh, c1.y) * 1.6;
        storm_part(&o, fo + R * nz.cut * 0.7, fo + R * 0.26, fo - R * 0.26, 30.0);
    }
    if (dc < R) {
        o.over = max(top + c1.y * 0.5 - y, 0.0) * sqrt(1.0 - rq * rq);
    }
    // ---- flanking line: towering cumulus stepping down away from the storm.
    let fl = flank_dir(adir);
    if (inten > 0.5 && y < base + H * 0.55 && seg_dist(f, c + fl * R * 0.9, c + fl * R * 5.8) < R * 1.1) {
        let sp = R * 1.25;
        let tl = dot(f - c, fl) / sp;
        let k0 = clamp(floor(tl), 1.0, 4.0);
        for (var j = 0; j < 2; j++) {
            let k = clamp(k0 + f32(j), 1.0, 4.0);
            let side = vec2<f32>(-fl.y, fl.x) * (fract(k * 0.618 + c.x * 1.3e-4) - 0.5) * R * 0.7;
            let cen = c + fl * sp * k + side;
            let rk = R * (0.52 - 0.06 * k);
            let hk = H * (0.6 - 0.1 * k) * (0.55 + 0.45 * inten);
            let dk = length(f - cen);
            let rr = dk / rk;
            let s0 = rk * 0.95 - dk;
            let b0 = (y - base) * 1.5;
            let t0 = (base + hk - y) - hk * 0.5 * rr * rr;
            let fk = min(min(s0 + rk * nz.cut * 1.6, b0 + (nz.big.y - 0.5) * 70.0), t0 + rk * nz.cut * 2.0);
            let ak = rk * 0.75;
            storm_part(&o, fk, min(min(s0 + ak, b0 + 35.0), t0 + ak), min(min(s0 - ak, b0 - 35.0), t0 - ak), mix(140.0, 30.0, smoothstep(0.0, 0.5, (y - base) / hk)));
        }
    }
    // ---- shelf cloud on the gust front ahead of the rain core and a lowered
    // wall cloud under the updraft (strong cells).
    let gd = length(f - rc);
    if (inten > 0.62 && y < base + 80.0 && y > base - 600.0 && gd < c3.x * 1.3 + R * 0.5) {
        let g = f - rc;
        let ca = dot(g, adir) / max(gd, 1.0);
        let arc = smoothstep(-0.2, 0.5, ca);
        let ring = c3.x * 1.3;
        let dr = gd - ring;
        // Wedge: lowest at the leading edge, rising back into the base.
        let depth = 380.0 * arc * smoothstep(-R * 1.1, 0.0, dr) * smoothstep(0.62, 0.9, inten);
        if (depth > 10.0) {
            let fs = min(min(y - (base - depth) + (nz.fine - 0.5) * 90.0 + nz.cut * 300.0, (R * 0.15 - dr) * 0.5 + nz.cut * R * 0.3), base + 60.0 - y);
            let f0 = min(min(y - (base - depth), (R * 0.15 - dr) * 0.5), base + 60.0 - y);
            let as_ = R * 0.12 + 160.0;
            storm_part(&o, fs, f0 + as_, f0 - as_, 110.0);
        }
        let wc = c - adir * R * 0.2;
        let dw = length(f - wc) / (R * 0.38);
        let fw = min((1.0 - dw) * R * 0.38 + nz.cut * R * 0.2, y - (base - 260.0 * smoothstep(0.7, 0.95, inten) + 160.0 * dw * dw));
        let fw0 = min(min((1.0 - dw) * R * 0.38, y - (base - 260.0 * smoothstep(0.7, 0.95, inten) + 160.0 * dw * dw)), base + 60.0 - y);
        storm_part(&o, min(fw, base + 60.0 - y), fw0 + R * 0.08, fw0 - R * 0.08, 60.0);
    }
    // ---- anvil: a smooth flat-topped plume attached to the tower top,
    // spreading downwind and thinning into fibrous ice, curving down into
    // the tower, mammatus pouches hanging under it.
    if (c1.w > 0.0 && y > c3.w - H * 0.15 - 650.0 && y < top + 500.0) {
        let a = f - c - adir * STORM_TILT * H;
        let s = dot(a, adir);
        let n = a.x * adir.y - a.y * adir.x;
        let L = c1.w;
        let ds = clamp(s / L, 0.0, 1.0);
        let W = R * 1.15 + max(s, 0.0) * 0.5;
        let back = R * 1.15;
        let sl = select(max(s - L * 0.45, 0.0) / (L * 0.55), -s / back, s < 0.0);
        let rag = (nz.fib - 0.5) * 0.22 * smoothstep(0.3, 0.8, ds) + nz.cut * 0.3 * (1.0 - ds);
        let rho0 = length(vec2<f32>(n / W, sl));
        let at = top;
        let ab = c3.w;
        let ra = max(length(a) / R - 0.7, 0.0);
        let funnel = exp(-ra * ra * 2.5);
        // Billowy near the tower, smooth and streaky downstream; a smooth top.
        let lump0 = mix(nz.cut * R * 0.5, (nz.fib - 0.5) * 160.0, smoothstep(0.0, 0.45, ds));
        let lmax = max(R * 0.18, 80.0);
        // Variant 0: this sample; in the noise-free pass also the extremes
        // any noise could reach (the rag moves the outline, which changes
        // the thickness a lot near the edge): the field's bounds.
        let nv = select(1, 3, nz.bounds);
        for (var v = 0; v < nv; v++) {
            var rho = rho0 - rag;
            var lump = lump0;
            var pouch = smoothstep(0.35, 0.95, nz.big.z);
            // The underside sags and bulges on the turret scale (no flat shelf).
            var und = (nz.big.y - 0.5) * 700.0;
            if (v == 1) {
                rho = rho0 - 0.26;
                lump = lmax;
                pouch = 1.0;
                und = 350.0;
            } else if (v == 2) {
                rho = rho0 + 0.26;
                lump = -lmax;
                pouch = 0.0;
                und = -350.0;
            }
            if (rho >= 1.0) {
                continue;
            }
            let lens = sqrt(max(1.0 - rho * rho, 0.0));
            // Flat top, a little domed over the updraft, sagging downstream.
            let ty = at + 300.0 * lens * (1.0 - ds) * (1.0 - ds) - 450.0 * ds * ds;
            // Half-lens section: flat top, the underside rising to thin edges.
            let thick = (at - ab) * 0.8 * mix(1.0, 0.12, pow(ds, 0.6)) * pow(lens, 0.7);
            // The anvil's base curves down into the tower.
            var by = mix(ty - thick, ab - H * 0.12, funnel);
            // Mammatus: rounded pouches under the anvil downstream of the tower.
            let mm = smoothstep(0.08, 0.3, ds) * (1.0 - smoothstep(0.55, 0.9, rho + ds * 0.3)) * smoothstep(0.62, 0.85, inten);
            by -= mm * 520.0 * pouch * pouch + und * (1.0 - funnel);
            // Rounded (pillow) edges rather than a cut wall.
            let vy = (y - (ty + by) * 0.5) / max((ty - by) * 0.5, 50.0);
            let sc = min(W * 0.6, max(ty - by, 100.0) * 0.8);
            let re = (1.0 - length(vec2<f32>(rho, max(abs(vy) - 0.3, 0.0) / 0.7))) * sc;
            let fa = min(min(y - by + lump * 0.7, ty - y + lump * 0.25), re + lump * 0.5);
            if (v == 1) {
                o.hi = max(o.hi, fa + 30.0);
                continue;
            }
            if (v == 2) {
                o.lo = max(o.lo, fa - 30.0);
                continue;
            }
            o.over = max(o.over, clamp(ty - max(y, by), 0.0, thick));
            if (fa > o.field - 40.0) {
                let wa = smoothstep(-40.0, 40.0, fa - o.field);
                // Ice: an order of magnitude less extinction than the tower's
                // water droplets (soft, light passes through the thin parts).
                o.dens = mix(o.dens, mix(0.3, 0.07, sqrt(ds)) * mix(1.0, 0.6 + 0.7 * nz.fib, smoothstep(0.1, 0.6, ds)), wa);
                // (A soft underside: ice falling out, never a hard shelf.)
                let under = 1.0 - smoothstep(-0.2, 0.3, vy);
                o.edge = mix(o.edge, mix(mix(70.0, 420.0, ds), mix(260.0, 420.0, ds), under), wa);
                o.anvil = wa;
                o.field = max(o.field, fa);
            }
        }
    }
    return o;
}

// All cells listed at p (and in `mask`), unioned. Sets g_mask: the cells
// whose noisy field can reach the surface.
var<private> g_mask: u32;
fn storm_cells(ids: vec4<u32>, mask: u32, p: vec3<f32>, f: vec2<f32>, nz: StormNoise) -> StormCell {
    var g = storm_none();
    g_mask = 0u;
    for (var k = 0; k < 4; k++) {
        if (ids[k] == 0u) {
            continue;
        }
        if ((mask & (1u << u32(k))) == 0u || !storm_cell_near(ids[k] - 1u, p.y)) {
            continue;
        }
        let g2 = storm_cell(ids[k] - 1u, p, f, nz);
        if (g2.hi > -g2.edge * 1.5) {
            g_mask |= 1u << u32(k);
        }
        let ov = max(g.over, g2.over);
        let hi = max(g.hi, g2.hi);
        let lo = max(g.lo, g2.lo);
        if (g2.field > g.field) {
            g = g2;
        }
        g.over = ov;
        g.hi = hi;
        g.lo = lo;
    }
    return g;
}

// Storm density at p: x = density, y = anvil share; `lod` 0 = full detail,
// 1 = turrets + billows (light rays), 2 = envelope + turrets, 3 = like 1 for
// a far primary sample, 4 = envelope only (no noise lookups). Sets g_skip:
// clear distance (m) a march may step over.
var<private> g_storm: StormCell;
var<private> g_skip: f32;
fn storm_density(p: vec3<f32>, lod_in: i32) -> vec2<f32> {
    g_skip = 0.0;
    // (lod 3: a primary sample far away: lod 1 detail, but it feeds the lighting.)
    let main = lod_in == 0 || lod_in == 3;
    // (lod 5: the coarse march's probe: lod 1 behind the noise-free bounds.)
    let lod = select(lod_in, 1, lod_in == 3 || lod_in == 5);
    let bounds = true;
    if (main) {
        g_storm = storm_none();
    }
    let sa = storm_a(p.xz);
    if (sa.x < 0.003 || p.y > U.storm2.y) {
        return vec2<f32>(0.0);
    }
    let ids = storm_ids(p.xz);
    if (all(ids == vec4<u32>(0u))) {
        return vec2<f32>(0.0);
    }
    let f = p.xz - U.drift.xy;
    // Without noise first: far outside (most samples) or deep inside, the
    // noise cannot matter.
    // (Primary and light samples sit at the surface: straight to the noise.)
    var g0 = storm_none();
    var mask = 15u;
    if (bounds) {
        g0 = storm_cells(ids, 15u, p, f, storm_noise_neutral());
        mask = g_mask;
    }
    if (bounds && g0.hi < -g0.edge * 1.5) {
        g_skip = -g0.hi;
        if (main) {
            g_storm = g0;
        }
        return vec2<f32>(0.0);
    }
    let fade = smoothstep(0.0, 0.06, sa.x);
    if (lod == 4) {
        // Envelope only (long light / shadow rays, the ground shadow map).
        return vec2<f32>(smoothstep(-g0.edge, g0.edge * 2.0, g0.field) * g0.dens * fade, g0.anvil);
    }
    // (Deep inside the opaque towers only: the anvil's ice is translucent,
    // its insides show.)
    if (bounds && !main && g0.lo > g0.edge * 4.0 && g0.dens > 0.8) {
        if (main) {
            g_storm = g0;
        }
        return vec2<f32>(g0.dens * 1.2 * fade, g0.anvil);
    }
    // The billows rise slowly with the updraft and drift with the wind.
    let t = U.eye.w;
    let q = p - wind_shape() - vec3<f32>(0.0, t * 1.5, 0.0);
    var nz = storm_noise_neutral();
    nz.bounds = false;
    nz.big = textureSampleLevel(shape_tex, samp_rep, q * vec3<f32>(1.0 / 7000.0, 1.0 / 6000.0, 1.0 / 7000.0), 0.0);
    if (lod < 2) {
        let ns = textureSampleLevel(shape_tex, samp_rep, q * (1.0 / 2100.0) + vec3<f32>(0.37, 0.11, 0.73), 0.0);
        nz.fine = ns.y * 0.6 + ns.z * 0.4;
        // Anvil ice: long fibres streaming along the upper wind.
        let i0 = max(select(ids.z, ids.x, ids.x > 0u), 1u) - 1u;
        let ad = SC.c[i0 * 4u + 2u].xy;
        if (p.y > SC.c[i0 * 4u + 3u].w - 1500.0) {
            let al = dot(q.xz, ad);
            let ac = q.x * ad.y - q.z * ad.x;
            let fb = textureSampleLevel(shape_tex, samp_rep, vec3<f32>(al / 11000.0, q.y / 3000.0, ac / 3800.0), 0.0);
            nz.fib = clamp(fb.x * 0.4 + fb.y * 0.3 + nz.big.z * 0.3, 0.0, 1.0);
        }
    }
    // Stacked turrets (inverted Worley: rounded lobes, creases between) with
    // cauliflower on them.
    nz.cut = (nz.big.y - 0.5) * 0.42 + (nz.big.z - 0.5) * 0.2 + (nz.fine - 0.5) * 0.1;
    let g = storm_cells(ids, mask, p, f, nz);
    var field = g.field;
    if (main) {
        g_storm = g;
    }
    if (field < -g.edge * 1.5) {
        return vec2<f32>(0.0);
    }
    if (lod == 0 && length(p - U.eye.xyz) < 9000.0) {
        // Fine erosion right at the surface (near: further away it is
        // below the pixel and only costs steps): crisp at the tops, wispy at the base.
        // (Coarser and weaker in the anvil: its thin plate would show the
        // detail texture's tiling.)
        let dq = vec3<f32>(q.x * 0.8 + q.z * 0.6, q.y + t * 2.0, q.z * 0.8 - q.x * 0.6) * mix(1.0 / 380.0, 1.0 / 900.0, g.anvil);
        let dn = textureSampleLevel(detail_tex, samp_rep, dq, 0.0);
        let df = dn.x * 0.625 + dn.y * 0.25 + dn.z * 0.125;
        field += (df - 0.55) * min(g.edge, 120.0) * mix(1.1, 0.5, g.anvil);
    }
    let d = smoothstep(0.0, g.edge, field) * g.dens * fade;
    return vec2<f32>(d * (0.85 + 0.35 * smoothstep(g.edge, g.edge * 4.0, field)), g.anvil);
}

// B-spline filtered lenticular cover (the grid is coarse: bilinear contours
// would show its texels as steps).
fn lent_smooth(xz: vec2<f32>) -> f32 {
    if (!wx_lent()) {
        return 0.0;
    }
    let n = f32(textureDimensions(lent_tex).x);
    let uv = (xz - U.lent.xy) / U.lent.z;
    if (any(uv < vec2<f32>(0.0)) || any(uv > vec2<f32>(1.0))) {
        return 0.0;
    }
    let st = uv * n - 0.5;
    let i = floor(st);
    let fr = st - i;
    let w0 = (1.0 - fr) * (1.0 - fr) * (1.0 - fr) / 6.0;
    let w1 = (4.0 - 6.0 * fr * fr + 3.0 * fr * fr * fr) / 6.0;
    let w3 = fr * fr * fr / 6.0;
    let w2 = 1.0 - w0 - w1 - w3;
    let s0 = w0 + w1;
    let s1 = w2 + w3;
    let o0 = (i - 0.5 + w1 / s0) / n;
    let o1 = (i + 1.5 + w3 / s1) / n;
    let a = textureSampleLevel(lent_tex, samp_clamp, vec2<f32>(o0.x, o0.y), 0.0).x;
    let b = textureSampleLevel(lent_tex, samp_clamp, vec2<f32>(o1.x, o0.y), 0.0).x;
    let cc = textureSampleLevel(lent_tex, samp_clamp, vec2<f32>(o0.x, o1.y), 0.0).x;
    let d = textureSampleLevel(lent_tex, samp_clamp, vec2<f32>(o1.x, o1.y), 0.0).x;
    return mix(mix(a, b, s1.x), mix(cc, d, s1.x), s1.y);
}

// Lenticular lens field: smooth lenses stacked like plates on the wave crests,
// thickest in the middle, tapering to clean knife edges. x = field (> 0
// inside, metres), y = height fraction in the lens (lighting).
fn lent_env(p: vec3<f32>) -> vec2<f32> {
    let c = lent_smooth(p.xz);
    if (c < 0.2) {
        return vec2<f32>(-1e3, 0.0);
    }
    var best = vec2<f32>(-1e3, 0.0);
    for (var k = 0; k < 3; k++) {
        let kk = f32(k);
        let need = 0.25 + 0.22 * kk;
        if (c < need) {
            continue;
        }
        let cc = clamp((c - need) / (1.0 - need), 0.0, 1.0);
        // Thickest in the middle, tapering to thin knife edges (a lens, not
        // a slab); the upper plates smaller and thinner.
        let h = U.lent2.y * 1.5 * (1.0 - 0.3 * kk) * pow(cc, 0.85);
        // Each plate is a little domed (higher in the middle of the crest).
        let zc = U.lent.w + kk * 560.0 + 90.0 * cc - 30.0;
        let dy = p.y - zc;
        // Lens cross-section: flatter underside, rounded top.
        let hh = select(h * 0.8, h * 1.1, dy > 0.0);
        let fld = hh - abs(dy);
        if (fld > best.x) {
            best = vec2<f32>(fld, clamp(dy / max(hh, 1.0) * 0.5 + 0.5, 0.0, 1.0));
        }
    }
    return best;
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
// (Set while lighting fair-weather cloud near storms: their light rays and
// sky probe ignore the storm field — it would cost a lot and change little.)
var<private> g_no_storm: bool;
fn density(p: vec3<f32>, w0: vec4<f32>, lod: i32) -> f32 {
    let base = U.shape.x;
    let y = p.y - base;
    var w = w0;
    // Thunderstorms and lenticulars (wx/): their own fields (storm shelf and
    // wall clouds hang below the lowest base).
    var dx = 0.0;
    if (wx_storm() && !g_no_storm) {
        if (p.y < U.shape.y) {
            dx = storm_density(p, lod).x;
        }
        // No fair-weather cumulus inside the storm towers.
        if (w.x > 0.003) {
            w.x *= 1.0 - storm_b(p.xz).x;
        }
    }
    if (p.y < U.drift.z - 40.0 || p.y > U.shape.y) {
        return dx;
    }
    if (wx_lent() && abs(p.y - U.lent.w - 560.0) < U.lent2.y * 1.6 + 700.0) {
        let le = lent_env(p);
        if (le.x > -60.0) {
            // Smooth, polished lenses; the air streams through them (the
            // noise drifts, the lens stays): only a faint ripple on the edge.
            let q = p - wind_shape() * 3.0;
            let dn = textureSampleLevel(detail_tex, samp_rep, q * DETAIL_SCALE * 0.3, 0.0);
            let fl = le.x + (dn.x - 0.5) * 22.0;
            dx = max(dx, smoothstep(0.0, 45.0, fl) * 0.85);
        }
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
        let dome = sqrt(w.x) - pow(max(hc, 0.0), 1.6) - tuck;
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
        return dx;
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
    return max(smoothstep(0.0, 0.4, d) * (0.5 + 0.5 * d), dx);
}

// One cell's rain shaft at p (field xz `f`): curtains hanging from the base,
// slanted by the shear, densest in the core, thinning (virga) toward the bottom.
fn rain_cell(i: u32, p: vec3<f32>, f: vec2<f32>, curtain: f32) -> f32 {
    let c0 = SC.c[i * 4u];
    let c2 = SC.c[i * 4u + 2u];
    let c3 = SC.c[i * 4u + 3u];
    let base = c0.w;
    if (p.y > base + 60.0 || c3.y < 0.01) {
        return 0.0;
    }
    let bot = max(c3.z, U.storm.y);
    if (p.y < bot) {
        return 0.0;
    }
    // The rain left the base upwind of here: the shafts slant with the shear.
    let fall = max(base - p.y, 0.0);
    let rc = c0.xy + c2.zw + U.storm.zw * fall;
    let d = length(f - rc) / c3.x;
    // Ragged outline that wanders with height; a dense core.
    let edge = 1.0 + (curtain - 0.5) * 0.5;
    let shaft = 1.0 - smoothstep(edge * 0.3, edge, d);
    if (shaft <= 0.0) {
        return 0.0;
    }
    let core = exp(-d * d * 2.2);
    // Virga thins out as it evaporates; rain reaches the ground in a dense foot.
    let virga = U.storm2.x > 0.3 || c3.z > U.storm.y + 50.0;
    let taper = smoothstep(bot, bot + max(base - bot, 1.0) * select(0.05, 0.55, virga), p.y);
    // (Fading in under the base: no hard line where a ragged base is higher.)
    let top = 1.0 - smoothstep(base - 250.0, base + 60.0, p.y) * 0.6;
    return c3.y * shaft * (0.35 + 0.65 * curtain) * (0.5 + 0.5 * core) * taper * top;
}

// Optical depth (sigma included) toward the sun through the storm envelopes
// beyond the light march's reach (anvils shading the towers and the air
// under them, towers shading each other).
fn storm_sun_od(p: vec3<f32>) -> f32 {
    let sun = U.sun_dir.xyz;
    var od = 0.0;
    var dists = array<f32, 3>(900.0, 2600.0, 6000.0);
    var lens = array<f32, 3>(1100.0, 2600.0, 4800.0);
    for (var k = 0; k < 3; k++) {
        let q = p + sun * dists[k];
        if (q.y > U.storm2.y) {
            break;
        }
        od += storm_density(q, 4).x * lens[k];
    }
    return od * U.shape.z;
}

// Rain shafts / virga under the storms: falling rain (0..1) at p.
fn rain_at(p: vec3<f32>) -> f32 {
    if (!wx_storm() || U.fog.w > 0.5 || p.y > U.storm2.y) {
        return 0.0;
    }
    let sa = storm_a(p.xz);
    if (sa.w < 0.003) {
        return 0.0;
    }
    // The cell raining most here (its own slot in the storm map: a tower
    // nearby must not crowd the shaft out of the cell lists).
    let uv = storm_uv(p.xz);
    if (any(uv < vec2<f32>(0.0)) || any(uv >= vec2<f32>(1.0))) {
        return 0.0;
    }
    let tc = vec2<i32>(uv * U.wmap.w);
    let id = u32(textureLoad(storm_b_tex, tc, 0).w + 0.5);
    if (id == 0u) {
        return 0.0;
    }
    let f = p.xz - U.drift.xy;
    // Curtains: streaks that sway as they fall (warped, only mildly
    // stretched noise: no rectangular columns).
    let q = vec3<f32>(f.x, p.y + U.eye.w * 9.0, f.y);
    let wq = textureSampleLevel(shape_tex, samp_rep, vec3<f32>(q.x / 9000.0, q.y / 9000.0, q.z / 9000.0) + vec3<f32>(0.5, 0.2, 0.8), 0.0);
    let qw = q + vec3<f32>(wq.y - 0.5, 0.0, wq.z - 0.5) * 1600.0;
    let n = textureSampleLevel(shape_tex, samp_rep, vec3<f32>(qw.x / 2600.0, qw.y / 4600.0, qw.z / 2600.0), 0.0);
    let curtain = clamp(smoothstep(0.15, 0.85, n.x * 0.5 + n.y * 0.3 + n.z * 0.2) * 0.65 + 0.3, 0.0, 1.0);
    var r = rain_cell(id - 1u, p, f, curtain);
    // A second shaft overlapping (two cells' rain): the low-level list.
    let t = textureLoad(storm_a_tex, tc, 0);
    let i2 = u32(t.y + 0.5);
    if (i2 > 0u && i2 != id) {
        r = max(r, rain_cell(i2 - 1u, p, f, curtain));
    }
    return r;
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

fn cloud_shadow_round(p: vec3<f32>) -> f32 {
    let sun = U.sun_dir.xyz;
    let xz = p.xz - sun.xz * (p.y / max(sun.y, 0.05));
    let uv = (xz - U.shadow.xy) * U.shadow.z + 0.5;
    let s = textureSampleLevel(shadow_tex, samp_clamp, uv, 0.0).x;
    return mix(s, 1.0, smoothstep(0.3, 0.48, length(uv - 0.5)));
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
        // (U.sun_dir.w: the key light's — sun's or moon's — irradiance relative to sun_e().)
        let sc = (m.ray * pr + vec3<f32>(m.mie * pm)) * sun_trans(p.y, sun.y) * sun_e() * U.sun_dir.w;
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

// Optical depth toward the sun through a storm (sigma included): the storm
// field alone (no fair-weather cloud inside a storm), three growing steps
// (the turrets' own billows for the first, the envelope beyond).
fn storm_light_od(p: vec3<f32>) -> f32 {
    let sun = U.sun_dir.xyz;
    var steps = array<f32, 3>(40.0, 160.0, 600.0);
    var od = 0.0;
    var tl = 0.0;
    for (var j = 0; j < 3; j++) {
        let l = steps[j];
        let q = p + sun * (tl + l * 0.5);
        tl += l;
        od += storm_density(q, select(4, 1, j == 0)).x * l;
    }
    return od * U.shape.z;
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

// Distance along the ray to the nearest fair-weather layer it can enter
// (0 inside one): how far a storm-free stride may go.
fn fair_slab_dist(p: vec3<f32>, rd: vec3<f32>) -> f32 {
    var d = 1e9;
    for (var k = 0; k < 4; k++) {
        var lo = 0.0;
        var hi = -1.0;
        if (k == 0 && U.seed.z != 0u && U.cells.y > 0.0) {
            lo = U.shape.x - 60.0;
            hi = U.shape.x + U.cells2.y + 60.0;
        } else if (k == 1 && U.cells2.z > 0.0) {
            lo = U.layers.x - 60.0;
            hi = U.layers.x + U.shape.w + 60.0;
        } else if (k == 2 && U.layers.y > 0.0) {
            lo = U.layers.z - 60.0;
            hi = U.layers.z + U.layers.w + 60.0;
        } else if (k == 3 && wx_lent()) {
            lo = U.lent.w - U.lent2.y * 2.0 - 150.0;
            hi = U.lent.w + 1120.0 + U.lent2.y * 2.0 + 150.0;
        }
        if (hi < lo) {
            continue;
        }
        if (p.y >= lo && p.y <= hi) {
            return 0.0;
        }
        if (p.y < lo && rd.y > 1e-5) {
            d = min(d, (lo - p.y) / rd.y);
        } else if (p.y > hi && rd.y < -1e-5) {
            d = min(d, (hi - p.y) / rd.y);
        }
    }
    return d;
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

// Storm weather below Ultra traces a checkerboard: half the rays per frame,
// packed into the left half of the trace target (a half-width viewport:
// whole warps skip work); fs_temporal unpacks and fills the gaps from the
// neighbours and the history.
fn cb_half() -> bool {
    return U.storm2.z > 0.5;
}
fn cb_parity(y: i32) -> i32 {
    return (y + i32(U.seed.y & 1u)) & 1;
}
// Is trace pixel q traced this frame, and where is it stored?
fn cb_traced(q: vec2<i32>) -> bool {
    return !cb_half() || (q.x & 1) == cb_parity(q.y);
}
fn cb_store(q: vec2<i32>) -> vec2<i32> {
    return select(q, vec2<i32>(q.x >> 1, q.y), cb_half());
}

@fragment
fn fs_trace(in: FsOut) -> TraceOut {
    let fpx = vec2<i32>(in.pos.xy);
    let px = select(fpx, vec2<i32>(fpx.x * 2 + cb_parity(fpx.y), fpx.y), cb_half());
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
    // (Thunderstorms are seen from much further than fair-weather cloud.)
    // (Not in the water reflection: it stays cheap.)
    limit = min(limit, select(MAX_T, 75000.0, wx_storm() && U.fog.w < 0.5));

    var o: TraceOut;
    var T = 1.0;
    var L = vec3<f32>(0.0);
    var tsum = 0.0;
    var wsum = 0.0;

    var ylo = U.drift.z - 40.0;
    if (wx_storm() && U.fog.w < 0.5) {
        ylo = min(ylo, U.storm.y);
    }
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
        var qsteps = array<f32, 4>(1.6, 1.25, 1.0, 0.75);
        let qstep = select(1.0, 1.7, U.fog.w > 0.5) * qsteps[min(U.seed.w, 3u)];
        var t = t0 + step_at(t0) * qstep * jit;
        // Cheap coarse march (base shape only) until a cloud is hit, then step
        // back and march finely with full detail + lighting until the ray has
        // been out of cloud for a while.
        var fine = false;
        var empty = 0;
        // Storm weather strides long through clear air; a hit after a long
        // stride goes back to where the air was clear and walks in normally
        // (`careful`), so no front surface is skipped.
        var last = 0.0;
        var careful = false;
        // Two decks + cumulus can need more steps than one layer.
        var max_it = select(220, 300, U.cells2.z + U.layers.y > 0.0);
        if (wx_storm() && U.fog.w < 0.5) {
            max_it += 100;
        }
        max_it = i32(f32(max_it) / qsteps[min(U.seed.w, 3u)]);
        for (var i = 0; i < max_it; i++) {
            if (t >= t1 || T < 0.008) {
                break;
            }
            let base_dt = step_at(t) * qstep;
            let p = ro + rd * t;
            let w = weather(p.xz);
            if (!fine) {
                var occ = w.x + w.z + w.w;
                if (wx_storm()) {
                    let sa = storm_a(p.xz);
                    occ += sa.x + sa.w;
                }
                if (wx_lent()) {
                    occ += lent_at(p.xz);
                }
                if (occ < 0.003) {
                    // (Under the cloud with storms about, the march starts at the
                    // eye: empty air is crossed in big strides.)
                    var stride = base_dt * 2.5;
                    if (wx_storm() && !careful) {
                        // Clear of the storm maps (their footprints are
                        // conservative by a kilometre) and of any fair-weather
                        // layer: long strides.
                        stride = max(stride, max(250.0, min(1200.0, fair_slab_dist(p, rd))));
                    }
                    t += stride;
                    last = stride;
                    continue;
                }
                g_skip = 0.0;
                if (density(p, w, 5) > 0.0 || rain_at(p) > 0.002) {
                    if (last > base_dt * 2.6 && !careful) {
                        t = max(t - last, t0);
                        careful = true;
                        last = 0.0;
                        continue;
                    }
                    careful = false;
                    fine = true;
                    empty = 0;
                    // (Jittered: a stride over clear air lands every pixel
                    // on the same smooth surface: no coherent step bands.)
                    t = max(t - base_dt * (0.75 + 1.5 * jit), t0);
                    continue;
                }
                // Clear of the storm clouds by a known distance: stride over
                // it (but not into a fair-weather layer).
                var stride = base_dt * 1.5;
                if (g_skip > 0.0) {
                    var lim = fair_slab_dist(p, rd);
                    if (lim <= 0.0 && w.x + w.z + w.w < 0.003) {
                        lim = 600.0;
                    }
                    stride = max(stride, min(g_skip * 0.4, lim));
                }
                if (careful) {
                    stride = base_dt * 1.5;
                }
                t += stride;
                last = stride;
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
            // (Storm weather: far samples skip the finest erosion and take
            // fewer light steps.)
            let far = wx_storm() && t > 16000.0;
            let d = density(p, w, select(0, 3, far));
            let gs = g_storm;
            if (gs.field > -gs.edge) {
                // Storm cloud: big and dense; coarser steps at the surface
                // (the jittered start keeps them from banding).
                dt = base_dt * (0.6 + 0.9 * (1.0 - T));
            }
            if (gs.anvil > 0.5 && gs.field > gs.edge && d < 0.4) {
                // Inside the thin ice of the anvil: longer steps (it is smooth).
                dt = max(dt * 1.5, gs.edge * 0.7);
            }
            let rr = rain_at(p);
            if (d <= 0.002 && rr > 0.002) {
                // Rain shaft / virga only: grey curtains, lit by the dim light
                // under the storm and the sun from the side (forward scattering).
                empty = 0;
                let sig = rr * 3.2e-3;
                // The low sun shines in under the anvil and lights the shafts
                // (big drops scatter strongly forward); the storm above shades them.
                // (The shadow map fades out radially here: its square edge
                // would cut straight lines through the far shafts.)
                var sun_vis = cloud_shadow_round(p) * smoothstep(-0.02, 0.05, sun.y);
                if (sun_vis > 0.03) {
                    let sod = storm_sun_od(p);
                    sun_vis *= mix(exp(-sod), 1.0 / (1.0 + 0.2 * sod), 0.3);
                }
                let amb_r = (amb_bot * 0.1 + vec3<f32>(0.78, 0.85, 1.0) * luma(amb_top) * 0.07) * mix(1.0, 0.55, rr);
                let S = E * sun_vis * (0.04 + 0.1 * hg(mu, 0.75)) + amb_r;
                // (Rain is a smooth medium: long steps.)
                let rdt = max(dt * 1.8, clamp(t * 0.035, 70.0, 420.0));
                let tr = exp(-sig * rdt);
                let wgt = T * (1.0 - tr);
                L += S * wgt;
                tsum += t * wgt;
                wsum += wgt;
                T *= tr;
                t += rdt;
                continue;
            }
            if (d > 0.002) {
                empty = 0;
                let sig = d * U.shape.z + rr * 3.2e-3;
                // Smooth decks need fewer light samples than billowy cumulus.
                let in_storm = gs.field > -gs.edge;
                var od = 0.0;
                if (in_storm) {
                    od = storm_light_od(p);
                } else {
                    g_no_storm = true;
                    od = light_od(p, select(6, 4, deck_only || far));
                    g_no_storm = false;
                }
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
                // (In a storm the cloud overhead darkens it instead: storm_dark.)
                var sky_vis = 1.0;
                if (!in_storm) {
                    let pu = p + vec3<f32>(0.0, 80.0, 0.0);
                    g_no_storm = true;
                    let occ = density(pu, w, 1) * 160.0;
                    g_no_storm = false;
                    sky_vis = exp(-occ * U.shape.z * 0.12);
                }
                // Inside a thunderstorm: height within the tower, and the
                // kilometres of cloud overhead darken it (the dark base).
                var storm_dark = 1.0;
                var amb_b = amb_bot;
                if (wx_storm()) {
                    if (gs.field > -gs.edge) {
                        hf = clamp(gs.h, 0.0, 1.0);
                        // Sky light diffusing down through the cloud overhead
                        // (near the surface the sky to the side still lights it);
                        // the ground under the storm is in its shadow.
                        let deep = 1.0 / (1.0 + 0.0012 * gs.over);
                        storm_dark = mix(mix(1.0, deep, 0.3), deep, smoothstep(0.0, 600.0, gs.field));
                        amb_b = amb_bot * 0.4;
                        // Multiple scattering in a very thick cloud: sunlight
                        // diffuses in from the lit turrets around, so walls
                        // away from the sun glow instead of going flat grey.
                        sunL += 0.035 * exp(-max(gs.field, 0.0) / 350.0) * (1.0 - gs.anvil * 0.5);
                        // Anvils / towers between the sun and this sample (beyond
                        // the light march): deep shadow, a little diffuse leak.
                        var sod = 0.0;
                        if (sunL > 0.004) {
                            sod = storm_sun_od(p);
                        }
                        sunL *= mix(exp(-sod), 1.0 / (1.0 + 0.15 * sod), 0.35);
                    }
                }
                var amb = mix(amb_b, amb_top * (0.35 + 0.65 * sky_vis), smoothstep(-0.05, 0.7, hf)) * storm_dark;
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
    let skipped = !cb_traced(px);
    var cur = vec4<f32>(0.0);
    var cd = vec2<f32>(0.0);
    if (!skipped) {
        cur = textureLoad(trace_col, cb_store(px), 0);
        cd = textureLoad(trace_dep, cb_store(px), 0).xy;
    }
    var o: TraceOut;
    // Neighbourhood statistics for variance clipping (traced rays only: in
    // the storm checkerboard every other ray was skipped, a < 0).
    var m1 = vec4<f32>(0.0);
    var m2 = vec4<f32>(0.0);
    var nn = 0.0;
    var dsum = 0.0;
    var dlim = 0.0;
    var dn = 0.0;
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let q = clamp(px + vec2<i32>(x, y), vec2<i32>(0), dims - 1);
            if (!cb_traced(q)) {
                continue;
            }
            let c = textureLoad(trace_col, cb_store(q), 0);
            m1 += c;
            m2 += c * c;
            nn += 1.0;
            if (abs(x) + abs(y) == 1) {
                let qd = textureLoad(trace_dep, cb_store(q), 0);
                dsum += qd.y;
                dlim += qd.x;
                dn += 1.0;
            }
        }
    }
    m1 /= max(nn, 1.0);
    m2 /= max(nn, 1.0);
    if (skipped) {
        // Not traced this frame: the neighbours' mean stands in (and the
        // history below carries the detail).
        cur = m1;
        cd = select(vec2<f32>(FAR_DEPTH), vec2<f32>(dlim, dsum) / max(dn, 1.0), dn > 0.0);
    }
    if (U.sun_col.w >= 0.999) {
        o.col = cur;
        o.dep = cd;
        return o;
    }
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
            // (An interpolated ray only nudges the history.)
            res = mix(h, cur, select(U.sun_col.w, U.sun_col.w * 0.5, skipped));
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
    let d = min(textureLoad(dist_tex, px, 0).x, select(MAX_T, 90000.0, wx_storm()));
    let hdims = vec2<i32>(textureDimensions(hist_col));
    let hp = (vec2<f32>(px) + 0.5) * 0.5 - 0.5;
    let b = vec2<i32>(floor(hp));
    let f = hp - floor(hp);
    var acc = vec4<f32>(0.0);
    var wsum = 0.0;
    var dsum = 0.0;
    var dwsum = 1e-6;
    for (var k = 0; k < 4; k++) {
        let o = vec2<i32>(k & 1, k >> 1);
        let tc = clamp(b + o, vec2<i32>(0), hdims - 1);
        let c = textureLoad(hist_col, tc, 0);
        let hd = textureLoad(hist_dep, tc, 0);
        let l = hd.x;
        let bw = select(1.0 - f.x, f.x, o.x == 1) * select(1.0 - f.y, f.y, o.y == 1);
        let rel = abs(l - d) / max(d, 1.0);
        let dw = 1.0 / (1.0 + rel * rel * 4000.0);
        let w = bw * dw + 1e-5;
        acc += c * w;
        // (Cloud depth weighted by opacity: clear neighbours' far depth must
        // not drag the silhouette's depth behind the cloud.)
        let op = 1.0 - clamp(c.a, 0.0, 1.0);
        dsum += hd.y * w * op;
        dwsum += w * op;
        wsum += w;
    }
    let c = acc / wsum;
    var rgb = c.rgb;
    let T = clamp(c.a, 0.0, 1.0);
    // Lightning: the flash lights the cloud from inside (diffusing out over
    // kilometres) and the air / rain between (single scattering along the
    // ray). Added after the temporal filter: no smearing, no ghosts.
    if (U.flash[0].w > 0.0 || U.flash[1].w > 0.0) {
        let uv = (vec2<f32>(px) + 0.5) / U.screen.xy;
        let rd = ray_dir(uv);
        let cd = min(select(dsum / dwsum, FAR_DEPTH, dwsum < 1e-4), d);
        let col = vec3<f32>(0.82, 0.88, 1.0);
        for (var k = 0; k < 2; k++) {
            let fl = U.flash[k];
            if (fl.w <= 0.0) {
                continue;
            }
            let cp = rd * cd;
            let dc = distance(cp, fl.xyz);
            // The flash diffuses through kilometres of cloud: the light that
            // leaves the surface falls off with the distance travelled inside
            // (a thick cloud's interior glows, thin edges let it through dimly).
            let glow = fl.w / (PI * (dc * dc + 1.5e6)) * exp(-dc / 2500.0);
            let opac = 1.0 - T;
            rgb += col * glow * opac * opac * 0.7;
            // Air glow up to the cloud / scene: ∫ dt / (r² + h²) along the ray.
            // Haze scatters it; rain and cloud between swallow most of it
            // (no milky veil over the whole view).
            let lim = min(d, 40000.0);
            let t0 = dot(fl.xyz, rd);
            let h = max(length(fl.xyz - rd * t0), 300.0);
            let I = (atan((lim - t0) / h) - atan(-t0 / h)) / h;
            let sig = 3.0e-6 * U.fog.y + 2.5e-6 * clamp(U.storm.x, 0.0, 1.0);
            rgb += col * fl.w * sig * I / (4.0 * PI) * exp(-length(fl.xyz) / 15000.0);
        }
    }
    return vec4<f32>(rgb, T);
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
        od += density(p, w, 4) * dt;
    }
    // Forward scattering keeps cloud shadows from going fully black.
    let tr = exp(-od * U.shape.z * 0.6);
    return vec4<f32>(mix(0.06, 1.0, tr));
}
