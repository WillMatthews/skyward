// Bloom (downsample / upsample chain) and final tonemap (+ contrast-adaptive
// sharpening after TAA, lens dirt, chromatic aberration, film grain,
// display brightness / gamma, HDR output).

@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var bloom_tex: texture_2d<f32>;

struct PostU {
    params: vec4<f32>, // x exposure, y bloom strength, z first-downsample flag (tonemap: look power), w saturation (first downsample: glare cap, 0 = none)
    texel: vec4<f32>,  // xy = 1/size of src, z = 1 / pre-exposure
    fx: vec4<f32>,     // tonemap: x sharpen 0..1, y unused, z lens dirt, w chromatic aberration
    fx2: vec4<f32>,    // tonemap: x film grain, y time (s), z brightness, w display gamma
    hdr: vec4<f32>,    // tonemap: x HDR output, y paper white / 80 nits, z peak / paper white
};
@group(0) @binding(3) var<uniform> pu: PostU;
struct AtmosOut {
    sh: array<vec4<f32>, 4>,
    sun_eye: vec4<f32>,
    expo: vec4<f32>,
};
@group(0) @binding(4) var<storage, read> atmos: AtmosOut;

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

fn s(uv: vec2<f32>) -> vec3<f32> {
    return textureSampleLevel(src, samp, uv, 0.0).rgb;
}

fn karis(c: vec3<f32>) -> f32 {
    // On the exposed brightness (the HDR image is pre-exposed at night;
    // x 0.58 keeps the daytime weighting as it was, expo ~1.7 at noon).
    return 1.0 / (1.0 + dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)) * atmos.expo.x * 0.58 * 0.25);
}

// What feeds the glare (first downsample only): at night the point lights
// (lamps, landing lights, strobes) are thousands of times brighter than the
// adapted scene, and their full energy spread over the bloom chain turns
// into a milky veil across the screen. Above `params.w` (exposed luminance;
// 0 = no cap, by day) the excess is dropped: lights keep a tight glow.
fn glare_cap(c: vec3<f32>) -> vec3<f32> {
    let cap = pu.params.w;
    if (cap <= 0.0) {
        return c;
    }
    let l = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)) * atmos.expo.x;
    return c * min(1.0, cap / max(l, 1e-6));
}

// 13-tap downsample (Jimenez 2014).
@fragment
fn fs_down(in: FsOut) -> @location(0) vec4<f32> {
    let t = pu.texel.xy;
    let uv = in.uv;
    let a0 = s(uv + t * vec2<f32>(-2.0, -2.0));
    let b0 = s(uv + t * vec2<f32>(0.0, -2.0));
    let c0 = s(uv + t * vec2<f32>(2.0, -2.0));
    let d0 = s(uv + t * vec2<f32>(-2.0, 0.0));
    let e0 = s(uv);
    let f0 = s(uv + t * vec2<f32>(2.0, 0.0));
    let gg0 = s(uv + t * vec2<f32>(-2.0, 2.0));
    let h0 = s(uv + t * vec2<f32>(0.0, 2.0));
    let i0 = s(uv + t * vec2<f32>(2.0, 2.0));
    let j0 = s(uv + t * vec2<f32>(-1.0, -1.0));
    let k0 = s(uv + t * vec2<f32>(1.0, -1.0));
    let l0 = s(uv + t * vec2<f32>(-1.0, 1.0));
    let m0 = s(uv + t * vec2<f32>(1.0, 1.0));
    if (pu.params.z > 0.5) {
        // first level: Karis average to kill fireflies
        let a = glare_cap(a0);
        let b = glare_cap(b0);
        let c = glare_cap(c0);
        let d = glare_cap(d0);
        let e = glare_cap(e0);
        let f = glare_cap(f0);
        let gg = glare_cap(gg0);
        let h = glare_cap(h0);
        let i = glare_cap(i0);
        let j = glare_cap(j0);
        let k = glare_cap(k0);
        let l = glare_cap(l0);
        let m = glare_cap(m0);
        let g0 = (a + b + d + e) * 0.25;
        let g1 = (b + c + e + f) * 0.25;
        let g2 = (d + e + gg + h) * 0.25;
        let g3 = (e + f + h + i) * 0.25;
        let g4 = (j + k + l + m) * 0.25;
        let w0 = karis(g0) * 0.125;
        let w1 = karis(g1) * 0.125;
        let w2 = karis(g2) * 0.125;
        let w3 = karis(g3) * 0.125;
        let w4 = karis(g4) * 0.5;
        let r = (g0 * w0 + g1 * w1 + g2 * w2 + g3 * w3 + g4 * w4) / (w0 + w1 + w2 + w3 + w4);
        return vec4<f32>(r, 1.0);
    }
    var r = e0 * 0.125;
    r += (a0 + c0 + gg0 + i0) * 0.03125;
    r += (b0 + d0 + f0 + h0) * 0.0625;
    r += (j0 + k0 + l0 + m0) * 0.125;
    return vec4<f32>(r, 1.0);
}

// 3x3 tent upsample, additively blended into the next larger level.
@fragment
fn fs_up(in: FsOut) -> @location(0) vec4<f32> {
    let t = pu.texel.xy;
    let uv = in.uv;
    var r = s(uv) * 4.0;
    r += (s(uv + vec2<f32>(-t.x, 0.0)) + s(uv + vec2<f32>(t.x, 0.0)) + s(uv + vec2<f32>(0.0, -t.y)) + s(uv + vec2<f32>(0.0, t.y))) * 2.0;
    r += s(uv + vec2<f32>(-t.x, -t.y)) + s(uv + vec2<f32>(t.x, -t.y)) + s(uv + vec2<f32>(-t.x, t.y)) + s(uv + vec2<f32>(t.x, t.y));
    return vec4<f32>(r / 16.0, 1.0);
}

// AgX (Troy Sobotka) with the polynomial sigmoid fit by Benjamin Wrensch.
fn agx_contrast(x: vec3<f32>) -> vec3<f32> {
    let x2 = x * x;
    let x4 = x2 * x2;
    return 15.5 * x4 * x2 - 40.14 * x4 * x + 31.96 * x4 - 6.868 * x2 * x + 0.4298 * x2 + 0.1191 * x - 0.00232;
}

fn agx(c: vec3<f32>) -> vec3<f32> {
    let m = mat3x3<f32>(
        vec3<f32>(0.842479062253094, 0.0423282422610123, 0.0423756549057051),
        vec3<f32>(0.0784335999999992, 0.878468636469772, 0.0784336),
        vec3<f32>(0.0792237451477643, 0.0791661274605434, 0.879142973793104));
    let min_ev = -12.47393;
    let max_ev = 4.026069;
    var x = m * c;
    x = clamp(log2(max(x, vec3<f32>(1e-10))), vec3<f32>(min_ev), vec3<f32>(max_ev));
    x = (x - min_ev) / (max_ev - min_ev);
    return agx_contrast(x);
}

fn agx_look(v: vec3<f32>) -> vec3<f32> {
    // Gentle "punchy" look: a little extra contrast and saturation.
    let slope = vec3<f32>(1.0);
    let power = vec3<f32>(pu.params.z);
    let sat = pu.params.w;
    var x = pow(max(v * slope, vec3<f32>(0.0)), power);
    let l = dot(x, vec3<f32>(0.2126, 0.7152, 0.0722));
    return l + sat * (x - l);
}

fn agx_eotf(v: vec3<f32>) -> vec3<f32> {
    let mi = mat3x3<f32>(
        vec3<f32>(1.19687900512017, -0.0528968517574562, -0.0529716355144438),
        vec3<f32>(-0.0980208811401368, 1.15190312990417, -0.0980434501171241),
        vec3<f32>(-0.0990297440797205, -0.0989611768448433, 1.15107367264116));
    // Output is display-referred sRGB-ish (gamma 2.2); linearise for the sRGB target.
    return pow(max(mi * v, vec3<f32>(0.0)), vec3<f32>(2.2));
}

// Scene radiance per cd/m² (== common.wgsl LUX).
const LUX: f32 = 1.953125e-5;

// Night vision: below a few cd/m² the rods take over from the cones — colour
// fades and what is left shifts toward blue (Purkinje). Mesopic blend on the
// absolute luminance: photopic above ~5 cd/m², scotopic below ~0.005.
fn night_vision(c: vec3<f32>) -> vec3<f32> {
    // (texel.z: 1 / pre-exposure of the HDR image.)
    let lum = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)) * pu.texel.z / LUX;
    let lg = log2(max(lum, 1e-7));
    let s = 1.0 - smoothstep(-7.6, 2.3, lg);
    // Rod (scotopic) luminance from linear sRGB, peak ~507 nm.
    let scot = max(dot(c, vec3<f32>(0.02, 0.58, 0.48)), 0.0);
    let blue = vec3<f32>(0.56, 0.74, 1.0) / 0.711;
    return mix(c, blue * scot, s * 0.78);
}

// Contrast-adaptive sharpening (AMD CAS) on the HDR image: weights from the
// tonemapped-ish neighbourhood, applied to HDR values.
fn sharpened(uv: vec2<f32>, amount: f32) -> vec3<f32> {
    let t = pu.texel.xy;
    let c = s(uv);
    if (amount <= 0.0) {
        return c;
    }
    let n = s(uv + vec2<f32>(0.0, -t.y));
    let so = s(uv + vec2<f32>(0.0, t.y));
    let e = s(uv + vec2<f32>(t.x, 0.0));
    let w = s(uv + vec2<f32>(-t.x, 0.0));
    let k = atmos.expo.x;
    let lc = luma_t(c, k);
    let ln = luma_t(n, k);
    let ls = luma_t(so, k);
    let le = luma_t(e, k);
    let lw = luma_t(w, k);
    let mn = min(lc, min(min(ln, ls), min(le, lw)));
    let mx = max(lc, max(max(ln, ls), max(le, lw)));
    let amp = sqrt(clamp(min(mn, 1.0 - mx) / max(mx, 1e-4), 0.0, 1.0));
    let wgt = -amp / mix(8.0, 5.0, amount);
    let res = (c + (n + so + e + w) * wgt) / (1.0 + 4.0 * wgt);
    return max(res, vec3<f32>(0.0));
}

fn luma_t(c: vec3<f32>, k: f32) -> f32 {
    let l = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)) * k;
    return l / (1.0 + l);
}

fn hash2(p: vec2<f32>) -> f32 {
    return fract(sin(dot(p, vec2<f32>(12.9898, 78.233))) * 43758.5453);
}

// Lens dirt: soft round smudges of random size and strength, one per cell
// (jittered), over a 3×3 cell neighbourhood.
fn smudges(p: vec2<f32>) -> f32 {
    let c = floor(p);
    var acc = 0.0;
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let cell = c + vec2<f32>(f32(x), f32(y));
            let h = vec3<f32>(hash2(cell), hash2(cell + 17.31), hash2(cell + 41.7));
            let centre = cell + vec2<f32>(h.x, h.y);
            let r = 0.25 + 0.6 * h.z;
            let d = length(p - centre) / r;
            let w = 1.0 - smoothstep(0.0, 1.0, d);
            acc += w * w * (0.3 + 0.7 * hash2(cell + 5.13));
        }
    }
    return clamp(acc, 0.0, 1.0);
}

@fragment
fn fs_tonemap(in: FsOut) -> @location(0) vec4<f32> {
    var hdr = sharpened(in.uv, pu.fx.x);
    if (pu.fx.w > 0.0) {
        // Chromatic aberration: red and blue pulled apart toward the edges.
        let d = (in.uv - 0.5) * 0.0022 * pu.fx.w;
        hdr = vec3<f32>(s(in.uv + d).r, hdr.g, s(in.uv - d).b);
    }
    var bloom = textureSampleLevel(bloom_tex, samp, in.uv, 0.0).rgb;
    if (pu.fx.z > 0.0) {
        // Dirt on the lens, lit by the glare around it.
        let q = in.uv * vec2<f32>(pu.texel.y / pu.texel.x, 1.0);
        let dirt = smudges(q * 5.0) * 0.7 + smudges(q * 13.0 + 3.7) * 0.3;
        bloom *= 1.0 + dirt * 3.5 * pu.fx.z;
    }
    let expo = atmos.expo.x * pu.params.x;
    // Night: the eye adapts locally (lit windows don't blind it, the dark
    // sky stays readable): compress the range around mid grey, more the
    // darker the adaptation (exposure multiplier ~2 by day, 1e3..1e5 at
    // night). The bloom (glare) is added afterwards, linearly, so its faint
    // tails aren't lifted into big haloes.
    var c = night_vision(hdr) * expo * (1.0 - pu.params.y);
    // (Adaptation: the exposure of absolute radiance = expo x pre-exposure.)
    let nk = smoothstep(3.9, 8.5, log(max(atmos.expo.x / pu.texel.z, 1.0)));
    let gam = mix(1.0, 0.58, nk);
    let l = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
    if (l > 1e-9) {
        c *= pow(l / 0.18, gam - 1.0);
    }
    // Glare follows the same compression where it is bright, but faint glare
    // is never amplified.
    var b = night_vision(bloom) * expo * pu.params.y;
    let lb = dot(b, vec3<f32>(0.2126, 0.7152, 0.0722));
    if (lb > 0.05) {
        b *= pow(lb / 0.05, gam - 1.0);
    }
    c += b;
    c = agx_eotf(agx_look(agx(c)));
    // subtle vignette
    let v = in.uv - 0.5;
    c *= 1.0 - dot(v, v) * 0.18;
    // Display space: brightness (black lift / crush), gamma, grain, dither.
    var d = pow(max(c, vec3<f32>(0.0)), vec3<f32>(1.0 / 2.2));
    if (pu.fx2.z != 0.0) {
        d = clamp(d + pu.fx2.z * 0.25 * (1.0 - d) * (1.0 - d), vec3<f32>(0.0), vec3<f32>(1.0));
    }
    if (pu.fx2.w > 0.0 && abs(pu.fx2.w - 2.2) > 0.001) {
        d = pow(d, vec3<f32>(pu.fx2.w / 2.2));
    }
    if (pu.fx2.x > 0.0) {
        // Film grain: luminance noise, strongest in the mid-tones, new every frame.
        let gn = hash2(in.pos.xy + vec2<f32>(fract(pu.fx2.y * 7.31) * 913.0, fract(pu.fx2.y * 3.17) * 571.0)) - 0.5;
        let mid = 4.0 * dot(d, vec3<f32>(0.333)) * (1.0 - dot(d, vec3<f32>(0.333)));
        d += vec3<f32>(gn * 0.09 * pu.fx2.x * (0.35 + 0.65 * mid));
    }
    // dither against banding (in display space: ~1/255 steps)
    let n = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453) - 0.5;
    c = pow(max(d + n / 255.0, vec3<f32>(0.0)), vec3<f32>(2.2));
    if (pu.hdr.x > 0.5) {
        // HDR (scRGB, 1.0 = 80 nits): diffuse white at the paper white,
        // the top of the AgX curve expanded toward the display's peak.
        let l = dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
        let boost = 1.0 + (pu.hdr.z - 1.0) * pow(smoothstep(0.6, 1.0, l), 2.0);
        c = c * pu.hdr.y * boost;
    }
    return vec4<f32>(c, 1.0);
}
