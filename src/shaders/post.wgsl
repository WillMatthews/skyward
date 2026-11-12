// Bloom (downsample / upsample chain) and final tonemap.

@group(0) @binding(0) var src: texture_2d<f32>;
@group(0) @binding(1) var samp: sampler;
@group(0) @binding(2) var bloom_tex: texture_2d<f32>;

struct PostU {
    params: vec4<f32>, // x exposure, y bloom strength, z first-downsample flag, w unused
    texel: vec4<f32>,  // xy = 1/size of src
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
    return 1.0 / (1.0 + dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)) * 0.25);
}

// 13-tap downsample (Jimenez 2014).
@fragment
fn fs_down(in: FsOut) -> @location(0) vec4<f32> {
    let t = pu.texel.xy;
    let uv = in.uv;
    let a = s(uv + t * vec2<f32>(-2.0, -2.0));
    let b = s(uv + t * vec2<f32>(0.0, -2.0));
    let c = s(uv + t * vec2<f32>(2.0, -2.0));
    let d = s(uv + t * vec2<f32>(-2.0, 0.0));
    let e = s(uv);
    let f = s(uv + t * vec2<f32>(2.0, 0.0));
    let gg = s(uv + t * vec2<f32>(-2.0, 2.0));
    let h = s(uv + t * vec2<f32>(0.0, 2.0));
    let i = s(uv + t * vec2<f32>(2.0, 2.0));
    let j = s(uv + t * vec2<f32>(-1.0, -1.0));
    let k = s(uv + t * vec2<f32>(1.0, -1.0));
    let l = s(uv + t * vec2<f32>(-1.0, 1.0));
    let m = s(uv + t * vec2<f32>(1.0, 1.0));
    if (pu.params.z > 0.5) {
        // first level: Karis average to kill fireflies
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
    var r = e * 0.125;
    r += (a + c + gg + i) * 0.03125;
    r += (b + d + f + h) * 0.0625;
    r += (j + k + l + m) * 0.125;
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

@fragment
fn fs_tonemap(in: FsOut) -> @location(0) vec4<f32> {
    let hdr = textureSampleLevel(src, samp, in.uv, 0.0).rgb;
    let bloom = textureSampleLevel(bloom_tex, samp, in.uv, 0.0).rgb;
    let expo = atmos.expo.x * pu.params.x;
    var c = mix(hdr, bloom, pu.params.y) * expo;
    c = agx_eotf(agx_look(agx(c)));
    // subtle vignette
    let v = in.uv - 0.5;
    c *= 1.0 - dot(v, v) * 0.18;
    // dither against banding (in display space: ~1/255 steps)
    let n = fract(sin(dot(in.pos.xy, vec2<f32>(12.9898, 78.233))) * 43758.5453) - 0.5;
    c = pow(max(pow(c, vec3<f32>(1.0 / 2.2)) + n / 255.0, vec3<f32>(0.0)), vec3<f32>(2.2));
    return vec4<f32>(c, 1.0);
}
