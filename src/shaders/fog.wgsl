// Ground fog / mist / precipitation haze, applied per pixel after the clouds
// (gfx.rs). atmos_core.wgsl is prepended. The fog is a shallow, horizontally
// uniform layer (density depends on height only), so the optical depth along
// each pixel's ray is integrated exactly here instead of through the coarse
// aerial-perspective froxels. Output is premultiplied: dst = rgb + dst * a.

struct FogU {
    inv_view_proj: mat4x4<f32>, // eye-relative (rotation only)
    eye: vec4<f32>,             // xyz, w = fog extinction at the ground (/km)
    fog: vec4<f32>,             // x fog top (m), y cloud cover, z top-of-atmosphere sun irradiance, w haze
    sun: vec4<f32>,             // xyz toward the sun, w = screen width
    screen: vec4<f32>,          // w, h, 1/w, 1/h
};

@group(0) @binding(0) var<uniform> F: FogU;
@group(0) @binding(1) var dist_tex: texture_2d<f32>;
@group(0) @binding(2) var trans_tex: texture_2d<f32>;
@group(0) @binding(3) var lin: sampler;
struct AtmosOut {
    sh: array<vec4<f32>, 4>,
    sun_eye: vec4<f32>,
    expo: vec4<f32>,
};
@group(0) @binding(4) var<storage, read> atmos: AtmosOut;

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

fn density(h: f32) -> f32 {
    let top = max(F.fog.x, 1.0);
    return F.eye.w * (1.0 - smoothstep(top * 0.55, top, h));
}

fn irr(n: vec3<f32>) -> vec3<f32> {
    return max(atmos.sh[0].rgb + atmos.sh[1].rgb * n.x + atmos.sh[2].rgb * n.y + atmos.sh[3].rgb * n.z, vec3<f32>(0.0));
}

@fragment
fn fs_fog(in: FsOut) -> @location(0) vec4<f32> {
    let px = vec2<i32>(in.pos.xy);
    let uv = in.pos.xy * F.screen.zw;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let p = F.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    let rd = normalize(p.xyz / p.w);
    let h0 = F.eye.y;
    let top = F.fog.x;
    var tmax = textureLoad(dist_tex, px, 0).x;
    if (tmax >= 60000.0) {
        tmax = 1.0e6;
    }
    // Part of the ray inside the fog slab (h <= top).
    var t0 = 0.0;
    var t1 = tmax;
    if (abs(rd.y) > 1e-5) {
        let tt = (top - h0) / rd.y;
        if (h0 > top) {
            if (rd.y >= 0.0) {
                return vec4<f32>(0.0, 0.0, 0.0, 1.0);
            }
            t0 = tt;
        } else if (rd.y > 0.0) {
            t1 = min(t1, tt);
        }
    } else if (h0 > top) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    t1 = min(t1, 200000.0);
    if (t1 <= t0) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    // Optical depth: density depends on height only (8-point midpoint rule
    // over the slab part of the ray, enough for the smooth top profile).
    var od = 0.0;
    let n = 8;
    let dt = (t1 - t0) / f32(n);
    for (var i = 0; i < n; i++) {
        let h = h0 + rd.y * (t0 + (f32(i) + 0.5) * dt);
        od += density(max(h, 0.0)) * dt * 0.001;
    }
    let T = exp(-od);
    if (T > 0.9999) {
        return vec4<f32>(0.0, 0.0, 0.0, 1.0);
    }
    // In-scattered light: diffuse sky / cloud-deck light from all around plus
    // forward-scattered sun (dimmed by the fog above and by cloud cover).
    let hm = clamp(min(h0, top) * 0.5, 0.0, top);
    let r = RG + hm * 0.001;
    var tsun = vec3<f32>(0.0);
    if (!hits_ground(r, F.sun.y)) {
        tsun = textureSampleLevel(trans_tex, lin, trans_uv(r, F.sun.y), 0.0).rgb;
    }
    let mu = dot(rd, F.sun.xyz);
    let g = 0.6;
    let hg = (1.0 - g * g) / (4.0 * PI * pow(max(1.0 + g * g - 2.0 * g * mu, 1e-4), 1.5));
    let cover = F.fog.y;
    let e_sun = tsun * F.fog.z * (1.0 - 0.9 * cover) * (hg + 0.04);
    // Mostly light from above (the bright fog / cloud top), a little from the ground.
    let e_amb = (irr(vec3<f32>(0.0, 1.0, 0.0)) * 0.85 + vec3<f32>(luma(irr(vec3<f32>(0.0, -1.0, 0.0)))) * 0.15) / PI;
    let L = e_amb * 1.05 + e_sun;
    return vec4<f32>(L * (1.0 - T), T);
}
