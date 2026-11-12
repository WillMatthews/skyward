// Visual effects (src/vfx): instanced soft particles, ribbons / meshes and
// heat haze. atmos_core.wgsl + common.wgsl are prepended (group 0 = the scene
// globals: sun / moon, sky irradiance, cloud shadows, aerial perspective,
// pre-exposure). Drawn after the clouds and ground fog, premultiplied
// alpha (dst = src + dst * (1 - a)); emissive parts write a ≈ 0 (additive).
//
// Cost: lighting (sun with transmittance + cloud shadow, sky, fires) and the
// aerial perspective are evaluated per vertex (per particle); the fragment
// shader only shapes the puff (2-octave noise, a sphere normal for
// self-shadowing) and fades it softly against the scene.

struct VfxU {
    off: vec4<f32>,               // frame origin - eye (xyz), w = time (s)
    screen: vec4<f32>,            // w, h, 1/w, 1/h
    fires: array<vec4<f32>, 4>,   // fire lights: eye-relative position, intensity (cd)
    misc: vec4<f32>,              // x = haze strength scale
};

struct GP {
    pos: vec4<f32>,    // relative to the frame origin, w = radius
    col: vec4<f32>,    // albedo / emission, opacity
    vel: vec4<f32>,    // m/s, w = kind
    extra: vec4<f32>,  // age 0..1, rotation, seed, emissive scale
};

@group(1) @binding(0) var<uniform> V: VfxU;
@group(1) @binding(1) var<storage, read> PS: array<GP>;
@group(1) @binding(2) var dist_tex: texture_2d<f32>;
@group(1) @binding(3) var scene_tex: texture_2d<f32>;

const K_SMOKE: f32 = 0.0;
const K_FIRE: f32 = 1.0;
const K_DUST: f32 = 2.0;
const K_SPRAY: f32 = 3.0;
const K_SPARK: f32 = 4.0;
const K_STEAM: f32 = 5.0;
const K_TRAIL: f32 = 10.0;
const K_SMOKETRAIL: f32 = 11.0;
const K_VAPOUR: f32 = 12.0;
const K_DIAMONDS: f32 = 13.0;
const K_HAZE: f32 = 14.0;
// Emission units → scene radiance (≈ cd/m² × LUX; flames ~ 20-50 kcd/m²).
const E_SCALE: f32 = 0.06;

struct VO {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) rel: vec3<f32>,
    @location(2) @interpolate(flat) kind: f32,
    @location(3) col: vec4<f32>,
    @location(4) extra: vec4<f32>,
    @location(5) centre: vec3<f32>,
    @location(6) size: f32,
    // Lighting per vertex: direct (sun / moon × phase), ambient (sky + fires),
    // aerial perspective (inscatter rgb, transmittance a).
    @location(7) direct: vec3<f32>,
    @location(8) ambient: vec3<f32>,
    @location(9) fog: vec4<f32>,
};

fn corner(vi: u32) -> vec2<f32> {
    let c = vi % 6u;
    let x = select(-1.0, 1.0, c == 1u || c == 2u || c == 4u);
    let y = select(-1.0, 1.0, c == 2u || c == 4u || c == 5u);
    return vec2<f32>(x, y);
}

fn n2(p: vec2<f32>) -> f32 {
    return 0.62 * noise2(p) + 0.38 * noise2(p * 2.07 + vec2<f32>(5.2, 1.3));
}

fn hg(c: f32, gg: f32) -> f32 {
    let d = 1.0 + gg * gg - 2.0 * gg * c;
    return (1.0 - gg * gg) / (12.566371 * pow(max(d, 1e-4), 1.5));
}

// Phase asymmetry per kind: condensation scatters strongly forward.
fn kind_g(kind: f32) -> f32 {
    if (kind == K_STEAM || kind == K_TRAIL || kind == K_VAPOUR) {
        return 0.65;
    }
    if (kind == K_SPRAY) {
        return 0.5;
    }
    if (kind == K_DUST) {
        return 0.4;
    }
    return 0.25;
}

struct Light {
    direct: vec3<f32>,
    ambient: vec3<f32>,
    fog: vec4<f32>,
};

// Light arriving at an eye-relative point (scene radiance units, pre-exposed).
fn light_at(rel: vec3<f32>, gg: f32, clip: vec4<f32>) -> Light {
    var o: Light;
    let wp = rel + g.cam_pos.xyz;
    let L = g.sun_dir.xyz;
    let d = normalize(rel);
    let phase = mix(1.0, 12.566371 * hg(dot(L, d), gg), 0.65);
    o.direct = sun_irradiance(wp) * phase / PI;
    var amb = (sky_irradiance(vec3<f32>(0.0, 1.0, 0.0)) * 0.75 + sky_irradiance(vec3<f32>(0.0, -1.0, 0.0)) * 0.25) / PI;
    for (var i = 0; i < 4; i++) {
        let f = V.fires[i];
        if (f.w > 0.0) {
            let v = f.xyz - rel;
            let d2 = max(dot(v, v), 9.0);
            amb += vec3<f32>(1.0, 0.48, 0.16) * (f.w * LUX / d2) * g.pre.x * 0.3 / PI;
        }
    }
    o.ambient = amb;
    // Aerial perspective at this point's screen position.
    let ndc = clip.xy / max(clip.w, 1e-4);
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    let dist = length(rel);
    let s = ap_slice_of(dist * 0.001);
    let ap = textureSampleLevel(ap_tex, lin_samp, vec3<f32>(clamp(uv, vec2<f32>(0.0), vec2<f32>(1.0)), s), 0.0);
    let k = smoothstep(0.0, 1.0 / AP_N, s);
    o.fog = vec4<f32>(ap.rgb * g.sky.x * k, mix(1.0, ap.a, k));
    return o;
}

@vertex
fn vs_particle(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> VO {
    let p = PS[ii];
    var o: VO;
    let c = corner(vi);
    let centre = p.pos.xyz + V.off.xyz;
    let size = p.pos.w;
    let kind = p.vel.w;
    let d = normalize(centre);
    var right = cross(d, vec3<f32>(0.0, 1.0, 0.0));
    if (dot(right, right) < 1e-6) {
        right = vec3<f32>(1.0, 0.0, 0.0);
    }
    right = normalize(right);
    let up = cross(right, d);
    var pos: vec3<f32>;
    if (kind == K_SPARK || kind == K_SPRAY) {
        // Streak along the screen-plane velocity (motion blur).
        let v = p.vel.xyz;
        let vt = v - d * dot(v, d);
        let l = length(vt);
        var ax = right;
        if (l > 1e-3) {
            ax = vt / l;
        }
        let ortho = normalize(cross(d, ax));
        let exposure = select(0.012, 0.035, kind == K_SPARK);
        let half = size + l * exposure;
        pos = centre + ax * c.x * half + ortho * c.y * size;
    } else {
        let a = p.extra.y;
        let cs = cos(a);
        let sn = sin(a);
        let rc = vec2<f32>(c.x * cs - c.y * sn, c.x * sn + c.y * cs);
        pos = centre + (right * rc.x + up * rc.y) * size;
    }
    o.clip = g.view_proj * vec4<f32>(pos, 1.0);
    o.uv = c;
    o.rel = pos;
    o.kind = kind;
    o.col = p.col;
    o.extra = p.extra;
    o.centre = centre;
    o.size = size;
    let lt = light_at(centre, kind_g(kind), g.view_proj * vec4<f32>(centre, 1.0));
    o.direct = lt.direct;
    o.ambient = lt.ambient;
    o.fog = lt.fog;
    return o;
}

fn soft(px: vec2<f32>, d: f32, size: f32) -> f32 {
    let scene = textureLoad(dist_tex, vec2<i32>(px), 0).x;
    let s = clamp((scene - d) / max(size * 0.6, 0.05), 0.0, 1.0);
    // Fade out quads that engulf the camera.
    let near = clamp((d - 0.3) / max(size * 0.8, 0.3), 0.0, 1.0);
    return s * near;
}

// Scattered light for a puff with normal n (self-shadowing on the far side from the light).
fn shade(in: VO, albedo: vec3<f32>, n: vec3<f32>, self_shadow: f32) -> vec3<f32> {
    let wrap = 0.5 + 0.5 * dot(n, g.sun_dir.xyz);
    let s = mix(1.0, wrap, self_shadow);
    return albedo * (in.direct * s + in.ambient);
}

@fragment
fn fs_particle(in: VO) -> @location(0) vec4<f32> {
    let r2 = dot(in.uv, in.uv);
    if (r2 >= 1.0) {
        discard;
    }
    let px = in.clip.xy;
    let d = length(in.rel);
    let sf = soft(px, d, in.size);
    if (sf <= 0.001) {
        discard;
    }
    let kind = in.kind;
    let age = in.extra.x;
    let seed = in.extra.z;
    let t = V.off.w;
    if (kind == K_FIRE || kind == K_SPARK) {
        var core: f32;
        var e: vec3<f32>;
        if (kind == K_SPARK) {
            core = pow(1.0 - r2, 1.5);
            e = in.col.rgb;
        } else {
            let n = n2(in.uv * 1.6 + vec2<f32>(seed, seed * 0.7) + vec2<f32>(0.0, -t * 1.7));
            core = clamp((1.0 - r2) * (0.35 + 1.1 * n) - 0.12, 0.0, 1.0);
            core = core * core;
            // Hot yellow when young, orange-red, then sooty as it cools.
            let cool = smoothstep(0.15, 0.9, age);
            e = in.col.rgb * mix(vec3<f32>(1.25, 1.1, 0.85), vec3<f32>(0.75, 0.35, 0.15), cool);
        }
        let a = core * in.col.a * sf;
        let rad = e * E_SCALE * a * g.pre.x * in.fog.a;
        let occl = select(0.0, a * 0.25, kind == K_FIRE);
        return vec4<f32>(rad + in.fog.rgb * occl, occl);
    }
    // Scattering kinds.
    var dens: f32;
    var ss = 0.75;
    if (kind == K_SPRAY) {
        dens = pow(1.0 - r2, 1.2);
        ss = 0.2;
    } else {
        let n = n2(in.uv * 1.3 + vec2<f32>(seed * 1.3, seed) + vec2<f32>(t * 0.05, -t * 0.08));
        dens = clamp((1.0 - r2) * (0.45 + 1.1 * n) - 0.1, 0.0, 1.0);
        if (kind == K_STEAM) {
            ss = 0.45;
        } else if (kind == K_DUST) {
            ss = 0.6;
        }
    }
    let a = clamp(dens * in.col.a, 0.0, 1.0) * sf;
    if (a < 0.002) {
        discard;
    }
    let off = (in.rel - in.centre) / max(in.size, 1e-3);
    let dv = normalize(in.rel);
    let nrm = normalize(off - dv * sqrt(max(1.0 - r2, 0.0)));
    let c = shade(in, in.col.rgb, nrm, ss) * in.fog.a + in.fog.rgb;
    return vec4<f32>(c * a, a);
}

// ---------------------------------------------------------------- ribbons / meshes

struct MIn {
    @location(0) pos: vec4<f32>,
    @location(1) uv: vec4<f32>,
};

@vertex
fn vs_mesh(v: MIn) -> VO {
    var o: VO;
    let rel = v.pos.xyz + V.off.xyz;
    o.clip = g.view_proj * vec4<f32>(rel, 1.0);
    o.uv = v.uv.xy;
    o.rel = rel;
    o.kind = v.pos.w;
    o.col = vec4<f32>(v.uv.z, v.uv.w, 0.0, 0.0);
    o.extra = vec4<f32>(0.0);
    o.centre = rel;
    o.size = max(v.uv.w, 0.5);
    if (v.pos.w == K_HAZE) {
        o.direct = vec3<f32>(0.0);
        o.ambient = vec3<f32>(0.0);
        o.fog = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    } else {
        let lt = light_at(rel, kind_g(v.pos.w), o.clip);
        o.direct = lt.direct;
        o.ambient = lt.ambient;
        o.fog = lt.fog;
    }
    return o;
}

@fragment
fn fs_mesh(in: VO) -> @location(0) vec4<f32> {
    let px = in.clip.xy;
    let d = length(in.rel);
    let kind = in.kind;
    let inten = in.col.x;
    let t = V.off.w;
    let v = in.uv.y;
    if (kind == K_DIAMONDS) {
        // Shock cells in the afterburner core: bright diamonds decaying downstream.
        let x = in.uv.x;
        let cell = 0.85;
        let ph = fract(x / cell + 0.35);
        // Diamond: widest at the cell's shock, pinched between cells.
        let w = 1.0 - smoothstep(0.0, 1.0, abs(v) / (0.15 + 0.5 * (1.0 - abs(ph - 0.5) * 2.0)));
        let dia = exp(-pow((ph - 0.5) / 0.16, 2.0)) * w;
        let decay = exp(-x / (1.6 + 1.6 * inten));
        let core = (1.0 - smoothstep(0.0, 0.7, abs(v))) * exp(-x / 3.0);
        let e = vec3<f32>(0.8, 0.86, 1.0) * dia * 22.0 * decay + vec3<f32>(1.0, 0.55, 0.45) * core * 2.5;
        let rad = e * E_SCALE * inten * g.pre.x * soft(px, d, 0.4) * in.fog.a;
        return vec4<f32>(rad, 0.0);
    }
    var a = 0.0;
    var albedo = vec3<f32>(0.9);
    var ss = 0.3;
    if (kind == K_TRAIL || kind == K_SMOKETRAIL) {
        let tube = sqrt(max(1.0 - v * v, 0.0));
        let n = n2(vec2<f32>(in.uv.x * 0.6, v * 1.5) + vec2<f32>(t * 0.02, 0.0));
        a = inten * tube * (0.6 + 0.6 * n);
        if (kind == K_SMOKETRAIL) {
            albedo = vec3<f32>(0.12);
            ss = 0.7;
        }
    } else if (kind == K_VAPOUR) {
        // Streaky, flickering condensation (sheets and the transonic cone).
        let n = n2(vec2<f32>(in.uv.x * 2.0, in.uv.y * 0.7) + vec2<f32>(t * 3.0, t * 0.7));
        let streak = n2(vec2<f32>(in.uv.x * 9.0, t * 4.0));
        let edge = 1.0 - smoothstep(0.55, 1.0, abs(v));
        a = inten * edge * clamp(n * 1.6 - 0.35, 0.0, 1.0) * (0.6 + 0.6 * streak);
        ss = 0.15;
    }
    a = clamp(a, 0.0, 1.0) * soft(px, d, 1.0);
    if (a < 0.002) {
        discard;
    }
    let nrm = normalize(-normalize(in.rel) + vec3<f32>(0.0, 0.3, 0.0));
    let c = shade(in, albedo, nrm, ss) * in.fog.a + in.fog.rgb;
    return vec4<f32>(c * a, a);
}

// ---------------------------------------------------------------- heat haze

@fragment
fn fs_haze(in: VO) -> @location(0) vec4<f32> {
    let px = in.clip.xy;
    let d = length(in.rel);
    let v = in.uv.y;
    let x = in.uv.x;
    let len = max(in.col.y, 1.0);
    let t = V.off.w;
    let along = x / len;
    // Strongest just behind the nozzle, fading downstream and toward the edges.
    let mask = (1.0 - smoothstep(0.2, 1.0, abs(v))) * smoothstep(0.0, 0.05, along) * (1.0 - smoothstep(0.35, 1.0, along));
    let scene = textureLoad(dist_tex, vec2<i32>(px), 0).x;
    if (scene < d - 0.5 || mask <= 0.001) {
        discard;
    }
    let nx = n2(vec2<f32>(x * 0.9 - t * 9.0, v * 2.0)) - 0.5;
    let ny = n2(vec2<f32>(x * 0.9 - t * 9.0 + 7.3, v * 2.0 + 3.1)) - 0.5;
    // Refraction offset in pixels: the plume subtends less at a distance.
    let k = in.col.x * V.misc.x * mask * V.screen.y * 0.0035 * min(8.0 / max(d, 1.0), 1.5);
    let q = clamp(px + vec2<f32>(nx, ny) * k, vec2<f32>(0.0), V.screen.xy - 1.0);
    let c = textureLoad(scene_tex, vec2<i32>(q), 0).rgb;
    let a = min(mask * 1.5, 1.0);
    return vec4<f32>(c * a, a);
}
