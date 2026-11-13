// Visual effects (src/vfx): instanced soft particles, ribbons / meshes and
// heat haze. atmos_core.wgsl + common.wgsl are prepended (group 0 = the scene
// globals: key light (sun / moon), sky irradiance, cloud shadows, aerial
// perspective, pre-exposure). Drawn after the clouds and ground fog,
// premultiplied alpha (dst = src + dst * (1 - a)); emission adds with a ≈ 0.
//
// * Flames are emissive by temperature (blackbody luminance and colour,
//   1000-1900 K) on a domain-warped flow-noise field advected along the
//   flame; cool regions turn to soot (absorbing, lit like smoke).
// * Smoke / dust / steam: lit by the key light through the other smoke (the
//   CPU's per-particle transmittance, vfx/gpu.rs), by the sky (with the
//   column above occluding it) and by the fires (inverse square, through
//   the smoke), with a per-pixel sphere normal for self-shadowing.
// * Everything is soft against the scene and hidden by clouds in front of it
//   (the clouds' half-resolution transmittance + mean depth).
// * Ribbons (contrails, vortex cores) keep a minimum on-screen width and
//   trade some opacity for it (no aliasing at a distance, still bold).
// * Condensation volumes (transonic vapour, wing-surface fog) are ray-marched
//   in the aircraft's frame inside boxes (vs_vol / fs_vol): density fields
//   with crisp boundaries where the physics has them (the shock, the edge of
//   the expansion), streaky and flickering inside.

struct VfxU {
    off: vec4<f32>,               // frame origin - eye (xyz), w = time (s)
    screen: vec4<f32>,            // w, h, 1/w, 1/h
    fires: array<vec4<f32>, 4>,   // fire lights: eye-relative position, intensity (cd)
    misc: vec4<f32>,              // x = haze strength scale, y = cloud result bound, z = vapour step factor (quality)
    key: vec4<f32>,               // toward the key light
    vol: array<vec4<f32>, 20>,    // condensation volumes (vfx/mod.rs `Vfx::volume`)
};

struct GP {
    pos: vec4<f32>,    // relative to the frame origin, w = radius
    col: vec4<f32>,    // albedo, opacity
    vel: vec4<f32>,    // m/s (fire: flame axis), w = kind
    extra: vec4<f32>,  // age 0..1, rotation, seed, stretch
    light: vec4<f32>,  // key-light transmittance, fire irradiance (lux), temperature (K), sky visibility
};

@group(1) @binding(0) var<uniform> V: VfxU;
@group(1) @binding(1) var<storage, read> PS: array<GP>;
@group(1) @binding(2) var dist_tex: texture_2d<f32>;
@group(1) @binding(3) var scene_tex: texture_2d<f32>;
@group(1) @binding(4) var cloud_col: texture_2d<f32>;
@group(1) @binding(5) var cloud_dep: texture_2d<f32>;

const K_SMOKE: f32 = 0.0;
const K_FIRE: f32 = 1.0;
const K_DUST: f32 = 2.0;
const K_SPRAY: f32 = 3.0;
const K_SPARK: f32 = 4.0;
const K_STEAM: f32 = 5.0;
const K_TRAIL: f32 = 10.0;
const K_SMOKETRAIL: f32 = 11.0;
const K_DIAMONDS: f32 = 13.0;
const K_HAZE: f32 = 14.0;
const K_VORTEX: f32 = 15.0;
// Emission units of sparks / diamonds → scene radiance (≈ cd/m² × LUX).
const E_SCALE: f32 = 0.06;
// Fire light colour (≈ 1600 K blackbody), luminance 1 (== sim.rs fire lights).
const FIRE_RGB: vec3<f32> = vec3<f32>(2.4, 0.66, 0.1);

struct VO {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) rel: vec3<f32>,
    @location(2) @interpolate(flat) kind: f32,
    @location(3) col: vec4<f32>,
    @location(4) extra: vec4<f32>,
    @location(5) centre: vec3<f32>,
    @location(6) size: f32,
    // Lighting per vertex: direct (key light × phase), ambient (sky),
    // aerial perspective (inscatter rgb, transmittance a).
    @location(7) direct: vec3<f32>,
    @location(8) ambient: vec3<f32>,
    @location(9) fog: vec4<f32>,
    @location(10) light: vec4<f32>,
    @location(11) aux: vec4<f32>,
    // Visibility through the clouds in front.
    @location(12) cloud: f32,
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
    if (kind == K_STEAM) {
        return 0.7;
    }
    if (kind == K_TRAIL || kind == K_VORTEX) {
        // Dense young ice / droplet lines: multiple scattering evens the
        // phase function out (bright white from every side).
        return 0.45;
    }
    if (kind == K_SPRAY) {
        return 0.5;
    }
    if (kind == K_DUST) {
        return 0.45;
    }
    return 0.3;
}

// ---------------------------------------------------------------- blackbody

// Linear-RGB chromaticity of a blackbody at `tk` kelvin, luminance 1.
fn bb_col(tk: f32) -> vec3<f32> {
    let t = clamp(tk, 800.0, 6500.0) / 100.0;
    let gr = clamp((99.4708 * log(t) - 161.1196) / 255.0, 0.0, 1.0);
    let bl = select(clamp((138.5177 * log(max(t - 10.0, 1e-3)) - 305.0448) / 255.0, 0.0, 1.0), 0.0, t <= 19.0);
    let c = pow(vec3<f32>(1.0, gr, bl), vec3<f32>(2.2));
    return c / max(dot(c, vec3<f32>(0.2126, 0.7152, 0.0722)), 1e-4);
}

// Luminance (cd/m²) of a black body at `tk` (Wien at 555 nm, 2800 K ≈ 2.5e7).
fn bb_lum(tk: f32) -> f32 {
    return 2.5e7 * exp(9.25 - 25900.0 / max(tk, 500.0));
}

// A camera's highlight limit for large emitters: roll the exposed value off
// above ~7 stops over mid grey, so a fire at night saturates instead of
// flooding the glare (bloom) with a screen-sized halo.
fn highlight(rad: vec3<f32>) -> vec3<f32> {
    let v = dot(rad, vec3<f32>(0.2126, 0.7152, 0.0722)) * atmos.expo.x;
    let k = 2.2;
    if (v <= k) {
        return rad;
    }
    // Compressed (not clipped): hotter parts stay brighter, but a large
    // emitter never reaches the glare's threshold by much (no halo dome).
    return rad * (min(k * pow(v / k, 0.25), 4.5) / v);
}

// Soot fraction of a flame puff at temperature `tk` (== vfx/gpu.rs `soot`).
fn soot(tk: f32) -> f32 {
    return smoothstep(0.0, 1.0, clamp((1350.0 - tk) / 400.0, 0.0, 1.0));
}

// ---------------------------------------------------------------- lighting

struct Light {
    direct: vec3<f32>,
    ambient: vec3<f32>,
    fog: vec4<f32>,
};

// Light arriving at an eye-relative point (scene radiance units, pre-exposed).
fn light_at(rel: vec3<f32>, gg: f32, clip: vec4<f32>, fires: bool) -> Light {
    var o: Light;
    let wp = rel + g.cam_pos.xyz;
    let L = g.sun_dir.xyz;
    let d = normalize(rel);
    let phase = mix(1.0, 12.566371 * hg(dot(L, d), gg), 0.7);
    o.direct = sun_irradiance(wp) * phase / PI;
    var amb = (sky_irradiance(vec3<f32>(0.0, 1.0, 0.0)) * 0.75 + sky_irradiance(vec3<f32>(0.0, -1.0, 0.0)) * 0.25) / PI;
    if (fires) {
        for (var i = 0; i < 4; i++) {
            let f = V.fires[i];
            if (f.w > 0.0) {
                let v = f.xyz - rel;
                let d2 = max(dot(v, v), 25.0);
                amb += FIRE_RGB * (f.w * LUX / d2) * g.pre.x / PI;
            }
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

// Visibility of a point at distance `d` through the clouds in front of it
// (the clouds' half-resolution transmittance and mean depth).
fn cloud_vis(clip: vec4<f32>, d: f32) -> f32 {
    if (V.misc.y < 0.5 || clip.w <= 1e-3) {
        return 1.0;
    }
    let ndc = clip.xy / clip.w;
    let uv = clamp(vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5), vec2<f32>(0.0), vec2<f32>(0.9999));
    let dims = vec2<f32>(textureDimensions(cloud_col));
    let px = vec2<i32>(uv * dims);
    let tr = clamp(textureLoad(cloud_col, px, 0).a, 0.0, 1.0);
    if (tr > 0.995) {
        return 1.0;
    }
    let cd = textureLoad(cloud_dep, px, 0).y;
    // In front of the cloud: untouched; behind it: its transmittance.
    return mix(1.0, tr, smoothstep(cd * 0.92, cd * 1.04, d));
}

// ---------------------------------------------------------------- particles

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
        let exposure = select(0.012, 0.03, kind == K_SPARK);
        let half = size + l * exposure;
        pos = centre + ax * c.x * half + ortho * c.y * size;
    } else if (kind == K_FIRE && p.extra.w > 1.05) {
        // Flames: a quad stretched along the flame axis (as seen on screen),
        // its base at the particle (tongues lick downstream).
        let v = p.vel.xyz;
        let vt = v - d * dot(v, d);
        let l = length(vt);
        var ax = up;
        if (l > 1e-3) {
            ax = vt / l;
        }
        // Seen along its axis a flame is round, not long.
        let st = mix(1.0, p.extra.w, clamp(l / max(length(v), 1e-3), 0.0, 1.0));
        let ortho = normalize(cross(d, ax));
        pos = centre + ax * (c.y * st + st - 1.0) * size * 0.85 + ortho * c.x * size;
        o.aux = vec4<f32>(st, 0.0, 0.0, 0.0);
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
    o.light = p.light;
    let cc = g.view_proj * vec4<f32>(centre, 1.0);
    let lt = light_at(centre, kind_g(kind), cc, false);
    o.direct = lt.direct;
    o.ambient = lt.ambient;
    o.fog = lt.fog;
    o.cloud = cloud_vis(o.clip, length(pos));
    return o;
}

fn soft(px: vec2<f32>, d: f32, size: f32) -> f32 {
    let scene = textureLoad(dist_tex, vec2<i32>(px), 0).x;
    let s = clamp((scene - d) / max(size * 0.6, 0.05), 0.0, 1.0);
    // Fade out quads that engulf the camera.
    let near = clamp((d - 0.3) / max(size * 0.8, 0.3), 0.0, 1.0);
    return s * near;
}

// Light scattered by a puff (albedo `alb`, unit normal `n`): the key light
// through the smoke toward it with self-shadowing, the sky (occluded by the
// column above) and the fires (from below / their side).
fn shade(in: VO, alb: vec3<f32>, n: vec3<f32>, self_shadow: f32) -> vec3<f32> {
    let wrap = 0.5 + 0.5 * dot(n, g.sun_dir.xyz);
    let s = mix(1.0, wrap * (0.6 + 0.8 * wrap), self_shadow);
    let key = in.direct * in.light.x * s;
    let sky = in.ambient * mix(0.3, 1.0, in.light.w) * (0.8 + 0.2 * n.y);
    let fire = FIRE_RGB * (in.light.y * LUX * g.pre.x / PI) * (0.65 - 0.45 * n.y);
    // Multiple scattering in bright media (steam, condensation) fills the shadows.
    let ms = alb * in.direct * (1.0 - in.light.x * s) * 0.35 * dot(alb, vec3<f32>(0.33));
    return alb * (key + sky + fire) + ms;
}

// Billowing density of a smoke puff: 3 octaves, slowly turning over.
fn billow(uv: vec2<f32>, seed: f32, age: f32, t: f32) -> f32 {
    let q = uv * 1.25 + vec2<f32>(seed * 1.3, seed);
    let w = vec2<f32>(noise2(q * 0.9 + vec2<f32>(t * 0.04, 0.0)), noise2(q * 0.9 + vec2<f32>(3.7, -t * 0.05))) - 0.5;
    let n = 0.55 * noise2(q + w * 1.2 + vec2<f32>(0.0, -age * 0.6)) + 0.3 * noise2(q * 2.3 + w * 0.8 + vec2<f32>(age * 0.9, 0.0)) + 0.15 * noise2(q * 5.1 - vec2<f32>(t * 0.1, age));
    return n;
}

@fragment
fn fs_particle(in: VO) -> @location(0) vec4<f32> {
    let kind = in.kind;
    var r2 = dot(in.uv, in.uv);
    let flame = kind == K_FIRE && in.aux.x > 0.5;
    if (flame) {
        // (Flame quads: x across, y along the axis, 0 at the base.)
        r2 = 0.0;
    }
    if (r2 >= 1.0) {
        discard;
    }
    let px = in.clip.xy;
    let d = length(in.rel);
    let sf = soft(px, d, in.size) * in.cloud;
    if (sf <= 0.001) {
        discard;
    }
    let age = in.extra.x;
    let seed = in.extra.z;
    let t = V.off.w;
    if (kind == K_SPARK) {
        let core = pow(1.0 - r2, 1.5);
        let a = core * in.col.a * sf;
        return vec4<f32>(in.col.rgb * E_SCALE * a * g.pre.x * in.fog.a, 0.0);
    }
    if (kind == K_FIRE && !flame) {
        // Fireball puff: a rolling, billowing ball, hottest in its dense
        // core; it cools from the outside in, into black soot.
        let n = billow(in.uv, seed, age * 3.0, t * 4.0);
        let edge = 1.0 - r2;
        let den = smoothstep(0.05, 0.7, edge * (0.25 + 1.3 * n) - 0.1 * (1.0 - edge));
        if (den <= 0.002) {
            discard;
        }
        // Patchy: soot-darkened pockets between hot cores.
        let hotspot = noise2(in.uv * 2.3 + vec2<f32>(seed * 0.7, age * 2.0 - t * 0.5));
        let tk = in.light.z * (0.68 + 0.2 * smoothstep(0.1, 0.8, den) + 0.14 * hotspot);
        let emis = (1.0 - exp(-den * 4.0)) * in.col.a;
        let em = highlight(bb_col(tk) * bb_lum(tk) * emis * LUX * g.pre.x) * in.fog.a * sf;
        let so = soot(tk);
        let a_soot = clamp(den * 1.4, 0.0, 1.0) * so * in.col.a;
        let off = (in.rel - in.centre) / max(in.size, 1e-3);
        let nrm = normalize(off - normalize(in.rel) * sqrt(max(edge, 0.0)));
        let sc = highlight(shade(in, in.col.rgb, nrm, 0.7)) * in.fog.a + in.fog.rgb;
        let a = clamp(max(a_soot, emis * 0.5 * (1.0 - so)), 0.0, 1.0) * sf;
        return vec4<f32>(em + sc * a_soot * sf, a);
    }
    if (flame) {
        // Flame: y 0 (base) .. 1 (tip); flow noise advected along the flame.
        let st = in.aux.x;
        let y = in.uv.y * 0.5 + 0.5;
        let x = in.uv.x;
        let q = vec2<f32>(x * 1.1, y * st * 1.3) + vec2<f32>(seed, seed * 0.37);
        let tt = t * 1.9 + seed;
        let w = vec2<f32>(noise2(q * 1.3 + vec2<f32>(0.0, -tt * 0.7)), noise2(q * 1.3 + vec2<f32>(5.3, -tt * 0.8))) - 0.5;
        let n = 0.55 * noise2(q * 1.8 + w * 1.6 + vec2<f32>(0.0, -tt)) + 0.3 * noise2(q * 3.7 + w * 1.1 + vec2<f32>(seed, -tt * 1.7)) + 0.15 * noise2(q * 7.9 + vec2<f32>(-seed, -tt * 2.9));
        // Teardrop envelope: wide at the base, narrowing toward the tip; the
        // noise eats into it from the top (licking tongues).
        let hw = mix(0.95, 0.3, y) + (n - 0.5) * 0.5;
        let env = clamp(1.0 - pow(abs(x) / max(hw, 0.05), 2.0), 0.0, 1.0) * smoothstep(0.0, 0.12, y);
        // (Fades out before the quad's edges: no straight cut-off tips.)
        let rim = (1.0 - smoothstep(0.7, 0.98, y)) * (1.0 - smoothstep(0.75, 0.98, abs(x)));
        let den = clamp((env * (0.25 + 1.5 * n) - y * 0.55 - 0.08) * rim, 0.0, 1.0);
        if (den <= 0.002) {
            discard;
        }
        // Temperature: the particle's core temperature in the dense parts,
        // cooler at its fringes.
        let tk = in.light.z * (0.8 + 0.2 * smoothstep(0.0, 0.5, den));
        let emis = (1.0 - exp(-den * 5.0)) * in.col.a;
        let lum = bb_lum(tk) * emis;
        let em = highlight(bb_col(tk) * lum * LUX * g.pre.x) * in.fog.a * sf;
        // Cool parts: soot, lit like smoke.
        let so = soot(tk);
        let a_soot = clamp(den * 1.6, 0.0, 1.0) * so * in.col.a;
        let off = vec3<f32>(0.0, in.uv.y, 0.0);
        let nrm = normalize(vec3<f32>(x * 0.7, 0.5 + in.uv.y * 0.5, 0.0) - normalize(in.rel) * 0.6 + off * 0.0);
        let sc = highlight(shade(in, in.col.rgb, nrm, 0.6)) * in.fog.a + in.fog.rgb;
        // Luminous flames hide a little of what is behind them.
        let a = clamp(max(a_soot, emis * 0.35 * (1.0 - so)), 0.0, 1.0) * sf;
        return vec4<f32>(em + sc * a_soot * sf, a);
    }
    // Scattering kinds.
    var dens: f32;
    var ss = 0.7;
    if (kind == K_SPRAY) {
        dens = pow(1.0 - r2, 1.2);
        ss = 0.2;
    } else {
        let n = billow(in.uv, seed, age, t);
        let edge = 1.0 - r2;
        dens = smoothstep(0.02, 0.75, edge * edge * (0.2 + 1.5 * n));
        if (kind == K_STEAM) {
            ss = 0.45;
        } else if (kind == K_DUST) {
            ss = 0.55;
        }
    }
    let a = clamp(dens * in.col.a, 0.0, 1.0) * sf;
    if (a < 0.002) {
        discard;
    }
    let off = (in.rel - in.centre) / max(in.size, 1e-3);
    let dv = normalize(in.rel);
    let nrm = normalize(off - dv * sqrt(max(1.0 - r2, 0.0)));
    // (Fire-lit smoke at night is compressed like the flames.)
    let c = highlight(shade(in, in.col.rgb, nrm, ss)) * in.fog.a + in.fog.rgb;
    return vec4<f32>(c * a, a);
}

// ---------------------------------------------------------------- ribbons / meshes

struct MIn {
    @location(0) pos: vec4<f32>,
    @location(1) uv: vec4<f32>,
    @location(2) aux: vec4<f32>,
};

@vertex
fn vs_mesh(v: MIn) -> VO {
    var o: VO;
    var rel = v.pos.xyz + V.off.xyz;
    var inten = v.uv.z;
    let kind = v.pos.w;
    if (kind == K_TRAIL || kind == K_SMOKETRAIL || kind == K_VORTEX) {
        // Ribbon: centre line + side vector; at least ~1.6 px wide (no
        // aliasing), fainter for it — but less than in proportion: a thin
        // bright line against the sky stays bold to the eye.
        let hw = 0.5 * v.uv.w;
        let px = length(rel) / max(g.sky.w, 1.0);
        let hwd = max(hw, 0.8 * px);
        rel += v.aux.xyz * v.uv.y * hwd;
        inten *= pow(hw / hwd, 0.55);
    }
    o.clip = g.view_proj * vec4<f32>(rel, 1.0);
    o.uv = v.uv.xy;
    o.rel = rel;
    o.kind = kind;
    o.col = vec4<f32>(inten, v.uv.w, 0.0, 0.0);
    o.extra = vec4<f32>(0.0);
    o.centre = rel;
    o.size = max(v.uv.w, 0.5);
    o.aux = v.aux;
    o.light = vec4<f32>(1.0, 0.0, 0.0, 1.0);
    if (kind == K_HAZE) {
        o.direct = vec3<f32>(0.0);
        o.ambient = vec3<f32>(0.0);
        o.fog = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        o.cloud = 1.0;
    } else {
        let lt = light_at(rel, kind_g(kind), o.clip, true);
        o.direct = lt.direct;
        o.ambient = lt.ambient;
        o.fog = lt.fog;
        o.cloud = cloud_vis(o.clip, length(rel));
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
        // Afterburner plume with shock diamonds. x = metres from the nozzle,
        // v = across the plume (-1..1 of its local half-width), aux.w = cell
        // length, aux.z = nozzle radius.
        let x = in.uv.x;
        let ls = max(in.aux.w, 0.2);
        let rn = max(in.aux.z, 0.1);
        let k = x / ls - 0.55;
        let cellf = fract(k + 0.5) - 0.5;
        let cell_i = floor(k + 0.5);
        // Lateral extent in metres of this pixel, relative to the jet radius.
        let lat = abs(v) * in.col.y / rn;
        // Diamond: |dx| / (half cell) + lat / width <= 1; crisp edges.
        let wdi = 0.62 * exp(-max(cell_i, 0.0) * 0.07);
        let rh = abs(cellf) / 0.5 + lat / wdi;
        let dia = smoothstep(1.0, 0.6, rh) * (0.4 + 0.6 * smoothstep(0.75, 0.0, rh));
        // Mach disk: a thin bright bar across the jet at the cell's shock.
        let disk = exp(-pow(cellf / 0.05, 2.0)) * smoothstep(0.6, 0.35, lat);
        let decay = exp(-max(cell_i, 0.0) / (0.8 + 1.2 * inten)) * step(-0.5, k);
        // Hot core / plume envelope (the model's flame carries the rest).
        let env = exp(-lat * lat * 3.0) * exp(-x / (2.5 + 3.0 * inten)) * smoothstep(0.0, 0.3, x);
        let hot = mix(vec3<f32>(0.72, 0.82, 1.0), vec3<f32>(1.0, 0.72, 0.42), smoothstep(0.0, 4.0, cell_i));
        let e = hot * (dia * 14.0 + disk * 26.0) * decay + vec3<f32>(1.0, 0.42, 0.28) * env * 1.6;
        let fl = 0.92 + 0.08 * noise2(vec2<f32>(x * 3.0 - t * 40.0, v * 2.0));
        // (Tested from in front of the jet's axis: the flame mesh around it doesn't hide it.)
        let rad = e * fl * E_SCALE * inten * g.pre.x * soft(px, d - 1.2 * rn, 0.3) * in.fog.a * in.cloud;
        return vec4<f32>(rad, 0.0);
    }
    var a = 0.0;
    var albedo = vec3<f32>(0.92);
    var ss = 0.3;
    var nrm = normalize(-normalize(in.rel) + vec3<f32>(0.0, 0.3, 0.0));
    if (kind == K_TRAIL || kind == K_SMOKETRAIL || kind == K_VORTEX) {
        // Gaussian cross-section; lumpy along its length as it ages.
        let age = in.aux.w;
        let sub = in.uv.x;
        let side = normalize(in.aux.xyz);
        let view = -normalize(in.rel);
        if (kind == K_VORTEX) {
            // Vortex core: a thin tube of condensation whose striations wind
            // around it (helical bands slanted across the tube), flickering.
            let ang = asin(clamp(v, -0.999, 0.999));
            let helix = 0.5 + 0.5 * sin(sub * 40.0 - ang * 2.0 - t * 9.0);
            let wisp = n2(vec2<f32>(sub * 6.0 - t * 2.0, v * 1.5));
            let prof = exp(-v * v * 3.0);
            a = inten * prof * (0.45 + 0.4 * helix + 0.35 * wisp) * (1.0 - smoothstep(0.6, 1.0, abs(v)));
            nrm = normalize(side * v + view * sqrt(max(1.0 - v * v, 0.0)));
            ss = 0.2;
            albedo = vec3<f32>(0.95);
        } else {
            let prof = exp(-v * v * 2.2);
            let lump = n2(vec2<f32>(sub * 0.35, v * 1.2 + age * 0.02));
            let fine = noise2(vec2<f32>(sub * 2.2, v * 3.0));
            a = inten * prof * mix(1.0, 0.45 + 0.9 * lump * (0.8 + 0.4 * fine), smoothstep(2.0, 30.0, age));
            // Lit as a tube: normal from the cross-section.
            nrm = normalize(side * v + view * sqrt(max(1.0 - v * v, 0.0)));
            if (kind == K_SMOKETRAIL) {
                albedo = vec3<f32>(0.12);
                ss = 0.7;
            } else {
                // Young contrails are optically thick: bright, white, a little
                // denser in the core than a gaussian.
                a = min(a * 1.25, 0.97);
                albedo = vec3<f32>(0.97);
                ss = 0.15;
            }
        }
    }
    a = clamp(a, 0.0, 1.0) * soft(px, d, 0.6) * in.cloud;
    if (a < 0.002) {
        discard;
    }
    let c = shade(in, albedo, nrm, ss) * in.fog.a + in.fog.rgb;
    return vec4<f32>(c * a, a);
}

// ---------------------------------------------------------------- condensation volumes

// V.vol (vfx/mod.rs `Vfx::volume`), body frame (+z nose, +y up, +x left):
//  0 aircraft position (frame-origin relative), 1 attitude quaternion,
//  2 wing bell (z front, z shock, r front, r shock), 3 (y centre, AoA k, amount, Mach),
//  4 canopy bell, 5 (y centre, amount, AoA k, -),
//  6 wing fog (amount, fuselage r, semi-span, depth), 7 (tip deflection L, R),
//  8..11 wing stations (x, leading edge z, trailing edge z, y),
//  12..17 boxes (lo, hi): bells, left wing, right wing.

fn qrot(q: vec4<f32>, v: vec3<f32>) -> vec3<f32> {
    return v + 2.0 * cross(q.xyz, cross(q.xyz, v) + q.w * v);
}

fn h3(p: vec3<f32>) -> f32 {
    var q = fract(p * 0.1031);
    q += dot(q, q.yzx + 33.33);
    return fract((q.x + q.y) * q.z);
}

// Smooth 3D value noise 0..1.
fn n3(p: vec3<f32>) -> f32 {
    let i = floor(p);
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = mix(mix(h3(i), h3(i + vec3<f32>(1.0, 0.0, 0.0)), u.x), mix(h3(i + vec3<f32>(0.0, 1.0, 0.0)), h3(i + vec3<f32>(1.0, 1.0, 0.0)), u.x), u.y);
    let b = mix(mix(h3(i + vec3<f32>(0.0, 0.0, 1.0)), h3(i + vec3<f32>(1.0, 0.0, 1.0)), u.x), mix(h3(i + vec3<f32>(0.0, 1.0, 1.0)), h3(i + vec3<f32>(1.0, 1.0, 1.0)), u.x), u.y);
    return mix(a, b, u.z);
}

// Transonic condensation bell: density at body point `p` and its outward
// direction (for the lighting). `b` = (z front, z shock, r front, r shock),
// `yc` centre height, `ak` angle-of-attack factor.
fn bell(p: vec3<f32>, b: vec4<f32>, yc: f32, ak: f32, t: f32, seed: f32) -> vec4<f32> {
    let len = max(b.x - b.y, 0.3);
    let s = (b.x - p.z) / len;
    if (s < 0.0 || s > 1.3) {
        return vec4<f32>(0.0);
    }
    let sc = min(s, 1.0);
    // A cone flaring from the front (the leading-edge / canopy expansion)
    // to the shock, slightly bell-shaped.
    let rad = b.z + (b.w - b.z) * pow(sc, 0.75);
    let ycc = yc + 0.12 * ak * b.w * sc;
    let q = vec2<f32>(p.x, (p.y - ycc) / 0.9);
    let r = length(q);
    if (r > rad * (1.04 + 0.18 * abs(ak))) {
        return vec4<f32>(0.0);
    }
    let cs = q / max(r, 1e-3);
    // Larger over the suction side; its rim ripples a little.
    let rim = rad * (1.0 + 0.18 * ak * cs.y) * (1.0 + 0.07 * (noise2(cs * 2.5 + vec2<f32>(t * 2.5 + seed, 0.0)) - 0.5));
    if (r > rim) {
        return vec4<f32>(0.0);
    }
    // The shock: a crisp, rippling face that jitters as the Mach number
    // (and so its position) fluctuates.
    let wob = 0.045 * (noise2(cs * 1.8 + vec2<f32>(0.0, t * 6.0 + seed)) - 0.5) + 0.015 * sin(t * 13.0 + seed * 3.0);
    let es = 1.0 + wob;
    // Streamwise striations round the body (thin, high contrast, slowly
    // varying along it), coarser patches, fine flicker.
    let ang = cs * b.w;
    let streak = smoothstep(0.25, 0.85, noise2(ang * 3.2 + vec2<f32>(p.z * 0.18 - t * 0.7 + seed + r * 0.8, 0.0)));
    let blot = noise2(ang * 0.9 + vec2<f32>(p.z * 0.5 - t * 1.6 + seed, 0.0));
    let fine = noise2(ang * 6.0 + vec2<f32>(p.z * 0.4 + r * 1.5 - t * 9.0 + seed, 0.0));
    // Crisp outer surface.
    let outer = 1.0 - smoothstep(rim * 0.955, rim, r);
    var d = 0.0;
    if (s < es) {
        // Condensation builds up as the flow expands toward the shock. Most
        // of it sits in the bell's outer layer (the cone reads as a shell
        // with a translucent body), thin wisps inside.
        let build = smoothstep(0.0, 0.1, s) * (0.35 + 0.65 * pow(sc, 1.3));
        let skin = 0.18 + 0.82 * smoothstep(0.45, 0.93, r / rim);
        let body = (0.25 + 0.95 * streak) * (0.55 + 0.6 * blot) * (0.85 + 0.3 * fine);
        d = build * skin * outer * body * (1.0 - smoothstep(es - 0.012, es, s));
    } else {
        // Wisps torn off behind the shock.
        d = (1.0 - smoothstep(es, 1.3, s)) * streak * outer * 0.35 * fine;
    }
    return vec4<f32>(d, cs.x, cs.y, 0.0);
}

// Wing-section at |x| from the four stations: (leading edge, trailing edge, y, 0..1 along them).
fn wing_station(ax: f32) -> vec4<f32> {
    var a = V.vol[8];
    var b = V.vol[9];
    var k = 0.0;
    if (ax > V.vol[10].x) {
        a = V.vol[10];
        b = V.vol[11];
        k = 2.0;
    } else if (ax > V.vol[9].x) {
        a = V.vol[9];
        b = V.vol[10];
        k = 1.0;
    }
    let f = clamp((ax - a.x) / max(b.x - a.x, 1e-3), 0.0, 1.0);
    let w = mix(a, b, f);
    return vec4<f32>(w.y, w.z, w.w, (k + f) / 3.0);
}

// Fog over the forward upper surface of the wings (the suction peak).
fn wing_fog(p: vec3<f32>, t: f32) -> f32 {
    let ax = abs(p.x);
    if (ax < V.vol[8].x || ax > V.vol[11].x) {
        return 0.0;
    }
    let st = wing_station(ax);
    let fus = V.vol[6].y;
    let sb = clamp((ax - fus) / max(V.vol[6].z - fus, 0.5), 0.0, 1.0);
    let tip = select(V.vol[7].y, V.vol[7].x, p.x >= 0.0);
    let wy = st.z + sb * sb * (6.0 - 4.0 * sb + sb * sb) / 3.0 * tip;
    let chord = max(st.x - st.y, 0.3);
    let f = (st.x - p.z) / chord;
    if (f < -0.05 || f > 0.75) {
        return 0.0;
    }
    // Upper surface of a ~15 % section, thickest near 30 % chord; the fog
    // starts inside the wing (the scene depth cuts it at the real skin).
    let fc = clamp(f, 0.0, 1.0);
    let surf = wy + 0.075 * chord * sqrt(fc) * (1.0 - fc) * 2.2;
    let h = p.y - surf;
    let depth = V.vol[6].w * (1.0 - 0.5 * st.w);
    let floor_h = wy + 0.01 * chord - surf;
    if (h < floor_h || h > depth * 1.4) {
        return 0.0;
    }
    // Ends where the pressure recovers (a ragged, flickering line).
    let fe = 0.45 + 0.16 * n3(vec3<f32>(ax * 0.9, 0.0, t * 3.0)) + 0.04 * sin(ax * 2.3 + t * 9.0);
    let along = smoothstep(-0.04, 0.04, f) * (1.0 - smoothstep(fe - 0.05, fe, f));
    // Chordwise streaks, thicker in billows.
    let streak = n3(vec3<f32>(ax * 4.5, f * 1.2 - t * 5.0, h * 2.0));
    let top = depth * (0.5 + 0.9 * n3(vec3<f32>(ax * 0.9, f * 2.0 - t * 3.0, 7.0)));
    let vert = (1.0 - smoothstep(0.4 * top, top, h));
    let span = 1.0 - smoothstep(0.75, 1.0, st.w);
    return along * vert * span * (0.45 + 0.8 * streak);
}

@vertex
fn vs_vol(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> VO {
    var o: VO;
    let lo = V.vol[12u + 2u * ii].xyz;
    let hi = V.vol[13u + 2u * ii].xyz;
    // A box: face vi / 6, corner vi % 6 (both windings drawn, fs_vol keeps the far side).
    let face = vi / 6u;
    let c = corner(vi) * 0.5 + 0.5;
    let sd = f32(face % 2u);
    var u = vec3<f32>(c.x, c.y, sd);
    if (face / 2u == 0u) {
        u = vec3<f32>(sd, c.x, c.y);
    } else if (face / 2u == 1u) {
        u = vec3<f32>(c.x, sd, c.y);
    }
    let base = V.vol[0].xyz + V.off.xyz;
    let rel = qrot(V.vol[1], mix(lo, hi, u)) + base;
    o.clip = g.view_proj * vec4<f32>(rel, 1.0);
    o.uv = vec2<f32>(0.0);
    o.rel = rel;
    o.kind = f32(ii);
    o.col = vec4<f32>(0.0);
    o.extra = vec4<f32>(0.0);
    o.centre = base;
    o.size = 1.0;
    o.aux = vec4<f32>(0.0);
    o.light = vec4<f32>(1.0, 0.0, 0.0, 1.0);
    let lt = light_at(rel, 0.55, o.clip, true);
    o.direct = lt.direct;
    o.ambient = lt.ambient;
    o.fog = lt.fog;
    o.cloud = cloud_vis(o.clip, length(rel));
    return o;
}

@fragment
fn fs_vol(in: VO) -> @location(0) vec4<f32> {
    let ii = u32(in.kind + 0.5);
    let lo = V.vol[12u + 2u * ii].xyz;
    let hi = V.vol[13u + 2u * ii].xyz;
    let q = V.vol[1];
    let qi = vec4<f32>(-q.xyz, q.w);
    let d = length(in.rel);
    let dw = in.rel / max(d, 1e-4);
    let eye = qrot(qi, -(V.vol[0].xyz + V.off.xyz));
    let dir = qrot(qi, dw);
    // Ray / box in the body frame.
    let inv = select(vec3<f32>(1e8), 1.0 / dir, abs(dir) > vec3<f32>(1e-6));
    let ta = (lo - eye) * inv;
    let tb = (hi - eye) * inv;
    let t0 = max(max(min(ta.x, tb.x), min(ta.y, tb.y)), min(ta.z, tb.z));
    let t1 = min(min(max(ta.x, tb.x), max(ta.y, tb.y)), max(ta.z, tb.z));
    // Only the far side of the box shades (one fragment per pixel).
    if (t1 <= max(t0, 0.0) || d < 0.5 * (max(t0, 0.0) + t1)) {
        discard;
    }
    let px = in.clip.xy;
    let scene = textureLoad(dist_tex, vec2<i32>(px), 0).x;
    let s0 = max(t0, 0.05);
    let s1 = min(t1, scene);
    if (s1 <= s0) {
        discard;
    }
    let t = V.off.w;
    let wing = ii > 0u;
    // (misc.z: the effects quality's step factor.)
    let n = max(i32(f32(select(32, 24, wing)) * V.misc.z), 8);
    let dt = (s1 - s0) / f32(n);
    // Interleaved gradient noise: per-pixel start offset (no banding).
    let jit = fract(52.9829189 * fract(dot(px, vec2<f32>(0.06711056, 0.00583715))));
    let sun = qrot(qi, g.sun_dir.xyz);
    var k = 0.0;
    if (wing) {
        k = V.vol[6].x * 6.0 / max(V.vol[6].w, 0.08);
    }
    var tr = 1.0;
    var lum = vec3<f32>(0.0);
    let alb = 0.96;
    for (var i = 0; i < n; i++) {
        let p = eye + dir * (s0 + (f32(i) + jit) * dt);
        var den = 0.0;
        var wrap = 0.5 + 0.5 * sun.y;
        if (wing) {
            den = wing_fog(p, t) * k;
        } else {
            let a = bell(p, V.vol[2], V.vol[3].x, V.vol[3].y, t, 0.0);
            var dd = a.x * V.vol[3].z * 3.8 / max(V.vol[2].w, 0.5);
            var nl = vec3<f32>(a.y, a.z, 0.0);
            if (V.vol[5].y > 0.0) {
                let c = bell(p, V.vol[4], V.vol[5].x, V.vol[5].z, t, 4.7);
                let dc = c.x * V.vol[5].y * 3.8 / max(V.vol[4].w, 0.3);
                if (dc > dd) {
                    nl = vec3<f32>(c.y, c.z, 0.0);
                }
                dd = max(dd, dc);
            }
            den = dd;
            wrap = 0.5 + 0.5 * dot(nl, sun);
        }
        if (den > 1e-4) {
            let e = exp(-den * dt);
            // Key light (wrapped: the far side is lit through the cloud by
            // multiple scattering), sky light, a bounce of the key light.
            let s = in.direct * (0.4 + 0.6 * wrap) + in.ambient + in.direct * 0.2;
            lum += tr * (1.0 - e) * alb * s;
            tr *= e;
            if (tr < 0.01) {
                break;
            }
        }
    }
    let a = (1.0 - tr) * in.cloud;
    if (a < 0.002) {
        discard;
    }
    let c = lum * in.fog.a * in.cloud + in.fog.rgb * a;
    return vec4<f32>(c, a);
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
