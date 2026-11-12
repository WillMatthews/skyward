// Falling rain / snow around the camera (gfx.rs). Instanced camera-facing
// streaks, no vertex buffers: each instance is a drop at a hashed position in
// one of three nested boxes around the eye; the boxes wrap, so drops are
// anchored in the moving air (fall speed + wind) and parallax correctly as
// the camera flies through them. Each drop is drawn as the segment it sweeps
// relative to the camera during the exposure (motion blur), occluded by the
// scene via the eye-distance texture. Additive-premultiplied over the HDR.

struct RainU {
    view_proj: mat4x4<f32>, // eye-relative (rotation only)
    eye: vec4<f32>,         // xyz, w = time (s)
    cam_vel: vec4<f32>,     // camera velocity (m/s), w = exposure (s)
    params: vec4<f32>,      // x intensity 0..1, y kind (0 rain, 1 snow), z wind x, w wind z
    sun: vec4<f32>,         // xyz toward the sun, w = sun irradiance scale (cloud cover)
    screen: vec4<f32>,      // w, h, focal length (px), drizzle 0/1
};

@group(0) @binding(0) var<uniform> R: RainU;
@group(0) @binding(1) var dist_tex: texture_2d<f32>;
struct AtmosOut {
    sh: array<vec4<f32>, 4>,
    sun_eye: vec4<f32>,
    expo: vec4<f32>,
};
@group(0) @binding(2) var<storage, read> atmos: AtmosOut;

/// Instances per layer (the app draws 3 * PER_LAYER * intensity-ish).
const PER_LAYER: u32 = 9000u;

struct VOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) along: f32,     // -1..1 across the streak, 0..1 along
    @location(1) across: f32,
    @location(2) dist: f32,
    @location(3) alpha: f32,
    @location(4) col: vec3<f32>,
};

fn hash(n: u32) -> u32 {
    var x = n;
    x ^= x >> 16u;
    x *= 0x7feb352du;
    x ^= x >> 15u;
    x *= 0x846ca68bu;
    x ^= x >> 16u;
    return x;
}

fn h01(n: u32) -> f32 {
    return f32(hash(n) >> 8u) * (1.0 / 16777216.0);
}

fn irr(n: vec3<f32>) -> vec3<f32> {
    return max(atmos.sh[0].rgb + atmos.sh[1].rgb * n.x + atmos.sh[2].rgb * n.y + atmos.sh[3].rgb * n.z, vec3<f32>(0.0));
}

@vertex
fn vs_rain(@builtin(vertex_index) vi: u32, @builtin(instance_index) ii: u32) -> VOut {
    var o: VOut;
    o.clip = vec4<f32>(2.0, 2.0, 2.0, 1.0); // culled unless written below
    o.along = 0.0;
    o.across = 0.0;
    o.dist = 0.0;
    o.alpha = 0.0;
    o.col = vec3<f32>(0.0);
    let snow = R.params.y > 0.5;
    let layer = ii / PER_LAYER;
    let box = select(select(150.0, 50.0, layer == 1u), 16.0, layer == 0u);
    let seed = ii * 3u + 17u;
    let t = R.eye.w;
    // Fall speed: rain 4-9 m/s by drop size (drizzle ~2), snow ~1 m/s.
    let size = h01(seed + 7u);
    var fall = select(mix(4.5, 9.0, size), mix(0.8, 1.6, size), snow);
    if (R.screen.w > 0.5 && !snow) {
        fall = mix(1.5, 3.0, size);
    }
    var v = vec3<f32>(R.params.z, -fall, R.params.w);
    var p = vec3<f32>(h01(seed), h01(seed + 1u), h01(seed + 2u)) * box + v * t;
    if (snow) {
        // Flakes flutter.
        let ph = h01(seed + 9u) * 6.283;
        let wob = vec3<f32>(sin(t * 1.3 + ph), 0.0, cos(t * 1.1 + ph * 1.7)) * 0.45;
        p += wob;
        v += vec3<f32>(cos(t * 1.3 + ph) * 0.58, 0.0, -sin(t * 1.1 + ph * 1.7) * 0.5);
    }
    // Wrap into the box centred on the eye.
    let rel = (p - R.eye.xyz) - box * floor((p - R.eye.xyz) / box + 0.5);
    let world = R.eye.xyz + rel;
    if (world.y < 0.0) {
        return o;
    }
    // Segment swept relative to the camera during the exposure.
    let vr = v - R.cam_vel.xyz;
    // Flakes read as dots: the eye tracks them (shorter effective exposure).
    let exposure = R.cam_vel.w * select(1.0, 0.25, snow);
    var a = rel;
    var b = rel - vr * exposure;
    if (!snow) {
        // Rain streaks are at least ~3 cm long (the eye integrates them).
        let len = length(b - a);
        b = a - normalize(vr + vec3<f32>(0.0, -1e-3, 0.0)) * max(len, 0.03 + 0.04 * size);
    }
    let ca = R.view_proj * vec4<f32>(a, 1.0);
    let cb = R.view_proj * vec4<f32>(b, 1.0);
    if (ca.w < 0.3 || cb.w < 0.3) {
        return o;
    }
    let sa = ca.xy / ca.w;
    let sb = cb.xy / cb.w;
    let asp = R.screen.x / R.screen.y;
    var d = (sb - sa) * vec2<f32>(asp, 1.0);
    if (length(d) < 1e-6) {
        d = vec2<f32>(0.0, 1.0);
    }
    d = normalize(d);
    let nrm = vec2<f32>(-d.y, d.x) / vec2<f32>(asp, 1.0);
    let dist = length(a);
    // Physical width (drop ~2 mm, flake ~8-15 mm); at least ~1 px, thinner
    // ones get fainter instead.
    let wm = select(0.0018 + 0.0012 * size, 0.012 + 0.014 * size, snow);
    let wpx = wm * R.screen.z / dist;
    let wdraw = max(wpx, 1.3);
    let cover = sqrt(min(wpx / 1.3, 1.0));
    let corner = vi % 6u;
    let side = select(-1.0, 1.0, corner == 1u || corner == 2u || corner == 4u);
    let end = select(0.0, 1.0, corner == 2u || corner == 4u || corner == 5u);
    let c = mix(ca, cb, end);
    let s = mix(sa, sb, end);
    // Pad the ends of snow flakes so they stay round.
    var ext = 0.0;
    if (snow) {
        ext = wdraw / R.screen.y * 2.0;
    }
    let sp = s + nrm * side * wdraw / R.screen.y + d / vec2<f32>(asp, 1.0) * (end * 2.0 - 1.0) * ext;
    o.clip = vec4<f32>(sp * c.w, c.z, c.w);
    o.along = end;
    o.across = side;
    o.dist = dist;
    // Fade the outer edge of each box (no popping where it wraps) and far layers.
    let edge = 1.0 - smoothstep(0.35, 0.5, max(max(abs(rel.x), abs(rel.y)), abs(rel.z)) / box);
    let inten = R.params.x;
    // Fewer drops at low intensity: drop a share of the instances.
    let keep = step(h01(seed + 5u), 0.15 + 0.85 * inten);
    let base_a = select(0.5, 0.9, snow) * select(1.0, 0.6, R.screen.w > 0.5);
    o.alpha = base_a * cover * edge * keep * (0.5 + 0.5 * size);
    // Lighting: drops refract the bright sky; flakes are white diffusers.
    let sky = irr(vec3<f32>(0.0, 1.0, 0.0)) / PI;
    let sun_l = atmos.sun_eye.rgb * R.sun.w;
    if (snow) {
        o.col = sky * 0.9 + sun_l * max(R.sun.y, 0.0) * 0.3;
    } else {
        o.col = sky * 1.3 + sun_l * 0.02;
    }
    return o;
}

@fragment
fn fs_rain(in: VOut) -> @location(0) vec4<f32> {
    let px = vec2<i32>(in.clip.xy);
    let scene = textureLoad(dist_tex, px, 0).x;
    if (in.dist > scene) {
        discard;
    }
    let snow = R.params.y > 0.5;
    var a = in.alpha;
    if (snow) {
        let r = length(vec2<f32>(in.across, in.along * 2.0 - 1.0));
        a *= 1.0 - smoothstep(0.5, 1.0, abs(in.across));
    } else {
        // Soft across, tapered ends.
        a *= (1.0 - abs(in.across)) * smoothstep(0.0, 0.25, in.along) * smoothstep(1.0, 0.6, in.along);
    }
    if (a < 0.002) {
        discard;
    }
    return vec4<f32>(in.col * a, 1.0 - a);
}
