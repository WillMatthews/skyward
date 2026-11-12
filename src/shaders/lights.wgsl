// Light sprites, moving traffic, the ground light map, stars and aircraft
// light beams (lights.rs). atmos_core.wgsl + common.wgsl are prepended.
//
// Every light is a point source of known intensity (cd). Its irradiance at
// the eye, E = I / d², is spread over a gaussian footprint of at least
// ~0.75 px sigma with exactly that total energy: distant lights get dimmer,
// never smaller than a pixel, so they neither alias nor sparkle.

@group(0) @binding(12) var cover_tex: texture_2d<f32>;
@group(0) @binding(13) var dist_full: texture_2d<f32>;

const K_OMNI: u32 = 0u;
const K_STREET: u32 = 1u;
const K_BIDIR: u32 = 2u;
const K_UNIDIR: u32 = 3u;
const K_FLASHER: u32 = 4u;
const K_PAPI: u32 = 5u;
const K_BEACON: u32 = 6u;
const K_FLOOD: u32 = 8u;
const K_OBSTACLE: u32 = 9u;
const K_SPOT: u32 = 10u;
const K_TAXI: u32 = 11u;
const K_FAR: u32 = 12u;
const K_STROBE: u32 = 13u;

// Physical size (m) of a lamp's glowing part: sets the sprite size up close.
const LAMP_SIZE: f32 = 0.35;
// Smallest sprite (pixels, gaussian sigma).
const MIN_SIGMA: f32 = 0.72;

struct LIn {
    @location(0) pos: vec3<f32>,
    @location(1) cd: f32,
    @location(2) col: u32,
    @location(3) dir: u32,
    @location(4) a: u32,
    @location(5) b: u32,
};

struct SOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) q: vec2<f32>,                         // offset from the centre (px)
    @location(1) @interpolate(flat) core: vec4<f32>,   // rgb peak radiance, w = 1 / (2 sigma²)
    @location(2) @interpolate(flat) halo: vec4<f32>,   // rgb peak radiance, w = 1 / (2 sigma²)
    @location(3) @interpolate(flat) ldist: f32,        // eye distance of the light (m)
};

fn unpack_rgb(c: u32) -> vec3<f32> {
    return unpack4x8unorm(c).rgb;
}

fn px_per_rad() -> f32 {
    return g.sky.w;
}

fn corner(vi: u32) -> vec2<f32> {
    var c = array<vec2<f32>, 6>(vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, -1.0), vec2<f32>(1.0, 1.0), vec2<f32>(-1.0, 1.0));
    return c[vi % 6u];
}

fn culled() -> SOut {
    var o: SOut;
    o.clip = vec4<f32>(2.0, 2.0, 2.0, 1.0);
    o.q = vec2<f32>(0.0);
    o.core = vec4<f32>(0.0);
    o.halo = vec4<f32>(0.0);
    o.ldist = 0.0;
    return o;
}

// Ground-fog optical depth between two heights over a slant distance.
fn fog_od(h0: f32, h1: f32, dist: f32) -> f32 {
    let ext = g.misc.z * 0.001;
    let top = g.misc.w;
    if (ext <= 0.0 || top <= 1.0) {
        return 0.0;
    }
    let lo = min(h0, h1);
    let hi = max(h0, h1);
    let span = max(hi - lo, 1e-3);
    // Fraction of the path inside the (soft-topped) layer.
    let inside = clamp((top * 0.8 - lo) / span, 0.0, 1.0);
    return ext * dist * select(inside, select(0.0, 1.0, lo < top * 0.8), span < 1.0);
}

// Intensity factor and colour of a light toward a viewer in direction `tv`
// (unit, from the light to the eye). Returns rgb * factor.
fn emission(k: u32, l: LIn, tv: vec3<f32>) -> vec3<f32> {
    let c = unpack_rgb(l.col);
    let d4 = unpack4x8snorm(l.dir);
    let D = d4.xyz;
    let t = g.cam_pos.w;
    let el = asin(clamp(tv.y, -1.0, 1.0));
    switch k {
        case 1u: { // street lamp: bright from below / the side, cut-off optics hide it from above
            return c * mix(1.0, 0.28, smoothstep(-0.25, 0.75, tv.y)) * g.city.x;
        }
        case 8u: { // floodlight mast: aimed down
            return c * mix(1.0, 0.35, smoothstep(-0.1, 0.8, tv.y)) * g.city.x;
        }
        case 12u: { // aggregated far-city lights (hidden where detailed tiles exist)
            let code = u32(bitcast<f32>(l.b));
            let cov = textureLoad(cover_tex, vec2<i32>(i32(code & 127u), i32(code >> 7u)), 0).r;
            return c * (1.0 - step(0.5, cov)) * mix(1.0, 0.45, smoothstep(-0.2, 0.8, tv.y)) * g.city.x;
        }
        case 2u: { // runway: bidirectional, beam along the axis (toe-in, 0..12 deg up)
            let s = dot(tv, D);
            let col = select(unpack_rgb(l.a), c, s < 0.0);
            let h = normalize(vec3<f32>(tv.x, 0.0, tv.z) + vec3<f32>(0.0, 1e-4, 0.0));
            let along = abs(dot(h, D));
            let beam = 0.08 + 0.92 * smoothstep(0.55, 0.97, along) * (1.0 - smoothstep(0.22, 0.6, el));
            return col * beam * g.misc.x;
        }
        case 3u, 4u, 5u: { // directional toward D (approach lights, flashers, PAPI)
            let f = dot(tv, D);
            let h = normalize(vec3<f32>(tv.x, 0.0, tv.z) + vec3<f32>(0.0, 1e-4, 0.0));
            let fh = dot(h, normalize(vec3<f32>(D.x, 0.0, D.z) + vec3<f32>(0.0, 1e-4, 0.0)));
            var beam = smoothstep(0.82, 0.98, fh) * (1.0 - smoothstep(0.25, 0.55, el)) + 0.01 * step(0.0, f);
            var col = c;
            if (k == 4u) {
                // Sequenced flasher: 2 Hz, outer to inner, ~40 ms flash.
                let ph = fract(t * 2.0 + bitcast<f32>(l.a));
                beam *= smoothstep(0.07, 0.0, ph) * 1.5;
            } else if (k == 5u) {
                // PAPI: white above its angle, red below (~2 arcmin transition).
                let ang = bitcast<f32>(l.a);
                let w = smoothstep(-0.0006, 0.0006, el - ang);
                col = mix(vec3<f32>(1.0, 0.04, 0.02), vec3<f32>(1.0, 0.92, 0.82), w);
            }
            return col * beam * g.misc.x;
        }
        case 6u: { // rotating aerodrome beacon: white and green lobes, 12 rpm
            let az = atan2(tv.x, -tv.z);
            let rot = t * 6.2831853 * 0.2;
            let w = pow(max(cos(az - rot), 0.0), 60.0);
            let gr = pow(max(cos(az - rot - 3.14159265), 0.0), 60.0);
            let elf = 0.12 + 0.88 * exp(-max(el - 0.02, 0.0) / 0.12);
            return (c * w + unpack_rgb(l.a) * gr) * elf * g.misc.y;
        }
        case 9u: { // aircraft-warning light (night only)
            return c * g.sun_true.w;
        }
        case 13u: { // obstacle strobe, 40 flashes / min
            let ph = fract(t * 0.6667 + bitcast<f32>(l.a));
            return c * smoothstep(0.05, 0.0, ph) * max(g.sun_true.w, 0.15);
        }
        case 10u: { // aircraft landing / taxi light: a beam toward D
            let cosa = bitcast<f32>(l.a);
            let f = dot(tv, D);
            return c * (smoothstep(cosa - 0.08, cosa + 0.02, f) + 0.003 * step(0.0, f));
        }
        case 11u: { // taxiway
            return c * g.misc.x * (1.0 - 0.6 * smoothstep(0.3, 0.9, el));
        }
        default: { // omni (aircraft nav lights, flashes switched per frame)
            return c;
        }
    }
}

// Common sprite setup. `halo_pass`: compute the haze / fog halo instead of the core.
fn sprite(l: LIn, vi: u32, halo_pass: bool) -> SOut {
    let k = l.col >> 24u;
    let p = tile_pos(l.pos);
    let v = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    let dist = length(v);
    if (dist < 0.05 || l.cd <= 0.0) {
        return culled();
    }
    let tv = -v / dist;
    let rgb = emission(k, l, tv) * l.cd;
    if (max(rgb.r, max(rgb.g, rgb.b)) <= 0.0) {
        return culled();
    }
    let cc = g.view_proj * vec4<f32>(p, 1.0);
    if (cc.w <= 0.0) {
        return culled();
    }
    let ndc = cc.xy / cc.w;
    if (any(abs(ndc) > vec2<f32>(1.2))) {
        return culled();
    }
    let ppr = px_per_rad();
    // Haze: aerial-perspective transmittance to the light.
    let uv = vec2<f32>(ndc.x * 0.5 + 0.5, 0.5 - ndc.y * 0.5);
    let ap = textureSampleLevel(ap_tex, lin_samp, vec3<f32>(uv, ap_slice_of(dist * 0.001)), 0.0).a;
    let ta = mix(1.0, ap, smoothstep(0.0, 1.0 / AP_N, ap_slice_of(dist * 0.001)));
    // Irradiance at the eye (scene units).
    let e = rgb * (LUX / (dist * dist));
    let expo = atmos.expo.x;
    var o: SOut;
    var radius: f32;
    if (!halo_pass) {
        // Glowing size: lamp heads / floods ~35 cm, runway / aircraft lenses ~10 cm.
        var size = LAMP_SIZE;
        if (k == K_OMNI || k == K_BIDIR || k == K_UNIDIR || k == K_PAPI || k == K_TAXI || k == K_OBSTACLE) {
            size = 0.1;
        }
        let sig = max(size / dist * ppr, MIN_SIGMA);
        let sr = sig / ppr;
        // (Pre-exposed; clamped to stay inside fp16.)
        let peak = min(e * ta / (6.2831853 * sr * sr) * g.pre.x, vec3<f32>(6.0e4));
        if (max(peak.r, max(peak.g, peak.b)) * expo < 0.004) {
            return culled();
        }
        o.core = vec4<f32>(peak, 0.5 / (sig * sig));
        o.halo = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        // Out to where the gaussian falls below ~1/512 of its (clamped) peak.
        let lit = max(peak.r, max(peak.g, peak.b)) * expo;
        radius = sig * clamp(sqrt(2.0 * log(max(lit * 512.0, 2.0))), 2.0, 6.0);
    } else {
        // Light scattered out of the direct path by haze and fog arrives
        // from around the light: a soft halo (forward-scattering share).
        let od_f = fog_od(g.cam_pos.y, g.cam_pos.y - v.y, dist);
        let tf = exp(-od_f);
        let scat = (1.0 - ta) * 0.15 + (1.0 - tf) * 0.45;
        // Angular spread: wider in dense fog (multiple scattering).
        let ang = mix(0.012, 0.05, clamp(od_f / 3.0, 0.0, 1.0));
        let sig = clamp(ang * ppr, 3.0, 48.0);
        let sr = sig / ppr;
        let peak = min(e * scat * tf * exp(-od_f * 0.15) / (6.2831853 * sr * sr) * g.pre.x, vec3<f32>(6.0e4));
        if (max(peak.r, max(peak.g, peak.b)) * expo < 0.002 || dist > 6000.0) {
            return culled();
        }
        o.core = vec4<f32>(0.0, 0.0, 0.0, 1.0);
        o.halo = vec4<f32>(peak, 0.5 / (sig * sig));
        // Out to where it falls below ~1/1000 of the visible level.
        let lit = max(peak.r, max(peak.g, peak.b)) * expo;
        radius = sig * clamp(sqrt(2.0 * log(max(lit * 1000.0, 2.0))), 2.0, 5.0);
    }
    let cr = corner(vi);
    o.q = cr * radius;
    // Pull the quad toward the eye so it isn't cut by the surface it stands on.
    let pull = min(radius / ppr * dist * 1.2, dist * 0.3);
    let pc = g.view_proj * vec4<f32>(p * (1.0 - pull / dist), 1.0);
    o.clip = vec4<f32>(pc.xy + cr * radius * 2.0 * g.screen.zw * pc.w, pc.z, pc.w);
    o.ldist = dist;
    return o;
}

@vertex
fn vs_sprite(@builtin(vertex_index) vi: u32, l: LIn) -> SOut {
    return sprite(l, vi, false);
}

@vertex
fn vs_halo(@builtin(vertex_index) vi: u32, l: LIn) -> SOut {
    return sprite(l, vi, true);
}

struct SpriteFOut {
    @location(0) col: vec4<f32>,
    @location(1) dist: f32,
};

@fragment
fn fs_sprite(in: SOut) -> SpriteFOut {
    let r2 = dot(in.q, in.q);
    var o: SpriteFOut;
    o.col = vec4<f32>(in.core.rgb * exp(-r2 * in.core.w), 0.0);
    o.dist = 0.0;
    return o;
}

// Halo: drawn after the fog pass over the resolved image; hidden by geometry
// in front of the light (the scattering happens between the eye and it).
@fragment
fn fs_halo(in: SOut) -> @location(0) vec4<f32> {
    let d = textureLoad(dist_full, vec2<i32>(in.clip.xy), 0).r;
    let vis = smoothstep(in.ldist * 0.7, in.ldist * 0.95, d);
    let r2 = dot(in.q, in.q);
    return vec4<f32>(in.halo.rgb * exp(-r2 * in.halo.w) * vis, 0.0);
}

// ---------------------------------------------------------------- traffic

struct TIn {
    @location(0) ab: vec4<f32>,      // a.xz, b.xz (tile frame)
    @location(1) info: vec4<f32>,    // width, seed, cars, speed
    @location(2) yy: vec4<f32>,      // ground height at a, b (terrain)
};

// 6 cars x 2 directions x 2 lights (head, tail) x 6 vertices.
@vertex
fn vs_traffic(@builtin(vertex_index) vi: u32, t: TIn) -> SOut {
    let corner_i = vi % 6u;
    let light = (vi / 6u) % 2u;     // 0 head, 1 tail
    let side = (vi / 12u) % 2u;     // direction of travel
    let car = vi / 24u;
    let cars = t.info.z * g.misc2.x;
    if (f32(car) + 0.5 > cars * (0.7 + 0.6 * fract(t.info.y * 13.7 + f32(side) * 0.37))) {
        return culled();
    }
    var a = t.ab.xy;
    var b = t.ab.zw;
    var ya = t.yy.x;
    var yb = t.yy.y;
    if (side == 1u) {
        let tmp = a;
        a = b;
        b = tmp;
        ya = t.yy.y;
        yb = t.yy.x;
    }
    let len = distance(a, b);
    let d = (b - a) / max(len, 1e-3);
    let n = vec2<f32>(-d.y, d.x);
    // Keep left (UK): the lane is on the left of the travel direction.
    let lane = -n * (t.info.x * 0.25 + 0.4);
    let speed = t.info.w * (0.85 + 0.3 * fract(t.info.y * 7.1 + f32(car) * 0.61));
    let s = fract(t.info.y * 3.7 + f32(side) * 0.5 + (f32(car) + 0.3 * fract(f32(car) * 0.73)) / max(cars, 1.0) + g.cam_pos.w * speed / max(len, 1.0)) * len;
    let fade = smoothstep(0.0, 12.0, s) * smoothstep(0.0, 12.0, len - s);
    let ofs = select(-2.1, 2.1, light == 0u);
    let xz = a + d * (s + ofs) + lane;
    var l: LIn;
    l.pos = vec3<f32>(xz.x, mix(ya, yb, clamp((s + ofs) / max(len, 1e-3), 0.0, 1.0)) + 0.7, xz.y);
    let fwd = vec3<f32>(d.x, 0.0, d.y);
    var col: vec3<f32>;
    var cd: f32;
    // The direction to the eye decides how much of the beam is seen.
    let p = tile_pos(l.pos);
    let tv = -normalize(p + vec3<f32>(0.0, g.pattern.w, 0.0));
    if (light == 0u) {
        col = vec3<f32>(1.0, 0.93, 0.8);
        cd = 250.0 + 12000.0 * pow(max(dot(tv, fwd), 0.0), 12.0);
    } else {
        col = vec3<f32>(1.0, 0.04, 0.02);
        cd = 45.0 + 160.0 * pow(max(-dot(tv, fwd), 0.0), 4.0);
    }
    l.cd = cd * fade * g.city.x;
    l.col = pack4x8unorm(vec4<f32>(col, 0.0));
    l.dir = 0u;
    l.a = 0u;
    l.b = 0u;
    return sprite(l, corner_i, false);
}

// ---------------------------------------------------------------- ground light map

struct MOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) q: vec2<f32>,                       // metres from the lamp (horizontal)
    @location(1) @interpolate(flat) rgb: vec3<f32>,  // colour * ground intensity (cd) * LUX
    @location(2) @interpolate(flat) h: f32,
};

@vertex
fn vs_map(@builtin(vertex_index) vi: u32, l: LIn) -> MOut {
    var o: MOut;
    let k = l.col >> 24u;
    var on = 0.0;
    if (k == K_STREET || k == K_FLOOD) {
        on = g.city.x;
    }
    let i0 = bitcast<f32>(l.a);
    let h = max(bitcast<f32>(l.b), 1.0);
    if (on <= 0.0 || i0 <= 0.0) {
        o.clip = vec4<f32>(2.0, 2.0, 2.0, 1.0);
        return o;
    }
    let p = tile_pos(l.pos);
    let r = h * 3.0;
    let cr = corner(vi);
    let m = (p.xz + cr * r - g.lmap.xy) * g.lmap.z * 2.0;
    o.clip = vec4<f32>(m.x, -m.y, 0.0, 1.0);
    o.q = cr * r;
    o.rgb = unpack_rgb(l.col) * i0 * on * LUX;
    o.h = h;
    return o;
}

@fragment
fn fs_map(in: MOut) -> @location(0) vec4<f32> {
    let r2 = dot(in.q, in.q);
    let h = in.h;
    let s2 = r2 + h * h;
    // Cosine law on the ground, widened by the lamp's batwing optics, cut off ~2.8 h.
    var e = h / (s2 * sqrt(s2)) * (1.0 + 1.2 * r2 / s2);
    e *= 1.0 - smoothstep(1.4 * h, 2.95 * h, sqrt(r2));
    return vec4<f32>(in.rgb * e, 0.0);
}

// ---------------------------------------------------------------- stars

struct StarIn {
    @location(0) dir_e: vec4<f32>,  // equatorial unit vector, irradiance (scene units)
    @location(1) col: vec4<f32>,    // rgb, w = twinkle seed
};

@vertex
fn vs_star(@builtin(vertex_index) vi: u32, s: StarIn) -> SOut {
    let w = g.stars0.xyz * s.dir_e.x + g.stars1.xyz * s.dir_e.y + g.stars2.xyz * s.dir_e.z;
    if (w.y < -0.02) {
        return culled();
    }
    // Extinction (airmass) and twinkling (stronger low down).
    let tr = transmittance(abs(g.cam_pos.y), max(w.y, 0.0));
    let air = 1.0 / max(w.y + 0.05, 0.08);
    let tw = 1.0 + 0.22 * min(air * 0.4, 1.0) * sin(g.cam_pos.w * (9.0 + 7.0 * s.col.w) + s.col.w * 50.0) * sin(g.cam_pos.w * 13.1 + s.col.w * 17.0);
    let e = s.dir_e.w * s.col.rgb * tr * tw * smoothstep(-0.02, 0.03, w.y);
    let ppr = px_per_rad();
    let sig = MIN_SIGMA * 1.05;
    let sr = sig / ppr;
    let peak = e / (6.2831853 * sr * sr) * g.pre.x;
    let expo = atmos.expo.x;
    let lit = max(peak.r, max(peak.g, peak.b)) * expo;
    if (lit < 0.004) {
        return culled();
    }
    let cc = g.view_proj * vec4<f32>(w, 0.0);
    if (cc.w <= 0.0) {
        return culled();
    }
    var o: SOut;
    let cr = corner(vi);
    let radius = sig * clamp(sqrt(2.0 * log(max(lit * 512.0, 2.0))), 2.0, 5.0);
    o.q = cr * radius;
    o.core = vec4<f32>(peak, 0.5 / (sig * sig));
    o.halo = vec4<f32>(0.0, 0.0, 0.0, 1.0);
    o.clip = vec4<f32>(cc.xy + cr * radius * 2.0 * g.screen.zw * cc.w, 0.0, cc.w);
    o.ldist = 1e9;
    return o;
}

// ---------------------------------------------------------------- aircraft light beams

struct BOut {
    @builtin(position) pos: vec4<f32>,
};

@vertex
fn vs_full(@builtin(vertex_index) i: u32) -> BOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: BOut;
    o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    return o;
}

// Scattering coefficient of the air near the ground (/m): haze + fog + rain.
fn scat_coeff(h: f32) -> f32 {
    let hk = max(h, 0.0) * 0.001;
    let haze = g.sun_e.w;
    let e = exp(-hk / 1.2);
    let bl = haze * e * (1.0 + 2.0 * (1.0 - smoothstep(0.6, 2.2, hk)));
    var s = (MIE_S * max(bl, e) + 0.0116 * exp(-hk / 8.0)) * 0.001;
    let top = g.misc.w;
    if (g.misc.z > 0.0 && top > 1.0) {
        s += g.misc.z * 0.001 * 0.97 * (1.0 - smoothstep(top * 0.55, top, h));
    }
    s += g.wx.y * 0.0004 + g.wx.z * 0.0012;
    return s;
}

fn ign(p: vec2<f32>) -> f32 {
    return fract(52.9829189 * fract(dot(p, vec2<f32>(0.06711056, 0.00583715))));
}

// In-scattered light of the aircraft's spot / point lights along each pixel's
// ray (single scattering, Henyey-Greenstein), up to the opaque surface.
@fragment
fn fs_beams(in: BOut) -> @location(0) vec4<f32> {
    let n = i32(g.misc2.y);
    if (n <= 0) {
        return vec4<f32>(0.0);
    }
    let px = vec2<i32>(in.pos.xy);
    let uv = in.pos.xy * g.screen.zw;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let pp = g.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    let rd = normalize(pp.xyz / pp.w);
    let dmax = min(textureLoad(dist_full, px, 0).r, 2000.0);
    let jit = ign(in.pos.xy + fract(g.cam_pos.w * 7.31) * 97.0);
    var acc = vec3<f32>(0.0);
    let gg = 0.55;
    for (var i = 0; i < n; i++) {
        let lp = g.alights[i * 3].xyz;
        let range = g.alights[i * 3].w;
        let ld = g.alights[i * 3 + 1].xyz;
        let cos_o = g.alights[i * 3 + 1].w;
        let cd = g.alights[i * 3 + 2].rgb;
        let cos_i = g.alights[i * 3 + 2].w;
        // March the part of the ray near the light.
        let tc = clamp(dot(lp, rd), 0.0, dmax);
        let closest = length(lp - rd * tc);
        if (closest > range) {
            continue;
        }
        let t0 = max(tc - range, 0.0);
        let t1 = min(tc + range, dmax);
        if (t1 <= t0) {
            continue;
        }
        let ns = 20;
        let dt = (t1 - t0) / f32(ns);
        var s = vec3<f32>(0.0);
        for (var k = 0; k < ns; k++) {
            let t = t0 + (f32(k) + jit) * dt;
            let q = rd * t;
            let lq = q - lp;
            // (The beam forms a few metres from the lens: no near-field singularity.)
            let r2 = max(dot(lq, lq), 0.25) + 36.0;
            let r = sqrt(r2);
            let ldir = lq / r;
            var cone = 1.0;
            if (cos_o > -0.99) {
                cone = smoothstep(cos_o, cos_i, dot(ldir, ld));
            }
            if (cone <= 0.0) {
                continue;
            }
            let c = dot(ldir, -rd);
            let ph = (1.0 - gg * gg) / (4.0 * PI * pow(max(1.0 + gg * gg - 2.0 * gg * c, 1e-4), 1.5));
            let h = g.cam_pos.y + q.y;
            let sc = scat_coeff(h);
            s += cd * (cone * ph * sc / r2) * exp(-sc * (t + r)) * dt;
        }
        acc += s * LUX;
    }
    return vec4<f32>(min(acc * g.pre.x, vec3<f32>(6.0e4)), 0.0);
}
