// Atmosphere LUT builders (compute). atmos_core.wgsl is prepended.
//   cs_transmittance  256x64   T(r, mu)                      (once)
//   cs_multiscatter   32x32    Psi_ms(r, mu_s)                (once)
//   cs_skyview        192x108  sky radiance per unit sun irradiance at the eye
//   cs_aerial         32^3     aerial perspective froxels (in-scatter, transmittance)
//   cs_ambient        1 group  L1 SH sky irradiance + exposure metering

struct AtmosU {
    inv_view_proj: mat4x4<f32>, // camera of the AP volume
    eye: vec4<f32>,             // metres; w = haze
    sun: vec4<f32>,             // direction toward the sun; w = cloud cover 0..1
    sun_e: vec4<f32>,           // top-of-atmosphere sun irradiance (rgb); w = mirrored (|y|) flag
    params: vec4<f32>,          // x reference illuminance, y cloud base (m), z LUT scale (sky-view / aerial
                                // in-scatter are stored x this so faint night skies stay inside fp16), w exposure key
    fog: vec4<f32>,             // x ground fog extinction (/km), y fog top (km), z overcast deck transmission, w unused
    moon: vec4<f32>,            // toward the moon; w = moon irradiance / sun irradiance (top of the atmosphere)
    night: vec4<f32>,           // x airglow radiance, y city upward radiance (both / sun_e), z artificial metering illuminance,
                                // w pre-exposure (sun_e above includes it; expo is divided by it)
    glow_tint: vec4<f32>,       // skyglow colour
    glow: array<vec4<f32>, 32>, // skyglow radiance / sun_e: 8 elevation x 16 azimuth bins
};

struct AtmosOut {
    sh: array<vec4<f32>, 4>,    // irradiance E(n) = sh0 + sh1 * n.x + sh2 * n.y + sh3 * n.z (rgb)
    sun_eye: vec4<f32>,         // sun irradiance at the eye (rgb, after transmittance)
    expo: vec4<f32>,            // x exposure multiplier, y metered illuminance
};

@group(0) @binding(0) var<uniform> A: AtmosU;
@group(0) @binding(1) var trans_tex: texture_2d<f32>;
@group(0) @binding(2) var ms_tex: texture_2d<f32>;
@group(0) @binding(3) var lin: sampler;
@group(0) @binding(4) var trans_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(5) var ms_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(6) var sky_out: texture_storage_2d<rgba16float, write>;
@group(0) @binding(7) var ap_out: texture_storage_3d<rgba16float, write>;
@group(0) @binding(8) var<storage, read_write> outb: AtmosOut;
@group(0) @binding(9) var sky_tex: texture_2d<f32>;

fn haze() -> f32 {
    return A.eye.w;
}

fn med(h_km: f32) -> Medium {
    return medium(h_km, haze(), A.fog.xy);
}

fn trans_at(r: f32, mu: f32) -> vec3<f32> {
    if (hits_ground(r, mu)) {
        return vec3<f32>(0.0);
    }
    return textureSampleLevel(trans_tex, lin, trans_uv(r, mu), 0.0).rgb;
}

fn ms_at(r: f32, mu_s: f32) -> vec3<f32> {
    let uv = vec2<f32>(clamp(mu_s * 0.5 + 0.5, 0.0, 1.0), clamp((r - RG) / (RT - RG), 0.0, 1.0));
    let t = uv * (1.0 - 1.0 / MS_N) + 0.5 / MS_N;
    return textureSampleLevel(ms_tex, lin, t, 0.0).rgb;
}

// ------------------------------------------------------------------ transmittance

@compute @workgroup_size(8, 8)
fn cs_transmittance(@builtin(global_invocation_id) id: vec3<u32>) {
    if (f32(id.x) >= TRANS_W || f32(id.y) >= TRANS_H) {
        return;
    }
    let uv = (vec2<f32>(id.xy) + 0.5) / vec2<f32>(TRANS_W, TRANS_H);
    let rm = trans_rmu(uv);
    let r = rm.x;
    let mu = rm.y;
    let len = max(ray_sphere(r, mu, RT), 0.0);
    // A shallow ground-fog layer needs its own fine steps.
    var tf = 0.0;
    let fog_top = A.fog.y;
    if (A.fog.x > 0.0 && r - RG < fog_top) {
        tf = min(len, select(5.0, (fog_top - (r - RG)) / max(mu, 1e-3), mu > 1e-3));
    }
    var od = vec3<f32>(0.0);
    let nf = 12;
    for (var i = 0; i < nf; i++) {
        let dtf = tf / f32(nf);
        let t = (f32(i) + 0.5) * dtf;
        let ri = sqrt(r * r + t * t + 2.0 * r * mu * t);
        od += med(ri - RG).ext * dtf;
    }
    let n = 48;
    let dt = (len - tf) / f32(n);
    for (var i = 0; i < n; i++) {
        let t = tf + (f32(i) + 0.5) * dt;
        let ri = sqrt(r * r + t * t + 2.0 * r * mu * t);
        od += med(ri - RG).ext * dt;
    }
    textureStore(trans_out, vec2<i32>(id.xy), vec4<f32>(exp(-od), 1.0));
}

// ------------------------------------------------------------------ multiple scattering

var<workgroup> ms_l: array<vec3<f32>, 64>;
var<workgroup> ms_f: array<vec3<f32>, 64>;

@compute @workgroup_size(64)
fn cs_multiscatter(@builtin(workgroup_id) wg: vec3<u32>, @builtin(local_invocation_index) li: u32) {
    let uv = (vec2<f32>(wg.xy) + 0.5) / MS_N;
    let mu_s = uv.x * 2.0 - 1.0;
    let r = RG + clamp(uv.y, 0.002, 0.998) * (RT - RG);
    let sun = vec3<f32>(sqrt(max(1.0 - mu_s * mu_s, 0.0)), mu_s, 0.0);
    // Fibonacci sphere direction.
    let i = f32(li);
    let z = 1.0 - 2.0 * (i + 0.5) / 64.0;
    let a = i * 2.39996323;
    let s = sqrt(max(1.0 - z * z, 0.0));
    let d = vec3<f32>(s * cos(a), z, s * sin(a));
    let o = vec3<f32>(0.0, r, 0.0);
    let mu = d.y;
    var tmax = ray_sphere(r, mu, RT);
    let ground = hits_ground(r, mu);
    if (ground) {
        let disc = r * r * (mu * mu - 1.0) + RG * RG;
        tmax = -r * mu - sqrt(max(disc, 0.0));
    }
    let n = 20;
    let dt = max(tmax, 0.0) / f32(n);
    var T = vec3<f32>(1.0);
    var L = vec3<f32>(0.0);
    var F = vec3<f32>(0.0);
    let iso = 1.0 / (4.0 * PI);
    for (var k = 0; k < n; k++) {
        let t = (f32(k) + 0.5) * dt;
        let p = o + d * t;
        let rp = length(p);
        let m = med(rp - RG);
        let sc = m.ray + vec3<f32>(m.mie);
        let tsun = trans_at(rp, dot(p / rp, sun));
        let Ts = exp(-m.ext * dt);
        let S = sc * iso * tsun;
        let Sint = (S - S * Ts) / max(m.ext, vec3<f32>(1e-7));
        let Fint = (sc - sc * Ts) / max(m.ext, vec3<f32>(1e-7));
        L += T * Sint;
        F += T * Fint;
        T *= Ts;
    }
    if (ground) {
        let p = o + d * tmax;
        let up = normalize(p);
        let mus = dot(up, sun);
        L += T * trans_at(RG + 0.001, mus) * max(mus, 0.0) * GROUND_ALBEDO / PI;
    }
    ms_l[li] = L / 64.0;
    ms_f[li] = F / 64.0;
    workgroupBarrier();
    for (var st = 32u; st > 0u; st = st >> 1u) {
        if (li < st) {
            ms_l[li] += ms_l[li + st];
            ms_f[li] += ms_f[li + st];
        }
        workgroupBarrier();
    }
    if (li == 0u) {
        let psi = ms_l[0] / (1.0 - min(ms_f[0], vec3<f32>(0.99)));
        textureStore(ms_out, vec2<i32>(wg.xy), vec4<f32>(psi, 1.0));
    }
}

// ------------------------------------------------------------------ single ray in-scatter

struct Scat {
    L: vec3<f32>,
    T: vec3<f32>,
};

// In-scatter along a ray from planet-frame point o, direction d, length tmax (km),
// per unit sun irradiance. `mirror`: heights are |y| of the local frame (water reflection).
fn integrate(o: vec3<f32>, d: vec3<f32>, tmax: f32, n: i32, sun: vec3<f32>, quad: bool) -> Scat {
    return integrate_city(o, d, tmax, n, sun, quad, 0.0);
}

// `city`: isotropic in-scatter of the lit city's upward light (radiance /
// sun_e) added to every sample, fading with height — the orange haze over a
// city at night (the aerial-perspective volume uses it; the sky-view LUT gets
// the skyglow table instead).
fn integrate_city(o: vec3<f32>, d: vec3<f32>, tmax: f32, n: i32, sun: vec3<f32>, quad: bool, city: f32) -> Scat {
    var T = vec3<f32>(1.0);
    var L = vec3<f32>(0.0);
    let c = dot(d, sun);
    let pr = rayleigh_phase(c);
    let pm = mie_phase(c, 0.78);
    var tprev = 0.0;
    // Under a cloud deck the air is lit by the grey deck, not the sun.
    let cover = A.sun.w;
    let cb = max(A.params.y, 100.0) * 0.001;
    let r_cb = RG + cb;
    let deck_l = trans_at(r_cb, sun.y) * max(sun.y, 0.0) * 0.32 / PI * A.fog.z + vec3<f32>(0.002) * smoothstep(-0.1, 0.1, sun.y);
    for (var k = 0; k < n; k++) {
        var f = (f32(k) + 0.5) / f32(n);
        var f1 = (f32(k) + 1.0) / f32(n);
        if (quad) {
            f = f * f;
            f1 = f1 * f1;
        }
        let t = tmax * f;
        let tn = tmax * f1;
        let dt = tn - tprev;
        tprev = tn;
        var p = o + d * t;
        if (A.sun_e.w > 0.5) {
            p.y = RG + abs(p.y - RG);
        }
        let rp = length(p);
        let up = p / rp;
        let mus = dot(up, sun);
        // Ground fog is applied per pixel by the fog pass (gfx.rs), not here.
        let m = medium(rp - RG, haze(), vec2<f32>(0.0));
        let below = 1.0 - smoothstep(cb, cb + 0.3, rp - RG);
        let shade = 1.0 - 0.92 * cover * cover * below;
        let tsun = trans_at(rp, mus) * shade;
        var iso = deck_l * (cover * below * 0.5);
        if (city > 0.0) {
            // Ground carpet irradiance pi*L below, scattered isotropically (1/4pi), over a finite city.
            let hk = (rp - RG) / 2.5;
            iso += A.glow_tint.rgb * (city * 0.25 / (1.0 + hk * hk));
        }
        let S = (m.ray * pr + vec3<f32>(m.mie * pm)) * tsun + (m.ray + vec3<f32>(m.mie)) * (ms_at(rp, mus) * mix(1.0, 0.5, cover * below) + iso);
        let Ts = exp(-m.ext * dt);
        L += T * (S - S * Ts) / max(m.ext, vec3<f32>(1e-7));
        T *= Ts;
    }
    var s: Scat;
    s.L = L;
    s.T = T;
    return s;
}

// ------------------------------------------------------------------ night sky

// Light-pollution skyglow from the table (radiance / sun_e).
fn skyglow(d: vec3<f32>) -> f32 {
    let el = asin(clamp(d.y, -1.0, 1.0));
    // Elevation bins at 90 deg * (i / 7)^2.
    let fe = sqrt(clamp(el / (0.5 * PI), 0.0, 1.0)) * 7.0;
    let e0 = min(i32(fe), 6);
    let te = fe - f32(e0);
    let fa = skyview_az(d) * 16.0 - 0.5;
    let a0 = i32(floor(fa));
    let ta = fa - f32(a0);
    let a1 = (a0 + 1 + 16) % 16;
    let a0w = (a0 + 16) % 16;
    let i00 = e0 * 16 + a0w;
    let i01 = e0 * 16 + a1;
    let i10 = (e0 + 1) * 16 + a0w;
    let i11 = (e0 + 1) * 16 + a1;
    let g00 = A.glow[i00 / 4][i00 % 4];
    let g01 = A.glow[i01 / 4][i01 % 4];
    let g10 = A.glow[i10 / 4][i10 % 4];
    let g11 = A.glow[i11 / 4][i11 % 4];
    var g = mix(mix(g00, g01, ta), mix(g10, g11, ta), te);
    // Below the horizon the glow fades into the distant (dark) ground.
    g *= smoothstep(-0.08, 0.0, d.y);
    return g;
}

// Natural night sky (airglow + unresolved starlight, brighter toward the
// horizon: van Rhijn) plus light-pollution skyglow, per unit sun irradiance.
fn night_sky(d: vec3<f32>) -> vec3<f32> {
    let z = clamp(d.y, 0.0, 1.0);
    let vr = 1.0 / sqrt(max(1.0 - 0.96 * (1.0 - z * z), 0.05));
    let air = A.night.x * vr * smoothstep(-0.05, 0.02, d.y);
    return vec3<f32>(0.75, 0.85, 1.0) * air + A.glow_tint.rgb * skyglow(d);
}

// ------------------------------------------------------------------ sky-view LUT

@compute @workgroup_size(8, 8)
fn cs_skyview(@builtin(global_invocation_id) id: vec3<u32>) {
    if (f32(id.x) >= SKY_W || f32(id.y) >= SKY_H) {
        return;
    }
    let uv0 = (vec2<f32>(id.xy) + 0.5) / vec2<f32>(SKY_W, SKY_H);
    let uv = vec2<f32>((uv0.x - 0.5 / SKY_W) / (1.0 - 1.0 / SKY_W), (uv0.y - 0.5 / SKY_H) / (1.0 - 1.0 / SKY_H));
    let h = max(A.eye.y * 0.001, 0.002);
    let r = RG + h;
    let vh = sqrt(r * r - RG * RG);
    let beta = acos(clamp(vh / r, -1.0, 1.0));
    let zh = PI - beta;
    var vz: f32;
    if (uv.y < 0.5) {
        var c = 2.0 * uv.y;
        c = 1.0 - c;
        c = c * c;
        c = 1.0 - c;
        vz = zh * c;
    } else {
        var c = uv.y * 2.0 - 1.0;
        c = c * c;
        vz = zh + beta * c;
    }
    // Absolute azimuth (clockwise from north = -z), texel centres on the full circle.
    let az = uv0.x * 2.0 * PI;
    let sun = normalize(A.sun.xyz);
    let d = vec3<f32>(sin(vz) * sin(az), cos(vz), -sin(vz) * cos(az));
    let o = vec3<f32>(0.0, r, 0.0);
    var tmax = ray_sphere(r, d.y, RT);
    let ground = hits_ground(r, d.y);
    if (ground) {
        let disc = r * r * (d.y * d.y - 1.0) + RG * RG;
        tmax = -r * d.y - sqrt(max(disc, 0.0));
    }
    let s = integrate(o, d, max(tmax, 0.0), 32, sun, true);
    var L = s.L;
    if (ground) {
        let p = o + d * tmax;
        let up = normalize(p);
        let mus = dot(up, sun);
        // Distant ground beyond the streamed scenery: lit by sun and a sky term.
        let e = trans_at(RG + 0.001, mus) * max(mus, 0.0) + ms_at(RG + 0.001, mus) * 2.0 + vec3<f32>(0.04, 0.05, 0.07) * smoothstep(-0.1, 0.3, mus);
        L += s.T * e * GROUND_ALBEDO / PI;
    }
    // Moonlit sky: the same scattering, lit by the moon (only worth it at night).
    let mk = A.moon.w;
    if (mk > 0.0 && A.sun.y < 0.05) {
        let moon = normalize(A.moon.xyz);
        let sm = integrate(o, d, max(tmax, 0.0), 20, moon, true);
        L += sm.L * mk;
        if (ground) {
            let p = o + d * tmax;
            let mus = dot(normalize(p), moon);
            let e = trans_at(RG + 0.001, mus) * max(mus, 0.0) + ms_at(RG + 0.001, mus) * 2.0;
            L += sm.T * e * GROUND_ALBEDO / PI * mk;
        }
    }
    L += night_sky(d) * select(vec3<f32>(1.0), s.T, ground);
    textureStore(sky_out, vec2<i32>(id.xy), vec4<f32>(L * A.params.z, 1.0));
}

// ------------------------------------------------------------------ aerial perspective

@compute @workgroup_size(4, 4, 4)
fn cs_aerial(@builtin(global_invocation_id) id: vec3<u32>) {
    let uv = (vec2<f32>(id.xy) + 0.5) / AP_N;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let wp = A.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    let d = normalize(wp.xyz / wp.w); // eye-relative inverse
    let dist = ap_slice_dist((f32(id.z) + 0.5) / AP_N);
    let o = vec3<f32>(0.0, RG + A.eye.y * 0.001, 0.0);
    let n = 6 + i32(id.z) / 2;
    let s = integrate_city(o, d, dist, n, normalize(A.sun.xyz), false, A.night.y);
    var L = s.L;
    let mk = A.moon.w;
    if (mk > 0.0 && A.sun.y < 0.05) {
        L += integrate(o, d, dist, max(n / 2, 4), normalize(A.moon.xyz), false).L * mk;
    }
    textureStore(ap_out, vec3<i32>(id), vec4<f32>(L * A.params.z, dot(s.T, vec3<f32>(1.0 / 3.0))));
}

// ------------------------------------------------------------------ ambient SH + exposure

var<workgroup> sh_acc: array<array<vec3<f32>, 4>, 64>;

fn sky_lut(d: vec3<f32>) -> vec3<f32> {
    let h = max(A.eye.y * 0.001, 0.002);
    return textureSampleLevel(sky_tex, lin, skyview_uv(RG + h, d, A.sun.xyz), 0.0).rgb / A.params.z;
}

@compute @workgroup_size(64)
fn cs_ambient(@builtin(local_invocation_index) li: u32) {
    let sun = normalize(A.sun.xyz);
    let cover = A.sun.w;
    let r0 = RG + 0.01;
    let tsun0 = trans_at(r0, sun.y);
    // A cloud deck acts as a grey diffuser lit from above.
    // (+ a share of the clear-sky light so it fades properly through dusk)
    let zen = sky_lut(vec3<f32>(0.0, 1.0, 0.0));
    // fog.z: how much light a thick deck lets through (1 = typical stratocumulus).
    let moon = normalize(A.moon.xyz);
    let tmoon0 = trans_at(r0, moon.y) * A.moon.w;
    // The deck is lit from above (sun, moon) and from below by the city.
    let deck = (tsun0 * max(sun.y, 0.0) * 0.32 / PI + tmoon0 * max(moon.y, 0.0) * 0.32 / PI + vec3<f32>(luma(zen)) * 0.8) * A.fog.z
        + A.glow_tint.rgb * (A.night.y * 0.55);
    var acc = array<vec3<f32>, 4>(vec3<f32>(0.0), vec3<f32>(0.0), vec3<f32>(0.0), vec3<f32>(0.0));
    for (var j = 0u; j < 4u; j++) {
        let i = f32(li * 4u + j);
        let z = 1.0 - 2.0 * (i + 0.5) / 256.0;
        let a = i * 2.39996323;
        let s = sqrt(max(1.0 - z * z, 0.0));
        // y-up sphere
        let d = vec3<f32>(s * cos(a), z, s * sin(a));
        var L = sky_lut(d);
        if (d.y > 0.0) {
            let cl = cover * smoothstep(-0.02, 0.12, d.y);
            L = mix(L, deck * luma(L) / max(luma(L), 1e-6) * 0.0 + deck, cl);
        } else {
            // Ground bounce: under cloud the ground is dimmer; the lit city glows.
            L *= 1.0 - 0.6 * cover;
            L += A.glow_tint.rgb * A.night.y * smoothstep(0.0, -0.1, d.y);
        }
        let w = 4.0 * PI / 256.0;
        acc[0] += L * w * 0.282095;
        acc[1] += L * w * 0.488603 * d.x;
        acc[2] += L * w * 0.488603 * d.y;
        acc[3] += L * w * 0.488603 * d.z;
    }
    sh_acc[li] = acc;
    workgroupBarrier();
    for (var st = 32u; st > 0u; st = st >> 1u) {
        if (li < st) {
            for (var k = 0; k < 4; k++) {
                sh_acc[li][k] += sh_acc[li + st][k];
            }
        }
        workgroupBarrier();
    }
    if (li == 0u) {
        let e = A.sun_e.rgb;
        let c0 = PI * 0.282095;
        let c1 = 2.0 * PI / 3.0 * 0.488603;
        let s0 = sh_acc[0][0] * c0 * e;
        let sx = sh_acc[0][1] * c1 * e;
        let sy = sh_acc[0][2] * c1 * e;
        let sz = sh_acc[0][3] * c1 * e;
        outb.sh[0] = vec4<f32>(s0, 0.0);
        outb.sh[1] = vec4<f32>(sx, 0.0);
        outb.sh[2] = vec4<f32>(sy, 0.0);
        outb.sh[3] = vec4<f32>(sz, 0.0);
        let r_eye = RG + max(A.eye.y * 0.001, 0.002);
        let sun_eye = trans_at(r_eye, sun.y) * e;
        outb.sun_eye = vec4<f32>(sun_eye, 0.0);
        // Incident-light metering: sun on a horizontal surface (mostly hidden
        // under heavy cloud) + sky irradiance from above.
        let e_up = max(s0 + sy, vec3<f32>(0.0));
        let e_sun = trans_at(r0, sun.y) * e * max(sun.y, 0.0) * (1.0 - 0.92 * cover);
        let e_moon = tmoon0 * e * max(moon.y, 0.0) * (1.0 - 0.92 * cover);
        // Artificial light the eye adapts to (street-lit city in view).
        let metered = luma(e_up + e_sun + e_moon) / A.night.w + A.night.z;
        // Exposure key: compress the range so dusk is darker than noon (but
        // still readable) — a camera with partial auto-exposure.
        // k maps a grey card under the metered light to `key` x albedo, but
        // only compensates 82% of the change from the reference illuminance.
        let ref_e = A.params.x;
        // Partial adaptation: a grey card under the metered light maps to
        // key * (m / ref)^0.3 — dusk darker than noon. Below dusk the eye keeps
        // adapting (scotopic vision) a little faster so moonlit ground stays
        // readable; the tonemapper adds the Purkinje shift.
        let m = max(metered, ref_e * 2.0e-8);
        let dusk = ref_e * 0.004;
        var k = A.params.w * PI / m * pow(m / ref_e, 0.3);
        if (m < dusk) {
            k *= pow(dusk / m, 0.25);
        }
        // (The HDR image is pre-exposed by night.w.)
        outb.expo = vec4<f32>(k / A.night.w, metered, 0.0, 0.0);
    }
}
