// Physically based atmosphere (Hillaire 2020): shared constants and LUT
// parameterisations. Prepended to atmosphere.wgsl (LUT builders), the scene
// shaders and clouds.wgsl. Distances in km, planet centre at (0, -RG, 0) of a
// local frame whose origin is on the ground under the camera; world +Y = up.

const PI: f32 = 3.14159265;
const RG: f32 = 6360.0;            // ground radius (km)
const RT: f32 = 6460.0;            // top of the atmosphere (km)
const RAY_S: vec3<f32> = vec3<f32>(5.802e-3, 13.558e-3, 33.1e-3); // Rayleigh scattering /km
const RAY_H: f32 = 8.0;
const MIE_S: f32 = 3.996e-3;       // Mie scattering /km (clear air, scaled by haze)
const MIE_A: f32 = 0.444e-3;       // Mie absorption /km
const MIE_H: f32 = 1.2;
const OZONE_A: vec3<f32> = vec3<f32>(0.650e-3, 1.881e-3, 0.085e-3);
const GROUND_ALBEDO: vec3<f32> = vec3<f32>(0.11, 0.12, 0.09);

const TRANS_W: f32 = 256.0;
const TRANS_H: f32 = 64.0;
const MS_N: f32 = 32.0;
const SKY_W: f32 = 256.0;
const SKY_H: f32 = 108.0;
const AP_N: f32 = 32.0;
const AP_MAX_KM: f32 = 64.0;       // aerial perspective volume depth

struct Medium {
    ray: vec3<f32>,      // Rayleigh scattering
    mie: f32,            // Mie scattering
    ext: vec3<f32>,      // total extinction
};

// `haze` multiplies the boundary-layer Mie density (1 = clear-air reference).
// `fog`: x = extra extinction near the ground (/km: fog, mist, precipitation),
// y = top of that layer (km).
fn medium(h_km: f32, haze: f32, fog: vec2<f32>) -> Medium {
    let h = max(h_km, 0.0);
    let dr = exp(-h / RAY_H);
    // Boundary-layer aerosols: a well-mixed layer with a soft top plus the
    // reference exponential profile above it.
    let bl = haze * exp(-h / MIE_H) * (1.0 + 2.0 * (1.0 - smoothstep(0.6, 2.2, h)));
    let dm = exp(-h / MIE_H) + max(bl - exp(-h / MIE_H), 0.0);
    let doz = max(0.0, 1.0 - abs(h - 25.0) / 15.0);
    var m: Medium;
    m.ray = RAY_S * dr;
    m.mie = MIE_S * dm;
    m.ext = m.ray + vec3<f32>((MIE_S + MIE_A) * dm) + OZONE_A * doz;
    // Fog / mist: water droplets scatter almost without absorption.
    let top = max(fog.y, 1e-3);
    let f = fog.x * (1.0 - smoothstep(top * 0.55, top, h));
    m.mie += f * 0.97;
    m.ext += vec3<f32>(f);
    return m;
}

fn ray_sphere(r: f32, mu: f32, R: f32) -> f32 {
    // Distance along a ray from radius r with cos(zenith) mu to sphere R (far hit), -1 if none.
    let disc = r * r * (mu * mu - 1.0) + R * R;
    if (disc < 0.0) {
        return -1.0;
    }
    return -r * mu + sqrt(disc);
}

fn hits_ground(r: f32, mu: f32) -> bool {
    return mu < 0.0 && r * r * (mu * mu - 1.0) + RG * RG >= 0.0;
}

fn trans_uv(r: f32, mu: f32) -> vec2<f32> {
    let H = sqrt(RT * RT - RG * RG);
    let rho = sqrt(max(r * r - RG * RG, 0.0));
    let disc = r * r * (mu * mu - 1.0) + RT * RT;
    let d = max(0.0, -r * mu + sqrt(max(disc, 0.0)));
    let dmin = RT - r;
    let dmax = rho + H;
    let xm = (d - dmin) / max(dmax - dmin, 1e-4);
    let xr = rho / H;
    // Half-texel remap so the LUT edges are sampled at texel centres.
    return vec2<f32>(0.5 / TRANS_W + xm * (1.0 - 1.0 / TRANS_W), 0.5 / TRANS_H + xr * (1.0 - 1.0 / TRANS_H));
}

fn trans_rmu(uv0: vec2<f32>) -> vec2<f32> {
    let uv = vec2<f32>((uv0.x - 0.5 / TRANS_W) / (1.0 - 1.0 / TRANS_W), (uv0.y - 0.5 / TRANS_H) / (1.0 - 1.0 / TRANS_H));
    let H = sqrt(RT * RT - RG * RG);
    let rho = H * clamp(uv.y, 0.0, 1.0);
    let r = sqrt(rho * rho + RG * RG);
    let dmin = RT - r;
    let dmax = rho + H;
    let d = dmin + clamp(uv.x, 0.0, 1.0) * (dmax - dmin);
    var mu = 1.0;
    if (d > 1e-4) {
        mu = (H * H - rho * rho - d * d) / (2.0 * r * d);
    }
    return vec2<f32>(r, clamp(mu, -1.0, 1.0));
}

fn rayleigh_phase(c: f32) -> f32 {
    return 3.0 / (16.0 * PI) * (1.0 + c * c);
}

// Cornette-Shanks Mie phase.
fn mie_phase(c: f32, g: f32) -> f32 {
    let g2 = g * g;
    let k = 3.0 / (8.0 * PI) * (1.0 - g2) / (2.0 + g2);
    return k * (1.0 + c * c) / pow(max(1.0 + g2 - 2.0 * g * c, 1e-4), 1.5);
}

// Sky-view LUT mapping (Hillaire): x = absolute azimuth (clockwise from
// north, so the sun, the moon and light-pollution skyglow can all live in one
// LUT), y = zenith angle, split at the horizon with extra resolution near it.
// (`sun` is unused; kept for the callers' signature.)
fn skyview_az(d: vec3<f32>) -> f32 {
    // 0..1 from north through east.
    return fract(atan2(d.x, -d.z) / (2.0 * PI) + 1.0);
}

fn skyview_uv(r: f32, d: vec3<f32>, sun: vec3<f32>) -> vec2<f32> {
    let vh = sqrt(max(r * r - RG * RG, 0.0));
    let cos_beta = vh / r;
    let beta = acos(clamp(cos_beta, -1.0, 1.0));
    let zh = PI - beta;
    let vz = acos(clamp(d.y, -1.0, 1.0));
    var v: f32;
    if (vz < zh) {
        var c = vz / zh;
        c = 1.0 - c;
        c = sqrt(max(c, 0.0));
        c = 1.0 - c;
        v = c * 0.5;
    } else {
        var c = (vz - zh) / max(beta, 1e-4);
        c = sqrt(clamp(c, 0.0, 1.0));
        v = c * 0.5 + 0.5;
    }
    // Texel centres at (i + 0.5) / SKY_W cover the full circle; the seam at
    // north is within half a texel (ClampToEdge there).
    let u = skyview_az(d);
    v = 0.5 / SKY_H + v * (1.0 - 1.0 / SKY_H);
    return vec2<f32>(u, v);
}

fn ap_slice_dist(s: f32) -> f32 {
    // slice coordinate 0..1 -> km (quadratic: more resolution near the eye)
    return s * s * AP_MAX_KM;
}

fn ap_slice_of(dist_km: f32) -> f32 {
    return sqrt(clamp(dist_km / AP_MAX_KM, 0.0, 1.0));
}

// AgX-ish luminance weights.
fn luma(c: vec3<f32>) -> f32 {
    return dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
}
