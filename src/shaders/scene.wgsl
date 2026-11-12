// Main lit geometry: ground layers, buildings, trees, aircraft. GGX + Lambert
// lighting from the physically based sun/sky (common.wgsl), aerial
// perspective from the froxel volume. Writes HDR colour + eye distance (m).

struct VIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) col: vec3<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) mat: f32,
};

struct VOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wpos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) col: vec3<f32>,
    @location(3) uv: vec2<f32>,
    @location(4) @interpolate(flat) mat: f32,
    // Trees: x = this tree's leaf cover 0..1 (1 = evergreen / not a tree),
    // y = conifer flag, z = per-tree seed.
    @location(5) @interpolate(flat) tree: vec3<f32>,
};

struct FOut {
    @location(0) col: vec4<f32>,
    @location(1) dist: f32,
};

@vertex
fn vs_main(v: VIn) -> VOut {
    var o: VOut;
    let p = tile_pos(v.pos);
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = v.nrm;
    o.col = v.col;
    o.uv = v.uv;
    o.mat = v.mat;
    o.tree = vec3<f32>(1.0, 0.0, 0.0);
    return o;
}

// ---------------------------------------------------------------- season

// Leaf cover of one deciduous tree (0 bare .. 1 full): trees drop / gain
// their leaves at slightly different times.
fn tree_leaf(seed: f32) -> f32 {
    let leaf = g.season.x;
    let k = seed * 0.75;
    return smoothstep(k, k + 0.3, leaf * 1.05);
}

// Seasonal foliage tint (sRGB-ish, like the instance tint) for a deciduous tree.
fn season_tint(base: vec3<f32>, seed: f32) -> vec3<f32> {
    var c = base;
    // Spring: fresh, light, yellowish green.
    c = mix(c, vec3<f32>(0.30, 0.40, 0.12), g.season.z * 0.6 * (0.5 + 0.5 * fract(seed * 7.3)));
    // Late summer: darker, dustier.
    c = mix(c, c * vec3<f32>(0.95, 0.92, 0.8), g.season2.x);
    // Autumn: each tree turns at its own pace, to its own colour.
    let r = fract(seed * 13.7);
    let turn = clamp(g.season.y * 1.35 - fract(seed * 5.1) * 0.45, 0.0, 1.0);
    var au = vec3<f32>(0.44, 0.36, 0.07);              // yellow (birch, lime, field maple)
    if (r > 0.3) { au = vec3<f32>(0.46, 0.24, 0.05); }  // orange (beech, maple)
    if (r > 0.62) { au = vec3<f32>(0.32, 0.18, 0.07); } // russet brown (oak)
    if (r > 0.85) { au = vec3<f32>(0.40, 0.12, 0.06); } // red (cherry, sweetgum)
    // Oaks / planes hang on to dull greens and browns longer.
    c = mix(c, au, turn);
    return c;
}

struct TIn {
    @location(5) pos_h: vec4<f32>,   // xyz base, w height (m)
    @location(6) tint_rot: vec4<f32>, // rgb foliage tint (sRGB), w yaw (rad)
};

// Instanced unit-height trees (streamed per tile).
@vertex
fn vs_tree(v: VIn, t: TIn) -> VOut {
    let c = cos(t.tint_rot.w);
    let s = sin(t.tint_rot.w);
    let h = t.pos_h.w;
    // Per-tree crown width variation.
    let wk = 0.8 + 0.45 * fract(sin(dot(t.pos_h.xz, vec2<f32>(12.9898, 78.233))) * 43758.5453);
    let lp = v.pos * vec3<f32>(h * wk, h, h * wk);
    // Seeds use the tile-local instance position: stable across origin shifts.
    let p = tile_pos(t.pos_h.xyz) + vec3<f32>(c * lp.x - s * lp.z, lp.y, s * lp.x + c * lp.z);
    var o: VOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = vec3<f32>(c * v.nrm.x - s * v.nrm.z, v.nrm.y, s * v.nrm.x + c * v.nrm.z);
    let foliage = v.mat > 10.5 && v.mat < 11.5;
    let seed = fract(t.pos_h.x * 0.137 + t.pos_h.z * 0.071);
    let conifer = v.uv.x > 0.5;
    var tint = t.tint_rot.xyz;
    var leaf = 1.0;
    if (foliage && !conifer && !is_refl()) {
        tint = season_tint(tint, seed);
    }
    if (foliage && !conifer) {
        leaf = tree_leaf(seed);
    }
    o.col = select(v.col, v.col * tint, foliage);
    // Foliage: uv = (height within the tree 0..1, per-tree seed).
    o.uv = select(v.uv, vec2<f32>(v.pos.y, seed), foliage);
    o.mat = v.mat;
    o.tree = vec3<f32>(leaf, select(0.0, 1.0, conifer), seed);
    return o;
}

// ---------------------------------------------------------------- BRDF

fn d_ggx(nh: f32, a: f32) -> f32 {
    let a2 = a * a;
    let d = nh * nh * (a2 - 1.0) + 1.0;
    return a2 / (PI * d * d);
}

fn v_smith(nv: f32, nl: f32, a: f32) -> f32 {
    let a2 = a * a;
    let gv = nl * sqrt(nv * nv * (1.0 - a2) + a2);
    let gl = nv * sqrt(nl * nl * (1.0 - a2) + a2);
    return 0.5 / max(gv + gl, 1e-5);
}

fn f_schlick(f0: vec3<f32>, vh: f32) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - vh, 5.0);
}

// Split-sum environment BRDF approximation (Karis, mobile).
fn env_brdf(f0: vec3<f32>, rough: f32, nv: f32) -> vec3<f32> {
    let c0 = vec4<f32>(-1.0, -0.0275, -0.572, 0.022);
    let c1 = vec4<f32>(1.0, 0.0425, 1.04, -0.04);
    let r = rough * c0 + c1;
    let a004 = min(r.x * r.x, exp2(-9.28 * nv)) * r.x + r.y;
    let ab = vec2<f32>(-1.04, 1.04) * a004 + r.zw;
    return f0 * ab.x + ab.y;
}

struct Surf {
    albedo: vec3<f32>,
    n: vec3<f32>,
    rough: f32,
    f0: vec3<f32>,
    ao: f32,
    emit: vec3<f32>,
    alpha: f32,
    transl: f32,    // foliage translucency
    env: f32,       // environment (sky) reflection weight
    moving: f32,    // 1 = the aircraft: no reprojected (last-frame) SSAO
};

fn default_surf(albedo: vec3<f32>, n: vec3<f32>) -> Surf {
    var s: Surf;
    s.albedo = albedo;
    s.n = n;
    s.rough = 0.85;
    s.f0 = vec3<f32>(0.03);
    s.ao = 1.0;
    s.emit = vec3<f32>(0.0);
    s.alpha = 1.0;
    s.transl = 0.0;
    s.env = 1.0;
    s.moving = 0.0;
    return s;
}

// Lighting at one surface point. `rough_aa` already includes specular AA.
// `rel`: eye-relative position (shadow / AO matrices are eye-relative too).
fn light(s: Surf, rel: vec3<f32>, V: vec3<f32>, dist: f32, px: vec2<f32>, ngeo: vec3<f32>) -> vec3<f32> {
    let L = g.sun_dir.xyz;
    let n = s.n;
    let nl = dot(n, L);
    let nv = max(dot(n, V), 1e-4);
    let sh = shadow_factor(rel, ngeo, dist);
    let E = sun_irradiance(rel + g.cam_pos.xyz) * sh;
    let a = max(s.rough * s.rough, 0.002);
    // Last frame's AO is reprojected for static geometry; on the moving
    // aircraft it would land on whatever was behind it (flicker / "shake").
    let ao = s.ao * select(ssao(rel), 1.0, s.moving > 0.5);
    var col = vec3<f32>(0.0);
    if (nl > 0.0) {
        let H = normalize(L + V);
        let nh = max(dot(n, H), 0.0);
        let vh = max(dot(V, H), 0.0);
        let F = f_schlick(s.f0, vh);
        let spec = d_ggx(nh, a) * v_smith(nv, nl, a) * F;
        col += (s.albedo / PI * (vec3<f32>(1.0) - F) + spec) * E * nl;
    }
    // Foliage: light through the leaves when the sun is behind them.
    if (s.transl > 0.0) {
        let back = pow(clamp(dot(-V, L), 0.0, 1.0), 3.0);
        let wrap = clamp(-nl * 0.6 + 0.4, 0.0, 1.0);
        col += s.albedo * vec3<f32>(1.0, 1.08, 0.6) * E * s.transl * (0.12 * wrap + 0.55 * back) / PI;
    }
    // Ambient: SH sky irradiance (+ ground bounce), specular from the sky-view.
    col += s.albedo / PI * sky_irradiance(n) * ao;
    let R = reflect(-V, n);
    let fe = env_brdf(s.f0, s.rough, nv);
    let sharp = sky_reflection(normalize(vec3<f32>(R.x, max(R.y, -0.3), R.z)));
    let blurry = sky_irradiance(R) / PI;
    let env = mix(sharp, blurry, smoothstep(0.15, 0.6, s.rough));
    // Horizon occlusion: reflections pointing into the ground see the ground.
    let hocc = clamp(1.0 + 1.6 * dot(R, ngeo), 0.0, 1.0);
    col += env * fe * s.env * ao * hocc * hocc;
    // Street lights (ground light map).
    col += s.albedo / PI * street_light(rel, n) * mix(1.0, ao, 0.5);
    // The aircraft's landing / taxi lights, strobes and beacon (no shadows).
    let na = i32(g.misc2.y);
    for (var i = 0; i < na; i++) {
        let al = aircraft_light(i, rel);
        let anl = dot(n, al.l);
        if (anl > 0.0 && max(al.e.r, max(al.e.g, al.e.b)) > 0.0) {
            let H = normalize(al.l + V);
            let F = f_schlick(s.f0, max(dot(V, H), 0.0));
            let spec = d_ggx(max(dot(n, H), 0.0), a) * v_smith(nv, anl, a) * F;
            col += (s.albedo / PI * (vec3<f32>(1.0) - F) + spec) * al.e * anl;
        }
    }
    col += pre_exposed(s.emit);
    return col;
}

fn spec_aa(rough: f32, dndx: vec3<f32>, dndy: vec3<f32>) -> f32 {
    let variance = 0.25 * (dot(dndx, dndx) + dot(dndy, dndy));
    let k = min(2.0 * variance, 0.18);
    let a = rough * rough;
    return sqrt(sqrt(clamp(a * a + k, 0.0, 1.0)));
}

// ---------------------------------------------------------------- weather on surfaces

// Rain-drop rings on standing water: normal xz offset.
// `np` pattern space, `f` cells per metre.
fn rain_ripples(np: vec2<f32>, f: f32, t: f32, fade: f32) -> vec2<f32> {
    let rain = g.wx.y;
    if (rain <= 0.0 || fade > 0.25) {
        return vec2<f32>(0.0);
    }
    let per = nper(f);
    var gsum = vec2<f32>(0.0);
    for (var k = 0; k < 2; k++) {
        let q = np * (per / NP) + vec2<f32>(f32(k) * 17.3, f32(k) * 5.1);
        let cell = floor(q);
        let wc = wrapc(cell, per);
        let h = hash22(wc + 3.7);
        let ph = fract(t * (0.9 + 0.4 * h.x) + h.y);
        let c = cell + 0.25 + 0.5 * h;
        let d = q - c;
        let r = length(d);
        let ring = ph * 0.6;
        let w = sin((r - ring) * 40.0) * exp(-abs(r - ring) * 25.0) * (1.0 - ph) * step(hash12(wc + f32(k)), rain * 1.2);
        gsum += normalize(d + vec2<f32>(1e-4)) * w;
    }
    return gsum * 0.35 * (1.0 - fade * 4.0);
}

// Reflection of the scene in wet, rough paving: the planar reflection blurred
// along the screen's vertical (anisotropic: the highlight stretches toward
// the viewer), weighted by the Fresnel term of a water film.
fn wet_streaks(px: vec2<f32>, n: vec3<f32>, V: vec3<f32>, dist: f32) -> vec3<f32> {
    let nv = max(dot(n, V), 1e-3);
    let fres = 0.02 + 0.98 * pow(1.0 - nv, 5.0);
    let suv = px * g.screen.zw;
    // Streak length in uv: longer at grazing angles, shorter far away.
    let len = 0.07 * (0.35 + 0.65 * (1.0 - nv)) / (1.0 + dist * 0.0015);
    var acc = vec3<f32>(0.0);
    var wsum = 0.0;
    // Jittered taps (interleaved gradient noise) so the streak is continuous.
    let jit = fract(52.9829189 * fract(dot(px, vec2<f32>(0.06711056, 0.00583715))) + g.cam_pos.w * 0.61803) - 0.5;
    for (var k = -6; k <= 6; k++) {
        let f = (f32(k) + jit) / 6.0;
        let w = exp(-f * f * 2.5);
        let uv = clamp(suv + vec2<f32>(0.0, f * len), vec2<f32>(0.001), vec2<f32>(0.999));
        acc += textureSampleLevel(refl_tex, lin_samp, uv, 0.0).rgb * w;
        wsum += w;
    }
    return acc / wsum * fres * 0.8;
}

// Puddle mask 0..1 on flat paved ground (only when wet).
fn puddle_mask(np: vec2<f32>, fw: f32) -> f32 {
    let wet = g.wx.x;
    if (wet < 0.3) {
        return 0.0;
    }
    let n = pfbm(np, 0.09, vec2<f32>(0.0)) * 0.7 + pna(np, 0.6, vec2<f32>(0.0), fw * 0.6) * 0.3;
    let thr = mix(0.78, 0.6, smoothstep(0.3, 1.0, wet));
    return smoothstep(thr, thr + 0.05, n);
}

// Wet, puddled paving: darker, glossier, with mirror-like puddles that use
// the planar reflection (the ground is at y = 0, exactly the mirror plane).
fn wet_paving(s0: Surf, wp: vec3<f32>, px: vec2<f32>, fw: f32, porous: f32) -> Surf {
    var s = s0;
    let wet = g.wx.x;
    if (wet <= 0.0) {
        return s;
    }
    s.albedo *= mix(1.0, mix(0.62, 0.45, porous), wet);
    s.rough = mix(s.rough, 0.22, wet * 0.85);
    s.f0 = mix(s.f0, vec3<f32>(0.03), wet);
    return s;
}

// Snow / frost on up-facing surfaces. `hold`: how well snow settles (1 grass, 0.4 roads).
fn snowy(s0: Surf, np: vec2<f32>, n: vec3<f32>, hold: f32, fw: f32) -> Surf {
    var s = s0;
    let up = smoothstep(0.55, 0.9, n.y);
    let snow = g.wx.w * hold;
    if (snow > 0.0) {
        let patchy = pna(np, 0.35, vec2<f32>(0.0), fw * 0.35) * 0.5 + pna(np, 0.05, vec2<f32>(0.0), fw * 0.05) * 0.5;
        let cover = clamp(snow * 1.6 - (1.0 - patchy) * 0.8, 0.0, 1.0) * up;
        s.albedo = mix(s.albedo, vec3<f32>(0.80, 0.82, 0.86), cover);
        s.rough = mix(s.rough, 0.7, cover);
    }
    let frost = g.wx2.z * hold;
    if (frost > 0.0) {
        s.albedo = mix(s.albedo, vec3<f32>(0.55, 0.58, 0.62), frost * up * 0.55 * (0.7 + 0.3 * pna(np, 0.8, vec2<f32>(0.0), fw * 0.8)));
    }
    return s;
}

// ---------------------------------------------------------------- water

fn wave_grad(p: vec2<f32>, t: f32, fade: f32) -> vec2<f32> {
    // Sum of directional sine waves (analytic slope) + fine noise ripples.
    var gsum = vec2<f32>(0.0);
    // Waves run with the surface wind; their height follows its speed.
    let wv = g.wx2.xy;
    let ws = length(wv);
    var dir = select(vec2<f32>(0.80, 0.60), wv / max(ws, 1e-3), ws > 0.5);
    var amp = 0.16 * clamp(0.35 + ws / 7.0, 0.35, 2.2);
    var k = 0.11;
    var spd = 1.2;
    let rot = mat2x2<f32>(0.80, 0.60, -0.60, 0.80);
    for (var i = 0; i < 7; i++) {
        // Wave vector snapped to the pattern period (p is pattern space).
        let kv = round(dir * (k * NP / TAU)) * (TAU / NP);
        let ph = dot(kv, p) + t * spd * sqrt(9.81 * k) * 0.35 + f32(i) * 1.7;
        // Sharper crests: derivative of a peaked wave.
        let w = cos(ph) * (1.0 + 0.4 * sin(ph));
        let atten = 1.0 - smoothstep(0.5, 2.0, fade * k * 2.0);
        gsum += kv * (amp * w * atten);
        dir = rot * dir;
        dir = vec2<f32>(dir.x, dir.y * 1.1);
        dir = normalize(dir);
        k *= 1.73;
        amp *= 0.62;
        spd *= 0.9;
    }
    return gsum;
}

fn shade_water(in: VOut, px: vec2<f32>, fw: f32) -> vec3<f32> {
    let t = g.cam_pos.w;
    let rel = in.wpos;
    let wp = rel + g.cam_pos.xyz;
    let to_cam = -rel;
    let dist = length(to_cam);
    let V = to_cam / dist;
    let np = pat(rel);
    let gr = wave_grad(np, t, fw) + rain_ripples(np, 2.2, t, fw * 0.05);
    // Slope variance lost to filtering widens the highlight instead (spec AA).
    let lost = clamp(fw * 0.35, 0.0, 1.0);
    var n = normalize(vec3<f32>(-gr.x, 1.0, -gr.y));
    let nv = max(dot(n, V), 1e-3);
    let f0 = 0.02;
    let fres = f0 + (1.0 - f0) * pow(1.0 - nv, 5.0);

    var refl: vec3<f32>;
    let R = reflect(-V, n);
    if (is_refl()) {
        refl = sky_reflection(vec3<f32>(R.x, abs(R.y), R.z));
    } else {
        let suv = px * g.screen.zw + n.xz * 0.05 / (1.0 + dist * 0.002);
        refl = textureSampleLevel(refl_tex, lin_samp, clamp(suv, vec2<f32>(0.001), vec2<f32>(0.999)), 0.0).rgb;
    }
    let L = g.sun_dir.xyz;
    let sh = shadow_factor(rel, vec3<f32>(0.0, 1.0, 0.0), dist);
    let E = sun_irradiance(wp) * sh;
    // Body colour: light scattered back up from the turbid (Thames-green) water column.
    let scat = vec3<f32>(0.020, 0.026, 0.018);
    let amb = sky_irradiance(vec3<f32>(0.0, 1.0, 0.0));
    let body = scat * (E * max(L.y, 0.0) + amb) / PI * 3.0;
    var col = mix(body, refl, fres);
    // Sun glint (GGX, roughness grows with distance).
    let a = mix(0.035, 0.22, lost);
    let H = normalize(L + V);
    let nl = max(dot(n, L), 0.0);
    let spec = d_ggx(max(dot(n, H), 0.0), a) * v_smith(nv, nl, a) * f_schlick(vec3<f32>(f0), max(dot(V, H), 0.0));
    col += spec * E * nl;
    return col;
}

// ---------------------------------------------------------------- facades

// Lit rooms seen from outside (scene radiance): a living room at ~150 lux
// has ~45 cd/m² walls, behind ~75 % glass; offices are brighter inside but
// behind tinted low-e glazing; shop windows are bright.
const WIN_HOME: f32 = 24.0 * 1.953125e-5 * 0.75;
const WIN_OFFICE: f32 = 45.0 * 1.953125e-5 * 0.35;
const WIN_SHOP: f32 = 180.0 * 1.953125e-5;

// Colour of a lit window: mostly warm (incandescent / warm LED), some
// neutral, a few cool-white LED.
fn window_colour(h: f32, h2: f32) -> vec3<f32> {
    var c = vec3<f32>(1.0, 0.66, 0.36);
    if (h > 0.62) { c = vec3<f32>(1.0, 0.8, 0.58); }
    if (h > 0.88) { c = vec3<f32>(0.82, 0.9, 1.0); }
    return c * (0.85 + 0.3 * h2);
}

fn wall_surface(in: VOut, n: vec3<f32>, base_srgb: vec3<f32>, fwuv: vec2<f32>) -> Surf {
    let u = in.uv.x;
    let y = in.uv.y;
    let seed = floor(u / 4096.0);
    let r0 = hash12(vec2<f32>(seed, 3.1));
    let r1 = hash12(vec2<f32>(seed, 7.7));
    let r2 = hash12(vec2<f32>(seed, 11.3));
    let fl = 2.9 + 0.8 * r0;                 // floor height
    let bay = 2.4 + 1.8 * r1;                // window pitch
    let ww = 0.42 + 0.26 * r2;               // window width fraction
    let wh = 0.48 + 0.18 * hash12(vec2<f32>(seed, 1.9));
    let shop = (i32(seed) & 1) == 1;
    let cx = u / bay;
    let cy = y / fl;
    let fx = fract(cx);
    let fy = fract(cy);
    let cell = floor(vec2<f32>(cx, cy));
    var albedo = srgb_to_lin(base_srgb);
    // Material texture: brick-ish fine noise for warm colours, render/stone otherwise.
    let warm = base_srgb.r - base_srgb.b;
    let brick = noise_aa(vec2<f32>(u * 4.0, y * 12.0), fwuv.x * 4.0);
    albedo *= 0.9 + 0.2 * mix(noise_aa(vec2<f32>(u, y) * 1.3, fwuv.x * 1.3), brick, smoothstep(0.05, 0.2, warm));
    // Weathering: darker near the ground and in vertical streaks.
    let streak = noise_aa(vec2<f32>(u * 0.9, y * 0.04 + seed), fwuv.x * 0.9);
    albedo *= (0.82 + 0.18 * smoothstep(0.0, 4.0, y)) * (0.92 + 0.12 * streak);
    var s = default_surf(albedo, n);
    s.rough = 0.8;
    // Slab bands between floors.
    let band = smoothstep(0.0, 0.03, fy) * (1.0 - smoothstep(0.93, 0.97, fy));
    s.albedo *= mix(1.0, 0.86 + 0.25 * r0, (1.0 - band) * step(1.0, cy));
    // Windows, faded to their average where they get smaller than ~2 px.
    let lod = 1.0 - smoothstep(0.15, 0.45, max(fwuv.x / bay, fwuv.y / fl));
    let mx = (1.0 - ww) * 0.5;
    let my = 0.3;
    var win = step(mx, fx) * step(fx, 1.0 - mx) * step(my, fy) * step(fy, my + wh) * step(1.0, cy);
    var is_shop = false;
    if (shop && y < 4.2) {
        // Shopfront: tall glazing, a fascia band above.
        let sx = fract(u / 6.0);
        win = step(0.06, sx) * step(sx, 0.94) * step(0.25, y) * step(y, 3.1);
        is_shop = true;
        if (y > 3.25 && y < 4.0) {
            let fh = hash12(vec2<f32>(floor(u / 6.0), seed));
            s.albedo = srgb_to_lin(mix(vec3<f32>(0.12, 0.14, 0.18), vec3<f32>(0.55, 0.12, 0.10), step(0.5, fh))) * (0.6 + 0.6 * fh);
            s.rough = 0.45;
        }
    }
    let rw = hash12(cell + seed * 1.31);
    // Windows are recessed: at grazing angles the reveals hide the room.
    let recess = mix(0.3, 1.0, smoothstep(0.05, 0.55, abs(dot(n, normalize(-in.wpos)))));
    let night = g.sun_true.w * recess;
    // Who's home: per building, per floor, per window (stable hashes, so
    // windows go dark one by one as the evening goes on).
    // Homes follow the evening curve; a third of the buildings are offices,
    // shops or warehouses that empty after work.
    let commercial = hash12(vec2<f32>(seed, 9.1)) < 0.33;
    let bfac = (0.2 + 1.5 * hash12(vec2<f32>(seed, 21.7))) * select(g.city.z, g.city.y * 0.6, commercial);
    let ffac = 0.4 + 1.2 * hash12(vec2<f32>(seed * 3.1, cell.y + 0.5));
    let occ = clamp(bfac * ffac, 0.0, 0.95);
    let wcol = window_colour(hash12(cell * 1.7 + seed), hash12(cell + 1.1));
    // Shops: lit until late evening, a few all night.
    let hrs = g.city.w;
    let shop_on = mix(0.2, 1.0, step(14.0, hrs) * (1.0 - smoothstep(21.5, 23.0, hrs)));
    if (win > 0.5 && lod > 0.0) {
        // Glass: dark interior + reflection; blinds / curtains vary per window.
        let blind = step(0.55, rw) * step(1.0 - 0.6 * hash12(cell * 1.7 + 3.0), fract((fy - my) / wh));
        // Daylit rooms: walls / furniture seen through the glass.
        var interior = vec3<f32>(0.05, 0.048, 0.045) * (0.5 + 1.0 * rw);
        interior = mix(interior, srgb_to_lin(vec3<f32>(0.78, 0.75, 0.68)) * 0.45, blind);
        if (is_shop) {
            interior = vec3<f32>(0.03, 0.03, 0.028) + vec3<f32>(0.05, 0.04, 0.03) * hash12(cell + 9.0);
        }
        let glass = default_surf(interior, normalize(n + vec3<f32>(rw - 0.5, 0.0, hash12(cell + 4.2) - 0.5) * 0.03));
        let lit = step(hash12(cell * 2.3 + seed), occ) * night;
        // Curtains / blinds: dimmer, warmer, diffuse glow.
        let level = (0.45 + 0.9 * rw) * mix(1.0, 0.45, blind);
        // A few flickering TVs.
        let tv = step(0.94, hash12(cell * 3.7 + seed)) * step(19.0, hrs);
        let flick = 0.6 + 0.4 * sin(g.cam_pos.w * (3.0 + 5.0 * rw) + rw * 40.0) * sin(g.cam_pos.w * 7.3 + rw * 11.0);
        let wc = mix(wcol, vec3<f32>(0.45, 0.6, 1.0) * flick, tv);
        var gs = glass;
        gs.rough = 0.06;
        gs.f0 = vec3<f32>(0.06);
        gs.emit = wc * lit * level * WIN_HOME + select(vec3<f32>(0.0), vec3<f32>(1.0, 0.86, 0.66) * shop_on * night * WIN_SHOP, is_shop);
        s.albedo = mix(s.albedo, gs.albedo, lod);
        s.n = gs.n;
        s.rough = mix(s.rough, gs.rough, lod);
        s.f0 = mix(s.f0, gs.f0, lod);
        s.emit = gs.emit * lod;
    } else if (lod > 0.0) {
        // Reveal / sill shadow around the opening.
        let e = step(mx - 0.05, fx) * step(fx, 1.0 - mx + 0.05) * step(my - 0.06, fy) * step(fy, my + wh + 0.03) * step(1.0, cy);
        s.albedo *= 1.0 - 0.3 * e * lod;
        let sill = step(mx - 0.06, fx) * step(fx, 1.0 - mx + 0.06) * step(my - 0.07, fy) * step(fy, my - 0.02) * step(1.0, cy);
        s.albedo = mix(s.albedo, s.albedo * 1.35 + 0.02, sill * lod);
    }
    // Far away: average of glass + wall (lit share per floor while the
    // floors are resolved, per building beyond).
    let avg_win = ww * wh;
    let far_glass = vec3<f32>(0.05);
    s.albedo = mix(mix(s.albedo, far_glass, avg_win * 0.8), s.albedo, lod);
    let floor_lod = 1.0 - smoothstep(0.25, 0.7, fwuv.y / fl);
    let occ_far = clamp(bfac * mix(1.0, ffac, floor_lod), 0.0, 0.95) * step(1.0, cy);
    s.emit += (1.0 - lod) * night * avg_win * occ_far * 0.7 * WIN_HOME * vec3<f32>(1.0, 0.74, 0.47);
    s.f0 = mix(mix(s.f0, vec3<f32>(0.04), avg_win), s.f0, lod);
    s.ao = 0.7 + 0.3 * smoothstep(0.0, 5.0, y);
    return s;
}

fn glass_surface(in: VOut, n: vec3<f32>, tint_srgb: vec3<f32>, fwuv: vec2<f32>) -> Surf {
    let u = in.uv.x;
    let y = in.uv.y;
    let seed = floor(u / 4096.0);
    let r0 = hash12(vec2<f32>(seed, 2.3));
    let fl = 3.7 + 0.6 * r0;
    let mull = 1.35 + 0.5 * hash12(vec2<f32>(seed, 8.1));
    let spandrel = 0.18 + 0.2 * hash12(vec2<f32>(seed, 4.4));
    let fins = hash12(vec2<f32>(seed, 6.6)) < 0.3;
    let fx = fract(u / mull);
    let fy = fract(y / fl);
    let cell = floor(vec2<f32>(u / mull, y / fl));
    let tint = srgb_to_lin(tint_srgb);
    let lodx = 1.0 - smoothstep(0.15, 0.45, fwuv.x / mull);
    let lody = 1.0 - smoothstep(0.15, 0.45, fwuv.y / fl);
    let lod = min(lodx, lody);
    let r = hash12(cell);
    var s = default_surf(tint * 0.05, n);
    // Panel waviness breaks up reflections like real curtain walls.
    let wob = vec3<f32>(r - 0.5, (hash12(cell + 7.1) - 0.5) * 0.4, hash12(cell + 3.3) - 0.5) * 0.02 * lod;
    s.n = normalize(n + wob);
    s.rough = 0.05 + 0.04 * r;
    s.f0 = mix(vec3<f32>(0.14), tint * 0.3 + 0.06, 0.5);
    s.albedo = tint * (0.06 + 0.06 * r);
    let night = g.sun_true.w * mix(0.45, 1.0, smoothstep(0.05, 0.55, abs(dot(n, normalize(-in.wpos)))));
    // Offices: whole floors stay lit (cleaners, late workers), the rest go dark.
    let bfac = 0.4 + 1.2 * hash12(vec2<f32>(seed, 31.3));
    let floor_on = step(hash12(vec2<f32>(seed * 1.7, cell.y + 0.5)), g.city.y * bfac);
    let ph = hash12(cell * 1.9 + seed);
    let panel = select(step(0.93, ph), step(0.18, ph) * (0.45 + 0.75 * fract(ph * 7.3)), floor_on > 0.5);
    let tint_l = mix(vec3<f32>(0.78, 0.88, 1.0), vec3<f32>(1.0, 0.82, 0.58), step(0.6, hash12(vec2<f32>(seed, 5.5))));
    // Far: floors resolved (bands) → per-floor share; beyond → building average.
    let floor_lod = 1.0 - smoothstep(0.25, 0.7, fwuv.y / fl);
    let far_share = mix(clamp(g.city.y * bfac, 0.0, 1.0) * 0.5 + 0.03, floor_on * 0.7 + (1.0 - floor_on) * 0.05, floor_lod);
    // Unresolved, the light comes through the tinted curtain wall.
    let far_tint = mix(tint_l * mix(vec3<f32>(1.0), normalize(tint + vec3<f32>(0.05)) * 1.6, 0.5), tint_l, lod);
    s.emit = far_tint * mix(far_share, panel, lod) * night * WIN_OFFICE;
    // Mullions and spandrel bands fade separately (floors stay visible further).
    let mullion = mix(0.05, step(fx, 0.04), lodx);
    let band = mix(spandrel, step(1.0 - spandrel, fy), lody);
    let fin = select(0.0, mix(0.1, step(fx, 0.10), lodx), fins);
    let opaque = max(max(mullion, band), fin);
    s.albedo = mix(s.albedo, tint * 0.45 + 0.02, opaque);
    s.rough = mix(s.rough, 0.35, opaque);
    s.f0 = mix(s.f0, vec3<f32>(0.04), opaque);
    s.emit *= 1.0 - opaque;
    s.ao = 0.8 + 0.2 * smoothstep(0.0, 10.0, y);
    return s;
}

// ---------------------------------------------------------------- ground

// Grass colour through the year (sRGB-ish): fresh in spring, parched in late
// summer, dull olive in winter.
fn grass_season(c0: vec3<f32>) -> vec3<f32> {
    var c = c0;
    let l = dot(c, vec3<f32>(0.3, 0.55, 0.15));
    c = mix(c, c * vec3<f32>(1.1, 1.08, 0.85), g.season.z * 0.6);
    c = mix(c, mix(vec3<f32>(l), c, 0.55) * vec3<f32>(1.2, 1.05, 0.72), g.season2.x * 0.55);
    c = mix(c, mix(vec3<f32>(l), c, 0.5) * vec3<f32>(0.92, 0.9, 0.78), g.season.w * 0.75);
    return c;
}

fn field_surface(wp: vec3<f32>, np: vec2<f32>, n: vec3<f32>, fw: f32, dist: f32) -> vec3<f32> {
    // Warped patchwork of ~256 m fields with hedgerow edges.
    let w = vec2<f32>(pfbm(np, 0.0011, vec2<f32>(0.0)), pfbm(np, 0.0011, vec2<f32>(31.0))) * 260.0;
    let q = (np + w) / 256.0;
    let cell = wrapc(floor(q), NP / 256.0);
    let f = fract(q);
    let h = hash12(cell);
    let h2 = hash12(cell + 17.0);
    // Crop calendar (year phase y; each field a few weeks off its neighbours).
    let y = fract(g.season2.y + (h2 - 0.5) * 0.06);
    let str = g.season2.z;
    let ploughed = mix(vec3<f32>(0.36, 0.28, 0.19), vec3<f32>(0.42, 0.33, 0.23), h2);
    let stubble = mix(vec3<f32>(0.58, 0.50, 0.34), vec3<f32>(0.64, 0.56, 0.38), h2);
    let gold = mix(vec3<f32>(0.62, 0.52, 0.27), vec3<f32>(0.68, 0.57, 0.30), h2);
    let green = mix(vec3<f32>(0.24, 0.34, 0.11), vec3<f32>(0.30, 0.39, 0.14), h2);
    let shoots = mix(ploughed, vec3<f32>(0.29, 0.35, 0.15), 0.55);
    var c: vec3<f32>;
    var crop = true;
    if (h < 0.40) {
        // Pasture: green all year, duller and darker in winter.
        c = mix(vec3<f32>(0.20, 0.30, 0.10), vec3<f32>(0.27, 0.36, 0.13), h2);
        crop = false;
    } else if (h < 0.60) {
        // Winter wheat / barley: drilled Oct, green through winter, gold Jul, stubble Aug, ploughed Sep.
        c = mix(shoots, green, smoothstep(0.0, 0.25, y));
        c = mix(c, gold, smoothstep(0.47, 0.54, y));
        c = mix(c, stubble, smoothstep(0.58, 0.62, y));
        c = mix(c, ploughed, smoothstep(0.68, 0.72, y));
        c = mix(c, shoots, smoothstep(0.80, 0.90, y));
    } else if (h < 0.76) {
        // Spring crops: ploughed over winter, green May-Jul, gold Aug, stubble Sep.
        c = mix(ploughed, shoots, smoothstep(0.24, 0.32, y));
        c = mix(c, green, smoothstep(0.32, 0.42, y));
        c = mix(c, gold, smoothstep(0.55, 0.63, y));
        c = mix(c, stubble, smoothstep(0.68, 0.72, y));
        c = mix(c, ploughed, smoothstep(0.80, 0.86, y));
    } else if (h < 0.84) {
        // Oilseed rape: leafy green over winter, bright yellow in late April / May.
        let leafy = vec3<f32>(0.20, 0.30, 0.12);
        c = mix(leafy, vec3<f32>(0.78, 0.70, 0.10), smoothstep(0.28, 0.31, y) * (1.0 - smoothstep(0.38, 0.42, y)));
        c = mix(c, vec3<f32>(0.40, 0.42, 0.20), smoothstep(0.40, 0.44, y));
        c = mix(c, stubble * 0.85, smoothstep(0.55, 0.6, y));
        c = mix(c, ploughed, smoothstep(0.64, 0.68, y));
        c = mix(c, leafy, smoothstep(0.70, 0.80, y));
    } else if (h < 0.92) {
        c = ploughed;
    } else {
        // Hay meadow: lush, cut in June (pale), regrows.
        c = mix(vec3<f32>(0.24, 0.31, 0.14), vec3<f32>(0.30, 0.36, 0.17), h2);
        c = mix(c, vec3<f32>(0.45, 0.44, 0.24), smoothstep(0.44, 0.46, y) * (1.0 - smoothstep(0.5, 0.58, y)));
        crop = false;
    }
    if (!crop) {
        c = grass_season(c);
    }
    // No seasons in the tropics: the old summer palette.
    c = mix(mix(vec3<f32>(0.33, 0.37, 0.15), vec3<f32>(0.40, 0.42, 0.20), h2), c, str);
    // Crop rows / mowing lanes in a per-field direction.
    let ang = h2 * PI;
    let dir = vec2<f32>(cos(ang), sin(ang));
    let local = (f - 0.5) * 256.0; // metres from the field's centre
    let rows = sin(dot(local, dir) * 2.1) * 0.5 + 0.5;
    let rows_lod = 1.0 - smoothstep(0.1, 0.5, fw * 0.35);
    let tram = smoothstep(0.92, 1.0, sin(dot(local, dir) * 0.26)) * (1.0 - smoothstep(1.0, 4.0, fw * 0.26));
    c *= 1.0 + (rows - 0.5) * 0.12 * rows_lod * step(0.42, h) - tram * 0.1;
    // In-field variation.
    c *= 0.86 + 0.28 * pna(np, 0.025, vec2<f32>(h * 9.0), fw * 0.025);
    c *= 0.93 + 0.14 * pna(np, 0.21, vec2<f32>(0.0), fw * 0.21);
    // Hedgerows: dark, slightly lumpy boundaries.
    let e = min(min(f.x, 1.0 - f.x), min(f.y, 1.0 - f.y)) * 256.0;
    let hw = 2.5 + 2.0 * pn(np, 0.05, vec2<f32>(0.0));
    let hedge = 1.0 - smoothstep(hw, hw + max(1.5, fw * 1.5), e);
    c = mix(c, vec3<f32>(0.10, 0.15, 0.06) * (0.7 + 0.6 * pna(np, 0.3, vec2<f32>(0.0), fw * 0.3)), hedge * 0.85);
    // Very far: hints of villages.
    let far = smoothstep(9000.0, 12000.0, length(wp.xz - g.cam_pos.xz));
    let sub_mask = smoothstep(0.55, 0.7, pfbm(np, 0.00035, vec2<f32>(0.0))) * far;
    let suburb = vec3<f32>(0.30, 0.29, 0.27) * (0.8 + 0.4 * pn(np, 0.08, vec2<f32>(0.0)));
    return mix(c, suburb, sub_mask);
}

// ---------------------------------------------------------------- main

@fragment
fn fs_main(in: VOut) -> FOut {
    let m = i32(in.mat + 0.5);
    let rel = in.wpos; // eye-relative
    let wp = rel + g.cam_pos.xyz; // world (heights, sun / cloud shadow)
    let np = pat(rel);
    let px = in.clip.xy;
    let to_cam = -rel;
    let dist = length(to_cam);
    let V = to_cam / dist;
    // Derivatives up front (uniform control flow).
    let ngeo0 = normalize(in.nrm);
    let dndx = dpdx(ngeo0);
    let dndy = dpdy(ngeo0);
    let fwp = max(length(dpdx(rel)), length(dpdy(rel)));  // metres per pixel
    let fwuv = fwidth(in.uv);
    var ngeo = ngeo0;
    // Two-sided geometry: face the viewer.
    if (dot(ngeo, V) < 0.0 && m != 1) {
        ngeo = -ngeo;
    }
    var o: FOut;
    o.dist = dist;
    if (m == 1) {
        o.col = vec4<f32>(apply_fog_px(shade_water(in, px, fwp), px, dist), 1.0);
        return o;
    }
    if (m == 17) {
        // Floodlit crowns of towers: luminance (cd/m²) in uv.x, at night only.
        let c = pre_exposed(srgb_to_lin(in.col) * in.uv.x * LUX * g.sun_true.w);
        o.col = vec4<f32>(apply_fog_px(c, px, dist), 1.0);
        return o;
    }
    if (m == 13) {
        // Self-lit: nav lights, strobes, flame (uv.x intensity, uv.y > 0.5 = flame).
        let c = pre_exposed(srgb_to_lin(in.col) * in.uv.x * 2.0);
        var a = 1.0;
        if (in.uv.y > 0.5) {
            a = clamp(luma(srgb_to_lin(in.col)) * 1.6, 0.0, 1.0);
        }
        o.col = vec4<f32>(apply_fog_px(c, px, dist), a);
        return o;
    }
    var s = default_surf(srgb_to_lin(in.col), ngeo);
    // Aircraft materials (paint, canopy, propeller, bare metal).
    s.moving = select(0.0, 1.0, m == 8 || m == 9 || m == 10 || m == 14);
    var puddle = 0.0;
    switch m {
        case 6: { // grass / vegetation ground
            let n1 = pfbm(np, 0.012, vec2<f32>(0.0));
            let n2 = pna(np, 0.08, vec2<f32>(5.0), fwp * 0.08);
            s.albedo *= (0.66 + 0.68 * n1) * (0.86 + 0.28 * pna(np, 0.5, vec2<f32>(0.0), fwp * 0.5));
            // Clumps of darker, bluer grass / clover.
            s.albedo = mix(s.albedo, s.albedo * vec3<f32>(0.78, 0.9, 0.85), smoothstep(0.55, 0.8, n2));
            s.albedo = mix(s.albedo, s.albedo * vec3<f32>(1.18, 1.04, 0.72), smoothstep(0.4, 0.8, pn(np, 0.004, vec2<f32>(0.0))));
            // Mowing stripes, only up close.
            let stripe = step(0.5, fract(np.x * (nper(1.0 / 7.0) / NP) + 0.3 * pn(np, 0.01, vec2<f32>(0.0))));
            s.albedo *= 1.0 + (stripe - 0.5) * 0.12 * (1.0 - smoothstep(0.3, 1.5, fwp));
            s.albedo = srgb_to_lin(grass_season(pow(s.albedo, vec3<f32>(1.0 / 2.2))));
            s.albedo *= 1.0 - 0.18 * g.wx.x;
            s.rough = 0.9;
            s = snowy(s, np, ngeo, 1.0, fwp);
        }
        case 12: { // base ground: procedural fields (uv.x = 0) or urban ground (uv.x = 1)
            if (in.uv.x > 0.5) {
                s.albedo *= (0.84 + 0.32 * pna(np, 0.04, vec2<f32>(0.0), fwp * 0.04)) * (0.9 + 0.2 * pna(np, 0.6, vec2<f32>(0.0), fwp * 0.6));
                s.rough = 0.85;
                s = wet_paving(s, wp, px, fwp, 0.6);
                s = snowy(s, np, ngeo, 0.8, fwp);
            } else {
                s.albedo = srgb_to_lin(field_surface(wp, np, ngeo, fwp, dist)) * 0.95;
                s.albedo *= 1.0 - 0.22 * g.wx.x;
                s.rough = 0.92;
                s = snowy(s, np, ngeo, 1.0, fwp);
            }
        }
        case 7: { // asphalt / concrete
            let patchy = pna(np, 0.06, vec2<f32>(3.0), fwp * 0.06);
            s.albedo *= 0.8 + 0.4 * pna(np, 0.9, vec2<f32>(0.0), fwp * 0.9) * (0.6 + 0.4 * patchy);
            s.albedo *= 0.92 + 0.16 * patchy;
            s.rough = 0.75;
            s.f0 = vec3<f32>(0.04);
            s = wet_paving(s, wp, px, fwp, 0.2);
            puddle = puddle_mask(np, fwp);
            s = snowy(s, np, ngeo, 0.5, fwp);
        }
        case 15: { // runway: uv = (metres along, metres across); markings are separate geometry
            let along = in.uv.x;
            let across = in.uv.y;
            s.albedo *= 0.85 + 0.3 * pna(np, 0.7, vec2<f32>(0.0), fwp * 0.7);
            // Darker, polished centre (rubber + wear) and tyre marks in the touchdown zones.
            let centre = 1.0 - smoothstep(4.0, 14.0, abs(across));
            s.albedo *= 1.0 - 0.18 * centre;
            let tz = smoothstep(250.0, 330.0, along) * (1.0 - smoothstep(700.0, 1100.0, along));
            let streaks = noise_aa(vec2<f32>(along * 0.02, across * 3.0), fwp * 3.0);
            let tyre = tz * smoothstep(0.45, 0.8, streaks) * (1.0 - smoothstep(2.0, 9.0, abs(abs(across) - 3.5)));
            s.albedo *= 1.0 - 0.55 * tyre;
            s.rough = 0.7 - 0.15 * centre;
            s.f0 = vec3<f32>(0.04);
            s = wet_paving(s, wp, px, fwp, 0.1);
            puddle = puddle_mask(np, fwp) * 0.6;
            s = snowy(s, np, ngeo, 0.35, fwp);
        }
        case 16: { // road: uv.x = metres along, uv.y = metres across + 1000 * width*10
            let code = floor((in.uv.y + 500.0) / 1000.0);
            let across = in.uv.y - code * 1000.0;
            let w = code * 0.1;
            let hw = w * 0.5;
            let along = in.uv.x;
            let patchy = pna(np, 0.05, vec2<f32>(7.0), fwp * 0.05);
            s.albedo *= (0.82 + 0.36 * pna(np, 0.9, vec2<f32>(0.0), fwp * 0.9)) * (0.9 + 0.2 * patchy);
            // Worn wheel tracks and darker gutters.
            let lane = 3.4;
            s.albedo *= 1.0 - 0.18 * (1.0 - smoothstep(0.0, 0.6, hw - abs(across)));
            var paint = 0.0;
            let aa = max(fwp, 0.02);
            if (w >= 6.5) {
                // Centre line (dashed), lane lines on wide roads, solid edge lines.
                let dash = step(fract(along / 9.0), 0.4);
                let line = 1.0 - smoothstep(0.06, 0.06 + aa, abs(across));
                paint = max(paint, line * dash);
                if (w >= 12.0) {
                    let lanes = floor(hw / lane);
                    let x = abs(across);
                    let k = fract(x / lane + 0.5) - 0.5;
                    let ln = (1.0 - smoothstep(0.06, 0.06 + aa, abs(k * lane))) * step(lane * 0.5, x) * step(x, lanes * lane - lane * 0.5) * step(fract(along / 9.0 + 0.5), 0.33);
                    paint = max(paint, ln);
                }
                let edge = 1.0 - smoothstep(0.07, 0.07 + aa, abs(abs(across) - (hw - 0.45)));
                paint = max(paint, edge);
            }
            // Fade markings to their average when they get thin on screen.
            let vis = 1.0 - smoothstep(0.05, 0.2, fwp);
            s.albedo = mix(s.albedo, vec3<f32>(0.62, 0.62, 0.6), paint * mix(0.12, 0.85, vis) * (0.75 + 0.25 * patchy));
            s.rough = 0.78 - 0.1 * paint;
            s.f0 = vec3<f32>(0.04);
            s = wet_paving(s, wp, px, fwp, 0.1);
            // Puddles collect in the gutters and wheel ruts.
            let rut = 1.0 - smoothstep(0.0, 1.2, hw - abs(across));
            puddle = max(puddle_mask(np, fwp), rut * smoothstep(0.5, 0.9, g.wx.x) * smoothstep(0.45, 0.6, pna(np, 0.3, vec2<f32>(0.0), fwp * 0.3)));
            // Snow is cleared from the wheel tracks.
            s = snowy(s, np, ngeo, 0.35 + 0.5 * rut, fwp);
        }
        case 5: { // roofs
            if (ngeo.y > 0.95) {
                // Flat roofs: membrane / gravel, plant boxes and skylights.
                s.albedo *= 0.78 + 0.35 * pna(np, 0.35, vec2<f32>(0.0), fwp * 0.35);
                let gcell = wrapc(floor(np / 4.0), NP / 4.0);
                let k = hash12(gcell);
                let fr = fract(np / 4.0);
                let box = step(0.93, k) * step(0.2, fr.x) * step(fr.x, 0.8) * step(0.2, fr.y) * step(fr.y, 0.8);
                s.albedo = mix(s.albedo, vec3<f32>(0.32, 0.33, 0.34), box * (1.0 - smoothstep(0.1, 0.6, fwp)));
                s.rough = 0.8;
                s = wet_paving(s, wp, px, fwp, 0.3);
            } else {
                // Pitched: tile / slate courses along the slope.
                let course = fract(wp.y / 0.22);
                let lodr = 1.0 - smoothstep(0.03, 0.12, fwp);
                s.albedo *= 1.0 - 0.18 * smoothstep(0.75, 1.0, course) * lodr;
                s.albedo *= 0.85 + 0.3 * pna(np, 1.7, vec2<f32>(wp.y), fwp * 1.7);
                s.rough = mix(0.7, 0.3, g.wx.x);
                s.albedo *= 1.0 - 0.3 * g.wx.x;
            }
            s = snowy(s, np, ngeo, 1.0, fwp);
        }
        case 2: { // masonry wall with punched windows
            s = wall_surface(in, ngeo, in.col, fwuv);
        }
        case 3: { // curtain-wall glass tower
            s = glass_surface(in, ngeo, in.col, fwuv);
        }
        case 8: { // glossy aircraft paint
            s.rough = 0.28;
            s.f0 = vec3<f32>(0.05);
        }
        case 9: { // canopy glass
            s.albedo = vec3<f32>(0.01, 0.012, 0.015);
            s.rough = 0.04;
            s.f0 = vec3<f32>(0.06);
        }
        case 10: { // propeller: uv.x 0 = blade (opaque), 1 = blurred disc
            if (in.uv.x > 0.5) {
                s.albedo = vec3<f32>(0.04);
                s.alpha = 0.18 + 0.22 * smoothstep(0.3, 1.0, in.uv.y);
            } else {
                s.albedo = vec3<f32>(0.035);
                s.rough = 0.5;
            }
        }
        case 11: { // foliage: uv = (height in tree 0..1, seed)
            let hgt = in.uv.x;
            let clump = pn(np, 1.3, vec2<f32>(wp.y * 1.1 + in.uv.y * 50.0));
            s.albedo *= 0.72 + 0.55 * clump;
            // Leaf fall: bare twiggy crowns where the leaves are gone.
            let leaf = in.tree.x;
            if (leaf < 0.999) {
                let lm = pn(np, 1.1, vec2<f32>(wp.y * 0.9 + in.tree.z * 40.0)) * 0.8 + pn(np, 4.0, vec2<f32>(wp.y * 3.0)) * 0.2;
                let has_leaf = smoothstep(1.02 - leaf, 1.1 - leaf, lm + 0.08 * clump);
                // Twigs: a fine see-through tangle of grey-brown branches.
                let tw = pna(np, 6.0, vec2<f32>(wp.y * 3.1, -wp.y * 2.3), fwp * 6.0);
                let twig_a = mix(0.25, 0.8, smoothstep(0.35, 0.7, tw)) * (0.55 + 0.45 * smoothstep(0.2, 0.6, 1.0 - hgt));
                let twig = srgb_to_lin(vec3<f32>(0.25, 0.21, 0.18)) * (0.7 + 0.5 * tw);
                s.albedo = mix(twig, s.albedo, has_leaf);
                s.alpha = mix(twig_a, 1.0, has_leaf);
                if (is_refl()) {
                    s.alpha = 1.0;
                }
            }
            // Leaf-clump normal noise so crowns aren't smooth blobs.
            let jn = vec3<f32>(pn(np, 2.1, vec2<f32>(wp.y)) - 0.5, 0.0, pn(np.yx, 2.1, vec2<f32>(-wp.y)) - 0.5);
            s.n = normalize(ngeo + jn * 0.5 * (1.0 - smoothstep(0.1, 0.6, fwp)));
            s.ao = (0.45 + 0.55 * smoothstep(0.25, 1.0, hgt)) * (0.75 + 0.25 * clump);
            s.rough = 0.75;
            s.transl = 0.9 * mix(0.2, 1.0, in.tree.x);
            s.env = 0.5;
            s.albedo *= 1.0 - 0.15 * g.wx.x;
            s = snowy(s, np, ngeo, 0.6, fwp);
        }
        case 14: { // bare metal
            s.f0 = srgb_to_lin(in.col);
            s.albedo = s.f0 * 0.15;
            s.rough = 0.32;
        }
        default: {}
    }
    s.rough = spec_aa(s.rough, dndx, dndy);
    var col = light(s, rel, V, dist, px, ngeo);
    // Wet paving: glossy reflections smeared toward the viewer (the long
    // streaks of street lights and headlights on a wet road at night).
    let paved = m == 7 || m == 15 || m == 16 || (m == 12 && in.uv.x > 0.5) || (m == 5 && ngeo.y > 0.95);
    if (paved && g.wx.x > 0.15 && !is_refl() && wp.y < 0.5) {
        col += wet_streaks(px, ngeo, V, dist) * smoothstep(0.15, 0.6, g.wx.x) * (1.0 - puddle);
    }
    // Puddles: a thin sheet of water mirroring the scene (planar reflection).
    if (puddle > 0.0 && wp.y < 0.5) {
        let rip = rain_ripples(np, 2.2 * 1.7, g.cam_pos.w, fwp * 0.1);
        let n = normalize(vec3<f32>(-rip.x, 1.0, -rip.y));
        let nv = max(dot(n, V), 1e-3);
        let fres = 0.02 + 0.98 * pow(1.0 - nv, 5.0);
        var refl: vec3<f32>;
        if (is_refl()) {
            let R = reflect(-V, n);
            refl = sky_reflection(vec3<f32>(R.x, abs(R.y), R.z));
        } else {
            let suv = px * g.screen.zw + n.xz * 0.04 / (1.0 + dist * 0.003);
            refl = textureSampleLevel(refl_tex, lin_samp, clamp(suv, vec2<f32>(0.001), vec2<f32>(0.999)), 0.0).rgb;
        }
        let under = col * 0.75;
        col = mix(col, mix(under, refl, fres), puddle);
    }
    o.col = vec4<f32>(apply_fog_px(col, px, dist), s.alpha);
    return o;
}

// ---------------------------------------------------------------- sky

// Equatorial coordinates of a world direction.
fn to_equatorial(d: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(dot(g.stars0.xyz, d), dot(g.stars1.xyz, d), dot(g.stars2.xyz, d));
}

fn hash33(p: vec3<f32>) -> vec3<f32> {
    var q = fract(p * vec3<f32>(0.1031, 0.1030, 0.0973));
    q += dot(q, q.yxz + 33.33);
    return fract((q.xxy + q.yxx) * q.zyx);
}

// Stars too faint for the catalogue (mag 6..8.5): hashed in equatorial space,
// energy-conserving gaussians at least ~0.75 px wide (no sparkle).
fn faint_stars(d: vec3<f32>) -> vec3<f32> {
    let e = to_equatorial(d);
    let ppr = g.sky.w;
    let sig = max(0.75 / ppr, 1e-6);
    var acc = vec3<f32>(0.0);
    for (var layer = 0; layer < 2; layer++) {
        let n = select(120.0, 77.0, layer == 1);
        let p = e * n;
        let cell = floor(p);
        let h = hash33(cell + f32(layer) * 71.3);
        if (h.x < 0.42) {
            let sp = normalize((cell + 0.3 + 0.4 * hash33(cell + 13.1)) / n);
            let dd = length(e - sp);
            let mag = 6.0 + 2.5 * pow(h.y, 0.6) + f32(layer) * 0.3;
            let ev = 2.54e-6 * pow(10.0, -0.4 * mag) * LUX;
            let r = dd / sig;
            let col = mix(vec3<f32>(1.0, 0.85, 0.7), vec3<f32>(0.8, 0.88, 1.0), h.z);
            acc += col * ev / (2.0 * PI * sig * sig) * exp(-0.5 * r * r);
        }
    }
    return acc;
}

// The Milky Way: a band along the galactic plane, brightest toward the
// centre (Sagittarius), cut by dust lanes. ~21 mag/arcsec² at its brightest.
fn milky_way(d: vec3<f32>) -> vec3<f32> {
    let e = to_equatorial(d);
    // J2000 equatorial → galactic.
    let gx = -0.0548756 * e.x - 0.8734371 * e.y - 0.4838350 * e.z;
    let gy = 0.4941094 * e.x - 0.4448296 * e.y + 0.7469822 * e.z;
    let gz = -0.8676661 * e.x - 0.1980764 * e.y + 0.4559838 * e.z;
    let b = asin(clamp(gz, -1.0, 1.0));
    let l = atan2(gy, gx);
    let core = exp(-l * l / 0.9);
    let width = 0.16 + 0.12 * core;
    var band = exp(-(b / width) * (b / width)) * (0.3 + 0.7 * core) + 0.35 * exp(-(l * l + b * b * 6.0) / 0.12);
    // Clumpy star clouds and dark dust lanes.
    let q = vec2<f32>(l * 5.0, b * 9.0);
    let clump = fbm2(q * 1.3 + vec2<f32>(3.1, 7.7));
    let dust = smoothstep(0.45, 0.75, fbm2(q * 2.1 + vec2<f32>(11.0, 2.0))) * exp(-(b / 0.06) * (b / 0.06));
    band *= (0.55 + 0.9 * clump) * (1.0 - 0.75 * dust);
    return vec3<f32>(0.9, 0.92, 1.0) * band * (4.0e-4 * LUX);
}

// Approximate selenographic albedo: the near-side maria as soft patches.
fn moon_albedo(p: vec3<f32>) -> f32 {
    // (lat, lon, radius) in degrees.
    var maria = array<vec3<f32>, 18>(
        vec3<f32>(33.0, -16.0, 11.0), vec3<f32>(28.0, 17.0, 6.5), vec3<f32>(8.5, 31.0, 8.5), vec3<f32>(17.0, 59.0, 5.0),
        vec3<f32>(-8.0, 51.0, 6.5), vec3<f32>(-15.0, 35.0, 3.5), vec3<f32>(-21.0, -17.0, 7.0), vec3<f32>(-24.0, -39.0, 4.0),
        vec3<f32>(20.0, -55.0, 12.0), vec3<f32>(4.0, -52.0, 10.0), vec3<f32>(-6.0, -38.0, 7.0), vec3<f32>(56.0, -15.0, 4.5),
        vec3<f32>(56.0, 12.0, 4.5), vec3<f32>(13.0, 4.0, 3.5), vec3<f32>(-10.0, -23.0, 4.5), vec3<f32>(7.0, -31.0, 5.0),
        vec3<f32>(30.0, -35.0, 7.0), vec3<f32>(-3.0, 15.0, 4.0));
    var dark = 0.0;
    for (var i = 0; i < 18; i++) {
        let m = maria[i] * (PI / 180.0);
        let c = vec3<f32>(cos(m.x) * sin(m.y), sin(m.x), cos(m.x) * cos(m.y));
        let ang = acos(clamp(dot(p, c), -1.0, 1.0));
        dark = max(dark, smoothstep(m.z * 1.2, m.z * 0.7, ang));
    }
    let tex = fbm2(p.xy * 9.0 + p.z * 3.0);
    var a = mix(1.15, 0.58, dark * (0.8 + 0.4 * tex)) * (0.9 + 0.2 * tex);
    // Tycho and its rays.
    let ty = vec3<f32>(cos(-0.75) * sin(-0.19), sin(-0.75), cos(-0.75) * cos(-0.19));
    a += 0.5 * smoothstep(0.03, 0.0, acos(clamp(dot(p, ty), -1.0, 1.0)));
    return a;
}

// The moon's disc: rgb radiance (before extinction), a = coverage.
fn moon_disc(d: vec3<f32>) -> vec4<f32> {
    let m = g.moon.xyz;
    // Angular radius: 0.26° for real, drawn 1.8x larger — on a ~60° screen the
    // true disc is a few pixels, far smaller than the eye perceives it.
    let R = 0.00452 * 1.8;
    let cm = dot(d, m);
    if (cm < cos(R * 1.3) || g.sky.y <= 0.0) {
        return vec4<f32>(0.0);
    }
    // Disc frame: up = toward the celestial pole, x = cross(m, up) (lunar east).
    let ncp = g.stars2.xyz;
    let up = normalize(ncp - m * dot(ncp, m));
    let rt = cross(m, up);
    let off = d - m * cm;
    let x = dot(off, rt) / R;
    let y = dot(off, up) / R;
    let r = sqrt(x * x + y * y);
    let aa = 1.0 - smoothstep(1.0 - 1.2 / max(R * g.sky.w, 1.0), 1.0, r);
    if (aa <= 0.0) {
        return vec4<f32>(0.0);
    }
    let z = sqrt(max(1.0 - r * r, 0.0));
    let p = vec3<f32>(x, y, z);
    let nw = rt * x + up * y - m * z;
    let mu0 = dot(nw, g.sun_true.xyz);
    let alb = moon_albedo(p);
    var L = g.sky.z * alb;
    if (mu0 > 0.0) {
        L += g.sky.y * alb * 2.0 * mu0 / (mu0 + max(z, 1e-3));
    }
    // Regolith is slightly red.
    return vec4<f32>(vec3<f32>(1.0, 0.93, 0.84) * L, aa);
}

struct SkyOut {
    @builtin(position) pos: vec4<f32>,
    @location(0) ndc: vec2<f32>,
};

@vertex
fn vs_sky(@builtin(vertex_index) i: u32) -> SkyOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: SkyOut;
    o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    o.ndc = o.pos.xy;
    return o;
}

@fragment
fn fs_sky(in: SkyOut) -> FOut {
    let p = g.inv_view_proj * vec4<f32>(in.ndc, 0.5, 1.0);
    // inv_view_proj is eye-relative (rotation only): no large-coordinate cancellation.
    let d = normalize(p.xyz / p.w);
    var c = sky_radiance(d);
    // Sun disc with limb darkening (radiance clamped to fit fp16).
    let sun = g.sun_true.xyz;
    let mu = dot(d, sun);
    let cosr = cos(0.00475);
    if (mu > cosr) {
        let x = clamp((1.0 - mu) / (1.0 - cosr), 0.0, 1.0);
        let limb = 1.0 - 0.6 * (1.0 - sqrt(1.0 - x));
        let T = transmittance(abs(g.cam_pos.y), sun.y);
        let disk = vec3<f32>(2.5 * g.pre.x) * T * limb / (2.0 * PI * (1.0 - cosr));
        var cs = 1.0;
        if (g.sun_dir.y == sun.y) {
            cs = cloud_shadow(g.cam_pos.xyz);
        }
        c += min(disk * cs, vec3<f32>(30000.0));
    }
    // Night sky: faint stars and the Milky Way (the bright catalogue stars are
    // sprites, lights.wgsl), then the moon in front of them.
    if (g.sun_true.w > 0.0 && d.y > -0.03) {
        let T = transmittance(abs(g.cam_pos.y), max(d.y, 0.0));
        c += pre_exposed(faint_stars(d) + milky_way(d)) * T * smoothstep(-0.03, 0.02, d.y) * g.sun_true.w;
    }
    let md = moon_disc(d);
    if (md.a > 0.0) {
        let T = transmittance(abs(g.cam_pos.y), g.moon.y);
        // The moon is ~10^5 x brighter than the night landscape the eye is
        // adapted to; we still see its maria. Hold the disc at a bright but
        // unsaturated level (its light on the world is the key light, unchanged).
        var mrad = pre_exposed(md.rgb * T);
        let shown = luma(mrad) * atmos.expo.x;
        let cap = 9.0;
        if (shown > cap) {
            mrad *= cap / shown;
        }
        c = mix(c, sky_radiance(d) + mrad, md.a);
    }
    var o: FOut;
    o.col = vec4<f32>(c, 1.0);
    o.dist = 65000.0;
    return o;
}
