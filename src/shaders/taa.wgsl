// Anti-aliasing and camera post effects on the HDR image (gfx/taa.rs):
//   fs_taa    temporal AA / temporal upscaler (TAAU): jitter-aware
//             reconstruction of the current frame at the output resolution,
//             motion vectors (camera reprojection from the eye-distance
//             buffer, object velocities from the aircraft velocity pass,
//             cloud distance for the clouds), Catmull-Rom history, variance
//             clipping in YCoCg, disocclusion test against the history's
//             distances, luminance-weighted blend.
//   fs_fxaa   FXAA (the cheap option).
//   fs_mblur  camera / per-object motion blur.
//   fs_dof    depth of field (gather, scatter-as-gather weights).
// Everything is eye-relative: no world coordinates, so the floating origin
// moving changes nothing here (the CPU keeps the previous eye in the same frame).

struct TaaU {
    inv_vp: mat4x4<f32>,  // this frame, unjittered, eye-relative (rotation only): ray = normalize(p.xyz / p.w)
    prev_vp: mat4x4<f32>, // last frame's unjittered view-projection, relative to this frame's eye
    jitter: vec4<f32>,    // xy this frame's jitter (input px, +x right, +y down), z history valid, w exposure ratio (now / then)
    in_size: vec4<f32>,   // input (scene) w, h, 1/w, 1/h
    out_size: vec4<f32>,  // output w, h, 1/w, 1/h
    eye_delta: vec4<f32>, // this eye − last frame's eye (m), w = clouds bound
    params: vec4<f32>,    // x min blend, y max blend, z effect strength (motion blur: shutter; dof: aperture px), w mode
    focus: vec4<f32>,     // x focus distance (m, 0 = auto: screen centre), y max blur px, z object-blur on, w frame
};

@group(0) @binding(0) var<uniform> T: TaaU;
@group(0) @binding(1) var cur_tex: texture_2d<f32>;
@group(0) @binding(2) var dist_tex: texture_2d<f32>;
@group(0) @binding(3) var vel_tex: texture_2d<f32>;
@group(0) @binding(4) var hist_tex: texture_2d<f32>;
@group(0) @binding(5) var lin: sampler;
@group(0) @binding(6) var cloud_col: texture_2d<f32>;
@group(0) @binding(7) var cloud_dep: texture_2d<f32>;

const SKY: f32 = 60000.0;

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

fn luma(c: vec3<f32>) -> f32 {
    return dot(c, vec3<f32>(0.2126, 0.7152, 0.0722));
}

fn rgb_to_ycocg(c: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(0.25 * c.r + 0.5 * c.g + 0.25 * c.b, 0.5 * c.r - 0.5 * c.b, -0.25 * c.r + 0.5 * c.g - 0.25 * c.b);
}

fn ycocg_to_rgb(c: vec3<f32>) -> vec3<f32> {
    return vec3<f32>(c.x + c.y - c.z, c.x + c.z, c.x - c.y - c.z);
}

// Reversible "Karis" weighting: HDR → [0, 1) so a few very bright samples
// don't dominate the neighbourhood statistics (fireflies, specular sparkle).
fn compress(c: vec3<f32>) -> vec3<f32> {
    return c / (1.0 + max(c.r, max(c.g, c.b)));
}

fn uncompress(c: vec3<f32>) -> vec3<f32> {
    return c / max(1.0 - max(c.r, max(c.g, c.b)), 1e-4);
}

fn ray_dir(uv: vec2<f32>) -> vec3<f32> {
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let p = T.inv_vp * vec4<f32>(ndc, 0.5, 1.0);
    return normalize(p.xyz / p.w);
}

fn in_load(tex: texture_2d<f32>, p: vec2<i32>) -> vec4<f32> {
    let d = vec2<i32>(T.in_size.xy);
    return textureLoad(tex, clamp(p, vec2<i32>(0), d - 1), 0);
}

// Motion of an output-space uv: (uv in last frame − uv now), the eye
// distance the history should have there (or < 0 = don't test), and whether
// it is an object (aircraft) pixel.
struct Motion {
    mv: vec2<f32>,
    prev_dist: f32,
    dist: f32,
    // Spread of the eye distances in the 3×3 neighbourhood (slopes, edges).
    range: f32,
    object: bool,
};

fn motion_at(uv: vec2<f32>) -> Motion {
    // The nearest surface in the 3×3 input neighbourhood (dilated motion:
    // edges move with the foreground).
    let ip = vec2<i32>(floor(uv * T.in_size.xy));
    var best = 1e9;
    var far = 0.0;
    var bp = ip;
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let q = ip + vec2<i32>(x, y);
            let d = in_load(dist_tex, q).x;
            far = max(far, d);
            if (d < best) {
                best = d;
                bp = q;
            }
        }
    }
    var m: Motion;
    m.dist = best;
    m.range = far - best;
    m.object = false;
    // An aircraft (velocity pass) in front: its own motion, if it is what
    // the scene shows there (not hidden behind something nearer).
    let v = in_load(vel_tex, bp);
    if (v.z > 0.0 && abs(v.z - best) < 0.03 * v.z + 0.6) {
        m.mv = v.xy;
        m.prev_dist = v.w;
        m.object = true;
        return m;
    }
    var d = best;
    // Clouds in front of the surface: they move with their own distance.
    if (T.eye_delta.w > 0.5) {
        let cdims = vec2<i32>(textureDimensions(cloud_col));
        let cp = clamp(vec2<i32>(uv * vec2<f32>(cdims)), vec2<i32>(0), cdims - 1);
        let ct = textureLoad(cloud_col, cp, 0).a;
        let cd = textureLoad(cloud_dep, cp, 0).y;
        if (ct < 0.5 && cd > 1.0 && cd < d) {
            d = cd;
        }
    }
    let rd = ray_dir(uv);
    var clip: vec4<f32>;
    if (d >= SKY) {
        // Sky: direction only.
        clip = T.prev_vp * vec4<f32>(rd, 0.0);
        m.prev_dist = -1.0;
    } else {
        let p = rd * d;
        clip = T.prev_vp * vec4<f32>(p, 1.0);
        m.prev_dist = length(p + T.eye_delta.xyz);
    }
    if (clip.w <= 1e-5) {
        m.mv = vec2<f32>(10.0);
        return m;
    }
    let pn = clip.xy / clip.w;
    let puv = vec2<f32>(pn.x * 0.5 + 0.5, 0.5 - pn.y * 0.5);
    m.mv = puv - uv;
    return m;
}

// Catmull-Rom history sample in 5 bilinear taps (Jimenez, "Filmic SMAA").
fn history_catmull(uv: vec2<f32>) -> vec3<f32> {
    let size = T.out_size.xy;
    let sp = uv * size;
    let tc = floor(sp - 0.5) + 0.5;
    let f = sp - tc;
    let w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    let w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    let w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    let w3 = f * f * (-0.5 + 0.5 * f);
    let w12 = w1 + w2;
    let tc0 = (tc - 1.0) * T.out_size.zw;
    let tc3 = (tc + 2.0) * T.out_size.zw;
    let tc12 = (tc + w2 / w12) * T.out_size.zw;
    var c = textureSampleLevel(hist_tex, lin, vec2<f32>(tc12.x, tc0.y), 0.0).rgb * (w12.x * w0.y);
    c += textureSampleLevel(hist_tex, lin, vec2<f32>(tc0.x, tc12.y), 0.0).rgb * (w0.x * w12.y);
    c += textureSampleLevel(hist_tex, lin, vec2<f32>(tc12.x, tc12.y), 0.0).rgb * (w12.x * w12.y);
    c += textureSampleLevel(hist_tex, lin, vec2<f32>(tc3.x, tc12.y), 0.0).rgb * (w3.x * w12.y);
    c += textureSampleLevel(hist_tex, lin, vec2<f32>(tc12.x, tc3.y), 0.0).rgb * (w12.x * w3.y);
    let wsum = w12.x * w0.y + w0.x * w12.y + w12.x * w12.y + w3.x * w12.y + w12.x * w3.y;
    return max(c / wsum, vec3<f32>(0.0));
}

struct TaaOut {
    @location(0) col: vec4<f32>,
};

@fragment
fn fs_taa(in: FsOut) -> TaaOut {
    let uv = in.pos.xy * T.out_size.zw;
    // ---- this frame, reconstructed at the output pixel centre. Input texel
    // k's sample sits at (k + 0.5 − jitter) in unjittered input pixels.
    let p_in = uv * T.in_size.xy;
    let base = vec2<i32>(floor(p_in + T.jitter.xy));
    var sum = vec3<f32>(0.0);
    var wsum = 0.0;
    var wmax = 0.0;
    var m1 = vec3<f32>(0.0);
    var m2 = vec3<f32>(0.0);
    var cmin = vec3<f32>(1e9);
    var cmax = vec3<f32>(-1e9);
    // Footprint: one input pixel, wider when upscaling a lot (no holes).
    let up = max(T.out_size.x * T.in_size.z, 1.0);
    let sharp = 2.29 / mix(1.0, up, 0.5);
    for (var y = -1; y <= 1; y++) {
        for (var x = -1; x <= 1; x++) {
            let k = base + vec2<i32>(x, y);
            let c = max(in_load(cur_tex, k).rgb, vec3<f32>(0.0));
            let spos = vec2<f32>(k) + 0.5 - T.jitter.xy;
            let d = spos - p_in;
            // Gaussian fit of Blackman-Harris (radius ~1 input px).
            let w = exp(-sharp * dot(d, d));
            let cc = compress(c);
            // (The reconstruction is linear: the sun keeps its energy for the bloom.)
            sum += c * w;
            wsum += w;
            wmax = max(wmax, w);
            let yc = rgb_to_ycocg(cc);
            m1 += yc;
            m2 += yc * yc;
            cmin = min(cmin, yc);
            cmax = max(cmax, yc);
        }
    }
    let cur = sum / max(wsum, 1e-5);
    let mot = motion_at(uv);
    let here_d = in_load(dist_tex, vec2<i32>(floor(p_in))).x;
    var o: TaaOut;
    let puv = uv + mot.mv;
    let off = any(puv < vec2<f32>(0.0)) || any(puv > vec2<f32>(1.0));
    if (T.jitter.z < 0.5 || off) {
        o.col = vec4<f32>(cur, here_d);
        return o;
    }
    // ---- history.
    var hist = history_catmull(puv) * T.jitter.w;
    // Disocclusion: the history's surfaces (2×2 around the reprojected
    // point) must bracket the distance this point had last frame.
    var disoccluded = false;
    let mpx = length(mot.mv * T.out_size.xy);
    // (Only where something moved: a still view's history is valid. The
    // tolerance grows with the local depth spread — grazing ground, edges.)
    let moved = mpx > 0.05 || dot(T.eye_delta.xyz, T.eye_delta.xyz) > 1e-6;
    if (mot.prev_dist > 0.0 && moved) {
        let hp = puv * T.out_size.xy - 0.5;
        let hb = vec2<i32>(floor(hp));
        let hd = vec2<i32>(T.out_size.xy) - 1;
        var lo = 1e9;
        var hi = 0.0;
        for (var k = 0; k < 4; k++) {
            let q = clamp(hb + vec2<i32>(k & 1, k >> 1), vec2<i32>(0), hd);
            let dh = textureLoad(hist_tex, q, 0).a;
            lo = min(lo, dh);
            hi = max(hi, dh);
        }
        let tol = 0.06 * mot.prev_dist + 0.5 + mot.range;
        disoccluded = mot.prev_dist < lo - tol || mot.prev_dist > hi + tol;
    }
    // ---- variance clipping (YCoCg, compressed), loose when still (lets the
    // jitter converge), tight when moving (no ghosting).
    let mean = m1 / 9.0;
    let sd = sqrt(max(m2 / 9.0 - mean * mean, vec3<f32>(0.0)));
    // Still pixels (a parked camera, the cockpit) keep sub-pixel detail the
    // current frame missed (thin lines, wires): a loose box; moving ones a
    // tight box bounded by the neighbourhood's extremes.
    let still = 1.0 - smoothstep(0.05, 1.5, mpx);
    let gamma = mix(1.0, 3.0, still);
    var bmin = mean - sd * gamma;
    var bmax = mean + sd * gamma;
    if (still < 0.5) {
        bmin = max(bmin, cmin);
        bmax = min(bmax, cmax);
    }
    let hy = rgb_to_ycocg(compress(hist));
    // Clip toward the box centre (not per-axis clamp: no hue shifts).
    let centre = 0.5 * (bmin + bmax);
    let ext = max(0.5 * (bmax - bmin), vec3<f32>(1e-5));
    let dv = hy - centre;
    let ts = abs(dv / ext);
    let mt = max(ts.x, max(ts.y, ts.z));
    var hc = hy;
    if (mt > 1.0) {
        hc = centre + dv / mt;
    }
    let hist_l = uncompress(clamp(ycocg_to_rgb(hc), vec3<f32>(0.0), vec3<f32>(0.999999)));
    // ---- blend: more of this frame when moving fast, when this output
    // pixel has no input sample nearby (upscaling), or disoccluded.
    var a = mix(T.params.x, T.params.y, clamp(mpx / 12.0, 0.0, 1.0));
    a = a * mix(0.35, 1.0, wmax);
    if (disoccluded) {
        a = max(a, 0.5);
    }
    a = clamp(a, 0.02, 1.0);
    // Luminance-weighted (anti-flicker) blend.
    let wc = a / (1.0 + luma(cur));
    let wh = (1.0 - a) / (1.0 + luma(hist_l));
    var res = (cur * wc + hist_l * wh) / (wc + wh);
    if (any(res != res)) {
        res = cur;
    }
    o.col = vec4<f32>(res, here_d);
    return o;
}

// ---------------------------------------------------------------- FXAA

fn fx_luma(c: vec3<f32>) -> f32 {
    // Perceptual: log-ish compression of the HDR luminance.
    let l = luma(c) * T.jitter.w;
    return sqrt(l / (1.0 + l));
}

fn fx_tap(uv: vec2<f32>) -> vec3<f32> {
    return textureSampleLevel(cur_tex, lin, uv, 0.0).rgb;
}

@fragment
fn fs_fxaa(in: FsOut) -> @location(0) vec4<f32> {
    // FXAA 3.11 "quality" style: local contrast test, edge direction, search
    // along the edge (8 steps), sub-pixel blend.
    let rcp = T.in_size.zw;
    let uv = in.pos.xy * rcp;
    let c = fx_tap(uv);
    let lm = fx_luma(c);
    let ln = fx_luma(fx_tap(uv + vec2<f32>(0.0, -rcp.y)));
    let ls = fx_luma(fx_tap(uv + vec2<f32>(0.0, rcp.y)));
    let le = fx_luma(fx_tap(uv + vec2<f32>(rcp.x, 0.0)));
    let lw = fx_luma(fx_tap(uv + vec2<f32>(-rcp.x, 0.0)));
    let rmax = max(max(ln, ls), max(le, max(lw, lm)));
    let rmin = min(min(ln, ls), min(le, min(lw, lm)));
    let range = rmax - rmin;
    if (range < max(0.0312, rmax * 0.125)) {
        return vec4<f32>(c, 1.0);
    }
    let lnw = fx_luma(fx_tap(uv + vec2<f32>(-rcp.x, -rcp.y)));
    let lne = fx_luma(fx_tap(uv + vec2<f32>(rcp.x, -rcp.y)));
    let lsw = fx_luma(fx_tap(uv + vec2<f32>(-rcp.x, rcp.y)));
    let lse = fx_luma(fx_tap(uv + vec2<f32>(rcp.x, rcp.y)));
    // Sub-pixel aliasing amount.
    let lavg = (2.0 * (ln + ls + le + lw) + lnw + lne + lsw + lse) / 12.0;
    let sub = clamp(abs(lavg - lm) / range, 0.0, 1.0);
    let subpx = smoothstep(0.0, 1.0, sub);
    let sub_blend = subpx * subpx * 0.75;
    let horz = abs(lnw + lne - 2.0 * ln) + 2.0 * abs(lw + le - 2.0 * lm) + abs(lsw + lse - 2.0 * ls);
    let vert = abs(lnw + lsw - 2.0 * lw) + 2.0 * abs(ln + ls - 2.0 * lm) + abs(lne + lse - 2.0 * le);
    let is_h = horz >= vert;
    var step = select(rcp.x, rcp.y, is_h);
    var l1 = select(lw, ln, is_h);
    var l2 = select(le, ls, is_h);
    let g1 = abs(l1 - lm);
    let g2 = abs(l2 - lm);
    var lpair: f32;
    if (g1 >= g2) {
        step = -step;
        lpair = l1;
    } else {
        lpair = l2;
    }
    let grad = max(g1, g2) * 0.25;
    let lavg_edge = 0.5 * (lm + lpair);
    var e_uv = uv;
    if (is_h) {
        e_uv.y += step * 0.5;
    } else {
        e_uv.x += step * 0.5;
    }
    let dir = select(vec2<f32>(0.0, rcp.y), vec2<f32>(rcp.x, 0.0), is_h);
    var p1 = e_uv - dir;
    var p2 = e_uv + dir;
    var d1 = fx_luma(fx_tap(p1)) - lavg_edge;
    var d2 = fx_luma(fx_tap(p2)) - lavg_edge;
    var done1 = abs(d1) >= grad;
    var done2 = abs(d2) >= grad;
    var quality = array<f32, 8>(1.0, 1.5, 2.0, 2.0, 2.0, 2.0, 4.0, 8.0);
    for (var i = 0; i < 8; i++) {
        if (done1 && done2) {
            break;
        }
        if (!done1) {
            p1 -= dir * quality[i];
            d1 = fx_luma(fx_tap(p1)) - lavg_edge;
            done1 = abs(d1) >= grad;
        }
        if (!done2) {
            p2 += dir * quality[i];
            d2 = fx_luma(fx_tap(p2)) - lavg_edge;
            done2 = abs(d2) >= grad;
        }
    }
    let dist1 = select(uv.y - p1.y, uv.x - p1.x, is_h);
    let dist2 = select(p2.y - uv.y, p2.x - uv.x, is_h);
    let near1 = dist1 < dist2;
    let dmin = min(dist1, dist2);
    let span = dist1 + dist2;
    let px_off = -dmin / span + 0.5;
    let centre_smaller = lm < lavg_edge;
    let good = (select(d2, d1, near1) < 0.0) != centre_smaller;
    var off = select(0.0, px_off, good);
    off = max(off, sub_blend);
    var f_uv = uv;
    if (is_h) {
        f_uv.y += off * step;
    } else {
        f_uv.x += off * step;
    }
    return vec4<f32>(fx_tap(f_uv), 1.0);
}

// ---------------------------------------------------------------- motion blur

@fragment
fn fs_mblur(in: FsOut) -> @location(0) vec4<f32> {
    let uv = in.pos.xy * T.out_size.zw;
    let c0 = textureSampleLevel(cur_tex, lin, uv, 0.0);
    let m = motion_at(uv);
    var mv = -m.mv; // screen motion over the last frame (uv)
    if (m.object && T.focus.z < 0.5) {
        mv = vec2<f32>(0.0); // camera-only blur keeps the aircraft sharp
    }
    if (T.jitter.z < 0.5) {
        return c0;
    }
    // Shutter: a fraction of the frame interval, capped in pixels.
    var vpx = mv * T.out_size.xy * T.params.z;
    let len = length(vpx);
    let maxpx = T.focus.y;
    if (len > maxpx) {
        vpx *= maxpx / len;
    }
    if (length(vpx) < 0.5) {
        return c0;
    }
    let n = 10;
    let dv = vpx * T.out_size.zw;
    // Per-pixel noise offset (TAA / grain hide the banding).
    let jit = fract(52.9829189 * fract(dot(in.pos.xy, vec2<f32>(0.06711056, 0.00583715))) + T.focus.w * 0.618034) - 0.5;
    var acc = c0.rgb;
    var wsum = 1.0;
    for (var i = 0; i < n; i++) {
        let t = (f32(i) + 0.5 + jit) / f32(n) - 0.5;
        let q = uv + dv * t;
        if (any(q < vec2<f32>(0.0)) || any(q > vec2<f32>(1.0))) {
            continue;
        }
        // Don't smear a moving background over a sharp, nearer object.
        let qd = in_load(dist_tex, vec2<i32>(q * T.in_size.xy)).x;
        let w = select(1.0, 0.0, qd < m.dist * 0.8 && m.object == false);
        acc += textureSampleLevel(cur_tex, lin, q, 0.0).rgb * w;
        wsum += w;
    }
    return vec4<f32>(acc / wsum, c0.a);
}

// ---------------------------------------------------------------- depth of field

fn coc_px(d: f32, focus: f32) -> f32 {
    // Thin lens (normalised): blur ∝ |1/f − 1/d|, in output pixels.
    let k = abs(1.0 / max(focus, 0.1) - 1.0 / max(d, 0.1)) * focus;
    return clamp(k * T.params.z, 0.0, T.focus.y);
}

@fragment
fn fs_dof(in: FsOut) -> @location(0) vec4<f32> {
    let uv = in.pos.xy * T.out_size.zw;
    let c0 = textureSampleLevel(cur_tex, lin, uv, 0.0);
    var focus = T.focus.x;
    if (focus <= 0.0) {
        // Auto-focus: the nearest of five taps around the screen centre.
        focus = 1e9;
        var taps = array<vec2<f32>, 5>(vec2<f32>(0.0), vec2<f32>(0.04, 0.0), vec2<f32>(-0.04, 0.0), vec2<f32>(0.0, 0.04), vec2<f32>(0.0, -0.04));
        for (var k = 0; k < 5; k++) {
            let o = taps[k];
            focus = min(focus, in_load(dist_tex, vec2<i32>((vec2<f32>(0.5) + o) * T.in_size.xy)).x);
        }
    }
    let d0 = in_load(dist_tex, vec2<i32>(uv * T.in_size.xy)).x;
    let r0 = coc_px(d0, focus);
    if (r0 < 0.5) {
        return c0;
    }
    // Vogel disk, 16 taps; a tap counts if its own blur reaches this pixel
    // (sharp foreground doesn't bleed into the blurred background).
    var acc = c0.rgb;
    var wsum = 1.0;
    let rot = fract(52.9829189 * fract(dot(in.pos.xy, vec2<f32>(0.06711056, 0.00583715)))) * 6.2831853;
    for (var i = 0; i < 16; i++) {
        let r = sqrt((f32(i) + 0.5) / 16.0) * r0;
        let a = f32(i) * 2.39996 + rot;
        let q = uv + vec2<f32>(cos(a), sin(a)) * r * T.out_size.zw;
        let qd = in_load(dist_tex, vec2<i32>(clamp(q, vec2<f32>(0.0), vec2<f32>(0.9999)) * T.in_size.xy)).x;
        let rq = coc_px(qd, focus);
        let w = clamp(rq - r + 1.0, 0.0, 1.0) * select(1.0, clamp(rq / max(r0, 1e-3), 0.0, 1.0), qd < d0);
        acc += textureSampleLevel(cur_tex, lin, q, 0.0).rgb * w;
        wsum += w;
    }
    return vec4<f32>(acc / wsum, c0.a);
}
