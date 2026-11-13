// Weather drama on the GPU (wx/gpu.rs): lightning channels and the
// windscreen (rain drops, wipers, misting, frost). Bindings are unique across
// the module (the passes use separate auto layouts).

// ------------------------------------------------------------------ lightning

struct BoltU {
    view_proj: mat4x4<f32>, // eye-relative
    screen: vec4<f32>,      // w, h, pre-exposure, 0
};
@group(0) @binding(0) var<uniform> B: BoltU;
@group(0) @binding(1) var dist_tex: texture_2d<f32>;
@group(0) @binding(2) var cloud_col: texture_2d<f32>;
@group(0) @binding(3) var cloud_dep: texture_2d<f32>;

struct BoltV {
    @builtin(position) pos: vec4<f32>,
    @location(0) k: vec3<f32>,
    @location(1) dist: f32,
};

@vertex
fn vs_bolt(@location(0) p: vec3<f32>, @location(1) k: vec3<f32>) -> BoltV {
    var o: BoltV;
    o.pos = B.view_proj * vec4<f32>(p, 1.0);
    o.k = k;
    o.dist = length(p);
    return o;
}

@fragment
fn fs_bolt(v: BoltV) -> @location(0) vec4<f32> {
    let px = vec2<i32>(v.pos.xy);
    let dims = vec2<i32>(textureDimensions(dist_tex));
    let scene = textureLoad(dist_tex, clamp(px, vec2<i32>(0), dims - 1), 0).x;
    if (v.dist > scene + 40.0) {
        discard;
    }
    // Hidden by the cloud in front of it (half-resolution cloud result:
    // a = transmittance, depth.y = mean cloud distance).
    let hd = vec2<i32>(textureDimensions(cloud_col));
    let hp = clamp(px / 2, vec2<i32>(0), hd - 1);
    let cc = textureLoad(cloud_col, hp, 0);
    let cdist = textureLoad(cloud_dep, hp, 0).y;
    let behind = smoothstep(cdist - 400.0, cdist + 400.0, v.dist);
    let vis = mix(1.0, clamp(cc.a, 0.0, 1.0), behind);
    let x = abs(v.k.x);
    var prof = 1.0 - smoothstep(0.45, 1.0, x);
    if (v.k.z > 0.5) {
        prof = exp(-x * x * 5.0);
    }
    let col = vec3<f32>(0.84, 0.88, 1.0);
    return vec4<f32>(col * v.k.y * prof * vis, 0.0);
}

// ------------------------------------------------------------------ windscreen

struct GlassU {
    inv_view_proj: mat4x4<f32>, // eye-relative
    screen: vec4<f32>,          // w, h, 1/w, 1/h
    state: vec4<f32>,           // time, rain 0..1, airspeed m/s, wet time s
    state2: vec4<f32>,          // fog, frost, ice, focal length (px)
    wiper: vec4<f32>,           // phase 0..1, running, stroke s, since stop s
    // Per pane: origin (eye-relative, w = valid), normal, u, v, (lo.x, lo.y, hi.x, hi.y),
    // (wiper pivot u, v, arm length (0 = none), park angle x sweep sign).
    panes: array<array<vec4<f32>, 6>, 2>,
};
@group(0) @binding(5) var<uniform> G: GlassU;
@group(0) @binding(6) var scene_tex: texture_2d<f32>;
@group(0) @binding(7) var gdist_tex: texture_2d<f32>;
@group(0) @binding(8) var lin: sampler;
@group(0) @binding(9) var glass_tex: texture_2d<f32>;

const SWEEP: f32 = 1.75; // == glass.rs SWEEP
const TAU: f32 = 6.283185307;

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

fn hash4(c: vec2<i32>, k: u32) -> vec4<f32> {
    var v = vec4<u32>(bitcast<u32>(c.x), bitcast<u32>(c.y), k, k * 7u + 3u) * 1664525u + 1013904223u;
    v.x += v.y * v.w;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v.w += v.y * v.z;
    v ^= v >> vec4<u32>(16u);
    v.x += v.y * v.w;
    v.y += v.z * v.x;
    v.z += v.x * v.y;
    v.w += v.y * v.z;
    return vec4<f32>(v >> vec4<u32>(8u)) * (1.0 / 16777216.0);
}

fn vnoise(p: vec2<f32>) -> f32 {
    let i = vec2<i32>(floor(p));
    let f = fract(p);
    let u = f * f * (3.0 - 2.0 * f);
    let a = hash4(i, 91u).x;
    let b = hash4(i + vec2<i32>(1, 0), 91u).x;
    let c = hash4(i + vec2<i32>(0, 1), 91u).x;
    let d = hash4(i + vec2<i32>(1, 1), 91u).x;
    return mix(mix(a, b, u.x), mix(c, d, u.x), u.y);
}

// Seconds since the blade last passed angle a (== Wipers::age_at).
fn wiper_age(a: f32) -> f32 {
    let stroke = max(G.wiper.z, 0.3);
    let x = clamp(1.0 - 2.0 * clamp(a / SWEEP, 0.0, 1.0), -1.0, 1.0);
    let p1 = acos(x) / TAU;
    let p2 = 1.0 - p1;
    if (G.wiper.y < 0.5) {
        return max(G.wiper.w, 0.0) + (1.0 - p2) * stroke;
    }
    let ph = G.wiper.x;
    let b1 = select(ph - p1 + 1.0, ph - p1, ph >= p1);
    let b2 = select(ph - p2 + 1.0, ph - p2, ph >= p2);
    return min(b1, b2) * stroke;
}

struct Drop {
    off: vec2<f32>, // refraction offset (pane metres)
    mask: f32,
    rim: f32,
    spec: f32,
};

// One layer of drops on a grid of `cs` metres: radius up to `rmax`, present
// where the glass has been collecting rain for long enough (`fill` 0..1),
// stretched / slid along the pane's v axis by the airflow.
fn drop_layer(q: vec2<f32>, cs: f32, rmax: f32, k: u32, fill: f32, stretch: f32) -> Drop {
    var o: Drop;
    o.off = vec2<f32>(0.0);
    o.mask = 0.0;
    o.rim = 0.0;
    o.spec = 0.0;
    let p = q / cs;
    let id = vec2<i32>(floor(p));
    for (var dy = -1; dy <= 1; dy++) {
        for (var dx = -1; dx <= 1; dx++) {
            let cid = id + vec2<i32>(dx, dy);
            let h = hash4(cid, k);
            if (h.x > fill) {
                continue;
            }
            let c = vec2<f32>(cid) + 0.15 + 0.7 * h.yz;
            let r = rmax * (0.3 + 0.7 * h.w * h.w) / cs;
            var d = p - c;
            d.y /= stretch;
            let dd = length(d) / r;
            if (dd < 1.0 && (1.0 - dd) > o.mask) {
                let n = d / r;
                // A lens: the view through it is shrunk and flipped.
                o.off = -n * r * cs * 1.6;
                o.mask = smoothstep(1.0, 0.82, dd);
                o.rim = smoothstep(0.55, 1.0, dd);
                o.spec = pow(max(dot(n, vec2<f32>(-0.45, 0.6)), 0.0), 6.0) * (1.0 - dd);
            }
        }
    }
    return o;
}

fn scene_at(uv: vec2<f32>) -> vec3<f32> {
    return textureSampleLevel(scene_tex, lin, uv, 0.0).rgb;
}

@fragment
fn fs_glass(in: FsOut) -> @location(0) vec4<f32> {
    let px = vec2<i32>(in.pos.xy);
    let uv = (vec2<f32>(px) + 0.5) * G.screen.zw;
    let ndc = vec2<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0);
    let pr = G.inv_view_proj * vec4<f32>(ndc, 0.5, 1.0);
    let rd = normalize(pr.xyz / pr.w);
    let scene_d = textureLoad(gdist_tex, px, 0).x;
    var pane = -1;
    var q = vec2<f32>(0.0);
    var t_hit = 0.0;
    var edge = 0.0;
    for (var i = 0; i < 2; i++) {
        let P = G.panes[i];
        if (P[0].w < 0.5) {
            continue;
        }
        let n = P[1].xyz;
        let den = dot(rd, n);
        if (abs(den) < 1e-4) {
            continue;
        }
        let t = dot(P[0].xyz, n) / den;
        if (t < 0.05 || t > 4.0 || scene_d < t - 0.06) {
            continue;
        }
        let h = rd * t - P[0].xyz;
        let qq = vec2<f32>(dot(h, P[2].xyz), dot(h, P[3].xyz));
        let e = min(min(qq.x - P[4].x, P[4].z - qq.x), min(qq.y - P[4].y, P[4].w - qq.y));
        if (e > -0.005) {
            pane = i;
            q = qq;
            t_hit = t;
            edge = e;
        }
    }
    if (pane < 0) {
        return vec4<f32>(0.0);
    }
    let P = G.panes[pane];
    let time = G.state.x;
    let rain = G.state.y;
    let speed = G.state.z;
    // How long this spot has been collecting rain: since the blade last
    // passed (wiped area), else since the rain started.
    var age = G.state.w;
    let piv = P[5];
    if (piv.z > 0.0) {
        let rel = q - piv.xy;
        let r = length(rel);
        let ang = atan2(rel.y, rel.x);
        let a0 = abs(piv.w);
        var a = select(a0 - ang, ang - a0, piv.w > 0.0);
        a = a - TAU * round(a / TAU);
        if (r < piv.z * 1.03 && r > piv.z * 0.1 && a > -0.04 && a < SWEEP + 0.04) {
            age = min(age, wiper_age(clamp(a, 0.0, SWEEP)));
        }
    }
    // Airflow: at speed the drops are blown up the glass and stretched into streaks.
    let blow = smoothstep(15.0, 60.0, speed);
    let slide = vec2<f32>(0.0, time * (0.02 + 0.35 * blow));
    let stretch = 1.0 + 2.5 * blow;
    let rate = rain * 0.9 + 0.02;
    let fill_big = clamp(1.0 - exp(-rate * age * 0.45), 0.0, 0.85) * step(0.004, rain + 0.002 * min(G.state.w, 1.0));
    let fill_small = clamp(1.0 - exp(-rate * age * 1.6), 0.0, 0.9) * step(0.004, rain + 0.002 * min(G.state.w, 1.0));
    let big = drop_layer(q + slide, 0.016, 0.0045 * (1.0 - 0.4 * blow), 7u, fill_big, stretch);
    let small = drop_layer(q + slide * 1.3, 0.0065, 0.0016, 13u, fill_small, 1.0 + 1.2 * blow);
    // Pane metres → screen uv: d(screen) ≈ d(pane) / t × focal.
    let m2uv = G.state2.w / max(t_hit, 0.1) * G.screen.zw;
    var off = vec2<f32>(0.0);
    var mask = 0.0;
    var rim = 0.0;
    var spec = 0.0;
    if (big.mask > small.mask) {
        off = big.off;
        mask = big.mask;
        rim = big.rim;
        spec = big.spec;
    } else {
        off = small.off;
        mask = small.mask;
        rim = small.rim;
        spec = small.spec;
    }
    // A thin wet film everywhere it rains (ripples), cleared by the blade.
    let film = rain * smoothstep(0.0, 1.5, age);
    let wob = (vec2<f32>(vnoise(q * 90.0 + vec2<f32>(0.0, time * 3.0)), vnoise(q * 90.0 + vec2<f32>(17.0, time * 3.0))) - 0.5) * 0.004 * film;
    let suv = uv + vec2<f32>(off.x, -off.y) * m2uv * mask + wob * m2uv;
    var col = scene_at(suv);
    // Drops: slightly darker image, dark refracting rim, a sky glint.
    let sky = scene_at(vec2<f32>(uv.x, max(uv.y - 0.25, 0.02)));
    col = mix(col, col * 0.82 + sky * 0.05, mask * 0.6);
    col = mix(col, col * 0.35, rim * mask * 0.8);
    col += sky * spec * mask * 1.4;
    // Misting (inside) and frost / ice (creeping in from the frame).
    let fog = G.state2.x;
    let fr = max(G.state2.y, G.state2.z);
    var blur = col;
    if (fog > 0.01 || fr > 0.01) {
        var acc = vec3<f32>(0.0);
        for (var k = 0; k < 8; k++) {
            let a = f32(k) * 0.785 + 0.3;
            acc += scene_at(uv + vec2<f32>(cos(a), sin(a)) * 9.0 * G.screen.zw);
        }
        blur = acc / 8.0;
    }
    if (fog > 0.01) {
        let pattern = clamp(fog * (0.7 + 0.6 * vnoise(q * 12.0)) * (1.0 - 0.5 * smoothstep(0.1, 0.4, (q.y - P[4].y) / max(P[4].w - P[4].y, 0.1))), 0.0, 1.0);
        let haze = vec3<f32>(dot(blur, vec3<f32>(0.2126, 0.7152, 0.0722))) * 0.9 + blur * 0.25;
        col = mix(col, haze, pattern);
    }
    if (fr > 0.01) {
        let n = vnoise(q * 40.0) * 0.6 + vnoise(q * 160.0) * 0.3 + vnoise(q * 600.0) * 0.1;
        let reach = fr * 0.32;
        let m = 1.0 - smoothstep(reach - 0.05, reach + 0.01, edge + (n - 0.5) * 0.09 * (0.5 + fr));
        let crystals = smoothstep(0.55, 0.75, vnoise(q * 900.0));
        let lum = dot(blur, vec3<f32>(0.2126, 0.7152, 0.0722));
        let frost_col = blur * 0.3 + vec3<f32>(0.82, 0.88, 0.95) * lum * (0.9 + 0.6 * crystals);
        col = mix(col, frost_col, clamp(m, 0.0, 1.0) * 0.92);
    }
    // Soft pane edge.
    let a = smoothstep(-0.005, 0.01, edge);
    return vec4<f32>(col, a);
}

@fragment
fn fs_blit(in: FsOut) -> @location(0) vec4<f32> {
    return textureLoad(glass_tex, vec2<i32>(in.pos.xy), 0);
}
