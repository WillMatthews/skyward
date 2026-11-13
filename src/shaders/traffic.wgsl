// Road traffic (traffic/gpu.rs): instanced vehicles through the scene's own
// fs_main (materials, lighting, shadows, fog). Appended to atmos_core +
// common + scene.wgsl.
//
// Instances (traffic::VehInst): eye-relative position (this frame),
// heading (unit x, z), road grade, body colour (sRGB rgba8),
// kind | flags << 8 | seed << 16.

struct TInst {
    @location(5) pos: vec3<f32>,
    @location(6) dir: vec2<f32>,
    @location(7) grade: f32,
    @location(8) col: u32,
    @location(9) info: u32,
};

// Body / trailer colours for box vans and lorries (picked by seed).
fn box_colour(seed: u32) -> vec3<f32> {
    var c = array<vec3<f32>, 16>(
        vec3<f32>(0.88, 0.88, 0.87), vec3<f32>(0.84, 0.84, 0.83), vec3<f32>(0.66, 0.67, 0.68), vec3<f32>(0.88, 0.88, 0.87),
        vec3<f32>(0.80, 0.80, 0.79), vec3<f32>(0.86, 0.86, 0.85), vec3<f32>(0.52, 0.53, 0.55), vec3<f32>(0.88, 0.88, 0.87),
        vec3<f32>(0.84, 0.84, 0.83), vec3<f32>(0.22, 0.30, 0.45), vec3<f32>(0.88, 0.88, 0.87), vec3<f32>(0.45, 0.14, 0.12),
        vec3<f32>(0.80, 0.80, 0.79), vec3<f32>(0.25, 0.36, 0.28), vec3<f32>(0.70, 0.62, 0.40), vec3<f32>(0.30, 0.31, 0.33));
    return c[(seed >> 3u) & 15u];
}

// Lamp luminance (cd/m²) by lamp tag: 1 headlamp, 2 tail / brake, 3 amber sign.
fn lamp_lum(tag: f32, braking: bool) -> f32 {
    let on = g.city.x;
    if (tag < 1.5) {
        // Daytime running lights, dipped beams once the street lights are on.
        return mix(6000.0, 40000.0, on);
    }
    if (tag < 2.5) {
        return mix(0.0, 1400.0, on) + select(0.0, 7000.0, braking);
    }
    return 1600.0 * on;
}

struct VehFrame {
    origin: vec3<f32>,   // eye-relative
    left: vec3<f32>,
    up: vec3<f32>,
    fwd: vec3<f32>,
};

fn veh_frame(i: TInst) -> VehFrame {
    let h = vec3<f32>(i.dir.x, 0.0, i.dir.y);
    var f: VehFrame;
    f.fwd = normalize(h + vec3<f32>(0.0, clamp(i.grade, -0.3, 0.3), 0.0));
    f.left = normalize(cross(vec3<f32>(0.0, 1.0, 0.0), f.fwd));
    f.up = cross(f.fwd, f.left);
    f.origin = i.pos;
    return f;
}

@vertex
fn vs_vehicle(v: VIn, i: TInst) -> VOut {
    let f = veh_frame(i);
    let p = f.origin + f.left * v.pos.x + f.up * v.pos.y + f.fwd * v.pos.z;
    var o: VOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = f.left * v.nrm.x + f.up * v.nrm.y + f.fwd * v.nrm.z;
    var col = v.col;
    if (col.x < -1.5) {
        col = box_colour(i.info >> 16u);
    } else if (col.x < -0.5) {
        col = unpack4x8unorm(i.col).rgb;
    }
    o.col = col;
    o.uv = v.uv;
    o.mat = v.mat;
    if (v.mat > 12.5 && v.mat < 13.5 && v.uv.y > 0.5) {
        let braking = ((i.info >> 8u) & 1u) != 0u;
        o.uv = vec2<f32>(lamp_lum(v.uv.y, braking) * LUX * 0.5, 0.0);
    }
    o.tree = vec3<f32>(1.0, 0.0, 0.0);
    return o;
}
