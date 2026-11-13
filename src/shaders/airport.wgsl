// Airport instances (airport/gfx.rs): parked aircraft and ground vehicles.
// Appended to atmos_core + common + scene.wgsl: vertex-coloured instances
// go through the scene's own fs_main (materials, lighting, fog), textured
// parked aircraft through fs_parked (the aircraft shader's surface model).
// Instances: an eye-relative model matrix per instance (locations 5..8).

struct InstIn {
    @location(5) m0: vec4<f32>,
    @location(6) m1: vec4<f32>,
    @location(7) m2: vec4<f32>,
    @location(8) m3: vec4<f32>,
};

fn inst_mat(i: InstIn) -> mat4x4<f32> {
    return mat4x4<f32>(i.m0, i.m1, i.m2, i.m3);
}

@vertex
fn vs_inst(v: VIn, i: InstIn) -> VOut {
    let m = inst_mat(i);
    let p = (m * vec4<f32>(v.pos, 1.0)).xyz;
    var o: VOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = normalize((m * vec4<f32>(v.nrm, 0.0)).xyz);
    o.col = v.col;
    o.uv = v.uv;
    o.mat = v.mat;
    o.tree = vec3<f32>(1.0, 0.0, 0.0);
    return o;
}

@group(1) @binding(0) var pk_tex: texture_2d<f32>;
@group(1) @binding(1) var pk_samp: sampler;

struct PkIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) uv: vec2<f32>,
    // Material colour (sRGB) + opacity; f0, roughness, alpha mode / 2, textured.
    @location(3) col: vec4<f32>,
    @location(4) params: vec4<f32>,
};

struct PkOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wpos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) col: vec4<f32>,
    @location(4) params: vec4<f32>,
};

@vertex
fn vs_parked(v: PkIn, i: InstIn) -> PkOut {
    let m = inst_mat(i);
    let p = (m * vec4<f32>(v.pos, 1.0)).xyz;
    var o: PkOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = (m * vec4<f32>(v.nrm, 0.0)).xyz;
    o.uv = v.uv;
    o.col = v.col;
    o.params = v.params;
    return o;
}

@fragment
fn fs_parked(in: PkOut) -> FOut {
    let tex = textureSample(pk_tex, pk_samp, in.uv);
    let rel = in.wpos;
    let px = in.clip.xy;
    let dist = length(rel);
    let V = -rel / max(dist, 1e-4);
    var ngeo = normalize(in.nrm);
    let dndx = dpdx(ngeo);
    let dndy = dpdy(ngeo);
    if (dot(ngeo, V) < 0.0) {
        ngeo = -ngeo;
    }
    var albedo = srgb_to_lin(in.col.rgb);
    var alpha = in.col.a;
    if (in.params.w > 0.5) {
        albedo *= tex.rgb;
        alpha *= tex.a;
    }
    let mode = in.params.z * 2.0;
    if (mode > 0.5 && mode < 1.5 && alpha < 0.45) {
        discard;
    }
    var s = default_surf(albedo, ngeo);
    s.f0 = vec3<f32>(in.params.x);
    s.rough = spec_aa(max(in.params.y, 0.05), dndx, dndy);
    if (mode > 1.5) {
        // Windows / canopies: dark tinted glass, mostly reflection.
        s.albedo = albedo * 0.08 + vec3<f32>(0.008, 0.01, 0.012);
        s.rough = 0.05;
        s.f0 = vec3<f32>(0.06);
    }
    let col = light(s, rel, V, dist, px, ngeo);
    var o: FOut;
    o.col = vec4<f32>(apply_fog_px(col, px, dist), 1.0);
    o.dist = dist;
    return o;
}
