// Textured aircraft (imported FlightGear models, see gfx_aircraft.rs).
// Appended to atmos_core + common + scene.wgsl: reuses the scene's lighting
// (`light`, `Surf`, fog, shadows) so the aircraft sits in the same world.
// Group 0 = scene globals, group 1 = per-draw instances, group 2 = texture.

struct AcInst {
    model: mat4x4<f32>,   // node → eye-relative world
    color: vec4<f32>,     // material diffuse (linear) + opacity
    emis: vec4<f32>,      // emission (scene radiance units), w = interior flag
    params: vec4<f32>,    // x = f0 (specular), y = roughness, z = alpha mode (0 opaque, 1 cut-out, 2 translucent), w = textured
};

@group(1) @binding(0) var<storage, read> ac_inst: array<AcInst>;
@group(2) @binding(0) var ac_tex: texture_2d<f32>;
@group(2) @binding(1) var ac_samp: sampler;

struct AcIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) uv: vec2<f32>,
};

struct AcOut {
    @builtin(position) clip: vec4<f32>,
    @location(0) wpos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) uv: vec2<f32>,
    @location(3) @interpolate(flat) inst: u32,
};

@vertex
fn vs_aircraft(v: AcIn, @builtin(instance_index) ii: u32) -> AcOut {
    let m = ac_inst[ii].model;
    let p = (m * vec4<f32>(v.pos, 1.0)).xyz;
    var o: AcOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = (m * vec4<f32>(v.nrm, 0.0)).xyz;
    o.uv = v.uv;
    o.inst = ii;
    return o;
}

@fragment
fn fs_aircraft(in: AcOut) -> FOut {
    let inst = ac_inst[in.inst];
    let tex = textureSample(ac_tex, ac_samp, in.uv);
    let rel = in.wpos;
    let px = in.clip.xy;
    let dist = length(rel);
    let V = -rel / max(dist, 1e-4);
    var ngeo = normalize(in.nrm);
    let dndx = dpdx(ngeo);
    let dndy = dpdy(ngeo);
    // Two-sided surfaces (and the inside of the cockpit) face the viewer.
    if (dot(ngeo, V) < 0.0) {
        ngeo = -ngeo;
    }
    var albedo = inst.color.rgb;
    var alpha = inst.color.a;
    if (inst.params.w > 0.5) {
        albedo *= tex.rgb;
        alpha *= tex.a;
    }
    let mode = inst.params.z;
    if (mode > 0.5 && mode < 1.5 && alpha < 0.45) {
        discard;
    }
    if (is_refl() && alpha < 0.5) {
        discard;
    }
    var s = default_surf(albedo, ngeo);
    s.f0 = vec3<f32>(inst.params.x);
    s.rough = spec_aa(inst.params.y, dndx, dndy);
    s.moving = 1.0;
    s.emit = inst.emis.rgb;
    s.alpha = select(1.0, alpha, mode > 0.5);
    if (mode > 1.5) {
        // Glass: mostly reflection, little body colour.
        s.albedo *= alpha;
    }
    let col = light(s, rel, V, dist, px, ngeo);
    var o: FOut;
    o.col = vec4<f32>(apply_fog_px(col, px, dist), s.alpha);
    o.dist = dist;
    return o;
}
