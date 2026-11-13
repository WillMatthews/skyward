// Textured aircraft (imported FlightGear models, see gfx_aircraft.rs).
// Appended to atmos_core + common + scene.wgsl: reuses the scene's lighting
// (`light`, `Surf`, fog, shadows) so the aircraft sits in the same world.
// Group 0 = scene globals, group 1 = per-draw instances, group 2 = texture.

struct AcInst {
    model: mat4x4<f32>,   // node → eye-relative world
    color: vec4<f32>,     // material diffuse (linear) + opacity
    emis: vec4<f32>,      // emission (scene radiance units), w = interior flag
    params: vec4<f32>,    // x = f0 (specular), y = roughness, z = mode (0 opaque, 1 cut-out, 2 translucent, 3 glass display), w = textured (display: luminance)
    uvm: vec4<f32>,       // UV transform 2x2 (column-major): drum counters, attitude balls, display windows
    uvo: vec4<f32>,       // UV offset (xy)
};

@group(1) @binding(0) var<storage, read> ac_inst: array<AcInst>;
@group(1) @binding(1) var<storage, read> ac_dyn: AcDyn;
@group(2) @binding(0) var ac_tex: texture_2d<f32>;
@group(2) @binding(1) var ac_samp: sampler;

struct AcIn {
    @location(0) pos: vec3<f32>,
    @location(1) nrm: vec3<f32>,
    @location(2) uv: vec2<f32>,
    // Structural part, signed span station, chord offset, exterior (models.rs).
    @location(3) part: vec4<f32>,
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
    let k = u32(v.part.x + 0.5);
    let pm = ac_dyn.parts[min(k, 15u)];
    let p = (pm * vec4<f32>((m * vec4<f32>(v.pos, 1.0)).xyz + ac_flex(v.part), 1.0)).xyz;
    var o: AcOut;
    o.clip = g.view_proj * vec4<f32>(p, 1.0);
    o.wpos = p + vec3<f32>(0.0, g.pattern.w, 0.0);
    o.nrm = (pm * vec4<f32>(ac_flex_normal(v.part, (m * vec4<f32>(v.nrm, 0.0)).xyz, normalize(m[0].xyz)), 0.0)).xyz;
    let t = ac_inst[ii];
    o.uv = vec2<f32>(t.uvm.x * v.uv.x + t.uvm.z * v.uv.y, t.uvm.y * v.uv.x + t.uvm.w * v.uv.y) + t.uvo.xy;
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
    if (mode > 2.5) {
        // Glass display: the rendered screen (sRGB view → linear) as emission
        // behind smooth cover glass, which still reflects the sky and the sun.
        var s = default_surf(vec3<f32>(0.012), ngeo);
        s.f0 = vec3<f32>(inst.params.x);
        s.rough = spec_aa(inst.params.y, dndx, dndy);
        s.moving = 1.0;
        s.emit = tex.rgb * inst.params.w;
        let col = light(s, rel, V, dist, px, ngeo);
        var o: FOut;
        o.col = vec4<f32>(apply_fog_px(col, px, dist), 1.0);
        o.dist = dist;
        return o;
    }
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
    // Airframe ice (wx/): rough white rime building back from the leading
    // edges (surfaces facing the airflow), on the outside only.
    let ice = ac_dyn.vib.z;
    if (ice > 0.001 && inst.emis.w < 0.5 && mode < 1.5) {
        let fwd = normalize(inst.model[2].xyz);
        let facing = dot(normalize(in.nrm), fwd);
        let grain = fract(sin(dot(floor(in.wpos * 60.0), vec3<f32>(12.9898, 78.233, 37.719))) * 43758.5453);
        let m = smoothstep(0.92 - 0.55 * ice, 1.0 - 0.45 * ice, max(facing, 0.0) * (0.9 + 0.2 * grain)) * min(ice * 2.5, 1.0);
        s.albedo = mix(s.albedo, vec3<f32>(0.78, 0.82, 0.86) * (0.85 + 0.25 * grain), m);
        s.f0 = mix(s.f0, vec3<f32>(0.02), m);
        s.rough = mix(s.rough, 0.75, m);
    }
    // Cockpit backlighting lights the painted markings, not the black panel.
    s.emit = select(inst.emis.rgb, inst.emis.rgb * albedo, inst.emis.w > 0.5);
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

