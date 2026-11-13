// Aircraft into the shadow cascades (see gfx_aircraft.rs). Same uniform as shadow.wgsl.
struct ShadowU {
    light_vp: mat4x4<f32>,
    season: vec4<f32>,
};
struct AcInst {
    model: mat4x4<f32>,
    color: vec4<f32>,
    emis: vec4<f32>,
    params: vec4<f32>,
    uvm: vec4<f32>,
    uvo: vec4<f32>,
};
@group(0) @binding(0) var<uniform> su: ShadowU;
@group(1) @binding(0) var<storage, read> ac_inst: array<AcInst>;
@group(1) @binding(1) var<storage, read> ac_dyn: AcDyn;

@vertex
fn vs_ac_shadow(@location(0) pos: vec3<f32>, @location(1) part: vec4<f32>, @builtin(instance_index) ii: u32) -> @builtin(position) vec4<f32> {
    let pm = ac_dyn.parts[min(u32(part.x + 0.5), 15u)];
    let p = (pm * vec4<f32>((ac_inst[ii].model * vec4<f32>(pos, 1.0)).xyz + ac_flex(part), 1.0)).xyz;
    return su.light_vp * vec4<f32>(p, 1.0);
}
