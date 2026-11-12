// Copy of the HDR scene for the heat-haze pass (vfx/gpu.rs).

struct BOut {
    @builtin(position) pos: vec4<f32>,
};

@vertex
fn vs_blit(@builtin(vertex_index) i: u32) -> BOut {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    var o: BOut;
    o.pos = vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
    return o;
}

@group(0) @binding(0) var blit_src: texture_2d<f32>;

@fragment
fn fs_blit(in: BOut) -> @location(0) vec4<f32> {
    return textureLoad(blit_src, vec2<i32>(in.pos.xy), 0);
}
