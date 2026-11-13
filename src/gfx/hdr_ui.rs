//! HDR output (scRGB swapchain, 1.0 = 80 nits): the interface is drawn into
//! an sRGB texture and composited over the scene at the paper-white level —
//! egui's colours are display-referred and would be far too dim (and
//! gamma-wrong) written straight into a linear HDR target.

/// Format the interface is drawn in.
pub const UI_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8UnormSrgb;

const SHADER: &str = r#"
@group(0) @binding(0) var ui_tex: texture_2d<f32>;
@group(0) @binding(1) var<uniform> k: vec4<f32>;

@vertex
fn vs_full(@builtin(vertex_index) i: u32) -> @builtin(position) vec4<f32> {
    let uv = vec2<f32>(f32((i << 1u) & 2u), f32(i & 2u));
    return vec4<f32>(uv * 2.0 - 1.0, 0.0, 1.0);
}

@fragment
fn fs_comp(@builtin(position) pos: vec4<f32>) -> @location(0) vec4<f32> {
    // Premultiplied (egui blends premultiplied), linear (sRGB view): scale to paper white.
    let c = textureLoad(ui_tex, vec2<i32>(pos.xy), 0);
    return vec4<f32>(c.rgb * k.x, c.a);
}
"#;

pub struct HdrUi {
    pipe: wgpu::RenderPipeline,
    ubuf: wgpu::Buffer,
    tex: Option<(wgpu::TextureView, u32, u32)>,
}

impl HdrUi {
    pub fn new(device: &wgpu::Device, out: wgpu::TextureFormat) -> HdrUi {
        let module = device.create_shader_module(wgpu::ShaderModuleDescriptor { label: Some("hdr ui"), source: wgpu::ShaderSource::Wgsl(SHADER.into()) });
        let premul = wgpu::BlendComponent { src_factor: wgpu::BlendFactor::One, dst_factor: wgpu::BlendFactor::OneMinusSrcAlpha, operation: wgpu::BlendOperation::Add };
        let pipe = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("hdr ui"),
            layout: None,
            vertex: wgpu::VertexState { module: &module, entry_point: Some("vs_full"), compilation_options: Default::default(), buffers: &[] },
            primitive: Default::default(),
            depth_stencil: None,
            multisample: Default::default(),
            fragment: Some(wgpu::FragmentState {
                module: &module,
                entry_point: Some("fs_comp"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState { format: out, blend: Some(wgpu::BlendState { color: premul, alpha: premul }), write_mask: wgpu::ColorWrites::ALL })],
            }),
            multiview_mask: None,
            cache: None,
        });
        let ubuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("hdr ui"),
            size: 16,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        HdrUi { pipe, ubuf, tex: None }
    }

    /// The interface target (`w`×`h`, cleared by the caller's pass).
    pub fn target(&mut self, device: &wgpu::Device, w: u32, h: u32) -> wgpu::TextureView {
        if self.tex.as_ref().is_none_or(|t| t.1 != w || t.2 != h) {
            let view = device
                .create_texture(&wgpu::TextureDescriptor {
                    label: Some("hdr ui"),
                    size: wgpu::Extent3d { width: w.max(1), height: h.max(1), depth_or_array_layers: 1 },
                    mip_level_count: 1,
                    sample_count: 1,
                    dimension: wgpu::TextureDimension::D2,
                    format: UI_FORMAT,
                    usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
                    view_formats: &[],
                })
                .create_view(&Default::default());
            self.tex = Some((view, w, h));
        }
        self.tex.as_ref().unwrap().0.clone()
    }

    /// Composite the interface over `out` with white at `paper_white` nits.
    pub fn composite(&self, device: &wgpu::Device, queue: &wgpu::Queue, encoder: &mut wgpu::CommandEncoder, out: &wgpu::TextureView, paper_white: f32) {
        let Some((view, _, _)) = &self.tex else { return };
        queue.write_buffer(&self.ubuf, 0, bytemuck::cast_slice(&[paper_white / 80.0, 0.0f32, 0.0, 0.0]));
        let bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("hdr ui"),
            layout: &self.pipe.get_bind_group_layout(0),
            entries: &[
                wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(view) },
                wgpu::BindGroupEntry { binding: 1, resource: self.ubuf.as_entire_binding() },
            ],
        });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("hdr ui"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: out,
                depth_slice: None,
                resolve_target: None,
                ops: wgpu::Operations { load: wgpu::LoadOp::Load, store: wgpu::StoreOp::Store },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
            multiview_mask: None,
        });
        pass.set_pipeline(&self.pipe);
        pass.set_bind_group(0, &bg, &[]);
        pass.draw(0..3, 0..1);
    }
}
