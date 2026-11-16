//! The bathymetry on the GPU: a two-layer `Rg16Float` array texture (layer
//! 0 the fine region, 1 the coarse one; texels hold depth × weight, weight)
//! and the uniforms that place it (scene.wgsl `bathy_at`).

use super::{Image, Region};
use crate::osm::Proj;
use glam::Vec3;

pub struct BathyGpu {
    tex: wgpu::Texture,
    view: wgpu::TextureView,
    res: u32,
    layers: [Option<Region>; 2],
}

impl BathyGpu {
    pub fn new(device: &wgpu::Device) -> Self {
        // Without bathymetry a 1×1 stand-in (never sampled: the layers are off).
        let res = if super::enabled() { super::RES } else { 1 };
        let tex = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("bathymetry"),
            size: wgpu::Extent3d { width: res, height: res, depth_or_array_layers: 2 },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rg16Float,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });
        let view = tex.create_view(&wgpu::TextureViewDescriptor { dimension: Some(wgpu::TextureViewDimension::D2Array), ..Default::default() });
        BathyGpu { tex, view, res, layers: [None, None] }
    }

    pub fn view(&self) -> &wgpu::TextureView {
        &self.view
    }

    /// Upload finished region images (call once per frame).
    pub fn update(&mut self, queue: &wgpu::Queue) {
        for img in super::take_ready() {
            self.upload(queue, &img);
        }
    }

    fn upload(&mut self, queue: &wgpu::Queue, img: &Image) {
        if img.res != self.res {
            return;
        }
        let l = img.region.level as u32;
        queue.write_texture(
            wgpu::TexelCopyTextureInfo { texture: &self.tex, mip_level: 0, origin: wgpu::Origin3d { x: 0, y: 0, z: l }, aspect: wgpu::TextureAspect::All },
            bytemuck::cast_slice(&img.texels),
            wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(self.res * 4), rows_per_image: Some(self.res) },
            wgpu::Extent3d { width: self.res, height: self.res, depth_or_array_layers: 1 },
        );
        self.layers[l as usize] = Some(img.region);
    }

    /// Shader uniforms for an eye at `eye` (current frame of `proj`):
    /// per layer its north-west corner relative to the eye (x, z) and
    /// 1 / its size (x, z; 0 = no layer), then (water clarity, 0, 0, 0).
    pub fn params(&self, proj: &Proj, eye: Vec3) -> [[f32; 4]; 3] {
        let mut out = [[0.0; 4]; 3];
        for (k, r) in self.layers.iter().enumerate() {
            if let Some(p) = r.and_then(|r| place(proj, eye, &r)) {
                out[k] = p;
            }
        }
        let (lat, lon) = proj.to_latlon(glam::vec2(eye.x, eye.z));
        out[2][0] = super::clarity(lat, lon);
        out
    }
}

/// A region's placement: (corner x, z relative to the eye, 1 / size x, z).
pub fn place(proj: &Proj, eye: Vec3, r: &Region) -> Option<[f32; 4]> {
    let (w, s, e, n) = r.bounds();
    let (x0, z0) = proj.to_world64(n, w);
    let (x1, z1) = proj.to_world64(s, e);
    let (sx, sz) = (x1 - x0, z1 - z0);
    (sx > 1.0 && sz > 1.0).then(|| [(x0 - eye.x as f64) as f32, (z0 - eye.z as f64) as f32, (1.0 / sx) as f32, (1.0 / sz) as f32])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn placement_maps_corners_to_uv() {
        let proj = Proj::new(51.5, 0.5);
        let r = Region::around(0, 51.5, 0.9);
        let eye = Vec3::new(1000.0, 900.0, -2000.0);
        let p = place(&proj, eye, &r).unwrap();
        // The shader: uv = (rel.xz - corner) * inv.
        let uv = |lat: f64, lon: f64| {
            let (x, z) = proj.to_world64(lat, lon);
            (((x - eye.x as f64) as f32 - p[0]) * p[2], ((z - eye.z as f64) as f32 - p[1]) * p[3])
        };
        let (w, s, e, n) = r.bounds();
        let (u0, v0) = uv(n, w);
        let (u1, v1) = uv(s, e);
        assert!(u0.abs() < 1e-4 && v0.abs() < 1e-4, "{u0} {v0}");
        assert!((u1 - 1.0).abs() < 1e-4 && (v1 - 1.0).abs() < 1e-4, "{u1} {v1}");
        let (uc, vc) = uv(r.lat, r.lon);
        assert!((uc - 0.5).abs() < 1e-4 && (vc - 0.5).abs() < 1e-4);
        // About 96 km across.
        assert!((1.0 / p[3] - 96_000.0).abs() < 50.0, "{}", 1.0 / p[3]);
    }
}
