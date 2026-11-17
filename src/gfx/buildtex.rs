//! Building facade / roof texture arrays (buildmat.rs, scene.wgsl): albedo
//! (sRGB, 512 px) and normal + roughness (256 px), one layer per material,
//! with full mip chains. Natively they load at startup from
//! `assets/buildings/`; the browser downloads that directory and uploads it
//! when it arrives (neutral grey until then). With
//! `SKYWARD_BUILDING_TEX=0` the arrays are 1 px placeholders.

use crate::buildmat::{ALBEDO_PX, LAYERS, NORMAL_PX};

pub struct BuildingTextures {
    albedo: wgpu::Texture,
    normal: wgpu::Texture,
    pub albedo_view: wgpu::TextureView,
    pub normal_view: wgpu::TextureView,
    pub samp: wgpu::Sampler,
    /// Layers uploaded (or nothing more to do).
    done: bool,
    bytes: u64,
}

fn levels(px: u32) -> u32 {
    32 - px.leading_zeros()
}

/// Box-filtered mip chain of linear data (normals / roughness).
fn mips_linear(w: u32, rgba: &[u8]) -> Vec<Vec<u8>> {
    let mut out = Vec::new();
    let (mut cur, mut cw) = (rgba.to_vec(), w);
    while cw > 1 {
        let nw = cw / 2;
        let mut next = vec![0u8; (nw * nw * 4) as usize];
        for y in 0..nw {
            for x in 0..nw {
                for k in 0..4 {
                    let at = |dx: u32, dy: u32| cur[(((y * 2 + dy) * cw + x * 2 + dx) * 4 + k) as usize] as u32;
                    next[((y * nw + x) * 4 + k) as usize] = ((at(0, 0) + at(1, 0) + at(0, 1) + at(1, 1) + 2) / 4) as u8;
                }
            }
        }
        out.push(next.clone());
        cur = next;
        cw = nw;
    }
    out
}

/// A layer image as RGBA8 of the expected square size.
fn decode(path: &str, px: u32) -> Option<Vec<u8>> {
    #[cfg(target_arch = "wasm32")]
    if let Some((w, h, rgba, _)) = crate::platform::web::take_decoded_texture(path) {
        return (w == px && h == px).then_some(rgba);
    }
    let bytes = crate::assets::read(path).ok()?;
    let img = image::load_from_memory(&bytes).ok()?.to_rgba8();
    (img.width() == px && img.height() == px).then(|| img.into_raw())
}

impl BuildingTextures {
    pub fn new(device: &wgpu::Device, queue: &wgpu::Queue) -> Self {
        let on = crate::buildmat::enabled();
        let n = if on { LAYERS.len() as u32 } else { 1 };
        let (ap, np) = if on { (ALBEDO_PX, NORMAL_PX) } else { (1, 1) };
        let make = |label: &str, px: u32, format: wgpu::TextureFormat| {
            device.create_texture(&wgpu::TextureDescriptor {
                label: Some(label),
                size: wgpu::Extent3d { width: px, height: px, depth_or_array_layers: n },
                mip_level_count: levels(px),
                sample_count: 1,
                dimension: wgpu::TextureDimension::D2,
                format,
                usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
                view_formats: &[],
            })
        };
        let albedo = make("building albedo", ap, wgpu::TextureFormat::Rgba8UnormSrgb);
        let normal = make("building normal", np, wgpu::TextureFormat::Rgba8Unorm);
        let view = |t: &wgpu::Texture| t.create_view(&wgpu::TextureViewDescriptor { dimension: Some(wgpu::TextureViewDimension::D2Array), ..Default::default() });
        let samp = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("building tex"),
            address_mode_u: wgpu::AddressMode::Repeat,
            address_mode_v: wgpu::AddressMode::Repeat,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::MipmapFilterMode::Linear,
            anisotropy_clamp: 8,
            ..Default::default()
        });
        let bytes = |px: u32| (px as u64 * px as u64 * 4 * 4 / 3) * n as u64;
        let mut t = BuildingTextures { albedo_view: view(&albedo), normal_view: view(&normal), albedo, normal, samp, done: false, bytes: bytes(ap) + bytes(np) };
        // Neutral until the real layers arrive.
        for l in 0..n {
            t.write(queue, l, &vec![150u8; (ap * ap * 4) as usize], &vec![128u8; (np * np * 4) as usize], false);
        }
        if !on {
            t.done = true;
        }
        #[cfg(not(target_arch = "wasm32"))]
        t.load(queue);
        t
    }

    /// One layer's albedo (sRGB) and normal + roughness, level 0; mips built here.
    fn write(&self, queue: &wgpu::Queue, layer: u32, albedo: &[u8], normal: &[u8], real: bool) {
        let up = |tex: &wgpu::Texture, px: u32, l0: &[u8], mips: Vec<Vec<u8>>| {
            for (lvl, data) in std::iter::once(l0).chain(mips.iter().map(|m| m.as_slice())).enumerate() {
                let s = (px >> lvl).max(1);
                queue.write_texture(
                    wgpu::TexelCopyTextureInfo { texture: tex, mip_level: lvl as u32, origin: wgpu::Origin3d { x: 0, y: 0, z: layer }, aspect: wgpu::TextureAspect::All },
                    data,
                    wgpu::TexelCopyBufferLayout { offset: 0, bytes_per_row: Some(s * 4), rows_per_image: Some(s) },
                    wgpu::Extent3d { width: s, height: s, depth_or_array_layers: 1 },
                );
            }
        };
        let (ap, np) = (self.albedo.width(), self.normal.width());
        // (Placeholders are uniform: their mips are the same bytes, smaller.)
        let amips = if real { crate::gfx_aircraft::mip_chain(ap, ap, albedo) } else { (1..levels(ap)).map(|i| albedo[..((ap >> i) * (ap >> i) * 4) as usize].to_vec()).collect() };
        let nmips = if real { mips_linear(np, normal) } else { (1..levels(np)).map(|i| normal[..((np >> i) * (np >> i) * 4) as usize].to_vec()).collect() };
        up(&self.albedo, ap, albedo, amips);
        up(&self.normal, np, normal, nmips);
    }

    /// Decode and upload every layer (a missing / bad file keeps its placeholder).
    fn load(&mut self, queue: &wgpu::Queue) {
        if self.done {
            return;
        }
        let mut missing = Vec::new();
        for (i, (name, _)) in LAYERS.iter().enumerate() {
            let a = decode(&format!("buildings/{name}.jpg"), ALBEDO_PX);
            let n = decode(&format!("buildings/{name}_n.jpg"), NORMAL_PX);
            match (a, n) {
                (Some(a), Some(n)) => self.write(queue, i as u32, &a, &n, true),
                _ => missing.push(*name),
            }
        }
        self.done = true;
        if !missing.is_empty() {
            eprintln!("building textures missing: {}", missing.join(", "));
        }
    }

    /// Browser: start the download, upload once it has arrived.
    pub fn poll(&mut self, queue: &wgpu::Queue) {
        if self.done {
            return;
        }
        #[cfg(target_arch = "wasm32")]
        {
            use crate::platform::web::DirState;
            match crate::platform::web::dir_state("buildings/") {
                DirState::Absent => crate::platform::web::preload_dir("buildings/"),
                DirState::Loading(..) => {}
                DirState::Ready => self.load(queue),
                DirState::Failed(_) => self.done = true,
            }
        }
        #[cfg(not(target_arch = "wasm32"))]
        self.load(queue);
    }

    pub fn bytes(&self) -> u64 {
        self.bytes
    }
}
