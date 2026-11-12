//! `regress image GOLDEN.png TEST.png [--max-mean-de 3] [--max-bad 0.02] [--min-ssim 0.92] [--diff OUT.png]`
//! `regress audio GOLDEN.wav TEST.wav [--max-spectral-db 6] [--max-loudness-db 2]`
//!
//! `regress pack IN.wav OUT.wav`: store an audio golden compactly (mono, 16-bit, same rate).
//!
//! Prints one line of metrics; exit 0 = within tolerance, 1 = regression, 2 = usage / IO error.
//!
//! Images are compared perceptually: both are box-downsampled 2× (TAA / dither
//! noise), converted to CIELAB, and scored by the mean and 99th-percentile
//! CIE76 ΔE, the fraction of clearly different pixels (ΔE > 20), and the
//! mean SSIM of lightness over 8×8 windows. Audio is compared by its spectral
//! envelope: 32 log-spaced bands (40 Hz – 16 kHz) per 2048-sample frame, mean
//! RMS dB difference across bands and frames, plus the overall loudness.

use std::process::exit;

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let opt = |k: &str, d: f64| args.iter().position(|a| a == k).and_then(|i| args.get(i + 1)).and_then(|v| v.parse().ok()).unwrap_or(d);
    let res = match (args.first().map(|s| s.as_str()), args.get(1), args.get(2)) {
        (Some("image"), Some(a), Some(b)) => {
            let diff = args.iter().position(|a| a == "--diff").and_then(|i| args.get(i + 1)).cloned();
            image_cmp(a, b, diff.as_deref()).map(|m| {
                let ok = m.mean_de <= opt("--max-mean-de", 3.0) && m.bad <= opt("--max-bad", 0.02) && m.ssim >= opt("--min-ssim", 0.92);
                println!("mean dE {:.2}  p99 dE {:.1}  >20 dE {:.2}%  SSIM {:.4}  {}", m.mean_de, m.p99_de, m.bad * 100.0, m.ssim, if ok { "ok" } else { "REGRESSION" });
                ok
            })
        }
        (Some("audio"), Some(a), Some(b)) => audio_cmp(a, b).map(|(spec, loud, len)| {
            let ok = spec <= opt("--max-spectral-db", 6.0) && loud <= opt("--max-loudness-db", 2.0) && len <= 0.05;
            println!("spectral {:.2} dB  loudness {:+.2} dB  length {:+.1}%  {}", spec, loud, len * 100.0, if ok { "ok" } else { "REGRESSION" });
            ok
        }),
        (Some("pack"), Some(a), Some(b)) => pack(a, b).map(|_| true),
        _ => Err("usage: regress image|audio GOLDEN TEST [options]  |  regress pack IN.wav OUT.wav".into()),
    };
    match res {
        Ok(true) => {}
        Ok(false) => exit(1),
        Err(e) => {
            eprintln!("regress: {e}");
            exit(2)
        }
    }
}

// ---------------------------------------------------------------- images

pub struct ImageMetrics {
    pub mean_de: f64,
    pub p99_de: f64,
    pub bad: f64,
    pub ssim: f64,
}

fn load_lab(path: &str) -> Result<(usize, usize, Vec<[f64; 3]>), String> {
    let img = image::open(path).map_err(|e| format!("{path}: {e}"))?.to_rgb8();
    let (w, h) = (img.width() as usize / 2, img.height() as usize / 2);
    let lin = |c: u8| {
        let c = c as f64 / 255.0;
        if c <= 0.04045 { c / 12.92 } else { ((c + 0.055) / 1.055).powf(2.4) }
    };
    let mut out = Vec::with_capacity(w * h);
    for y in 0..h {
        for x in 0..w {
            let mut rgb = [0.0; 3];
            for (dx, dy) in [(0, 0), (1, 0), (0, 1), (1, 1)] {
                let p = img.get_pixel((2 * x + dx) as u32, (2 * y + dy) as u32);
                for c in 0..3 {
                    rgb[c] += lin(p[c]) * 0.25;
                }
            }
            out.push(lab(rgb));
        }
    }
    Ok((w, h, out))
}

/// Linear sRGB → CIELAB (D65).
pub fn lab([r, g, b]: [f64; 3]) -> [f64; 3] {
    let x = (0.4124 * r + 0.3576 * g + 0.1805 * b) / 0.95047;
    let y = 0.2126 * r + 0.7152 * g + 0.0722 * b;
    let z = (0.0193 * r + 0.1192 * g + 0.9505 * b) / 1.08883;
    let f = |t: f64| if t > 216.0 / 24389.0 { t.cbrt() } else { (24389.0 / 27.0 * t + 16.0) / 116.0 };
    let (fx, fy, fz) = (f(x), f(y), f(z));
    [116.0 * fy - 16.0, 500.0 * (fx - fy), 200.0 * (fy - fz)]
}

pub fn image_cmp(a: &str, b: &str, diff: Option<&str>) -> Result<ImageMetrics, String> {
    let (w, h, la) = load_lab(a)?;
    let (w2, h2, lb) = load_lab(b)?;
    if (w, h) != (w2, h2) {
        return Err(format!("size differs: {}x{} vs {}x{}", w * 2, h * 2, w2 * 2, h2 * 2));
    }
    let de: Vec<f64> = la.iter().zip(&lb).map(|(p, q)| ((p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2) + (p[2] - q[2]).powi(2)).sqrt()).collect();
    let mean_de = de.iter().sum::<f64>() / de.len() as f64;
    let mut sorted = de.clone();
    sorted.sort_by(|x, y| x.total_cmp(y));
    let p99_de = sorted[(sorted.len() as f64 * 0.99) as usize];
    let bad = de.iter().filter(|d| **d > 20.0).count() as f64 / de.len() as f64;
    let ssim = ssim_l(w, h, &la, &lb);
    if let Some(path) = diff {
        let img = image::RgbImage::from_fn(w as u32, h as u32, |x, y| {
            let d = (de[y as usize * w + x as usize] / 30.0).min(1.0);
            let g = (la[y as usize * w + x as usize][0] * 0.6) as u8;
            image::Rgb([(g as f64 * (1.0 - d) + 255.0 * d) as u8, (g as f64 * (1.0 - d)) as u8, (g as f64 * (1.0 - d)) as u8])
        });
        img.save(path).map_err(|e| e.to_string())?;
    }
    Ok(ImageMetrics { mean_de, p99_de, bad, ssim })
}

/// Mean SSIM of CIELAB lightness over 8×8 windows (stride 4).
fn ssim_l(w: usize, h: usize, a: &[[f64; 3]], b: &[[f64; 3]]) -> f64 {
    let (c1, c2) = ((0.01f64 * 100.0).powi(2), (0.03f64 * 100.0).powi(2));
    let (mut sum, mut n) = (0.0, 0usize);
    for y0 in (0..h.saturating_sub(8)).step_by(4) {
        for x0 in (0..w.saturating_sub(8)).step_by(4) {
            let (mut ma, mut mb) = (0.0, 0.0);
            for y in y0..y0 + 8 {
                for x in x0..x0 + 8 {
                    ma += a[y * w + x][0];
                    mb += b[y * w + x][0];
                }
            }
            ma /= 64.0;
            mb /= 64.0;
            let (mut va, mut vb, mut cov) = (0.0, 0.0, 0.0);
            for y in y0..y0 + 8 {
                for x in x0..x0 + 8 {
                    let (da, db) = (a[y * w + x][0] - ma, b[y * w + x][0] - mb);
                    va += da * da;
                    vb += db * db;
                    cov += da * db;
                }
            }
            let (va, vb, cov) = (va / 63.0, vb / 63.0, cov / 63.0);
            sum += ((2.0 * ma * mb + c1) * (2.0 * cov + c2)) / ((ma * ma + mb * mb + c1) * (va + vb + c2));
            n += 1;
        }
    }
    if n == 0 { 1.0 } else { sum / n as f64 }
}

// ---------------------------------------------------------------- audio

fn load_mono(path: &str) -> Result<(u32, Vec<f32>), String> {
    let mut r = hound::WavReader::open(path).map_err(|e| format!("{path}: {e}"))?;
    let spec = r.spec();
    let ch = spec.channels as usize;
    let raw: Vec<f32> = match spec.sample_format {
        hound::SampleFormat::Float => r.samples::<f32>().map(|s| s.unwrap_or(0.0)).collect(),
        hound::SampleFormat::Int => {
            let scale = 1.0 / (1i64 << (spec.bits_per_sample - 1)) as f32;
            r.samples::<i32>().map(|s| s.unwrap_or(0) as f32 * scale).collect()
        }
    };
    Ok((spec.sample_rate, raw.chunks(ch).map(|c| c.iter().sum::<f32>() / ch as f32).collect()))
}

/// In-place radix-2 FFT (len a power of two).
fn fft(re: &mut [f64], im: &mut [f64]) {
    let n = re.len();
    let mut j = 0;
    for i in 1..n {
        let mut bit = n >> 1;
        while j & bit != 0 {
            j ^= bit;
            bit >>= 1;
        }
        j |= bit;
        if i < j {
            re.swap(i, j);
            im.swap(i, j);
        }
    }
    let mut len = 2;
    while len <= n {
        let ang = -2.0 * std::f64::consts::PI / len as f64;
        for s in (0..n).step_by(len) {
            for k in 0..len / 2 {
                let (wr, wi) = ((ang * k as f64).cos(), (ang * k as f64).sin());
                let (a, b) = (s + k, s + k + len / 2);
                let (xr, xi) = (re[b] * wr - im[b] * wi, re[b] * wi + im[b] * wr);
                re[b] = re[a] - xr;
                im[b] = im[a] - xi;
                re[a] += xr;
                im[a] += xi;
            }
        }
        len <<= 1;
    }
}

/// Per frame, 32 log-spaced band levels (dB).
pub fn bands(rate: u32, x: &[f32]) -> Vec<[f64; 32]> {
    const N: usize = 2048;
    let edges: Vec<f64> = (0..=32).map(|i| 40.0 * (16000.0f64 / 40.0).powf(i as f64 / 32.0)).collect();
    let mut out = Vec::new();
    let mut s = 0;
    while s + N <= x.len() {
        let mut re: Vec<f64> = (0..N).map(|i| x[s + i] as f64 * (0.5 - 0.5 * (2.0 * std::f64::consts::PI * i as f64 / N as f64).cos())).collect();
        let mut im = vec![0.0; N];
        fft(&mut re, &mut im);
        let mut b = [0.0f64; 32];
        for k in 1..N / 2 {
            let f = k as f64 * rate as f64 / N as f64;
            if let Some(i) = edges.windows(2).position(|e| f >= e[0] && f < e[1]) {
                b[i] += re[k] * re[k] + im[k] * im[k];
            }
        }
        out.push(b.map(|e| 10.0 * (e + 1e-10).log10()));
        s += N / 2;
    }
    out
}

/// Mono 16-bit copy of a render (a fifth of the size of the f32 stereo output).
pub fn pack(src: &str, dst: &str) -> Result<(), String> {
    let (rate, x) = load_mono(src)?;
    let spec = hound::WavSpec { channels: 1, sample_rate: rate, bits_per_sample: 16, sample_format: hound::SampleFormat::Int };
    let mut w = hound::WavWriter::create(dst, spec).map_err(|e| e.to_string())?;
    for s in x {
        w.write_sample((s.clamp(-1.0, 1.0) * 32767.0).round() as i16).map_err(|e| e.to_string())?;
    }
    w.finalize().map_err(|e| e.to_string())
}

pub fn audio_cmp(a: &str, b: &str) -> Result<(f64, f64, f64), String> {
    let (ra, xa) = load_mono(a)?;
    let (rb, xb) = load_mono(b)?;
    if ra != rb {
        return Err(format!("sample rate differs: {ra} vs {rb}"));
    }
    let len = (xb.len() as f64 - xa.len() as f64) / xa.len().max(1) as f64;
    let (ba, bb) = (bands(ra, &xa), bands(rb, &xb));
    let n = ba.len().min(bb.len()).max(1);
    // Floors: −100 dB bands (silence) don't dominate.
    let spec =
        (0..n.min(ba.len()).min(bb.len())).map(|f| (0..32).map(|i| (ba[f][i].max(-60.0) - bb[f][i].max(-60.0)).powi(2)).sum::<f64>() / 32.0).map(f64::sqrt).sum::<f64>() / n as f64;
    let rms = |x: &[f32]| 10.0 * ((x.iter().map(|s| (*s as f64).powi(2)).sum::<f64>() / x.len().max(1) as f64) + 1e-12).log10();
    Ok((spec, rms(&xb) - rms(&xa), len.abs()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn lab_white_black() {
        let w = lab([1.0, 1.0, 1.0]);
        assert!((w[0] - 100.0).abs() < 0.1 && w[1].abs() < 0.5 && w[2].abs() < 0.5);
        assert!(lab([0.0, 0.0, 0.0])[0].abs() < 1e-9);
    }

    #[test]
    fn fft_finds_a_tone() {
        let rate = 48_000;
        let x: Vec<f32> = (0..8192).map(|i| (2.0 * std::f32::consts::PI * 1000.0 * i as f32 / rate as f32).sin()).collect();
        let b = bands(rate, &x);
        let loudest = (0..32).max_by(|&i, &j| b[2][i].total_cmp(&b[2][j])).unwrap();
        let edges: Vec<f64> = (0..=32).map(|i| 40.0 * (16000.0f64 / 40.0).powf(i as f64 / 32.0)).collect();
        assert!(edges[loudest] <= 1000.0 && edges[loudest + 1] > 1000.0);
    }

    #[test]
    fn ssim_identical_is_one() {
        let px: Vec<[f64; 3]> = (0..64 * 64).map(|i| [(i % 97) as f64, 0.0, 0.0]).collect();
        assert!((ssim_l(64, 64, &px, &px) - 1.0).abs() < 1e-9);
    }
}
