//! Audio device output via cpal (ALSA / PulseAudio-PipeWire on Linux,
//! CoreAudio, WASAPI, and WebAudio on wasm32 with cpal's `wasm-bindgen`
//! feature).

use super::params::{Event, Frame};
use super::synth::{Settings, Synth};
use cpal::traits::{DeviceTrait, HostTrait, StreamTrait};
use cpal::{FromSample, SampleFormat, SizedSample};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};

/// State shared between the game thread and the audio callback (which only
/// ever `try_lock`s it).
pub struct Shared {
    pub frame: Frame,
    pub seq: u64,
    pub events: Vec<Event>,
    pub settings: Settings,
    /// Audio-thread cost: nanoseconds spent rendering / audio seconds rendered.
    pub cpu_ns: u64,
    pub cpu_audio_s: f64,
}

pub struct Output {
    _stream: cpal::Stream,
    pub sample_rate: u32,
    pub name: String,
    failed: Arc<AtomicBool>,
}

impl Output {
    pub fn failed(&self) -> bool {
        self.failed.load(Ordering::Relaxed)
    }

    pub fn open(shared: Arc<Mutex<Shared>>) -> Result<Output, String> {
        let host = cpal::default_host();
        // SKYWARD_AUDIO_DEVICE=<part of a device id> picks another output.
        let wanted = std::env::var("SKYWARD_AUDIO_DEVICE").ok().filter(|s| !s.is_empty());
        let device = match &wanted {
            Some(w) => host
                .output_devices()
                .map_err(|e| e.to_string())?
                .find(|d| d.id().is_ok_and(|id| id.to_string().contains(w.as_str())))
                .ok_or_else(|| format!("no output device matching {w:?}"))?,
            None => host.default_output_device().ok_or_else(|| "no default output device".to_string())?,
        };
        let name = device.id().map(|d| d.to_string()).unwrap_or_else(|_| "default output".into());
        let cfg = device.default_output_config().map_err(|e| e.to_string())?;
        let failed = Arc::new(AtomicBool::new(false));
        let sr = cfg.sample_rate();
        let fmt = cfg.sample_format();
        let config: cpal::StreamConfig = cfg.into();
        let stream = match fmt {
            SampleFormat::F32 => build::<f32>(&device, &config, shared, failed.clone()),
            SampleFormat::I16 => build::<i16>(&device, &config, shared, failed.clone()),
            SampleFormat::U16 => build::<u16>(&device, &config, shared, failed.clone()),
            SampleFormat::I32 => build::<i32>(&device, &config, shared, failed.clone()),
            SampleFormat::F64 => build::<f64>(&device, &config, shared, failed.clone()),
            SampleFormat::U8 => build::<u8>(&device, &config, shared, failed.clone()),
            SampleFormat::I8 => build::<i8>(&device, &config, shared, failed.clone()),
            f => return Err(format!("unsupported sample format {f}")),
        }?;
        stream.play().map_err(|e| e.to_string())?;
        Ok(Output { _stream: stream, sample_rate: sr, name, failed })
    }
}

fn build<T>(device: &cpal::Device, config: &cpal::StreamConfig, shared: Arc<Mutex<Shared>>, failed: Arc<AtomicBool>) -> Result<cpal::Stream, String>
where
    T: SizedSample + FromSample<f32>,
{
    let channels = config.channels.max(1) as usize;
    let sr = config.sample_rate as f32;
    let mut synth = Synth::new(sr);
    let mut seq = u64::MAX;
    let mut events: Vec<Event> = Vec::with_capacity(256);
    let mut buf: Vec<f32> = vec![0.0; 2 * 4096];
    #[allow(unused_mut)]
    let (mut cpu_ns, mut cpu_s) = (0u64, 0.0f64);
    let err_failed = failed.clone();
    device
        .build_output_stream(
            *config,
            move |data: &mut [T], _: &cpal::OutputCallbackInfo| {
                #[cfg(not(target_arch = "wasm32"))]
                let t0 = std::time::Instant::now();
                // Never block the audio thread: take the newest frame if the
                // game thread isn't holding the lock right now.
                if let Ok(mut sh) = shared.try_lock() {
                    if sh.seq != seq {
                        seq = sh.seq;
                        synth.push_frame(&sh.frame);
                    }
                    std::mem::swap(&mut events, &mut sh.events);
                    synth.set_settings(sh.settings);
                    sh.cpu_ns = cpu_ns;
                    sh.cpu_audio_s = cpu_s;
                }
                for e in events.drain(..) {
                    synth.event(&e);
                }
                let frames = data.len() / channels;
                let mut done = 0;
                while done < frames {
                    let n = (frames - done).min(buf.len() / 2);
                    synth.render(&mut buf[..n * 2]);
                    for i in 0..n {
                        let l = buf[i * 2];
                        let r = buf[i * 2 + 1];
                        let o = &mut data[(done + i) * channels..(done + i + 1) * channels];
                        match channels {
                            1 => o[0] = T::from_sample(0.5 * (l + r)),
                            _ => {
                                o[0] = T::from_sample(l);
                                o[1] = T::from_sample(r);
                                for x in o.iter_mut().skip(2) {
                                    *x = T::from_sample(0.0f32);
                                }
                            }
                        }
                    }
                    done += n;
                }
                #[cfg(not(target_arch = "wasm32"))]
                {
                    cpu_ns += t0.elapsed().as_nanos() as u64;
                    cpu_s += frames as f64 / sr as f64;
                }
            },
            move |e: cpal::Error| {
                use cpal::ErrorKind as K;
                match e.kind() {
                    K::DeviceChanged | K::Xrun | K::RealtimeDenied => {}
                    _ => {
                        eprintln!("audio stream error: {e}");
                        err_failed.store(true, Ordering::Relaxed);
                    }
                }
            },
            None,
        )
        .map_err(|e| e.to_string())
}
