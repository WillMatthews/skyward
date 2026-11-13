//! A simulated network link: latency, jitter, loss, duplication. Used by the
//! tests (lossy-link convergence) and, through `SKYWARD_NET_SIM`, by the
//! game to try a bad connection on a LAN.

/// Link parameters (one direction).
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LinkParams {
    /// One-way latency, s.
    pub latency: f64,
    /// Uniform extra delay 0..jitter, s (reorders packets).
    pub jitter: f64,
    /// Probability a packet is lost.
    pub loss: f64,
    /// Probability a packet arrives twice.
    pub duplicate: f64,
}

impl Default for LinkParams {
    fn default() -> Self {
        LinkParams { latency: 0.0, jitter: 0.0, loss: 0.0, duplicate: 0.0 }
    }
}

impl LinkParams {
    /// "lat=75,jitter=15,loss=2,dup=0.5" (ms / ms / % / %).
    pub fn parse(s: &str) -> Option<LinkParams> {
        let mut p = LinkParams::default();
        for kv in s.split(',') {
            let (k, v) = kv.split_once('=')?;
            let v: f64 = v.trim().parse().ok()?;
            match k.trim() {
                "lat" | "latency" => p.latency = v / 1000.0,
                "jitter" => p.jitter = v / 1000.0,
                "loss" => p.loss = v / 100.0,
                "dup" => p.duplicate = v / 100.0,
                _ => return None,
            }
        }
        Some(p)
    }
}

pub struct Link {
    pub p: LinkParams,
    rng: u64,
    queue: Vec<(f64, Vec<u8>)>,
    pub sent: u64,
    pub lost: u64,
}

impl Link {
    pub fn new(p: LinkParams, seed: u64) -> Link {
        Link { p, rng: seed.max(1), queue: Vec::new(), sent: 0, lost: 0 }
    }

    fn rnd(&mut self) -> f64 {
        self.rng ^= self.rng << 13;
        self.rng ^= self.rng >> 7;
        self.rng ^= self.rng << 17;
        (self.rng >> 11) as f64 / (1u64 << 53) as f64
    }

    pub fn send(&mut self, now: f64, data: Vec<u8>) {
        self.sent += 1;
        if self.rnd() < self.p.loss {
            self.lost += 1;
            return;
        }
        let t = now + self.p.latency + self.rnd() * self.p.jitter;
        if self.rnd() < self.p.duplicate {
            let t2 = t + self.rnd() * self.p.jitter;
            self.queue.push((t2, data.clone()));
        }
        self.queue.push((t, data));
    }

    /// Packets due by `now`, in arrival order.
    pub fn recv(&mut self, now: f64) -> Vec<Vec<u8>> {
        self.queue.sort_by(|a, b| a.0.total_cmp(&b.0));
        let n = self.queue.iter().take_while(|q| q.0 <= now).count();
        self.queue.drain(..n).map(|q| q.1).collect()
    }
}
