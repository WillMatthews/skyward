//! Client ↔ server clock synchronisation (NTP-style).
//!
//! The client pings with its local time; the server answers with its own.
//! Offset = server + rtt / 2 − local_receive. Samples with the lowest RTT
//! have the least queueing asymmetry, so the estimate is the median offset
//! of the fastest quarter of the recent samples, and the clock *slews*
//! towards it (no jumps once synced, at most 5 ms per second) so remote
//! aircraft never stutter when the estimate moves.

use std::collections::VecDeque;

const WINDOW: usize = 32;

#[derive(Clone, Debug)]
pub struct ClockSync {
    samples: VecDeque<(f64, f64)>, // (rtt, offset)
    /// Offset in use (server = local + offset).
    offset: f64,
    target: f64,
    synced: bool,
    /// Smallest RTT seen recently, s.
    pub rtt_min: f64,
    /// Smoothed RTT, s.
    pub rtt: f64,
    /// RTT jitter (mean absolute deviation), s.
    pub jitter: f64,
}

impl Default for ClockSync {
    fn default() -> Self {
        ClockSync { samples: VecDeque::new(), offset: 0.0, target: 0.0, synced: false, rtt_min: 0.0, rtt: 0.0, jitter: 0.0 }
    }
}

impl ClockSync {
    /// A pong: `sent` = our local send time, `server` = the server's time
    /// when it answered, `now` = local receive time.
    pub fn sample(&mut self, sent: f64, server: f64, now: f64) {
        let rtt = now - sent;
        if !(0.0..5.0).contains(&rtt) {
            return;
        }
        let off = server + rtt * 0.5 - now;
        self.samples.push_back((rtt, off));
        if self.samples.len() > WINDOW {
            self.samples.pop_front();
        }
        if self.rtt == 0.0 {
            self.rtt = rtt;
        } else {
            self.jitter = 0.9 * self.jitter + 0.1 * (rtt - self.rtt).abs();
            self.rtt = 0.9 * self.rtt + 0.1 * rtt;
        }
        let mut by_rtt: Vec<(f64, f64)> = self.samples.iter().copied().collect();
        by_rtt.sort_by(|a, b| a.0.total_cmp(&b.0));
        self.rtt_min = by_rtt[0].0;
        let k = (by_rtt.len() / 4).max(1);
        let mut best: Vec<f64> = by_rtt[..k].iter().map(|s| s.1).collect();
        best.sort_by(|a, b| a.total_cmp(b));
        self.target = best[best.len() / 2];
        if !self.synced {
            self.offset = self.target;
            self.synced = true;
        } else if (self.target - self.offset).abs() > 0.5 {
            // A big disagreement (the server restarted, a host clock jump): snap.
            self.offset = self.target;
        }
    }

    /// Advance the slew (call every frame).
    pub fn update(&mut self, dt: f64) {
        let max = 0.005 * dt.max(0.0) + 1e-6;
        self.offset += (self.target - self.offset).clamp(-max, max);
    }

    pub fn synced(&self) -> bool {
        self.synced
    }

    /// Server time now.
    pub fn server_time(&self, local: f64) -> f64 {
        local + self.offset
    }

    pub fn offset(&self) -> f64 {
        self.offset
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converges_with_asymmetric_jitter() {
        let mut c = ClockSync::default();
        let true_off = 1234.5678;
        let mut rng = 7u64;
        let mut rnd = || {
            rng ^= rng << 13;
            rng ^= rng >> 7;
            rng ^= rng << 17;
            (rng % 10_000) as f64 / 10_000.0
        };
        let mut t = 10.0;
        for _ in 0..200 {
            // 75 ms each way + up to 40 ms of queueing on either leg.
            let up = 0.075 + rnd() * 0.04;
            let down = 0.075 + rnd() * 0.04;
            let server = t + up + true_off;
            c.sample(t, server, t + up + down);
            c.update(0.5);
            t += 0.5;
        }
        assert!((c.server_time(t) - (t + true_off)).abs() < 0.008, "error {}", c.server_time(t) - (t + true_off));
        assert!((c.rtt - 0.19).abs() < 0.03);
    }

    #[test]
    fn slews_not_jumps() {
        let mut c = ClockSync::default();
        c.sample(0.0, 100.05, 0.1); // offset 100
        assert!((c.offset() - 100.0).abs() < 1e-9);
        for i in 0..40 {
            c.sample(i as f64, i as f64 + 100.05 + 0.02, i as f64 + 0.1); // offset 100.02
        }
        let before = c.offset();
        c.update(1.0);
        assert!((c.offset() - before).abs() <= 0.005 + 1e-6);
    }
}
