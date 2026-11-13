//! A [`Client`] on a real UDP socket (native only): non-blocking, polled
//! once per frame. `SKYWARD_NET_SIM="lat=75,jitter=10,loss=2"` puts a
//! simulated bad link (both directions) in between, for trying the
//! interpolation on a LAN.

use crate::client::{Client, ClientEvent, JoinRequest};
use crate::link::{Link, LinkParams};
use crate::proto::MAX_RECV;
use std::io;
use std::net::{SocketAddr, UdpSocket};

pub struct UdpClient {
    sock: UdpSocket,
    pub server: SocketAddr,
    pub client: Client,
    sim: Option<(Link, Link)>,
}

impl UdpClient {
    pub fn connect(server: SocketAddr, req: JoinRequest, now: f64, nonce: u64) -> io::Result<UdpClient> {
        let bind: SocketAddr = if server.is_ipv4() { "0.0.0.0:0".parse().unwrap() } else { "[::]:0".parse().unwrap() };
        let sock = UdpSocket::bind(bind)?;
        sock.set_nonblocking(true)?;
        let sim = std::env::var("SKYWARD_NET_SIM").ok().and_then(|s| LinkParams::parse(&s)).map(|p| (Link::new(p, nonce ^ 0x5151), Link::new(p, nonce ^ 0xA2A2)));
        let mut c = UdpClient { sock, server, client: Client::connect(req, now, nonce), sim };
        c.flush(now);
        Ok(c)
    }

    fn flush(&mut self, now: f64) {
        for p in self.client.take_outgoing() {
            match self.sim.as_mut() {
                Some((up, _)) => up.send(now, p),
                None => {
                    let _ = self.sock.send_to(&p, self.server);
                }
            }
        }
        if let Some((up, _)) = self.sim.as_mut() {
            for p in up.recv(now) {
                let _ = self.sock.send_to(&p, self.server);
            }
        }
    }

    /// Receive, advance, send. Call once per frame.
    pub fn poll(&mut self, now: f64) -> Vec<ClientEvent> {
        let mut ev = Vec::new();
        let mut buf = [0u8; 2048];
        let mut got: Vec<Vec<u8>> = Vec::new();
        for _ in 0..512 {
            match self.sock.recv_from(&mut buf) {
                Ok((n, from)) if from == self.server && n <= MAX_RECV => got.push(buf[..n].to_vec()),
                Ok(_) => {}
                Err(_) => break,
            }
        }
        if let Some((_, down)) = self.sim.as_mut() {
            for p in got.drain(..) {
                down.send(now, p);
            }
            got = down.recv(now);
        }
        for p in got {
            ev.extend(self.client.handle(&p, now));
        }
        ev.extend(self.client.update(now));
        self.flush(now);
        ev
    }

    pub fn disconnect(&mut self, now: f64) {
        self.client.disconnect();
        self.sim = None;
        self.flush(now);
    }
}
