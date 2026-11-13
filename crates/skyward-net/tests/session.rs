//! Whole sessions over simulated links (deterministic, no sockets): lossy
//! link pose error, 32 clients, handshake refusals, moderation.

use skyward_net::client::{Client, ClientEvent, ConnectError, JoinRequest};
use skyward_net::interp::{Interpolator, Tangent, q_angle, q_from_rotvec, q_mul, q_norm};
use skyward_net::link::{Link, LinkParams};
use skyward_net::proto::*;
use skyward_net::server::{Server, ServerConfig};
use std::collections::HashMap;
use std::net::SocketAddr;

struct Node {
    addr: SocketAddr,
    client: Client,
    up: Link,
    down: Link,
    events: Vec<ClientEvent>,
    interp: HashMap<u16, Interpolator>,
    players: HashMap<u16, PlayerInfo>,
    chat: Vec<(u16, String)>,
}

struct Net {
    server: Server,
    nodes: Vec<Node>,
    now: f64,
}

fn req(name: &str, password: &str) -> JoinRequest {
    JoinRequest { name: name.into(), aircraft: "kestrel".into(), password: password.into(), version: "test".into() }
}

impl Net {
    fn new(cfg: ServerConfig) -> Net {
        Net { server: Server::new(cfg, [9; 32], 0.0), nodes: Vec::new(), now: 0.0 }
    }

    fn join(&mut self, name: &str, password: &str, link: LinkParams) -> usize {
        let i = self.nodes.len();
        let addr: SocketAddr = format!("10.0.{}.{}:{}", i / 200, i % 200 + 1, 40000 + i).parse().unwrap();
        let client = Client::connect(req(name, password), self.now, 1000 + i as u64);
        self.nodes.push(Node {
            addr,
            client,
            up: Link::new(link, 77 + i as u64),
            down: Link::new(link, 991 + i as u64),
            events: Vec::new(),
            interp: HashMap::new(),
            players: HashMap::new(),
            chat: Vec::new(),
        });
        i
    }

    /// Advance by `dt` (one frame).
    fn step(&mut self, dt: f64) {
        self.now += dt;
        let now = self.now;
        for n in &mut self.nodes {
            for p in n.client.take_outgoing() {
                n.up.send(now, p);
            }
            for p in n.up.recv(now) {
                self.server.handle(n.addr, &p, now);
            }
        }
        self.server.tick(now);
        for (to, p) in std::mem::take(&mut self.server.out) {
            if let Some(n) = self.nodes.iter_mut().find(|n| n.addr == to) {
                n.down.send(now, p);
            }
        }
        for n in &mut self.nodes {
            let mut ev = Vec::new();
            for p in n.down.recv(now) {
                ev.extend(n.client.handle(&p, now));
            }
            ev.extend(n.client.update(now));
            let st = n.client.server_time(now);
            for e in &ev {
                if let ClientEvent::Msg(m) = e {
                    match m {
                        Msg::Snapshot { entries, .. } => {
                            for (id, s) in entries {
                                n.interp.entry(*id).or_default().push(*s, st);
                            }
                        }
                        Msg::Player(p) => {
                            n.players.insert(p.id, p.clone());
                        }
                        Msg::Left { id, .. } => {
                            n.players.remove(id);
                            n.interp.remove(id);
                        }
                        Msg::Chat { from, text } => n.chat.push((*from, text.clone())),
                        _ => {}
                    }
                }
            }
            n.events.extend(ev);
        }
    }
}

/// A 45°-banked level turn at 110 m/s near London — the "truth" trajectory.
fn truth(t: f64) -> AcState {
    let tan = Tangent::new(51.5, -0.1);
    let v = 110.0;
    let w = 9.81 * 45f64.to_radians().tan() / v;
    let r = v / w;
    let a = w * t;
    let p = [r - r * a.cos(), 900.0, -r * a.sin()];
    let (lat, lon, alt) = tan.to_geo(p);
    let yaw = q_from_rotvec([0.0, std::f64::consts::PI - a, 0.0]);
    let bank = q_from_rotvec([0.0, 0.0, 45f64.to_radians()]);
    let q = q_norm(q_mul(yaw, bank));
    AcState {
        t,
        lat,
        lon,
        alt: alt as f32,
        rot: [q[0] as f32, q[1] as f32, q[2] as f32, q[3] as f32],
        vel: [(v * a.sin()) as f32, 0.0, (-v * a.cos()) as f32],
        omega: [0.0, -w as f32, 0.0],
        n_engines: 1,
        lights: lights::NAV,
        ..Default::default()
    }
}

fn pos_err(a: &AcState, b: &AcState) -> f64 {
    let tan = Tangent::new(b.lat, b.lon);
    let (pa, pb) = (tan.to_local(a.lat, a.lon, a.alt as f64), tan.to_local(b.lat, b.lon, b.alt as f64));
    ((pa[0] - pb[0]).powi(2) + (pa[1] - pb[1]).powi(2) + (pa[2] - pb[2]).powi(2)).sqrt()
}

fn connect_all(net: &mut Net) {
    for _ in 0..600 {
        net.step(1.0 / 60.0);
        if net.nodes.iter().all(|n| n.client.connected() && n.client.clock.synced()) {
            return;
        }
    }
    panic!("not all connected: {:?}", net.nodes.iter().map(|n| n.client.phase.clone()).collect::<Vec<_>>());
}

/// 150 ms RTT, ±10 ms jitter, 2 % loss each way: the remote aircraft drawn by A
/// stays on B's true path (compared at the playout time).
#[test]
fn lossy_link_pose_error_bounded() {
    let link = LinkParams { latency: 0.075, jitter: 0.02, loss: 0.02, duplicate: 0.005 };
    let mut net = Net::new(ServerConfig::default());
    let a = net.join("Alpha", "", link);
    let b = net.join("Bravo", "", link);
    connect_all(&mut net);
    let bravo_id = net.nodes[b].client.player;
    let mut errs: Vec<f64> = Vec::new();
    let mut ang: Vec<f64> = Vec::new();
    let mut extrap_frames = 0;
    let frames = 60 * 60;
    for f in 0..frames {
        // B flies the truth trajectory on the server clock.
        let (now, st) = (net.now, net.nodes[b].client.server_time(net.now));
        net.nodes[b].client.set_state(truth(st), now);
        net.step(1.0 / 60.0);
        let now_srv = net.nodes[a].client.server_time(net.now);
        if let Some(ip) = net.nodes[a].interp.get_mut(&bravo_id) {
            if let Some(s) = ip.sample(now_srv, 1.0 / 60.0) {
                if f > 300 {
                    let tr = truth(s.state.t);
                    errs.push(pos_err(&s.state, &tr));
                    let q = |s: &AcState| [s.rot[0] as f64, s.rot[1] as f64, s.rot[2] as f64, s.rot[3] as f64];
                    ang.push(q_angle(q(&s.state), q(&tr)).to_degrees());
                    if s.extrapolated > 0.0 {
                        extrap_frames += 1;
                    }
                }
            }
        }
    }
    errs.sort_by(|x, y| x.total_cmp(y));
    ang.sort_by(|x, y| x.total_cmp(y));
    let p = |v: &Vec<f64>, q: f64| v[((v.len() - 1) as f64 * q) as usize];
    let delay = net.nodes[a].interp[&bravo_id].delay;
    println!(
        "lossy link (150 ms RTT, 2 % loss): pos error median {:.3} m, p99 {:.3} m, max {:.3} m; attitude p99 {:.3}°, max {:.3}°; extrapolated {:.1} % of frames; playout delay {:.0} ms; clock offset err {:.1} ms",
        p(&errs, 0.5),
        p(&errs, 0.99),
        errs.last().unwrap(),
        p(&ang, 0.99),
        ang.last().unwrap(),
        100.0 * extrap_frames as f64 / errs.len() as f64,
        delay * 1000.0,
        (net.nodes[a].client.server_time(net.now) - net.now).abs() * 1000.0
    );
    assert!(errs.len() > 2500);
    assert!(p(&errs, 0.5) < 0.05, "median {}", p(&errs, 0.5));
    assert!(p(&errs, 0.99) < 1.0, "p99 {}", p(&errs, 0.99));
    assert!(*errs.last().unwrap() < 4.0, "max {}", errs.last().unwrap());
    assert!(p(&ang, 0.99) < 1.0);
}

#[test]
fn thirty_two_clients() {
    let cfg = ServerConfig { max_players: 32, ..Default::default() };
    let mut net = Net::new(cfg);
    let link = LinkParams { latency: 0.01, jitter: 0.005, loss: 0.01, duplicate: 0.0 };
    for i in 0..32 {
        net.join(&format!("Pilot{i}"), "", link);
    }
    connect_all(&mut net);
    assert_eq!(net.server.peers.len(), 32);
    // A 33rd is refused: full.
    let x = net.join("Late", "", LinkParams::default());
    for _ in 0..60 {
        net.step(1.0 / 60.0);
    }
    assert!(net.nodes[x].events.iter().any(|e| matches!(e, ClientEvent::Failed(ConnectError::Rejected { reason: RejectReason::Full, .. }))));
    net.nodes.pop();
    // Everyone flies for 5 s, someone chats, garbage arrives interleaved.
    let t0 = net.now;
    let mut junk = 1u64;
    for f in 0..300 {
        for (i, n) in net.nodes.iter_mut().enumerate() {
            let st = n.client.server_time(net.now);
            let mut s = truth(st + i as f64);
            s.lat += i as f64 * 0.001;
            n.client.set_state(s, net.now);
        }
        if f == 10 {
            net.nodes[5].client.send(Msg::Chat { from: 0, text: "hello everyone".into() });
        }
        for _ in 0..4 {
            junk ^= junk << 13;
            junk ^= junk >> 7;
            junk ^= junk << 17;
            let mut b = Packet::Data { token: junk, ack: AckHeader::default(), payload: vec![junk as u8; (junk % 300) as usize] }.encode();
            if junk.is_multiple_of(3) {
                b.truncate(9);
            }
            let addr = net.nodes[(junk % 32) as usize].addr;
            net.server.handle(addr, &b, net.now);
        }
        net.step(1.0 / 60.0);
    }
    let secs = net.now - t0;
    for (i, n) in net.nodes.iter().enumerate() {
        assert_eq!(n.players.len(), 31, "client {i} knows {} players", n.players.len());
        assert_eq!(n.interp.len(), 31, "client {i} has states of {} aircraft", n.interp.len());
        assert!(n.chat.iter().any(|(_, t)| t == "hello everyone"), "client {i} missed the chat");
        assert!(n.client.connected());
    }
    let st = net.server.stats;
    println!(
        "32 clients: server out {:.0} kB/s ({:.0} packets/s), in {:.0} kB/s; malformed dropped {}",
        st.bytes_out as f64 / 1e3 / net.now,
        st.packets_out as f64 / net.now,
        st.bytes_in as f64 / 1e3 / net.now,
        st.malformed
    );
    assert!(st.malformed > 0);
    let _ = secs;
}

#[test]
fn refusals_have_reasons() {
    let cfg = ServerConfig { password: "hunter2".into(), max_players: 4, ..Default::default() };
    let mut net = Net::new(cfg);
    let bad = net.join("Mallory", "wrong", LinkParams::default());
    let good = net.join("Alice", "hunter2", LinkParams::default());
    for _ in 0..60 {
        net.step(1.0 / 60.0);
    }
    assert!(net.nodes[bad].events.iter().any(|e| matches!(e, ClientEvent::Failed(ConnectError::Rejected { reason: RejectReason::BadPassword, .. }))));
    assert!(net.nodes[good].client.connected());
    // Version mismatch: a Hello with another protocol number.
    let addr: SocketAddr = "10.9.9.9:5000".parse().unwrap();
    net.server.handle(addr, &Packet::Hello { proto: PROTOCOL + 1, nonce: 5 }.encode(), net.now);
    let (_, reply) = net.server.out.pop().unwrap();
    match Packet::decode(&reply).unwrap() {
        Packet::Reject { reason, server_proto, .. } => {
            assert_eq!(reason, RejectReason::VersionMismatch);
            assert_eq!(server_proto, PROTOCOL);
        }
        p => panic!("{p:?}"),
    }
    // Forged cookie: silently ignored.
    let n = net.server.out.len();
    net.server.handle(addr, &Packet::Join { nonce: 5, cookie: 1, name: "x".into(), aircraft: "kestrel".into(), version: String::new(), proof: [0; 32] }.encode(), net.now);
    assert_eq!(net.server.out.len(), n);
    // Unreachable server: the client gives up with a clear error.
    let mut c = Client::connect(req("Lonely", ""), 0.0, 3);
    let mut failed = None;
    for i in 0..400 {
        for e in c.update(i as f64 * 0.02) {
            if let ClientEvent::Failed(f) = e {
                failed = Some(f);
            }
        }
    }
    assert_eq!(failed, Some(ConnectError::Unreachable));
}

#[test]
fn kick_ban_and_names() {
    let mut net = Net::new(ServerConfig::default());
    let a = net.join("Alice", "", LinkParams::default());
    let b = net.join("alice", "", LinkParams::default());
    connect_all(&mut net);
    let names: Vec<String> = net.server.list(net.now).into_iter().map(|r| r.1).collect();
    assert!(names.contains(&"Alice".to_string()) && names.contains(&"alice (2)".to_string()), "{names:?}");
    net.server.kick("alice (2)", "testing", true).unwrap();
    for _ in 0..30 {
        net.step(1.0 / 60.0);
    }
    assert!(net.nodes[b].events.iter().any(|e| matches!(e, ClientEvent::Closed { reason: RejectReason::Banned, .. })));
    assert!(net.nodes[a].players.is_empty(), "Alice saw Bob leave");
    // The banned address cannot come back.
    let addr = net.nodes[b].addr;
    net.server.handle(addr, &Packet::Hello { proto: PROTOCOL, nonce: 9 }.encode(), net.now);
    let (_, r) = net.server.out.pop().unwrap();
    assert!(matches!(Packet::decode(&r).unwrap(), Packet::Reject { reason: RejectReason::Banned, .. }));
    // Shutdown tells the rest.
    net.server.shutdown("maintenance");
    for _ in 0..10 {
        net.step(1.0 / 60.0);
    }
    assert!(net.nodes[a].events.iter().any(|e| matches!(e, ClientEvent::Closed { reason: RejectReason::ServerShutdown, .. })));
}
