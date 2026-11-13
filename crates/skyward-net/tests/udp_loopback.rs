//! Real sockets on 127.0.0.1: a server thread, clients, and a direct
//! discovery probe (what the browser does for manual / favourite servers).

#![cfg(not(target_arch = "wasm32"))]

use skyward_net::client::{ClientEvent, JoinRequest};
use skyward_net::discovery::{ScanEvent, Scanner, Via};
use skyward_net::proto::*;
use skyward_net::server::{Server, ServerConfig};
use skyward_net::udp::UdpClient;
use std::net::UdpSocket;
use std::sync::Arc;
use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

#[test]
fn loopback_session_and_probe() {
    let sock = UdpSocket::bind("127.0.0.1:0").unwrap();
    sock.set_read_timeout(Some(Duration::from_millis(2))).unwrap();
    let addr = sock.local_addr().unwrap();
    let stop = Arc::new(AtomicBool::new(false));
    let t0 = Instant::now();
    let st = stop.clone();
    let th = std::thread::spawn(move || {
        let cfg = ServerConfig { name: "Loopback".into(), game_port: addr.port(), motd: "welcome".into(), ..Default::default() };
        let mut server = Server::new(cfg, [3; 32], 0.0);
        let mut buf = [0u8; 2048];
        while !st.load(Ordering::Relaxed) {
            let now = t0.elapsed().as_secs_f64();
            while let Ok((n, from)) = sock.recv_from(&mut buf) {
                server.handle(from, &buf[..n], now);
                sock.set_nonblocking(true).unwrap();
            }
            sock.set_nonblocking(false).unwrap();
            server.tick(now);
            for (to, b) in server.out.drain(..) {
                let _ = sock.send_to(&b, to);
            }
        }
        server.chat_log.clone()
    });

    let now = || t0.elapsed().as_secs_f64();
    let mut clients: Vec<UdpClient> = (0..3)
        .map(|i| {
            UdpClient::connect(addr, JoinRequest { name: format!("P{i}"), aircraft: "kestrel".into(), password: String::new(), version: "test".into() }, now(), 50 + i).unwrap()
        })
        .collect();
    let mut known = vec![0usize; 3];
    let mut got_chat = vec![false; 3];
    let deadline = Instant::now() + Duration::from_secs(10);
    let mut sent_chat = false;
    while Instant::now() < deadline {
        for (i, c) in clients.iter_mut().enumerate() {
            let t = now();
            if c.client.connected() {
                c.client.set_state(AcState { lat: 51.5, lon: 0.05, alt: 500.0, rot: [0.0, 0.0, 0.0, 1.0], ..Default::default() }, t);
            }
            for e in c.poll(t) {
                match e {
                    ClientEvent::Msg(Msg::Player(_)) => known[i] += 1,
                    ClientEvent::Msg(Msg::Chat { text, .. }) if text == "over" => got_chat[i] = true,
                    ClientEvent::Failed(f) => panic!("client {i}: {f:?}"),
                    _ => {}
                }
            }
        }
        if !sent_chat && clients.iter().all(|c| c.client.connected()) {
            clients[0].client.send(Msg::Chat { from: 0, text: "over".into() });
            sent_chat = true;
        }
        if known.iter().all(|&k| k >= 2) && got_chat.iter().all(|&g| g) {
            break;
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    assert!(known.iter().all(|&k| k >= 2), "players known: {known:?}");
    assert!(got_chat.iter().all(|&g| g), "chat: {got_chat:?}");
    assert!(clients[0].client.clock.synced());

    // Direct discovery probe + details.
    let mut sc = Scanner::new(99).unwrap();
    sc.probe(addr, now(), Via::Direct);
    sc.request_details(addr);
    let (mut info, mut details) = (None, None);
    let deadline = Instant::now() + Duration::from_secs(3);
    while Instant::now() < deadline && (info.is_none() || details.is_none()) {
        for e in sc.poll(now()) {
            match e {
                ScanEvent::Server { addr: a, info: i, rtt_ms, .. } => {
                    assert_eq!(a, addr);
                    assert!(rtt_ms < 500.0);
                    info = i;
                }
                ScanEvent::Details { details: d, .. } => details = Some(d),
            }
        }
        std::thread::sleep(Duration::from_millis(5));
    }
    let info = info.expect("discovery reply");
    assert_eq!(info.name, "Loopback");
    assert_eq!(info.players, 3);
    let details = details.expect("details reply");
    assert_eq!(details.motd, "welcome");
    assert_eq!(details.players.len(), 3);

    for c in &mut clients {
        c.disconnect(now());
    }
    std::thread::sleep(Duration::from_millis(50));
    stop.store(true, Ordering::Relaxed);
    let chat = th.join().unwrap();
    assert!(chat.iter().any(|(n, t)| n == "P0" && t == "over"));
}
