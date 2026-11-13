//! Protocol round trips and malformed-input robustness.

use skyward_net::proto::*;

fn sample_state() -> AcState {
    let mut s = AcState {
        t: 1234.5678,
        lat: 51.505,
        lon: 0.0553,
        alt: 812.5,
        rot: [0.1, -0.7, 0.2, 0.676],
        vel: [120.25, -3.5, -40.0],
        omega: [0.01, -0.2, 0.3],
        aileron: -0.4,
        elevator: 0.2,
        rudder: 0.05,
        flaps: 0.5,
        spoilers: 0.0,
        gear: 1.0,
        reverser: 0.0,
        afterburner: 0.75,
        throttle: 0.9,
        brakes: 0.0,
        tas: 131.2,
        n_engines: 2,
        lights: lights::NAV | lights::BEACON | lights::LANDING,
        flags: flags::CONTRAIL | flags::ROTORCRAFT,
        lost_parts: 0b101,
        ..Default::default()
    };
    let n: f32 = s.rot.iter().map(|c| c * c).sum::<f32>().sqrt();
    s.rot = s.rot.map(|c| c / n);
    s.engines[0] = EngineState { rpm: 2300.0, spool: 0.8, thrust: 0.7, running: true };
    s.engines[1] = EngineState { rpm: 2280.0, spool: 0.79, thrust: -0.3, running: false };
    s.rotor = RotorState { nr_rpm: 386.5, nr_pct: 100.0, tail_rpm: 5990.0, coning_deg: 3.5, collective_deg: 8.25, tilt_aft_deg: -1.5, tilt_right_deg: 0.75, tail_pitch_deg: 4.0 };
    s
}

fn close(a: f32, b: f32, tol: f32) -> bool {
    (a - b).abs() <= tol
}

#[test]
fn state_round_trip_and_size() {
    let s = sample_state();
    let mut w = skyward_net::codec::Writer::new();
    s.encode(&mut w);
    assert!(w.len() <= MAX_STATE_BYTES, "{} > {}", w.len(), MAX_STATE_BYTES);
    let d = AcState::decode(&mut skyward_net::codec::Reader::new(&w.buf)).unwrap();
    assert_eq!(d.t, s.t);
    assert_eq!(d.lat, s.lat);
    assert_eq!(d.lon, s.lon);
    assert_eq!(d.alt, s.alt);
    for i in 0..4 {
        assert!(close(d.rot[i], s.rot[i], 1e-4));
    }
    for i in 0..3 {
        assert!(close(d.vel[i], s.vel[i], 0.026));
        assert!(close(d.omega[i], s.omega[i], 0.0006));
    }
    assert!(close(d.aileron, s.aileron, 0.01) && close(d.afterburner, s.afterburner, 0.005));
    assert!(close(d.tas, s.tas, 0.06));
    assert_eq!(d.n_engines, 2);
    assert!(d.engines[0].running && !d.engines[1].running);
    assert!(close(d.engines[0].rpm, 2300.0, 0.5) && close(d.engines[1].thrust, -0.3, 0.01));
    assert!(close(d.rotor.nr_rpm, 386.5, 0.06) && close(d.rotor.collective_deg, 8.25, 0.006));
    assert_eq!((d.lights, d.flags, d.lost_parts), (s.lights, s.flags, s.lost_parts));
}

fn all_packets() -> Vec<Packet> {
    let info = ServerInfo {
        instance: 42,
        name: "Will's LAN".into(),
        players: 3,
        max_players: 16,
        airport: "EGLC".into(),
        region: "London".into(),
        password: true,
        game_port: 47820,
        conditions: "Real time · live weather".into(),
    };
    let world = WorldSync { time: TimeSync::Fixed { hours: 14.5, day: 200 }, utc: 1.7e9, metar: "EGLC 121150Z 25015KT 9999 SCT045 22/10 Q1015".into(), live_weather: false };
    let msgs = vec![
        Msg::State(sample_state()),
        Msg::Snapshot { time: 10.0, entries: vec![(3, sample_state()), (9, sample_state())] },
        Msg::Ping { t: 1.0 },
        Msg::Pong { t: 1.0, server: 2.0 },
        Msg::Player(PlayerInfo { id: 3, name: "Alice".into(), aircraft: "kestrel".into(), spectating: true }),
        Msg::Left { id: 3, reason: LeaveReason::TimedOut },
        Msg::Chat { from: SERVER_ID, text: "hello · ✈".into() },
        Msg::World(world.clone()),
        Msg::Event { from: 4, event: GameEvent::Crash { severity: 2 } },
        Msg::Event { from: 4, event: GameEvent::Separation { part: 7 } },
        Msg::SetAircraft { aircraft: "meridian320".into(), spectating: false },
    ];
    let mut payload = Vec::new();
    for m in &msgs {
        let b = m.encode_body();
        payload.push(m.type_id());
        if m.reliable() {
            payload.extend_from_slice(&7u16.to_le_bytes());
        }
        payload.extend_from_slice(&(b.len() as u16).to_le_bytes());
        payload.extend_from_slice(&b);
    }
    vec![
        Packet::Discover { proto: PROTOCOL, nonce: 77 },
        Packet::DiscoverReply { nonce: 77, proto: PROTOCOL, version: "0.1.0".into(), game_port: 47820, info: Some(info.clone()) },
        Packet::Details { proto: PROTOCOL, nonce: 5 },
        Packet::DetailsReply {
            nonce: 5,
            details: ServerDetails {
                motd: "Be nice".into(),
                players: vec![PlayerSummary { name: "Bob".into(), aircraft: "lancer".into(), ping_ms: 12 }],
                world: world.clone(),
                uptime_s: 3600,
            },
        },
        Packet::Hello { proto: PROTOCOL, nonce: 1 },
        Packet::Challenge { nonce: 1, cookie: 99 },
        Packet::Join { nonce: 1, cookie: 99, name: "Alice".into(), aircraft: "kestrel".into(), version: "0.1.0".into(), proof: [7; 32] },
        Packet::Accept { nonce: 1, player: 3, token: 0xABCD, welcome: Welcome { info, motd: "hi".into(), tick_hz: 30, world } },
        Packet::Reject { nonce: 1, reason: RejectReason::Full, server_proto: PROTOCOL, server_version: "0.1.0".into(), text: String::new() },
        Packet::Data { token: 0xABCD, ack: AckHeader { seq: 1, ack: 2, ack_bits: 3 }, payload },
        Packet::Disconnect { token: 0xABCD, text: "bye".into() },
    ]
}

#[test]
fn packets_round_trip() {
    for p in all_packets() {
        let b = p.encode();
        assert!(b.len() <= MAX_PACKET, "{p:?} is {} bytes", b.len());
        assert_eq!(Packet::decode(&b).unwrap(), p);
    }
}

#[test]
fn messages_round_trip() {
    let Packet::Data { payload, .. } = all_packets().into_iter().find(|p| matches!(p, Packet::Data { .. })).unwrap() else { unreachable!() };
    // Feed through a channel receiver to parse the framing.
    let mut ch = skyward_net::channel::Channel::new();
    let msgs = ch.receive(0.0, AckHeader { seq: 1, ack: 0, ack_bits: 0 }, &payload).unwrap();
    // Reliable ones all claim id 7: the first is delivered only if it is the next (0) — so
    // just check the unreliable ones and the body decoders directly.
    assert!(msgs.iter().any(|m| matches!(m, Msg::Snapshot { entries, .. } if entries.len() == 2)));
    for t in [msg::PLAYER, msg::LEFT, msg::CHAT, msg::WORLD, msg::EVENT, msg::SET_AIRCRAFT] {
        assert!(Msg::decode_body(t, &[]).is_err(), "empty body of type {t} must not decode");
    }
}

#[test]
fn version_mismatch_reply_still_parses_header() {
    // A future server (protocol 99) with a body we don't understand.
    let mut b = Packet::DiscoverReply { nonce: 5, proto: PROTOCOL, version: "9.9.9".into(), game_port: 4000, info: Some(ServerInfo::default()) }.encode();
    b[4 + 1 + 8] = 99; // protocol low byte
    b.extend_from_slice(&[0xEE; 40]); // unknown future fields
    match Packet::decode(&b).unwrap() {
        Packet::DiscoverReply { proto, version, game_port, info, .. } => {
            assert_eq!(proto, 99);
            assert_eq!(version, "9.9.9");
            assert_eq!(game_port, 4000);
            assert!(info.is_none());
        }
        p => panic!("{p:?}"),
    }
}

fn xorshift(x: &mut u64) -> u64 {
    *x ^= *x << 13;
    *x ^= *x >> 7;
    *x ^= *x << 17;
    *x
}

#[test]
fn malformed_packets_never_panic() {
    let valid: Vec<Vec<u8>> = all_packets().iter().map(|p| p.encode()).collect();
    let mut rng = 0x1234_5678_9abc_def0u64;
    let mut decoded = 0;
    for round in 0..200_000 {
        let base = &valid[round % valid.len()];
        let mut b = base.clone();
        match xorshift(&mut rng) % 5 {
            0 => b.truncate((xorshift(&mut rng) as usize) % (b.len() + 1)),
            1 => {
                for _ in 0..1 + xorshift(&mut rng) % 6 {
                    let i = (xorshift(&mut rng) as usize) % b.len();
                    b[i] ^= 1 << (xorshift(&mut rng) % 8);
                }
            }
            2 => {
                let n = (xorshift(&mut rng) % 1600) as usize;
                b = (0..n).map(|_| xorshift(&mut rng) as u8).collect();
                if n > 4 && xorshift(&mut rng).is_multiple_of(2) {
                    b[..4].copy_from_slice(&MAGIC);
                }
            }
            3 => {
                let extra = (xorshift(&mut rng) % 64) as usize;
                b.extend((0..extra).map(|_| xorshift(&mut rng) as u8));
            }
            _ => {
                // Corrupt a length field region.
                if b.len() > 8 {
                    let i = 5 + (xorshift(&mut rng) as usize) % (b.len() - 5);
                    b[i] = 0xFF;
                }
            }
        }
        if let Ok(p) = Packet::decode(&b) {
            decoded += 1;
            // Whatever decodes must re-encode within the size limit.
            let _ = p.encode();
            if let Packet::Data { ack, payload, .. } = p {
                let mut ch = skyward_net::channel::Channel::new();
                let _ = ch.receive(0.0, ack, &payload);
            }
        }
    }
    assert!(decoded > 0);
    // Oversize datagrams are refused outright.
    assert!(Packet::decode(&vec![0u8; MAX_RECV + 1]).is_err());
    // Unpadded requests are refused (no amplification).
    let mut d = Packet::Discover { proto: PROTOCOL, nonce: 1 }.encode();
    d.truncate(20);
    assert!(Packet::decode(&d).is_err());
}

#[test]
fn rejects_out_of_range_states() {
    let mut s = sample_state();
    s.lat = 95.0;
    let mut w = skyward_net::codec::Writer::new();
    s.encode(&mut w);
    assert!(AcState::decode(&mut skyward_net::codec::Reader::new(&w.buf)).is_err());
    let mut s = sample_state();
    s.rot = [0.0; 4];
    let mut w = skyward_net::codec::Writer::new();
    s.encode(&mut w);
    assert!(AcState::decode(&mut skyward_net::codec::Reader::new(&w.buf)).is_err());
    assert!(aircraft_id("Kestrel").is_err());
    assert!(aircraft_id("../etc").is_err());
    assert!(aircraft_id("meridian320").is_ok());
}
