//! Local IPv4 interfaces: where to send discovery broadcasts and which
//! addresses to announce over mDNS. Unix reads `getifaddrs`; elsewhere the
//! primary address (the route to a documentation address — nothing is sent)
//! and the limited broadcast address are used.

use std::net::{Ipv4Addr, UdpSocket};

#[derive(Clone, Debug, PartialEq)]
pub struct Iface {
    pub name: String,
    pub addr: Ipv4Addr,
    pub netmask: Ipv4Addr,
    /// Subnet broadcast address (None on point-to-point links).
    pub broadcast: Option<Ipv4Addr>,
    pub loopback: bool,
}

/// Directed broadcast of a subnet.
pub fn subnet_broadcast(addr: Ipv4Addr, mask: Ipv4Addr) -> Ipv4Addr {
    Ipv4Addr::from(u32::from(addr) | !u32::from(mask))
}

/// IPv4 interfaces that are up.
pub fn ipv4() -> Vec<Iface> {
    let mut v = platform();
    if v.iter().all(|i| i.loopback) {
        // Fallback: the address of the default route.
        if let Some(a) = primary() {
            v.push(Iface {
                name: "default".into(),
                addr: a,
                netmask: Ipv4Addr::new(255, 255, 255, 0),
                broadcast: Some(subnet_broadcast(a, Ipv4Addr::new(255, 255, 255, 0))),
                loopback: false,
            });
        }
    }
    v
}

/// The address the default route leaves from (no packet is sent).
pub fn primary() -> Option<Ipv4Addr> {
    let s = UdpSocket::bind((Ipv4Addr::UNSPECIFIED, 0)).ok()?;
    s.connect((Ipv4Addr::new(192, 0, 2, 1), 9)).ok()?;
    match s.local_addr().ok()?.ip() {
        std::net::IpAddr::V4(a) if !a.is_unspecified() => Some(a),
        _ => None,
    }
}

/// Where a LAN discovery broadcast goes: every subnet's broadcast address
/// plus the limited broadcast (255.255.255.255), deduplicated.
pub fn broadcast_targets() -> Vec<Ipv4Addr> {
    let mut t: Vec<Ipv4Addr> = ipv4().into_iter().filter(|i| !i.loopback).filter_map(|i| i.broadcast).collect();
    t.push(Ipv4Addr::BROADCAST);
    t.sort();
    t.dedup();
    t
}

#[cfg(unix)]
fn platform() -> Vec<Iface> {
    let mut out = Vec::new();
    // SAFETY: getifaddrs allocates a list we walk read-only and free once;
    // every pointer is checked for null before use.
    unsafe {
        let mut head: *mut libc::ifaddrs = std::ptr::null_mut();
        if libc::getifaddrs(&mut head) != 0 {
            return out;
        }
        let mut cur = head;
        while !cur.is_null() {
            let ifa = &*cur;
            cur = ifa.ifa_next;
            if ifa.ifa_addr.is_null() || (*ifa.ifa_addr).sa_family as i32 != libc::AF_INET {
                continue;
            }
            let flags = ifa.ifa_flags;
            if flags & libc::IFF_UP as u32 == 0 {
                continue;
            }
            let sin = &*(ifa.ifa_addr as *const libc::sockaddr_in);
            let addr = Ipv4Addr::from(u32::from_be(sin.sin_addr.s_addr));
            let netmask = if ifa.ifa_netmask.is_null() {
                Ipv4Addr::new(255, 255, 255, 0)
            } else {
                let m = &*(ifa.ifa_netmask as *const libc::sockaddr_in);
                Ipv4Addr::from(u32::from_be(m.sin_addr.s_addr))
            };
            let loopback = flags & libc::IFF_LOOPBACK as u32 != 0;
            let broadcast = (flags & libc::IFF_BROADCAST as u32 != 0).then(|| subnet_broadcast(addr, netmask));
            let name = if ifa.ifa_name.is_null() { String::new() } else { std::ffi::CStr::from_ptr(ifa.ifa_name).to_string_lossy().into_owned() };
            out.push(Iface { name, addr, netmask, broadcast, loopback });
        }
        libc::freeifaddrs(head);
    }
    out
}

#[cfg(not(unix))]
fn platform() -> Vec<Iface> {
    Vec::new()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn broadcast_maths() {
        assert_eq!(subnet_broadcast(Ipv4Addr::new(192, 168, 1, 20), Ipv4Addr::new(255, 255, 255, 0)), Ipv4Addr::new(192, 168, 1, 255));
        assert_eq!(subnet_broadcast(Ipv4Addr::new(10, 1, 2, 3), Ipv4Addr::new(255, 0, 0, 0)), Ipv4Addr::new(10, 255, 255, 255));
    }

    #[test]
    fn targets_include_limited_broadcast() {
        assert!(broadcast_targets().contains(&Ipv4Addr::BROADCAST));
    }
}
