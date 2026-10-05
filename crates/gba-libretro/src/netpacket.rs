//! libretro netpacket interface (env 78): the wireless adapter and the link
//! cable over the frontend's netplay.
//!
//! Trophy Hub's host (and any libretro netplay frontend) drives cores that
//! implement `RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE`. We hand it a struct
//! of callbacks; on session start it gives us a `send_fn` for pushing bytes at
//! peers and calls our `receive` for each inbound packet. We bridge that to
//! `gba_core::rfu`, which holds the whole protocol and is unit-tested against
//! a model of this transport.
//!
//! **Netpacket state lives in its own global, deliberately separate from the
//! core's `State`.** The frontend calls `receive` from inside `poll_receive`,
//! which we call from `retro_run` while `State` is already borrowed, so
//! reaching for the core from the callback would be a re-entrant `&mut`. The
//! callback therefore only queues bytes; `retro_run` drains the queue into the
//! core afterwards. The same shape as PocketRust's link cable, for the same
//! reason.
//!
//! **The cable is the one exception, and it has to be.** A cable packet is
//! handled inside `receive` and answered from there, because the peer that sent
//! it is BLOCKED waiting for the answer: hardware presents a child's
//! SIOMLT_SEND continuously, so making the peer wait for our next frame would
//! stall it for 16 ms per transfer and the Pokemon protocol wants nine of them
//! a frame. That is safe for the same reason the rest of this is: the cable's
//! state is a `CableProto` in this module's own global and never touches
//! `State`.

use gba_core::cable::{CableProto, LinkCable, MAX_UNITS, PACKET_LEN};
use std::cell::UnsafeCell;
use std::ffi::{c_char, c_void};
use std::time::{Duration, Instant};

pub const RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE: u32 = 78;
const RETRO_NETPACKET_RELIABLE: i32 = 1 << 0;
const RETRO_NETPACKET_FLUSH_HINT: i32 = 1 << 2;

/// The protocol string the frontend compares between peers before it will pair
/// them.
///
/// Our adapter packets are byte-identical to gpSP's `RFU1` format, so claiming
/// gpSP's `"gpSP v1.0"` here would let a device running gpSP into a session with
/// one running this core, which would be a useful reference peer. That is
/// deliberately NOT done: a peer that is not genuinely compatible must refuse a
/// session rather than corrupt one silently. Bump the number here whenever the
/// wire changes in a way an older build would mis-parse.
///
/// Bumped from `-rfu-1` when the cable landed, so the string stays an honest
/// statement of what this build speaks. **In Trophy Hub that is all it is.**
/// Checked at source: the host's netpacket transport only prints
/// `protocol=%s` at registration and never compares it between peers, so
/// bumping it does NOT make a mixed-version pair refuse here. A frontend that
/// does compare gets a clean refusal; on this one, a device running a
/// pre-cable build pairs and then never answers a clock. Both devices come off
/// the same APK, which is what actually keeps them in step.
///
/// The Android side documents this core as advertising `-rfu-1`
/// (`NetplayPlatformOverride.kt`), which this makes stale. Nothing reads it.
static PROTOCOL: &[u8] = b"pocketrustadvance-link-4\0";

/// How long a parent waits for a child's reply before giving up and reporting
/// the transfer to the game as a failure.
///
/// Deliberately shorter than PocketRust's 500 ms. A Game Boy cable moves one
/// byte per transfer; this one runs up to nine transfers an emulated frame, so
/// the same timeout would stall for over a second of wall clock per frame with
/// a dead peer. 150 ms still allows fifteen retransmits, which is generous
/// against a LAN round trip under a millisecond and a WAN one of tens.
const EXCHANGE_TIMEOUT: Duration = Duration::from_millis(150);

/// Re-send an unanswered clock this often. Nothing beneath this protocol
/// retransmits: the host's send ignores the reliable flag and calls sendto.
///
/// **Was 10 ms, which was below the round trip it was meant to outlast.**
/// Measured between the owner's two devices on his own Wi-Fi: 6.0 ms min,
/// 8.1 ms mean, 10.7 ms max. So a 10 ms timer fired on a large fraction of
/// perfectly healthy transfers, sent a duplicate, and the reply that was
/// already in flight arrived to find the parent had moved its expectations on.
/// 40 ms sits clear of the jitter and still allows three attempts inside the
/// timeout. A genuinely lost packet now costs 40 ms instead of 10, which is the
/// right way round: loss measured 0% over the same link, spurious retransmits
/// were happening constantly.
const RETRANSMIT_AFTER: Duration = Duration::from_millis(40);

/// How long to wait between polls while blocked.
///
/// **This replaces a 2000-iteration hot spin, which was actively harmful here.**
/// Each spin called the frontend's `poll_receive`, which takes the host's receive
/// queue lock, so the parent hammered that lock thousands of times per transfer
/// while the host's rx thread was trying to take the same lock to deliver the
/// reply the parent was waiting for. The spin was designed for a sub-millisecond
/// round trip; against a real 8 ms one it buys nothing and contends with the
/// thread it depends on. 250 us polls the reply in about 32 steps over a typical
/// wait and bounds the latency this adds to a quarter of a millisecond.
const POLL_INTERVAL: Duration = Duration::from_micros(250);

/// Consecutive give-ups after which the cable stops waiting at all.
///
/// Without this a peer that has gone away costs the full timeout on each of
/// nine transfers a frame, and the emulator appears to hang instead of the game
/// reaching its own link-error screen. Any inbound cable packet clears it, so a
/// peer that comes back is picked straight up.
const FAILURES_BEFORE_OPEN_CIRCUIT: u32 = 3;

type SendFn = unsafe extern "C" fn(flags: i32, buf: *const c_void, len: usize, client_id: u16);
type PollReceiveFn = unsafe extern "C" fn();
type StartFn = unsafe extern "C" fn(u16, SendFn, PollReceiveFn);
type ReceiveFn = unsafe extern "C" fn(*const c_void, usize, u16);
type StopFn = unsafe extern "C" fn();
type PollFn = unsafe extern "C" fn();
type ConnectedFn = unsafe extern "C" fn(u16) -> bool;
type DisconnectedFn = unsafe extern "C" fn(u16);

#[repr(C)]
pub struct RetroNetpacketCallback {
    start: Option<StartFn>,
    receive: Option<ReceiveFn>,
    stop: Option<StopFn>,
    poll: Option<PollFn>,
    connected: Option<ConnectedFn>,
    disconnected: Option<DisconnectedFn>,
    protocol_version: *const c_char,
}

impl RetroNetpacketCallback {
    /// Are the three callbacks a session actually needs all present? A struct
    /// handed over with a null `start` is accepted by the frontend and then
    /// silently never drives the core.
    pub fn has_start_receive_stop(&self) -> bool {
        self.start.is_some() && self.receive.is_some() && self.stop.is_some()
    }
}

// SAFETY: the frontend only reads this struct (function pointers and a static
// string), and we share it as a &'static.
unsafe impl Sync for RetroNetpacketCallback {}

/// The callback struct handed to the frontend via env 78.
pub static CALLBACK: RetroNetpacketCallback = RetroNetpacketCallback {
    start: Some(np_start),
    receive: Some(np_receive),
    stop: Some(np_stop),
    poll: None,
    connected: None,
    disconnected: None,
    protocol_version: PROTOCOL.as_ptr() as *const c_char,
};

struct Net {
    send_fn: Option<SendFn>,
    poll_receive_fn: Option<PollReceiveFn>,
    active: bool,
    /// Our own id in the session. Kept for diagnostics; the adapter addresses
    /// peers by the ids the frontend reports on inbound packets.
    self_id: u16,
    /// Packets received inside `poll_receive`, waiting to be handed to the
    /// core once it is no longer borrowed. Cable packets never reach here: they
    /// are answered in `np_receive`.
    inbox: Vec<(u16, Vec<u8>)>,
    /// The link cable's wire protocol. Separate from the adapter's because they
    /// are two different devices on the same port, told apart by packet magic.
    cable: CableProto,
    /// Peer ids we have exchanged a cable packet with. The frontend reports
    /// joins only to the host, so a child learns its peers from the wire.
    cable_peers: [bool; MAX_UNITS],
    /// Consecutive exchanges the cable gave up on.
    cable_failures: u32,
    /// Exchanges completed and exchanges abandoned, for the heartbeat.
    pub cable_done: u64,
    pub cable_lost: u64,
    /// Clocks re-sent because no reply had arrived yet.
    ///
    /// The first device run put the parent at 16.6 ms per transfer against a
    /// measured 8.1 ms round trip, so something was costing roughly a second
    /// round trip and a spurious retransmit was the obvious suspect. Counted
    /// rather than reasoned about, because that guess has been wrong before.
    pub cable_rtx: u64,
    /// Wall time a parent spent blocked, summed, counted and peaked. The sum and
    /// the count give the mean per transfer, which is the number that decides
    /// whether a blocking cable can hold 60 fps at all.
    pub cable_wait_us: u64,
    pub cable_waits: u64,
    pub cable_wait_max_us: u64,
}

impl Net {
    const fn new() -> Net {
        Net {
            send_fn: None,
            poll_receive_fn: None,
            active: false,
            self_id: 0,
            inbox: Vec::new(),
            cable: CableProto::new(),
            cable_peers: [false; MAX_UNITS],
            cable_failures: 0,
            cable_done: 0,
            cable_lost: 0,
            cable_rtx: 0,
            cable_wait_us: 0,
            cable_waits: 0,
            cable_wait_max_us: 0,
        }
    }

    /// Drop everything the cable was holding. Run on both session edges: a
    /// sequence number or a half-finished exchange carried into a new session
    /// would be answered as if it belonged to it.
    fn reset_cable(&mut self) {
        self.cable.reset();
        self.cable_peers = [false; MAX_UNITS];
        self.cable_failures = 0;
    }

    /// Units on the bus including us: 2 while a session is live, 1 otherwise.
    ///
    /// A live session means the frontend paired these devices, so the peer is
    /// taken to be there from the start rather than only once it has spoken. A
    /// game reads SD to decide whether to offer linking at all, and making that
    /// wait for the first packet would mean nothing ever sends one.
    ///
    /// Capped at 2 because that is what the cable implements. A third device
    /// would otherwise get a bus whose SD says four units are ready while slots
    /// 2 and 3 carry nothing, which is worse than a session that plainly only
    /// links two. `cable_extra_peers` counts the ones left out.
    fn cable_units(&self) -> u8 {
        // Elected, not merely connected. A session that is live but whose roles
        // are unsettled must look like a cable with nothing on the far end: the
        // game reads SD low, decides it has no partner, and does not offer to
        // link. The alternative is what the second device session did, where both
        // ends believed they were the parent and both games offered the leader
        // menu.
        if self.active && !self.cable.refusing() && self.cable.elected() {
            2
        } else {
            1
        }
    }

    /// Peers beyond the one this cable can carry.
    fn cable_extra_peers(&self) -> usize {
        self.cable_peers.iter().filter(|&&p| p).count().saturating_sub(1)
    }
}

struct Global(UnsafeCell<Net>);
// SAFETY: libretro serializes every call into the core, and the netpacket
// callbacks are documented to run on the emulation thread inside `retro_run`
// or inside one another, never concurrently.
unsafe impl Sync for Global {}

static NET: Global = Global(UnsafeCell::new(Net::new()));

fn with_net<R>(f: impl FnOnce(&mut Net) -> R) -> R {
    // SAFETY: see `Global`.
    unsafe { f(&mut *NET.0.get()) }
}

unsafe extern "C" fn np_start(client_id: u16, send: SendFn, poll_receive: PollReceiveFn) {
    with_net(|n| {
        n.send_fn = Some(send);
        n.poll_receive_fn = Some(poll_receive);
        n.self_id = client_id;
        n.active = true;
        n.inbox.clear();
        n.reset_cable();
        // An identity for the role election. The frontend's peer number cannot
        // carry the roles (see `NetCable::id`), so the two ends compare numbers
        // they each draw and the lower one is the parent. The clock supplies the
        // entropy, mixed with our own peer number so that two devices starting in
        // the same microsecond still differ.
        let nanos = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0);
        let nonce = nanos
            .wrapping_mul(0x9E37_79B9_7F4A_7C15)
            .wrapping_add((client_id as u64) << 17)
            .wrapping_add(std::process::id() as u64);
        n.cable.set_identity(nonce);
    });
    flush_cable_outbox();
}

unsafe extern "C" fn np_receive(buf: *const c_void, len: usize, client_id: u16) {
    if buf.is_null() || len == 0 {
        return;
    }
    // SAFETY: the frontend guarantees `buf` is readable for `len` bytes for the
    // duration of this call, so the copy has to happen here.
    let bytes = unsafe { std::slice::from_raw_parts(buf as *const u8, len) }.to_vec();
    if is_cable_packet(&bytes) {
        with_net(|n| {
            if let Some(slot) = n.cable_peers.get_mut(client_id as usize) {
                *slot = true;
            }
            // Hearing anything at all means the peer is alive, so a cable that
            // had stopped waiting starts waiting again.
            n.cable_failures = 0;
            n.cable.on_packet(&bytes);
        });
        // A reply leaves now rather than at the end of the frame: the peer that
        // sent the clock is blocked on it.
        flush_cable_outbox();
        return;
    }
    with_net(|n| n.inbox.push((client_id, bytes)));
}

/// Is this one of the cable's packets rather than the adapter's? Both ride the
/// same session and each keys on its own four-byte magic.
fn is_cable_packet(bytes: &[u8]) -> bool {
    // The FAMILY, not the whole magic. A peer on a different cable version has
    // to be recognised as ours so the mismatch can be named; matching the whole
    // magic would make it look like no peer at all and the session would
    // silently do nothing.
    // And the length bound is the FAMILY's, not ours. It used to be PACKET_LEN,
    // which quietly undid the paragraph above the moment the layout grew: a
    // 9-byte version-2 packet is shorter than a version-3 one, so it failed this
    // test, went to the adapter's inbox, and the version mismatch was never
    // counted. Two devices on different builds would have seen no peer at all.
    bytes.len() >= 4 && bytes[..3] == gba_core::cable::FAMILY
}

/// Put everything the cable has queued on the wire.
fn flush_cable_outbox() {
    for pkt in with_net(|n| n.cable.take_outbox()) {
        send(RETRO_NETPACKET_BROADCAST, &pkt);
    }
}

unsafe extern "C" fn np_stop() {
    with_net(|n| {
        n.send_fn = None;
        n.poll_receive_fn = None;
        n.active = false;
        n.inbox.clear();
        n.reset_cable();
    });
}

/// Is a netplay session live?
pub fn is_active() -> bool {
    with_net(|n| n.active)
}

/// Our own peer id in the session, as the frontend assigned it.
pub fn self_id() -> u16 {
    with_net(|n| n.self_id)
}

/// Ask the frontend to deliver anything waiting, then take what arrived.
///
/// Calling `poll_receive` re-enters us through `np_receive`, which is why that
/// callback only touches this module's own state.
pub fn poll_receive() {
    let poll = with_net(|n| n.poll_receive_fn);
    if let Some(poll) = poll {
        // SAFETY: called on the emulation thread, which is where libretro
        // requires it. It re-enters us through `np_receive`, so nothing may be
        // borrowed across this call.
        unsafe { poll() };
    }
}

pub fn drain_inbound() -> Vec<(u16, Vec<u8>)> {
    let poll = with_net(|n| n.poll_receive_fn);
    if let Some(poll) = poll {
        // SAFETY: called from `retro_run` on the emulation thread, which is
        // where libretro requires it.
        unsafe { poll() };
    }
    with_net(|n| std::mem::take(&mut n.inbox))
}

/// Every peer in the session.
pub const RETRO_NETPACKET_BROADCAST: u16 = 0xFFFF;

/// Put one packet on the wire.
///
/// Reliable and flushed: the adapter protocol has no sequence numbers of its
/// own, so a dropped connect or disconnect would strand a player, and a
/// re-ordered data block would corrupt a trade.
pub fn send(to: u16, bytes: &[u8]) {
    let send_fn = with_net(|n| n.send_fn);
    if let Some(send_fn) = send_fn {
        // SAFETY: called from `retro_run`, as libretro requires.
        unsafe {
            send_fn(
                RETRO_NETPACKET_RELIABLE | RETRO_NETPACKET_FLUSH_HINT,
                bytes.as_ptr() as *const c_void,
                bytes.len(),
                to,
            );
        }
    }
}

/// Cable transfers completed and transfers given up on, and peers this cable
/// could not carry. All three because a silent cable and a broken one look
/// identical from outside, which is exactly how two adapter bugs stayed
/// invisible for a whole device session.
pub struct CableStats {
    pub done: u64,
    pub lost: u64,
    pub extra_peers: usize,
    pub bad_version: u64,
    pub rtx: u64,
    pub wait_us: u64,
    pub waits: u64,
    pub wait_max_us: u64,
    /// Clocks a child answered while a previous transfer sat uncollected. The
    /// precondition of a trade that corrupts while every other counter here
    /// reads perfect, so this is the one number on the heartbeat that can be
    /// non-zero while the link looks healthy.
    pub stale_risk: u64,
    /// Scanlines this child waited for its parent, and times it stopped waiting.
    /// A trade with holds and no starvation is the pacing working; starvation
    /// means it stopped trusting the parent and ran free, which is where a
    /// desync starts.
    pub holds: u64,
    pub starved: u64,
    /// This end's position minus the peer's, in emulated cycles, from the last
    /// reply. One device's log can answer "how far apart were we" without the
    /// other's.
    pub skew: i32,
    /// Clocks answered before the game had rearmed, and clocks answered at all.
    /// The precise form of the stale-word hazard, where `stale_risk` is a proxy.
    pub cold: u64,
    pub answered: u64,
    /// Which end the election made us, and how many identity collisions it broke.
    /// `None` means the roles are not settled, which the game sees as no peer.
    pub role: Option<u8>,
    pub ties: u64,
}

pub fn cable_stats() -> CableStats {
    with_net(|n| CableStats {
        done: n.cable_done,
        lost: n.cable_lost,
        extra_peers: n.cable_extra_peers(),
        bad_version: n.cable.version_mismatch,
        rtx: n.cable_rtx,
        wait_us: n.cable_wait_us,
        waits: n.cable_waits,
        wait_max_us: n.cable_wait_max_us,
        stale_risk: n.cable.stale_risk,
        holds: n.cable.holds,
        starved: n.cable.starved,
        skew: n.cable.skew().unwrap_or(0),
        cold: n.cable.answered_cold,
        answered: n.cable.answered,
        role: n.cable.role(),
        ties: n.cable.election_ties,
    })
}

/// The cable the serial engine drives. Holds nothing: all state is in the NET
/// global, so the core can own this across a session start and stop.
pub struct NetCable;

impl LinkCable for NetCable {
    fn units(&self) -> u8 {
        with_net(|n| n.cable_units())
    }

    /// Which end of the cable this is, elected on the wire.
    ///
    /// **It used to be the frontend's peer number, and that was wrong.** Measured
    /// on the owner's two devices on 2026-10-05: the tablet saw the phone as
    /// `cid=1` and the phone saw the tablet as `cid=1`, so both took themselves
    /// for peer 0, both played the parent, and both games offered the leader
    /// menu. An earlier session of the same pair came out 0 and 1, so the
    /// numbering is not stable between sessions either, and the run that worked
    /// worked by luck.
    ///
    /// Zero while unsettled is safe because `cable_units` reports one unit until
    /// the election completes, so the game sees no partner and never clocks.
    fn id(&self) -> u8 {
        with_net(|n| n.cable.role().unwrap_or(0))
    }

    fn set_output(&mut self, word: u16) {
        with_net(|n| n.cable.set_output(word));
    }

    /// Charge this core's progress to the cable, and put a parent's position on
    /// the wire once a frame.
    ///
    /// The tick is sent from here rather than from the core because it is a
    /// transport concern: the core only says how much time passed. A child's
    /// `advance` sends nothing.
    fn advance(&mut self, cycles: u32) {
        let send = with_net(|n| {
            n.cable.advance(cycles);
            // A tick is a PARENT'S position, so only an elected parent may send
            // one: an unsettled end emitting ticks would feed its peer's pacing
            // budget while claiming a role it has not got.
            let ticked = n.cable.role() == Some(0) && n.cable.maybe_tick();
            // And while unsettled, say hello once a frame until the peer answers.
            // The first announcement can go out while the peer's session is still
            // coming up, and a lost one must not leave the pair silent forever.
            let said_hello = n.cable.maybe_hello();
            ticked || said_hello
        });
        if send {
            flush_cable_outbox();
        }
    }

    /// Wait for the parent to catch up, if this child has run ahead of it.
    ///
    /// Sleeps the same 250 us the parent's own wait uses, so a held child costs
    /// one timer sleep per interval rather than a spin. The caller pumps the
    /// transport between calls, and `CableProto::hold` bounds its own patience,
    /// so a parent that has gone away cannot freeze this core.
    fn hold(&mut self) -> bool {
        if !with_net(|n| n.cable.hold()) {
            return false;
        }
        std::thread::sleep(POLL_INTERVAL);
        true
    }

    fn parent_start(&mut self, own: u16) {
        with_net(|n| n.cable.begin_exchange(own));
        // Out now, not at the end of the frame. The whole point of starting the
        // exchange here and collecting it at completion is that the round trip
        // overlaps the transfer's own duration.
        flush_cable_outbox();
    }

    fn parent_result(&mut self) -> Option<[u16; MAX_UNITS]> {
        let (seq, open_circuit) = with_net(|n| {
            (
                n.cable.awaiting(),
                n.cable_failures >= FAILURES_BEFORE_OPEN_CIRCUIT,
            )
        });
        let seq = seq?;
        if open_circuit {
            // The peer has not answered three exchanges running. Fail this one
            // at once instead of spending the timeout on it: nine transfers a
            // frame times 150 ms is a frozen emulator, where failing fast gets
            // the game to its own link-error screen.
            with_net(|n| {
                n.cable.abandon(seq);
                n.cable_lost += 1;
            });
            flush_cable_outbox();
            return None;
        }

        let began = Instant::now();
        let deadline = began + EXCHANGE_TIMEOUT;
        let mut next_retry = began + RETRANSMIT_AFTER;
        loop {
            // Short borrow, poll, short borrow: `poll_receive` re-enters us
            // through `np_receive`, which borrows the same global.
            poll_receive();
            if let Some(word) = with_net(|n| n.cable.take_reply(seq)) {
                let waited = began.elapsed().as_micros() as u64;
                let own = with_net(|n| {
                    n.cable_failures = 0;
                    n.cable_done += 1;
                    n.cable_wait_us += waited;
                    n.cable_waits += 1;
                    n.cable_wait_max_us = n.cable_wait_max_us.max(waited);
                    // The word LATCHED when this exchange started, not whatever
                    // the register holds now: a game that writes SIOMLT_SEND
                    // again mid-transfer must not change what this transfer
                    // carried, or the parent's own slot disagrees with what the
                    // child was told.
                    n.cable.pending_out()
                });
                return Some([own, word, gba_core::cable::ABSENT, gba_core::cable::ABSENT]);
            }
            if !with_net(|n| n.active) {
                break; // the session was torn down under us
            }
            let now = Instant::now();
            if now >= deadline {
                break;
            }
            if now >= next_retry {
                with_net(|n| {
                    n.cable.retry_exchange();
                    n.cable_rtx += 1;
                });
                flush_cable_outbox();
                next_retry = now + RETRANSMIT_AFTER;
            }
            std::thread::sleep(POLL_INTERVAL);
        }

        let waited = began.elapsed().as_micros() as u64;
        with_net(|n| {
            n.cable.abandon(seq);
            n.cable_failures += 1;
            n.cable_lost += 1;
            n.cable_wait_us += waited;
            n.cable_waits += 1;
            n.cable_wait_max_us = n.cable_wait_max_us.max(waited);
        });
        flush_cable_outbox(); // abandon only queues the give-up
        None
    }

    fn flush(&mut self) {
        with_net(|n| {
            n.cable.flush_pending();
            n.cable_failures = 0;
        });
    }

    fn child_clock(&mut self) -> Option<(u16, u16)> {
        let got = with_net(|n| n.cable.child_clock());
        if got.is_some() {
            with_net(|n| n.cable_done += 1);
        }
        got
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Session state is a process global, and cargo runs tests on parallel
    /// threads, so anything that touches it has to be serialized here.
    ///
    /// This is not paranoia: without it the suite corrupted the heap. Every
    /// test still PASSED and the process then died at exit with
    /// STATUS_HEAP_CORRUPTION, because two threads were pushing into the same
    /// `Vec` at once. Production is unaffected, since libretro serializes every
    /// call into a core onto the emulation thread, which is the same guarantee
    /// `Global` documents.
    static NET_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    /// Take the lock and start from a clean session.
    fn locked() -> std::sync::MutexGuard<'static, ()> {
        let g = NET_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        with_net(|n| {
            n.inbox.clear();
            n.send_fn = None;
            n.poll_receive_fn = None;
            n.active = false;
        });
        g
    }

    #[test]
    fn the_env_command_is_the_literal_78() {
        // Asserted against the number rather than against our own constant,
        // for the same reason the rumble interface is: a test phrased in terms
        // of the symbol it is checking agrees with any bug in it. 78 is plain,
        // with no experimental bit, unlike the sensor interface.
        assert_eq!(RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE, 78);
        assert_eq!(RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE & 0x1_0000, 0);
    }

    #[test]
    fn the_protocol_string_is_nul_terminated() {
        // It is handed to the frontend as a C string. Without the terminator
        // the frontend reads off the end of our static and compares garbage,
        // which would fail to pair two identical builds.
        assert_eq!(*PROTOCOL.last().unwrap(), 0);
        assert!(PROTOCOL[..PROTOCOL.len() - 1].iter().all(|&b| b != 0));
    }

    #[test]
    fn the_protocol_string_is_not_gpsps() {
        // Our packets are deliberately byte-compatible with gpSP, which makes
        // it tempting to claim its version string and gain a reference peer.
        // Until the session layer has been checked against gpSP end to end,
        // pairing with it would risk corrupting a real trade rather than
        // failing cleanly.
        assert_ne!(PROTOCOL, b"gpSP v1.0\0");
    }

    #[test]
    fn a_cable_packet_is_answered_here_and_never_reaches_the_inbox() {
        let _g = locked();
        // The peer that sends a clock is BLOCKED on the reply, so queueing it
        // for the next frame the way adapter packets are queued would stall it
        // for 16 ms per transfer.
        let mut clock = [0u8; PACKET_LEN];
        clock[..4].copy_from_slice(&gba_core::cable::MAGIC.to_be_bytes());
        clock[4] = gba_core::cable::TAG_CLOCK;
        clock[5] = 7;
        clock[6..8].copy_from_slice(&0x8FFFu16.to_be_bytes());
        with_net(|n| n.cable.set_output(0xB9A0));
        unsafe { np_receive(clock.as_ptr() as *const c_void, clock.len(), 0) };
        assert!(
            with_net(|n| n.inbox.is_empty()),
            "a cable packet must not be queued for the adapter to parse"
        );
        assert_eq!(
            with_net(|n| n.cable.child_clock()),
            Some((0x8FFF, 0xB9A0)),
            "the transfer has to reach the serial engine"
        );
        assert!(with_net(|n| n.cable_peers[0]), "and the sender counts as a peer");
    }

    #[test]
    fn an_adapter_packet_still_goes_to_the_inbox() {
        let _g = locked();
        // Both devices ride the same session. Telling them apart by magic is the
        // whole mechanism, so a regression here breaks the 43 adapter titles
        // rather than the cable.
        let rfu = [0x52u8, 0x46, 0x55, 0x31, 1, 0, 0, 0, 0, 0, 0, 0];
        unsafe { np_receive(rfu.as_ptr() as *const c_void, rfu.len(), 1) };
        assert_eq!(with_net(|n| n.inbox.len()), 1);
        assert_eq!(with_net(|n| n.cable.child_clock()), None);
        assert!(!is_cable_packet(&rfu));
    }

    /// The advertised protocol string has to name the wire version it speaks.
    ///
    /// These are two different refusals and they have to agree. The core refuses
    /// a peer on another wire by the version byte in every packet, which works
    /// and is tested. The string is the refusal the FRONTEND would perform, and
    /// Trophy Hub's host currently throws it away, which is filed. A discarded
    /// string is still not a licence to advertise something false: the day the
    /// host compares it, a stale string would pair two builds that cannot talk.
    /// Pinned together here so a wire bump that forgets the string fails on the
    /// desk rather than on two phones.
    #[test]
    fn the_protocol_string_advertises_the_wire_version_it_speaks() {
        let advertised = std::str::from_utf8(&PROTOCOL[..PROTOCOL.len() - 1]).unwrap();
        let expected =
            format!("pocketrustadvance-link-{}", gba_core::cable::WIRE_VERSION as char);
        assert_eq!(
            advertised, expected,
            "the string and the wire version must name the same cable"
        );
    }

    #[test]
    fn the_protocol_string_refuses_a_build_that_predates_the_cable() {
        // An older build answers no cable clock at all, which to a player is a
        // trade that hangs rather than a session that will not start.
        assert_ne!(PROTOCOL, b"pocketrustadvance-rfu-1\0");
    }

    #[test]
    fn a_null_or_empty_packet_is_ignored() {
        let _g = locked();
        unsafe {
            np_receive(std::ptr::null(), 16, 1);
            np_receive([1u8, 2, 3].as_ptr() as *const c_void, 0, 1);
        }
        assert!(with_net(|n| n.inbox.is_empty()));
    }

    #[test]
    fn a_received_packet_is_copied_out_with_its_sender() {
        let _g = locked();
        let buf = [0xDEu8, 0xAD, 0xBE, 0xEF];
        unsafe { np_receive(buf.as_ptr() as *const c_void, buf.len(), 7) };
        let got = with_net(|n| std::mem::take(&mut n.inbox));
        assert_eq!(got, vec![(7u16, vec![0xDE, 0xAD, 0xBE, 0xEF])]);
    }

    #[test]
    fn stopping_a_session_drops_everything_including_queued_packets() {
        let _g = locked();
        unsafe {
            np_receive([9u8].as_ptr() as *const c_void, 1, 1);
            np_stop();
        }
        assert!(!is_active());
        assert!(with_net(|n| n.inbox.is_empty()));
        assert!(with_net(|n| n.send_fn.is_none()));
    }
}
