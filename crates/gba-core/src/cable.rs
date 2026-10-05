//! The link cable, in Multi-Player mode.
//!
//! This is a real cable rather than a per-game protocol fake. gpSP, the
//! incumbent, has no cable at all: it recognises 16 ROMs by game code and
//! emulates the other GBA's *software* protocol for each one, which is why
//! Mario Kart Super Circuit has never linked on it. Modelling the port instead
//! means any game that uses Multi-Player mode works without being in a list.
//!
//! # How a Multi-Player transfer works
//!
//! Up to four units share one bus. The unit at the head of the chain is the
//! parent (ID 0) and is the only one that can start a transfer; the others are
//! children and are driven by the parent's clock. One transfer moves four
//! half-words at once: every unit's SIOMLT_SEND lands in every unit's
//! SIOMULTI<n>, where n is that unit's ID. Absent units read FFFF.
//!
//! So a transfer is a *simultaneous exchange*, not a request and a response,
//! and the child's word has to already be in SIOMLT_SEND when the parent
//! clocks. That is true on hardware too, which is why games write SIOMLT_SEND
//! from the serial interrupt handler of the previous transfer.
//!
//! # Why a blocking exchange is affordable here
//!
//! The parent cannot finish a transfer without the child's word, so it has to
//! wait for the network. Measured from the reference implementations: the
//! Pokemon link protocol runs **9 transfers per frame** once connected (gpSP's
//! `SLAVE_IRQ_CYCLES_C` is 28672 cycles, a 1+8 half-word frame per video
//! frame). PocketRust's Game Boy cable already ships a blocking per-transfer
//! exchange over this same transport at **17 per frame** (a byte is 4096
//! cycles of a 70224-cycle frame) and trades Pokemon on the owner's devices.
//! Nine is the easier number.
//!
//! The waiting is also overlapped with work rather than paid up front: the
//! parent starts the exchange, runs out the transfer's real duration (5755
//! cycles at 115200 baud with two units, about 343 us of emulated time), and
//! only blocks for whatever round trip is left over.
//!
//! # Scope
//!
//! **Two units.** A child assembles the result locally, because with two units
//! the parent's word plus its own is the whole set. Three and four unit sessions
//! need the parent to broadcast the assembled set and are deliberately not
//! implemented rather than half implemented: the transport reports a two-unit
//! bus no matter how many peers the session holds, and counts the ones it left
//! out, so a game sees a cable that plainly links two rather than one whose SD
//! claims four units are ready while two of the slots carry nothing.
//!
//! # One simplification worth knowing
//!
//! Each end computes the transfer's duration from its OWN SIOCNT baud field
//! rather than from the parent's, which is what hardware would use. It is not
//! carried on the wire because both ends of a session are the same game and set
//! the same rate; if they ever differed the words would still be right and only
//! the child's interrupt timing would drift. The clock packet has a spare byte
//! if that assumption ever has to go.
//!
//! # Layers
//!
//! [`CableProto`] is the wire format and nothing else: no sockets, no clock, no
//! callbacks. It consumes packets and produces them through queues so both ends
//! can be driven against each other on a desk. [`LinkCable`] is what the serial
//! engine talks to, so the same engine serves the networked transport and
//! [`local_pair`], which wires two cores together in one process.

use std::collections::VecDeque;

/// Cycles one Multi-Player transfer occupies, indexed by SIOCNT's baud field
/// (bits 0-1) and then by the number of attached units minus one.
///
/// Taken from mGBA's `GBASIOCyclesPerTransfer`, which is the local oracle for
/// values GBATEK gives only as bit rates. They are close to the arithmetic and
/// not equal to it: 16 bits at 9600 bps is 1.667 ms, or 27963 cycles of the
/// 16.78 MHz clock, against the 31976 here, because a transfer carries
/// start/stop framing per unit as well as the data.
pub const TRANSFER_CYCLES: [[u32; MAX_UNITS]; 4] = [
    [31976, 63427, 94884, 125829], //   9600 bps
    [8378, 16241, 24104, 31457],   //  38400 bps
    [5750, 10998, 16241, 20972],   //  57600 bps
    [3140, 5755, 8376, 10486],     // 115200 bps
];

/// Why there is no "make the transfer longer" fix, recorded so nobody tries it
/// again: it was tried, on device, and the arithmetic forbids it.
///
/// A networked transfer cannot complete without the peer's word, so each one
/// costs a round trip. Measured between the owner's two devices on his own
/// Wi-Fi: 6.0 ms min, 8.1 ms mean, 10.7 ms max, 0% loss, with Android already
/// holding WIFI_MODE_FULL_LOW_LATENCY. Pokemon's connected mode issues NINE
/// transfers per video frame.
///
/// Stretching the emulated transfer so the round trip hides inside it needs each
/// transfer to last longer than 8 ms. Letting the game fit nine of them into one
/// 280896-cycle frame needs each to last less than 1.86 ms. **Both cannot hold.**
/// Tried at 12 ms: both devices held 60 fps and the game said "we have a
/// transmission error" the instant it entered connected mode, because its
/// nine-word frame now needed six and a half video frames. Our cable was
/// blameless, with 8827 transfers completed on each end, counts agreeing to the
/// digit, and not one give-up or retransmit.
///
/// So nine round trips, 72 ms, have to happen inside what the game believes is
/// one 16.7 ms frame. The frame has to STRETCH IN WALL TIME, on both devices
/// together: a trade runs at about 14 fps and completes, instead of running at
/// 60 and failing. There is no method here for that yet. The approach that was
/// tried, a `wait_for_clock` on this trait which blocked a child as soon as its
/// game wrote SIOMLT_SEND, is reverted and described below; `docs/LOCKSTEP.md`
/// carries the design that replaces it.

/// What still has to be solved, so the next session starts from the finding and
/// not from the idea: **the two emulated clocks have to be held together, and
/// that cannot be done from a signal local to one device.**
///
/// Tried and reverted: blocking a child as soon as its game writes SIOMLT_SEND.
/// That write means "the next word is ready", NOT "I am waiting", and the game
/// goes on to do a whole frame of other work after it. Measured on device: the
/// child fell to 6.6 fps against a parent at 56.7, which is the first test's
/// ninefold divergence pointing the other way. A readiness signal is not a
/// waiting signal.
///
/// The information that actually decides it is the PARENT'S emulated frame
/// count, which only the parent has, so it has to go on the wire. A child
/// throttles at a frame boundary only while it is ahead. During the handshake
/// both ends run at 60 fps, the counts stay level and nothing throttles; in
/// connected mode the parent slows to the round trip and the child is held to
/// match. Note the hazard that rules out blocking mid-frame: a child blocked
/// while several clocks arrive answers them all from one `own_output`, which
/// corrupts a trade while leaving the wire looking perfectly healthy.
///
/// Units one Multi-Player bus can hold, which is also the number of SIOMULTI
/// slots.
pub const MAX_UNITS: usize = 4;

/// What an absent unit's slot reads. The bus is pulled high and nothing drives
/// it, so a game tests for this exact value to count its peers.
pub const ABSENT: u16 = 0xFFFF;

/// The serial engine's view of the cable.
///
/// One instance per core. Everything that can block or touch a socket is behind
/// this trait, so the register model in `memory.rs` is the same code on a desk
/// and on a phone.
pub trait LinkCable {
    /// Units on the bus including this one. 1 means the peer has gone.
    fn units(&self) -> u8;
    /// This unit's Multi-Player ID. 0 is the parent.
    fn id(&self) -> u8;
    /// Present `word` as this unit's SIOMLT_SEND.
    ///
    /// Called on every store to the register, not once per transfer: a child's
    /// reply is produced by its *transport* the moment the parent's clock
    /// arrives, long before its emulation looks at the port again, so the
    /// transport has to already hold the current value.
    fn set_output(&mut self, word: u16);
    /// Parent: begin clocking `own` out to the children.
    fn parent_start(&mut self, own: u16);
    /// Parent: every unit's word for the transfer [`LinkCable::parent_start`]
    /// began, blocking until the children answer.
    ///
    /// `None` means the cable gave up waiting, which the register model reports
    /// to the game as a transfer error rather than as made up data.
    fn parent_result(&mut self) -> Option<[u16; MAX_UNITS]>;
    /// Child: the parent clocked a transfer into us, as `(parent's word, the
    /// word we answered with)`.
    ///
    /// The second half matters: the child must show in its own SIOMULTI slot
    /// exactly what the peer saw, which is the value its transport answered
    /// with, not whatever SIOMLT_SEND holds by the time its emulation catches
    /// up.
    fn child_clock(&mut self) -> Option<(u16, u16)>;
    /// Throw away transfers in flight, because the game just selected
    /// Multi-Player mode and anything from before that is not addressed to the
    /// protocol it is about to run.
    ///
    /// Both players reach the link menu seconds apart, so a parent clocks for a
    /// while before its peer is listening. Without this the child inherits a
    /// queue of handshake attempts from before it was ready, delivers them with
    /// their interrupts, and reports a backlog that makes it look far behind
    /// when it has simply not started.
    fn flush(&mut self);
}

/// Wire-format family, "CBL".
///
/// Three bytes rather than a tag byte so a cable packet can never be mistaken
/// for a wireless adapter one: both ride the same netplay session, and
/// `rfu::net_receive` keys on its own "RFU1" magic for the same reason.
///
/// **The family and the version are separate on purpose.** A packet whose whole
/// magic failed to match would be indistinguishable from the adapter's, so a
/// peer running a different cable version would look like no peer at all and the
/// session would quietly do nothing. Splitting them means a mismatch is a thing
/// this core can SEE, name, and refuse.
pub const FAMILY: [u8; 3] = *b"CBL";

/// The cable wire version this build speaks, byte 3 of every packet.
///
/// The ASCII digit rather than the number, so a packet dump reads "CBL2" and
/// the family and version are one readable token on the wire. Bump it whenever
/// the layout or the MEANING of a field changes.
///
/// Went to `2` for a change with no layout in it at all: how a transfer is paced,
/// and whether a child throttles itself to its parent. Both ends decide that from
/// shared constants and never send them, so two builds that disagree would
/// quietly drift their protocol clocks apart, which is the exact silent-mismatch
/// failure this version byte exists to prevent. A semantic agreement counts as
/// the wire. There is no
/// negotiation and there should not be: two devices come off the same APK, so a
/// mismatch is a mis-install rather than a case to support.
pub const WIRE_VERSION: u8 = b'2';

/// The full four-byte header of a packet this build will act on, "CBL2".
pub const MAGIC: u32 = 0x4342_4C32;

/// `[magic, 1, seq, word, 0]`: "I am clocking `word` into you as exchange
/// `seq`."
pub const TAG_CLOCK: u8 = 1;
/// `[magic, 2, seq, word, lag]`: "for exchange `seq` I was presenting `word`,
/// and my game is `lag` transfers behind on picking up what you clocked in."
pub const TAG_REPLY: u8 = 2;
/// `[magic, 3, seq, 0, 0]`: "I waited out exchange `seq` and gave up."
///
/// Costs one packet on a path that is already broken and tells the peer it was
/// the one not answering, which is the only way either device can name the side
/// that is holding up the cable.
pub const TAG_GAVE_UP: u8 = 3;

/// Every cable packet is this long: magic, tag, seq, word, lag.
pub const PACKET_LEN: usize = 9;

/// Recently answered sequence numbers kept for duplicate suppression. One
/// exchange is outstanding at a time, so anything past a couple is slack;
/// eight covers a run of lost clocks, which advance the parent's counter past
/// values the child never saw.
const SEEN_WINDOW: usize = 8;

/// Backlog a child reports while it is keeping up.
///
/// ONE, not zero. The reply is built after the arriving transfer has been
/// pushed, so the transfer being answered is itself in the queue and a child
/// that is perfectly in step reports one. Zero only ever appears as the value
/// `peer_lag` holds before any reply has come back, which is why the two
/// initialisers below spell it out rather than using this.
pub const LAG_HEALTHY: u8 = 1;

/// Transfers a child will hold for its serial engine before dropping the
/// oldest.
///
/// Deliberately shallow. A child that is behind by more than a few transfers
/// has already broken the game's protocol, and holding a long backlog would
/// deliver stale words later rather than letting the game's own error path run.
const CHILD_QUEUE_MAX: usize = 8;

fn put(out: &mut [u8; PACKET_LEN], tag: u8, seq: u8, word: u16, lag: u8) {
    out[..4].copy_from_slice(&MAGIC.to_be_bytes());
    out[4] = tag;
    out[5] = seq;
    out[6..8].copy_from_slice(&word.to_be_bytes());
    out[8] = lag;
}

/// The cable's wire protocol.
///
/// Paired, like PocketRust's Game Boy cable and for the same reason: an
/// unpaired announcement cannot be told apart from an echo of your own word, so
/// a parent would read its own data back as the peer's. Each clock carries a
/// sequence number and the parent consumes only the reply that carries the same
/// one.
pub struct CableProto {
    /// What this unit presents on the bus (its SIOMLT_SEND).
    own_output: u16,
    /// Next sequence number for a parent exchange. Wraps; only equality is used.
    next_seq: u8,
    /// The exchange awaiting a reply, if any.
    awaiting: Option<u8>,
    /// The word being clocked in `awaiting`, kept so a lost clock can be re-sent.
    pending_out: u16,
    /// Sequence numbers already answered, so a retransmitted clock is
    /// idempotent.
    seen: [Option<u8>; SEEN_WINDOW],
    seen_pos: usize,
    /// The exact reply sent for the most recent new clock, replayed verbatim if
    /// that clock arrives again. Recomputing it would answer a stale clock with
    /// a fresh word.
    last_reply: [u8; PACKET_LEN],
    /// The reply that matched `awaiting`.
    reply: Option<u16>,
    /// Transfers a parent clocked into us, awaiting pickup by the serial engine,
    /// as `(parent's word, the word we answered with)`. Its depth is our lag.
    child_input: VecDeque<(u16, u16)>,
    /// Transfers dropped because the serial engine never collected them.
    pub dropped: u64,
    /// Packets from a peer speaking a cable version this build does not.
    ///
    /// Non-zero means the two devices are on different builds, and it latches:
    /// the cable refuses to carry anything from then on rather than guessing at
    /// a layout it does not know. A link that degrades into corrupt gameplay is
    /// far harder to attribute than one that plainly will not start.
    pub version_mismatch: u64,
    /// The backlog the peer reported in its most recent reply.
    peer_lag: u8,
    /// The peer told us it gave up waiting on one of our replies.
    peer_gave_up: bool,
    /// Packets for the transport to send.
    outbox: VecDeque<[u8; PACKET_LEN]>,
}

impl Default for CableProto {
    fn default() -> Self {
        Self::new()
    }
}

impl CableProto {
    pub const fn new() -> CableProto {
        CableProto {
            own_output: ABSENT,
            next_seq: 0,
            awaiting: None,
            pending_out: ABSENT,
            seen: [None; SEEN_WINDOW],
            seen_pos: 0,
            last_reply: [0; PACKET_LEN],
            reply: None,
            child_input: VecDeque::new(),
            dropped: 0,
            version_mismatch: 0,
            peer_lag: 0, // nothing reported yet, see LAG_HEALTHY
            peer_gave_up: false,
            outbox: VecDeque::new(),
        }
    }

    /// Forget all in-flight state. Called when a session starts or stops.
    pub fn reset(&mut self) {
        self.own_output = ABSENT;
        self.next_seq = 0;
        self.awaiting = None;
        self.pending_out = ABSENT;
        self.seen = [None; SEEN_WINDOW];
        self.seen_pos = 0;
        self.last_reply = [0; PACKET_LEN];
        self.reply = None;
        self.child_input.clear();
        self.dropped = 0;
        self.version_mismatch = 0;
        self.peer_lag = 0; // nothing reported yet, see LAG_HEALTHY
        self.peer_gave_up = false;
        self.outbox.clear();
    }

    /// What this unit presents on the bus.
    pub fn output(&self) -> u16 {
        self.own_output
    }

    /// Transfers accepted that our serial engine has not completed yet.
    pub fn lag(&self) -> u8 {
        self.child_input.len().min(u8::MAX as usize) as u8
    }

    /// The backlog the peer reported in its last reply. 0 while it keeps up.
    pub fn peer_lag(&self) -> u8 {
        self.peer_lag
    }

    /// Has the peer told us it gave up waiting on one of our replies? Consumes
    /// the flag.
    pub fn take_peer_gave_up(&mut self) -> bool {
        std::mem::replace(&mut self.peer_gave_up, false)
    }

    /// Present `word` on the bus. A child's reply is answered from this.
    pub fn set_output(&mut self, word: u16) {
        self.own_output = word;
    }

    /// Begin a parent exchange of `out`. Returns the sequence number to wait on.
    pub fn begin_exchange(&mut self, out: u16) -> u8 {
        let seq = self.next_seq;
        self.next_seq = self.next_seq.wrapping_add(1);
        self.awaiting = Some(seq);
        self.pending_out = out;
        self.reply = None;
        self.own_output = out;
        let mut pkt = [0u8; PACKET_LEN];
        put(&mut pkt, TAG_CLOCK, seq, out, 0);
        self.outbox.push_back(pkt);
        seq
    }

    /// Re-send the clock for the exchange still being awaited.
    ///
    /// Nothing beneath this protocol retransmits: the host's send ignores the
    /// reliable flag and calls sendto directly. A child answers duplicates from
    /// cache, so re-sending has no side effects.
    pub fn retry_exchange(&mut self) {
        if let Some(seq) = self.awaiting {
            let mut pkt = [0u8; PACKET_LEN];
            put(&mut pkt, TAG_CLOCK, seq, self.pending_out, 0);
            self.outbox.push_back(pkt);
        }
    }

    /// The word latched for the exchange in flight.
    ///
    /// Hardware shifts SIOMLT_SEND out over the transfer window, so a game that
    /// writes the register again mid-transfer does not change what this transfer
    /// carries. Reading `output` at completion instead would show the new word in
    /// the parent's own slot and disagree with what the child was told.
    pub fn pending_out(&self) -> u16 {
        self.pending_out
    }

    /// The exchange waiting on a reply, if any. The transport needs it to know
    /// which sequence number to block on.
    pub fn awaiting(&self) -> Option<u8> {
        self.awaiting
    }

    /// Take the peer's reply for `seq`, if it has arrived.
    pub fn take_reply(&mut self, seq: u8) -> Option<u16> {
        if self.awaiting == Some(seq) {
            if let Some(word) = self.reply.take() {
                self.awaiting = None;
                return Some(word);
            }
        }
        None
    }

    /// Give up on `seq` and tell the peer so.
    pub fn abandon(&mut self, seq: u8) {
        if self.awaiting == Some(seq) {
            self.awaiting = None;
            self.reply = None;
            let mut pkt = [0u8; PACKET_LEN];
            put(&mut pkt, TAG_GAVE_UP, seq, 0, 0);
            self.outbox.push_back(pkt);
        }
    }

    /// Has a peer on a different cable build been heard from? The cable refuses
    /// to carry anything once one has.
    pub fn refusing(&self) -> bool {
        self.version_mismatch > 0
    }

    /// Drop transfers in flight without forgetting the sequence numbers.
    ///
    /// Narrower than [`CableProto::reset`] on purpose: clearing `seen` would make
    /// a retransmitted clock look new and queue the same transfer twice.
    pub fn flush_pending(&mut self) {
        self.child_input.clear();
        self.awaiting = None;
        self.reply = None;
    }

    /// Take the oldest transfer a parent clocked into us.
    pub fn child_clock(&mut self) -> Option<(u16, u16)> {
        self.child_input.pop_front()
    }

    /// Packets the transport should put on the wire.
    pub fn take_outbox(&mut self) -> Vec<[u8; PACKET_LEN]> {
        self.outbox.drain(..).collect()
    }

    /// Consume one inbound packet.
    ///
    /// A reply is answered here, in the transport, rather than when the game
    /// next looks at the port: hardware presents a child's SIOMLT_SEND
    /// continuously, so making the peer wait for our emulation to come round
    /// would stall it for a frame.
    pub fn on_packet(&mut self, pkt: &[u8]) {
        if pkt.len() < PACKET_LEN || pkt[..3] != FAMILY {
            return;
        }
        if pkt[3] != WIRE_VERSION {
            // A peer on a different cable build. Count it and carry nothing:
            // the fields past here may not mean what this build thinks, and a
            // trade that corrupts is worse than a trade that will not start.
            self.version_mismatch += 1;
            return;
        }
        if self.version_mismatch > 0 {
            return; // latched: this pair is not going to link
        }
        let (tag, seq) = (pkt[4], pkt[5]);
        let word = u16::from_be_bytes([pkt[6], pkt[7]]);
        match tag {
            TAG_CLOCK => {
                if self.seen.contains(&Some(seq)) {
                    // A duplicate clock gets the identical reply back. Dropping
                    // it would leave the peer waiting out its whole timeout, and
                    // recomputing it would answer a stale clock with a fresh
                    // word, which is the corruption the pairing exists to stop.
                    if self.last_reply[4] == TAG_REPLY && self.last_reply[5] == seq {
                        let pkt = self.last_reply;
                        self.outbox.push_back(pkt);
                    }
                    return;
                }
                self.seen[self.seen_pos] = Some(seq);
                self.seen_pos = (self.seen_pos + 1) % SEEN_WINDOW;
                let answered = self.own_output;
                if self.child_input.len() >= CHILD_QUEUE_MAX {
                    self.child_input.pop_front();
                    self.dropped += 1;
                }
                self.child_input.push_back((word, answered));
                let mut reply = [0u8; PACKET_LEN];
                put(&mut reply, TAG_REPLY, seq, answered, self.lag());
                self.last_reply = reply;
                self.outbox.push_back(reply);
            }
            TAG_REPLY => {
                self.peer_lag = pkt[8];
                if self.awaiting == Some(seq) {
                    self.reply = Some(word);
                }
                // A reply for any other seq is stale by definition: the parent
                // has already moved on or given up. Drop it.
            }
            TAG_GAVE_UP => {
                self.peer_gave_up = true;
            }
            _ => {}
        }
    }
}

/// Two cable ends wired together in one process.
///
/// For desk tests and for any front-end that runs both cores itself. Shared
/// memory rather than a queue, so a parent reading the child's presented word
/// is reading a value the child really presents right now: there is nothing in
/// flight and so nothing to pair, which is exactly what makes this a weaker
/// test than the networked path and why [`CableProto`] has its own.
pub fn local_pair() -> (LocalCable, LocalCable) {
    let wire = std::rc::Rc::new(std::cell::RefCell::new(LocalWire::default()));
    (
        LocalCable { wire: wire.clone(), side: 0 },
        LocalCable { wire, side: 1 },
    )
}

#[derive(Default)]
struct LocalWire {
    /// Each side's presented word.
    out: [u16; 2],
    /// Transfers delivered to the child and not yet collected.
    to_child: VecDeque<(u16, u16)>,
    /// The parent's word for the exchange in flight.
    pending: Option<u16>,
}

pub struct LocalCable {
    wire: std::rc::Rc<std::cell::RefCell<LocalWire>>,
    side: usize,
}

impl LinkCable for LocalCable {
    fn units(&self) -> u8 {
        2
    }

    fn id(&self) -> u8 {
        self.side as u8
    }

    fn set_output(&mut self, word: u16) {
        self.wire.borrow_mut().out[self.side] = word;
    }

    fn parent_start(&mut self, own: u16) {
        let mut w = self.wire.borrow_mut();
        w.out[self.side] = own;
        w.pending = Some(own);
    }

    fn parent_result(&mut self) -> Option<[u16; MAX_UNITS]> {
        let mut w = self.wire.borrow_mut();
        let own = w.pending.take()?;
        let peer = w.out[1 - self.side];
        w.to_child.push_back((own, peer));
        Some([own, peer, ABSENT, ABSENT])
    }

    fn child_clock(&mut self) -> Option<(u16, u16)> {
        self.wire.borrow_mut().to_child.pop_front()
    }

    fn flush(&mut self) {
        let mut w = self.wire.borrow_mut();
        w.to_child.clear();
        w.pending = None;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A model of the networked transport, deliberately meaner than the real
    /// one: nothing is delivered until a test says so, so a protocol that only
    /// works when packets happen to arrive at the convenient moment fails here.
    struct Wire {
        proto: [CableProto; 2],
    }

    impl Wire {
        fn new() -> Wire {
            Wire { proto: [CableProto::new(), CableProto::new()] }
        }
        /// Move everything side `from` has queued into the other side.
        fn deliver(&mut self, from: usize) {
            for pkt in self.proto[from].take_outbox() {
                self.proto[1 - from].on_packet(&pkt);
            }
        }
        /// Throw away everything side `from` queued, as a lost datagram.
        fn drop_all(&mut self, from: usize) {
            self.proto[from].take_outbox();
        }
    }

    #[test]
    fn a_transfer_carries_each_units_word_to_the_other() {
        let mut w = Wire::new();
        w.proto[1].set_output(0xB9A0);
        let seq = w.proto[0].begin_exchange(0x8FFF);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(seq),
            Some(0xB9A0),
            "the parent must read the child's presented word"
        );
        assert_eq!(
            w.proto[1].child_clock(),
            Some((0x8FFF, 0xB9A0)),
            "the child must see the parent's word and the word it answered with"
        );
    }

    #[test]
    fn a_child_that_is_keeping_up_reports_the_healthy_backlog() {
        // The parent reads this to tell a child that is in step from one that is
        // falling behind, so the value a healthy child sends has to be the one
        // LAG_HEALTHY names. It was named as zero while the code sends one.
        let mut w = Wire::new();
        w.proto[1].set_output(0xB9A0);
        let seq = w.proto[0].begin_exchange(0x8FFF);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(seq), Some(0xB9A0));
        assert_eq!(
            w.proto[1].child_clock(),
            Some((0x8FFF, 0xB9A0)),
            "the child drained the transfer, so it is as caught up as it can be"
        );
        assert_eq!(
            w.proto[0].peer_lag(),
            LAG_HEALTHY,
            "a child in step must report the backlog LAG_HEALTHY calls healthy"
        );
    }

    #[test]
    fn the_word_a_transfer_carries_is_latched_when_it_starts() {
        // Hardware shifts SIOMLT_SEND out over the transfer window, so a game
        // that writes the register again before the transfer lands has not
        // changed what this transfer carries. The transport fills the parent's
        // own slot from this, and reading the live value instead would show the
        // parent a word the child was never told about.
        let mut p = CableProto::new();
        p.begin_exchange(0xAAAA);
        p.set_output(0xBBBB);
        assert_eq!(p.pending_out(), 0xAAAA, "the in-flight transfer keeps its word");
        assert_eq!(p.output(), 0xBBBB, "while the bus now presents the new one");
    }

    #[test]
    fn a_reply_for_an_exchange_the_parent_moved_past_is_not_mistaken_for_this_one() {
        let mut w = Wire::new();
        w.proto[1].set_output(0xAAAA);
        let a = w.proto[0].begin_exchange(0x1111);
        w.deliver(0);
        // The child's reply to A is still in flight; the parent gives up and
        // starts B.
        w.proto[0].abandon(a);
        w.proto[0].take_outbox();
        w.proto[1].set_output(0xBBBB);
        let b = w.proto[0].begin_exchange(0x2222);
        w.proto[0].take_outbox(); // B's clock never reaches the child
        // A's late reply lands now.
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(b),
            None,
            "A's reply must not answer B, or the parent reads a word from the wrong transfer"
        );
    }

    #[test]
    fn a_duplicate_clock_is_answered_from_cache_rather_than_re_read() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x1234);
        let seq = w.proto[0].begin_exchange(0x0001);
        let clock = w.proto[0].take_outbox();
        w.proto[1].on_packet(&clock[0]);
        // The child's game moves on and presents a new word before the lost
        // reply is retried.
        w.proto[1].set_output(0x5678);
        w.proto[1].take_outbox(); // the first reply was lost
        w.proto[1].on_packet(&clock[0]);
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(seq),
            Some(0x1234),
            "the retried reply must carry the word presented when the clock first landed"
        );
        assert_eq!(
            w.proto[1].child_clock(),
            Some((0x0001, 0x1234)),
            "a duplicate clock must not queue the transfer twice"
        );
        assert_eq!(w.proto[1].child_clock(), None);
    }

    #[test]
    fn a_lost_clock_is_recovered_by_retransmission() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x4242);
        let seq = w.proto[0].begin_exchange(0x9999);
        w.drop_all(0);
        w.proto[0].retry_exchange();
        w.deliver(0);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(seq), Some(0x4242));
    }

    #[test]
    fn a_reply_consumed_once_is_not_available_twice() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x0F0F);
        let seq = w.proto[0].begin_exchange(0x00FF);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(seq), Some(0x0F0F));
        assert_eq!(
            w.proto[0].take_reply(seq),
            None,
            "a second take would let one transfer complete twice"
        );
    }

    #[test]
    fn a_child_that_never_collects_reports_its_backlog_to_the_parent() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x3333);
        for _ in 0..3 {
            let seq = w.proto[0].begin_exchange(0x4444);
            w.deliver(0);
            w.deliver(1);
            w.proto[0].take_reply(seq);
        }
        assert_eq!(w.proto[1].lag(), 3, "three transfers accepted, none collected");
        assert_eq!(
            w.proto[0].peer_lag(),
            3,
            "the parent can only learn the child is behind if the child says so"
        );
        w.proto[1].child_clock();
        let seq = w.proto[0].begin_exchange(0x5555);
        w.deliver(0);
        w.deliver(1);
        w.proto[0].take_reply(seq);
        assert_eq!(w.proto[0].peer_lag(), 3, "one collected, one more accepted");
    }

    #[test]
    fn a_child_that_falls_far_behind_drops_the_oldest_rather_than_hoarding() {
        let mut w = Wire::new();
        for i in 0..(CHILD_QUEUE_MAX + 4) {
            let seq = w.proto[0].begin_exchange(i as u16);
            w.deliver(0);
            w.deliver(1);
            w.proto[0].take_reply(seq);
        }
        assert_eq!(w.proto[1].child_input.len(), CHILD_QUEUE_MAX);
        assert_eq!(w.proto[1].dropped, 4);
        assert_eq!(
            w.proto[1].child_clock().map(|(parent, _)| parent),
            Some(4),
            "the queue must hold the NEWEST transfers; a stale word is worse than none"
        );
    }

    #[test]
    fn giving_up_tells_the_peer_it_was_the_one_not_answering() {
        let mut w = Wire::new();
        let seq = w.proto[0].begin_exchange(0x1111);
        w.drop_all(0);
        w.proto[0].abandon(seq);
        w.deliver(0);
        assert!(
            w.proto[1].take_peer_gave_up(),
            "the child has no other way to learn it stalled the cable"
        );
        assert!(!w.proto[1].take_peer_gave_up(), "the flag is consumed");
    }

    #[test]
    fn a_reply_that_arrives_after_giving_up_is_inert() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x7777);
        let seq = w.proto[0].begin_exchange(0x8888);
        w.deliver(0);
        w.proto[0].abandon(seq);
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(seq),
            None,
            "an abandoned exchange must stay abandoned, or the register model completes twice"
        );
    }

    #[test]
    fn a_peer_on_a_different_cable_version_is_refused_rather_than_guessed_at() {
        // The frontend is handed a protocol string so two peers can refuse each
        // other when they disagree, and Trophy Hub's host throws it away: it
        // prints the string once and never compares it. So a mismatched pair
        // connects. Without this the fields past the header would be read as if
        // they meant what this build thinks, and the failure would surface as
        // corrupt gameplay rather than as a link that will not start.
        let mut p = CableProto::new();
        p.set_output(0xB9A0);
        let mut future = [0u8; PACKET_LEN];
        future[..3].copy_from_slice(&FAMILY);
        future[3] = b'9'; // a cable version this build does not speak
        future[4] = TAG_CLOCK;
        future[6..8].copy_from_slice(&0x8FFFu16.to_be_bytes());
        p.on_packet(&future);
        assert_eq!(p.version_mismatch, 1);
        assert!(p.refusing(), "the cable has to say it will not carry this");
        assert_eq!(p.child_clock(), None, "and carry nothing");
        assert!(p.take_outbox().is_empty(), "answering would be claiming to understand it");

        // It LATCHES: a peer that also sends well-formed packets does not get a
        // half-working cable out of it.
        let mut ok = [0u8; PACKET_LEN];
        put(&mut ok, TAG_CLOCK, 0, 0x8FFF, 0);
        p.on_packet(&ok);
        assert_eq!(
            p.child_clock(),
            None,
            "once the pair is known to disagree, nothing more is carried"
        );
    }

    #[test]
    fn a_packet_without_the_cable_magic_is_ignored() {
        let mut p = CableProto::new();
        // A wireless adapter packet on the same session: "RFU1" and 12 bytes.
        let rfu = [0x52, 0x46, 0x55, 0x31, 1, 0, 0, 0, 0, 0, 0, 0];
        p.on_packet(&rfu);
        assert_eq!(p.child_clock(), None);
        assert!(p.take_outbox().is_empty(), "an adapter packet must not draw a cable reply");
        p.on_packet(&[0x43, 0x42, 0x4C, 0x31, TAG_CLOCK]); // truncated
        assert_eq!(p.child_clock(), None);
    }

    #[test]
    fn a_reset_drops_everything_in_flight() {
        let mut w = Wire::new();
        w.proto[1].set_output(0x1234);
        let seq = w.proto[0].begin_exchange(0x4321);
        w.deliver(0);
        w.proto[1].reset();
        assert_eq!(w.proto[1].child_clock(), None);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(seq), None, "a reset child answers nothing");
    }

    #[test]
    fn the_local_pair_moves_a_word_in_both_directions() {
        let (mut parent, mut child) = local_pair();
        assert_eq!(parent.id(), 0);
        assert_eq!(child.id(), 1);
        child.set_output(0xB9A0);
        parent.parent_start(0x8FFF);
        assert_eq!(parent.parent_result(), Some([0x8FFF, 0xB9A0, ABSENT, ABSENT]));
        assert_eq!(child.child_clock(), Some((0x8FFF, 0xB9A0)));
    }

    #[test]
    fn the_transfer_table_is_the_one_mgba_measured() {
        // Asserted against the numbers, not against our own table, because a
        // test phrased in terms of the constant it checks agrees with any typo
        // in it. Two units at 115200 baud is what Pokemon trading uses.
        assert_eq!(TRANSFER_CYCLES[3][1], 5755);
        assert_eq!(TRANSFER_CYCLES[0][0], 31976);
        // A transfer has to be long enough for the game to observe the busy bit
        // and short enough that nine of them fit in a frame's 280896 cycles.
        assert!(TRANSFER_CYCLES[3][1] * 9 < 280_896);
    }
}
