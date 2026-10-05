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

    /// This core emulated `cycles` more cycles. Called once per scanline while
    /// the game is in Multi-Player mode.
    ///
    /// A parent uses it to decide when to put its position on the wire; a child
    /// spends it out of the budget the parent's position granted. Both ends
    /// measure only their own progress, which is the property the two reverted
    /// attempts lacked.
    fn advance(&mut self, cycles: u32) {
        let _ = cycles;
    }

    /// Must this core stop before emulating the next scanline?
    ///
    /// Only ever true on a child, and only while it is ahead of the parent's
    /// last reported position. The caller loops on this, pumping its transport
    /// between calls, so an implementation may block for a short interval before
    /// answering and MUST eventually answer false on its own: a parent that has
    /// gone away has to leave the child running, not frozen.
    fn hold(&mut self) -> bool {
        false
    }
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
///
/// Went to `3` for the pacing: every packet now carries the sender's position in
/// emulated cycles, and a parent sends [`TAG_TICK`] once a frame. Both halves
/// matter for refusal. The layout grew, so a v2 packet is 9 bytes where this
/// build needs 13, and the meaning changed, so a v3 child paired with a v2
/// parent would never hear a tick, would starve, and would silently give up
/// pacing. A session that degrades is harder to attribute than one that refuses.
///
/// Went to `4` because the ROLES are now elected on this wire instead of taken
/// from the frontend's peer numbering. Measured on two devices: each numbered
/// the other peer 1 and itself 0, so both played the parent, both games offered
/// the leader menu, and both clocked at each other. A v4 peer paired with a v3
/// one must refuse rather than have one end elect while the other trusts its
/// frontend. See [`TAG_HELLO`].
pub const WIRE_VERSION: u8 = b'4';

/// The full four-byte header of a packet this build will act on, "CBL4".
pub const MAGIC: u32 = 0x4342_4C34;

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

/// `[magic, 4, 0, 0, 0, time]`: "I am a parent and I have reached `time`."
///
/// The 60 Hz floor under the pacing. A child's budget is fed by the parent's
/// position, and a position carried only by transfers stops arriving the moment
/// the parent's game stops clocking, which is most of a menu. Without this the
/// pacing would freeze a child whenever the link went quiet, which is a worse
/// bug than the one it fixes. One 13-byte datagram per frame is noise next to a
/// transfer every 28672 cycles.
///
/// Same job as mGBA's `SIO_EV_HARD_SYNC`, which fires every 0x80000 cycles
/// whether or not a transfer is happening.
pub const TAG_TICK: u8 = 4;

/// `[magic, 5, 0, nonce_hi, 0, nonce_lo]`: "my identity is this, which end am I?"
///
/// Role election, on the wire, because the frontend's numbering cannot carry it.
/// Measured on the owner's two devices on 2026-10-05: the tablet saw the phone as
/// `cid=1` and the phone saw the tablet as `cid=1`, so both took themselves for
/// peer 0 and both played the parent. An earlier session of the same pair came
/// out 0 and 1, so the numbering is not stable either, and a role that is right
/// by luck is worse than one that is wrong, because it hides for a session.
///
/// The rule: the LOWER identity is the parent. Both ends compute it from the same
/// two numbers and cannot disagree. A collision is broken by perturbing our own
/// identity and saying hello again, rather than by either end assuming. Until an
/// identity has been heard from the peer the bus reports ONE unit, so the game
/// reads SD low, concludes it has no partner, and never offers to link: the
/// failure direction is a link that will not start rather than two leaders.
pub const TAG_HELLO: u8 = 5;

/// Every cable packet is this long: magic, tag, seq, word, lag, time.
pub const PACKET_LEN: usize = 13;

/// Byte offset of the sender's position, a big-endian u32 of emulated cycles
/// since this cable end was reset.
///
/// Session-relative and never compared to the peer's absolutely: the two cores
/// power on seconds apart and share no epoch, so a child integrates the
/// DIFFERENCES between successive parent positions. That needs no shared anchor
/// and its failure direction is safe, because a lost or late packet can only
/// slow a child down, never let it run ahead.
pub const TIME_OFF: usize = 9;

/// How far ahead of the parent's last reported position a child may emulate: one
/// frame.
///
/// Not a tuning knob picked for feel. It is the largest value that cannot hide a
/// whole protocol frame: Pokemon's connected mode clocks every 28672 cycles, so
/// one frame of slack is about nine transfers, and a child that is a frame ahead
/// has already run every transfer the parent has issued. Smaller would throttle
/// the handshake, which must stay at 60 fps.
pub const HORIZON: u32 = 280_896;

/// Consecutive [`LinkCable::hold`] answers before a child stops waiting and runs
/// free.
///
/// The fail-open bound. The transport waits about 250 us between calls, so this
/// is roughly 300 ms: comfortably past the parent's own 150 ms give-up timeout,
/// so a child does not start free-running while its parent is merely slow, and
/// short enough that a parent which has gone away releases the child in a third
/// of a second rather than freezing the screen. A frozen emulator is a worse
/// failure than a desynchronised one, because the player cannot even quit the
/// link from inside the game.
pub const HOLD_SPINS_MAX: u32 = 1200;

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

fn put(out: &mut [u8; PACKET_LEN], tag: u8, seq: u8, word: u16, lag: u8, time: u32) {
    out[..4].copy_from_slice(&MAGIC.to_be_bytes());
    out[4] = tag;
    out[5] = seq;
    out[6..8].copy_from_slice(&word.to_be_bytes());
    out[8] = lag;
    out[TIME_OFF..TIME_OFF + 4].copy_from_slice(&time.to_be_bytes());
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
    /// Clocks accepted while a previous transfer was still uncollected.
    ///
    /// The precondition of the one cable failure no other instrument can see.
    /// A clock is answered from `own_output`, which only the game refreshes, in
    /// the serial interrupt of the PREVIOUS transfer. So a clock arriving
    /// before that interrupt has been serviced is answered with the word armed
    /// for the transfer before it, the parent reads the same word twice, and
    /// the protocol frame's checksum fails while every other counter here says
    /// the link is perfect. Counted rather than refused: a game is free to
    /// present one word across several transfers, and gating on a dirty flag
    /// would hang it.
    pub stale_risk: u64,
    /// Clocks answered without the game having refreshed SIOMLT_SEND since the
    /// last one we answered.
    ///
    /// `stale_risk` only sees the case where the PREVIOUS transfer was still
    /// uncollected, and that is a proxy: a child can collect a transfer, be
    /// asked again before its interrupt handler has run, and answer with the
    /// previous word while its queue is empty. Same corruption, invisible to
    /// that counter. This is the precondition exactly: an answer given cold.
    ///
    /// Still a counter and not a gate, for the reason the design records: a game
    /// is free to present one word across several transfers, so refusing here
    /// would hang a conforming game.
    pub answered_cold: u64,
    /// Clocks answered at all, so the first answer of a session is not counted
    /// as cold when nothing has been armed yet.
    pub answered: u64,
    /// Has the game stored SIOMLT_SEND since the last clock we answered?
    armed_since_answer: bool,
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
    /// This end's own position: emulated cycles since the cable was reset.
    local_cycles: u32,
    /// Position at which a parent owes the next [`TAG_TICK`].
    next_tick_at: u32,
    /// The parent's position as of its most recent packet. `None` until one
    /// arrives, which is also what "pacing has not engaged" means on a child.
    parent_time: Option<u32>,
    /// Cycles this child may still emulate before it is a whole frame ahead of
    /// the parent's last reported position. Signed because a long scanline can
    /// overshoot, and the overshoot has to be paid back rather than forgiven.
    budget: i64,
    /// Is this end pacing itself against a parent at all? False on a parent, on
    /// a child before first contact, and on a child whose parent went silent.
    engaged: bool,
    /// Cycles emulated since the last parent packet, for the silent-parent case
    /// where the child is NOT held and so never spins.
    since_parent: u32,
    /// Consecutive holds, reset by any parent packet. Bounded by
    /// [`HOLD_SPINS_MAX`] so a vanished parent cannot freeze a child.
    hold_spins: u32,
    /// Scanlines this child has been held, and the times it gave up waiting.
    /// Both on the heartbeat: the first says the pacing is working, the second
    /// says it stopped trusting the parent.
    pub holds: u64,
    pub starved: u64,
    /// The peer's position as of its last reply, for diagnostics only. It paces
    /// nobody; it lets ONE device's log answer "how far apart were we", which
    /// until now needed both logs side by side.
    peer_time: Option<u32>,
    /// This end's identity for the role election, 48 bits. Supplied by the
    /// transport, which is the layer that has a clock and an address to derive
    /// one from.
    identity: u64,
    /// The peer's identity, once it has said hello. `None` is "the roles are not
    /// settled", which is also "this bus has one unit on it".
    peer_identity: Option<u64>,
    /// Identities that arrived equal to ours, so one of them had to move.
    pub election_ties: u64,

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
            stale_risk: 0,
            answered_cold: 0,
            answered: 0,
            armed_since_answer: false,
            version_mismatch: 0,
            peer_lag: 0, // nothing reported yet, see LAG_HEALTHY
            peer_gave_up: false,
            outbox: VecDeque::new(),
            local_cycles: 0,
            next_tick_at: HORIZON,
            parent_time: None,
            budget: 0,
            engaged: false,
            since_parent: 0,
            hold_spins: 0,
            holds: 0,
            starved: 0,
            peer_time: None,
            identity: 0,
            peer_identity: None,
            election_ties: 0,
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
        self.stale_risk = 0;
        self.answered_cold = 0;
        self.answered = 0;
        self.armed_since_answer = false;
        self.version_mismatch = 0;
        self.peer_lag = 0; // nothing reported yet, see LAG_HEALTHY
        self.peer_gave_up = false;
        self.outbox.clear();
        self.local_cycles = 0;
        self.next_tick_at = HORIZON;
        self.parent_time = None;
        self.budget = 0;
        self.engaged = false;
        self.since_parent = 0;
        self.hold_spins = 0;
        self.holds = 0;
        self.starved = 0;
        self.peer_time = None;
        // The identity survives a reset; the peer's does not. A session edge
        // means the pairing has to be made again, and re-deriving our own number
        // would only add a chance of colliding with the one we already sent.
        self.peer_identity = None;
        self.election_ties = 0;
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
        self.armed_since_answer = true;
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
        put(&mut pkt, TAG_CLOCK, seq, out, 0, self.local_cycles);
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
            put(&mut pkt, TAG_CLOCK, seq, self.pending_out, 0, self.local_cycles);
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
            put(&mut pkt, TAG_GAVE_UP, seq, 0, 0, self.local_cycles);
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
        // The game just selected Multi-Player mode, so the budget earned
        // against whatever it was doing before means nothing. Disengaging rather
        // than zeroing leaves the child free-running until the parent's next
        // packet, which is the safe direction: a child that is briefly too fast
        // catches a clock late, where a child wrongly held sees no clocks at all.
        self.parent_time = None;
        self.budget = 0;
        self.engaged = false;
        self.since_parent = 0;
        self.hold_spins = 0;
    }

    /// Take the oldest transfer a parent clocked into us.
    pub fn child_clock(&mut self) -> Option<(u16, u16)> {
        self.child_input.pop_front()
    }

    /// Set this end's identity and announce it.
    ///
    /// Called by the transport at session start. 48 bits of it are used, which is
    /// what fits beside the tag in a packet, and the odds of two devices drawing
    /// the same number are negligible; a collision is handled anyway, because
    /// "negligible" is how the last three evenings started.
    pub fn set_identity(&mut self, nonce: u64) {
        self.identity = nonce & 0xFFFF_FFFF_FFFF;
        self.peer_identity = None;
        self.say_hello();
    }

    /// Queue an identity announcement.
    pub fn say_hello(&mut self) {
        let mut pkt = [0u8; PACKET_LEN];
        let hi = (self.identity >> 32) as u16;
        let lo = self.identity as u32;
        put(&mut pkt, TAG_HELLO, 0, hi, 0, lo);
        self.outbox.push_back(pkt);
    }

    /// Say hello again if the roles are still unsettled. Returns whether it did.
    ///
    /// **Driven by the FRONT-END's frames, not by emulated cycles, and that is not
    /// a detail.** The emulated-cycle version only ran while the game had selected
    /// Multi-Player mode, and the game will not select it until it sees a partner,
    /// which it cannot do until the election completes. The election has to work
    /// with no emulated time passing at all, which
    /// `the_roles_settle_without_the_game_ever_entering_multi_player_mode` pins
    /// down.
    ///
    /// It repeats because the first announcement can go out while the peer's
    /// session is still coming up, and a lost one must not leave the pair silent.
    pub fn hello_tick(&mut self) -> bool {
        if self.peer_identity.is_some() || self.identity == 0 {
            return false;
        }
        self.say_hello();
        true
    }

    /// Which end this unit is, or `None` while the roles are unsettled.
    ///
    /// The lower identity is the parent. Both ends compute this from the same two
    /// numbers, so they cannot disagree, which is the whole point: the previous
    /// version asked the frontend and the two devices got different answers.
    pub fn role(&self) -> Option<u8> {
        let peer = self.peer_identity?;
        if self.identity == peer {
            return None; // a tie is not a role, it is a re-roll
        }
        Some(u8::from(self.identity > peer))
    }

    /// Are the roles settled? While this is false the bus must report one unit.
    pub fn elected(&self) -> bool {
        self.role().is_some()
    }

    /// This end emulated `cycles` more cycles.
    ///
    /// Both roles call it. A parent uses it only to know when it owes a tick; a
    /// child spends it, and a child that is NOT being held still ages out a
    /// silent parent here, which is the case `hold` cannot see because a free
    /// child never asks.
    pub fn advance(&mut self, cycles: u32) {
        self.local_cycles = self.local_cycles.wrapping_add(cycles);
        if !self.engaged {
            return;
        }
        self.budget -= cycles as i64;
        self.since_parent = self.since_parent.saturating_add(cycles);
        if self.since_parent >= HORIZON * 2 {
            // Two frames with nothing from the parent. It is not slow, it is
            // gone, or its game left Multi-Player mode. Stop pacing against a
            // position that is no longer being updated.
            self.disengage();
        }
    }

    /// This end's own position, in cycles since the cable was reset.
    pub fn local_cycles(&self) -> u32 {
        self.local_cycles
    }

    /// Parent only: put this position on the wire if a frame has passed since the
    /// last one. Returns whether a tick was queued.
    ///
    /// Called once per scanline from the same place as [`CableProto::advance`],
    /// so the tick lands within a scanline of the frame boundary.
    pub fn maybe_tick(&mut self) -> bool {
        // Wrapping-safe: the difference read as a signed value is negative until
        // the position actually reaches the deadline, whatever side of 2^32 both
        // happen to sit on.
        if (self.local_cycles.wrapping_sub(self.next_tick_at) as i32) < 0 {
            return false;
        }
        self.next_tick_at = self.local_cycles.wrapping_add(HORIZON);
        let mut pkt = [0u8; PACKET_LEN];
        put(&mut pkt, TAG_TICK, 0, 0, 0, self.local_cycles);
        self.outbox.push_back(pkt);
        true
    }

    /// Must this child stop before emulating another scanline?
    ///
    /// The park predicate is `budget exhausted AND nothing uncollected`, and the
    /// second half is what keeps the stale-word hazard shut: a child is only ever
    /// parked with its queue empty, which means it has serviced the last transfer
    /// and its game has armed the next word. See
    /// `a_parked_child_answers_two_clocks_with_the_word_armed_for_the_first` for
    /// what the other case costs.
    ///
    /// Counts its own consecutive calls so a parent that has gone away releases
    /// the child instead of freezing it.
    pub fn hold(&mut self) -> bool {
        // An UNARMED child must keep running, even past its horizon.
        //
        // This is the condition the design claimed was implied and the devices
        // proved was not. "Nothing uncollected" does not mean "armed": a child
        // collects a transfer, parks before its interrupt handler has stored the
        // next word, and answers the following clock with the previous one. Both
        // ends then agree on a wrong word, which is why cable_wordsum matched
        // while the game still refused the trade, and cable_cold counted 297 of
        // them in 5893 transfers.
        //
        // Running on costs at most one handler's worth of overshoot against a
        // one-frame horizon, and it makes "parked implies armed" true by
        // construction instead of by appeal to what the game ought to do.
        let unarmed = self.answered > 0 && !self.armed_since_answer;
        if !self.engaged || self.budget > 0 || !self.child_input.is_empty() || unarmed {
            self.hold_spins = 0;
            return false;
        }
        self.hold_spins += 1;
        if self.hold_spins > HOLD_SPINS_MAX {
            self.disengage();
            self.starved += 1;
            return false;
        }
        self.holds += 1;
        true
    }

    /// Stop pacing and run free. Every path here is a fail-open: a child that
    /// cannot tell where its parent is must keep emulating.
    fn disengage(&mut self) {
        self.engaged = false;
        self.budget = 0;
        self.parent_time = None;
        self.since_parent = 0;
        self.hold_spins = 0;
    }

    /// Is this end pacing itself against a parent?
    pub fn pacing(&self) -> bool {
        self.engaged
    }

    /// Cycles this child may still emulate before it must wait.
    pub fn budget(&self) -> i64 {
        self.budget
    }

    /// How far this end is ahead of the peer's last reported position, in cycles.
    /// Diagnostics only, and `None` until the peer has sent one.
    pub fn skew(&self) -> Option<i32> {
        self.peer_time
            .map(|t| self.local_cycles.wrapping_sub(t) as i32)
    }

    /// Credit a child's budget with the parent's progress.
    ///
    /// Differences, not absolutes: the two cores share no epoch. A duplicate or
    /// reordered packet reads as a non-positive difference and credits nothing,
    /// which is why the retransmit path replaying a cached reply verbatim is
    /// correct rather than merely harmless: a retransmit is not new progress.
    fn credit(&mut self, parent_time: u32) {
        let delta = match self.parent_time {
            // First contact: one frame of slack, so a child that connects
            // mid-frame is not held before it has heard anything.
            None => HORIZON as i64,
            Some(prev) => {
                let d = parent_time.wrapping_sub(prev) as i32;
                if d <= 0 {
                    0
                } else {
                    (d as i64).min(HORIZON as i64)
                }
            }
        };
        self.parent_time = Some(parent_time);
        self.engaged = true;
        self.since_parent = 0;
        self.hold_spins = 0;
        self.budget = (self.budget + delta).min(HORIZON as i64);
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
        // Family and version BEFORE length, and that order is the whole point of
        // splitting them. A version-2 packet is 9 bytes where this build needs
        // 13, so a length check first would discard it as malformed and the
        // mismatch would never be counted: two devices on different builds would
        // see NO PEER instead of a refusal, which is the silent failure the split
        // exists to prevent. Caught by
        // `a_peer_on_the_previous_wire_version_never_paces_a_child`, which failed
        // against exactly that ordering.
        if pkt.len() < 4 || pkt[..3] != FAMILY {
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
        if pkt.len() < PACKET_LEN {
            return; // our own version, but truncated: nothing safe to read
        }
        let (tag, seq) = (pkt[4], pkt[5]);
        let word = u16::from_be_bytes([pkt[6], pkt[7]]);
        let time = u32::from_be_bytes([
            pkt[TIME_OFF],
            pkt[TIME_OFF + 1],
            pkt[TIME_OFF + 2],
            pkt[TIME_OFF + 3],
        ]);
        // Anything a PARENT sends carries its position, and every one of them
        // feeds the budget, including a retransmitted clock: the retransmit is a
        // duplicate transfer but it is not duplicate progress, it was stamped
        // when it was sent.
        if tag == TAG_CLOCK || tag == TAG_TICK {
            self.credit(time);
        }
        match tag {
            TAG_HELLO => {
                let peer = ((word as u64) << 32) | time as u64;
                if peer == self.identity {
                    // Both ends drew the same number. Move ours rather than let
                    // either end decide it is the parent: a tie broken by
                    // assumption is two parents, which is the failure this whole
                    // tag exists to end.
                    self.election_ties += 1;
                    self.identity = self
                        .identity
                        .wrapping_mul(0x9E37_79B9_7F4A_7C15)
                        .wrapping_add(1)
                        & 0xFFFF_FFFF_FFFF;
                    self.peer_identity = None;
                    self.say_hello();
                } else {
                    let first = self.peer_identity != Some(peer);
                    self.peer_identity = Some(peer);
                    // Answer so the peer learns ours too. Only on a new identity,
                    // so two ends cannot trade hellos forever.
                    if first {
                        self.say_hello();
                    }
                }
            }
            TAG_TICK => {
                // Nothing else to do. The position was the whole message, and it
                // is what keeps a child paced through menus, naming screens and
                // every other stretch where the parent's game is not clocking.
            }
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
                if self.answered > 0 && !self.armed_since_answer {
                    // The game has not stored a new word since the last clock we
                    // answered, so this transfer carries the previous one.
                    self.answered_cold += 1;
                }
                self.answered += 1;
                self.armed_since_answer = false;
                if !self.child_input.is_empty() {
                    // The previous transfer has not reached the game's handler,
                    // so `answered` is the word that handler would have
                    // replaced. See the field's own doc.
                    self.stale_risk += 1;
                }
                if self.child_input.len() >= CHILD_QUEUE_MAX {
                    self.child_input.pop_front();
                    self.dropped += 1;
                }
                self.child_input.push_back((word, answered));
                let mut reply = [0u8; PACKET_LEN];
                put(&mut reply, TAG_REPLY, seq, answered, self.lag(), self.local_cycles);
                self.last_reply = reply;
                self.outbox.push_back(reply);
            }
            TAG_REPLY => {
                self.peer_lag = pkt[8];
                self.peer_time = Some(time);
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

    /// The one cable failure no instrument could see: a child whose emulation is
    /// not running answers two clocks with one word.
    ///
    /// This is the hazard the lockstep design exists to prevent, demonstrated
    /// against today's code rather than described. `own_output` is refreshed
    /// only when the child's GAME stores SIOMLT_SEND, which it does in the
    /// serial interrupt of the previous transfer. A child that has emulated
    /// nothing since the last clock therefore still holds the previous word,
    /// and hands it back. The parent reads it as fresh data.
    ///
    /// What makes it the worst available shape: every other counter stays
    /// perfect. Both ends complete two transfers, nothing is lost, nothing is
    /// retransmitted, no version is refused. Only the game notices, as a
    /// checksum failure it reports as a bad cable. `stale_risk` is the counter
    /// that makes it visible; it is not a fix, and the pacing is what will
    /// prevent the condition.
    ///
    /// This test stays true after the pacing lands, which is why it asserts the
    /// stale answer rather than a corrected one: the fix removes the PARKED
    /// CHILD, it does not change what this layer answers when asked while
    /// parked. A protocol that could answer correctly here would need to know
    /// what the game has not yet told it.
    /// A parent crawling at the round trip, a child that would run at 60, and the
    /// child ends up at the parent's pace with nothing stale.
    ///
    /// This is the whole design in one test. The parent emulates only while it is
    /// not waiting on a reply, which is what `NetCable::parent_result` really does
    /// over a socket: nine blocking round trips a frame is where the 14 fps comes
    /// from. The child would happily run a scanline per tick. What has to come out
    /// is that the child tracks the parent instead of running nine protocol frames
    /// into the future, and that every clock is answered with a word the child's
    /// game armed for it.
    ///
    /// **It is a model of the transport, not the transport.** Time is ticks, the
    /// delay is fixed, and nothing here is a socket or a thread. What it does
    /// cover is the arithmetic the two reverted attempts got wrong: who is paced,
    /// by what signal, and whether the stale-word precondition ever occurs.
    #[test]
    fn a_child_tracks_a_parent_that_is_crawling_at_the_round_trip() {
        // A tick is the WALL time it takes to emulate one scanline: 16.7 ms over
        // 228 lines is about 73 us at 60 fps. The measured round trip between the
        // owner's two devices is 8.1 ms, which is therefore about 110 scanlines
        // of wall time, 55 each way. Getting this wrong is what makes a model
        // reassuring and useless: at a few ticks each way the parent is barely
        // slower than the child and the pacing never engages, which is a
        // statement about the model rather than about the cable.
        const ONE_WAY_TICKS: u32 = 55;
        const CLOCK_GAP: u32 = 28672; // the protocol's own spacing

        struct Delayed {
            proto: [CableProto; 2],
            /// (arrive_at_tick, to_side, packet)
            flight: Vec<(u32, usize, [u8; PACKET_LEN])>,
            tick: u32,
        }
        impl Delayed {
            fn send_all(&mut self, from: usize) {
                let at = self.tick + ONE_WAY_TICKS;
                for pkt in self.proto[from].take_outbox() {
                    self.flight.push((at, 1 - from, pkt));
                }
            }
            fn deliver_due(&mut self) {
                let now = self.tick;
                let mut still = Vec::new();
                for (at, to, pkt) in std::mem::take(&mut self.flight) {
                    if at <= now {
                        self.proto[to].on_packet(&pkt);
                    } else {
                        still.push((at, to, pkt));
                    }
                }
                self.flight = still;
            }
        }

        let mut w = Delayed { proto: [CableProto::new(), CableProto::new()], flight: Vec::new(), tick: 0 };

        // The games' words. The child's handler arms the first before anything.
        let parent_words: [u16; 9] =
            [0xA55A, 0x8FFF, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E];
        let child_words: [u16; 9] =
            [0x5AA5, 0xB9A0, 0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E];
        w.proto[1].set_output(child_words[0]);

        let (mut issued, mut collected) = (0usize, 0usize);
        let mut awaiting: Option<u8> = None;
        let mut parent_since_clock = 0u32;
        let (mut parent_lines, mut child_lines) = (0u32, 0u32);
        // The two counters share no epoch, which is the same reason the wire
        // carries differences rather than positions: the child runs freely until
        // the parent's first packet reaches it, so only the skew ACCUMULATED
        // after pacing engaged is bounded by the horizon.
        let mut skew_at_engagement: Option<i32> = None;
        let mut read_back: Vec<(u16, u16)> = Vec::new();

        for tick in 0..20_000u32 {
            w.tick = tick;
            w.deliver_due();

            // ---- the parent. It emulates only while it is not waiting. ----
            if let Some(seq) = awaiting {
                if let Some(word) = w.proto[0].take_reply(seq) {
                    read_back.push((parent_words[issued - 1], word));
                    awaiting = None;
                    parent_since_clock = 0;
                }
            } else {
                w.proto[0].advance(1232);
                parent_lines += 1;
                parent_since_clock += 1232;
                w.proto[0].maybe_tick();
                if issued < 9 && parent_since_clock >= CLOCK_GAP {
                    let seq = w.proto[0].begin_exchange(parent_words[issued]);
                    awaiting = Some(seq);
                    issued += 1;
                }
            }
            w.send_all(0);

            // ---- the child. It would run flat out; the cable decides. ----
            if skew_at_engagement.is_none() && w.proto[1].pacing() {
                skew_at_engagement =
                    Some(w.proto[1].local_cycles().wrapping_sub(w.proto[0].local_cycles()) as i32);
            }
            if !w.proto[1].hold() {
                w.proto[1].advance(1232);
                child_lines += 1;
                // Its serial engine collects one transfer per scanline, and its
                // handler arms the next word, exactly as the register model does.
                if let Some((_parent_word, _answered)) = w.proto[1].child_clock() {
                    collected += 1;
                    // A real handler always stores SIOMLT_SEND, even when it has
                    // nothing new to say, and the distinction matters now that an
                    // unarmed child refuses to park: a model that stopped arming
                    // after the last word would show the child running free past
                    // its horizon for reasons no game would produce.
                    w.proto[1].set_output(child_words[collected.min(8)]);
                }
            }
            w.send_all(1);

            // The trade is over once the ninth transfer is collected and its
            // reply is home. Running on past that would only measure two
            // unthrottled cores, since a parent that has stopped clocking feeds
            // the child a tick a frame and the child is free by design.
            if collected == 9 && awaiting.is_none() && read_back.len() == 9 {
                break;
            }
        }

        assert_eq!(issued, 9, "the parent got all nine transfers out");
        assert_eq!(collected, 9, "and the child collected every one of them");
        assert_eq!(
            read_back.len(),
            9,
            "the parent read a reply for each: none timed out in this model"
        );
        for (i, (sent, got)) in read_back.iter().enumerate() {
            assert_eq!(*sent, parent_words[i], "transfer {i} carried the wrong word out");
            assert_eq!(
                *got, child_words[i],
                "transfer {i}: the parent must read the word the child's handler armed for it"
            );
        }
        assert_eq!(
            w.proto[1].stale_risk, 0,
            "no clock was ever answered while a transfer sat uncollected"
        );
        assert_eq!(w.proto[1].starved, 0, "and the child never gave up on the parent");
        assert!(w.proto[1].holds > 0, "a child that never waited was not being paced");

        // The point of the whole design: the child did NOT run nine times further
        // than the parent. Within one frame, which is the horizon.
        let skew = w.proto[1].local_cycles().wrapping_sub(w.proto[0].local_cycles()) as i32;
        let base = skew_at_engagement.expect("the pacing never engaged");
        assert!(
            skew - base <= 280_896,
            "the child gained {} cycles on the parent after pacing engaged, past the              one-frame horizon",
            skew - base
        );
        // What the throttle actually did, in this model's own units: the child
        // spent more of the trade waiting than emulating. Unpaced it would have
        // run a scanline every tick, roughly six times the parent's progress,
        // which is the ratio the first device run measured.
        assert!(
            w.proto[1].holds > child_lines as u64,
            "the child emulated {child_lines} scanlines and waited {} times, so it was              hardly throttled (the parent managed {parent_lines})",
            w.proto[1].holds
        );
    }

    /// Two ends must come out of the election as opposite halves of one cable.
    ///
    /// This is the bug the second device session found, stated as a test. The
    /// frontend numbered each device 0 and its peer 1, so both played the parent,
    /// both games offered the leader menu, and both clocked at each other. The
    /// roles cannot come from a number only one device computes.
    #[test]
    fn two_ends_elect_opposite_halves_of_the_cable() {
        let mut w = Wire::new();
        assert!(!w.proto[0].elected(), "nothing is settled before hello");
        assert_eq!(w.proto[0].role(), None);

        w.proto[0].set_identity(0x0000_1111_2222);
        w.proto[1].set_identity(0x0000_3333_4444);
        w.deliver(0);
        w.deliver(1);

        assert_eq!(w.proto[0].role(), Some(0), "the lower identity is the parent");
        assert_eq!(w.proto[1].role(), Some(1), "and the higher one is the child");
        assert!(w.proto[0].elected() && w.proto[1].elected());
        assert_eq!(w.proto[0].election_ties, 0);

        // Both ends computed the same answer from the same two numbers, which is
        // the property that makes it impossible for them to disagree.
        assert_ne!(
            w.proto[0].role(),
            w.proto[1].role(),
            "two parents is the failure this exists to prevent"
        );
    }

    /// The election must complete with no emulated time passing at all.
    ///
    /// The chicken and egg that broke the first build of this: the bus reports one
    /// unit until the roles are settled, so the game reads SD low and never
    /// selects Multi-Player mode, and the cable is only charged with emulated
    /// cycles WHILE that mode is selected. An election driven by emulated time can
    /// therefore never start. This drives it the way the front-end does, one tick
    /// per displayed frame, and calls `advance` zero times.
    #[test]
    fn the_roles_settle_without_the_game_ever_entering_multi_player_mode() {
        let mut w = Wire::new();
        w.proto[0].set_identity(0x0000_0000_0100);
        w.proto[1].set_identity(0x0000_0000_0200);
        // Not one cycle of emulated time, and the first announcements are lost.
        w.drop_all(0);
        w.drop_all(1);
        for frame in 0..3 {
            let (a, b) = (w.proto[0].hello_tick(), w.proto[1].hello_tick());
            if frame == 0 {
                assert!(a && b, "both lost their first hello, so both must ask again");
            }
            w.deliver(0);
            w.deliver(1);
        }
        assert!(
            !w.proto[0].hello_tick() && !w.proto[1].hello_tick(),
            "and once settled neither keeps announcing"
        );
        assert_eq!(w.proto[0].role(), Some(0));
        assert_eq!(w.proto[1].role(), Some(1));
    }

    /// The order the hellos arrive in must not change the outcome.
    #[test]
    fn the_election_does_not_depend_on_who_spoke_first() {
        for first in [0usize, 1] {
            let mut w = Wire::new();
            w.proto[0].set_identity(0x0000_AAAA_0001);
            w.proto[1].set_identity(0x0000_AAAA_0002);
            w.deliver(first);
            w.deliver(1 - first);
            w.deliver(first);
            assert_eq!(w.proto[0].role(), Some(0), "first={first}");
            assert_eq!(w.proto[1].role(), Some(1), "first={first}");
        }
    }

    /// An identity collision is broken, not assumed away.
    ///
    /// If both ends drew the same number, neither may decide it is the parent. One
    /// of them has to move, and a tie is explicitly NOT a role.
    #[test]
    fn an_identity_collision_is_broken_rather_than_assumed() {
        let mut w = Wire::new();
        w.proto[0].set_identity(0x0000_DEAD_BEEF);
        w.proto[1].set_identity(0x0000_DEAD_BEEF);

        // Both say hello, both see their own number coming back at them.
        for _ in 0..6 {
            w.deliver(0);
            w.deliver(1);
        }
        assert!(w.proto[0].election_ties > 0, "the collision has to be noticed");
        assert_eq!(w.proto[0].elected(), w.proto[1].elected());
        if w.proto[0].elected() {
            assert_ne!(
                w.proto[0].role(),
                w.proto[1].role(),
                "whatever the re-roll produced, the two ends must not agree on a role"
            );
        }
    }

    /// Hellos stop once the roles are settled, and repeat until they are.
    ///
    /// A hello every frame forever would be noise, and no hello after the first
    /// lost one would be a session that never starts: the peer's own session can
    /// still be coming up when the first announcement goes out.
    #[test]
    fn hello_repeats_until_answered_and_then_stops() {
        let mut a = CableProto::new();
        a.set_identity(0x0000_0000_0007);
        a.take_outbox(); // the announcement set_identity queued

        assert!(a.hello_tick(), "still unsettled, so the next frame asks again");
        assert_eq!(a.take_outbox().len(), 1, "one announcement per frame, not a flood");

        let mut peer = [0u8; PACKET_LEN];
        put(&mut peer, TAG_HELLO, 0, 0, 0, 9);
        a.on_packet(&peer);
        assert_eq!(a.role(), Some(0), "identity 7 against 9 makes us the parent");
        assert!(!a.hello_tick(), "settled, so it stops");
    }

    /// A peer that restarts its session with a new identity is re-elected.
    #[test]
    fn a_peer_that_comes_back_with_a_new_identity_is_elected_again() {
        let mut a = CableProto::new();
        a.set_identity(0x0000_0000_0050);
        let mut hello = |n: u32, a: &mut CableProto| {
            let mut pkt = [0u8; PACKET_LEN];
            put(&mut pkt, TAG_HELLO, 0, 0, 0, n);
            a.on_packet(&pkt);
        };
        hello(0x80, &mut a);
        assert_eq!(a.role(), Some(0), "0x50 against 0x80 makes us the parent");
        hello(0x10, &mut a);
        assert_eq!(
            a.role(),
            Some(1),
            "the peer restarted with a lower number, so the roles swap rather than stick"
        );
    }

    /// Build a parent packet the way the wire carries one.
    fn parent_pkt(tag: u8, seq: u8, word: u16, time: u32) -> [u8; PACKET_LEN] {
        let mut pkt = [0u8; PACKET_LEN];
        put(&mut pkt, tag, seq, word, 0, time);
        pkt
    }

    /// A child may run one frame ahead of where the parent says it is, and not a
    /// scanline more.
    ///
    /// The horizon is what transmits the parent's slowness. In connected mode the
    /// parent is wall-paced by its own blocking round trip, so its reported
    /// position advances at about 14 fps, and this is what makes the child match
    /// it instead of running nine protocol frames into the future.
    #[test]
    fn the_child_never_runs_more_than_one_frame_past_the_parent() {
        let mut c = CableProto::new();

        // Before any parent packet there is nothing to pace against, and a child
        // that has heard nothing must run rather than wait.
        c.advance(280_896);
        assert!(!c.hold(), "a child that has heard nothing from a parent runs free");
        assert!(!c.pacing());

        // First contact grants one frame of slack. The absolute position is
        // arbitrary on purpose: the two cores share no epoch, only differences.
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 1_000_000));
        assert!(c.pacing(), "a parent packet engages the pacing");
        assert_eq!(c.budget(), 280_896, "one frame of slack");

        // 227 scanlines of 1232 cycles is 279664, so there is still 1232 left.
        for _ in 0..227 {
            c.advance(1232);
            assert!(!c.hold(), "a child inside the horizon must never be held");
        }
        assert_eq!(c.budget(), 1232);

        c.advance(1232);
        assert_eq!(c.budget(), 0);
        assert!(c.hold(), "a child a full frame ahead has to wait");
        assert_eq!(c.holds, 1);

        // One frame of parent progress unlocks exactly one frame of ours.
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 1_000_000 + 280_896));
        assert_eq!(c.budget(), 280_896);
        assert!(!c.hold(), "and it is released by the parent moving, nothing else");
    }

    /// An unarmed child keeps running instead of parking, even past the horizon.
    ///
    /// Measured on two devices: 297 of 5893 clocks were answered before the
    /// child's game had stored the next word, so the parent read the previous one
    /// back. Both ends agreed on it, which is why the word hash matched while the
    /// game still refused the trade. Parking is only safe once the handler has
    /// run, so "parked implies armed" is enforced here rather than assumed.
    #[test]
    fn an_unarmed_child_keeps_running_rather_than_parking() {
        let mut c = CableProto::new();
        c.set_output(0x1111);

        // A clock arrives and is answered, which consumes the armed word.
        c.on_packet(&parent_pkt(TAG_CLOCK, 0, 0x8FFF, 1_000));
        assert_eq!(c.child_clock(), Some((0x8FFF, 0x1111)), "the engine collects it");
        c.advance(280_896); // and the whole frame of budget is spent

        assert!(
            !c.hold(),
            "the handler has not stored the next word yet, so it must keep running"
        );
        assert_eq!(c.budget(), 0, "it is out of budget, which is not the question here");

        c.set_output(0x2222); // the handler runs
        assert!(c.hold(), "armed and out of budget, now parking is safe");
        assert_eq!(c.answered_cold, 0, "and no answer was given cold");
    }

    /// The handshake must not be throttled at all.
    ///
    /// This is where the second reverted attempt died: it held a child during the
    /// phase that was already working, and the device went to 6.6 fps against a
    /// parent at 56.7. Both ends at 60 fps have to produce zero holds, or the
    /// pacing has broken the phase it was never meant to touch.
    #[test]
    fn a_level_handshake_never_holds_the_child() {
        let mut c = CableProto::new();
        let mut parent_at = 0u32;
        for frame in 0..60 {
            parent_at = parent_at.wrapping_add(280_896);
            c.on_packet(&parent_pkt(TAG_TICK, 0, 0, parent_at));
            for line in 0..228 {
                assert!(!c.hold(), "frame {frame} line {line} was held at 60 fps");
                c.advance(1232);
            }
        }
        assert_eq!(c.holds, 0, "a level link must never hold");
        assert_eq!(c.starved, 0, "and never give up on a parent that is talking");
        assert!(c.pacing(), "while staying engaged the whole time");
    }

    /// A parent that goes away releases the child instead of freezing it.
    ///
    /// A frozen emulator is worse than a desynchronised one: the player cannot
    /// even leave the link from inside the game. So every path out of pacing is a
    /// fail-open, and this pins the bound rather than trusting it.
    #[test]
    fn a_parent_that_stops_answering_releases_a_held_child() {
        let mut c = CableProto::new();
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 500));
        c.advance(280_896); // spend the frame of slack

        // 1200 answers of "wait", at the transport's 250 us poll, is about 300 ms:
        // past the parent's own 150 ms give-up timeout, so a merely slow parent
        // does not lose its child.
        for i in 0..1200 {
            assert!(c.hold(), "hold {i} should still be waiting");
        }
        assert!(!c.hold(), "the 1201st answer has to let the child run");
        assert_eq!(c.starved, 1);
        assert!(!c.pacing(), "and the pacing is off until the parent comes back");

        // It comes back: pacing re-engages with a fresh frame of slack and no
        // memory of the old position.
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 9_000_000));
        assert!(c.pacing());
        assert_eq!(c.budget(), 280_896);
        assert!(!c.hold());
    }

    /// The silent parent the hold loop cannot see.
    ///
    /// A child with budget to spare never asks whether it should wait, so the
    /// spin bound above never runs. If the parent's game leaves Multi-Player mode
    /// mid-session, the only thing that notices is emulated time passing with no
    /// packet behind it.
    #[test]
    fn a_silent_parent_is_aged_out_by_emulated_time_as_well() {
        let mut c = CableProto::new();
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 77));
        assert!(c.pacing());
        // Two frames of emulated time with nothing from the parent.
        for _ in 0..456 {
            c.advance(1232);
        }
        assert!(!c.pacing(), "561792 cycles of silence is a parent that has gone");
        assert!(!c.hold());
        assert_eq!(c.starved, 0, "aged out rather than given up on while waiting");
    }

    /// A peer on the old wire is refused, and refusal must not look like a
    /// parent that is simply slow.
    #[test]
    fn a_peer_on_the_previous_wire_version_never_paces_a_child() {
        let mut c = CableProto::new();
        // Exactly what a version-2 build sends: 9 bytes, no time field.
        let mut v2 = [0u8; 9];
        v2[..3].copy_from_slice(&FAMILY);
        v2[3] = b'2';
        v2[4] = TAG_CLOCK;
        c.on_packet(&v2);
        assert_eq!(c.version_mismatch, 1, "it is ours, and it is not this version");
        assert!(!c.pacing(), "a refused peer must not engage the pacing");
        assert!(!c.hold(), "and must never hold the child");
        assert_eq!(c.child_clock(), None, "nor deliver a transfer");
    }

    /// The 60 Hz floor: one tick a frame, whether or not the game is clocking.
    #[test]
    fn a_parent_ticks_once_a_frame_and_not_twice() {
        let mut p = CableProto::new();
        for line in 0..227 {
            p.advance(1232);
            assert!(!p.maybe_tick(), "line {line} is still inside the first frame");
        }
        p.advance(1232);
        assert!(p.maybe_tick(), "280896 cycles is a frame, so a tick is owed");
        assert!(!p.maybe_tick(), "and owed once, not every time it is asked");

        let out = p.take_outbox();
        assert_eq!(out.len(), 1, "exactly one packet on the wire for the frame");
        assert_eq!(out[0][4], TAG_TICK);
        assert_eq!(
            u32::from_be_bytes([
                out[0][TIME_OFF],
                out[0][TIME_OFF + 1],
                out[0][TIME_OFF + 2],
                out[0][TIME_OFF + 3],
            ]),
            280_896,
            "carrying the position it was sent at"
        );

        for _ in 0..228 {
            p.advance(1232);
        }
        assert!(p.maybe_tick(), "and again the frame after");
    }

    /// Selecting Multi-Player mode drops the pacing with everything else.
    ///
    /// The budget was earned against whatever the parent was doing before the
    /// game committed to this protocol, so it means nothing now. Disengaging
    /// rather than zeroing is the safe direction: a child that is briefly too
    /// fast collects a clock late, where a child wrongly held sees none at all.
    #[test]
    fn the_mode_select_edge_drops_the_pacing_too() {
        let mut c = CableProto::new();
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 12_345));
        c.advance(280_896);
        assert!(c.hold());

        c.flush_pending();
        assert!(!c.pacing(), "pacing is dropped with the transfers");
        assert!(!c.hold(), "so the child runs until the parent speaks again");
        c.on_packet(&parent_pkt(TAG_TICK, 0, 0, 12_345 + 1232));
        assert_eq!(
            c.budget(),
            280_896,
            "and re-engaging grants a fresh frame, not a stale difference"
        );
    }

    #[test]
    fn a_parked_child_answers_two_clocks_with_the_word_armed_for_the_first() {
        let mut w = Wire::new();

        // The child's handler armed one word. Its core then stops: no further
        // store to SIOMLT_SEND, and nothing collects what arrives.
        w.proto[1].set_output(0xB9A0);

        let first = w.proto[0].begin_exchange(0x8FFF);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(first), Some(0xB9A0));

        let second = w.proto[0].begin_exchange(0x1234);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(second),
            Some(0xB9A0),
            "the parked child answers the second clock with the first word,              which is the corruption this whole design is about"
        );

        assert_eq!(
            w.proto[1].stale_risk, 1,
            "and exactly one clock was answered while a transfer sat uncollected"
        );
        assert_eq!(w.proto[1].dropped, 0, "nothing was dropped: the queue is nowhere near full");
        assert_eq!(w.proto[1].version_mismatch, 0, "and this is not a version problem");
        assert_eq!(w.proto[1].lag(), 2, "both transfers are still waiting for the serial engine");
    }

    /// The hazard `stale_risk` cannot see: a cold answer with an empty queue.
    ///
    /// This is the case that makes the queue-depth proxy insufficient. The child
    /// collects transfer N, so nothing is uncollected, and the next clock arrives
    /// before its interrupt handler has stored the word for N+1. `stale_risk`
    /// stays at zero and the parent still reads the previous word.
    #[test]
    fn a_clock_answered_before_the_game_rearmed_is_counted_even_with_an_empty_queue() {
        let mut w = Wire::new();
        w.proto[1].set_output(0xB9A0);

        let first = w.proto[0].begin_exchange(0x8FFF);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(w.proto[0].take_reply(first), Some(0xB9A0));
        // The child's serial engine collects it, so the queue is empty, but its
        // game has not reached the handler that stores the next word.
        assert_eq!(w.proto[1].child_clock(), Some((0x8FFF, 0xB9A0)));
        assert_eq!(w.proto[1].lag(), 0);

        let second = w.proto[0].begin_exchange(0x1234);
        w.deliver(0);
        w.deliver(1);
        assert_eq!(
            w.proto[0].take_reply(second),
            Some(0xB9A0),
            "the same word again, which is the corruption"
        );
        assert_eq!(
            w.proto[1].stale_risk, 0,
            "the queue was empty, so the older proxy sees nothing at all"
        );
        assert_eq!(
            w.proto[1].answered_cold, 1,
            "and the precise counter sees exactly one cold answer"
        );
    }

    /// A game that rearms between clocks is never counted cold, including the
    /// very first clock of a session.
    #[test]
    fn a_game_that_rearms_between_clocks_is_never_counted_cold() {
        let mut w = Wire::new();
        for i in 0..9u16 {
            w.proto[1].set_output(0x1000 + i);
            let seq = w.proto[0].begin_exchange(0x8F00 + i);
            w.deliver(0);
            w.deliver(1);
            assert_eq!(w.proto[0].take_reply(seq), Some(0x1000 + i), "transfer {i}");
            assert_eq!(w.proto[1].child_clock(), Some((0x8F00 + i, 0x1000 + i)));
        }
        assert_eq!(w.proto[1].answered, 9);
        assert_eq!(
            w.proto[1].answered_cold, 0,
            "nine armed answers, none of them cold, and the first is not counted"
        );
    }

    /// A child that is keeping up must NOT be counted as at risk.
    ///
    /// Without this the counter above would read as a fault on every healthy
    /// link, which is worse than not having it: an instrument that cries wolf
    /// gets ignored at exactly the moment it is right.
    #[test]
    fn a_child_that_collects_each_transfer_is_never_counted_as_at_risk() {
        let mut w = Wire::new();
        for i in 0..9u16 {
            // The game's handler: arm the next word, then the engine collects
            // the transfer before the next clock arrives.
            w.proto[1].set_output(0x1000 + i);
            let seq = w.proto[0].begin_exchange(0x8F00 + i);
            w.deliver(0);
            w.deliver(1);
            assert_eq!(w.proto[0].take_reply(seq), Some(0x1000 + i), "transfer {i}");
            assert_eq!(w.proto[1].child_clock(), Some((0x8F00 + i, 0x1000 + i)));
        }
        assert_eq!(w.proto[1].stale_risk, 0, "nine transfers, each collected, no risk");
        assert_eq!(w.proto[1].lag(), 0, "and nothing left in the queue");
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
        put(&mut ok, TAG_CLOCK, 0, 0x8FFF, 0, 0);
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
        // A wireless adapter packet on the same session: "RFU1" magic. Padded
        // to a cable packet's length on purpose, so what rejects it is the
        // family check and not the length check.
        let rfu = [0x52, 0x46, 0x55, 0x31, 1, 0, 0, 0, 0, 0, 0, 0, 0];
        assert_eq!(rfu.len(), PACKET_LEN, "the test is only meaningful at full length");
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
