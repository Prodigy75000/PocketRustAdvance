//! The Game Boy Advance Wireless Adapter (AGB-015), known in the libretro
//! world as the RFU.
//!
//! This is a *device* on the serial port, not a cable. The game talks to it in
//! Normal 32-bit mode, one word per transfer, and the adapter answers with a
//! word of its own. That makes it far more tractable than a link cable: there
//! is a bounded command set, the framing is self-describing, and because the
//! real hardware is wireless the protocol is already tolerant of latency. A
//! cable, by contrast, is a synchronisation problem with sub-frame deadlines,
//! which is why no libretro core emulates one.
//!
//! Reference is gpSP's `rfu.c`, which is the only readable implementation of
//! this protocol anywhere.
//!
//! The session layer (peers, broadcast, connections) is not here yet. What is
//! here is the complete device as seen by a game with nobody else around,
//! which is what the 43 adapter titles do at boot before any session exists.

/// The adapter's reply when it has nothing to say. The game reads the top bit
/// as "device idle".
const IDLE: u32 = 0x8000_0000;

/// Every command the game sends is framed with this in the high half-word.
const CMD_HEADER: u16 = 0x9966;

/// The two magic words of the power-on handshake. The game sends the first
/// (low half-word only) to wake the adapter, then a sequence ending in the
/// second, after which the adapter accepts commands.
const HS_HELLO: u16 = 0x494E;
const HS_DONE: u32 = 0xB0BB_8001;

// Commands, named for what they do rather than for the few hardware bits we
// can actually see.
const CMD_INIT1: u8 = 0x10;
const CMD_INIT2: u8 = 0x3D;
const CMD_LINKPWR: u8 = 0x11;
const CMD_SYSVER: u8 = 0x12;
const CMD_SYSSTAT: u8 = 0x13;
const CMD_SLOTSTAT: u8 = 0x14;
const CMD_CFGSTAT: u8 = 0x15;
const CMD_BCST_DATA: u8 = 0x16;
const CMD_SYSCFG: u8 = 0x17;
const CMD_HOST_START: u8 = 0x19;
const CMD_HOST_ACCEPT: u8 = 0x1A;
const CMD_HOST_STOP: u8 = 0x1B;
const CMD_BCRD_START: u8 = 0x1C;
const CMD_BCRD_FETCH: u8 = 0x1D;
const CMD_BCRD_STOP: u8 = 0x1E;
const CMD_CONNECT: u8 = 0x1F;
const CMD_ISCONNECTED: u8 = 0x20;
const CMD_CONCOMPL: u8 = 0x21;
const CMD_SEND_DATA: u8 = 0x24;
const CMD_SEND_DATAW: u8 = 0x25;
const CMD_RECV_DATA: u8 = 0x26;
const CMD_WAIT: u8 = 0x27;
const CMD_DISCONNECT: u8 = 0x30;
const CMD_RTX_WAIT: u8 = 0x37;

/// Responses the adapter sends while it, not the game, drives the clock.
const RESP_TIMEOUT: u8 = 0x27;
const RESP_DATA: u8 = 0x28;
const RESP_DISC: u8 = 0x29;

/// Connection status words reported by ISCONNECTED and CONCOMPL.
const CONN_INPROGRESS: u32 = 0x0100_0000;
const CONN_FAILED: u32 = 0x0200_0000;

/// The firmware version the adapter reports. Games compare against it, so it
/// is a fixed observed value rather than something we get to pick.
const FIRMWARE_VERSION: u32 = 0x0083_0117;

/// Longest response the adapter ever sends: four peers of seven words each,
/// which is what a broadcast-read fetch returns when the air is full.
const MAX_RESP: usize = 4 * 7;

/// One adapter frame, in system cycles: a sixtieth of a second.
const FRAME_CYCLES: u32 = 16_777_216 / 60;

// --- The wire, between two adapters -----------------------------------------
//
// Deliberately byte-identical to gpSP's RFU packets: four-byte big-endian
// words, a magic header, a type, a header word, then payload. Keeping the
// format identical costs nothing now and leaves the door open to pairing a
// gpSP device with one running this core, which would give us a reference peer
// to debug against. The protocol VERSION string we hand the frontend is our
// own, so the frontend will not actually pair us with gpSP until we say it is
// safe: a peer that is not truly compatible must refuse a session rather than
// corrupt it silently.

/// "RFU1".
const NET_HEADER: u32 = 0x5246_5531;

const PKT_BROADCAST: u32 = 0x00;
const PKT_CONNECT_REQ: u32 = 0x01;
const PKT_CONNECT_ACK: u32 = 0x02;
const PKT_CONNECT_NACK: u32 = 0x03;
const PKT_DISCONNECT: u32 = 0x04;
const PKT_HOST_SEND: u32 = 0x05;
const PKT_CLIENT_SEND: u32 = 0x06;
const PKT_CLIENT_ACK: u32 = 0x07;

/// The frontend's "send to everyone" client id.
pub const BROADCAST_ID: u16 = 0xFFFF;

/// Peers we will remember broadcasting. gpSP's cap, and the table is indexed
/// by the frontend's client id so it has to be at least as large as a session.
const MAX_PEERS: usize = 32;

/// A host re-announces itself this often, in frames: about twice a second.
const ANNOUNCE_FRAMES: u8 = 30;

/// Frames of silence before a peer's broadcast is forgotten, and before a host
/// gives up on a client. Both are about four seconds.
const PEER_TTL: u8 = 255;
const CLIENT_TTL: u8 = 240;

/// How many packets each direction will hold before dropping. A drop is
/// visible rather than silent, see `dropped`.
const QUEUE_DEPTH: usize = 16;

/// One packet the frontend should put on the wire.
pub struct OutPacket {
    /// Frontend client id, or `BROADCAST_ID`.
    pub to: u16,
    pub bytes: Vec<u8>,
}

fn put32(out: &mut Vec<u8>, v: u32) {
    out.extend_from_slice(&v.to_be_bytes());
}

fn get32(buf: &[u8], off: usize) -> u32 {
    u32::from_be_bytes([buf[off], buf[off + 1], buf[off + 2], buf[off + 3]])
}

/// A host another adapter has announced, as last heard.
#[derive(Clone, Copy, Default)]
struct Peer {
    valid: bool,
    device_id: u16,
    ttl: u8,
    /// The six words of game-supplied advertisement: in Pokemon this carries
    /// the trainer name and what the room is for.
    data: [u32; 6],
}

/// A client attached to us while we are the host.
#[derive(Clone, Copy, Default)]
struct HostClient {
    /// Adapter-assigned id. Zero means the slot is free.
    devid: u16,
    /// The frontend's id for that peer, which is how we address it.
    client_id: u16,
    /// Frames since we last heard from it.
    ttl: u8,
}

#[derive(Default)]
struct HostState {
    devid: u16,
    clients: [HostClient; 4],
    /// What we advertise, set by BCST_DATA.
    bdata: [u32; 6],
    /// Frames since our last announcement.
    tx_ttl: u8,
    /// Data each client has sent us, oldest first.
    inbox: [std::collections::VecDeque<Vec<u8>>; 4],
}

#[derive(Default)]
struct ClientState {
    devid: u16,
    clnum: u8,
    /// Frontend id of the host we are attached to.
    host_id: u16,
    /// Frames since we last heard from it.
    host_ttl: u8,
    inbox: std::collections::VecDeque<Vec<u8>>,
}

/// Where the adapter is in a single command exchange.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum Com {
    /// Powered down, waiting for the handshake to start.
    #[default]
    Reset,
    /// Mid-handshake, echoing words back to the game.
    Handshake,
    /// Idle, waiting for a command header.
    WaitCmd,
    /// A command header arrived with a payload; collecting it.
    WaitDat,
    /// Acknowledging the command just processed.
    RespCmd,
    /// Streaming the response payload.
    RespDat,
    /// Reporting a rejected command.
    RespErr,
    /// The error code that follows it.
    RespErr2,
    /// The game handed the clock over and is waiting for the adapter to report
    /// an event. This is where the two roles reverse.
    WaitEvent,
    /// An event is ready; pushing it while the adapter holds the clock.
    WaitResp,
}

/// What the adapter is doing on the air.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
enum Link {
    #[default]
    Idle,
    Host,
    Connecting,
    Client,
}

/// Does this ROM expect a wireless adapter?
///
/// Keyed on the four-character game code at 0xAC, because unlike the RTC there
/// is no library string to find: the adapter is a separate accessory, so
/// nothing in the ROM header or its libraries records that a game supports one.
/// See `rtc::detect` for the opposite case, where a library string is exact.
///
/// The list is gpSP's, re-verified against our own corpus, where all 43 codes
/// resolve to a real ROM:
///
/// ```text
/// Pokemon Emerald / FireRed / LeafGreen   BPE* BPR* BPG*   (18 ROMs)
/// Mario Golf Advance Tour                 BMG*             (8)
/// Mario Tennis Power Tour                 BTM*             (3)
/// Mega Man Battle Network 5 and 6         BRBE BRKE BR5E BR6E
/// Shrek Super Slam                        B4UE B4UP
/// Hamtaro Ham-Ham Games                   B85A B85P
/// Digimon Racing                          BDGE BDGP
/// Dragon Ball Z Buu-s Fury                BG3E
/// Lord of the Rings The Third Age         B3AE
/// The Chronicles of Narnia                B2WE
/// No No No Puzzle Chailien                BKRJ
/// ```
pub fn detect(rom: &[u8]) -> bool {
    const CODES: [&[u8; 4]; 43] = [
        b"BPEE", b"BPEJ", b"BPED", b"BPEF", b"BPES", b"BPEI", // Emerald
        b"BPRJ", b"BPRE", b"BPRS", b"BPRD", b"BPRI", b"BPRF", // FireRed
        b"BPGE", b"BPGS", b"BPGD", b"BPGI", b"BPGF", b"BPGJ", // LeafGreen
        b"BMGE", b"BMGJ", b"BMGP", b"BMGS", b"BMGF", b"BMGI", b"BMGD", b"BMGU",
        b"BTME", b"BTMJ", b"BTMP", // Mario Tennis
        b"BRBE", b"BRKE", b"BR5E", b"BR6E", // Battle Network 5 and 6
        b"B4UE", b"B4UP", b"B85A", b"B85P", b"BDGE", b"BDGP", b"BG3E", b"B3AE",
        b"B2WE", b"BKRJ",
    ];
    match rom.get(0xAC..0xB0) {
        Some(code) => CODES.iter().any(|c| c.as_slice() == code),
        None => false,
    }
}

/// The adapter.
pub struct Rfu {
    com: Com,
    link: Link,
    /// Command being processed, or the error code when rejecting one.
    cmd: u8,
    /// Words still to move in the current payload or response.
    len: u8,
    /// Words moved so far.
    pos: u8,
    /// Payload in, response out. The adapter reuses one buffer for both, which
    /// is why a response is built over the request that produced it.
    buf: [u32; MAX_RESP],
    /// Previous word, which the handshake echoes back inverted.
    prev: u32,
    /// Slave timeout and retransmit count, in adapter frames, as configured by
    /// SYSCFG.
    timeout: u8,
    rtx_max: u8,
    /// Cycles left before the adapter gives up waiting for an event, and
    /// before it decides a retransmit round has elapsed.
    timeout_cycles: u32,
    resp_cycles: u32,
    /// Commands the game has issued. Zero separates "the adapter is wrong"
    /// from "the game never asked", which look identical otherwise.
    pub commands: u64,
    /// Times the game has power-cycled the adapter by pulling PD high. A game
    /// that cannot get through the handshake resets in a tight loop, so this
    /// climbing while `commands` stays put is the signature of a failed
    /// handshake rather than of a failed command.
    pub resets: u64,

    // --- session, everything below is the air rather than the game ---
    /// Hosts we have heard announce themselves, indexed by frontend client id.
    peers: [Peer; MAX_PEERS],
    host: HostState,
    client: ClientState,
    /// Packets waiting for the frontend to send. The core cannot reach the
    /// network itself, so it produces these and the libretro layer drains
    /// them. That keeps this module pure and testable against a model wire.
    outbox: Vec<OutPacket>,
    /// Packets dropped because a queue was full. A silent drop in a trade
    /// shifts every later byte, so it is counted rather than ignored.
    pub dropped: u64,
    /// Peers seen and sessions formed, for the runner summary.
    pub peers_seen: u64,
    pub connections: u64,
    /// Bitmask of every command code the game has issued, bit `cmd & 0x3F`.
    /// One number that says exactly which of the protocol the game uses, which
    /// is far more useful on a device than a count: the adapter has no way to
    /// log and the games differ in which commands they drive.
    pub cmd_seen: u64,
    /// The same, for commands we do not implement. Non-zero here means the
    /// game is asking for something this adapter has never heard of.
    pub unknown_seen: u64,
    /// Our own id in the netplay session, as the frontend assigned it.
    ///
    /// The ONLY thing that distinguishes two otherwise identical emulators:
    /// same ROM, same save, same menus, same instruction stream. A real
    /// adapter has a unique id burned in and can derive one from a clock; a
    /// deterministic core can do neither without giving up reproducibility.
    self_id: u16,
}

impl Default for Rfu {
    fn default() -> Self {
        Self {
            com: Com::Reset,
            link: Link::Idle,
            cmd: 0,
            len: 0,
            pos: 0,
            buf: [0; MAX_RESP],
            prev: 0,
            timeout: 0,
            rtx_max: 0,
            timeout_cycles: 0,
            resp_cycles: 0,
            commands: 0,
            resets: 0,
            peers: [Peer::default(); MAX_PEERS],
            host: HostState::default(),
            client: ClientState::default(),
            outbox: Vec::new(),
            dropped: 0,
            peers_seen: 0,
            connections: 0,
            self_id: 0,
            cmd_seen: 0,
            unknown_seen: 0,
        }
    }
}

impl Rfu {
    pub fn new() -> Self {
        Self::default()
    }

    /// The game pulled PD high, which power-cycles the adapter. The two
    /// diagnostic counters deliberately survive, since they are about the run
    /// rather than about the device.
    pub fn reset(&mut self) {
        // Tell any attached clients we are going BEFORE wiping the table.
        // A game power-cycling the adapter mid-session would otherwise orphan
        // them, and they spin on a wait that never resolves. gpSP carries the
        // same fix; it was not in upstream.
        if self.link == Link::Host {
            for i in 0..4 {
                let c = self.host.clients[i];
                if c.devid != 0 {
                    self.send_cmd(c.client_id, PKT_DISCONNECT, c.devid as u32 | ((i as u32) << 16));
                }
            }
        }
        let (commands, resets) = (self.commands, self.resets);
        let (dropped, seen, conns) = (self.dropped, self.peers_seen, self.connections);
        let (self_id, cmd_seen, unknown_seen) = (self.self_id, self.cmd_seen, self.unknown_seen);
        let outbox = std::mem::take(&mut self.outbox);
        *self = Self::default();
        self.outbox = outbox;
        self.self_id = self_id;
        self.cmd_seen = cmd_seen;
        self.unknown_seen = unknown_seen;
        self.commands = commands;
        self.resets = resets + 1;
        self.dropped = dropped;
        self.peers_seen = seen;
        self.connections = conns;
    }

    /// True when the adapter holds the clock and the game should be listening.
    pub fn is_master(&self) -> bool {
        matches!(self.com, Com::WaitEvent | Com::WaitResp)
    }

    /// A human-readable state, for `GBA_RFULOG` and the runner summary.
    pub fn state_name(&self) -> &'static str {
        match (self.com, self.link) {
            (Com::Reset, _) => "reset",
            (Com::Handshake, _) => "handshake",
            (_, Link::Host) => "hosting",
            (_, Link::Connecting) => "connecting",
            (_, Link::Client) => "client",
            (_, Link::Idle) => "idle",
        }
    }

    /// Exchange one 32-bit word with the adapter while the *game* drives the
    /// clock. Returns the word shifted back in.
    pub fn transfer(&mut self, sent: u32) -> u32 {
        let reply = match self.com {
            Com::Reset => {
                // The adapter wakes on a single magic half-word.
                if sent as u16 == HS_HELLO {
                    self.com = Com::Handshake;
                }
                0
            }
            Com::Handshake => {
                if sent == HS_DONE {
                    self.com = Com::WaitCmd;
                }
                // The word just sent in the high half, the previous one
                // inverted in the low half. The game checks this to confirm
                // something is really there.
                (sent << 16) | (!self.prev & 0xFFFF)
            }
            Com::WaitCmd => {
                if (sent >> 16) as u16 == CMD_HEADER {
                    self.len = (sent >> 8) as u8;
                    self.cmd = sent as u8;
                    self.pos = 0;
                    self.commands += 1;
                    self.cmd_seen |= 1u64 << (self.cmd & 0x3F);
                    if self.len == 0 {
                        self.run_command();
                    } else {
                        self.com = Com::WaitDat;
                    }
                }
                IDLE
            }
            Com::WaitDat => {
                let i = self.pos as usize;
                if i < self.buf.len() {
                    self.buf[i] = sent;
                }
                self.pos += 1;
                if self.pos >= self.len {
                    self.pos = 0;
                    self.run_command();
                }
                IDLE
            }
            Com::RespCmd => {
                let reply = 0x9966_0080 | self.cmd as u32 | ((self.len as u32) << 8);
                // Three commands hand the clock to the adapter rather than
                // expecting a reply: the game is saying "tell me when
                // something happens".
                if matches!(self.cmd, CMD_WAIT | CMD_RTX_WAIT | CMD_SEND_DATAW) {
                    self.com = Com::WaitEvent;
                    self.timeout_cycles = self.timeout as u32 * FRAME_CYCLES;
                    self.resp_cycles = self.rtx_max as u32 * (FRAME_CYCLES / 6);
                } else {
                    self.com = if self.len > 0 { Com::RespDat } else { Com::WaitCmd };
                }
                reply
            }
            Com::RespDat => {
                let reply = self.buf[self.pos as usize % MAX_RESP];
                self.pos += 1;
                if self.pos >= self.len {
                    self.com = Com::WaitCmd;
                }
                reply
            }
            Com::RespErr => {
                self.com = Com::RespErr2;
                0x9966_01EE
            }
            Com::RespErr2 => {
                self.com = Com::WaitCmd;
                self.cmd as u32
            }
            // The adapter holds the clock in these two, so a game-driven
            // transfer here is the two ends disagreeing about who is master.
            // Hardware would shift nothing useful, and neither do we.
            Com::WaitEvent | Com::WaitResp => IDLE,
        };
        self.prev = sent;
        reply
    }

    /// Run the command in `self.cmd` over the payload in `self.buf`, replacing
    /// the payload with its response, and move to the state that reports it.
    fn run_command(&mut self) {
        match self.command_response() {
            Ok(words) => {
                self.len = words;
                self.com = Com::RespCmd;
            }
            Err(code) => {
                self.cmd = code;
                self.len = 1;
                self.com = Com::RespErr;
            }
        }
    }

    /// The command table. `Ok(n)` leaves an `n`-word response in `self.buf`;
    /// `Err(code)` rejects the command.
    fn command_response(&mut self) -> Result<u8, u8> {
        match self.cmd {
            // Acknowledged without further effect. The adapter clearly does
            // something with these, but a bare ack is all any game needs.
            CMD_INIT1 | CMD_INIT2 | CMD_CFGSTAT => Ok(0),

            // Tell the other end before dropping it, so nobody is left
            // waiting on a peer that has gone.
            //
            // The two roles mean DIFFERENT things here, and conflating them was
            // a bug. A client disconnects ITSELF and goes idle. A host is
            // dropping SELECTED clients, named by a bitmask in the payload, and
            // **stays hosting**: the room outlives any one guest. Dropping every
            // client and leaving host mode on a request to remove one is how a
            // Union Room ends up with two devices that both think they are the
            // host and neither of which will accept the other's data.
            CMD_DISCONNECT => {
                match self.link {
                    Link::Host => {
                        let mask = if self.len > 0 { self.buf[0] } else { 0 };
                        for i in 0..4 {
                            let c = self.host.clients[i];
                            if mask & (1 << i) != 0 && c.devid != 0 {
                                self.send_cmd(
                                    c.client_id,
                                    PKT_DISCONNECT,
                                    c.devid as u32 | ((i as u32) << 16),
                                );
                                self.host.clients[i] = HostClient::default();
                                self.host.inbox[i].clear();
                            }
                        }
                    }
                    Link::Client => {
                        let (host, devid, clnum) =
                            (self.client.host_id, self.client.devid, self.client.clnum);
                        self.send_cmd(
                            host,
                            PKT_DISCONNECT,
                            devid as u32 | ((clnum as u32) << 16),
                        );
                        self.client = ClientState::default();
                        self.link = Link::Idle;
                    }
                    _ => {}
                }
                Ok(0)
            }

            CMD_SYSCFG => {
                // Low byte is the slave timeout, next byte the retransmit
                // count, both in adapter frames.
                self.timeout = self.buf[0] as u8;
                self.rtx_max = (self.buf[0] >> 8) as u8;
                Ok(0)
            }

            CMD_SYSVER => {
                self.buf[0] = FIRMWARE_VERSION;
                Ok(1)
            }

            CMD_SYSSTAT => {
                self.buf[0] = match self.link {
                    Link::Host => (1 << 24) | self.host.devid as u32,
                    Link::Client => {
                        (5 << 24)
                            | ((1u32 << self.client.clnum) << 16)
                            | self.client.devid as u32
                    }
                    _ => 0,
                };
                Ok(1)
            }

            // Both of these report connected peers, of which we have none.
            // Slot status leads with a count; accept leads straight into the
            // list, so an empty list is a zero-word response.
            CMD_SLOTSTAT => {
                if self.link != Link::Host {
                    return Ok(0);
                }
                // Leads with a count, then one word per attached client.
                let mut n = 1;
                self.buf[0] = 0;
                for i in 0..4 {
                    let c = self.host.clients[i];
                    if c.devid != 0 {
                        self.buf[0] += 1;
                        self.buf[n] = c.devid as u32 | ((i as u32) << 16);
                        n += 1;
                    }
                }
                Ok(n as u8)
            }
            CMD_HOST_ACCEPT => {
                if self.link == Link::Idle {
                    return Err(1);
                }
                // Despite the name this lists who is attached rather than
                // accepting anyone: the adapter has already done that.
                let mut n = 0;
                for i in 0..4 {
                    let c = self.host.clients[i];
                    if c.devid != 0 {
                        self.buf[n] = c.devid as u32 | ((i as u32) << 16);
                        n += 1;
                    }
                }
                Ok(n as u8)
            }

            CMD_LINKPWR => {
                // Signal strength, one byte per slot. We have no real measure,
                // so an attached peer reads as full strength and an empty slot
                // as none.
                self.buf[0] = match self.link {
                    Link::Host => (0..4).fold(0u32, |acc, i| {
                        if self.host.clients[i].devid != 0 {
                            acc | (0xFF << (i * 8))
                        } else {
                            acc
                        }
                    }),
                    Link::Client => 0xFFFF_FFFF,
                    _ => 0,
                };
                Ok(1)
            }

            // Opening and closing a broadcast-read session need no state of
            // their own: we are always listening, and peers age out on their
            // own schedule.
            CMD_BCRD_START | CMD_BCRD_STOP => Ok(0),

            // The actual scan result: up to four hosts, each as a device-id
            // word followed by its six words of advertisement. An empty answer
            // is a legitimate one, and is what a real adapter reports in an
            // empty room.
            CMD_BCRD_FETCH => {
                let mut n = 0;
                for i in 0..MAX_PEERS {
                    if n + 7 > MAX_RESP {
                        break;
                    }
                    let p = self.peers[i];
                    if p.valid {
                        self.buf[n] = p.device_id as u32;
                        self.buf[n + 1..n + 7].copy_from_slice(&p.data);
                        n += 7;
                    }
                }
                Ok(n as u8)
            }

            // What a host advertises: in Pokemon this carries the trainer name
            // and what the room is for, which is what the other player reads
            // off the list before joining.
            CMD_BCST_DATA => {
                if self.len == 6 {
                    self.host.bdata.copy_from_slice(&self.buf[..6]);
                }
                Ok(0)
            }

            CMD_HOST_START => {
                if self.link == Link::Client {
                    return Err(1);
                }
                if self.link == Link::Idle {
                    self.host.devid = self.new_devid();
                    self.host.clients = Default::default();
                    self.link = Link::Host;
                }
                // Announce on the next frame rather than waiting out the
                // interval, so the other player sees the room promptly.
                self.host.tx_ttl = ANNOUNCE_FRAMES;
                Ok(0)
            }

            CMD_HOST_STOP => {
                if self.link == Link::Idle {
                    return Err(1);
                }
                // This stops accepting newcomers, so a host that already has
                // someone stays a host. Only an empty room goes idle.
                if self.link == Link::Host && self.host.clients.iter().all(|c| c.devid == 0) {
                    self.link = Link::Idle;
                }
                Ok(0)
            }

            CMD_CONNECT => {
                if self.link == Link::Host {
                    return Err(1);
                }
                // The game names a host it saw in a scan. Find whose broadcast
                // carried that device id and ask them to let us in.
                let want = self.buf[0] as u16;
                if let Some(i) = (0..MAX_PEERS)
                    .find(|&i| self.peers[i].valid && self.peers[i].device_id == want)
                {
                    self.link = Link::Connecting;
                    self.send_cmd(i as u16, PKT_CONNECT_REQ, want as u32);
                }
                // An unknown id is still acknowledged: ISCONNECTED then
                // reports the failure, which is a path the game handles.
                Ok(0)
            }

            CMD_ISCONNECTED => {
                if self.link == Link::Host {
                    return Err(1);
                }
                self.buf[0] = match self.link {
                    Link::Connecting => CONN_INPROGRESS,
                    Link::Idle => CONN_FAILED,
                    _ => self.client.devid as u32 | ((self.client.clnum as u32) << 16),
                };
                Ok(1)
            }

            CMD_CONCOMPL => {
                if self.link == Link::Host {
                    return Err(1);
                }
                // Games ask this even when no connection was attempted.
                if self.link == Link::Client {
                    self.buf[0] =
                        self.client.devid as u32 | ((self.client.clnum as u32) << 16);
                } else {
                    self.buf[0] = CONN_FAILED;
                    self.link = Link::Idle;
                }
                Ok(1)
            }

            // The game hands us a block to put on the air. The first word
            // is a length header whose encoding differs by role, and the rest
            // is the payload.
            CMD_SEND_DATA | CMD_SEND_DATAW | CMD_RTX_WAIT => {
                if self.len == 0 {
                    return Ok(0);
                }
                let header = self.buf[0];
                let words = (self.len - 1) as usize;
                let mut payload = Vec::with_capacity(words * 4);
                for i in 0..words {
                    put32(&mut payload, self.buf[1 + i]);
                }
                match self.link {
                    Link::Host => {
                        let blen = (header & 0x7F) as usize;
                        if blen <= payload.len() {
                            payload.truncate(blen);
                            for i in 0..4 {
                                let c = self.host.clients[i];
                                if c.devid != 0 {
                                    self.send_data(
                                        c.client_id,
                                        PKT_HOST_SEND,
                                        blen as u32,
                                        &payload,
                                    );
                                }
                            }
                        }
                    }
                    Link::Client => {
                        // A client encodes its length in a field whose offset
                        // depends on which slot the host gave it.
                        let shift = 8 + self.client.clnum as u32 * 5;
                        let blen = ((header >> shift) & 0x1F) as usize;
                        if blen <= payload.len() {
                            payload.truncate(blen);
                            let h = self.client.devid as u32
                                | ((self.client.clnum as u32) << 16)
                                | ((blen as u32) << 24);
                            let host = self.client.host_id;
                            self.send_data(host, PKT_CLIENT_SEND, h, &payload);
                        }
                    }
                    _ => {}
                }
                Ok(0)
            }

            CMD_WAIT => Ok(0),

            // Collect one queued block. The game reads it as the response to
            // this command, so an empty queue is a zero-word answer.
            CMD_RECV_DATA => {
                let block = match self.link {
                    Link::Client => self.client.inbox.pop_front(),
                    Link::Host => (0..4)
                        .find(|&i| {
                            self.host.clients[i].devid != 0 && !self.host.inbox[i].is_empty()
                        })
                        .and_then(|i| self.host.inbox[i].pop_front()),
                    _ => None,
                };
                match block {
                    Some(b) => {
                        let words = (b.len() + 3) / 4;
                        let words = words.min(MAX_RESP);
                        for i in 0..words {
                            let mut w = [0u8; 4];
                            for k in 0..4 {
                                w[k] = b.get(i * 4 + k).copied().unwrap_or(0);
                            }
                            self.buf[i] = u32::from_be_bytes(w);
                        }
                        Ok(words as u8)
                    }
                    None => Ok(0),
                }
            }

            // **Acknowledged, not rejected, and that is deliberate.**
            //
            // This shipped as `Err(1)` on the reasoning that a game told we
            // did something we did not will wait forever for the effect. The
            // reference implementation acks instead, and it drives these exact
            // games successfully, so the reasoning was wrong here: an error
            // frame derails a state machine that would have shrugged off an
            // empty ack. Measured on device, both players reported the other
            // as permanently busy.
            //
            // The code is recorded rather than swallowed, so an unimplemented
            // command shows up in the log as a bit rather than as a silence.
            _ => {
                self.unknown_seen |= 1u64 << (self.cmd & 0x3F);
                Ok(0)
            }
        }
    }

    // --- the air -------------------------------------------------------------

    /// Tell the adapter which peer the frontend thinks we are.
    ///
    /// Must be set before a room is hosted, because the device id is derived
    /// from it. Cheap and idempotent, so the front-end pushes it every frame.
    pub fn set_self_id(&mut self, id: u16) {
        self.self_id = id;
    }

    /// Drain the packets the frontend should put on the wire.
    pub fn take_outbox(&mut self) -> Vec<OutPacket> {
        std::mem::take(&mut self.outbox)
    }

    /// True when a session is worth keeping alive: hosting, attached, or
    /// part way between the two.
    pub fn in_session(&self) -> bool {
        matches!(self.link, Link::Host | Link::Client | Link::Connecting)
    }

    fn send_cmd(&mut self, to: u16, ptype: u32, header: u32) {
        let mut bytes = Vec::with_capacity(16);
        put32(&mut bytes, NET_HEADER);
        put32(&mut bytes, ptype);
        put32(&mut bytes, header);
        put32(&mut bytes, 0);
        self.outbox.push(OutPacket { to, bytes });
    }

    fn send_data(&mut self, to: u16, ptype: u32, header: u32, payload: &[u8]) {
        let mut bytes = Vec::with_capacity(12 + payload.len());
        put32(&mut bytes, NET_HEADER);
        put32(&mut bytes, ptype);
        put32(&mut bytes, header);
        bytes.extend_from_slice(payload);
        self.outbox.push(OutPacket { to, bytes });
    }

    /// Once a frame: re-announce a hosted room, and age out anything we have
    /// stopped hearing from.
    pub fn frame_update(&mut self) {
        if self.com == Com::Reset {
            return;
        }
        for p in self.peers.iter_mut() {
            if p.valid {
                p.ttl = p.ttl.saturating_sub(1);
                if p.ttl == 0 {
                    p.valid = false;
                }
            }
        }
        if self.link == Link::Host {
            self.host.tx_ttl = self.host.tx_ttl.saturating_add(1);
            if self.host.tx_ttl >= ANNOUNCE_FRAMES {
                self.host.tx_ttl = 0;
                let (devid, bdata) = (self.host.devid, self.host.bdata);
                let mut payload = Vec::with_capacity(24);
                for w in bdata {
                    put32(&mut payload, w);
                }
                self.send_data(BROADCAST_ID, PKT_BROADCAST, devid as u32, &payload);
            }
            for i in 0..4 {
                if self.host.clients[i].devid != 0 {
                    self.host.clients[i].ttl = self.host.clients[i].ttl.saturating_add(1);
                    if self.host.clients[i].ttl >= CLIENT_TTL {
                        self.host.clients[i] = HostClient::default();
                        self.host.inbox[i].clear();
                    }
                }
            }
        } else if self.link == Link::Client {
            self.client.host_ttl = self.client.host_ttl.saturating_add(1);
            if self.client.host_ttl >= CLIENT_TTL {
                self.link = Link::Idle;
                self.client = ClientState::default();
            }
        }
    }

    /// A packet arrived from `from`, the frontend id for that peer.
    pub fn net_receive(&mut self, buf: &[u8], from: u16) {
        if buf.len() < 12 || get32(buf, 0) != NET_HEADER {
            return;
        }
        let ptype = get32(buf, 4);
        let hdata = get32(buf, 8);
        let payload = &buf[12..];

        // Anything at all from our host proves it is alive. A host announces
        // twice a second even when idle, so this keeps a quiet session from
        // timing out.
        if self.link == Link::Client && from == self.client.host_id {
            self.client.host_ttl = 0;
        }

        match ptype {
            PKT_BROADCAST => {
                if (from as usize) < MAX_PEERS && payload.len() >= 24 {
                    if !self.peers[from as usize].valid {
                        self.peers_seen += 1;
                    }
                    let slot = &mut self.peers[from as usize];
                    slot.valid = true;
                    slot.device_id = hdata as u16;
                    slot.ttl = PEER_TTL;
                    for j in 0..6 {
                        slot.data[j] = get32(payload, j * 4);
                    }
                }
            }

            PKT_CONNECT_REQ => {
                if self.link != Link::Host {
                    self.send_cmd(from, PKT_CONNECT_NACK, 0);
                    return;
                }
                // Already attached: ignore, rather than hand out a second slot
                // to a peer that already has one.
                if self
                    .host
                    .clients
                    .iter()
                    .any(|c| c.devid != 0 && c.client_id == from)
                {
                    return;
                }
                match self.host.clients.iter().position(|c| c.devid == 0) {
                    Some(i) => {
                        let newid = self.new_devid();
                        self.host.clients[i] = HostClient {
                            devid: newid,
                            client_id: from,
                            ttl: 0,
                        };
                        self.connections += 1;
                        self.send_cmd(from, PKT_CONNECT_ACK, newid as u32 | ((i as u32) << 16));
                    }
                    None => self.send_cmd(from, PKT_CONNECT_NACK, 0),
                }
            }

            PKT_CONNECT_ACK => {
                if self.link == Link::Connecting {
                    self.client = ClientState {
                        devid: hdata as u16,
                        clnum: (hdata >> 16) as u8,
                        host_id: from,
                        host_ttl: 0,
                        inbox: Default::default(),
                    };
                    self.link = Link::Client;
                    self.connections += 1;
                }
            }

            PKT_CONNECT_NACK => {
                if self.link == Link::Connecting {
                    self.link = Link::Idle;
                }
            }

            PKT_DISCONNECT => {
                if self.link == Link::Client && from == self.client.host_id {
                    self.link = Link::Idle;
                    self.client = ClientState::default();
                } else if self.link == Link::Host {
                    if let Some(i) = self
                        .host
                        .clients
                        .iter()
                        .position(|c| c.client_id == from && c.devid != 0)
                    {
                        self.host.clients[i] = HostClient::default();
                        self.host.inbox[i].clear();
                    }
                }
            }

            PKT_HOST_SEND => {
                if self.link != Link::Client {
                    return;
                }
                let blen = (hdata & 0x7F) as usize;
                if payload.len() < blen {
                    return;
                }
                // Acknowledge first, so the host knows we are still here even
                // when our queue is full and the payload has to be dropped.
                let ack = self.client.devid as u32 | ((self.client.clnum as u32) << 16);
                self.send_cmd(from, PKT_CLIENT_ACK, ack);
                if self.client.inbox.len() >= QUEUE_DEPTH {
                    self.dropped += 1;
                } else {
                    self.client.inbox.push_back(payload[..blen].to_vec());
                }
            }

            PKT_CLIENT_SEND => {
                if self.link != Link::Host {
                    return;
                }
                let cdevid = hdata as u16;
                let slot = ((hdata >> 16) & 3) as usize;
                let blen = (hdata >> 24) as usize;
                // The slot is validated against the device id we handed out,
                // so a stale or forged packet cannot write into the queue of a
                // different client.
                if cdevid == 0 || self.host.clients[slot].devid != cdevid {
                    return;
                }
                if payload.len() < blen {
                    return;
                }
                self.host.clients[slot].ttl = 0;
                if self.host.inbox[slot].len() >= QUEUE_DEPTH {
                    self.dropped += 1;
                } else {
                    self.host.inbox[slot].push_back(payload[..blen].to_vec());
                }
            }

            PKT_CLIENT_ACK => {
                if self.link == Link::Host {
                    let devid = hdata as u16;
                    let slot = ((hdata >> 16) & 3) as usize;
                    if devid != 0 && self.host.clients[slot].devid == devid {
                        self.host.clients[slot].ttl = 0;
                    }
                }
            }

            _ => {}
        }
    }

    /// Is there anything from the air for the game to collect?
    fn data_available(&self) -> bool {
        match self.link {
            Link::Client => !self.client.inbox.is_empty(),
            Link::Host => {
                (0..4).any(|i| self.host.clients[i].devid != 0 && !self.host.inbox[i].is_empty())
            }
            _ => false,
        }
    }

    /// A device id for a host or a client slot.
    ///
    /// **This has to differ between two devices, and that is harder for us
    /// than for hardware.** A real adapter has an id burned in, and gpSP uses
    /// `rand() ^ time()`. A deterministic core has neither: two phones running
    /// the same ROM from the same save through the same menus execute the same
    /// instructions and would mint the same id.
    ///
    /// That is not a cosmetic collision. A game that scans and finds a room
    /// advertising its OWN device id has found itself, and will not connect to
    /// it. Measured on device 2026-10-03: both players saw each other in the
    /// Union Room and neither ever sent a single connect request, because the
    /// id came only from the command count and both sides had minted the same
    /// one. Every packet on the wire was a 36-byte broadcast and not one was
    /// unicast.
    ///
    /// So the high nibble-and-a-bit comes from the frontend's peer id, which
    /// is the one thing that genuinely differs, and the rest keeps a single
    /// device reproducible across runs.
    fn new_devid(&self) -> u16 {
        0x1000 | ((self.self_id & 0x0F) << 8) | ((self.commands as u16) & 0x00FF) | 1
    }

    /// Advance the adapter's own clock by `cycles`.
    ///
    /// Returns `Some(word)` when the adapter has taken the clock and wants to
    /// push a word at the game, which the caller lands in SIODATA32.
    ///
    /// `game_is_slave` is SIOCNT bit 0 clear, `so_si_clear` is bits 2 and 3
    /// both clear, and `armed` is the start bit set. The adapter will not talk
    /// over a game that still believes it is master, which is what the first
    /// of those three guards.
    pub fn step(
        &mut self,
        cycles: u32,
        game_is_slave: bool,
        so_si_clear: bool,
        armed: bool,
    ) -> Option<u32> {
        if self.com == Com::WaitEvent {
            self.timeout_cycles = self.timeout_cycles.saturating_sub(cycles);
            self.resp_cycles = self.resp_cycles.saturating_sub(cycles);
            if game_is_slave {
                if self.link == Link::Idle {
                    // Nothing is connected, which is the honest answer.
                    self.buf[0] = 0x9966_0000 | (1 << 8) | RESP_DISC as u32;
                    self.buf[1] = 0xF; // every slot disconnected
                    self.buf[2] = IDLE;
                    self.len = 3;
                    self.pos = 0;
                    self.com = Com::WaitResp;
                } else if self.data_available() {
                    // Something arrived from the air. The game answers this by
                    // issuing RECV_DATA to collect it.
                    self.buf[0] = 0x9966_0000 | RESP_DATA as u32;
                    self.buf[1] = IDLE;
                    self.len = 2;
                    self.pos = 0;
                    self.com = Com::WaitResp;
                } else if self.link == Link::Host && self.resp_cycles == 0 {
                    // A retransmit round elapsed with no client answering.
                    self.buf[0] = 0x9966_0000 | (1 << 8) | RESP_DATA as u32;
                    self.buf[1] = 0x0000_0F0F;
                    self.buf[2] = IDLE;
                    self.len = 3;
                    self.pos = 0;
                    self.com = Com::WaitResp;
                } else if self.timeout_cycles == 0 {
                    self.buf[0] = 0x9966_0000 | RESP_TIMEOUT as u32;
                    self.buf[1] = IDLE;
                    self.len = 2;
                    self.pos = 0;
                    self.com = Com::WaitResp;
                }
            }
        }

        if self.com == Com::WaitResp && so_si_clear && armed {
            let word = self.buf[self.pos as usize % MAX_RESP];
            self.pos += 1;
            if self.pos >= self.len {
                self.com = Com::WaitCmd;
            }
            return Some(word);
        }
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Walk the adapter through the power-on handshake and into command mode.
    fn handshake(rfu: &mut Rfu) {
        assert_eq!(rfu.transfer(0x0000_494E), 0, "the wake word is acked with zero");
        rfu.transfer(0x1234_5678);
        rfu.transfer(HS_DONE);
        assert_eq!(rfu.state_name(), "idle", "handshake complete");
    }

    #[test]
    fn ignores_traffic_until_the_wake_word() {
        let mut rfu = Rfu::new();
        // Anything that is not the wake word leaves it powered down. The last
        // of these is the wake word in the wrong half, which must not count.
        for w in [0u32, 0xFFFF_FFFF, 0x9966_0010, 0x494E_0000] {
            assert_eq!(rfu.transfer(w), 0);
            assert_eq!(rfu.state_name(), "reset", "{w:08X} must not wake the adapter");
        }
        assert_eq!(rfu.transfer(0x0000_494E), 0);
        assert_eq!(rfu.state_name(), "handshake");
    }

    #[test]
    fn handshake_echoes_the_previous_word_inverted() {
        let mut rfu = Rfu::new();
        rfu.transfer(0x0000_494E);
        let r = rfu.transfer(0xAAAA_5555);
        assert_eq!(r, (0xAAAA_5555u32 << 16) | (!0x0000_494Eu32 & 0xFFFF));
        let r = rfu.transfer(0x1111_2222);
        assert_eq!(r, (0x1111_2222u32 << 16) | (!0xAAAA_5555u32 & 0xFFFF));
    }

    #[test]
    fn version_command_returns_the_firmware_word() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        // A zero-payload command is processed as soon as its header lands.
        assert_eq!(rfu.transfer(0x9966_0012), IDLE);
        // Then the adapter acks it, and the response follows. Both are
        // asserted as literals rather than against our own constants: a test
        // phrased against the symbol it is checking agrees with any bug in it.
        assert_eq!(rfu.transfer(0), 0x9966_0192);
        assert_eq!(rfu.transfer(0), 0x0083_0117);
        // And it is back to waiting for the next command.
        assert_eq!(rfu.transfer(0), IDLE);
    }

    #[test]
    fn a_command_header_in_the_wrong_half_is_not_a_command() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        assert_eq!(rfu.transfer(0x0012_9966), IDLE);
        assert_eq!(rfu.commands, 0, "framing is checked, not just scanned for");
    }

    #[test]
    fn syscfg_payload_sets_the_timeouts() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        // One payload word: timeout 0x20 frames, 4 retransmits.
        rfu.transfer(0x9966_0117);
        rfu.transfer(0x0000_0420);
        assert_eq!(rfu.timeout, 0x20);
        assert_eq!(rfu.rtx_max, 4);
    }

    #[test]
    fn unknown_commands_are_acknowledged_and_recorded() {
        // This asserted the OPPOSITE until 2026-10-03, on the reasoning that a
        // game told we did something we did not will wait forever for the
        // effect. The reference acks, and drives these games successfully; an
        // error frame derails a state machine that would have shrugged off an
        // empty ack. The test was a faithful record of a wrong decision, which
        // is why it had to be rewritten rather than deleted.
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0039); // no such command
        assert_eq!(rfu.transfer(0), 0x9966_0039 | 0x80, "a plain ack, no payload");
        assert_eq!(rfu.transfer(0), IDLE, "and back to command wait");
        // But it is recorded, so an unimplemented command shows up in the log
        // as a bit rather than as a silence.
        assert_eq!(rfu.unknown_seen, 1 << 0x39);
        assert_ne!(rfu.cmd_seen & (1 << 0x39), 0);
    }

    #[test]
    fn a_host_disconnecting_one_client_keeps_the_room_and_the_others() {
        // A client disconnects ITSELF. A host drops the clients named in a
        // bitmask and STAYS HOSTING. Conflating the two left two devices both
        // believing they were the host, each rejecting the other's data as
        // coming from a non-client, which reads in game as the other trainer
        // being permanently busy.
        let mut host = Rfu::new();
        host.set_self_id(0);
        handshake(&mut host);
        cmd(&mut host, CMD_HOST_START, &[]);
        let mut req = Vec::new();
        put32(&mut req, NET_HEADER);
        put32(&mut req, PKT_CONNECT_REQ);
        put32(&mut req, 0x1234);
        put32(&mut req, 0);
        host.net_receive(&req, 1);
        host.net_receive(&req, 2);
        assert_eq!(cmd(&mut host, CMD_SLOTSTAT, &[])[0], 2, "two guests");

        // Drop slot 0 only.
        host.take_outbox();
        cmd(&mut host, CMD_DISCONNECT, &[0b0001]);
        assert_eq!(host.state_name(), "hosting", "the room outlives the guest");
        let slots = cmd(&mut host, CMD_SLOTSTAT, &[]);
        assert_eq!(slots[0], 1, "one guest left");
        assert_eq!(slots[1] >> 16, 1, "and it is the one in slot 1");
        // The dropped one was told.
        let out = host.take_outbox();
        assert!(out.iter().any(|p| get32(&p.bytes, 4) == PKT_DISCONNECT));
    }

    #[test]
    fn a_client_disconnecting_leaves_and_tells_the_host() {
        let (mut host, mut client) = joined_pair();
        client.take_outbox();
        cmd(&mut client, CMD_DISCONNECT, &[0]);
        assert_eq!(client.state_name(), "idle", "the client has left");
        deliver(&mut client, 1, &mut host, 0);
        assert_eq!(cmd(&mut host, CMD_SLOTSTAT, &[])[0], 0, "host sees it go");
        assert_eq!(host.state_name(), "hosting", "and keeps the room open");
    }

    #[test]
    fn hosting_is_entered_and_left() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0019); // host start
        assert_eq!(rfu.state_name(), "hosting");
        assert_ne!(rfu.host.devid, 0, "a host needs a device id");
        rfu.transfer(0); // consume the ack
        rfu.transfer(0x9966_001B); // host stop; no clients, so back to idle
        assert_eq!(rfu.state_name(), "idle");
    }

    #[test]
    fn host_stop_while_idle_is_an_error() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_001B);
        assert_eq!(rfu.transfer(0), 0x9966_01EE, "stopping a host we never started");
    }

    #[test]
    fn a_wait_hands_the_clock_over_and_reports_no_peers() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0027); // WAIT
        rfu.transfer(0); // ack; this is where the roles reverse
        assert!(rfu.is_master(), "the adapter now drives the clock");
        // Nothing happens while the game still believes it is master.
        assert_eq!(rfu.step(FRAME_CYCLES, false, true, true), None);
        // Once it switches to slave, the adapter reports the disconnection.
        let w = rfu.step(FRAME_CYCLES, true, true, true).expect("a response");
        assert_eq!(w, 0x9966_0000 | (1 << 8) | RESP_DISC as u32);
        assert_eq!(rfu.step(0, true, true, true), Some(0xF));
        assert_eq!(rfu.step(0, true, true, true), Some(IDLE));
        // Three words, then the adapter hands the clock back.
        assert_eq!(rfu.step(0, true, true, true), None);
        assert!(!rfu.is_master());
    }

    #[test]
    fn a_response_waits_for_the_game_to_be_listening() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0027);
        rfu.transfer(0);
        // The game is slave so a response is ready, but SO/SI are not clear
        // and no transfer is armed, so the adapter holds on to it.
        assert_eq!(rfu.step(FRAME_CYCLES, true, false, true), None);
        assert_eq!(rfu.step(0, true, true, false), None);
        assert!(rfu.step(0, true, true, true).is_some());
    }

    #[test]
    fn a_reset_powers_the_adapter_down_but_keeps_the_counters() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0019);
        let seen = rfu.commands;
        assert!(seen > 0);
        rfu.reset();
        assert_eq!(rfu.state_name(), "reset");
        assert_eq!(rfu.commands, seen, "the command count survives a reset");
        assert_eq!(rfu.resets, 1);
        // And the handshake is required all over again.
        assert_eq!(rfu.transfer(0x9966_0012), 0, "commands ignored while powered down");
        assert_eq!(rfu.commands, seen, "and not counted either");
    }

    // --- two adapters, through a model of the frontend wire -----------------
    //
    // The frontend gives every peer a client id and delivers a packet either to
    // one id or to everyone. That is the whole contract, so it models in a few
    // lines, and it lets the session layer be tested end to end with no
    // network, no device and no game.

    /// Move whatever `from` has queued into `to`, the way the frontend would.
    fn deliver(from: &mut Rfu, from_id: u16, to: &mut Rfu, to_id: u16) {
        for pkt in from.take_outbox() {
            if pkt.to == BROADCAST_ID || pkt.to == to_id {
                to.net_receive(&pkt.bytes, from_id);
            }
        }
    }

    /// Issue one command the way a game does, and return its response words.
    fn cmd(rfu: &mut Rfu, c: u8, payload: &[u32]) -> Vec<u32> {
        rfu.transfer(0x9966_0000 | ((payload.len() as u32) << 8) | c as u32);
        for &w in payload {
            rfu.transfer(w);
        }
        let ack = rfu.transfer(0);
        assert_eq!(
            ack & 0xFFFF_00FF,
            0x9966_0080 | c as u32,
            "command {c:#04X} was not acknowledged"
        );
        let n = ((ack >> 8) & 0xFF) as usize;
        (0..n).map(|_| rfu.transfer(0)).collect()
    }

    /// A host advertising, and a second adapter that scans and finds it.
    fn hosted_pair() -> (Rfu, Rfu, u16) {
        let mut host = Rfu::new();
        let mut client = Rfu::new();
        // Two devices are two different peers, and the fixtures have to say so.
        // Left at the default they are indistinguishable, which is exactly the
        // condition that hid the device-id collision from this suite.
        host.set_self_id(0);
        client.set_self_id(1);
        handshake(&mut host);
        handshake(&mut client);
        cmd(&mut host, CMD_BCST_DATA, &[11, 22, 33, 44, 55, 66]);
        cmd(&mut host, CMD_HOST_START, &[]);
        host.frame_update();
        deliver(&mut host, 0, &mut client, 1);
        let devid = host.host.devid;
        (host, client, devid)
    }

    #[test]
    fn a_scan_finds_a_hosted_room_and_its_advertisement() {
        let (_host, mut client, devid) = hosted_pair();
        let scan = cmd(&mut client, CMD_BCRD_FETCH, &[]);
        assert_eq!(scan.len(), 7, "one peer: a device id and six words");
        assert_eq!(scan[0], devid as u32);
        assert_eq!(
            &scan[1..7],
            &[11, 22, 33, 44, 55, 66],
            "the advertisement survives the wire, which is the trainer name"
        );
        assert_eq!(client.peers_seen, 1);
    }

    #[test]
    fn two_devices_running_in_lockstep_still_mint_different_device_ids() {
        // The bug this pins cost a device session. Two phones on the same ROM,
        // same save and same menus execute the same instructions, so anything
        // derived purely from emulated state is identical on both. A game that
        // scans and finds a room advertising its own id has found itself and
        // will not connect, which presents as the other player being busy
        // while not one connect request is ever sent.
        let mut a = Rfu::new();
        let mut b = Rfu::new();
        a.set_self_id(0);
        b.set_self_id(1);
        handshake(&mut a);
        handshake(&mut b);
        // Identical command streams from here on, deliberately.
        cmd(&mut a, CMD_HOST_START, &[]);
        cmd(&mut b, CMD_HOST_START, &[]);
        assert_ne!(
            a.host.devid, b.host.devid,
            "two devices must not advertise the same device id"
        );
        assert_ne!(a.host.devid, 0);
        assert_ne!(b.host.devid, 0);
    }

    #[test]
    fn a_scan_never_turns_up_the_scanning_device_itself() {
        // The end-to-end shape of the same bug: whatever a device finds in a
        // scan must not be its own id, or the game has nothing valid to join.
        let (host, mut client, devid) = hosted_pair();
        cmd(&mut client, CMD_HOST_START, &[]);
        let own = client.host.devid;
        let scan = cmd(&mut client, CMD_BCRD_FETCH, &[]);
        assert_eq!(scan[0], devid as u32, "it found the other room");
        assert_ne!(scan[0], own as u32, "and that room is not itself");
        assert_ne!(host.host.devid, own);
    }

    #[test]
    fn a_scan_finds_nothing_before_anyone_is_hosting() {
        let mut a = Rfu::new();
        let mut b = Rfu::new();
        handshake(&mut a);
        handshake(&mut b);
        a.frame_update();
        deliver(&mut a, 0, &mut b, 1);
        assert!(cmd(&mut b, CMD_BCRD_FETCH, &[]).is_empty());
    }

    #[test]
    fn a_client_joins_a_host_and_both_sides_agree() {
        let (mut host, mut client, devid) = hosted_pair();
        cmd(&mut client, CMD_BCRD_FETCH, &[]);
        cmd(&mut client, CMD_CONNECT, &[devid as u32]);
        assert_eq!(client.state_name(), "connecting");
        deliver(&mut client, 1, &mut host, 0);
        deliver(&mut host, 0, &mut client, 1);
        assert_eq!(client.state_name(), "client");
        assert_eq!(host.state_name(), "hosting");

        // The host lists one attached client, in slot 0.
        let slots = cmd(&mut host, CMD_SLOTSTAT, &[]);
        assert_eq!(slots[0], 1, "one client attached");
        assert_eq!(slots[1] >> 16, 0, "in slot 0");

        // And both ends report the connection through their status command.
        let st = cmd(&mut client, CMD_SYSSTAT, &[]);
        assert_eq!(st[0] >> 24, 5, "a client reports status 5");
        let st = cmd(&mut host, CMD_SYSSTAT, &[]);
        assert_eq!(st[0] >> 24, 1, "a host reports status 1");
    }

    #[test]
    fn connecting_to_an_id_nobody_advertised_fails_rather_than_hanging() {
        let (_host, mut client, devid) = hosted_pair();
        cmd(&mut client, CMD_BCRD_FETCH, &[]);
        cmd(&mut client, CMD_CONNECT, &[devid as u32 ^ 0x5A5A]);
        assert_eq!(client.state_name(), "idle", "no request went out");
        let r = cmd(&mut client, CMD_ISCONNECTED, &[]);
        assert_eq!(r[0], CONN_FAILED, "and the game is told so");
    }

    #[test]
    fn an_adapter_that_is_not_hosting_rejects_a_connection_request() {
        // An idle adapter must say no rather than ignore the request: a game
        // waiting on a reply that never comes is the worst outcome.
        let mut idle = Rfu::new();
        handshake(&mut idle);
        let mut req = Vec::new();
        put32(&mut req, NET_HEADER);
        put32(&mut req, PKT_CONNECT_REQ);
        put32(&mut req, 0x1234);
        put32(&mut req, 0);
        idle.net_receive(&req, 0);
        let out = idle.take_outbox();
        assert_eq!(out.len(), 1, "exactly one reply");
        assert_eq!(get32(&out[0].bytes, 4), PKT_CONNECT_NACK);

        // A host with all four slots taken must also refuse.
        let mut full = Rfu::new();
        handshake(&mut full);
        cmd(&mut full, CMD_HOST_START, &[]);
        for peer in 1..=4u16 {
            full.net_receive(&req, peer);
        }
        full.take_outbox();
        full.net_receive(&req, 5);
        let out = full.take_outbox();
        assert_eq!(get32(&out[0].bytes, 4), PKT_CONNECT_NACK, "no slots left");
    }

    #[test]
    fn a_host_accepts_four_clients_and_gives_each_its_own_slot() {
        let mut host = Rfu::new();
        handshake(&mut host);
        cmd(&mut host, CMD_HOST_START, &[]);
        let mut req = Vec::new();
        put32(&mut req, NET_HEADER);
        put32(&mut req, PKT_CONNECT_REQ);
        put32(&mut req, 0x1234);
        put32(&mut req, 0);
        for peer in 1..=4u16 {
            host.net_receive(&req, peer);
        }
        let slots = cmd(&mut host, CMD_SLOTSTAT, &[]);
        assert_eq!(slots[0], 4, "four attached");
        let assigned: Vec<u32> = slots[1..5].iter().map(|w| w >> 16).collect();
        assert_eq!(assigned, vec![0, 1, 2, 3], "one slot each");
        // And a repeat request from a peer already attached changes nothing.
        host.net_receive(&req, 1);
        assert_eq!(cmd(&mut host, CMD_SLOTSTAT, &[])[0], 4, "still four");
    }

    /// A joined pair, ready to exchange data.
    fn joined_pair() -> (Rfu, Rfu) {
        let (mut host, mut client, devid) = hosted_pair();
        cmd(&mut client, CMD_BCRD_FETCH, &[]);
        cmd(&mut client, CMD_CONNECT, &[devid as u32]);
        deliver(&mut client, 1, &mut host, 0);
        deliver(&mut host, 0, &mut client, 1);
        (host, client)
    }

    #[test]
    fn a_host_block_reaches_the_client() {
        let (mut host, mut client) = joined_pair();
        // Four bytes of payload: the header low 7 bits carry the byte count.
        cmd(&mut host, CMD_SEND_DATA, &[4, 0xDEAD_BEEF]);
        deliver(&mut host, 0, &mut client, 1);
        let got = cmd(&mut client, CMD_RECV_DATA, &[]);
        assert_eq!(got, vec![0xDEAD_BEEF], "the block arrives intact");
        // And the client acknowledged it, which is how the host knows it lives.
        assert!(client
            .take_outbox()
            .iter()
            .any(|p| get32(&p.bytes, 4) == PKT_CLIENT_ACK));
    }

    #[test]
    fn a_client_block_reaches_the_host() {
        let (mut host, mut client) = joined_pair();
        // A client encodes its length five bits up, offset by its slot number.
        cmd(&mut client, CMD_SEND_DATA, &[4 << 8, 0x0BAD_F00D]);
        deliver(&mut client, 1, &mut host, 0);
        let got = cmd(&mut host, CMD_RECV_DATA, &[]);
        assert_eq!(got, vec![0x0BAD_F00D]);
    }

    #[test]
    fn a_client_in_a_later_slot_encodes_its_length_in_a_different_field() {
        // Every other data test puts the client in slot 0, where the length
        // field happens to sit at the same offset for everyone. Slot 1 is what
        // actually exercises the per-slot shift, and a trade with three
        // players would corrupt silently if it were wrong.
        let mut host = Rfu::new();
        let mut client = Rfu::new();
        handshake(&mut host);
        handshake(&mut client);
        cmd(&mut host, CMD_HOST_START, &[]);

        // Fill slot 0 with somebody else so our client lands in slot 1.
        let mut req = Vec::new();
        put32(&mut req, NET_HEADER);
        put32(&mut req, PKT_CONNECT_REQ);
        put32(&mut req, 0x1234);
        put32(&mut req, 0);
        host.net_receive(&req, 7);
        host.take_outbox();

        host.frame_update();
        deliver(&mut host, 0, &mut client, 1);
        cmd(&mut client, CMD_BCRD_FETCH, &[]);
        let devid = host.host.devid;
        cmd(&mut client, CMD_CONNECT, &[devid as u32]);
        deliver(&mut client, 1, &mut host, 0);
        deliver(&mut host, 0, &mut client, 1);
        assert_eq!(client.client.clnum, 1, "second slot");

        // Four bytes, with the count at 8 + 1*5 = bit 13.
        cmd(&mut client, CMD_SEND_DATA, &[4 << 13, 0xFEED_FACE]);
        deliver(&mut client, 1, &mut host, 0);
        assert_eq!(
            cmd(&mut host, CMD_RECV_DATA, &[]),
            vec![0xFEED_FACE],
            "a slot 1 client is understood by the host"
        );
    }

    #[test]
    fn arriving_data_is_what_wakes_a_waiting_game() {
        let (mut host, mut client) = joined_pair();
        // The client hands the clock over and waits for something to happen.
        cmd(&mut client, CMD_WAIT, &[]);
        assert!(client.is_master());
        cmd(&mut host, CMD_SEND_DATA, &[4, 0x1234_5678]);
        deliver(&mut host, 0, &mut client, 1);
        // The adapter now reports data rather than a timeout or a disconnect.
        let w = client.step(1, true, true, true).expect("an event");
        assert_eq!(
            w,
            0x9966_0000 | RESP_DATA as u32,
            "data available, not a timeout"
        );
    }

    #[test]
    fn a_forged_slot_cannot_write_into_another_clients_queue() {
        let (mut host, _client) = joined_pair();
        // Slot 0 is taken by a client with a known device id. A packet naming
        // that slot with the wrong id must be discarded.
        let mut v = Vec::new();
        put32(&mut v, NET_HEADER);
        put32(&mut v, PKT_CLIENT_SEND);
        put32(&mut v, 0xDEAD | (0 << 16) | (4 << 24));
        put32(&mut v, 0x4141_4141);
        host.net_receive(&v, 9);
        assert!(
            cmd(&mut host, CMD_RECV_DATA, &[]).is_empty(),
            "a packet with the wrong device id is dropped"
        );
    }

    #[test]
    fn a_host_that_power_cycles_tells_its_clients_rather_than_orphaning_them() {
        let (mut host, mut client) = joined_pair();
        host.take_outbox();
        // The game pulls the adapter reset line mid-session.
        host.reset();
        deliver(&mut host, 0, &mut client, 1);
        assert_eq!(
            client.state_name(),
            "idle",
            "the client is told, instead of spinning on a wait that never ends"
        );
    }

    #[test]
    fn a_silent_host_eventually_times_out() {
        let (_host, mut client) = joined_pair();
        assert_eq!(client.state_name(), "client");
        for _ in 0..CLIENT_TTL as u32 + 1 {
            client.frame_update();
        }
        assert_eq!(client.state_name(), "idle", "a vanished host is given up on");
    }

    #[test]
    fn a_peer_that_stops_advertising_drops_off_the_scan() {
        let (_host, mut client, _devid) = hosted_pair();
        assert_eq!(cmd(&mut client, CMD_BCRD_FETCH, &[]).len(), 7);
        for _ in 0..PEER_TTL as u32 + 1 {
            client.frame_update();
        }
        assert!(
            cmd(&mut client, CMD_BCRD_FETCH, &[]).is_empty(),
            "a room nobody is announcing any more stops being listed"
        );
    }

    #[test]
    fn packets_are_byte_compatible_with_the_reference_implementation() {
        // The header is "RFU1" in big-endian, and a command packet is exactly
        // four words. Asserted as literal bytes rather than against our own
        // constants, because the point is agreeing with gpSP, not with
        // ourselves.
        let mut host = Rfu::new();
        handshake(&mut host);
        cmd(&mut host, CMD_HOST_START, &[]);
        host.frame_update();
        let out = host.take_outbox();
        let pkt = &out[0];
        assert_eq!(pkt.to, 0xFFFF, "a room announcement goes to everyone");
        assert_eq!(&pkt.bytes[0..4], b"RFU1");
        assert_eq!(&pkt.bytes[4..8], &[0, 0, 0, 0], "type 0 is a broadcast");
        assert_eq!(pkt.bytes.len(), 36, "header, type, device id, six words");
    }

    #[test]
    fn a_short_or_misheaded_packet_is_ignored() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.net_receive(&[0u8; 8], 1); // too short to carry a header
        let mut wrong = Vec::new();
        put32(&mut wrong, 0x5246_5532); // RFU2, not ours
        put32(&mut wrong, PKT_BROADCAST);
        put32(&mut wrong, 0x99);
        wrong.extend_from_slice(&[0u8; 24]);
        rfu.net_receive(&wrong, 1);
        assert_eq!(rfu.peers_seen, 0, "neither packet was believed");
    }

    #[test]
    fn detection_matches_the_adapter_titles_and_nothing_else() {
        let mut rom = vec![0u8; 0x100];
        for code in [b"BPEE", b"BPRE", b"BMGE", b"BR5E", b"BKRJ"] {
            rom[0xAC..0xB0].copy_from_slice(code);
            assert!(detect(&rom), "{} is an adapter title", std::str::from_utf8(code).unwrap());
        }
        // Ruby and Sapphire trade over a cable, not the adapter; Pokemon
        // Pinball is a Game Boy Player title; Mario Kart and Advance Wars are
        // cable too. None of them may match.
        for code in [b"AXVE", b"AXPE", b"BPPE", b"AMKE", b"AWRE", b"AGBJ"] {
            rom[0xAC..0xB0].copy_from_slice(code);
            assert!(!detect(&rom), "{} has no adapter", std::str::from_utf8(code).unwrap());
        }
        assert!(!detect(&[0u8; 8]), "a ROM too short to carry a code");
    }
}
