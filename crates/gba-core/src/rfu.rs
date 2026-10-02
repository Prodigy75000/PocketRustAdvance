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
    /// Our own device id once hosting. Zero until then.
    devid: u16,
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
            devid: 0,
            timeout_cycles: 0,
            resp_cycles: 0,
            commands: 0,
            resets: 0,
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
        let (commands, resets) = (self.commands, self.resets);
        *self = Self::default();
        self.commands = commands;
        self.resets = resets + 1;
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
            CMD_INIT1 | CMD_INIT2 | CMD_CFGSTAT | CMD_RECV_DATA | CMD_DISCONNECT => Ok(0),

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
                    Link::Host => (1 << 24) | self.devid as u32,
                    // With no session layer we are never a client.
                    _ => 0,
                };
                Ok(1)
            }

            // Both of these report connected peers, of which we have none.
            // Slot status leads with a count; accept leads straight into the
            // list, so an empty list is a zero-word response.
            CMD_SLOTSTAT => {
                if self.link == Link::Host {
                    self.buf[0] = 0;
                    Ok(1)
                } else {
                    Ok(0)
                }
            }
            CMD_HOST_ACCEPT => {
                if self.link == Link::Idle {
                    return Err(1);
                }
                Ok(0)
            }

            CMD_LINKPWR => {
                // Signal strength per slot. Nobody connected, no signal.
                self.buf[0] = 0;
                Ok(1)
            }

            // A broadcast-read session is how a game finds nearby hosts. With
            // no session layer there is never anything in the air, so a fetch
            // returns no peers. That is a legitimate answer rather than a
            // stub: it is exactly what a real adapter reports in an empty room.
            CMD_BCRD_START | CMD_BCRD_STOP | CMD_BCRD_FETCH => Ok(0),

            // The data a host puts in the air for others to find. Nothing is
            // listening yet, so record nothing and tell the game it landed.
            CMD_BCST_DATA => Ok(0),

            CMD_HOST_START => {
                if self.link == Link::Client {
                    return Err(1);
                }
                if self.link == Link::Idle {
                    self.devid = self.new_devid();
                    self.link = Link::Host;
                }
                Ok(0)
            }

            CMD_HOST_STOP => {
                if self.link == Link::Idle {
                    return Err(1);
                }
                // This stops accepting newcomers. With no clients attached
                // there is nothing left to host, so the adapter goes idle.
                if self.link == Link::Host {
                    self.link = Link::Idle;
                }
                Ok(0)
            }

            CMD_CONNECT => {
                if self.link == Link::Host {
                    return Err(1);
                }
                // The game named a host it saw in a broadcast. It cannot have
                // seen one, so acknowledge and let ISCONNECTED report the
                // failure, which is a path the game already handles.
                Ok(0)
            }

            CMD_ISCONNECTED => {
                if self.link == Link::Host {
                    return Err(1);
                }
                self.buf[0] = match self.link {
                    Link::Connecting => CONN_INPROGRESS,
                    Link::Idle => CONN_FAILED,
                    _ => 0,
                };
                Ok(1)
            }

            CMD_CONCOMPL => {
                if self.link == Link::Host {
                    return Err(1);
                }
                // Games ask this even when no connection was attempted.
                self.buf[0] = CONN_FAILED;
                self.link = Link::Idle;
                Ok(1)
            }

            // Sending with nobody to send to. The payload is accepted and
            // dropped, and the WAIT that follows reports the lack of a reply.
            CMD_SEND_DATA | CMD_SEND_DATAW | CMD_WAIT | CMD_RTX_WAIT => Ok(0),

            // An unknown command is rejected rather than quietly acknowledged.
            // A game that gets an ack for something we did not do waits
            // forever for the effect.
            _ => Err(1),
        }
    }

    /// A device id for hosting. Real adapters have one burned in; any non-zero
    /// value will do, and peers compare the whole half-word.
    fn new_devid(&self) -> u16 {
        // Derived from how much the game has already said to us, so a replay
        // of the same ROM is reproducible rather than random.
        0x1000 | ((self.commands as u16) & 0x0FFF) | 1
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
    fn unknown_commands_are_rejected_not_acknowledged() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0099); // no such command
        assert_eq!(rfu.transfer(0), 0x9966_01EE, "error frame");
        assert_eq!(rfu.transfer(0), 1, "error code");
        assert_eq!(rfu.transfer(0), IDLE, "and back to command wait");
    }

    #[test]
    fn hosting_is_entered_and_left() {
        let mut rfu = Rfu::new();
        handshake(&mut rfu);
        rfu.transfer(0x9966_0019); // host start
        assert_eq!(rfu.state_name(), "hosting");
        assert_ne!(rfu.devid, 0, "a host needs a device id");
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
