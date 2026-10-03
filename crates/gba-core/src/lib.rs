// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! gba-core: a clean-room Game Boy Advance emulator, written from scratch in
//! Rust.
//!
//! The CPU ([`cpu::Arm7tdmi`]) is validated against the TomHarte ARM7TDMI
//! vectors through the [`bus::Bus`] trait. [`GbaBus`] is the real system bus it
//! drives; [`Gba`] wires the two together and runs frames.

pub mod apu;
pub mod bus;
pub mod cpu;
pub mod memory;
pub mod ppu;
pub mod rfu;
pub mod rtc;
pub mod save;
pub mod sensor;
pub mod state;

pub use memory::GbaBus;
pub use ppu::{Ppu, SCREEN_H, SCREEN_W, TOTAL_LINES};

use cpu::Arm7tdmi;

/// GBA buttons, in KEYINPUT bit order.
#[derive(Clone, Copy)]
pub enum Button {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Right = 4,
    Left = 5,
    Up = 6,
    Down = 7,
    R = 8,
    L = 9,
}

/// A whole console: the CPU plus the system bus.
/// Audio output rate. The GBA's own sound hardware runs at 32768 Hz; the value
/// only needs to match [`Gba::SAMPLE_RATE`] and the AV-info the front-end reads.
const SAMPLE_RATE: u64 = 32_768;

pub struct Gba {
    pub cpu: Arm7tdmi,
    pub bus: GbaBus,
    /// Diagnostics: how many IRQs the CPU has taken since boot.
    pub irqs_taken: u64,
    /// Diagnostics: CPU instructions executed since boot.
    pub steps: u64,
    /// Diagnostics: how many of `steps` were executed inside the 16 KB BIOS.
    ///
    /// A cartridge that crashes and restarts through the BIOS looks perfectly
    /// healthy to any colour-based liveness check, because the boot animation
    /// draws and animates: Maniac Racers Advance sits on the Normmatt logo
    /// forever and scores 16 distinct colours, the same as a game resting on a
    /// dark title screen. This says WHO is executing, which a colour count
    /// cannot. A healthy cartridge enters the BIOS only for SWIs.
    pub bios_steps: u64,
    /// Diagnostics: cycles the CPU spent halted, i.e. the game had finished its
    /// work for that stretch and was waiting for an interrupt.
    ///
    /// This is a SPEED HEADROOM measure and the only one that scales past a
    /// handful of titles. A game that keeps up finishes its frame and idles; a
    /// game the emulated CPU is starving never idles at all. Finding Mario Kart
    /// running at half speed needed the owner's save state and an on-screen race
    /// timer, which does not generalise to thousands of ROMs.
    ///
    /// Read it as a screen, not a verdict: a game that busy-polls DISPSTAT
    /// instead of halting shows zero idle while being perfectly healthy.
    pub halt_cycles: u64,
    /// When false, scanline rendering is skipped (for profiling CPU vs PPU).
    pub render_enabled: bool,
    /// A fixed-rate audio clock: it advances by exactly one scanline's worth of
    /// cycles per rendered line, independent of `bus.cycles` (which our timing
    /// model inflates whenever DMA runs). Direct Sound is paced off this so a
    /// heavy graphics-DMA frame can't over-pump the FIFO and run the sample-buffer
    /// pointer off the end (which played garbage during screen transitions).
    audio_clock: u64,
    /// Debug: when set, `run_frame` reports the first time the CPU executes from
    /// unused address space (>= 0x10000000 = definitely a crashed/runaway PC),
    /// printing the branch that jumped there. Off by default (one cheap compare
    /// per instruction when enabled, nothing otherwise).
    pub trap_unused: bool,
    /// Debug: print the executing PC at the top of every scanline this frame.
    pub linepc: bool,
    trap_prev: u32,
    trap_from: u32,
    trapped: bool,
    /// The last 64 taken branches before the trap fired, oldest first. A runaway
    /// PC is never interesting at the point it lands: the question is always what
    /// called what to get there, and the single "prev branch from" address above
    /// answers it only when the jump is one hop deep. GTA Advance took four.
    pub trap_ring: Vec<(u32, u32)>,
}

impl Gba {
    /// Create a console with a cartridge ROM and optional BIOS. With no BIOS the
    /// CPU is booted directly into the cartridge (the post-BIOS register state).
    pub fn new(rom: Vec<u8>, bios: Vec<u8>) -> Self {
        let has_bios = !bios.is_empty();
        let mut bus = GbaBus::new(rom, bios);
        let mut cpu = Arm7tdmi::new();

        if has_bios && std::env::var_os("GBA_FULLBOOT").is_some() {
            // Full BIOS boot (reset vector, Supervisor, IRQ/FIQ masked): runs the
            // whole BIOS boot sequence + logo. Opt-in via GBA_FULLBOOT — a few
            // titles depend on the boot-time state it sets up (Pitfall Mayan
            // Adventure, Super Robot Taisen A, ...) that fast-boot skips.
            cpu.load_full(0x13 | (1 << 7) | (1 << 6), [0; 16], [0; 7], [0; 2], [0; 2], [0; 2], [0; 2], [0; 5]);
        } else if has_bios {
            // Fast/direct boot WITH the BIOS still loaded (DEFAULT): jump straight
            // to the cartridge with post-BIOS register state, skipping the BIOS
            // boot animation, but leave the BIOS image in place so SWIs run the
            // BIOS's own code and BIOS-ROM reads work. gpSP-style — no ~2 s boot
            // logo, and it rescues more games than full-boot (incl. titles that
            // loop the boot). hle_bios stays false.
            let mut r = [0u32; 16];
            r[13] = 0x0300_7F00;
            r[15] = 0x0800_0000;
            let svc = [0x0300_7FE0, 0];
            let irq = [0x0300_7FA0, 0];
            cpu.load_full(0x1F, r, [0; 7], svc, [0, 0], irq, [0; 2], [0; 5]);
        } else {
            // Direct boot: System mode at the cartridge entry, standard stacks.
            let mut r = [0u32; 16];
            r[13] = 0x0300_7F00; // user/system SP
            r[15] = 0x0800_0000; // cartridge entry
            let svc = [0x0300_7FE0, 0]; // SP_svc
            let irq = [0x0300_7FA0, 0]; // SP_irq
            cpu.load_full(0x1F, r, [0; 7], svc, [0, 0], irq, [0; 2], [0; 5]);
            cpu.hle_bios = true; // no real BIOS: emulate SWIs
        }
        cpu.reload_pipeline(&mut bus);

        Gba {
            cpu,
            bus,
            irqs_taken: 0,
            steps: 0,
            bios_steps: 0,
            halt_cycles: 0,
            render_enabled: true,
            audio_clock: 0,
            trap_unused: false,
            linepc: false,
            trap_prev: 0,
            trap_from: 0,
            trapped: false,
            trap_ring: Vec::new(),
        }
    }

    /// The audio sample rate reported to the front-end.
    pub const SAMPLE_RATE: u32 = SAMPLE_RATE as u32;

    /// Drain this frame's interleaved-stereo samples for the front-end.
    pub fn take_audio(&mut self) -> Vec<i16> {
        self.bus.apu.take_output()
    }

    /// Serialize the whole machine to a save-state blob. The ROM and BIOS are
    /// not included (the front-end already has them); everything else is.
    pub fn save_state(&self) -> Vec<u8> {
        let mut w = state::Writer::default();
        w.u32(state::MAGIC);
        w.u8(state::VERSION);
        self.cpu.serialize(&mut w);
        self.bus.serialize(&mut w);
        w.buf
    }

    /// Restore a save-state produced by [`save_state`] on the same cartridge.
    /// Returns false if the blob is not a valid state for this build.
    pub fn load_state(&mut self, data: &[u8]) -> bool {
        let mut r = state::Reader::new(data);
        if r.u32() != state::MAGIC || r.u8() != state::VERSION {
            return false;
        }
        self.cpu.deserialize(&mut r);
        self.bus.deserialize(&mut r);
        // Align the fixed audio clock to the APU's restored position (its own
        // clock domain, independent of bus.cycles) so audio resumes seamlessly
        // and deterministically. A pre-APU state leaves the APU at 0, which is
        // fine: audio simply restarts from 0 rather than bursting a backlog.
        self.audio_clock = self.bus.apu.cycle();
        !r.failed
    }

    /// Run one full frame (228 scanlines) and return the RGB555 framebuffer.
    pub fn run_frame(&mut self) -> &[u16] {
        for line in 0..TOTAL_LINES {
            if line == 0 {
                self.bus.frame_no = self.bus.frame_no.wrapping_add(1);
                // The adapter re-announces a hosted room about twice a second
                // and ages out peers it has stopped hearing from. Driven from
                // here rather than from the front-end so the offline runner
                // behaves the same way.
                if let Some(rfu) = self.bus.rfu.as_mut() {
                    rfu.frame_update();
                }
            }
            self.bus.line_cycle_base = self.bus.cycles;
            self.bus.audio_line_base = self.audio_clock;
            self.bus.ppu.begin_line(line);
            if self.linepc {
                let back = if self.cpu.thumb() { 4 } else { 8 };
                eprintln!("LINEPC L{line:<3} pc={:08X} m{:X} halted={}",
                    self.cpu.r[15].wrapping_sub(back), self.cpu.cpsr & 0xF, self.bus.halted as u8);
            }
            self.bus.raise_ppu_irqs(line as u16);
            self.bus.step_timers();
            self.bus.step_serial();
            if line == 160 {
                self.bus.trigger_dma(1); // V-blank DMA
            }
            if line < SCREEN_H as u32 {
                self.bus.trigger_dma(2); // H-blank DMA (approximate timing)
            }
            // Each scanline runs in two phases so DISPSTAT's H-blank status bit
            // is set for the part of the line that actually is H-blank. Running
            // all 1232 cycles in one go left bit 1 reading 0 always, and any
            // game that waits on H-blank by polling DISPSTAT spun there forever.
            let line_start = self.bus.cycles;
            for phase in 0..2 {
                let target = line_start
                    + if phase == 0 { ppu::HDRAW_CYCLES as u64 } else { ppu::CYCLES_PER_LINE as u64 };
                if phase == 1 {
                    self.bus.ppu.enter_hblank();
                }
                while self.bus.cycles < target {
                    // Take a pending IRQ at the instruction boundary; taking one also
                    // wakes the CPU from a HLE Halt / IntrWait.
                    if self.bus.irq_pending() && self.cpu.irq_ready() {
                        let pend = self.bus.ie & self.bus.if_;
                        for b in 0..16 {
                            if pend & (1 << b) != 0 {
                                self.bus.dbg_irq_src[b] += 1;
                            }
                        }
                        if self.linepc {
                            eprintln!("IRQTAKE L{line} cyc_in_line={} pend={:04X} from={:08X}",
                                self.bus.cycles - self.bus.line_cycle_base, pend,
                                self.cpu.r[15].wrapping_sub(if self.cpu.thumb() {4} else {8}));
                        }
                        self.cpu.take_irq(&mut self.bus);
                        self.bus.halted = false;
                        self.irqs_taken += 1;
                    }
                    if self.trap_unused && !self.trapped {
                        let back = if self.cpu.thumb() { 4 } else { 8 };
                        let exec = self.cpu.r[15].wrapping_sub(back);
                        let width = if self.cpu.thumb() { 2 } else { 4 };
                        if exec >= 0x1000_0000 {
                            eprintln!(
                                "TRAP: PC jumped into unused space: {:08X} -> {exec:08X} (prev branch from {:08X}, irqs={})",
                                self.trap_prev, self.trap_from, self.irqs_taken
                            );
                            let r = &self.cpu.r;
                            eprintln!(
                                "TRAP: r0-r7  {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
                                r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]
                            );
                            eprintln!(
                                "TRAP: r8-r15 {:08X} {:08X} {:08X} {:08X} {:08X} sp={:08X} lr={:08X} pc={:08X}",
                                r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15]
                            );
                            eprintln!("TRAP: the {} branches that led there:", self.trap_ring.len());
                            for (from, to) in self.trap_ring.iter() {
                                eprintln!("TRAP:   {from:08X} -> {to:08X}");
                            }
                            self.trapped = true;
                        } else {
                            if self.trap_prev != 0 && exec != self.trap_prev.wrapping_add(width) {
                                self.trap_from = self.trap_prev; // last in-range branch source
                                self.trap_ring.push((self.trap_prev, exec));
                                if self.trap_ring.len() > 64 {
                                    self.trap_ring.remove(0);
                                }
                            }
                            self.trap_prev = exec;
                        }
                    }
                    // Leaving Halt is NOT the same condition as taking an
                    // interrupt: hardware wakes on any enabled-and-requested
                    // interrupt, whether or not IME lets the CPU vector to the
                    // handler. Gating the wake on IME would park a game forever
                    // the moment it halted with interrupts masked.
                    if self.bus.halted && (self.bus.ie & self.bus.if_) != 0 {
                        self.bus.halted = false;
                    }
                    if self.bus.halted {
                        // Parked waiting for an interrupt: skip to the end of the line
                        // (the next scanline may raise the IRQ that wakes us).
                        self.halt_cycles += target.saturating_sub(self.bus.cycles);
                        self.bus.cycles = target;
                        break;
                    }
                    // Debug watchpoint bookkeeping (zero-cost unless a watch is set):
                    // the executing instruction sits two fetches behind R15.
                    if self.bus.watch_addr != 0 {
                        let back = if self.cpu.thumb() { 4 } else { 8 };
                        self.bus.cur_pc = self.cpu.r[15].wrapping_sub(back);
                        self.bus.cur_mode = self.cpu.cpsr & 0xF;
                    }
                    // One compare on the hot path, with the pipeline offset folded
                    // into the bound: R15 runs 8 bytes (ARM) or 4 (Thumb) ahead of
                    // the instruction being executed, so R15 < 0x4008 is exactly
                    // "executing inside the BIOS" for ARM and over-counts Thumb by
                    // at most the four bytes either side of the BIOS ceiling.
                    if self.cpu.r[15] < 0x4008 {
                        self.bios_steps += 1;
                    }
                    self.cpu.step(&mut self.bus);
                    self.steps += 1;
                }
            }
            if line < SCREEN_H as u32 && self.render_enabled {
                self.bus.ppu.render_line(line as usize);
            }
            // Generate this line's audio (one stereo sample per 512 cycles) from
            // the APU state as it stands after the line's CPU work, then let the
            // sound DMA top up any Direct Sound FIFO that has drained. Paced off
            // the fixed audio clock, not bus.cycles, so DMA can't inflate the rate.
            self.audio_clock = self.bus.audio_line_base + ppu::CYCLES_PER_LINE as u64;
            let periods = [self.bus.ds_timer_period(0), self.bus.ds_timer_period(1)];
            self.bus.apu.generate(periods, self.audio_clock);
            self.bus.refill_fifos();
        }

        &self.bus.ppu.framebuffer
    }

    /// Which motion sensor the cartridge carries, if any. The front-end asks
    /// this so it only turns the device's sensors on for a game that reads them.
    pub fn cart_sensor(&self) -> crate::sensor::CartSensor {
        self.bus.sensors.kind
    }

    /// Feed the accelerometer, in m/s^2. Used by the tilt carts (Yoshi
    /// Topsy-Turvy, Yoshi's Universal Gravitation, Koro Koro Puzzle).
    pub fn set_accelerometer(&mut self, x: f32, y: f32, z: f32) {
        self.bus.sensors.set_accelerometer(x, y, z);
    }

    /// True when the cartridge carries a real-time clock, so a front-end knows
    /// whether it needs to push the wall clock at all.
    pub fn cart_has_rtc(&self) -> bool {
        self.bus.sensors.rtc.present
    }

    /// True when the cartridge has a rumble motor, so a front-end knows whether
    /// to ask for the host's rumble interface at all.
    pub fn cart_has_rumble(&self) -> bool {
        self.bus.sensors.has_rumble
    }

    /// Is the cartridge driving its motor right now? Read once per frame and
    /// hand to the host; the game toggles it far faster than a phone's motor
    /// can follow, so this is a level, not an event.
    pub fn rumble_on(&mut self) -> bool {
        self.bus.sensors.take_rumble()
    }

    /// Push the wall clock, in seconds since the Unix epoch, **already shifted
    /// into the player's local time**.
    ///
    /// The conversion is the front-end's job on purpose: this core carries no
    /// timezone database and should not grow one, while the Android side
    /// already knows the user's zone. Push it every frame; a cartridge clock
    /// latches on demand and a stale value shows up as a game whose day never
    /// turns.
    pub fn set_rtc_unix_time(&mut self, secs: i64) {
        self.bus.sensors.rtc.unix_time = secs;
    }

    /// Diagnostics: how many times the game has latched the clock. Zero after a
    /// run separates "the clock is wrong" from "the game never asked", which
    /// look the same from outside.
    pub fn rtc_latches(&self) -> u64 {
        self.bus.sensors.rtc.latches
    }

    /// Is a wireless adapter attached to this cartridge?
    pub fn cart_has_rfu(&self) -> bool {
        self.bus.rfu.is_some()
    }

    /// Commands the game has sent the adapter, times it has power-cycled it,
    /// and where the adapter currently is. Same purpose as `rtc_latches`:
    /// zero commands separates "the adapter is wrong" from "the game never
    /// asked", and resets climbing while commands stay put is specifically a
    /// handshake that never completes.
    pub fn rfu_stats(&self) -> Option<(u64, u64, &'static str)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.commands, r.resets, r.state_name()))
    }

    /// Tell the adapter which peer the frontend thinks we are. The device id
    /// it advertises is derived from this, so without it two devices running
    /// the same ROM advertise the same id and refuse to join each other.
    pub fn rfu_set_self_id(&mut self, id: u16) {
        if let Some(rfu) = self.bus.rfu.as_mut() {
            rfu.set_self_id(id);
        }
    }

    /// Hand the adapter a packet that arrived from `from`, the frontend id of
    /// the peer that sent it.
    pub fn rfu_net_receive(&mut self, buf: &[u8], from: u16) {
        if let Some(rfu) = self.bus.rfu.as_mut() {
            rfu.net_receive(buf, from);
        }
    }

    /// Take the packets the adapter wants put on the wire.
    pub fn rfu_take_outbox(&mut self) -> Vec<crate::rfu::OutPacket> {
        match self.bus.rfu.as_mut() {
            Some(rfu) => rfu.take_outbox(),
            None => Vec::new(),
        }
    }

    /// Is the adapter hosting a room or attached to one?
    pub fn rfu_in_session(&self) -> bool {
        self.bus.rfu.as_ref().is_some_and(|r| r.in_session())
    }

    /// Peers heard from, sessions formed, and packets dropped for want of
    /// queue space. A drop in the middle of a trade shifts every later block,
    /// so it is counted rather than swallowed.
    pub fn rfu_session_stats(&self) -> Option<(u64, u64, u64)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.peers_seen, r.connections, r.dropped))
    }

    /// Bitmasks of every command code the game has issued, and of the ones we
    /// do not implement. Two numbers that say exactly which of the protocol a
    /// game drives, which cannot be learned any other way on a device.
    pub fn rfu_command_masks(&self) -> Option<(u64, u64)> {
        self.bus.rfu.as_ref().map(|r| (r.cmd_seen, r.unknown_seen))
    }

    /// Connection accounting: asked, no-peer, sent, nacked, requests received,
    /// requests refused. "The connect failed" has several distinct causes that
    /// need opposite fixes, and none of them errors on a device.
    /// What the adapter reported while holding the clock (timeout, event,
    /// disconnection) and how many blocks actually crossed the air.
    /// Clients attached right now, peers heard right now, and why clients
    /// have left: added, timed out, told to go, wiped by a restart.
    pub fn rfu_client_life(&self) -> Option<(u32, u32, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            let (clients, peers) = r.live_counts();
            (clients, peers, r.cl_added, r.cl_timeout, r.cl_told, r.cl_wiped)
        })
    }

    pub fn rfu_wait_stats(&self) -> Option<(u64, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            (r.resp_timeout, r.resp_data, r.resp_disc, r.blocks_out, r.blocks_in)
        })
    }

    pub fn rfu_connect_stats(&self) -> Option<(u64, u64, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            (
                r.conn_asked,
                r.conn_no_peer,
                r.conn_sent,
                r.conn_nacked,
                r.req_got,
                r.req_refused,
            )
        })
    }

    /// Feed the gyroscope, in rad/s. Used by WarioWare Twisted, which senses
    /// rotation about Z only.
    pub fn set_gyroscope(&mut self, x: f32, y: f32, z: f32) {
        self.bus.sensors.set_gyroscope(x, y, z);
    }

    /// Set a button's pressed state (KEYINPUT is active-low).
    pub fn set_button(&mut self, button: Button, pressed: bool) {
        let bit = 1u16 << (button as u16);
        if pressed {
            self.bus.keyinput &= !bit;
        } else {
            self.bus.keyinput |= bit;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Every single-register load spends one internal cycle moving the loaded
    /// value into the register file (LDR = 1S+1N+1I on the ARM7TDMI). Run the
    /// same Thumb program from IWRAM twice, once with eight loads and once
    /// with eight register adds, and the loads must cost exactly their data
    /// access (1 cycle in IWRAM) plus that internal cycle more. IWRAM code has
    /// no sequential/non-sequential distinction, so this isolates the I-cycle.
    #[test]
    fn a_load_costs_one_internal_cycle_on_top_of_its_data_access() {
        use crate::bus::{Access, Bus};
        fn run(body_op: u16) -> u64 {
            use crate::bus::{Access, Bus};
            // ROM (ARM): ldr r0, [pc]; bx r0  -> Thumb code at 0x03000005.
            let mut rom = vec![0u8; 0x100];
            for (i, w) in [0xE59F_0000u32, 0xE12F_FF10, 0x0300_0005].iter().enumerate() {
                rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            let mut gba = Gba::new(rom, Vec::new());
            // Thumb at 0x03000004: ldr r2, =0x03000100 ; 8x body ; b .
            let prog: [u16; 11] = [
                0x4A06, // ldr r2, [pc, #0x18]  (literal at 0x03000020)
                body_op, body_op, body_op, body_op, body_op, body_op, body_op, body_op,
                0xE7FE, // b .
                0x0000,
            ];
            for (i, h) in prog.iter().enumerate() {
                let a = 0x0300_0004 + i as u32 * 2;
                gba.bus.write16(a, *h, Access::NonSeq);
            }
            gba.bus.write32(0x0300_0020, 0x0300_0100, Access::NonSeq);
            let setup_cycles = gba.bus.cycles;
            // 2 ARM instructions + ldr r2 + 8 body instructions = 11 steps.
            for _ in 0..11 {
                gba.cpu.step(&mut gba.bus);
            }
            gba.bus.cycles - setup_cycles
        }
        let loads = run(0x6811); // ldr r1, [r2]
        let adds = run(0x1C11); // adds r1, r2, #0
        assert_eq!(
            loads - adds,
            8 * (1 + 1),
            "each load must pay its 1-cycle IWRAM data access plus 1 internal cycle"
        );
    }

    #[test]
    fn save_state_roundtrip_is_deterministic() {
        // A small ROM: enough for the CPU to run deterministically for a while.
        let rom = vec![0u8; 0x2000];
        let mut a = Gba::new(rom.clone(), Vec::new());
        for _ in 0..8 {
            a.run_frame();
        }
        let blob = a.save_state();

        // Restore into a fresh machine and confirm it resumes identically.
        let mut b = Gba::new(rom, Vec::new());
        assert!(b.load_state(&blob), "state should load");
        assert_eq!(a.cpu.r, b.cpu.r, "registers restored");
        assert_eq!(a.cpu.cpsr, b.cpu.cpsr);
        assert_eq!(a.bus.cycles, b.bus.cycles);

        // Advancing both from the same point stays bit-identical.
        for _ in 0..4 {
            a.run_frame();
            b.run_frame();
        }
        assert_eq!(a.cpu.r, b.cpu.r, "diverged after resume");
        assert_eq!(&a.bus.ppu.framebuffer[..], &b.bus.ppu.framebuffer[..]);

        // A garbage blob is rejected, not panicked on.
        assert!(!b.load_state(&[1, 2, 3]));
    }

    /// The CPU must be able to see DISPSTAT's H-blank flag (bit 1) go high and
    /// come back down within a frame. Games wait on H-blank by polling this bit
    /// as often as by taking the IRQ, and while it was hard-wired to 0 they
    /// spun forever (Konami Krazy Racers never left its boot loop).
    ///
    /// The ROM is six ARM instructions: count the polls where bit 1 is set (r1)
    /// against the total polls (r3). Both bounds matter. r1 == 0 is the bug this
    /// fixes; r1 == r3 would be a flag stuck on, which breaks the other half of
    /// the games (the ones that wait for H-blank to *end*).
    #[test]
    fn hblank_flag_is_visible_to_the_cpu_and_clears_again() {
        let code: [u32; 8] = [
            0xE3A0_0404, // mov  r0, #0x04000000
            0xE3A0_1000, // mov  r1, #0            ; polls that saw H-blank
            0xE3A0_3000, // mov  r3, #0            ; polls total
            0xE1D0_20B4, // ldrh r2, [r0, #4]      ; DISPSTAT
            0xE312_0002, // tst  r2, #2
            0x1281_1001, // addne r1, r1, #1
            0xE283_3001, // add  r3, r3, #1
            0xEAFF_FFFA, // b    -> the ldrh
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let mut gba = Gba::new(rom, Vec::new());
        gba.run_frame();

        let (seen, total) = (gba.cpu.r[1], gba.cpu.r[3]);
        assert!(total > 0, "the poll loop should have run at all");
        assert!(seen > 0, "H-blank flag never read as set in a whole frame");
        assert!(seen < total, "H-blank flag never read as clear: stuck on");
    }

    /// Build a ROM that divides `number` by zero through SWI 06h and then parks.
    fn div_by_zero_rom(number: u32) -> Vec<u8> {
        let code: [u32; 4] = [
            0xE3A0_0000 | number, // mov r0, #number
            0xE3A0_1000,          // mov r1, #0
            0xEF06_0000,          // swi 0x06        Div
            0xEAFF_FFFE,          // b .
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        rom
    }

    /// A stand-in BIOS in which every vector is an infinite loop. Any SWI that
    /// actually reaches 0x08 never returns, so "did the core answer this call
    /// itself?" becomes observable without shipping a real BIOS image into the
    /// test.
    fn hanging_bios() -> Vec<u8> {
        0xEAFF_FFFEu32.to_le_bytes().repeat(0x4000 / 4)
    }

    /// Measured against a real BIOS dump: 0/0 returns r0=1, r1=0, r3=1, and
    /// every other divide by zero hangs the BIOS forever. The bundled open BIOS
    /// hangs on 0/0 as well, which is what kept Blackthorne and the rest of the
    /// Blizzard/Interplay ports from booting in the shipping configuration.
    ///
    /// Both bounds are checked. 0/0 must be answered without reaching the
    /// vector, and N/0 must still reach it, because widening the interception
    /// to N/0 would invent a result the console never produces.
    #[test]
    fn zero_over_zero_is_answered_without_entering_the_bios() {
        for bios in [Vec::new(), hanging_bios()] {
            let mut gba = Gba::new(div_by_zero_rom(0), bios.clone());
            for _ in 0..2 {
                gba.run_frame();
            }
            let tag = if bios.is_empty() { "HLE" } else { "BIOS" };
            assert_eq!(gba.cpu.r[0], 1, "{tag}: 0/0 quotient");
            assert_eq!(gba.cpu.r[1], 0, "{tag}: 0/0 remainder");
            assert_eq!(gba.cpu.r[3], 1, "{tag}: 0/0 abs quotient");
            assert!(
                gba.cpu.r[15] >= 0x0800_0000,
                "{tag}: should have returned to the cartridge, not parked in the BIOS"
            );
        }
    }

    /// A stand-in BIOS whose every vector returns immediately without touching
    /// r0-r3. It models the defect exactly: a BIOS that runs the call and
    /// leaves r3 alone.
    fn returning_bios() -> Vec<u8> {
        0xE1B0_F00Eu32.to_le_bytes().repeat(0x4000 / 4) // movs pc, lr
    }

    /// Div returns three registers, not two: r0 = quotient, r1 = remainder and
    /// r3 = abs(quotient). Measured 100/7 against a real BIOS dump: 14, 2, 14.
    ///
    /// r3 is the one the bundled open BIOS forgets, and forgetting it is not
    /// visibly wrong: r0 and r1 are right, so the game computes correctly until
    /// it uses r3, and then it has whatever it was holding. Puppy Luv puts that
    /// stale value in its interrupt-handler table and branches to it.
    ///
    /// Note what this does NOT assert. r0 stays 100 here, because the stand-in
    /// BIOS never divides and we deliberately do not divide for it: the real
    /// BIOS is left to do the arithmetic and spend the cycles, and only r3 is
    /// seeded. Taking the whole call over instead was measured and cost Happy
    /// Feet its title screen in both regions, so "we did not compute r0" is a
    /// property worth pinning rather than an omission.
    #[test]
    fn divide_seeds_r3_without_taking_over_the_call() {
        let code: [u32; 9] = [
            0xE3A0_0064, // mov r0, #100
            0xE3A0_1007, // mov r1, #7
            0xE3A0_30FF, // mov r3, #0xFF     sentinel: untouched means not set
            0xEF06_0000, // swi 0x06          Div
            0xE3A0_4403, // mov r4, #0x03000000
            0xE584_0000, // str r0, [r4]
            0xE584_1004, // str r1, [r4, #4]
            0xE584_3008, // str r3, [r4, #8]
            0xEAFF_FFFE, // b .
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let mut gba = Gba::new(rom, returning_bios());
        for _ in 0..2 {
            gba.run_frame();
        }
        assert_eq!(gba.cpu.r[3], 14, "r3 must be seeded with abs(quotient)");
        assert_eq!(gba.cpu.r[0], 100, "the divide itself is left to the BIOS");
    }

    #[test]
    fn nonzero_over_zero_still_reaches_the_bios() {
        let mut gba = Gba::new(div_by_zero_rom(5), hanging_bios());
        for _ in 0..2 {
            gba.run_frame();
        }
        assert!(
            gba.cpu.r[15] < 0x0800_0000,
            "N/0 must not be intercepted: hardware hangs here, so we must too"
        );
    }

    /// `bios_steps` must count BIOS instructions and ONLY BIOS instructions.
    ///
    /// This is the signal that tells a crashed cartridge apart from a working
    /// one, and a colour count cannot: Maniac Racers Advance sits on the
    /// Normmatt boot logo forever and scores the same 16 distinct colours as a
    /// game on a dark title screen. Both halves are asserted separately, so a
    /// bound that is too wide (counting cartridge code) or an increment that
    /// never fires both fail.
    #[test]
    fn bios_steps_counts_the_bios_and_nothing_else() {
        // A stand-in BIOS: an endless loop parked at the SWI vector, so a
        // cartridge that calls a SWI never comes back and every instruction
        // after that one is BIOS.
        let mut bios = vec![0u8; 0x4000];
        bios[8..12].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes()); // 0x08: b .

        // Cartridge A never leaves its own code: `b .` at the entry point.
        let mut rom = vec![0u8; 0x2000];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let mut a = Gba::new(rom, bios.clone());
        a.run_frame();
        assert!(a.steps > 1000, "the cartridge should have run, got {}", a.steps);
        assert_eq!(
            a.bios_steps, 0,
            "a cartridge that never calls a SWI must not count as BIOS"
        );

        // Cartridge B vectors into the BIOS on its first instruction and stays.
        let mut rom = vec![0u8; 0x2000];
        rom[0..4].copy_from_slice(&0xEF00_0000u32.to_le_bytes()); // swi #0
        let mut b = Gba::new(rom, bios);
        b.run_frame();
        assert!(
            b.bios_steps > 1000,
            "a cartridge stuck in the BIOS must count as BIOS, got {} of {}",
            b.bios_steps,
            b.steps
        );
        assert!(
            b.bios_steps < b.steps,
            "the cartridge ran its own SWI first, so not every step is BIOS"
        );
    }
}