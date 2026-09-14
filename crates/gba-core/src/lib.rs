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
pub mod save;
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
    trap_prev: u32,
    trap_from: u32,
    trapped: bool,
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
            render_enabled: true,
            audio_clock: 0,
            trap_unused: false,
            trap_prev: 0,
            trap_from: 0,
            trapped: false,
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
            self.bus.ppu.begin_line(line);
            self.bus.raise_ppu_irqs(line as u16);
            self.bus.step_timers();
            if line == 160 {
                self.bus.trigger_dma(1); // V-blank DMA
            }
            if line < SCREEN_H as u32 {
                self.bus.trigger_dma(2); // H-blank DMA (approximate timing)
            }
            let target = self.bus.cycles + ppu::CYCLES_PER_LINE as u64;
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
                        self.trapped = true;
                    } else {
                        if self.trap_prev != 0 && exec != self.trap_prev.wrapping_add(width) {
                            self.trap_from = self.trap_prev; // last in-range branch source
                        }
                        self.trap_prev = exec;
                    }
                }
                if self.bus.halted {
                    // Parked waiting for an interrupt: skip to the end of the line
                    // (the next scanline may raise the IRQ that wakes us).
                    self.bus.cycles = target;
                    break;
                }
                // Debug watchpoint bookkeeping (zero-cost unless a watch is set):
                // the executing instruction sits two fetches behind R15.
                if self.bus.watch_addr != 0 {
                    let back = if self.cpu.thumb() { 4 } else { 8 };
                    self.bus.cur_pc = self.cpu.r[15].wrapping_sub(back);
                }
                self.cpu.step(&mut self.bus);
                self.steps += 1;
            }
            if line < SCREEN_H as u32 && self.render_enabled {
                self.bus.ppu.render_line(line as usize);
            }
            // Generate this line's audio (one stereo sample per 512 cycles) from
            // the APU state as it stands after the line's CPU work, then let the
            // sound DMA top up any Direct Sound FIFO that has drained. Paced off
            // the fixed audio clock, not bus.cycles, so DMA can't inflate the rate.
            self.audio_clock += ppu::CYCLES_PER_LINE as u64;
            let periods = [self.bus.ds_timer_period(0), self.bus.ds_timer_period(1)];
            self.bus.apu.generate(periods, self.audio_clock);
            self.bus.refill_fifos();
        }

        &self.bus.ppu.framebuffer
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
}