//! gba-core: a clean-room Game Boy Advance emulator, written from scratch in
//! Rust.
//!
//! The CPU ([`cpu::Arm7tdmi`]) is validated against the TomHarte ARM7TDMI
//! vectors through the [`bus::Bus`] trait. [`GbaBus`] is the real system bus it
//! drives; [`Gba`] wires the two together and runs frames.

pub mod bus;
pub mod cpu;
pub mod memory;
pub mod ppu;
pub mod save;
pub mod state;

// Coming online next: DMA, timers, IRQ delivery, APU.

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
/// GBA system clock (cycles per second).
const CLOCK: u64 = 16_777_216;
/// Cycles per rendered frame (228 scanlines).
const CYCLES_PER_FRAME: u64 = TOTAL_LINES as u64 * ppu::CYCLES_PER_LINE as u64;

pub struct Gba {
    pub cpu: Arm7tdmi,
    pub bus: GbaBus,
    /// Diagnostics: how many IRQs the CPU has taken since boot.
    pub irqs_taken: u64,
    /// Diagnostics: CPU instructions executed since boot.
    pub steps: u64,
    /// When false, scanline rendering is skipped (for profiling CPU vs PPU).
    pub render_enabled: bool,
    /// Interleaved stereo output samples produced this frame (drained by the
    /// front-end via [`Gba::take_audio`]). Silent for now, but emitted at the
    /// correct rate so an audio-synced libretro host paces us to real time —
    /// without this the host free-runs at the display refresh (double speed).
    audio: Vec<i16>,
    /// Fractional-sample accumulator (in clock-cycle units) for an exact rate.
    sample_error: u64,
}

impl Gba {
    /// Create a console with a cartridge ROM and optional BIOS. With no BIOS the
    /// CPU is booted directly into the cartridge (the post-BIOS register state).
    pub fn new(rom: Vec<u8>, bios: Vec<u8>) -> Self {
        let has_bios = !bios.is_empty();
        let mut bus = GbaBus::new(rom, bios);
        let mut cpu = Arm7tdmi::new();

        if has_bios {
            // Reset entry: Supervisor mode at the reset vector, IRQ/FIQ masked.
            cpu.load_full(0x13 | (1 << 7) | (1 << 6), [0; 16], [0; 7], [0; 2], [0; 2], [0; 2], [0; 2], [0; 5]);
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
            audio: Vec::new(),
            sample_error: 0,
        }
    }

    /// The audio sample rate reported to the front-end.
    pub const SAMPLE_RATE: u32 = SAMPLE_RATE as u32;

    /// Drain this frame's interleaved-stereo samples for the front-end.
    pub fn take_audio(&mut self) -> Vec<i16> {
        std::mem::take(&mut self.audio)
    }

    /// Serialize the whole machine to a save-state blob. The ROM and BIOS are
    /// not included (the front-end already has them); everything else is.
    pub fn save_state(&self) -> Vec<u8> {
        let mut w = state::Writer::default();
        w.u32(state::MAGIC);
        w.u8(state::VERSION);
        self.cpu.serialize(&mut w);
        self.bus.serialize(&mut w);
        w.u64(self.sample_error);
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
        self.sample_error = r.u64();
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
                    self.cpu.take_irq(&mut self.bus);
                    self.bus.halted = false;
                    self.irqs_taken += 1;
                }
                if self.bus.halted {
                    // Parked waiting for an interrupt: skip to the end of the line
                    // (the next scanline may raise the IRQ that wakes us).
                    self.bus.cycles = target;
                    break;
                }
                self.bus.cur_pc = self.cpu.r[15];
                self.cpu.step(&mut self.bus);
                self.steps += 1;
            }
            if line < SCREEN_H as u32 && self.render_enabled {
                self.bus.ppu.render_line(line as usize);
            }
        }

        // Emit this frame's audio at exactly SAMPLE_RATE (integer accumulator):
        // samples_this_frame = SAMPLE_RATE * CYCLES_PER_FRAME / CLOCK, carrying
        // the remainder so the long-run average is exact. Silent until the APU
        // exists — the point right now is pacing the host.
        self.sample_error += SAMPLE_RATE * CYCLES_PER_FRAME;
        let n = self.sample_error / CLOCK;
        self.sample_error %= CLOCK;
        self.audio.extend(std::iter::repeat(0).take(n as usize * 2)); // stereo

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
