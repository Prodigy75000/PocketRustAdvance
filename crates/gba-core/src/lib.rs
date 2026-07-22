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
pub struct Gba {
    pub cpu: Arm7tdmi,
    pub bus: GbaBus,
    /// Diagnostics: how many IRQs the CPU has taken since boot.
    pub irqs_taken: u64,
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

        Gba { cpu, bus, irqs_taken: 0 }
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
                self.cpu.step(&mut self.bus);
            }
            if line < SCREEN_H as u32 {
                self.bus.ppu.render_line(line as usize);
            }
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
