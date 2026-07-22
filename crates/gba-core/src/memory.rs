//! The real GBA memory bus: the 8 regions, their mirroring, cycle costs, and
//! I/O dispatch. Implements [`crate::bus::Bus`] so the CPU drives it exactly as
//! it drove the test harness. Region layout and mirroring follow GBATEK.

use crate::bus::{Access, Bus};
use crate::ppu::Ppu;

pub struct GbaBus {
    pub bios: Box<[u8]>,   // 16 KB
    pub ewram: Box<[u8]>,  // 256 KB
    pub iwram: Box<[u8]>,  // 32 KB
    pub rom: Box<[u8]>,    // up to 32 MB
    pub sram: Box<[u8]>,   // 64 KB
    pub ppu: Ppu,
    /// Catch-all for I/O registers not yet modeled (DMA/timers/IRQ/sound).
    io: Box<[u8]>, // 0x400
    /// KEYINPUT (0x4000130): bits are active-low, 1 = released.
    pub keyinput: u16,
    /// Interrupt controller: enable mask, request flags, master enable.
    pub ie: u16,
    pub if_: u16,
    pub ime: u16,
    /// The four hardware timers, and the cycle count they were last advanced to.
    timers: [Timer; 4],
    timer_cycles: u64,
    /// Latched (running) DMA source/dest/count per channel.
    dma_src: [u32; 4],
    dma_dst: [u32; 4],
    dma_count: [u32; 4],
    /// Free-running cycle counter; drives the frame/scanline pacing.
    pub cycles: u64,
    /// CPU halted (by the HLE Halt / IntrWait SWIs) until the next IRQ.
    pub halted: bool,
}

#[derive(Clone, Copy, Default)]
struct Timer {
    reload: u16,
    counter: u16,
    control: u16,
    subcycle: u32,
}

/// Add `ticks` to a timer counter, reloading on overflow. Returns how many times
/// it overflowed (for cascade + IRQ).
fn timer_add(counter: &mut u16, reload: u16, ticks: u32) -> u32 {
    if ticks == 0 {
        return 0;
    }
    let val = *counter as u32 + ticks;
    if val <= 0xFFFF {
        *counter = val as u16;
        return 0;
    }
    let period = 0x1_0000 - reload as u32; // ticks per overflow after the first
    let extra = val - 0x1_0000;
    *counter = (reload as u32 + extra % period) as u16;
    1 + extra / period
}

/// Minimal HLE stand-in for the BIOS IRQ handler, installed when no real BIOS is
/// provided. It saves scratch registers, loads the user handler pointer from
/// [0x03FFFFFC] (mirror of 0x03007FFC), calls it, restores, and returns.
const HLE_IRQ: &[(usize, u32)] = &[
    (0x18, 0xEA00_0000), // b 0x20
    (0x20, 0xE92D_500F), // stmfd sp!, {r0-r3, r12, lr}
    (0x24, 0xE3A0_0301), // mov r0, #0x04000000
    (0x28, 0xE28F_E000), // add lr, pc, #0
    (0x2C, 0xE510_F004), // ldr pc, [r0, #-4]
    (0x30, 0xE8BD_500F), // ldmfd sp!, {r0-r3, r12, lr}
    (0x34, 0xE25E_F004), // subs pc, lr, #4
];

impl GbaBus {
    pub fn new(rom: Vec<u8>, bios: Vec<u8>) -> Self {
        let mut b = vec![0u8; 16 * 1024];
        let n = bios.len().min(b.len());
        b[..n].copy_from_slice(&bios[..n]);
        if bios.is_empty() {
            for &(off, word) in HLE_IRQ {
                b[off..off + 4].copy_from_slice(&word.to_le_bytes());
            }
        }
        GbaBus {
            bios: b.into_boxed_slice(),
            ewram: vec![0; 256 * 1024].into_boxed_slice(),
            iwram: vec![0; 32 * 1024].into_boxed_slice(),
            rom: rom.into_boxed_slice(),
            sram: vec![0; 64 * 1024].into_boxed_slice(),
            ppu: Ppu::new(),
            io: vec![0; 0x400].into_boxed_slice(),
            keyinput: 0x03FF,
            ie: 0,
            if_: 0,
            ime: 0,
            timers: [Timer::default(); 4],
            timer_cycles: 0,
            dma_src: [0; 4],
            dma_dst: [0; 4],
            dma_count: [0; 4],
            cycles: 0,
            halted: false,
        }
    }

    // --- Timers ---------------------------------------------------------------

    /// Advance all four timers by the cycles elapsed since the last call,
    /// handling prescalers, cascade, overflow reload and timer IRQs.
    pub fn step_timers(&mut self) {
        let delta = (self.cycles - self.timer_cycles) as u32;
        self.timer_cycles = self.cycles;
        let mut prev_overflows = 0u32;
        for ch in 0..4 {
            let ctrl = self.timers[ch].control;
            if ctrl & 0x80 == 0 {
                prev_overflows = 0;
                continue;
            }
            let ticks = if ch > 0 && ctrl & 4 != 0 {
                prev_overflows // cascade / count-up
            } else {
                let period = [1u32, 64, 256, 1024][(ctrl & 3) as usize];
                self.timers[ch].subcycle += delta;
                let t = self.timers[ch].subcycle / period;
                self.timers[ch].subcycle %= period;
                t
            };
            let reload = self.timers[ch].reload;
            let overflows = timer_add(&mut self.timers[ch].counter, reload, ticks);
            if overflows > 0 && ctrl & 0x40 != 0 {
                self.if_ |= 1 << (3 + ch);
            }
            prev_overflows = overflows;
        }
    }

    // --- DMA ------------------------------------------------------------------

    fn io_u16(&self, off: u32) -> u16 {
        u16::from_le_bytes([self.io[off as usize], self.io[off as usize + 1]])
    }
    fn io_u32(&self, off: u32) -> u32 {
        let o = off as usize;
        u32::from_le_bytes([self.io[o], self.io[o + 1], self.io[o + 2], self.io[o + 3]])
    }

    /// Run enabled DMA channels whose start-timing matches (1=VBlank, 2=HBlank).
    pub fn trigger_dma(&mut self, timing: u16) {
        for ch in 0..4 {
            let control = self.io_u16(0xB0 + ch as u32 * 12 + 10);
            if control & 0x8000 != 0 && (control >> 12) & 3 == timing {
                self.run_dma(ch);
            }
        }
    }

    fn run_dma(&mut self, ch: usize) {
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        let word = control & 0x400 != 0;
        let size = if word { 4u32 } else { 2 };
        let dst_ctrl = (control >> 5) & 3;
        let src_ctrl = (control >> 7) & 3;
        let mut src = self.dma_src[ch];
        let mut dst = self.dma_dst[ch];
        let step = |a: u32, c: u16| match c {
            1 => a.wrapping_sub(size),
            2 => a,
            _ => a.wrapping_add(size),
        };
        for _ in 0..self.dma_count[ch] {
            if word {
                let v = self.read32_raw(src & !3);
                self.write(dst & !3, v, 4);
            } else {
                let v = self.read16_raw(src & !1) as u32;
                self.write(dst & !1, v, 2);
            }
            src = step(src, src_ctrl);
            dst = step(dst, dst_ctrl);
            self.cycles += size as u64;
        }
        self.dma_src[ch] = src;
        if control & 0x4000 != 0 {
            self.if_ |= 1 << (8 + ch); // DMA complete IRQ
        }
        let repeat = control & 0x200 != 0 && (control >> 12) & 3 != 0;
        if repeat {
            let cl = self.io_u16(base + 8) as u32;
            let max = if ch == 3 { 0x1_0000 } else { 0x4000 };
            self.dma_count[ch] = if cl == 0 { max } else { cl };
            self.dma_dst[ch] = if dst_ctrl == 3 {
                self.io_u32(base + 4) & 0x0FFF_FFFF // dest reload
            } else {
                dst
            };
        } else {
            self.dma_dst[ch] = dst;
            let cleared = control & 0x7FFF; // clear the enable bit
            self.io[base as usize + 10] = cleared as u8;
            self.io[base as usize + 11] = (cleared >> 8) as u8;
        }
    }

    /// Latch a DMA channel's registers when its enable bit is set, running it
    /// immediately if its timing is "immediate".
    fn start_dma(&mut self, ch: usize) {
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        if control & 0x8000 == 0 {
            return;
        }
        self.dma_src[ch] = self.io_u32(base) & 0x0FFF_FFFF;
        self.dma_dst[ch] = self.io_u32(base + 4) & 0x0FFF_FFFF;
        let cl = self.io_u16(base + 8) as u32;
        let max = if ch == 3 { 0x1_0000 } else { 0x4000 };
        self.dma_count[ch] = if cl == 0 { max } else { cl };
        if (control >> 12) & 3 == 0 {
            self.run_dma(ch); // immediate
        }
    }

    /// True when the CPU should take an IRQ: master-enabled and some enabled
    /// source is requesting. The CPU still gates on its own CPSR I bit.
    pub fn irq_pending(&self) -> bool {
        self.ime & 1 != 0 && (self.ie & self.if_) != 0
    }

    /// Raise the LCD interrupts for `line` according to DISPSTAT's enable bits.
    pub fn raise_ppu_irqs(&mut self, line: u16) {
        let stat = self.ppu.dispstat();
        if line == 160 && stat & 0x08 != 0 {
            self.if_ |= 1 << 0; // V-blank
        }
        if stat & 0x10 != 0 {
            self.if_ |= 1 << 1; // H-blank (every line, approximate timing)
        }
        if line == stat >> 8 && stat & 0x20 != 0 {
            self.if_ |= 1 << 2; // V-counter match
        }
    }

    fn access_cycles(addr: u32, word: bool) -> u64 {
        // Approximate wait-states (refined once WAITCNT + prefetch are modeled).
        match (addr >> 24) & 0xF {
            0x2 => if word { 6 } else { 3 },       // EWRAM (16-bit bus, 2 WS)
            0x5 | 0x6 => if word { 2 } else { 1 }, // PALRAM / VRAM (16-bit bus)
            0x8..=0xD => if word { 8 } else { 5 }, // ROM
            0xE | 0xF => 5,                        // SRAM
            _ => 1,                                // BIOS / IWRAM / I/O / OAM
        }
    }

    // --- Reads: composed little-endian from raw bytes (no side effects) --------

    fn read8_raw(&self, addr: u32) -> u8 {
        match (addr >> 24) & 0xF {
            0x0 => *self.bios.get((addr & 0x3FFF) as usize).unwrap_or(&0),
            0x2 => self.ewram[(addr & 0x3_FFFF) as usize],
            0x3 => self.iwram[(addr & 0x7FFF) as usize],
            0x4 => self.io_read8(addr),
            0x5 => self.ppu.read_pal8(addr),
            0x6 => self.ppu.read_vram8(addr),
            0x7 => self.ppu.read_oam8(addr),
            0x8..=0xD => {
                let o = (addr & 0x01FF_FFFF) as usize;
                *self.rom.get(o).unwrap_or(&0)
            }
            0xE | 0xF => self.sram[(addr & 0xFFFF) as usize],
            _ => 0,
        }
    }

    fn read16_raw(&self, addr: u32) -> u16 {
        u16::from_le_bytes([self.read8_raw(addr), self.read8_raw(addr + 1)])
    }
    fn read32_raw(&self, addr: u32) -> u32 {
        u32::from_le_bytes([
            self.read8_raw(addr),
            self.read8_raw(addr + 1),
            self.read8_raw(addr + 2),
            self.read8_raw(addr + 3),
        ])
    }

    fn io_read8(&self, addr: u32) -> u8 {
        let off = addr & 0x3FF;
        match off {
            0x000..=0x05F => (self.ppu.read_reg16(off & !1) >> ((off & 1) * 8)) as u8,
            0x130 => self.keyinput as u8,
            0x131 => (self.keyinput >> 8) as u8,
            0x100..=0x10F => {
                let ch = ((off - 0x100) / 4) as usize;
                let t = &self.timers[ch];
                match (off - 0x100) % 4 {
                    0 => t.counter as u8,
                    1 => (t.counter >> 8) as u8,
                    2 => t.control as u8,
                    _ => (t.control >> 8) as u8,
                }
            }
            0x200 => self.ie as u8,
            0x201 => (self.ie >> 8) as u8,
            0x202 => self.if_ as u8,
            0x203 => (self.if_ >> 8) as u8,
            0x208 => self.ime as u8,
            0x209 => (self.ime >> 8) as u8,
            _ => self.io[off as usize],
        }
    }

    // --- Writes: width-aware so display-memory quirks are honored --------------

    fn write(&mut self, addr: u32, val: u32, width: u32) {
        match (addr >> 24) & 0xF {
            0x2 => write_le(&mut self.ewram, (addr & 0x3_FFFF) as usize, val, width),
            0x3 => write_le(&mut self.iwram, (addr & 0x7FFF) as usize, val, width),
            0x4 => self.io_write(addr, val, width),
            0x5 => self.display_write(DisplayRegion::Pal, addr, val, width),
            0x6 => self.display_write(DisplayRegion::Vram, addr, val, width),
            0x7 => self.display_write(DisplayRegion::Oam, addr, val, width),
            0xE | 0xF => self.sram[(addr & 0xFFFF) as usize] = val as u8,
            _ => {} // BIOS / ROM are read-only
        }
    }

    fn io_write(&mut self, addr: u32, val: u32, width: u32) {
        let off = addr & 0x3FF;
        for i in 0..width {
            let o = off + i;
            let byte = (val >> (i * 8)) as u8;
            match o {
                0x000..=0x05F => {
                    // Merge the byte into its 16-bit register.
                    let reg = o & !1;
                    let cur = self.ppu.read_reg16(reg);
                    let merged = if o & 1 == 0 {
                        (cur & 0xFF00) | byte as u16
                    } else {
                        (cur & 0x00FF) | ((byte as u16) << 8)
                    };
                    self.ppu.write_reg16(reg, merged);
                }
                0x130 | 0x131 => {} // KEYINPUT read-only
                0x200 => self.ie = (self.ie & 0xFF00) | byte as u16,
                0x201 => self.ie = (self.ie & 0x00FF) | ((byte as u16) << 8),
                // IF is write-1-to-clear (interrupt acknowledge).
                0x202 => self.if_ &= !(byte as u16),
                0x203 => self.if_ &= !((byte as u16) << 8),
                0x208 => self.ime = (self.ime & 0xFF00) | byte as u16,
                0x209 => self.ime = (self.ime & 0x00FF) | ((byte as u16) << 8),
                0x100..=0x10F => {
                    let ch = ((o - 0x100) / 4) as usize;
                    let t = &mut self.timers[ch];
                    match (o - 0x100) % 4 {
                        0 => t.reload = (t.reload & 0xFF00) | byte as u16,
                        1 => t.reload = (t.reload & 0x00FF) | ((byte as u16) << 8),
                        2 => {
                            let was_on = t.control & 0x80 != 0;
                            t.control = (t.control & 0xFF00) | byte as u16;
                            if t.control & 0x80 != 0 && !was_on {
                                t.counter = t.reload; // enable reloads the counter
                                t.subcycle = 0;
                            }
                        }
                        _ => t.control = (t.control & 0x00FF) | ((byte as u16) << 8),
                    }
                }
                _ => {
                    self.io[o as usize] = byte;
                    // A DMA control high byte (enable bit) may start a channel.
                    if matches!(o, 0xBB | 0xC7 | 0xD3 | 0xDF) {
                        self.start_dma(((o - 0xBB) / 12) as usize);
                    }
                }
            }
        }
    }

    fn display_write(&mut self, region: DisplayRegion, addr: u32, val: u32, width: u32) {
        match region {
            DisplayRegion::Pal => {
                if width == 1 {
                    self.ppu.write_pal8(addr, val as u8);
                } else {
                    write_le(&mut self.ppu.palram, (addr & 0x3FF) as usize, val, width);
                }
            }
            DisplayRegion::Vram => {
                if width == 1 {
                    self.ppu.write_vram8(addr, val as u8);
                } else {
                    let i = Ppu::vram_index(addr);
                    write_le(&mut self.ppu.vram, i, val, width);
                }
            }
            DisplayRegion::Oam => {
                if width == 1 {
                    // OAM ignores byte writes.
                } else {
                    write_le(&mut self.ppu.oam, (addr & 0x3FF) as usize, val, width);
                }
            }
        }
    }
}

enum DisplayRegion {
    Pal,
    Vram,
    Oam,
}

/// Write the low `width` bytes of `val` little-endian into `mem` at `off`,
/// wrapping within the slice.
fn write_le(mem: &mut [u8], off: usize, val: u32, width: u32) {
    let len = mem.len();
    for i in 0..width as usize {
        mem[(off + i) % len] = (val >> (i * 8)) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus() -> GbaBus {
        GbaBus::new(vec![0; 0x100], Vec::new())
    }

    #[test]
    fn timer_overflow_raises_irq() {
        let mut b = bus();
        b.write16(0x0400_0100, 0xFFFE, Access::NonSeq); // reload
        b.write16(0x0400_0102, 0x00C0, Access::NonSeq); // enable + IRQ, prescaler 1
        b.timer_cycles = b.cycles; // ignore setup cycles
        b.cycles += 4;
        b.step_timers();
        assert!(b.if_ & (1 << 3) != 0, "timer0 overflow should request IRQ");
        assert_eq!(b.timers[0].counter, 0xFFFE);
    }

    #[test]
    fn dma_immediate_copy() {
        let mut b = bus();
        for i in 0..8u32 {
            b.write32(0x0300_0000 + i * 4, 0x1000 + i, Access::NonSeq);
        }
        b.write32(0x0400_00B0, 0x0300_0000, Access::NonSeq); // SAD
        b.write32(0x0400_00B4, 0x0300_0100, Access::NonSeq); // DAD
        b.write16(0x0400_00B8, 8, Access::NonSeq); // count
        b.write16(0x0400_00BA, 0x8000 | 0x400, Access::NonSeq); // enable, 32-bit, immediate
        for i in 0..8u32 {
            assert_eq!(b.read32(0x0300_0100 + i * 4, Access::NonSeq), 0x1000 + i);
        }
        // Enable bit self-clears after a non-repeating transfer.
        assert_eq!(b.io_u16(0xBA) & 0x8000, 0);
    }
}

impl Bus for GbaBus {
    fn read8(&mut self, addr: u32, _a: Access) -> u8 {
        self.cycles += Self::access_cycles(addr, false);
        self.read8_raw(addr)
    }
    fn read16(&mut self, addr: u32, _a: Access) -> u16 {
        self.cycles += Self::access_cycles(addr, false);
        self.read16_raw(addr)
    }
    fn read32(&mut self, addr: u32, _a: Access) -> u32 {
        self.cycles += Self::access_cycles(addr, true);
        self.read32_raw(addr)
    }
    fn write8(&mut self, addr: u32, val: u8, _a: Access) {
        self.cycles += Self::access_cycles(addr, false);
        self.write(addr, val as u32, 1);
    }
    fn write16(&mut self, addr: u32, val: u16, _a: Access) {
        self.cycles += Self::access_cycles(addr, false);
        self.write(addr, val as u32, 2);
    }
    fn write32(&mut self, addr: u32, val: u32, _a: Access) {
        self.cycles += Self::access_cycles(addr, true);
        self.write(addr, val, 4);
    }
    fn tick(&mut self, n: u32) {
        self.cycles += n as u64;
    }
    fn set_halted(&mut self, halted: bool) {
        self.halted = halted;
    }
}
