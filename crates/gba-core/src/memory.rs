//! The real GBA memory bus: the 8 regions, their mirroring, cycle costs, and
//! I/O dispatch. Implements [`crate::bus::Bus`] so the CPU drives it exactly as
//! it drove the test harness. Region layout and mirroring follow GBATEK.

use crate::apu::Apu;
use crate::bus::{Access, Bus};
use crate::ppu::Ppu;
use crate::save::Save;

pub struct GbaBus {
    pub bios: Box<[u8]>,   // 16 KB
    pub ewram: Box<[u8]>,  // 256 KB
    pub iwram: Box<[u8]>,  // 32 KB
    pub rom: Box<[u8]>,    // up to 32 MB
    /// Cartridge backup: SRAM / Flash (region 0xE-0xF) or EEPROM (region 0xD).
    pub save: Save,
    pub ppu: Ppu,
    pub apu: Apu,
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
    /// Debug write-watchpoint: current CPU PC, watched address (0 = off), and a
    /// capped log of (pc, value, width) writes that hit it.
    pub cur_pc: u32,
    pub watch_addr: u32,
    pub watch_hits: Vec<(u32, u32, u32)>,
    /// Debug counter: sound-FIFO DMA refills (4-word transfers) since boot.
    pub dbg_fifo_refills: u64,
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
        let save = Save::detect(&rom);
        GbaBus {
            bios: b.into_boxed_slice(),
            ewram: vec![0; 256 * 1024].into_boxed_slice(),
            iwram: vec![0; 32 * 1024].into_boxed_slice(),
            rom: rom.into_boxed_slice(),
            save,
            ppu: Ppu::new(),
            apu: Apu::new(),
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
            cur_pc: 0,
            watch_addr: 0,
            watch_hits: Vec::new(),
            dbg_fifo_refills: 0,
        }
    }

    pub fn serialize(&self, w: &mut crate::state::Writer) {
        w.u16(self.keyinput);
        w.u16(self.ie);
        w.u16(self.if_);
        w.u16(self.ime);
        w.u64(self.cycles);
        w.u64(self.timer_cycles);
        w.bool(self.halted);
        for i in 0..4 {
            w.u32(self.dma_src[i]);
            w.u32(self.dma_dst[i]);
            w.u32(self.dma_count[i]);
        }
        for t in &self.timers {
            w.u16(t.reload);
            w.u16(t.counter);
            w.u16(t.control);
            w.u32(t.subcycle);
        }
        w.bytes(&self.ewram);
        w.bytes(&self.iwram);
        w.bytes(&self.io);
        self.save.serialize(w);
        self.ppu.serialize(w);
        self.apu.serialize(w);
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        self.keyinput = r.u16();
        self.ie = r.u16();
        self.if_ = r.u16();
        self.ime = r.u16();
        self.cycles = r.u64();
        self.timer_cycles = r.u64();
        self.halted = r.bool();
        for i in 0..4 {
            self.dma_src[i] = r.u32();
            self.dma_dst[i] = r.u32();
            self.dma_count[i] = r.u32();
        }
        for t in &mut self.timers {
            t.reload = r.u16();
            t.counter = r.u16();
            t.control = r.u16();
            t.subcycle = r.u32();
        }
        r.bytes_into(&mut self.ewram);
        r.bytes_into(&mut self.iwram);
        r.bytes_into(&mut self.io);
        self.save.deserialize(r);
        self.ppu.deserialize(r);
        // The APU block was appended after the state format shipped; only read it
        // when the blob actually carries it, so pre-APU states still load.
        if r.remaining() > 8 {
            self.apu.deserialize(r);
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

    /// Debug: a DMA channel's live (latched) source and its control-register
    /// snapshot, plus the current per-channel running source. For diagnosing the
    /// sound FIFO DMA (channels 1/2).
    pub fn dma_dbg(&self, ch: usize) -> (u32, u32, u16, u32) {
        let base = 0xB0 + ch as u32 * 12;
        (self.io_u32(base) & 0x0FFF_FFFF, self.io_u32(base + 4) & 0x0FFF_FFFF, self.io_u16(base + 10), self.dma_src[ch])
    }

    /// The overflow period (in system cycles) of timer `ch`, or `None` if it is
    /// stopped or in cascade mode. Used by the APU to schedule Direct Sound FIFO
    /// pops at the audio rate the game programmed.
    pub fn ds_timer_period(&self, ch: usize) -> Option<u32> {
        let t = &self.timers[ch];
        if t.control & 0x80 == 0 || (ch > 0 && t.control & 4 != 0) {
            return None;
        }
        let prescaler = [1u32, 64, 256, 1024][(t.control & 3) as usize];
        Some((0x1_0000 - t.reload as u32) * prescaler)
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
        // EEPROM lives in region 0xD and is driven bit-serially by DMA; the
        // transfer length tells the chip its command and address width.
        let ee_src = self.save.is_eeprom() && (src >> 24) == 0xD;
        let ee_dst = self.save.is_eeprom() && (dst >> 24) == 0xD;
        if ee_src || ee_dst {
            self.save.eeprom_set_dma_len(self.dma_count[ch]);
        }
        for _ in 0..self.dma_count[ch] {
            if ee_src || ee_dst {
                let v = if ee_src {
                    self.save.eeprom_read_bit() as u32
                } else {
                    self.read16_raw(src & !1) as u32
                };
                if ee_dst {
                    self.save.eeprom_write_bit(v as u8);
                } else {
                    self.write(dst & !1, v, 2);
                }
            } else if word {
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

    /// Top up the Direct Sound FIFOs from their sound DMA channels. Hardware
    /// requests a refill when a FIFO drops to <= 4 words (16 bytes); DMA1/DMA2 in
    /// "special" timing service FIFO_A/FIFO_B respectively, always 4 words wide.
    pub fn refill_fifos(&mut self) {
        for ch in [1usize, 2] {
            let base = 0xB0 + ch as u32 * 12;
            let control = self.io_u16(base + 10);
            if control & 0x8000 == 0 || (control >> 12) & 3 != 3 {
                continue; // channel off, or not sound-FIFO timing
            }
            let dst = self.io_u32(base + 4) & 0x07FF_FFFF;
            let level = match dst {
                0x0400_00A0 => self.apu.fifo_a_len(),
                0x0400_00A4 => self.apu.fifo_b_len(),
                _ => continue,
            };
            if level <= 16 {
                self.run_sound_dma(ch, dst);
            }
        }
    }

    /// Transfer one FIFO request: 4 words from the (incrementing) source into the
    /// fixed FIFO port. The word count and destination-fixed behaviour are forced
    /// by the hardware regardless of the channel's programmed count/dest-control.
    fn run_sound_dma(&mut self, ch: usize, dst: u32) {
        self.dbg_fifo_refills += 1;
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        let src_ctrl = (control >> 7) & 3;
        let mut src = self.dma_src[ch];
        for _ in 0..4 {
            let v = self.read32_raw(src & !3);
            self.write(dst, v, 4);
            src = match src_ctrl {
                1 => src.wrapping_sub(4),
                2 => src,
                _ => src.wrapping_add(4),
            };
            self.cycles += 4;
        }
        self.dma_src[ch] = src;
        if control & 0x4000 != 0 {
            self.if_ |= 1 << (8 + ch); // DMA-complete IRQ
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
            0xE | 0xF => self.save.read(addr),
            _ => 0,
        }
    }

    /// Fast path for the large linear regions (BIOS/EWRAM/IWRAM/ROM): the region
    /// slice and the masked offset. `None` for the small side-effect regions
    /// (I/O, PAL/VRAM/OAM, SRAM), which stay on the byte-compose path.
    #[inline]
    fn linear_region(&self, addr: u32) -> Option<(&[u8], usize)> {
        match (addr >> 24) & 0xF {
            0x0 => Some((&self.bios, (addr & 0x3FFF) as usize)),
            0x2 => Some((&self.ewram, (addr & 0x3_FFFF) as usize)),
            0x3 => Some((&self.iwram, (addr & 0x7FFF) as usize)),
            0x8..=0xD => Some((&self.rom, (addr & 0x01FF_FFFF) as usize)),
            _ => None,
        }
    }

    #[inline]
    fn read16_raw(&self, addr: u32) -> u16 {
        // The instruction-fetch/data hot path: one region dispatch, one slice
        // read, instead of two per-byte dispatches.
        if let Some((mem, o)) = self.linear_region(addr) {
            if o + 2 <= mem.len() {
                return u16::from_le_bytes([mem[o], mem[o + 1]]);
            }
        }
        u16::from_le_bytes([self.read8_raw(addr), self.read8_raw(addr + 1)])
    }
    /// Debug-only word read (no cycle cost, no side effects).
    pub fn read32_dbg(&self, addr: u32) -> u32 {
        self.read32_raw(addr)
    }

    #[inline]
    fn read32_raw(&self, addr: u32) -> u32 {
        // The bus always returns the word-aligned value; the CPU rotates it for
        // an unaligned LDR. Reading the raw (unaligned) bytes instead corrupts
        // e.g. Pokemon's palette-fade loop (it LDRs colours from a halfword
        // offset), which halved the palette and garbled sprites.
        let addr = addr & !3;
        if let Some((mem, o)) = self.linear_region(addr) {
            if o + 4 <= mem.len() {
                return u32::from_le_bytes([mem[o], mem[o + 1], mem[o + 2], mem[o + 3]]);
            }
        }
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
            0x060..=0x0A7 => self.apu.read8(off),
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
        // 0x10000000-0xFFFFFFFF is unused address space (the upper region-decode
        // nibble is not mapped): writes there are ignored on hardware. Bailing
        // here is essential — otherwise a wild/uninitialised pointer store (which
        // real hardware harmlessly drops) would alias down into a real region and
        // corrupt it. Motocross Maniacs et al. store to such a scratch pointer and
        // would otherwise clobber their own copied IRQ handler in IWRAM.
        if addr >= 0x1000_0000 {
            return;
        }
        // ARM7TDMI force-aligns store addresses to the access width: STR writes at
        // addr & ~3, STRH at addr & ~1 (unlike LDR, a store does NOT rotate — the
        // register value just lands at the aligned address). Without this, a word
        // store to an unaligned address splits across the aligned slot's neighbour
        // and corrupts it. DBZ Legacy of Goku registers a DMA2 IRQ handler with an
        // unaligned STR to its ISR table; the raw-address write mangled the entry's
        // top byte, so the first DMA2 IRQ vectored into garbage and ran away. (The
        // load side of this was fixed earlier in read32_raw.)
        let addr = match width {
            4 => addr & !3,
            2 => addr & !1,
            _ => addr,
        };
        if self.watch_addr != 0 && addr >= self.watch_addr && addr < self.watch_addr + 0x4
            && self.watch_hits.len() < 200
        {
            // Stash width in the top nibble (palette values are 16-bit, so free).
            self.watch_hits.push((self.cur_pc, (width << 28) | addr, val));
        }
        match (addr >> 24) & 0xF {
            0x2 => write_le(&mut self.ewram, (addr & 0x3_FFFF) as usize, val, width),
            0x3 => write_le(&mut self.iwram, (addr & 0x7FFF) as usize, val, width),
            0x4 => self.io_write(addr, val, width),
            0x5 => self.display_write(DisplayRegion::Pal, addr, val, width),
            0x6 => self.display_write(DisplayRegion::Vram, addr, val, width),
            0x7 => self.display_write(DisplayRegion::Oam, addr, val, width),
            // SRAM/Flash: an 8-bit bus, so only the low byte is written per byte.
            0xE | 0xF => self.save.write(addr, val as u8),
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
                0x060..=0x0A7 => self.apu.write8(o, byte),
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
    let width = width as usize;
    // Fast path: the whole value fits without wrapping (the common case).
    if off + width <= mem.len() {
        for i in 0..width {
            mem[off + i] = (val >> (i * 8)) as u8;
        }
        return;
    }
    let len = mem.len();
    for i in 0..width {
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
    fn unused_high_addresses_do_not_alias_into_ram() {
        // Addresses >= 0x10000000 are unused space (the upper region-decode nibble
        // is not mapped); writes there must be dropped, NOT folded into a real
        // region. Before the fix, 0xE3A00057 aliased to IWRAM 0x0057 via a 4-bit
        // region mask and clobbered whatever lived there (e.g. a copied IRQ
        // handler), turning a harmless wild-pointer store into a boot crash.
        let mut b = bus();
        b.write32(0x0300_0054, 0xE28C_C004, Access::NonSeq); // real IWRAM word
        b.write32(0xE3A0_0057, 0x0000_0000, Access::NonSeq); // wild pointer store
        assert_eq!(
            b.read32(0x0300_0054, Access::NonSeq),
            0xE28C_C004,
            "a store to unused space (0xE3A00057) must not touch IWRAM"
        );
    }

    #[test]
    fn unaligned_word_store_force_aligns() {
        // ARM7TDMI STR ignores the low address bits: a word store to 0x...3 lands
        // at the word-aligned slot (0x...0), it does NOT straddle into the next
        // word. DBZ Legacy of Goku registers an IRQ handler this way; the raw
        // (unaligned) write corrupted the neighbouring ISR-table entry.
        let mut b = bus();
        b.write32(0x0300_0010, 0xAAAA_AAAA, Access::NonSeq); // neighbour slot
        b.write32(0x0300_000F, 0x0800_996D, Access::NonSeq); // unaligned -> 0x...0C
        assert_eq!(
            b.read32(0x0300_000C, Access::NonSeq),
            0x0800_996D,
            "unaligned word store must land at the aligned slot"
        );
        assert_eq!(
            b.read32(0x0300_0010, Access::NonSeq),
            0xAAAA_AAAA,
            "unaligned word store must not corrupt the next word"
        );
        // Halfword stores force-align to & ~1 the same way.
        b.write16(0x0300_0021, 0x1234, Access::NonSeq); // -> 0x...20
        assert_eq!(b.read16(0x0300_0020, Access::NonSeq), 0x1234);
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
        // Direct EEPROM access (e.g. a game polling write-ready).
        if self.save.is_eeprom() && (addr >> 24) == 0xD {
            return self.save.eeprom_read_bit() as u16;
        }
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
        if self.save.is_eeprom() && (addr >> 24) == 0xD {
            self.save.eeprom_write_bit(val as u8);
            return;
        }
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
