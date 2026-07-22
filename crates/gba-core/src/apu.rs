//! Clean-room GBA APU: the four PSG channels (identical to the DMG's sound
//! hardware) plus the two 8-bit Direct Sound FIFO channels. Implemented from
//! GBATEK's register maps and frequency formulas.
//!
//! Output is interleaved-stereo `i16` at 32768 Hz. The GBA system clock is
//! 16_777_216 Hz, so one output sample is produced every 512 system cycles
//! exactly. [`Apu::generate`] advances the channel state in 512-cycle chunks and
//! pushes one stereo frame per chunk; the front-end drains [`Apu::take_output`].
//!
//! Direct Sound: each FIFO is popped one byte per selected-timer overflow. The
//! pop times are scheduled analytically from the timer period the bus passes in,
//! which gives audio-rate FIFO stepping without threading fine-grained timer
//! ticks through the whole frame loop. FIFO refills (the sound DMA) are driven
//! by the bus from the FIFO fill levels this module exposes.

use crate::state::{Reader, Writer};

const CYCLES_PER_SAMPLE: u64 = 512;
/// Internal oversampling: the mixer is sampled this many times per output sample
/// and box-averaged down. This band-limits the zero-order-hold steps of the 8-bit
/// Direct Sound PCM and the PSG square edges, and averages out the small jitter
/// when a channel's rate isn't a clean divisor of the output grid, which is what
/// otherwise reads as a sprinkle of static. 512 / 4 = 128 cycles per sub-sample.
const OVERSAMPLE: u64 = 4;
const SUB_CYCLES: u64 = CYCLES_PER_SAMPLE / OVERSAMPLE;
/// System cycles per 512 Hz frame-sequencer tick (16_777_216 / 512).
const FS_PERIOD: u32 = 32_768;

const DUTY: [u8; 4] = [0b0000_0001, 0b1000_0001, 0b1000_0111, 0b0111_1110];

/// A tone/envelope generator shared by the two square channels and (minus the
/// duty) the noise channel.
#[derive(Clone, Copy, Default)]
struct Env {
    initial: u8,
    up: bool,
    period: u8,
    vol: u8,
    timer: u8,
}
impl Env {
    fn trigger(&mut self) {
        self.vol = self.initial;
        self.timer = self.period;
    }
    fn clock(&mut self) {
        if self.period == 0 {
            return;
        }
        if self.timer > 0 {
            self.timer -= 1;
        }
        if self.timer == 0 {
            self.timer = self.period;
            if self.up && self.vol < 15 {
                self.vol += 1;
            } else if !self.up && self.vol > 0 {
                self.vol -= 1;
            }
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Square {
    freq: u16, // 11-bit
    duty: u8,
    timer: i32,
    phase: u8,
    length: u16,
    length_enable: bool,
    env: Env,
    // sweep (channel 1 only; left at defaults for channel 2)
    sweep_period: u8,
    sweep_down: bool,
    sweep_shift: u8,
    sweep_timer: u8,
    sweep_enable: bool,
    sweep_shadow: u16,
    enabled: bool,
    dac_on: bool,
}
impl Square {
    fn advance(&mut self, n: u64) {
        if !self.enabled {
            return;
        }
        let period = ((2048 - self.freq as i32) * 16).max(1);
        self.timer -= n as i32;
        while self.timer <= 0 {
            self.timer += period;
            self.phase = (self.phase + 1) & 7;
        }
    }
    fn dac(&self) -> i32 {
        if !self.enabled || !self.dac_on {
            return 0;
        }
        let high = DUTY[self.duty as usize] & (1 << self.phase) != 0;
        (if high { self.env.vol } else { 0 }) as i32 - 8
    }
    fn clock_length(&mut self) {
        if self.length_enable && self.length > 0 {
            self.length -= 1;
            if self.length == 0 {
                self.enabled = false;
            }
        }
    }
    fn sweep_calc(&mut self) -> u16 {
        let delta = self.sweep_shadow >> self.sweep_shift;
        if self.sweep_down {
            self.sweep_shadow.wrapping_sub(delta)
        } else {
            self.sweep_shadow.wrapping_add(delta)
        }
    }
    fn clock_sweep(&mut self) {
        if !self.sweep_enable {
            return;
        }
        if self.sweep_timer > 0 {
            self.sweep_timer -= 1;
        }
        if self.sweep_timer == 0 {
            self.sweep_timer = if self.sweep_period == 0 { 8 } else { self.sweep_period };
            if self.sweep_period > 0 {
                let new = self.sweep_calc();
                if new > 2047 {
                    self.enabled = false;
                } else if self.sweep_shift > 0 {
                    self.sweep_shadow = new;
                    self.freq = new;
                    // Overflow check on the recomputed value.
                    if self.sweep_calc() > 2047 {
                        self.enabled = false;
                    }
                }
            }
        }
    }
    fn trigger(&mut self) {
        self.enabled = self.dac_on;
        if self.length == 0 {
            self.length = 64;
        }
        self.timer = ((2048 - self.freq as i32) * 16).max(1);
        self.env.trigger();
        // Sweep init (harmless for channel 2, whose sweep fields stay zero).
        self.sweep_shadow = self.freq;
        self.sweep_timer = if self.sweep_period == 0 { 8 } else { self.sweep_period };
        self.sweep_enable = self.sweep_period > 0 || self.sweep_shift > 0;
        if self.sweep_shift > 0 && self.sweep_calc() > 2047 {
            self.enabled = false;
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Wave {
    dac_on: bool,
    length: u16,
    length_enable: bool,
    freq: u16,
    volume: u8, // 0=mute,1=100%,2=50%,3=25%
    force75: bool,
    mode64: bool,
    bank: u8,
    timer: i32,
    pos: u8, // 0..63
    enabled: bool,
}
impl Wave {
    fn advance(&mut self, n: u64, ram: &[u8; 32]) {
        let _ = ram;
        if !self.enabled {
            return;
        }
        let period = ((2048 - self.freq as i32) * 8).max(1);
        self.timer -= n as i32;
        while self.timer <= 0 {
            self.timer += period;
            let len = if self.mode64 { 64 } else { 32 };
            self.pos = (self.pos + 1) % len;
        }
    }
    fn dac(&self, ram: &[u8; 32]) -> i32 {
        if !self.enabled || !self.dac_on || self.volume == 0 {
            return 0;
        }
        // In 32-sample mode playback runs through the bank selected by NR30 bit6;
        // in 64-sample mode it runs through all 32 bytes (64 nibbles).
        let idx = if self.mode64 {
            self.pos as usize
        } else {
            self.bank as usize * 32 + self.pos as usize
        };
        let byte = ram[(idx / 2) & 31];
        let nib = if idx & 1 == 0 { byte >> 4 } else { byte & 0xF };
        let level = if self.force75 {
            (nib as u32 * 3 / 4) as u8
        } else {
            nib >> (self.volume - 1)
        };
        level as i32 - 8
    }
    fn clock_length(&mut self) {
        if self.length_enable && self.length > 0 {
            self.length -= 1;
            if self.length == 0 {
                self.enabled = false;
            }
        }
    }
    fn trigger(&mut self) {
        self.enabled = self.dac_on;
        if self.length == 0 {
            self.length = 256;
        }
        self.timer = ((2048 - self.freq as i32) * 8).max(1);
        self.pos = 0;
    }
}

/// GB noise divisor table, in system cycles (already scaled x4 from the DMG's
/// 4.19 MHz clock to the GBA's 16.78 MHz).
const NOISE_DIV: [i32; 8] = [8, 16, 32, 48, 64, 80, 96, 112];

#[derive(Clone, Copy)]
struct Noise {
    length: u16,
    length_enable: bool,
    env: Env,
    div: u8,
    width7: bool,
    shift: u8,
    timer: i32,
    lfsr: u16,
    enabled: bool,
    dac_on: bool,
}
impl Default for Noise {
    fn default() -> Self {
        Noise {
            length: 0,
            length_enable: false,
            env: Env::default(),
            div: 0,
            width7: false,
            shift: 0,
            timer: 0,
            lfsr: 0x7FFF,
            enabled: false,
            dac_on: false,
        }
    }
}
impl Noise {
    fn period(&self) -> i32 {
        // 14+ shift disables the channel clock on hardware; clamp to keep it sane.
        ((NOISE_DIV[self.div as usize]) << self.shift.min(14) as i32).max(1) * 4
    }
    fn advance(&mut self, n: u64) {
        if !self.enabled {
            return;
        }
        self.timer -= n as i32;
        while self.timer <= 0 {
            self.timer += self.period();
            let bit = (self.lfsr ^ (self.lfsr >> 1)) & 1;
            self.lfsr >>= 1;
            self.lfsr |= bit << 14;
            if self.width7 {
                self.lfsr &= !(1 << 6);
                self.lfsr |= bit << 6;
            }
        }
    }
    fn dac(&self) -> i32 {
        if !self.enabled || !self.dac_on {
            return 0;
        }
        let high = self.lfsr & 1 == 0;
        (if high { self.env.vol } else { 0 }) as i32 - 8
    }
    fn clock_length(&mut self) {
        if self.length_enable && self.length > 0 {
            self.length -= 1;
            if self.length == 0 {
                self.enabled = false;
            }
        }
    }
    fn trigger(&mut self) {
        self.enabled = self.dac_on;
        if self.length == 0 {
            self.length = 64;
        }
        self.timer = self.period();
        self.lfsr = 0x7FFF;
        self.env.trigger();
    }
}

#[derive(Clone, Copy)]
struct Fifo {
    buf: [i8; 32],
    head: usize,
    len: usize,
}
impl Default for Fifo {
    fn default() -> Self {
        Fifo { buf: [0; 32], head: 0, len: 0 }
    }
}
impl Fifo {
    fn push(&mut self, v: i8) {
        if self.len < 32 {
            self.buf[(self.head + self.len) % 32] = v;
            self.len += 1;
        }
    }
    fn pop(&mut self) -> Option<i8> {
        if self.len == 0 {
            return None;
        }
        let v = self.buf[self.head];
        self.head = (self.head + 1) % 32;
        self.len -= 1;
        Some(v)
    }
    fn clear(&mut self) {
        self.head = 0;
        self.len = 0;
    }
}

pub struct Apu {
    /// Raw register bytes for I/O offsets 0x60..=0xA7 (readback + field decode),
    /// indexed by `off - 0x60`.
    reg: [u8; 0x48],
    wave_ram: [u8; 32],
    master_enable: bool,

    fs_cycle: u32,
    fs_step: u8,

    ch1: Square,
    ch2: Square,
    ch3: Wave,
    ch4: Noise,

    fifo_a: Fifo,
    fifo_b: Fifo,
    ds_a: i8,
    ds_b: i8,
    a_next: u64,
    b_next: u64,

    // DC-blocking high-pass state (integer one-pole, corner ~1.3 Hz), per side.
    // The `y` state is kept in Q12 fixed point (i64) so the feedback term never
    // hits a >>12 deadband and gets stuck on a non-zero offset in silence.
    hp_xl: i32,
    hp_yl: i64,
    hp_xr: i32,
    hp_yr: i64,
    // 2-tap moving-average state (previous filtered sample per side). Its null is
    // at the output Nyquist (16384 Hz) = exactly where the 16384 Hz Direct Sound
    // playback images, so it removes the hard DS-sample-boundary steps ("blips")
    // that the oversampler can't (they fall between output samples).
    ma_l: i32,
    ma_r: i32,

    cycle: u64,
    out: Vec<i16>,
}

impl Default for Apu {
    fn default() -> Self {
        Apu {
            reg: [0; 0x48],
            wave_ram: [0; 32],
            master_enable: false,
            fs_cycle: 0,
            fs_step: 0,
            ch1: Square::default(),
            ch2: Square::default(),
            ch3: Wave::default(),
            ch4: Noise::default(),
            fifo_a: Fifo::default(),
            fifo_b: Fifo::default(),
            ds_a: 0,
            ds_b: 0,
            a_next: 0,
            b_next: 0,
            hp_xl: 0,
            hp_yl: 0,
            hp_xr: 0,
            hp_yr: 0,
            ma_l: 0,
            ma_r: 0,
            cycle: 0,
            out: Vec::new(),
        }
    }
}

impl Apu {
    pub fn new() -> Self {
        Apu::default()
    }

    pub fn take_output(&mut self) -> Vec<i16> {
        std::mem::take(&mut self.out)
    }

    /// Align the APU's clock to `cycle` (dropping any FIFO-pop backlog). Called
    /// after a save-state load so audio resumes from the current time.
    pub fn resync(&mut self, cycle: u64) {
        self.cycle = cycle;
        self.a_next = cycle;
        self.b_next = cycle;
    }

    pub fn fifo_a_len(&self) -> usize {
        self.fifo_a.len
    }
    pub fn fifo_b_len(&self) -> usize {
        self.fifo_b.len
    }

    fn r16(&self, off: usize) -> u16 {
        u16::from_le_bytes([self.reg[off - 0x60], self.reg[off - 0x60 + 1]])
    }

    // --- I/O ------------------------------------------------------------------

    pub fn read8(&self, off: u32) -> u8 {
        let off = off as usize;
        match off {
            0x84 => {
                // SOUNDCNT_X: bit7 master enable, bits0-3 per-channel status.
                let mut v = (self.master_enable as u8) << 7;
                v |= self.ch1.enabled as u8;
                v |= (self.ch2.enabled as u8) << 1;
                v |= (self.ch3.enabled as u8) << 2;
                v |= (self.ch4.enabled as u8) << 3;
                v
            }
            0x90..=0x9F => self.wave_ram[off - 0x90],
            0xA0..=0xA7 => 0, // FIFOs are write-only
            0x60..=0xA7 => self.reg[off - 0x60],
            _ => 0,
        }
    }

    pub fn write8(&mut self, off: u32, val: u8) {
        let off = off as usize;
        // FIFO data ports: bytes are pushed straight into the ring.
        if (0xA0..=0xA3).contains(&off) {
            self.fifo_a.push(val as i8);
            return;
        }
        if (0xA4..=0xA7).contains(&off) {
            self.fifo_b.push(val as i8);
            return;
        }
        if (0x90..=0x9F).contains(&off) {
            self.wave_ram[off - 0x90] = val;
            self.reg[off - 0x60] = val;
            return;
        }
        if !(0x60..=0xA7).contains(&off) {
            return;
        }
        // NR52 master enable gates everything; while off, writes to the channel
        // registers (0x60..0x81) are ignored, matching DMG behaviour.
        if off == 0x84 {
            let on = val & 0x80 != 0;
            if !on {
                for r in self.reg.iter_mut().take(0x24) {
                    *r = 0;
                }
                self.ch1 = Square::default();
                self.ch2 = Square::default();
                self.ch3 = Wave::default();
                self.ch4 = Noise::default();
            }
            self.master_enable = on;
            return;
        }
        if !self.master_enable && off < 0x82 {
            return;
        }
        self.reg[off - 0x60] = val;
        self.decode(off);
    }

    /// Re-derive channel state after a byte write to `off`, triggering the owning
    /// channel when its NRx4 high byte has bit 7 set.
    fn decode(&mut self, off: usize) {
        match off {
            0x60..=0x65 => {
                let s = self.r16(0x62);
                self.ch1.env.period = ((s >> 8) & 7) as u8;
                self.ch1.env.up = s & 0x0800 != 0;
                self.ch1.env.initial = ((s >> 12) & 0xF) as u8;
                self.ch1.dac_on = s & 0xF800 != 0;
                self.ch1.duty = ((s >> 6) & 3) as u8;
                if off == 0x62 {
                    self.ch1.length = 64 - (s & 0x3F);
                }
                let sw = self.r16(0x60);
                self.ch1.sweep_shift = (sw & 7) as u8;
                self.ch1.sweep_down = sw & 8 != 0;
                self.ch1.sweep_period = ((sw >> 4) & 7) as u8;
                let x = self.r16(0x64);
                self.ch1.freq = x & 0x7FF;
                self.ch1.length_enable = x & 0x4000 != 0;
                if off == 0x65 && x & 0x8000 != 0 {
                    self.ch1.trigger();
                }
                if !self.ch1.dac_on {
                    self.ch1.enabled = false;
                }
            }
            0x68..=0x6D => {
                let s = self.r16(0x68);
                self.ch2.env.period = ((s >> 8) & 7) as u8;
                self.ch2.env.up = s & 0x0800 != 0;
                self.ch2.env.initial = ((s >> 12) & 0xF) as u8;
                self.ch2.dac_on = s & 0xF800 != 0;
                self.ch2.duty = ((s >> 6) & 3) as u8;
                if off == 0x68 {
                    self.ch2.length = 64 - (s & 0x3F);
                }
                let x = self.r16(0x6C);
                self.ch2.freq = x & 0x7FF;
                self.ch2.length_enable = x & 0x4000 != 0;
                if off == 0x6D && x & 0x8000 != 0 {
                    self.ch2.trigger();
                }
                if !self.ch2.dac_on {
                    self.ch2.enabled = false;
                }
            }
            0x70..=0x75 => {
                let n30 = self.reg[0x70 - 0x60];
                self.ch3.mode64 = n30 & 0x20 != 0;
                self.ch3.bank = ((n30 >> 6) & 1) as u8;
                self.ch3.dac_on = n30 & 0x80 != 0;
                let h = self.r16(0x72);
                if off == 0x72 || off == 0x73 {
                    self.ch3.length = 256 - (h & 0xFF);
                }
                self.ch3.volume = ((h >> 13) & 3) as u8;
                self.ch3.force75 = h & 0x8000 != 0;
                let x = self.r16(0x74);
                self.ch3.freq = x & 0x7FF;
                self.ch3.length_enable = x & 0x4000 != 0;
                if off == 0x75 && x & 0x8000 != 0 {
                    self.ch3.trigger();
                }
                if !self.ch3.dac_on {
                    self.ch3.enabled = false;
                }
            }
            0x78..=0x7D => {
                let s = self.r16(0x78);
                self.ch4.length = 64 - (s & 0x3F);
                self.ch4.env.period = ((s >> 8) & 7) as u8;
                self.ch4.env.up = s & 0x0800 != 0;
                self.ch4.env.initial = ((s >> 12) & 0xF) as u8;
                self.ch4.dac_on = s & 0xF800 != 0;
                let x = self.r16(0x7C);
                self.ch4.div = (x & 7) as u8;
                self.ch4.width7 = x & 8 != 0;
                self.ch4.shift = ((x >> 4) & 0xF) as u8;
                self.ch4.length_enable = x & 0x4000 != 0;
                if off == 0x7D && x & 0x8000 != 0 {
                    self.ch4.trigger();
                }
                if !self.ch4.dac_on {
                    self.ch4.enabled = false;
                }
            }
            0x83 => {
                // SOUNDCNT_H high byte: FIFO reset bits (11 = A, 15 = B).
                if self.reg[0x83 - 0x60] & 0x08 != 0 {
                    self.fifo_a.clear();
                    self.reg[0x83 - 0x60] &= !0x08;
                }
                if self.reg[0x83 - 0x60] & 0x80 != 0 {
                    self.fifo_b.clear();
                    self.reg[0x83 - 0x60] &= !0x80;
                }
            }
            _ => {}
        }
    }

    // --- Sample generation ----------------------------------------------------

    /// Advance to `target` system cycles, emitting one stereo sample per 512
    /// cycles. `timer_period[i]` is the current overflow period (in cycles) of
    /// timer `i`, or `None` if that timer is stopped; the selected one clocks the
    /// matching Direct Sound FIFO.
    pub fn generate(&mut self, timer_period: [Option<u32>; 2], target: u64) {
        // Defensive: if the clock jumped far ahead of us (e.g. a save-state load
        // left the APU cycle behind the bus cycle), snap forward instead of
        // emitting a huge silent backlog of samples.
        if target > self.cycle + 300_000 {
            self.resync(target);
        }
        while self.cycle + CYCLES_PER_SAMPLE <= target {
            let mut acc_l = 0i32;
            let mut acc_r = 0i32;
            for _ in 0..OVERSAMPLE {
                self.advance_chunk(SUB_CYCLES, timer_period);
                self.cycle += SUB_CYCLES;
                let (l, r) = self.mix_raw();
                acc_l += l;
                acc_r += r;
            }
            let (l, r) = self.dc_block(acc_l / OVERSAMPLE as i32, acc_r / OVERSAMPLE as i32);
            let out_l = (l + self.ma_l) / 2;
            let out_r = (r + self.ma_r) / 2;
            self.ma_l = l;
            self.ma_r = r;
            self.out.push(out_l.clamp(-32768, 32767) as i16);
            self.out.push(out_r.clamp(-32768, 32767) as i16);
        }
    }

    /// One-pole DC blocker per side: y[n] = x[n] - x[n-1] + (1 - 2^-12)*y[n-1],
    /// with `y` carried in Q12 (state = y * 4096) so the feedback decays cleanly
    /// to zero instead of sticking in a >>12 deadband. Removes the standing offset
    /// (and its steps when channels toggle) the DAC centring and Direct Sound
    /// latch leave behind, without touching tone.
    fn dc_block(&mut self, l: i32, r: i32) -> (i32, i32) {
        self.hp_yl += ((l - self.hp_xl) as i64) << 12;
        self.hp_yl -= self.hp_yl >> 12;
        self.hp_xl = l;
        self.hp_yr += ((r - self.hp_xr) as i64) << 12;
        self.hp_yr -= self.hp_yr >> 12;
        self.hp_xr = r;
        ((self.hp_yl >> 12) as i32, (self.hp_yr >> 12) as i32)
    }

    fn advance_chunk(&mut self, n: u64, timer_period: [Option<u32>; 2]) {
        self.ch1.advance(n);
        self.ch2.advance(n);
        self.ch3.advance(n, &self.wave_ram);
        self.ch4.advance(n);
        self.fs_cycle += n as u32;
        while self.fs_cycle >= FS_PERIOD {
            self.fs_cycle -= FS_PERIOD;
            self.tick_frame_seq();
        }
        let cnt_h = self.r16(0x82);
        let a_timer = ((cnt_h >> 10) & 1) as usize;
        let b_timer = ((cnt_h >> 14) & 1) as usize;
        let start = self.cycle;
        let end = self.cycle + n;
        self.pop_ds(false, timer_period[a_timer], start, end);
        self.pop_ds(true, timer_period[b_timer], start, end);
    }

    fn pop_ds(&mut self, is_b: bool, period: Option<u32>, start: u64, end: u64) {
        let period = match period {
            Some(p) if p > 0 => p as u64,
            _ => return,
        };
        let mut next = if is_b { self.b_next } else { self.a_next };
        if next < start {
            next = start; // (re)synchronise; never burst-catch-up a stale schedule
        }
        while next < end {
            let sample = if is_b { self.fifo_b.pop() } else { self.fifo_a.pop() };
            if let Some(s) = sample {
                if is_b {
                    self.ds_b = s;
                } else {
                    self.ds_a = s;
                }
            }
            next += period;
        }
        if is_b {
            self.b_next = next;
        } else {
            self.a_next = next;
        }
    }

    fn tick_frame_seq(&mut self) {
        let step = self.fs_step;
        self.fs_step = (self.fs_step + 1) & 7;
        if step & 1 == 0 {
            self.ch1.clock_length();
            self.ch2.clock_length();
            self.ch3.clock_length();
            self.ch4.clock_length();
        }
        if step == 2 || step == 6 {
            self.ch1.clock_sweep();
        }
        if step == 7 {
            self.ch1.env.clock();
            self.ch2.env.clock();
            self.ch4.env.clock();
        }
    }

    fn mix_raw(&self) -> (i32, i32) {
        if !self.master_enable {
            return (0, 0);
        }
        let cnt_l = self.r16(0x80);
        let cnt_h = self.r16(0x82);
        let right_vol = (cnt_l & 7) as i32;
        let left_vol = ((cnt_l >> 4) & 7) as i32;
        let dac = [self.ch1.dac(), self.ch2.dac(), self.ch3.dac(&self.wave_ram), self.ch4.dac()];
        let mut psg_l = 0i32;
        let mut psg_r = 0i32;
        for (i, &d) in dac.iter().enumerate() {
            if cnt_l & (0x1000 << i) != 0 {
                psg_l += d;
            }
            if cnt_l & (0x0100 << i) != 0 {
                psg_r += d;
            }
        }
        psg_l *= left_vol + 1;
        psg_r *= right_vol + 1;
        // PSG master ratio: bits 0-1 of SOUNDCNT_H (00=25%,01=50%,10=100%).
        let shift = match cnt_h & 3 {
            0 => 2,
            1 => 1,
            _ => 0,
        };
        psg_l >>= shift;
        psg_r >>= shift;

        // Direct Sound: 8-bit signed, 100% or 50% volume (bits 2/3).
        let a = self.ds_a as i32 * if cnt_h & 0x04 != 0 { 4 } else { 2 };
        let b = self.ds_b as i32 * if cnt_h & 0x08 != 0 { 4 } else { 2 };
        let mut l = psg_l;
        let mut r = psg_r;
        if cnt_h & 0x0200 != 0 {
            l += a;
        }
        if cnt_h & 0x0100 != 0 {
            r += a;
        }
        if cnt_h & 0x2000 != 0 {
            l += b;
        }
        if cnt_h & 0x1000 != 0 {
            r += b;
        }
        (l * 24, r * 24)
    }

    // --- Save-state -----------------------------------------------------------

    pub fn serialize(&self, w: &mut Writer) {
        w.bytes(&self.reg);
        w.bytes(&self.wave_ram);
        w.bool(self.master_enable);
        w.u32(self.fs_cycle);
        w.u8(self.fs_step);
        w.u64(self.cycle);
        w.u64(self.a_next);
        w.u64(self.b_next);
        w.u8(self.ds_a as u8);
        w.u8(self.ds_b as u8);
        w.i32(self.hp_xl);
        w.u64(self.hp_yl as u64);
        w.i32(self.hp_xr);
        w.u64(self.hp_yr as u64);
        w.i32(self.ma_l);
        w.i32(self.ma_r);
        for f in [&self.fifo_a, &self.fifo_b] {
            w.u8(f.len as u8);
            w.u8(f.head as u8);
            for &s in &f.buf {
                w.u8(s as u8);
            }
        }
        // Running channel timers/phases (registers alone don't capture these).
        for &(t, p) in &[(self.ch1.timer, self.ch1.phase), (self.ch2.timer, self.ch2.phase)] {
            w.i32(t);
            w.u8(p);
        }
        w.i32(self.ch3.timer);
        w.u8(self.ch3.pos);
        w.i32(self.ch4.timer);
        w.u16(self.ch4.lfsr);
    }

    pub fn deserialize(&mut self, r: &mut Reader) {
        let mut reg = [0u8; 0x48];
        r.bytes_into(&mut reg);
        self.reg = reg;
        let mut wr = [0u8; 32];
        r.bytes_into(&mut wr);
        self.wave_ram = wr;
        self.master_enable = r.bool();
        self.fs_cycle = r.u32();
        self.fs_step = r.u8();
        self.cycle = r.u64();
        self.a_next = r.u64();
        self.b_next = r.u64();
        self.ds_a = r.u8() as i8;
        self.ds_b = r.u8() as i8;
        self.hp_xl = r.i32();
        self.hp_yl = r.u64() as i64;
        self.hp_xr = r.i32();
        self.hp_yr = r.u64() as i64;
        self.ma_l = r.i32();
        self.ma_r = r.i32();
        for f in [&mut self.fifo_a, &mut self.fifo_b] {
            f.len = r.u8() as usize;
            f.head = r.u8() as usize;
            for s in f.buf.iter_mut() {
                *s = r.u8() as i8;
            }
        }
        // Re-derive channel field state from the restored registers, then patch
        // the running timers/phases we saved explicitly.
        for off in [0x65usize, 0x6D, 0x75, 0x7D, 0x83] {
            self.decode(off);
        }
        self.ch1.timer = r.i32();
        self.ch1.phase = r.u8();
        self.ch2.timer = r.i32();
        self.ch2.phase = r.u8();
        self.ch3.timer = r.i32();
        self.ch3.pos = r.u8();
        self.ch4.timer = r.i32();
        self.ch4.lfsr = r.u16();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn w16(a: &mut Apu, off: u32, v: u16) {
        a.write8(off, v as u8);
        a.write8(off + 1, (v >> 8) as u8);
    }

    #[test]
    fn square_channel_produces_a_waveform() {
        let mut a = Apu::new();
        a.write8(0x84, 0x80); // master enable
        w16(&mut a, 0x80, 0x7777); // full L/R volume, all channels both sides
        w16(&mut a, 0x82, 0x0002); // PSG ratio 100%
        // Channel 2: duty 50%, full volume, mid frequency, trigger.
        w16(&mut a, 0x68, 0xF080); // env: initial 15, no step; duty 2 (bit6-7=10)
        w16(&mut a, 0x6C, 0x8400); // freq 0x400, length disabled, trigger
        assert!(a.ch2.enabled, "channel 2 should be on after trigger");
        a.generate([None, None], 200_000);
        let peak = a.out.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
        assert!(peak > 1000, "square wave should swing, peak was {peak}");
    }

    #[test]
    fn direct_sound_pops_fifo_at_timer_rate() {
        let mut a = Apu::new();
        a.write8(0x84, 0x80);
        // DS A: full volume, both sides, timer 0. (bits: 2=full, 8/9 = R/L)
        w16(&mut a, 0x82, 0x0304);
        // Fill FIFO A with a step: +64 then -64 samples.
        for _ in 0..16 {
            a.write8(0xA0, 64);
        }
        for _ in 0..16 {
            a.write8(0xA0, (-64i8) as u8);
        }
        assert_eq!(a.fifo_a_len(), 32);
        // Timer 0 overflows every 512 cycles => one pop per output sample.
        a.generate([Some(512), None], 512 * 40);
        assert!(a.fifo_a_len() < 32, "FIFO should have drained");
        let peak = a.out.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
        assert!(peak > 1000, "Direct Sound should be audible, peak {peak}");
    }

    #[test]
    fn master_disable_silences_output() {
        let mut a = Apu::new();
        a.write8(0x84, 0x00); // master off
        w16(&mut a, 0x68, 0xF080);
        w16(&mut a, 0x6C, 0x8400);
        a.generate([None, None], 100_000);
        assert!(a.out.iter().all(|&s| s == 0), "no output while master disabled");
    }
}
