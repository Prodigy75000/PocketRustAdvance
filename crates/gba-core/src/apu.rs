// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

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

/// DAC steps to i16: the GBA's mixer output is 10 bits (0x400 steps, centred by
/// SOUNDBIAS), so 0x8000 / 0x200 = 64 maps one DAC step to 64 of i16 and makes
/// the hardware's full scale exactly i16's full scale.
///
/// **This is not a volume knob, it is the only non-arbitrary value.** It used to
/// be 24, which put our output 8.5 dB under gpSP for the same emulated mixer
/// state, and that is what the owner heard as "not as loud as gpSP". Measured
/// against both reference cores on an isolated PSG tone: gpSP uses 64, mGBA uses
/// 48 (it scales by a user volume of 0x100 * 3 >> 4). Anything below 64 throws
/// away output range the hardware was using.
///
/// **Do not reduce this to buy headroom for the DC blocker.** That was measured:
/// across eleven commercial titles the only one whose pre-clamp peak exceeds the
/// i16 rail is Metroid Zero Mission, at 1.05x, and it is also the only one that
/// clips the emulated DAC at all (172,094 times, where the other ten clip zero).
/// It saturates the console's own mixer, which is why gpSP rails on it too. Going
/// to 48 would cost every other game 2.5 dB to soften one game that distorts on
/// hardware, and quiet output is the complaint this constant exists to fix.
/// `dbg_peak` and `dbg_dac_clips` are the two counters that tell those apart.
const DAC_TO_I16: i32 = 64;

/// SOUNDBIAS reset value. Bits 0-9 are the bias level that shifts the signed mix
/// into the DAC's unsigned 10-bit window, and 0x200 centres it.
///
/// It has to be non-zero at reset or direct boot is silent-ish: with bias 0 the
/// clip in [`Apu::dac`] takes every negative sample to zero. Real hardware resets
/// this register to 0x200 and the BIOS writes it again; we direct-boot most of
/// the corpus, so nothing else would set it.
const SOUNDBIAS_RESET: u16 = 0x0200;

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
    /// Channel output with no DC component, scaled by 8 so that subtracting the
    /// waveform's mean stays exact in integers. The mixer converts to DAC steps.
    ///
    /// The obvious form, `(if high { vol } else { 0 }) - 8`, is the DMG DAC taken
    /// literally: digital zero sits on the bottom rail. It is also why this core
    /// sounded saturated once the output stage was scaled correctly. A channel
    /// enabled at volume 0 held a full -8, three of them stepped the mix by 384
    /// of the DAC's 512 negative steps, and the DC blocker then spent 125 ms
    /// bleeding it off with the whole signal riding near the rail. Measured worst
    /// standing offset 60% of full scale, against 12% for gpSP and mGBA.
    ///
    /// Subtracting the waveform's own mean (`k` high slots out of 8, times the
    /// volume) removes the offset at the source and leaves peak-to-peak exactly
    /// as it was, so volume 0 is silent and an envelope sweep no longer sweeps a
    /// DC level. That is also the shape gpSP produces, by scaling a bipolar
    /// pattern by the envelope instead.
    fn dac(&self) -> i32 {
        if !self.enabled || !self.dac_on {
            return 0;
        }
        let pat = DUTY[self.duty as usize];
        let high = pat & (1 << self.phase) != 0;
        let vol = self.env.vol as i32;
        (if high { vol * 8 } else { 0 }) - pat.count_ones() as i32 * vol
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
        // Scaled by 8, like the other channels. The volume has to scale the
        // CENTRED sample, not the raw nibble: shifting the nibble first drags
        // the whole waveform toward the bottom rail, so a perfectly centred wave
        // at 50% volume carried a DC of about -4.5 steps for no reason. 75% is a
        // GBA-only forced level and applies the same way.
        let centred = (nib as i32 - 8) * 8;
        if self.force75 {
            centred * 3 / 4
        } else {
            centred >> (self.volume - 1)
        }
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
        // DC-free and scaled by 8, as in Square::dac. The LFSR sits high about half
        // the time, so the mean is half the volume.
        let high = self.lfsr & 1 == 0;
        let vol = self.env.vol as i32;
        (if high { vol * 8 } else { 0 }) - 4 * vol
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
    /// The sample the next pop will return, without consuming it. Direct Sound
    /// reconstruction needs to see one sample ahead to interpolate towards it.
    fn peek(&self) -> Option<i8> {
        if self.len == 0 { None } else { Some(self.buf[self.head]) }
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
    /// Cycles between Direct Sound pops, as of the last pop, so the mixer can
    /// work out how far it is between two PCM samples. Re-derived on every pop, so
    /// it is deliberately NOT part of the save state.
    a_period: u32,
    b_period: u32,
    /// `(1 << 24) / period`, so the mixer can find its position between two PCM
    /// samples with a multiply instead of a divide. The mixer runs four times per
    /// output sample per channel, and a divide there was the only measurable cost
    /// of interpolating at all.
    a_recip: u32,
    b_recip: u32,

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

    // Debug counters (cumulative): FIFO pops that had data, and pops that found
    // the FIFO empty (underruns), summed across both Direct Sound channels.
    pub dbg_pops: u64,
    pub dbg_underruns: u64,
    /// Largest sample magnitude seen BEFORE the i16 clamp.
    ///
    /// The mixer's own ceiling is 511 DAC steps, so anything above 32704 here was
    /// put there by the DC blocker, not by the emulated hardware. That is the
    /// headroom the output stage has to leave: see the note on [`DAC_TO_I16`].
    pub dbg_peak: i32,
    /// Times the emulated DAC's own clip bit. This is the one the console would
    /// have done too, so it separates a game that really does saturate its mixer
    /// from our own post-mixer processing overshooting.
    pub dbg_dac_clips: u64,
}

impl Default for Apu {
    fn default() -> Self {
        let mut reg = [0u8; 0x48];
        reg[0x88 - 0x60..0x8A - 0x60].copy_from_slice(&SOUNDBIAS_RESET.to_le_bytes());
        Apu {
            reg,
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
            a_period: 0,
            b_period: 0,
            a_recip: 0,
            b_recip: 0,
            hp_xl: 0,
            hp_yl: 0,
            hp_xr: 0,
            hp_yr: 0,
            ma_l: 0,
            ma_r: 0,
            cycle: 0,
            out: Vec::new(),
            dbg_pops: 0,
            dbg_underruns: 0,
            dbg_peak: 0,
            dbg_dac_clips: 0,
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

    /// The APU's current position on the (fixed-rate) audio clock.
    pub fn cycle(&self) -> u64 {
        self.cycle
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
            self.dbg_peak = self.dbg_peak.max(out_l.abs()).max(out_r.abs());
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
            _ => {
                // Timer stopped: no pops, and no interpolation either, or the
                // mixer would ramp towards a sample that is never going to arrive.
                if is_b { self.b_period = 0 } else { self.a_period = 0 }
                return;
            }
        };
        let recip = ((1u64 << 24) / period) as u32;
        if is_b {
            self.b_period = period as u32;
            self.b_recip = recip;
        } else {
            self.a_period = period as u32;
            self.a_recip = recip;
        }
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
                self.dbg_pops += 1;
            } else {
                self.dbg_underruns += 1;
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

    fn mix_raw(&mut self) -> (i32, i32) {
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
        // The channel DACs hand back the waveform scaled by 8, and this converts
        // to the mixer's unit of an eighth of a DAC step: one channel at full
        // volume ends up spanning 240 DAC steps, so four of them at 100% reach 960
        // of the DAC's 1024 steps. That is the same range one Direct Sound channel
        // covers and it is the relationship to keep. The net weight used to be half
        // that, which measured exactly 6 dB under gpSP and mGBA on an isolated
        // tone.
        //
        // PSG ratio is bits 0-1 of SOUNDCNT_H (00=25%, 01=50%, 10=100%); 11 is
        // prohibited and is treated as 100%, which is what we did before.
        let shift = match cnt_h & 3 {
            0 => 3,
            1 => 2,
            _ => 1,
        };
        psg_l = (psg_l * (left_vol + 1) * 4) >> shift;
        psg_r = (psg_r * (right_vol + 1) * 4) >> shift;

        // Direct Sound: 8-bit signed PCM, at 100% or 50% volume (bits 2/3).
        let a = self.ds_level(false) >> if cnt_h & 0x04 != 0 { 0 } else { 1 };
        let b = self.ds_level(true) >> if cnt_h & 0x08 != 0 { 0 } else { 1 };
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
        (self.dac(l), self.dac(r))
    }

    /// The output stage: SOUNDBIAS, then the DAC's 10 bits, then i16.
    ///
    /// Hardware offsets the signed mix by the bias level, truncates it to the
    /// DAC's unsigned 10-bit range, and that clip is where a loud game's mix
    /// actually distorts. We had no clip at this point at all, only the final i16
    /// one, so we clipped roughly 2.7x later than the console did and every game
    /// paid for that headroom in volume. Clipping here, per sub-sample before the
    /// oversample average, is also where the console does it.
    fn dac(&mut self, mix: i32) -> i32 {
        // Everything above is in eighths of a DAC step, so the bias and the clip
        // window scale by 8 too. Clipping in the fine unit rather than rounding
        // first is what keeps the Direct Sound interpolation's resolution.
        let bias = (self.r16(0x88) & 0x3FF) as i32 * 8;
        let biased = mix + bias;
        let clipped = biased.clamp(0, 0x3FF * 8 + 7);
        if clipped != biased {
            self.dbg_dac_clips += 1;
        }
        (clipped - bias) * (DAC_TO_I16 / 8)
    }

    /// One Direct Sound channel at 100% volume, in eighths of a DAC step,
    /// LINEARLY INTERPOLATED towards the sample that will be popped next.
    ///
    /// Holding each PCM byte until the next pop is a zero-order hold, and its
    /// images fold back as broadband hiss. Measured between the harmonics at
    /// 4-10 kHz, holding put our noise floor 5.3 dB above gpSP's, which the owner
    /// heard as the output being grainy. gpSP interpolates the same way; this is
    /// the single thing it does to the FIFO stream that we did not.
    ///
    /// Degrades to a hold whenever there is nothing to interpolate towards: an
    /// empty FIFO, or a stopped timer. Ramping towards a sample that never arrives
    /// would be worse than the hold.
    fn ds_level(&self, is_b: bool) -> i32 {
        let (cur, at, period, recip, next) = if is_b {
            (self.ds_b, self.b_next, self.b_period, self.b_recip, self.fifo_b.peek())
        } else {
            (self.ds_a, self.a_next, self.a_period, self.a_recip, self.fifo_a.peek())
        };
        // 32 eighths per PCM step: a full-amplitude sample reaches 4096, which is
        // the 512 DAC steps one Direct Sound channel covers at 100%.
        let cur = cur as i32 * 32;
        let (Some(next), true) = (next, period > 0) else {
            return cur;
        };
        // How far through the current PCM sample we are, as 0..256.
        let left = at.saturating_sub(self.cycle).min(period as u64) as u32;
        let frac = (((period - left) as u64 * recip as u64) >> 16) as i32;
        cur + ((next as i32 * 32 - cur) * frac >> 8)
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
        // A version-1 state was captured when SOUNDBIAS was stored but never
        // read, and no game writes it (it relies on the hardware reset value),
        // so that copy is a zero. Honouring it would clip the whole negative half
        // of the mix to silence: measured max 12226 / min -176 on a LeafGreen
        // battle state before this migration existed.
        if r.version < 2 {
            reg[0x88 - 0x60..0x8A - 0x60].copy_from_slice(&SOUNDBIAS_RESET.to_le_bytes());
        }
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

    fn peak(a: &Apu) -> u16 {
        a.out.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0)
    }

    /// Absolute output LEVEL, not just "it swings".
    ///
    /// This test replaces a `peak > 1000` assertion that passed both before and
    /// after the output stage was 2.7x too quiet, which is how a core that was
    /// 10 dB under gpSP kept a green suite. The band comes from the hardware and
    /// not from our own constants: one PSG channel at full volume and 100% ratio
    /// swings 15 of the GBA DAC's 1024 steps times the x16 step weight, so 240
    /// of 2048 peak-to-peak, which is 23.4% of i16 peak-to-peak. The peak lands
    /// between 7680 (once the DC blocker has centred the asymmetric +7/-8 swing)
    /// and 8192 (before it has).
    ///
    /// A 24x output stage reads 1446 here and a 48x one reads 5760, so both the
    /// old value and mGBA's are outside the band.
    #[test]
    fn one_psg_channel_at_full_volume_is_a_quarter_of_full_scale() {
        let mut a = Apu::new();
        a.write8(0x84, 0x80); // master enable
        w16(&mut a, 0x80, 0x7777); // full L/R volume, all channels both sides
        w16(&mut a, 0x82, 0x0002); // PSG ratio 100%
        // Channel 2: duty 50%, full volume, mid frequency, trigger.
        w16(&mut a, 0x68, 0xF080); // env: initial 15, no step; duty 2 (bit6-7=10)
        w16(&mut a, 0x6C, 0x8400); // freq 0x400, length disabled, trigger
        assert!(a.ch2.enabled, "channel 2 should be on after trigger");
        a.generate([None, None], 200_000);
        let p = peak(&a);
        assert!((7000..=8400).contains(&p), "one PSG channel should peak near 8192, got {p}");
    }

    /// Halving the PSG ratio has to halve the output, because that is the only
    /// thing the ratio field does. Catches a ratio table that is off by a shift,
    /// which is a mistake the absolute test above cannot see on its own.
    #[test]
    fn the_psg_ratio_field_halves_the_level_each_step_down() {
        let level = |ratio: u16| {
            let mut a = Apu::new();
            a.write8(0x84, 0x80);
            w16(&mut a, 0x80, 0x7777);
            w16(&mut a, 0x82, ratio);
            w16(&mut a, 0x68, 0xF080);
            w16(&mut a, 0x6C, 0x8400);
            a.generate([None, None], 200_000);
            peak(&a) as i32
        };
        let (full, half, quarter) = (level(2), level(1), level(0));
        assert!(
            (full - half * 2).abs() <= full / 16,
            "50% should be half of 100%: {half} vs {full}"
        );
        assert!(
            (full - quarter * 4).abs() <= full / 16,
            "25% should be a quarter of 100%: {quarter} vs {full}"
        );
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
        let p = peak(&a);
        assert!(p > 1000, "Direct Sound should be audible, peak {p}");
    }

    /// One Direct Sound channel at 100% covers the WHOLE DAC range, so a FIFO
    /// carrying full-amplitude PCM has to reach i16's full scale. This is the
    /// level that matters in practice: the GBA sound driver mixes a game's music
    /// and samples in software and pushes the result through Direct Sound, so
    /// almost every commercial GBA game is this path and nothing else.
    ///
    /// A 24x output stage reads 12192 here, which is the 8.5 dB the owner heard.
    #[test]
    fn one_direct_sound_channel_at_full_amplitude_reaches_full_scale() {
        let mut a = Apu::new();
        a.write8(0x84, 0x80);
        w16(&mut a, 0x82, 0x0304); // DS A at 100%, both sides, timer 0
        // A 1 kHz square at full 8-bit amplitude: 16 pops high, 16 pops low, one
        // pop per output sample. Alternating per sample instead would be at the
        // output Nyquist and the mixer's smoothing would legitimately null it.
        for _ in 0..8 {
            for _ in 0..16 {
                a.write8(0xA0, 127);
            }
            for _ in 0..16 {
                a.write8(0xA0, (-128i8) as u8);
            }
            a.generate([Some(512), None], a.cycle + 512 * 32);
        }
        let p = peak(&a);
        assert!(p > 30_000, "full-amplitude Direct Sound should reach full scale, got {p}");
    }

    /// A channel that is enabled but silent must contribute NOTHING.
    ///
    /// This is the defect the owner heard as saturation once the output stage was
    /// scaled correctly: an enabled channel at volume 0 used to hold -8 DAC steps,
    /// three of them stepped the mix by 384 of the 512 available negative steps,
    /// and the DC blocker then spent 125 ms bleeding it off with the whole signal
    /// riding the rail. Checked on the mixer directly, because the DC blocker
    /// hides a STEADY offset and a test on the output stream would pass either
    /// way.
    #[test]
    fn an_enabled_psg_channel_at_volume_zero_adds_nothing_to_the_mix() {
        let mut a = Apu::new();
        a.write8(0x84, 0x80);
        w16(&mut a, 0x80, 0x7777);
        w16(&mut a, 0x82, 0x0002);
        // Volume 0, envelope direction up, period 0: the DAC is on (bit 11 set)
        // and the envelope never moves, so the channel is enabled and silent.
        w16(&mut a, 0x68, 0x0880);
        w16(&mut a, 0x6C, 0x8400);
        assert!(a.ch2.enabled && a.ch2.dac_on, "the channel must be enabled with its DAC on");
        assert_eq!(a.ch2.env.vol, 0, "and silent");
        assert_eq!(a.mix_raw(), (0, 0), "a silent channel must not offset the mix");
    }

    /// No duty cycle may shift the mix off centre. The 12.5% duty is the worst
    /// case and the one that proves it: seven of its eight slots are low, so the
    /// old bottom-anchored form left a standing -6.1 steps per channel at full
    /// volume, before any master volume multiplied it up.
    #[test]
    fn no_square_duty_leaves_a_standing_offset() {
        for duty in 0..4u8 {
            let mut a = Apu::new();
            a.write8(0x84, 0x80);
            w16(&mut a, 0x80, 0x7777);
            w16(&mut a, 0x82, 0x0002);
            w16(&mut a, 0x68, 0xF000 | (duty as u16) << 6);
            w16(&mut a, 0x6C, 0x8400);
            let sum: i32 = (0..8)
                .map(|p| {
                    a.ch2.phase = p;
                    a.mix_raw().0
                })
                .sum();
            let peak = (0..8)
                .map(|p| {
                    a.ch2.phase = p;
                    a.mix_raw().0.abs()
                })
                .max()
                .unwrap();
            assert!(peak > 1000, "duty {duty} should still produce a waveform, peak {peak}");
            assert_eq!(sum, 0, "duty {duty} left a standing offset over one period");
        }
    }

    /// The wave channel's volume has to scale the CENTRED sample. Shifting the raw
    /// nibble first drags the waveform toward the bottom rail, so a wave whose own
    /// mean is dead centre picked up a DC of about -4.5 steps at 50% volume purely
    /// from the volume control.
    #[test]
    fn the_wave_channel_volume_does_not_drag_the_waveform_off_centre() {
        for volume in 1..4u16 {
            let mut a = Apu::new();
            a.write8(0x84, 0x80);
            w16(&mut a, 0x80, 0x7777);
            w16(&mut a, 0x82, 0x0002);
            // Nibbles alternating 1 and 15, so the waveform's own mean is exactly
            // the centre value 8 and any offset found is the volume control's.
            for i in 0..16 {
                a.write8(0x90 + i, 0x1F);
            }
            w16(&mut a, 0x70, 0x0080); // NR30: DAC on, 32-sample mode, bank 0
            w16(&mut a, 0x72, volume << 13); // NR32: volume select
            w16(&mut a, 0x74, 0x8400); // trigger
            assert!(a.ch3.enabled && a.ch3.dac_on, "wave channel enabled at volume {volume}");
            let sum: i32 = (0..32)
                .map(|p| {
                    a.ch3.pos = p;
                    a.mix_raw().0
                })
                .sum();
            let peak = (0..32)
                .map(|p| {
                    a.ch3.pos = p;
                    a.mix_raw().0.abs()
                })
                .max()
                .unwrap();
            assert!(peak > 500, "volume {volume} should produce a waveform, peak {peak}");
            assert_eq!(sum, 0, "volume {volume} dragged a centred waveform off centre");
        }
    }

    /// SOUNDBIAS has to come up centred. With a bias of zero the clip in
    /// `dac` takes every negative sample to silence, and since most of the corpus
    /// direct-boots there is no BIOS write to rescue it.
    #[test]
    fn soundbias_resets_to_the_centred_value() {
        let a = Apu::new();
        let v = a.read8(0x88) as u16 | (a.read8(0x89) as u16) << 8;
        assert_eq!(v, 0x0200, "SOUNDBIAS must reset centred, not to zero");
    }

    /// The clip is at the DAC, where SOUNDBIAS put it, not at i16.
    ///
    /// Moving the bias down to 0x100 moves the floor from -0x200 to -0x100, so
    /// the same signal keeps its positive peak and loses half its negative one.
    /// Without the bias clip both runs would be identical, which is what makes
    /// this falsifiable rather than decorative.
    #[test]
    fn the_bias_level_decides_where_the_negative_half_clips() {
        let run = |bias: u16| {
            let mut a = Apu::new();
            a.write8(0x84, 0x80);
            w16(&mut a, 0x88, bias);
            w16(&mut a, 0x82, 0x0304); // DS A at 100%, both sides, timer 0
            for _ in 0..16 {
                a.write8(0xA0, 127);
            }
            for _ in 0..16 {
                a.write8(0xA0, (-128i8) as u8);
            }
            // Short window: the DC blocker has a 4096-sample time constant, so
            // over 32 samples it cannot disguise the asymmetry under test.
            a.generate([Some(512), None], 512 * 32);
            let hi = a.out.iter().copied().max().unwrap_or(0) as i32;
            let lo = a.out.iter().copied().min().unwrap_or(0) as i32;
            (hi, lo)
        };
        let (hi_centred, lo_centred) = run(0x0200);
        let (hi_low, lo_low) = run(0x0100);
        assert!(lo_centred < -30_000, "centred bias should reach the floor, got {lo_centred}");
        assert!(
            lo_low > -20_000,
            "a bias of 0x100 should clip the negative half at about -16384, got {lo_low}"
        );
        assert!(
            (hi_centred - hi_low).abs() < hi_centred / 8,
            "lowering the bias must not change the positive peak: {hi_centred} vs {hi_low}"
        );
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