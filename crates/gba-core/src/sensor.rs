// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Cartridge motion sensors: the Z-axis gyro in WarioWare Twisted, and the
//! X/Y tilt sensor in Yoshi Topsy-Turvy and Koro Koro Puzzle.
//!
//! Both are cartridge hardware rather than console hardware, so which one (if
//! either) exists is a property of the ROM. GBATEK documents the first
//! character of the game code at header 0xAC as identifying exactly this:
//!
//! ```text
//! K  Yoshi and Koro Koro Puzzle   (acceleration sensor)
//! R  WarioWare Twisted            (rumble and z-axis gyro sensor)
//! U  Boktai 1 and 2               (RTC and solar sensor)
//! V  Drill Dozer                  (rumble)
//! ```
//!
//! That is a better test than scanning the ROM for strings: one byte at a fixed
//! offset, and it does not fire on WarioWare Mega Microgames (code AZWE), which
//! shares most of a name with Twisted but carries no sensor at all.

/// Which motion sensor, if any, the cartridge carries.
#[derive(Clone, Copy, PartialEq, Eq, Debug, Default)]
pub enum CartSensor {
    #[default]
    None,
    /// Single-axis gyro measuring rotation about Z, read as a serial ADC over
    /// the GPIO port. WarioWare Twisted.
    Gyro,
    /// Two-axis accelerometer mapped into the top half of the SRAM window.
    /// Yoshi Topsy-Turvy, Yoshi's Universal Gravitation, Koro Koro Puzzle.
    Tilt,
}

impl CartSensor {
    /// Identify the cartridge's sensor from the game code's first character.
    ///
    /// This is a Nintendo CONVENTION on retail codes, not a hardware field, so
    /// anything that does not follow retail conventions can collide with it.
    /// Measured across 6131 ROMs on 2026-09-30, every distinct R code present:
    ///
    /// ```text
    /// RZWE  WARIOTWISTED   WarioWare Twisted (USA)        real gyro
    /// RZWJ  MAWARUWARIO    Mawaru Made in Wario (Japan)   real gyro
    /// RARE  BATTLETOADS    Battletoads (USA) (Proto)      NOT a gyro cart
    /// RARE  DK-PILOT       Diddy Kong Pilot (Proto)       NOT a gyro cart
    /// RKTG  Jetpack 2      homebrew                       NOT a gyro cart
    /// ```
    ///
    /// So the genuine gyro carts are RZW-something, and the false positives have
    /// one shared cause worth knowing: `RARE` is Rare's own name used as a
    /// placeholder game code in their unfinished builds, and it begins with R by
    /// coincidence. Any Rare prototype will look like a gyro cart here.
    ///
    /// The owner ran a 2005 Diddy Kong Pilot prototype and confirmed the
    /// helicopter is on the D-pad with no motion control at all. It is not an
    /// unfinished gyro game, it was never a gyro cart.
    ///
    /// NARROWING THIS IS NOT WORTH IT, and one plausible guard was measured and
    /// rejected rather than assumed: the maker code at 0xB0 does NOT discriminate,
    /// because it reads '01' for all nine of the above, false positives included.
    /// Rare's prototypes carry Nintendo's maker code and even the homebrew sets
    /// it. The alternative is a title list, which is the exact thing this check
    /// exists to avoid and which goes stale on the first new dump.
    ///
    /// The cost of a false positive is bounded and was verified: these three are
    /// byte-identical before and after gyro changes because they never touch the
    /// GPIO. On the Android side it costs one sensor listener for a ROM that will
    /// not read it. It would stop being harmless if a user-visible motion prompt
    /// were ever gated on this byte, because then a false positive becomes a lie
    /// on screen rather than a little battery.
    pub fn detect(rom: &[u8]) -> Self {
        match rom.get(0xAC) {
            Some(b'R') => CartSensor::Gyro,
            Some(b'K') => CartSensor::Tilt,
            _ => CartSensor::None,
        }
    }
}

// --- Calibration -------------------------------------------------------------

// Tilt centres and span, from GBATEK's measurements of a real cart: "X ranged
// between 0x2AF to 0x477, center at 0x392. Y ranged between 0x2C3 to 0x480,
// center at 0x3A0."
const TILT_CENTER_X: i32 = 0x392;
const TILT_CENTER_Y: i32 = 0x3A0;
// Half-span PER AXIS, taken from the narrower side of that axis's documented
// range so a full-scale host input cannot run past what the cart produces.
// The two axes genuinely differ: X sits 0xE3 above its floor and 0xE5 below its
// ceiling, Y sits 0xDD and 0xE0. Using one span for both pushed Y two counts
// under the documented floor at full tilt, which the range assertion caught.
const TILT_SPAN_X: i32 = 0xE3;
const TILT_SPAN_Y: i32 = 0xDD;

/// Which way round the tilt cart sees each axis. X positive, Y NEGATIVE, and
/// both measured on device rather than reasoned out.
///
/// These are per-axis and not one handedness flip, because the two axes were
/// established by different evidence and one of them was wrong for a month
/// without anything noticing.
///
/// X is positive and verified. The owner checked Yoshi Topsy-Turvy's side tilt
/// in portrait against footage of the real game: correct, including the detail
/// that Yoshi leans OPPOSITE to the tilt, which is the game keeping him upright
/// with respect to real gravity rather than a sign error.
///
/// Y is negative, and this is the 2026-09-30 fix. Koro Koro Puzzle is the FIRST
/// game in the corpus that asks for vertical tilt at all: Yoshi's gameplay is
/// side tilt and Twisted is single-axis gyro by construction, so this axis had
/// never been under test on any title. The owner's report, with the game's own
/// kataMuki prompt showing a downward arrow on screen: "I have to tilt my GBA
/// forward but what works for me is the opposite."
///
/// Tilting the phone's top edge away from you reduces Android's accelerometer Y
/// (upright reads +1g on Y, flat reads 0), so forward tilt has to move this ADC
/// UP, which is what the negation does.
///
/// This also reconciles mGBA, which had looked like it disagreed with us on X.
/// Its cart/gpio.c computes `0x3A0 - (x >> 22)` while its libretro port feeds
/// `ACCELEROMETER_X * 3e8` and `ACCELEROMETER_Y * -3e8`, so net of both steps it
/// does NOT negate X and DOES negate Y. Relative to raw Android device axes that
/// is exactly this pair. The earlier discrepancy was our reading of only half of
/// its pipeline, not a real disagreement.
const TILT_SIGN_X: f32 = 1.0;
const TILT_SIGN_Y: f32 = -1.0;

// Gyro centre and half-span, matched to mGBA.
//
// GBATEK gives the gyro's wiring and its bitstream but no resting value and no
// range, so these cannot come from a spec. They now come from the one reference
// implementation that consumes the SAME libretro sensor interface we do, rather
// than from a guess: mGBA's cart/gpio.c centres the sample on 0x700 ("Normalize
// to ~12 bits, focused on 0x700"), and its libretro port scales rad/s by -5.5e8
// before an arithmetic shift right of 21, which is 262.26 ADC counts per rad/s.
//
// The half-span is the distance from the centre DOWN to zero, because that is
// the side that runs out first: 0x700 counts below the centre is the floor,
// while 0x8FF remain above it. Sizing the span to the narrow side means a
// full-scale input cannot underflow the ADC.
//
// The previous 0x6C0 / 0x500 pair was an unmeasured guess and gave 160 counts
// per rad/s, so every rotation under-read by roughly a third. That matters more
// than it looks: Twisted integrates rate into an angle, so a scale error here
// does not just feel wrong, it accumulates.
const GYRO_CENTER: i32 = 0x700;
const GYRO_SPAN: i32 = 0x700;

/// Which way round the cartridge sees a rotation. NEGATIVE, and no longer a
/// guess.
///
/// This was +1.0, and the comment here admitted it was untested. It was wrong.
/// WarioWare Twisted's microgames ran mirrored: the owner reported tilting the
/// phone left and every microgame action going right, checked against footage of
/// the real game. Its menus felt correct, which is why this took a while to
/// believe, and that reading is still unexplained.
///
/// Two independent lines give the same answer.
///
/// The behaviour. mgba#3511 records real hardware from someone comparing
/// against a GBA: "tilting the console right would make the airplane in the
/// airplane microgame go right". Android TYPE_GYROSCOPE is right-handed about
/// the device axes with +Z out of the screen, so tilting the right edge down is
/// CLOCKWISE seen from the front, and therefore NEGATIVE z. For that to deflect
/// the cart the way real hardware does, negative z has to read ABOVE centre,
/// which is this sign and not the old one.
///
/// The code. mGBA's libretro port does the same negation explicitly, scaling
/// GYROSCOPE_Z by -5.5e8. It reads the identical libretro interface in the
/// identical units, so the disagreement was ours.
///
/// mgba#3511 is itself a DIFFERENT bug that happens to share the symptom;
/// endrift traced it to libogc on Wii. It is cited for the hardware behaviour it
/// records, not as a diagnosis of this one.
///
/// THE LIMIT OF THE EVIDENCE, worth stating because it is thinner than it looks:
/// only WarioWare Twisted and its Japanese release use this sensor, and they are
/// one engine. Diddy Kong Pilot carries an R game code but is not a gyro cart at
/// all, see [`CartSensor::detect`], so the second-engine confirmation this sign
/// deserves does not exist in the corpus. It rests on one game plus mGBA.
///
/// The bridge deliberately publishes unrotated device axes and leaves cartridge
/// coordinate conversion to the core, so the negation belongs HERE and not in
/// the bridge.
const GYRO_SIGN: f32 = -1.0;

/// Rotation rate, in rad/s, that deflects the sensor fully.
///
/// No longer a feel choice. With [`GYRO_SPAN`] this sets the scale to 0x700 over
/// 6.83, which is 262.3 ADC counts per rad/s and matches mGBA's 5.5e8 >> 21.
/// Full scale therefore lands where the ADC actually runs out rather than where
/// a guess put it: 6.83 rad/s, about 391 degrees per second.
///
/// mGBA reaches its own limit earlier and by accident, because 5.5e8 times a
/// rate past about 3.9 rad/s overflows the int32 it keeps the sample in. We
/// clamp instead, which keeps its scale without the wrap. A wrap would read as
/// a sudden direction reversal on a hard flick, which is the symptom we were
/// already chasing, so it is worth not reproducing.
///
/// MEASURED ON DEVICE 2026-09-30 and deliberately left alone. TH-Android logged
/// 238 seconds of the owner actually playing Twisted on an S25 Ultra, 11,324
/// samples over 119 windows:
///
/// ```text
/// median window peak |z|   1.20 rad/s
/// p90 window peak |z|      5.34
/// session max |z|          9.52
/// samples past this clamp  41 of 11,324, 0.36%, in 8 of 119 windows
/// ```
///
/// So ordinary play sits well inside the range and the ADC is not going unused.
/// The tail does clip: his hardest flick was 39% past full scale, and because
/// Twisted integrates rate into an angle, a clipped sample is rotation lost from
/// the angle rather than one momentarily wrong reading.
///
/// Widening it anyway would cost more than it buys, for a reason that is about
/// the reference rather than about feel. At mGBA's 262 counts per rad/s the ADC
/// floor IS 6.83, because the centre sits at 0x700 with 1792 counts beneath it.
/// Reaching 9.52 means either abandoning that scale or moving that centre, so the
/// choice is between a number taken from a working implementation and a number
/// fitted to the top 0.36% of one play session. The whole reason the sign was
/// wrong for as long as it was is that its constants were fitted rather than
/// referenced, so this stays referenced.
///
/// Raise it only if someone reports LOST ROTATION during normal play, which is
/// what saturation would actually feel like, rather than on the strength of a
/// peak figure. The owner's verdict on this build was that both Twisted and Yoshi
/// work in both orientations.
const GYRO_RATE_FULL_SCALE: f32 = 6.83;

fn clamp12(v: i32) -> u16 {
    v.clamp(0, 0x0FFF) as u16
}

// --- The device --------------------------------------------------------------

/// Cartridge sensor state: the GPIO port, the gyro's serial shifter, and the
/// tilt ADC.
#[derive(Clone)]
pub struct Sensors {
    pub kind: CartSensor,

    /// The cartridge clock, if this cart has one. It shares the GPIO port with
    /// the gyro and no cartridge carries both, so the port routes to whichever
    /// is present.
    pub rtc: crate::rtc::Rtc,
    /// Combined state of the four GPIO lines for the clock path. The gyro path
    /// keeps its own `data`/`dir` bookkeeping and is left alone.
    rtc_pins: u16,
    /// Last value the CPU wrote to the data register, re-applied when it
    /// changes a line's direction.
    rtc_latch: u16,

    // GPIO port at 0x080000C4/C6/C8. Only the low 4 bits of each are wired.
    data: u16,
    dir: u16,
    /// Bit 0: 0 = the port reads back as ROM (zeroes), 1 = readable.
    ctrl: u16,

    /// Serial shifter for the gyro: 4 dummy zero bits, then 12 data bits, MSB
    /// first, then endless zeroes.
    gyro_shifter: u16,
    gyro_bit: u32,
    /// Latched at each Start Conversion so a sample cannot tear mid-stream.
    gyro_sample: u16,

    /// Tilt ADC outputs, 12 bits each, and whether a conversion has finished.
    tilt_x: u16,
    tilt_y: u16,
    tilt_ready: bool,

    /// Diagnostics: how many times the game has started a conversion. Zero
    /// after a run means the game never talked to the sensor, which is the
    /// difference between "the sensor is wrong" and "the sensor is not wired".
    pub conversions: u64,
    /// Diagnostics: how many axis bytes (tilt) or serial bits (gyro) the game
    /// has read back.
    pub reads: std::cell::Cell<u64>,

    /// Live host input, normalised to roughly -1.0 ..= 1.0.
    in_gyro_z: f32,
    in_accel_x: f32,
    in_accel_y: f32,
}

impl Sensors {
    pub fn new(rom: &[u8]) -> Self {
        Sensors {
            kind: CartSensor::detect(rom),
            rtc: crate::rtc::Rtc::new(rom),
            rtc_pins: 0,
            rtc_latch: 0,
            data: 0,
            dir: 0,
            ctrl: 0,
            gyro_shifter: 0,
            gyro_bit: 0,
            gyro_sample: clamp12(GYRO_CENTER),
            tilt_x: clamp12(TILT_CENTER_X),
            tilt_y: clamp12(TILT_CENTER_Y),
            tilt_ready: true,
            conversions: 0,
            reads: std::cell::Cell::new(0),
            in_gyro_z: 0.0,
            in_accel_x: 0.0,
            in_accel_y: 0.0,
        }
    }

    /// True when this cartridge has GPIO hardware worth decoding at 0x080000C4.
    pub fn has_gpio(&self) -> bool {
        self.kind == CartSensor::Gyro || self.rtc.present
    }

    /// True when the top of the SRAM window is the tilt ADC rather than save
    /// memory. The tilt carts all save to EEPROM, so the window is free.
    pub fn has_tilt(&self) -> bool {
        self.kind == CartSensor::Tilt
    }

    // --- Host input ----------------------------------------------------------

    /// Rotation rate about Z, in rad/s.
    ///
    /// The UNIT is now settled: TH-Android's cartridge bridge publishes Android
    /// TYPE_GYROSCOPE through with no conversion and no axis remap, and Android
    /// documents that as rad/s about the device axes. So this matches its
    /// source, unlike the accelerometer, which turned out to be g and not the
    /// m/s^2 that looked obvious.
    ///
    /// [`GYRO_SIGN`] and the [`GYRO_SPAN`] / [`GYRO_RATE_FULL_SCALE`] pair are
    /// no longer guesses. Both are matched to mGBA, which reads this same
    /// libretro interface in these same units; see those constants.
    pub fn set_gyroscope(&mut self, _x: f32, _y: f32, z: f32) {
        self.in_gyro_z = (GYRO_SIGN * z / GYRO_RATE_FULL_SCALE).clamp(-1.0, 1.0);
    }

    /// Accelerometer as specific force in **g**, which is the unit the
    /// front-end reports and not m/s^2.
    ///
    /// This started out dividing by 9.81, on the assumption that the value
    /// arrived in m/s^2 the way Android's own SensorEvent does. It does not:
    /// PocketRust's `gb-libretro/src/sensor.rs` states the contract outright,
    /// "the frontend reports specific force in g", and its liveness threshold
    /// is a figure measured on a real tablet in those units (0.00293 g of
    /// jitter at rest). That core's tilt is known to work on device, so it is
    /// the authority here. Dividing again would have scaled every tilt down by
    /// about ten and read as "the sensor does nothing".
    ///
    /// One g is a device resting fully on that edge, which is the most tilt a
    /// player can express, so g maps straight onto full scale.
    pub fn set_accelerometer(&mut self, x: f32, y: f32, _z: f32) {
        self.in_accel_x = x.clamp(-1.0, 1.0);
        self.in_accel_y = y.clamp(-1.0, 1.0);
    }

    fn sample_gyro(&self) -> u16 {
        clamp12(GYRO_CENTER + (self.in_gyro_z * GYRO_SPAN as f32) as i32)
    }

    // --- GPIO port (gyro carts) ----------------------------------------------

    /// Read one of the GPIO half-words. Returns None when the port is in
    /// write-only mode, in which case the caller serves ROM as usual.
    pub fn gpio_read(&self, offset: u32) -> Option<u16> {
        if !self.has_gpio() || self.ctrl & 1 == 0 {
            return None;
        }
        if self.rtc.present {
            // The clock drives its lines when the port is written, so a read is
            // just the latched bus state.
            return Some(match offset {
                0xC4 => self.rtc_pins & 0xF,
                0xC6 => self.dir & 0xF,
                0xC8 => self.ctrl & 1,
                _ => 0,
            });
        }
        Some(match offset {
            0xC4 => {
                // Serial data appears on bit 2, and only while that line is
                // configured as an input. Output lines read back what was
                // written to them.
                let mut v = self.data & self.dir & 0xF;
                if self.dir & 0b100 == 0 {
                    self.reads.set(self.reads.get() + 1);
                    let bit = if self.gyro_bit < 16 {
                        (self.gyro_shifter >> (15 - self.gyro_bit)) & 1
                    } else {
                        0
                    };
                    v |= bit << 2;
                }
                v
            }
            0xC6 => self.dir & 0xF,
            0xC8 => self.ctrl & 1,
            _ => 0,
        })
    }

    /// Write one of the GPIO half-words. Returns true when the port consumed it.
    pub fn gpio_write(&mut self, offset: u32, val: u16) -> bool {
        if !self.has_gpio() {
            return false;
        }
        if self.rtc.present {
            // Writing either the data or the direction register re-drives the
            // CPU-owned lines and then clocks the chip, which is the only thing
            // that advances its state machine.
            match offset {
                0xC4 => {
                    self.rtc_latch = val & 0xF;
                    self.rtc_pins &= !self.dir;
                    self.rtc_pins |= self.rtc_latch & self.dir;
                    self.rtc.read_pins(&mut self.rtc_pins, self.dir);
                }
                0xC6 => {
                    self.dir = val & 0xF;
                    self.rtc_pins &= !self.dir;
                    self.rtc_pins |= self.rtc_latch & self.dir;
                    self.rtc.read_pins(&mut self.rtc_pins, self.dir);
                }
                0xC8 => self.ctrl = val & 1,
                _ => {}
            }
            return true;
        }
        match offset {
            0xC4 => {
                let prev = self.data;
                self.data = val & 0xF;
                // Only lines configured as outputs can drive the sensor.
                let driven = self.data & self.dir;
                let was = prev & self.dir;

                // Bit 0 rising: start a conversion. The sample is latched here
                // so a value arriving from the front-end mid-stream cannot tear
                // the 12 bits the game is part-way through clocking out.
                if driven & 1 != 0 && was & 1 == 0 {
                    self.conversions += 1;
                    self.gyro_sample = self.sample_gyro();
                    // 4 dummy zero bits then 12 data bits, MSB first. The
                    // dummies fall out for free by seating the sample in the
                    // low 12 bits of a 16-bit shifter.
                    self.gyro_shifter = self.gyro_sample & 0x0FFF;
                    self.gyro_bit = 0;
                }
                // Bit 1 rising: clock the next bit out.
                if driven & 0b10 != 0 && was & 0b10 == 0 {
                    self.gyro_bit = self.gyro_bit.saturating_add(1);
                }
                true
            }
            0xC6 => {
                self.dir = val & 0xF;
                true
            }
            0xC8 => {
                self.ctrl = val & 1;
                true
            }
            _ => false,
        }
    }

    // --- Tilt sensor (Yoshi / Koro Koro) -------------------------------------

    /// The tilt ADC occupies 0x0E008000..=0x0E0085FF. GBATEK:
    ///
    /// ```text
    /// E008000h (W) Write 55h to start sampling
    /// E008100h (W) Write AAh to start sampling
    /// E008200h (R) Lower 8 bits of X axis
    /// E008300h (R) Upper 4 bits of X, and bit7: ADC status (0=Busy, 1=Ready)
    /// E008400h (R) Lower 8 bits of Y axis
    /// E008500h (R) Upper 4 bits of Y axis
    /// ```
    pub fn tilt_read(&self, addr: u32) -> Option<u8> {
        if !self.has_tilt() {
            return None;
        }
        let hit = match addr & 0xFFFF {
            0x8200 | 0x8300 | 0x8400 | 0x8500 => true,
            _ => false,
        };
        if hit {
            self.reads.set(self.reads.get() + 1);
        }
        match addr & 0xFFFF {
            0x8200 => Some(self.tilt_x as u8),
            0x8300 => {
                Some(((self.tilt_x >> 8) as u8 & 0x0F) | if self.tilt_ready { 0x80 } else { 0 })
            }
            0x8400 => Some(self.tilt_y as u8),
            0x8500 => Some((self.tilt_y >> 8) as u8 & 0x0F),
            _ => None,
        }
    }

    /// Returns true when the tilt ADC consumed the write.
    pub fn tilt_write(&mut self, addr: u32, val: u8) -> bool {
        if !self.has_tilt() {
            return false;
        }
        match (addr & 0xFFFF, val) {
            (0x8000, 0x55) => {
                // First half of the two-write sampling handshake. Both axes are
                // latched here so the pair a game reads always comes from one
                // instant, and the status bit reads Busy until the second write
                // completes the handshake.
                self.tilt_x =
                    clamp12(TILT_CENTER_X + (TILT_SIGN_X * self.in_accel_x * TILT_SPAN_X as f32) as i32);
                self.tilt_y =
                    clamp12(TILT_CENTER_Y + (TILT_SIGN_Y * self.in_accel_y * TILT_SPAN_Y as f32) as i32);
                self.tilt_ready = false;
                self.conversions += 1;
                true
            }
            (0x8100, 0xAA) => {
                self.tilt_ready = true;
                true
            }
            // Anything else inside the ADC window is absorbed rather than
            // falling through to save memory, which does not live here.
            (0x8000..=0x85FF, _) => true,
            _ => false,
        }
    }

    /// Diagnostics: the latched axis values, so a caller can confirm host
    /// input is reaching the ADC rather than only that the game asked for it.
    pub fn debug_axes(&self) -> (u16, u16) {
        (self.tilt_x, self.tilt_y)
    }

    /// Diagnostics: the latched gyro sample and how far through its 16-bit
    /// stream the game has clocked.
    pub fn debug_gyro(&self) -> (u16, u32) {
        (self.gyro_sample, self.gyro_bit)
    }

    // --- Save state ----------------------------------------------------------

    pub fn serialize(&self, w: &mut crate::state::Writer) {
        w.u16(self.data);
        w.u16(self.dir);
        w.u16(self.ctrl);
        w.u16(self.gyro_shifter);
        w.u32(self.gyro_bit);
        w.u16(self.gyro_sample);
        w.u16(self.tilt_x);
        w.u16(self.tilt_y);
        w.bool(self.tilt_ready);
    }

    /// The clock block is appended at the very end of the save state rather
    /// than folded in here, so a state written before the clock existed still
    /// loads. `unix_time` is deliberately NOT stored: it is the wall clock, the
    /// front-end pushes it every frame, and restoring a stale one would make a
    /// loaded save think no time had passed.
    pub fn serialize_rtc(&self, w: &mut crate::state::Writer) {
        w.u16(self.rtc_pins);
        w.u16(self.rtc_latch);
        self.rtc.serialize(w);
    }

    pub fn deserialize_rtc(&mut self, r: &mut crate::state::Reader) {
        self.rtc_pins = r.u16();
        self.rtc_latch = r.u16();
        self.rtc.deserialize(r);
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        self.data = r.u16();
        self.dir = r.u16();
        self.ctrl = r.u16();
        self.gyro_shifter = r.u16();
        self.gyro_bit = r.u32();
        self.gyro_sample = r.u16();
        self.tilt_x = r.u16();
        self.tilt_y = r.u16();
        self.tilt_ready = r.bool();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom_with_code(code: &[u8; 4]) -> Vec<u8> {
        let mut rom = vec![0u8; 0x1000];
        rom[0xAC..0xB0].copy_from_slice(code);
        rom
    }

    /// The whole feature hangs off one byte, and the cost of getting it wrong
    /// is asymmetric: a missed sensor is a game that ignores tilt, while a
    /// false positive puts GPIO hardware in front of a cartridge that has none
    /// and can break a game that was working.
    #[test]
    fn sensor_kind_comes_from_the_game_code() {
        assert_eq!(CartSensor::detect(&rom_with_code(b"RZWE")), CartSensor::Gyro); // Twisted
        assert_eq!(CartSensor::detect(&rom_with_code(b"KYGE")), CartSensor::Tilt); // Yoshi
        assert_eq!(CartSensor::detect(&rom_with_code(b"KHPJ")), CartSensor::Tilt); // Koro Koro
        // Mega Microgames is not Twisted and has no sensor.
        assert_eq!(CartSensor::detect(&rom_with_code(b"AZWE")), CartSensor::None);
        // Boktai and Drill Dozer carry other hardware, not one of these two.
        assert_eq!(CartSensor::detect(&rom_with_code(b"U3IE")), CartSensor::None);
        assert_eq!(CartSensor::detect(&rom_with_code(b"V49E")), CartSensor::None);
        // A ROM too short to have a header must not panic or guess.
        assert_eq!(CartSensor::detect(&[0u8; 4]), CartSensor::None);
    }

    /// Clock out a full stream the way the game does and check the shape
    /// GBATEK describes: four dummy zeroes, then the 12 sample bits MSB first.
    fn clock_out(s: &mut Sensors) -> u16 {
        // All three lines as outputs except data (bit 2), which is the input.
        s.gpio_write(0xC6, 0b1011);
        s.gpio_write(0xC8, 1); // make the port readable
        s.gpio_write(0xC4, 0b0001); // Start Conversion, rising
        s.gpio_write(0xC4, 0b0000);
        let mut got = 0u16;
        for i in 0..16 {
            let bit = (s.gpio_read(0xC4).unwrap() >> 2) & 1;
            if i < 4 {
                assert_eq!(bit, 0, "bit {i} should be one of the four dummy zeroes");
            } else {
                got = (got << 1) | bit;
            }
            s.gpio_write(0xC4, 0b0010); // clock rising
            s.gpio_write(0xC4, 0b0000);
        }
        got
    }

    #[test]
    fn gyro_clocks_out_twelve_bits_after_four_dummies() {
        let mut s = Sensors::new(&rom_with_code(b"RZWE"));
        // Absolute numbers throughout, never GYRO_CENTER plus or minus
        // something. The version of this test that shipped the inverted sign
        // asserted against clamp12(GYRO_CENTER), so it moved with the constant
        // it was meant to check, and its two labels were the wrong way round:
        // it fed +100.0, which is anticlockwise, and called it clockwise. It
        // passed under either sign, which is the only reason it passed at all.
        assert_eq!(clock_out(&mut s), 0x700, "at rest the ADC sits at 0x700");

        // Tilting the console RIGHT reads ABOVE centre.
        //
        // Android TYPE_GYROSCOPE is right-handed about the device axes with +Z
        // out of the screen, so the right edge going down is clockwise seen from
        // the front, which is NEGATIVE z. mgba#3511 records what real hardware
        // does: "tilting the console right would make the airplane in the
        // airplane microgame go right".
        s.set_gyroscope(0.0, 0.0, -100.0);
        let right = clock_out(&mut s);
        s.set_gyroscope(0.0, 0.0, 100.0);
        let left = clock_out(&mut s);
        assert!(right > 0x700, "console tilted right reads above 0x700, got {right:#x}");
        assert!(left < 0x700, "console tilted left reads below 0x700, got {left:#x}");
        assert!(right <= 0x0FFF && left <= 0x0FFF, "12-bit ADC cannot exceed 0xFFF");

        // Scale, against mGBA's figure rather than against our own constants:
        // its libretro port maps rad/s to counts as 5.5e8 >> 21, so one rad/s
        // is 262.26 counts of deflection. This fails if GYRO_SPAN or
        // GYRO_RATE_FULL_SCALE is changed without meaning to.
        s.set_gyroscope(0.0, 0.0, -1.0);
        let one_rad = clock_out(&mut s) as i32 - 0x700;
        assert!(
            (one_rad - 262).abs() <= 2,
            "1 rad/s should deflect about 262 counts (mGBA 5.5e8 >> 21), got {one_rad}"
        );
    }

    #[test]
    fn gpio_stays_invisible_until_the_game_enables_it() {
        let mut s = Sensors::new(&rom_with_code(b"RZWE"));
        // Control bit 0 clear is write-only mode: reads fall through to ROM.
        assert_eq!(s.gpio_read(0xC4), None);
        s.gpio_write(0xC8, 1);
        assert!(s.gpio_read(0xC4).is_some());

        // A cartridge without the hardware never answers, no matter what is
        // written, so a plain game cannot have its ROM reads intercepted.
        let mut plain = Sensors::new(&rom_with_code(b"AZWE"));
        assert!(!plain.gpio_write(0xC8, 1));
        assert_eq!(plain.gpio_read(0xC4), None);
    }

    /// Read the latched 12-bit pair the way a game does, low byte then the
    /// four high bits.
    fn read_axes(s: &Sensors) -> (u16, u16) {
        let x = s.tilt_read(0x0E00_8200).unwrap() as u16
            | ((s.tilt_read(0x0E00_8300).unwrap() as u16 & 0x0F) << 8);
        let y = s.tilt_read(0x0E00_8400).unwrap() as u16
            | ((s.tilt_read(0x0E00_8500).unwrap() as u16 & 0x0F) << 8);
        (x, y)
    }

    #[test]
    fn tilt_samples_on_the_handshake_and_reports_ready() {
        let mut s = Sensors::new(&rom_with_code(b"KYGE"));
        // One g, i.e. resting fully on that edge. NOT 9.81: the front-end
        // reports specific force in g, so passing m/s^2 here would clamp and
        // hide a ten-fold scaling error rather than catch it.
        s.set_accelerometer(1.0, -1.0, 0.0);

        // Sampling is started by the documented pair of writes.
        assert!(s.tilt_write(0x0E00_8000, 0x55));
        // Between the two writes the ADC reports Busy.
        assert_eq!(s.tilt_read(0x0E00_8300).unwrap() & 0x80, 0);
        assert!(s.tilt_write(0x0E00_8100, 0xAA));
        assert_eq!(s.tilt_read(0x0E00_8300).unwrap() & 0x80, 0x80, "ready bit");

        let (x, y) = read_axes(&s);

        // Absolute numbers, and GBATEK's rather than ours. The previous version
        // of these two asserts was written as "above TILT_CENTER_X" and "below
        // TILT_CENTER_Y", which moves with the constants it is checking and,
        // worse, encoded the Y direction that turned out to be inverted. It
        // would have passed under either sign.
        //
        // DIRECTIONS COME FROM THE DEVICE, see TILT_SIGN_X and TILT_SIGN_Y.
        // +1g on X reads high; -1g on Y must ALSO read high, because tilting
        // the phone forward reduces Android's Y and has to move this ADC up.
        assert_eq!(x, 0x475, "+1g on X reads high, just under GBATEK 0x477 ceiling");
        assert_eq!(y, 0x47D, "-1g on Y must read ABOVE centre, not below it");

        // The other corner, which lands exactly on GBATEK's documented floors:
        // "X ranged between 0x2AF to 0x477" and "Y ranged between 0x2C3 to
        // 0x480". Full scale is supposed to reach the extremes a real cart
        // produces and go no further.
        s.set_accelerometer(-1.0, 1.0, 0.0);
        assert!(s.tilt_write(0x0E00_8000, 0x55));
        assert!(s.tilt_write(0x0E00_8100, 0xAA));
        let (x2, y2) = read_axes(&s);
        assert_eq!(x2, 0x2AF, "-1g on X lands on GBATEK's documented X floor");
        assert_eq!(y2, 0x2C3, "+1g on Y lands on GBATEK's documented Y floor");

        // A cart without the sensor must not answer in the SRAM window, which
        // is where its save memory lives.
        let plain = Sensors::new(&rom_with_code(b"AZWE"));
        assert_eq!(plain.tilt_read(0x0E00_8200), None);
    }
}
