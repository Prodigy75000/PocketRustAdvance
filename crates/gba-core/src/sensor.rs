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
/// If Twisted still feels twitchy now that the direction is right, this is the
/// knob, and TH-Android's temporary gyro logging measures the peak rate the
/// owner's hands actually produce.
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
        self.kind == CartSensor::Gyro
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
                self.tilt_x = clamp12(TILT_CENTER_X + (self.in_accel_x * TILT_SPAN_X as f32) as i32);
                self.tilt_y = clamp12(TILT_CENTER_Y + (self.in_accel_y * TILT_SPAN_Y as f32) as i32);
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

        let x = s.tilt_read(0x0E00_8200).unwrap() as u16
            | ((s.tilt_read(0x0E00_8300).unwrap() as u16 & 0x0F) << 8);
        let y = s.tilt_read(0x0E00_8400).unwrap() as u16
            | ((s.tilt_read(0x0E00_8500).unwrap() as u16 & 0x0F) << 8);
        assert!(x > TILT_CENTER_X as u16, "tilted one way reads above centre");
        assert!(y < TILT_CENTER_Y as u16, "tilted the other reads below centre");
        // GBATEK's ranges are the bound worth holding: outside them the cart
        // would be producing values a real one never does.
        assert!((0x2AF..=0x477).contains(&x), "X outside the documented range");
        assert!((0x2C3..=0x480).contains(&y), "Y outside the documented range");

        // A cart without the sensor must not answer in the SRAM window, which
        // is where its save memory lives.
        let plain = Sensors::new(&rom_with_code(b"AZWE"));
        assert_eq!(plain.tilt_read(0x0E00_8200), None);
    }
}
