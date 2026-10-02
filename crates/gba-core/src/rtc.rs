// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Seiko S-3511A real-time clock, the cartridge chip Pokemon Ruby, Sapphire and
//! Emerald, every Boktai, Legendz, Rockman EXE 4.5 and Sennen Kazoku carry.
//!
//! It hangs off the same three-wire GPIO port at 0x080000C4 as the gyro, and no
//! cartridge has both, so the port routes to one or the other.
//!
//! Without it those games boot and play and quietly never grow a berry, never
//! change tide and never reach night. That is a worse failure than a black
//! screen, because nothing on screen says anything is wrong.
//!
//! ## The wire protocol
//!
//! Three lines in the low bits of the GPIO data register: bit 0 SCK, bit 1 SIO,
//! bit 2 CS. CS low aborts any transfer and is the idle state. While CS is high,
//! a bit is sampled off SIO whenever SCK is low and committed on SCK's rising
//! edge, **least significant bit first**.
//!
//! The first byte of a transfer is the command:
//!
//! ```text
//!   bit 0-3  magic, must be 0x6, else the byte is ignored
//!   bit 4-6  command
//!   bit 7    1 = the game reads the following bytes, 0 = it writes them
//! ```
//!
//! Commands and their payload length: 0 force-reset (0 bytes), 2 date and time
//! (7), 3 force-IRQ (0), 4 control register (1), 6 time only (3). Everything
//! else is empty. The time payload is the last three bytes of the date-and-time
//! payload, which is why both index one shared buffer from the end.
//!
//! All payload bytes are BCD. Date and time is year (since 2000), month, day,
//! day of week, hour, minute, second.

/// Payload length in bytes for each of the eight command codes.
const COMMAND_BYTES: [i32; 8] = [
    0, // 0: force reset
    0, // 1: unused
    7, // 2: date and time
    0, // 3: force IRQ
    1, // 4: control register
    0, // 5: unused
    3, // 6: time
    0, // 6: unused
];

const CMD_RESET: u8 = 0;
const CMD_DATETIME: u8 = 2;
const CMD_FORCE_IRQ: u8 = 3;
const CMD_CONTROL: u8 = 4;
const CMD_TIME: u8 = 6;

/// A cartridge real-time clock. Inert unless `present`.
#[derive(Clone, Default)]
pub struct Rtc {
    /// True when the cartridge carries the chip. See [`detect`].
    pub present: bool,

    /// Control register. Bit 3 per-minute IRQ, bit 6 24-hour mode, bit 7 power
    /// off. Only bit 6 changes what we hand back.
    control: u8,
    /// The seven BCD bytes latched at the start of a read command, so a clock
    /// that ticks mid-transfer cannot tear the value the game is clocking out.
    time: [u8; 7],

    bits: u8,
    bits_read: u32,
    bytes_remaining: i32,
    command: u8,
    command_active: bool,
    /// SCK as of the previous pin update, so a rising edge can be seen.
    sck_edge: bool,
    sio_output: bool,

    /// Wall-clock seconds since the Unix epoch, **already shifted into the
    /// player's local time by the front-end**. This core has no timezone
    /// database and is not going to grow one: the Android side knows the user's
    /// zone and the desktop runner can ask the host, so the conversion belongs
    /// there. Zero means nobody has set it, and [`DEFAULT_UNIX_TIME`] stands in
    /// so a game never sees 1970.
    pub unix_time: i64,

    /// Diagnostics: how many times a game has latched the clock. Zero after a
    /// run is the difference between "the clock is wrong" and "the game never
    /// asked", which look identical from the outside.
    pub latches: u64,
}

/// Stand-in when no front-end has pushed a time: 2026-01-01 00:00:00.
/// Arbitrary, but it is a date these games accept, which 1970 is not for
/// anything that stores a year as two BCD digits since 2000.
pub const DEFAULT_UNIX_TIME: i64 = 1_767_225_600;

/// Does this cartridge carry an S-3511A?
///
/// Nintendo's RTC library leaves its version string in the ROM, and that is the
/// only honest marker: unlike the gyro and tilt carts there is no game-code
/// convention to lean on. Measured across the 2727 licensed ROMs on 2026-10-02,
/// the string appears in exactly 30, and the list is precisely the carts that
/// have the chip: all nine Pokemon Ruby/Sapphire language builds, all seven
/// Emerald builds, every Boktai, all three Legendz, Rockman EXE 4.5 and Sennen
/// Kazoku. Their game codes start with A, B and U, so the first-character trick
/// that identifies gyro and tilt carts cannot work here.
///
/// FireRed and LeafGreen do NOT contain it and do NOT have the chip, which is
/// the check that matters: they are the obvious false positive for any rule
/// phrased as "it is a Pokemon game".
pub fn detect(rom: &[u8]) -> bool {
    rom.windows(8).any(|w| w == b"SIIRTC_V")
}

/// Binary-coded decimal, two digits.
fn bcd(value: u32) -> u8 {
    ((value % 10) + ((value / 10) % 10) * 16) as u8
}

/// Civil date from a day count since 1970-01-01, by Howard Hinnant's algorithm.
/// Exact for any day this clock can reach and needs no lookup tables.
fn civil_from_days(days: i64) -> (i64, u32, u32) {
    let z = days + 719_468;
    let era = if z >= 0 { z } else { z - 146_096 } / 146_097;
    let doe = (z - era * 146_097) as u64; // 0..=146096
    let yoe = (doe - doe / 1460 + doe / 36524 - doe / 146_096) / 365; // 0..=399
    let doy = doe - (365 * yoe + yoe / 4 - yoe / 100); // 0..=365
    let mp = (5 * doy + 2) / 153; // 0..=11
    let d = (doy - (153 * mp + 2) / 5 + 1) as u32; // 1..=31
    let m = if mp < 10 { mp + 3 } else { mp - 9 } as u32; // 1..=12
    let y = yoe as i64 + era * 400 + i64::from(m <= 2);
    (y, m, d)
}

/// Day count since 1970-01-01 for a civil date, the inverse of
/// [`civil_from_days`]. Front-ends need it to turn the host's broken-down local
/// time back into the seconds this clock wants.
pub fn days_from_civil(y: i64, m: u32, d: u32) -> i64 {
    let y = y - i64::from(m <= 2);
    let era = if y >= 0 { y } else { y - 399 } / 400;
    let yoe = (y - era * 400) as u64; // 0..=399
    let mp = if m > 2 { m - 3 } else { m + 9 } as u64; // 0..=11
    let doy = (153 * mp + 2) / 5 + u64::from(d) - 1; // 0..=365
    let doe = yoe * 365 + yoe / 4 - yoe / 100 + doy; // 0..=146096
    era * 146_097 + doe as i64 - 719_468
}

impl Rtc {
    pub fn new(rom: &[u8]) -> Self {
        Rtc { present: detect(rom), sck_edge: true, sio_output: true, ..Default::default() }
    }

    /// Latch the seven BCD bytes from the current wall clock.
    fn latch(&mut self) {
        self.latches += 1;
        let t = if self.unix_time != 0 { self.unix_time } else { DEFAULT_UNIX_TIME };
        let days = t.div_euclid(86_400);
        let secs = t.rem_euclid(86_400) as u32;
        let (year, month, day) = civil_from_days(days);
        // 1970-01-01 was a Thursday, and the chip counts Sunday as 0.
        let weekday = (days + 4).rem_euclid(7) as u32;
        let hour = secs / 3600;
        self.time[0] = bcd((year - 2000).clamp(0, 99) as u32);
        self.time[1] = bcd(month);
        self.time[2] = bcd(day);
        self.time[3] = bcd(weekday);
        self.time[4] = bcd(if self.control & 0x40 != 0 { hour } else { hour % 12 });
        self.time[5] = bcd((secs / 60) % 60);
        self.time[6] = bcd(secs % 60);
    }

    /// The chip drives the lines the cartridge port has configured as inputs;
    /// the ones the CPU drives keep whatever it wrote.
    fn drive(pin_state: &mut u16, dir: u16, pins: u16) {
        *pin_state &= dir;
        *pin_state |= pins & !dir & 0xF;
    }

    /// Clock the chip against the current pin state. Mirrors the sequence the
    /// hardware runs: SCK is sampled low, committed on its rising edge, and the
    /// chip only drives SIO while it is answering a read.
    pub fn read_pins(&mut self, pin_state: &mut u16, dir: u16) {
        // Idle: the chip pulls SCK, CS and the unused line low and leaves SIO.
        Self::drive(pin_state, dir, *pin_state & 2);

        if *pin_state & 4 == 0 {
            // CS low aborts whatever was in flight.
            self.bits_read = 0;
            self.bytes_remaining = 0;
            self.command_active = false;
            self.command = 0;
            self.sck_edge = true;
            self.sio_output = true;
            Self::drive(pin_state, dir, 2);
            return;
        }

        if !self.command_active {
            Self::drive(pin_state, dir, 2);
            self.shift_in(*pin_state);
            if !self.sck_edge && *pin_state & 1 != 0 {
                self.bits_read += 1;
                if self.bits_read == 8 {
                    self.begin_command();
                }
            }
        } else if self.command & 0x80 == 0 {
            // The game is writing the payload.
            Self::drive(pin_state, dir, 2);
            self.shift_in(*pin_state);
            if !self.sck_edge && *pin_state & 1 != 0 {
                // A line that changes exactly on the clock edge loses the race
                // and reads as zero.
                if (self.bits >> self.bits_read) & 1 != ((*pin_state & 2) >> 1) as u8 {
                    self.bits &= !(1 << self.bits_read);
                }
                self.bits_read += 1;
                if self.bits_read == 8 {
                    self.process_byte();
                }
            }
        } else {
            // The chip is answering a read, one bit per falling edge of SCK.
            if self.sck_edge && *pin_state & 1 == 0 {
                self.sio_output = self.output_bit() != 0;
                self.bits_read += 1;
                if self.bits_read == 8 {
                    self.bytes_remaining -= 1;
                    if self.bytes_remaining <= 0 {
                        self.bytes_remaining = COMMAND_BYTES[((self.command >> 4) & 7) as usize];
                    }
                    self.bits_read = 0;
                }
            }
            Self::drive(pin_state, dir, u16::from(self.sio_output) << 1);
        }

        self.sck_edge = *pin_state & 1 != 0;
    }

    fn shift_in(&mut self, pin_state: u16) {
        if pin_state & 1 == 0 {
            self.bits &= !(1 << self.bits_read);
            self.bits |= (((pin_state & 2) >> 1) as u8) << self.bits_read;
        }
    }

    fn begin_command(&mut self) {
        let command = self.bits;
        // A byte whose low nibble is not the magic is not a command at all.
        if command & 0xF == 0x6 {
            self.command = command;
            let code = (command >> 4) & 7;
            self.bytes_remaining = COMMAND_BYTES[code as usize];
            self.command_active = true;
            match code {
                CMD_RESET => self.control = 0,
                CMD_DATETIME | CMD_TIME => self.latch(),
                CMD_FORCE_IRQ | CMD_CONTROL => {}
                _ => {}
            }
        }
        self.bits = 0;
        self.bits_read = 0;
    }

    fn process_byte(&mut self) {
        if (self.command >> 4) & 7 == CMD_CONTROL {
            self.control = self.bits;
        }
        self.bits = 0;
        self.bits_read = 0;
        self.bytes_remaining -= 1;
        if self.bytes_remaining <= 0 {
            self.bytes_remaining = COMMAND_BYTES[((self.command >> 4) & 7) as usize];
        }
    }

    fn output_bit(&self) -> u8 {
        let byte = match (self.command >> 4) & 7 {
            CMD_CONTROL => self.control,
            // The time-only command reads the tail of the same buffer, so both
            // index it from the end by how much is left to send.
            CMD_DATETIME | CMD_TIME => {
                let i = 7 - self.bytes_remaining;
                *self.time.get(i.clamp(0, 6) as usize).unwrap_or(&0xFF)
            }
            _ => 0xFF,
        };
        (byte >> self.bits_read) & 1
    }

    pub fn serialize(&self, w: &mut crate::state::Writer) {
        w.u8(self.control);
        for b in self.time {
            w.u8(b);
        }
        w.u8(self.bits);
        w.u32(self.bits_read);
        w.u32(self.bytes_remaining as u32);
        w.u8(self.command);
        w.bool(self.command_active);
        w.bool(self.sck_edge);
        w.bool(self.sio_output);
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        self.control = r.u8();
        for i in 0..7 {
            self.time[i] = r.u8();
        }
        self.bits = r.u8();
        self.bits_read = r.u32();
        self.bytes_remaining = r.u32() as i32;
        self.command = r.u8();
        self.command_active = r.bool();
        self.sck_edge = r.bool();
        self.sio_output = r.bool();
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom_with(marker: bool) -> Vec<u8> {
        let mut rom = vec![0u8; 0x1000];
        if marker {
            rom[0x800..0x808].copy_from_slice(b"SIIRTC_V");
        }
        rom
    }

    /// A tiny host for the three wires, so a test can speak the protocol the
    /// way a game does rather than poke the struct's innards.
    struct Port {
        rtc: Rtc,
        pin: u16,
        dir: u16,
    }
    impl Port {
        fn new() -> Self {
            // SCK, SIO and CS all driven by the CPU, which is how a game sends
            // a command. SIO flips to an input only to read the answer back.
            let mut p = Port { rtc: Rtc::new(&rom_with(true)), pin: 0, dir: 0b0111 };
            p.set(0b0000); // CS low, idle
            p
        }
        /// Drive the CPU-owned lines and clock the chip.
        fn set(&mut self, value: u16) {
            self.pin &= !self.dir;
            self.pin |= value & self.dir;
            self.rtc.read_pins(&mut self.pin, self.dir);
        }
        fn set_dir(&mut self, dir: u16) {
            self.dir = dir;
        }
        /// One command/payload bit, LSB first: present it with SCK low, then
        /// raise SCK to commit it.
        fn write_bit(&mut self, bit: u16) {
            self.set(0b100 | (bit << 1));
            self.set(0b101 | (bit << 1));
        }
        fn write_byte(&mut self, byte: u8) {
            for i in 0..8 {
                self.write_bit(((byte >> i) & 1) as u16);
            }
        }
        /// Read one byte back. SIO becomes an input for this.
        fn read_byte(&mut self) -> u8 {
            self.set_dir(0b0101); // SIO becomes an input
            let mut out = 0u8;
            for i in 0..8 {
                self.set(0b101); // SCK high
                self.set(0b100); // falling edge: the chip presents a bit
                out |= (((self.pin >> 1) & 1) as u8) << i;
            }
            out
        }
    }

    /// The marker scan is the whole of RTC detection, and both directions cost
    /// something real: a missed clock is a Pokemon save that never grows a
    /// berry, and a false positive puts a chip on the GPIO port of a cartridge
    /// that has none, where a game reading the port expects open bus.
    #[test]
    fn the_clock_is_found_by_its_library_marker_and_not_by_the_game_code() {
        assert!(detect(&rom_with(true)));
        assert!(!detect(&rom_with(false)));
        // The game-code trick that finds gyro and tilt carts must not be
        // reused here: these carts start with A, B and U.
        let mut pokemon = rom_with(true);
        pokemon[0xAC..0xB0].copy_from_slice(b"AXVE");
        assert!(detect(&pokemon), "Pokemon Ruby has the chip and an A game code");
        let mut firered = rom_with(false);
        firered[0xAC..0xB0].copy_from_slice(b"BPRE");
        assert!(!detect(&firered), "FireRed is a Pokemon game with NO clock");
    }

    /// Clock a real date through the wire protocol and read it back. The value
    /// is pinned, so this fails if the BCD packing, the civil-date arithmetic,
    /// the weekday epoch or the bit order move.
    #[test]
    fn the_date_and_time_command_reads_back_the_clock_in_bcd() {
        let mut p = Port::new();
        // 2026-10-02 18:45:07, a Friday. Computed, not guessed: my first try
        // was a day out and the weekday is exactly what would have hidden it.
        p.rtc.unix_time = 1_790_966_707;
        p.rtc.control = 0x40; // 24-hour mode

        p.set(0b001); // CS low, SCK high: idle
        p.set(0b101); // CS high: start
        p.write_byte(0x06 | (CMD_DATETIME << 4) | 0x80); // read date and time

        let got: Vec<u8> = (0..7).map(|_| p.read_byte()).collect();
        assert_eq!(
            got,
            vec![0x26, 0x10, 0x02, 0x05, 0x18, 0x45, 0x07],
            "year 26, month 10, day 02, Friday (5), 18:45:07, all BCD"
        );
        assert!(p.rtc.latches > 0, "the game latched the clock");
    }

    /// Twelve-hour mode is the default and the games that use it would read
    /// 18:00 as 6 if the hour were not folded.
    #[test]
    fn twelve_hour_mode_folds_the_hour() {
        let mut p = Port::new();
        p.rtc.unix_time = 1_790_966_707; // 18:45:07
        p.rtc.control = 0; // 12-hour
        p.set(0b001);
        p.set(0b101);
        p.write_byte(0x06 | (CMD_TIME << 4) | 0x80);
        let got: Vec<u8> = (0..3).map(|_| p.read_byte()).collect();
        assert_eq!(got, vec![0x06, 0x45, 0x07], "18:45:07 reads as 6:45:07");
    }

    /// The control register is writable, and 24-hour mode has to survive into
    /// the next latch or every clock read is twelve hours out half the day.
    #[test]
    fn writing_the_control_register_changes_how_the_hour_is_reported() {
        let mut p = Port::new();
        p.rtc.unix_time = 1_790_966_707; // 18:45:07
        p.set(0b001);
        p.set(0b101);
        p.write_byte(0x06 | (CMD_CONTROL << 4)); // write, not read
        p.write_byte(0x40);
        assert_eq!(p.rtc.control, 0x40);

        p.set(0b001);
        p.set(0b101);
        p.write_byte(0x06 | (CMD_TIME << 4) | 0x80);
        assert_eq!(p.read_byte(), 0x18, "now reports 18, not 6");
    }

    /// The two civil-date conversions must be exact inverses, because the core
    /// uses one and every front-end uses the other. An off-by-one here is a
    /// clock that is a day out, which is exactly the mistake I made writing the
    /// date test below and only caught because the weekday disagreed.
    #[test]
    fn the_civil_date_conversions_round_trip() {
        for days in [-1, 0, 1, 19_632, 20_000, 25_000, 40_000] {
            let (y, m, d) = civil_from_days(days);
            assert_eq!(days_from_civil(y, m, d), days, "{y}-{m}-{d}");
        }
        // Pinned against known dates rather than only against itself.
        assert_eq!(civil_from_days(0), (1970, 1, 1));
        assert_eq!(civil_from_days(20_728), (2026, 10, 2));
        assert_eq!(days_from_civil(2026, 10, 2), 20_728);
        // A leap day, the usual place these break.
        assert_eq!(civil_from_days(days_from_civil(2024, 2, 29)), (2024, 2, 29));
    }

    /// A byte whose low nibble is not the magic is not a command. Without this
    /// any stray toggling of the port would start a transfer.
    #[test]
    fn a_byte_without_the_magic_nibble_is_not_a_command() {
        let mut p = Port::new();
        p.set(0b001);
        p.set(0b101);
        p.write_byte(0x05 | (CMD_DATETIME << 4) | 0x80); // magic 5, not 6
        assert!(!p.rtc.command_active, "no command may be in flight");
        assert_eq!(p.rtc.latches, 0, "and the clock must not have been latched");
    }
}
