//! Cartridge backup memory: SRAM, Flash (64/128 KB) and EEPROM (512 B / 8 KB).
//!
//! The type is detected from the ID string every commercial ROM embeds
//! ("SRAM_Vnnn", "FLASH_Vnnn"/"FLASH512_Vnnn", "FLASH1M_Vnnn", "EEPROM_Vnnn").
//! SRAM is a plain byte array. Flash speaks a small command protocol (unlock
//! sequence + autoselect ID) which games probe at boot — returning nothing is
//! what white-screens Pokémon. EEPROM is a bit-serial protocol the game drives
//! over DMA; we key its address width off the DMA length.

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum SaveKind {
    None,
    Sram,
    Flash64,
    Flash128,
    Eeprom,
}

pub struct Save {
    pub kind: SaveKind,
    pub data: Vec<u8>,
    pub dirty: bool,

    // --- Flash command state ---
    flash_phase: u8,      // 0/1/2 progress through the AA/55/cmd unlock sequence
    flash_id_mode: bool,  // autoselect (device ID) mode
    flash_erase_prep: bool,
    flash_write_pending: bool,
    flash_bank_pending: bool,
    flash_bank: usize, // 0/1 for 128 KB

    // --- EEPROM serial state ---
    ee_addr_bits: u32, // 6 (512 B) or 14 (8 KB), learned from the DMA length
    ee_expect: u32,    // total bits in the current write transfer (from DMA len)
    ee_rx: u128,       // bits received so far (a 14-bit write stream is 81 bits)
    ee_rx_count: u32,
    ee_read: Vec<bool>, // queued bits to return (4 dummy + 64 data)
    ee_read_pos: usize,
}

impl Save {
    pub fn detect(rom: &[u8]) -> Save {
        let kind = detect_kind(rom);
        let size = match kind {
            SaveKind::Sram => 32 * 1024,
            SaveKind::Flash64 => 64 * 1024,
            SaveKind::Flash128 => 128 * 1024,
            SaveKind::Eeprom => 8 * 1024, // hold the largest; width set at runtime
            SaveKind::None => 0,
        };
        Save {
            kind,
            data: vec![0xFF; size], // erased flash/SRAM reads as 0xFF
            dirty: false,
            flash_phase: 0,
            flash_id_mode: false,
            flash_erase_prep: false,
            flash_write_pending: false,
            flash_bank_pending: false,
            flash_bank: 0,
            ee_addr_bits: 6,
            ee_expect: 0,
            ee_rx: 0,
            ee_rx_count: 0,
            ee_read: Vec::new(),
            ee_read_pos: 0,
        }
    }

    pub fn is_eeprom(&self) -> bool {
        self.kind == SaveKind::Eeprom
    }

    pub fn serialize(&self, w: &mut crate::state::Writer) {
        w.u8(self.flash_phase);
        w.bool(self.flash_id_mode);
        w.bool(self.flash_erase_prep);
        w.bool(self.flash_write_pending);
        w.bool(self.flash_bank_pending);
        w.u32(self.flash_bank as u32);
        w.u32(self.ee_addr_bits);
        w.u32(self.ee_expect);
        w.u128(self.ee_rx);
        w.u32(self.ee_rx_count);
        w.u32(self.ee_read_pos as u32);
        // Fixed-size queue (<= 68 bits) so the whole state has a stable length.
        w.u32(self.ee_read.len() as u32);
        for i in 0..68 {
            w.u8(self.ee_read.get(i).map(|&b| b as u8).unwrap_or(0));
        }
        w.bytes(&self.data);
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        self.flash_phase = r.u8();
        self.flash_id_mode = r.bool();
        self.flash_erase_prep = r.bool();
        self.flash_write_pending = r.bool();
        self.flash_bank_pending = r.bool();
        self.flash_bank = r.u32() as usize;
        self.ee_addr_bits = r.u32();
        self.ee_expect = r.u32();
        self.ee_rx = r.u128();
        self.ee_rx_count = r.u32();
        self.ee_read_pos = r.u32() as usize;
        let n = r.u32() as usize;
        let mut queue = Vec::with_capacity(n.min(68));
        for i in 0..68 {
            let b = r.bool();
            if i < n {
                queue.push(b);
            }
        }
        self.ee_read = queue;
        let data = r.bytes_vec();
        let m = data.len().min(self.data.len());
        self.data[..m].copy_from_slice(&data[..m]);
    }

    // --- SRAM / Flash region (0x0E000000-0x0E00FFFF) ---------------------------

    pub fn read(&self, addr: u32) -> u8 {
        let off = (addr & 0xFFFF) as usize;
        match self.kind {
            SaveKind::Sram => *self.data.get(off & 0x7FFF).unwrap_or(&0xFF),
            SaveKind::Flash64 | SaveKind::Flash128 => {
                if self.flash_id_mode {
                    let (man, dev) = self.flash_id();
                    match off {
                        0 => man,
                        1 => dev,
                        _ => 0,
                    }
                } else {
                    *self.data.get(self.flash_bank * 0x1_0000 + off).unwrap_or(&0xFF)
                }
            }
            _ => 0xFF,
        }
    }

    pub fn write(&mut self, addr: u32, val: u8) {
        let off = (addr & 0xFFFF) as usize;
        match self.kind {
            SaveKind::Sram => {
                if let Some(b) = self.data.get_mut(off & 0x7FFF) {
                    *b = val;
                    self.dirty = true;
                }
            }
            SaveKind::Flash64 | SaveKind::Flash128 => self.flash_write(off, val),
            _ => {}
        }
    }

    fn flash_id(&self) -> (u8, u8) {
        // Manufacturer, device. IDs games recognise for each size.
        if self.kind == SaveKind::Flash128 {
            (0x62, 0x13) // Sanyo 1 Mbit
        } else {
            (0x32, 0x1B) // Panasonic 512 Kbit
        }
    }

    fn flash_write(&mut self, off: usize, val: u8) {
        if self.flash_write_pending {
            let a = self.flash_bank * 0x1_0000 + off;
            if let Some(b) = self.data.get_mut(a) {
                *b = val;
                self.dirty = true;
            }
            self.flash_write_pending = false;
            return;
        }
        if self.flash_bank_pending {
            self.flash_bank = (val & 1) as usize;
            self.flash_bank_pending = false;
            return;
        }
        // The reset command (0xF0) returns to read mode from any phase, and is
        // usually issued as a single write (no unlock sequence).
        if val == 0xF0 {
            self.flash_id_mode = false;
            self.flash_phase = 0;
            return;
        }

        match self.flash_phase {
            0 if off == 0x5555 && val == 0xAA => self.flash_phase = 1,
            1 if off == 0x2AAA && val == 0x55 => self.flash_phase = 2,
            2 => {
                if off == 0x5555 {
                    match val {
                        0x90 => self.flash_id_mode = true,   // enter autoselect
                        0x80 => self.flash_erase_prep = true, // erase prepare
                        0xA0 => self.flash_write_pending = true, // program a byte
                        0xB0 => self.flash_bank_pending = true,  // set bank (128 KB)
                        0x10 if self.flash_erase_prep => {
                            self.data.fill(0xFF); // chip erase
                            self.dirty = true;
                            self.flash_erase_prep = false;
                        }
                        _ => {}
                    }
                } else if val == 0x30 && self.flash_erase_prep {
                    // Erase the 4 KB sector at `off` in the active bank.
                    let base = self.flash_bank * 0x1_0000 + (off & 0xF000);
                    for b in self.data.iter_mut().skip(base).take(0x1000) {
                        *b = 0xFF;
                    }
                    self.dirty = true;
                    self.flash_erase_prep = false;
                }
                self.flash_phase = 0;
            }
            _ => self.flash_phase = 0,
        }
    }

    // --- EEPROM (bit-serial over DMA, region 0x0D) -----------------------------

    /// A DMA touching the EEPROM has begun; its length reveals the transfer kind
    /// and (first time) the address width. 9/73 halfwords => 6-bit (512 B);
    /// 17/81 => 14-bit (8 KB). 68 is a data read-out.
    pub fn eeprom_set_dma_len(&mut self, len: u32) {
        match len {
            9 | 73 => self.ee_addr_bits = 6,
            17 | 81 => self.ee_addr_bits = 14,
            _ => {}
        }
        if len != 68 {
            // Start of a fresh command stream.
            self.ee_expect = len;
            self.ee_rx = 0;
            self.ee_rx_count = 0;
        }
    }

    /// Consume one command bit (bit 0 of a halfword written to EEPROM).
    pub fn eeprom_write_bit(&mut self, bit: u8) {
        self.ee_rx = (self.ee_rx << 1) | (bit as u128 & 1);
        self.ee_rx_count += 1;
        if self.ee_expect != 0 && self.ee_rx_count >= self.ee_expect {
            self.eeprom_finish();
            self.ee_expect = 0;
            self.ee_rx = 0;
            self.ee_rx_count = 0;
        }
    }

    fn eeprom_finish(&mut self) {
        let n = self.ee_rx_count;
        let bits = self.ee_rx;
        let cmd = (bits >> (n - 2)) & 0b11;
        let ab = self.ee_addr_bits;
        // Address follows the 2 command bits.
        let addr = ((bits >> (n - 2 - ab)) & ((1u128 << ab) - 1)) as usize;
        let block = (addr & (self.data.len() / 8 - 1)) * 8;
        if cmd == 0b11 {
            // Read: queue 4 dummy bits then the 64 data bits, MSB first.
            self.ee_read.clear();
            for _ in 0..4 {
                self.ee_read.push(false);
            }
            for i in 0..64 {
                let byte = self.data.get(block + i / 8).copied().unwrap_or(0);
                self.ee_read.push(byte & (0x80 >> (i % 8)) != 0);
            }
            self.ee_read_pos = 0;
        } else if cmd == 0b10 {
            // Write: the 64 bits after the address are the data (then a stop bit).
            let data = ((bits >> 1) & ((1u128 << 64) - 1)) as u64;
            for i in 0..8 {
                if let Some(b) = self.data.get_mut(block + i) {
                    *b = (data >> (56 - i * 8)) as u8;
                }
            }
            self.dirty = true;
        }
    }

    /// Produce one bit for a halfword read from EEPROM.
    pub fn eeprom_read_bit(&mut self) -> u8 {
        if self.ee_read_pos < self.ee_read.len() {
            let b = self.ee_read[self.ee_read_pos];
            self.ee_read_pos += 1;
            b as u8
        } else {
            1 // ready / idle
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn rom_with(id: &[u8]) -> Vec<u8> {
        let mut rom = vec![0u8; 0x400];
        rom[0x200..0x200 + id.len()].copy_from_slice(id); // word-aligned
        rom
    }

    #[test]
    fn detects_each_type() {
        assert_eq!(Save::detect(&rom_with(b"SRAM_V112")).kind, SaveKind::Sram);
        assert_eq!(Save::detect(&rom_with(b"FLASH_V124")).kind, SaveKind::Flash64);
        assert_eq!(Save::detect(&rom_with(b"FLASH1M_V103")).kind, SaveKind::Flash128);
        assert_eq!(Save::detect(&rom_with(b"EEPROM_V124")).kind, SaveKind::Eeprom);
        assert_eq!(Save::detect(&[0u8; 0x400]).kind, SaveKind::None);
    }

    #[test]
    fn flash_id_and_program() {
        let mut s = Save::detect(&rom_with(b"FLASH1M_V103"));
        // Autoselect: AA/55/90, then read manufacturer + device.
        s.write(0x0E00_5555, 0xAA);
        s.write(0x0E00_2AAA, 0x55);
        s.write(0x0E00_5555, 0x90);
        assert_eq!(s.read(0x0E00_0000), 0x62); // Sanyo manufacturer
        assert_eq!(s.read(0x0E00_0001), 0x13); // device
        s.write(0x0E00_5555, 0xF0); // leave autoselect
        // Program a byte: AA/55/A0, then the data at its address.
        s.write(0x0E00_5555, 0xAA);
        s.write(0x0E00_2AAA, 0x55);
        s.write(0x0E00_5555, 0xA0);
        s.write(0x0E00_1234, 0x42);
        assert_eq!(s.read(0x0E00_1234), 0x42);
    }

    #[test]
    fn eeprom_write_read_roundtrip() {
        let mut s = Save::detect(&rom_with(b"EEPROM_V124"));
        let addr = 5u64;
        let value = 0x0123_4567_89AB_CDEFu64;

        // Write stream (14-bit): [1,0] + addr(14) + data(64) + stop.
        s.eeprom_set_dma_len(81);
        for &b in &[1u8, 0] {
            s.eeprom_write_bit(b);
        }
        for i in (0..14).rev() {
            s.eeprom_write_bit(((addr >> i) & 1) as u8);
        }
        for i in (0..64).rev() {
            s.eeprom_write_bit(((value >> i) & 1) as u8);
        }
        s.eeprom_write_bit(0); // stop

        // Read request: [1,1] + addr(14) + stop.
        s.eeprom_set_dma_len(17);
        for &b in &[1u8, 1] {
            s.eeprom_write_bit(b);
        }
        for i in (0..14).rev() {
            s.eeprom_write_bit(((addr >> i) & 1) as u8);
        }
        s.eeprom_write_bit(0);

        // Read 68 bits: 4 dummy, then the 64 data bits MSB first.
        s.eeprom_set_dma_len(68);
        for _ in 0..4 {
            assert_eq!(s.eeprom_read_bit(), 0);
        }
        let mut got = 0u64;
        for _ in 0..64 {
            got = (got << 1) | s.eeprom_read_bit() as u64;
        }
        assert_eq!(got, value);
    }
}

fn detect_kind(rom: &[u8]) -> SaveKind {
    // The ID strings are word-aligned in commercial ROMs; scan 4-byte boundaries.
    let has = |needle: &[u8]| {
        let n = needle.len();
        (0..rom.len().saturating_sub(n))
            .step_by(4)
            .any(|i| &rom[i..i + n] == needle)
    };
    if has(b"EEPROM_V") {
        SaveKind::Eeprom
    } else if has(b"FLASH1M_V") {
        SaveKind::Flash128
    } else if has(b"FLASH512_V") || has(b"FLASH_V") {
        SaveKind::Flash64
    } else if has(b"SRAM_V") {
        SaveKind::Sram
    } else {
        SaveKind::None
    }
}
