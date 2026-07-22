//! The LCD video controller (PPU).
//!
//! This first cut owns the display memory (VRAM / palette / OAM) and the LCD I/O
//! registers, tracks the scanline/blank timing, and renders the bitmap modes
//! (3/4/5). The tile modes (0-2), affine backgrounds, sprites, windows and
//! blending render as the backdrop for now and are filled in next.
//!
//! Register/timing facts are from GBATEK (the clean-room GBA hardware reference
//! in docs/); nothing here derives from an emulator.

pub const SCREEN_W: usize = 240;
pub const SCREEN_H: usize = 160;

/// Dots (pixels) per scanline including H-blank, and total scanlines including
/// V-blank. Each dot is 4 CPU cycles.
pub const DOTS_PER_LINE: u32 = 308;
pub const TOTAL_LINES: u32 = 228;
pub const CYCLES_PER_DOT: u32 = 4;
pub const CYCLES_PER_LINE: u32 = DOTS_PER_LINE * CYCLES_PER_DOT; // 1232

pub struct Ppu {
    /// LCD I/O registers 0x04000000..0x04000060 as halfwords.
    regs: [u16; 0x40],
    pub vram: Box<[u8]>,   // 96 KB
    pub palram: Box<[u8]>, // 1 KB
    pub oam: Box<[u8]>,    // 1 KB
    /// Rendered frame as RGB555 halfwords (240x160).
    pub framebuffer: Box<[u16]>,
}

// Register indices (byte offset >> 1).
const DISPCNT: usize = 0x00 >> 1;
const DISPSTAT: usize = 0x04 >> 1;
const VCOUNT: usize = 0x06 >> 1;

impl Default for Ppu {
    fn default() -> Self {
        Ppu {
            regs: [0; 0x40],
            vram: vec![0; 96 * 1024].into_boxed_slice(),
            palram: vec![0; 1024].into_boxed_slice(),
            oam: vec![0; 1024].into_boxed_slice(),
            framebuffer: vec![0; SCREEN_W * SCREEN_H].into_boxed_slice(),
        }
    }
}

impl Ppu {
    pub fn new() -> Self {
        Self::default()
    }

    // --- LCD I/O registers (0x04000000..0x04000060) ---------------------------

    pub fn read_reg16(&self, offset: u32) -> u16 {
        self.regs[(offset as usize >> 1) & 0x3F]
    }

    /// Current DISPSTAT (for the interrupt controller's LCD IRQ decisions).
    pub fn dispstat(&self) -> u16 {
        self.regs[DISPSTAT]
    }

    pub fn write_reg16(&mut self, offset: u32, val: u16) {
        let i = (offset as usize >> 1) & 0x3F;
        match i {
            VCOUNT => {} // read-only
            DISPSTAT => {
                // Bits 0..2 are read-only status; keep them, take the rest.
                let ro = self.regs[DISPSTAT] & 0x0007;
                self.regs[DISPSTAT] = (val & !0x0007) | ro;
            }
            _ => self.regs[i] = val,
        }
    }

    // --- Display memory, with GBA mirroring -----------------------------------

    pub fn vram_index(addr: u32) -> usize {
        // 128 KB window; the top 32 KB mirror the 0x10000..0x18000 range.
        let o = (addr & 0x1_FFFF) as usize;
        if o >= 0x18000 {
            o - 0x8000
        } else {
            o
        }
    }

    pub fn read_pal8(&self, addr: u32) -> u8 {
        self.palram[(addr & 0x3FF) as usize]
    }
    pub fn read_vram8(&self, addr: u32) -> u8 {
        self.vram[Self::vram_index(addr)]
    }
    pub fn read_oam8(&self, addr: u32) -> u8 {
        self.oam[(addr & 0x3FF) as usize]
    }

    pub fn write_pal8(&mut self, addr: u32, v: u8) {
        // Palette is a 16-bit bus: a byte write mirrors into both halves.
        let a = (addr & 0x3FE) as usize;
        self.palram[a] = v;
        self.palram[a + 1] = v;
    }
    pub fn write_vram8(&mut self, addr: u32, v: u8) {
        // VRAM byte writes below the OBJ area mirror into both halves too.
        let i = Self::vram_index(addr) & !1;
        self.vram[i] = v;
        self.vram[i + 1] = v;
    }
    pub fn write_oam8(&mut self, _addr: u32, _v: u8) {
        // OAM ignores byte writes.
    }

    // --- Timing ---------------------------------------------------------------

    /// Advance to `line`, updating VCOUNT and the DISPSTAT V-blank/V-counter
    /// flags. Returns nothing; IRQ wiring comes with the interrupt controller.
    pub fn begin_line(&mut self, line: u32) {
        self.regs[VCOUNT] = line as u16;
        let mut stat = self.regs[DISPSTAT] & !0x0007;
        if line >= 160 && line != 227 {
            stat |= 0x0001; // V-blank
        }
        let vmatch = (self.regs[DISPSTAT] >> 8) & 0xFF;
        if line as u16 == vmatch {
            stat |= 0x0004; // V-counter match
        }
        self.regs[DISPSTAT] = stat;
    }

    // --- Rendering ------------------------------------------------------------

    /// Render one visible scanline (0..160) into the framebuffer.
    pub fn render_line(&mut self, line: usize) {
        let dispcnt = self.regs[DISPCNT];
        let mode = dispcnt & 0x7;

        if dispcnt & 0x80 != 0 {
            self.framebuffer[line * SCREEN_W..(line + 1) * SCREEN_W].fill(0x7FFF); // forced blank
            return;
        }

        match mode {
            3 | 4 | 5 => self.render_bitmap(line, mode, dispcnt),
            _ => self.render_tiled(line, mode, dispcnt),
        }
    }

    fn render_bitmap(&mut self, line: usize, mode: u16, dispcnt: u16) {
        let backdrop = self.backdrop();
        let row = &mut self.framebuffer[line * SCREEN_W..(line + 1) * SCREEN_W];
        match mode {
            3 => {
                let base = line * SCREEN_W * 2;
                for (x, px) in row.iter_mut().enumerate() {
                    let o = base + x * 2;
                    *px = u16::from_le_bytes([self.vram[o], self.vram[o + 1]]) & 0x7FFF;
                }
            }
            4 => {
                let frame = if dispcnt & 0x10 != 0 { 0xA000 } else { 0 };
                let base = frame + line * SCREEN_W;
                for (x, px) in row.iter_mut().enumerate() {
                    let idx = self.vram[base + x] as usize;
                    *px = u16::from_le_bytes([self.palram[idx * 2], self.palram[idx * 2 + 1]])
                        & 0x7FFF;
                }
            }
            _ => {
                let frame = if dispcnt & 0x10 != 0 { 0xA000 } else { 0 };
                for (x, px) in row.iter_mut().enumerate() {
                    if line < 128 && x < 160 {
                        let o = frame + (line * 160 + x) * 2;
                        *px = u16::from_le_bytes([self.vram[o], self.vram[o + 1]]) & 0x7FFF;
                    } else {
                        *px = backdrop;
                    }
                }
            }
        }
    }

    /// Tile modes 0-2: composite the enabled text backgrounds and sprites by
    /// priority. (Affine backgrounds — mode 1 BG2, mode 2 — are still TODO and
    /// render as transparent.)
    fn render_tiled(&mut self, line: usize, mode: u16, dispcnt: u16) {
        let backdrop = self.backdrop();
        let mut out = [backdrop; SCREEN_W];
        // Per-pixel winning (priority, rank); rank orders same-priority layers
        // with OBJ = 0 on top, then BG0..BG3.
        let mut win_prio = [4u8; SCREEN_W];
        let mut win_rank = [5u8; SCREEN_W];

        for bg in 0..4 {
            let is_text = match mode {
                0 => true,
                1 => bg < 2, // BG0/BG1 text; BG2 affine (TODO)
                _ => false,  // mode 2: BG2/BG3 affine (TODO)
            };
            if !is_text || dispcnt & (1 << (8 + bg)) == 0 {
                continue;
            }
            self.text_bg_line(bg, line, &mut out, &mut win_prio, &mut win_rank);
        }

        if dispcnt & 0x1000 != 0 {
            self.sprite_line(line, dispcnt, &mut out, &mut win_prio, &mut win_rank);
        }

        self.framebuffer[line * SCREEN_W..(line + 1) * SCREEN_W].copy_from_slice(&out);
    }

    /// Composite one text background's scanline into the pixel buffers.
    fn text_bg_line(
        &self,
        bg: usize,
        line: usize,
        out: &mut [u16; SCREEN_W],
        win_prio: &mut [u8; SCREEN_W],
        win_rank: &mut [u8; SCREEN_W],
    ) {
        let cnt = self.regs[0x08 / 2 + bg]; // BGxCNT
        let priority = (cnt & 3) as u8;
        let rank = bg as u8 + 1;
        let char_base = ((cnt >> 2) & 3) as usize * 0x4000;
        let is_8bpp = cnt & 0x80 != 0;
        let screen_base = ((cnt >> 8) & 0x1F) as usize * 0x800;
        let (w, h) = match (cnt >> 14) & 3 {
            0 => (256usize, 256usize),
            1 => (512, 256),
            2 => (256, 512),
            _ => (512, 512),
        };
        let hofs = (self.regs[0x10 / 2 + bg * 2] & 0x1FF) as usize; // BGxHOFS
        let vofs = (self.regs[0x12 / 2 + bg * 2] & 0x1FF) as usize; // BGxVOFS

        let bgy = (line + vofs) & (h - 1);
        let sb_y = bgy / 256;
        let ty = (bgy % 256) / 8;
        let py = bgy % 8;

        for x in 0..SCREEN_W {
            // A pixel already fully on top of this BG can't be beaten by it.
            if (priority, rank) >= (win_prio[x], win_rank[x]) {
                continue;
            }
            let bgx = (x + hofs) & (w - 1);
            let sb_x = bgx / 256;
            // Screenblock index within the (up to 2x2) map arrangement.
            let sb = match (w, h) {
                (512, 256) => sb_x,
                (256, 512) => sb_y,
                (512, 512) => sb_y * 2 + sb_x,
                _ => 0,
            };
            let tx = (bgx % 256) / 8;
            let map = screen_base + sb * 0x800 + (ty * 32 + tx) * 2;
            let entry = u16::from_le_bytes([self.vram[map], self.vram[map + 1]]);
            let tile = (entry & 0x3FF) as usize;
            let px = if entry & 0x400 != 0 { 7 - (bgx % 8) } else { bgx % 8 };
            let py = if entry & 0x800 != 0 { 7 - py } else { py };

            let idx = if is_8bpp {
                self.vram[char_base + tile * 64 + py * 8 + px] as usize
            } else {
                let b = self.vram[char_base + tile * 32 + py * 4 + px / 2];
                (if px & 1 == 0 { b & 0xF } else { b >> 4 }) as usize
            };
            if idx == 0 {
                continue; // transparent
            }
            let pal = if is_8bpp {
                idx
            } else {
                ((entry >> 12) & 0xF) as usize * 16 + idx
            };
            out[x] = u16::from_le_bytes([self.palram[pal * 2], self.palram[pal * 2 + 1]]) & 0x7FFF;
            win_prio[x] = priority;
            win_rank[x] = rank;
        }
    }

    /// Composite the regular (non-affine) sprites for one scanline.
    fn sprite_line(
        &self,
        line: usize,
        dispcnt: u16,
        out: &mut [u16; SCREEN_W],
        win_prio: &mut [u8; SCREEN_W],
        win_rank: &mut [u8; SCREEN_W],
    ) {
        const SIZE: [[(i32, i32); 4]; 3] = [
            [(8, 8), (16, 16), (32, 32), (64, 64)],  // square
            [(16, 8), (32, 8), (32, 16), (64, 32)],  // horizontal
            [(8, 16), (8, 32), (16, 32), (32, 64)],  // vertical
        ];
        let one_dim = dispcnt & 0x40 != 0;
        // Topmost sprite pixel so far (by priority then OBJ index).
        let mut obj_prio = [255u8; SCREEN_W];

        for i in 0..128 {
            let a0 = u16::from_le_bytes([self.oam[i * 8], self.oam[i * 8 + 1]]);
            let a1 = u16::from_le_bytes([self.oam[i * 8 + 2], self.oam[i * 8 + 3]]);
            let a2 = u16::from_le_bytes([self.oam[i * 8 + 4], self.oam[i * 8 + 5]]);

            let affine = a0 & 0x100 != 0;
            if affine {
                continue; // affine sprites: TODO
            }
            if a0 & 0x200 != 0 {
                continue; // disabled
            }
            let shape = ((a0 >> 14) & 3) as usize;
            let size = ((a1 >> 14) & 3) as usize;
            if shape == 3 {
                continue;
            }
            let (w, h) = SIZE[shape][size];

            let y = (a0 & 0xFF) as i32;
            let row = (line as i32 - y) & 0xFF; // 8-bit wrap
            if row >= h {
                continue;
            }
            let mut x = (a1 & 0x1FF) as i32;
            if x >= 256 {
                x -= 512; // 9-bit signed
            }

            let is_8bpp = a0 & 0x2000 != 0;
            let hflip = a1 & 0x1000 != 0;
            let vflip = a1 & 0x2000 != 0;
            let base_tile = (a2 & 0x3FF) as usize;
            let priority = ((a2 >> 10) & 3) as u8;
            let pal_bank = ((a2 >> 12) & 0xF) as usize;
            let unit_stride = if is_8bpp { 2 } else { 1 };
            let row_stride = if one_dim { (w / 8) as usize * unit_stride } else { 32 };

            let sy = if vflip { h - 1 - row } else { row };
            for col in 0..w {
                let sx = x + col;
                if sx < 0 || sx >= SCREEN_W as i32 {
                    continue;
                }
                let sx = sx as usize;
                let tex_x = if hflip { w - 1 - col } else { col };
                let unit = base_tile
                    + (sy / 8) as usize * row_stride
                    + (tex_x / 8) as usize * unit_stride;
                let base = 0x1_0000 + unit * 32;
                let (px, py) = ((tex_x % 8) as usize, (sy % 8) as usize);
                let idx = if is_8bpp {
                    self.vram[base + py * 8 + px] as usize
                } else {
                    let b = self.vram[base + py * 4 + px / 2];
                    (if px & 1 == 0 { b & 0xF } else { b >> 4 }) as usize
                };
                if idx == 0 {
                    continue; // transparent
                }
                if priority >= obj_prio[sx] {
                    continue; // an earlier, higher sprite already owns this pixel
                }
                obj_prio[sx] = priority;
                // OBJ palette is the second half of palette RAM (index 0x100+).
                let pal = 0x100 + if is_8bpp { idx } else { pal_bank * 16 + idx };
                let color =
                    u16::from_le_bytes([self.palram[pal * 2], self.palram[pal * 2 + 1]]) & 0x7FFF;
                // OBJ beats a BG of equal priority (rank 0).
                if (priority, 0u8) < (win_prio[sx], win_rank[sx]) {
                    out[sx] = color;
                    win_prio[sx] = priority;
                    win_rank[sx] = 0;
                }
            }
        }
    }

    /// Backdrop colour = palette entry 0.
    fn backdrop(&self) -> u16 {
        u16::from_le_bytes([self.palram[0], self.palram[1]]) & 0x7FFF
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn put16(mem: &mut [u8], off: usize, v: u16) {
        mem[off] = v as u8;
        mem[off + 1] = (v >> 8) as u8;
    }

    #[test]
    fn text_bg0_renders_a_tile() {
        let mut p = Ppu::new();
        put16(&mut p.palram, 2, 0x001F); // palette[1] = red
        for i in 0..32 {
            p.vram[i] = 0x11; // tile 0 (char base 0): every 4bpp pixel = index 1
        }
        put16(&mut p.vram, 0x800, 0); // screenblock 1: map entry 0 -> tile 0
        p.write_reg16(0x08, 0x0100); // BG0CNT: char base 0, screen base block 1
        p.write_reg16(0x00, 0x0100); // DISPCNT: mode 0, BG0 on
        p.render_line(0);
        assert_eq!(p.framebuffer[0], 0x001F);
        assert_eq!(p.framebuffer[7], 0x001F);
    }

    #[test]
    fn sprite_draws_over_backdrop() {
        let mut p = Ppu::new();
        put16(&mut p.palram, 0, 0x7C00); // backdrop = blue
        put16(&mut p.palram, 0x202, 0x03E0); // OBJ palette[1] = green
        for i in 0..32 {
            p.vram[0x1_0000 + i] = 0x11; // OBJ tile 0, index 1
        }
        // OAM sprite 0: y=0, x=0, 8x8, tile 0 (all attribute words zero).
        p.write_reg16(0x00, 0x1040); // DISPCNT: mode 0, OBJ on, 1D mapping
        p.render_line(0);
        assert_eq!(p.framebuffer[0], 0x03E0, "sprite pixel green");
        assert_eq!(p.framebuffer[8], 0x7C00, "outside sprite = backdrop");
    }

    #[test]
    fn sprite_beats_bg_at_equal_priority() {
        let mut p = Ppu::new();
        put16(&mut p.palram, 2, 0x001F); // BG palette[1] = red
        put16(&mut p.palram, 0x202, 0x03E0); // OBJ palette[1] = green
        for i in 0..32 {
            p.vram[i] = 0x11;
            p.vram[0x1_0000 + i] = 0x11;
        }
        put16(&mut p.vram, 0x800, 0);
        p.write_reg16(0x08, 0x0100); // BG0CNT priority 0
        p.write_reg16(0x00, 0x1140); // DISPCNT: mode 0, BG0 + OBJ, 1D
        p.render_line(0);
        assert_eq!(p.framebuffer[0], 0x03E0, "OBJ wins over BG at equal priority");
    }

    #[test]
    fn affine_regs_do_not_alias_control_regs() {
        // Regression: a bad index mask (& 0x2F) folded the affine parameters at
        // 0x20..0x3F onto the LCD control registers at 0x00..0x1F. Writing BG2PC
        // (0x24) then wiped DISPSTAT (0x04), stalling any game that enabled a
        // VBlank IRQ after setting up an affine background (e.g. Mario & Luigi).
        let mut p = Ppu::new();
        p.write_reg16(0x04, 0x0008); // DISPSTAT: VBlank IRQ enable
        p.write_reg16(0x24, 0x0000); // BG2PC = 0
        p.write_reg16(0x20, 0x0100); // BG2PA
        assert_eq!(p.dispstat() & 0x0008, 0x0008, "DISPSTAT survives affine writes");
        assert_eq!(p.read_reg16(0x00), 0x0000, "DISPCNT not touched by BG2PA");
        assert_eq!(p.read_reg16(0x24), 0x0000, "BG2PC read back independently");
        assert_eq!(p.read_reg16(0x20), 0x0100, "BG2PA read back independently");
    }
}
