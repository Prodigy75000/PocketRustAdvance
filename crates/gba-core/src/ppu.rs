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
    /// Internal affine reference points [x, y] (28-bit signed, 8.8 fixed) for
    /// BG2 and BG3. Latched from BGxX/BGxY at the top of the frame and advanced
    /// by PB/PD each scanline (a mid-frame write to BGxX/Y reloads them).
    bg_ref: [[i32; 2]; 2], // [bg2, bg3][x, y]
    /// Profiling switch: force colour effects off (measures blend cost).
    pub no_blend: bool,
}

// Register indices (byte offset >> 1).
const DISPCNT: usize = 0x00 >> 1;
const DISPSTAT: usize = 0x04 >> 1;
const VCOUNT: usize = 0x06 >> 1;
// Affine register indices: BG2 params at 0x20-0x2E, BG3 at 0x30-0x3E.
const BG2_PA: usize = 0x20 >> 1; // PA,PB,PC,PD at BG2_PA..+3
const BG2_X: usize = 0x28 >> 1; // BG2X (lo,hi) at BG2_X, BG2_X+1; BG2Y at +2,+3
const BG3_PA: usize = 0x30 >> 1;
const BG3_X: usize = 0x38 >> 1;

/// Sign-extend a 28-bit affine reference coordinate to i32.
fn sign_extend_28(v: u32) -> i32 {
    let v = v & 0x0FFF_FFFF;
    if v & 0x0800_0000 != 0 {
        (v | 0xF000_0000) as i32
    } else {
        v as i32
    }
}

/// Per-scanline layer compositor: keeps the top two opaque layers at each pixel
/// (colour + layer id + a packed (priority, rank) sort key) so the colour
/// special effects (BLDCNT) can blend the top with the one directly below it.
/// Layer ids match the BLDCNT target bits: BG0..BG3 = 0..3, OBJ = 4, backdrop
/// = 5. Smaller key wins; OBJ uses rank 0 to beat a same-priority BG.
struct Compositor {
    top_col: [u16; SCREEN_W],
    top_id: [u8; SCREEN_W],
    top_key: [u16; SCREEN_W],
    snd_col: [u16; SCREEN_W],
    snd_id: [u8; SCREEN_W],
    snd_key: [u16; SCREEN_W],
    /// Track the second layer only when a colour effect needs it (blending on).
    track_second: bool,
}

impl Compositor {
    fn new(backdrop: u16, track_second: bool) -> Self {
        let bd_key = (4u16 << 8) | 5; // below every BG/OBJ
        Compositor {
            top_col: [backdrop; SCREEN_W],
            top_id: [5; SCREEN_W],
            top_key: [bd_key; SCREEN_W],
            snd_col: [backdrop; SCREEN_W],
            snd_id: [5; SCREEN_W],
            snd_key: [bd_key; SCREEN_W],
            track_second,
        }
    }

    /// The key below which a new layer pixel is worth placing. Without blending
    /// only the winner matters (`top_key`); with it, a layer can still be second.
    #[inline]
    fn threshold(&self, x: usize) -> u16 {
        if self.track_second {
            self.snd_key[x]
        } else {
            self.top_key[x]
        }
    }

    #[inline]
    fn place(&mut self, x: usize, col: u16, id: u8, priority: u8, rank: u8) {
        let key = ((priority as u16) << 8) | rank as u16;
        if key < self.top_key[x] {
            if self.track_second {
                self.snd_col[x] = self.top_col[x];
                self.snd_id[x] = self.top_id[x];
                self.snd_key[x] = self.top_key[x];
            }
            self.top_col[x] = col;
            self.top_id[x] = id;
            self.top_key[x] = key;
        } else if self.track_second && key < self.snd_key[x] {
            self.snd_col[x] = col;
            self.snd_id[x] = id;
            self.snd_key[x] = key;
        }
    }
}

#[inline]
fn channels(c: u16) -> (i32, i32, i32) {
    ((c & 0x1F) as i32, ((c >> 5) & 0x1F) as i32, ((c >> 10) & 0x1F) as i32)
}
#[inline]
fn pack(r: i32, g: i32, b: i32) -> u16 {
    (r as u16) | ((g as u16) << 5) | ((b as u16) << 10)
}
/// BLDCNT mode 1: `a*EVA/16 + b*EVB/16` per channel, clamped to 31.
fn blend_alpha(a: u16, b: u16, eva: u16, evb: u16) -> u16 {
    let (ar, ag, ab) = channels(a);
    let (br, bg, bb) = channels(b);
    let (ea, eb) = (eva as i32, evb as i32);
    let f = |x: i32, y: i32| ((x * ea + y * eb) / 16).min(31);
    pack(f(ar, br), f(ag, bg), f(ab, bb))
}
/// BLDCNT mode 2: brighten toward white by EVY/16.
fn brighten(a: u16, evy: u16) -> u16 {
    let (r, g, b) = channels(a);
    let e = evy as i32;
    let f = |x: i32| x + ((31 - x) * e) / 16;
    pack(f(r), f(g), f(b))
}
/// BLDCNT mode 3: darken toward black by EVY/16.
fn darken(a: u16, evy: u16) -> u16 {
    let (r, g, b) = channels(a);
    let e = evy as i32;
    let f = |x: i32| x - (x * e) / 16;
    pack(f(r), f(g), f(b))
}

impl Default for Ppu {
    fn default() -> Self {
        Ppu {
            regs: [0; 0x40],
            vram: vec![0; 96 * 1024].into_boxed_slice(),
            palram: vec![0; 1024].into_boxed_slice(),
            oam: vec![0; 1024].into_boxed_slice(),
            framebuffer: vec![0; SCREEN_W * SCREEN_H].into_boxed_slice(),
            bg_ref: [[0; 2]; 2],
            no_blend: false,
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
        // Writing any half of a BGxX/BGxY register reloads that affine reference
        // point immediately (GBATEK).
        if (BG2_X..BG2_X + 4).contains(&i) {
            self.reload_bg_ref(0);
        } else if (BG3_X..BG3_X + 4).contains(&i) {
            self.reload_bg_ref(1);
        }
    }

    /// Reload BG2/BG3 (`bg` = 0/1) internal reference point from its BGxX/BGxY
    /// registers.
    fn reload_bg_ref(&mut self, bg: usize) {
        let xbase = if bg == 0 { BG2_X } else { BG3_X };
        let x = self.regs[xbase] as u32 | ((self.regs[xbase + 1] as u32) << 16);
        let y = self.regs[xbase + 2] as u32 | ((self.regs[xbase + 3] as u32) << 16);
        self.bg_ref[bg] = [sign_extend_28(x), sign_extend_28(y)];
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

        // Maintain the affine reference points: reload from BGxX/BGxY at the top
        // of the frame, then advance by PB/PD (the per-scanline deltas) each line.
        if line == 0 {
            self.reload_bg_ref(0);
            self.reload_bg_ref(1);
        } else if line < SCREEN_H as u32 {
            for bg in 0..2 {
                let pbase = if bg == 0 { BG2_PA } else { BG3_PA };
                let pb = self.regs[pbase + 1] as i16 as i32; // dx per scanline
                let pd = self.regs[pbase + 3] as i16 as i32; // dy per scanline
                self.bg_ref[bg][0] = self.bg_ref[bg][0].wrapping_add(pb);
                self.bg_ref[bg][1] = self.bg_ref[bg][1].wrapping_add(pd);
            }
        }

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

    /// Tile modes 0-2: composite the enabled backgrounds and sprites by
    /// priority. Mode 0 is four text BGs; mode 1 is BG0/BG1 text + BG2 affine;
    /// mode 2 is BG2/BG3 affine.
    fn render_tiled(&mut self, line: usize, mode: u16, dispcnt: u16) {
        let bldcnt = self.regs[0x50 / 2];
        let blend_mode = if self.no_blend { 0 } else { (bldcnt >> 6) & 3 };
        let mut comp = Compositor::new(self.backdrop(), blend_mode != 0);

        // Gather the enabled BGs and render them front-to-back (by priority,
        // then BG index) so covered pixels are pruned by the skip check.
        let mut layers: [(u8, usize, bool); 4] = [(0, 0, false); 4];
        let mut n = 0;
        for bg in 0..4 {
            if dispcnt & (1 << (8 + bg)) == 0 {
                continue;
            }
            let text = match mode {
                0 => true,
                1 => bg < 2, // BG0/BG1 text, BG2 affine, BG3 unused
                _ => false,  // mode 2: BG2/BG3 affine, BG0/BG1 unused
            };
            let affine = match mode {
                1 => bg == 2,
                2 => bg == 2 || bg == 3,
                _ => false,
            };
            if text || affine {
                let priority = (self.regs[0x08 / 2 + bg] & 3) as u8;
                layers[n] = (priority, bg, affine);
                n += 1;
            }
        }
        layers[..n].sort_unstable_by_key(|&(p, bg, _)| ((p as u16) << 8) | bg as u16);
        for &(_, bg, affine) in &layers[..n] {
            if affine {
                self.affine_bg_line(bg, &mut comp);
            } else {
                self.text_bg_line(bg, line, &mut comp);
            }
        }

        if dispcnt & 0x1000 != 0 {
            self.sprite_line(line, dispcnt, &mut comp);
        }

        let dst = &mut self.framebuffer[line * SCREEN_W..(line + 1) * SCREEN_W];
        if blend_mode == 0 {
            // No colour effect: the winning layer is the output.
            dst.copy_from_slice(&comp.top_col);
            return;
        }

        // Apply the colour special effects (BLDCNT) to the composited top layer.
        let first = bldcnt & 0x3F; // 1st-target layer mask
        let second = (bldcnt >> 8) & 0x3F; // 2nd-target layer mask
        let bldalpha = self.regs[0x52 / 2];
        let eva = (bldalpha & 0x1F).min(16);
        let evb = ((bldalpha >> 8) & 0x1F).min(16);
        let evy = (self.regs[0x54 / 2] & 0x1F).min(16);

        for x in 0..SCREEN_W {
            let tc = comp.top_col[x];
            let first_hit = first & (1 << comp.top_id[x]) != 0;
            dst[x] = match blend_mode {
                1 if first_hit && second & (1 << comp.snd_id[x]) != 0 => {
                    blend_alpha(tc, comp.snd_col[x], eva, evb)
                }
                2 if first_hit => brighten(tc, evy),
                3 if first_hit => darken(tc, evy),
                _ => tc,
            };
        }
    }

    /// Composite one affine background's scanline. BG2/BG3 (`bg` = 2/3) are
    /// 8bpp, 256-colour, using the internal reference point (already advanced to
    /// this scanline) plus PA/PC per pixel. Outside the map: transparent, or
    /// wrapped when the overflow bit is set.
    fn affine_bg_line(&self, bg: usize, comp: &mut Compositor) {
        let cnt = self.regs[0x08 / 2 + bg]; // BGxCNT
        let priority = (cnt & 3) as u8;
        let rank = bg as u8 + 1;
        let char_base = ((cnt >> 2) & 3) as usize * 0x4000;
        let screen_base = ((cnt >> 8) & 0x1F) as usize * 0x800;
        let wrap = cnt & 0x2000 != 0;
        let map_tiles = 16usize << ((cnt >> 14) & 3); // 16/32/64/128 tiles
        let map_px = (map_tiles * 8) as i32;

        let idx = bg - 2;
        let pbase = if bg == 2 { BG2_PA } else { BG3_PA };
        let pa = self.regs[pbase] as i16 as i32; // dx per pixel
        let pc = self.regs[pbase + 2] as i16 as i32; // dy per pixel
        let (mut cx, mut cy) = (self.bg_ref[idx][0], self.bg_ref[idx][1]);

        let key = ((priority as u16) << 8) | rank as u16;
        for x in 0..SCREEN_W {
            let visible = key < comp.threshold(x); // could still land as 1st or 2nd
            let mut tx = cx >> 8;
            let mut ty = cy >> 8;
            cx = cx.wrapping_add(pa);
            cy = cy.wrapping_add(pc);
            if !visible {
                continue;
            }
            if wrap {
                // Map dimensions are always powers of two, so wrap with a mask.
                tx &= map_px - 1;
                ty &= map_px - 1;
            } else if tx < 0 || ty < 0 || tx >= map_px || ty >= map_px {
                continue; // outside the map, transparent
            }
            let (tx, ty) = (tx as usize, ty as usize);
            let tile = self.vram[screen_base + (ty / 8) * map_tiles + tx / 8] as usize;
            let pal = self.vram[char_base + tile * 64 + (ty % 8) * 8 + (tx % 8)] as usize;
            if pal == 0 {
                continue; // transparent
            }
            let color =
                u16::from_le_bytes([self.palram[pal * 2], self.palram[pal * 2 + 1]]) & 0x7FFF;
            comp.place(x, color, bg as u8, priority, rank);
        }
    }

    /// Composite one text background's scanline into the compositor.
    fn text_bg_line(&self, bg: usize, line: usize, comp: &mut Compositor) {
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

        let key = ((priority as u16) << 8) | rank as u16;
        for x in 0..SCREEN_W {
            // Skip if this BG can't beat the winner (or, when blending, the second).
            if key >= comp.threshold(x) {
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
            let color =
                u16::from_le_bytes([self.palram[pal * 2], self.palram[pal * 2 + 1]]) & 0x7FFF;
            comp.place(x, color, bg as u8, priority, rank);
        }
    }

    /// Composite the regular (non-affine) sprites for one scanline.
    fn sprite_line(&self, line: usize, dispcnt: u16, comp: &mut Compositor) {
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
            // For non-affine sprites bit 9 = disabled; for affine it = double-size.
            if !affine && a0 & 0x200 != 0 {
                continue;
            }
            let shape = ((a0 >> 14) & 3) as usize;
            let size = ((a1 >> 14) & 3) as usize;
            if shape == 3 {
                continue;
            }
            let (w, h) = SIZE[shape][size]; // texture size
            let double = affine && a0 & 0x200 != 0;
            let (bw, bh) = if double { (w * 2, h * 2) } else { (w, h) }; // on-screen box

            let y = (a0 & 0xFF) as i32;
            let row = (line as i32 - y) & 0xFF; // 8-bit wrap
            if row >= bh {
                continue;
            }
            let mut x = (a1 & 0x1FF) as i32;
            if x >= 256 {
                x -= 512; // 9-bit signed
            }

            let is_8bpp = a0 & 0x2000 != 0;
            let base_tile = (a2 & 0x3FF) as usize;
            let priority = ((a2 >> 10) & 3) as u8;
            let pal_bank = ((a2 >> 12) & 0xF) as usize;
            let unit_stride = if is_8bpp { 2 } else { 1 };
            let row_stride = if one_dim { (w / 8) as usize * unit_stride } else { 32 };

            // Affine sprites carry a 2x2 matrix from one of 32 OAM parameter
            // groups (interleaved in the unused attr3 slots); non-affine use flip.
            let (pa, pb, pc, pd) = if affine {
                let g = ((a1 >> 9) & 0x1F) as usize * 0x20;
                let p = |o: usize| i16::from_le_bytes([self.oam[g + o], self.oam[g + o + 1]]) as i32;
                (p(0x06), p(0x0E), p(0x16), p(0x1E))
            } else {
                (0, 0, 0, 0)
            };
            let hflip = !affine && a1 & 0x1000 != 0;
            let vflip = !affine && a1 & 0x2000 != 0;

            for col in 0..bw {
                let sx = x + col;
                if sx < 0 || sx >= SCREEN_W as i32 {
                    continue;
                }
                let sx = sx as usize;
                // Map the screen pixel to a texture texel.
                let (tex_x, tex_y) = if affine {
                    let ox = col - bw / 2;
                    let oy = row - bh / 2;
                    let tx = ((pa * ox + pb * oy) >> 8) + w / 2;
                    let ty = ((pc * ox + pd * oy) >> 8) + h / 2;
                    if tx < 0 || ty < 0 || tx >= w || ty >= h {
                        continue; // outside the texture
                    }
                    (tx as usize, ty as usize)
                } else {
                    let tx = if hflip { w - 1 - col } else { col };
                    let ty = if vflip { h - 1 - row } else { row };
                    (tx as usize, ty as usize)
                };
                let unit =
                    base_tile + (tex_y / 8) * row_stride + (tex_x / 8) * unit_stride;
                let base = 0x1_0000 + unit * 32;
                let (px, py) = (tex_x % 8, tex_y % 8);
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
                // OBJ (layer id 4) beats a BG of equal priority (rank 0).
                comp.place(sx, color, 4, priority, 0);
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
    fn blend_effects_math() {
        // Alpha: red*8/16 + blue*8/16 -> (15,0,15).
        assert_eq!(blend_alpha(0x001F, 0x7C00, 8, 8), pack(15, 0, 15));
        // Alpha clamps each channel to 31 (white + white at full weight).
        assert_eq!(blend_alpha(0x7FFF, 0x7FFF, 16, 16), pack(31, 31, 31));
        // Darken red by half: 31 - 31*8/16 = 16.
        assert_eq!(darken(0x001F, 8), pack(16, 0, 0));
        // Brighten red by half: R stays 31, G/B rise 0 -> 15.
        assert_eq!(brighten(0x001F, 8), pack(31, 15, 15));
        // EVY = 0 is a no-op.
        assert_eq!(darken(0x1234 & 0x7FFF, 0), 0x1234 & 0x7FFF);
    }

    #[test]
    fn affine_bg2_renders_a_texel() {
        let mut p = Ppu::new();
        put16(&mut p.palram, 2, 0x03E0); // BG palette[1] = green
        for i in 0..64 {
            p.vram[i] = 1; // tile 0 (8bpp, char base 0): every texel index 1
        }
        p.vram[0x800] = 0; // affine map (screen base block 1) entry (0,0) -> tile 0
        p.write_reg16(0x0C, 0x0100); // BG2CNT: char base 0, screen base block 1
        p.write_reg16(0x20, 0x0100); // PA = 1.0 (8.8)
        p.write_reg16(0x26, 0x0100); // PD = 1.0
        p.write_reg16(0x28, 0); // BG2X = 0 (reloads the reference point)
        p.write_reg16(0x2C, 0); // BG2Y = 0
        p.write_reg16(0x00, 0x0402); // DISPCNT: mode 2, BG2 on
        p.begin_line(0); // latch the affine reference for this frame
        p.render_line(0);
        assert_eq!(p.framebuffer[0], 0x03E0, "affine BG2 texel renders");
        assert_eq!(p.framebuffer[7], 0x03E0, "identity transform fills the row");
    }

    #[test]
    fn affine_sprite_identity_matches_regular() {
        let mut p = Ppu::new();
        put16(&mut p.palram, 0x202, 0x03E0); // OBJ palette[1] = green
        for i in 0..64 {
            p.vram[0x1_0000 + i] = 1; // OBJ tile 0 (8bpp), index 1
        }
        // OAM sprite 0: affine (a0 bit8), 8x8, y=0, x=0, group 0, tile 0, 8bpp.
        put16(&mut p.oam, 0, 0x2100); // a0: affine + 256-colour (bit13) + y 0
        put16(&mut p.oam, 2, 0x0000); // a1: group 0, x 0, size 0
        put16(&mut p.oam, 4, 0x0000); // a2: tile 0, priority 0
        // Affine group 0 = identity matrix (PA=PD=1.0, PB=PC=0).
        put16(&mut p.oam, 0x06, 0x0100); // PA
        put16(&mut p.oam, 0x1E, 0x0100); // PD
        p.write_reg16(0x00, 0x1040); // DISPCNT: OBJ on, 1D mapping
        p.render_line(0);
        assert_eq!(p.framebuffer[0], 0x03E0, "affine (identity) sprite renders");
        assert_eq!(p.framebuffer[7], 0x03E0, "identity covers the whole 8px row");
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
