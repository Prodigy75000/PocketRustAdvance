//! Runs the GBA core for a few frames and writes the framebuffer to a PNG.
//!
//! With no argument it runs a built-in, hand-assembled mode-3 test ROM (set
//! DISPCNT to mode 3 + BG2, then fill VRAM with a gradient). With a path it
//! loads that .gba ROM instead.

use std::fs::File;
use std::io::BufWriter;

use gba_core::bus::{Access, Bus};
use gba_core::{Gba, SCREEN_H, SCREEN_W};

/// A tiny ARM program: DISPCNT = mode 3 + BG2, then fill 240*160 VRAM halfwords
/// with an incrementing value (a gradient), then spin.
fn test_rom() -> Vec<u8> {
    let words: [u32; 12] = [
        0xE3A00404, // MOV  R0, #0x04000000      ; DISPCNT
        0xE3A01003, // MOV  R1, #3               ; mode 3
        0xE3811B01, // ORR  R1, R1, #0x400       ; + BG2 enable
        0xE1C010B0, // STRH R1, [R0]
        0xE3A00406, // MOV  R0, #0x06000000      ; VRAM
        0xE3A02000, // MOV  R2, #0               ; colour = 0
        0xE3A03C96, // MOV  R3, #0x9600          ; 38400 pixels
        0xE0C020B2, // STRH R2, [R0], #2         ; loop:
        0xE2822001, // ADD  R2, R2, #1
        0xE2533001, // SUBS R3, R3, #1
        0x1AFFFFFB, // BNE  loop
        0xEAFFFFFE, // B    .                    ; spin
    ];
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// A mode-0 tile ROM: palette[1]=red, tile 0 = solid index 1, then enable BG0.
/// The tilemap defaults to all-zero (tile 0), so the whole screen becomes red.
fn tile_rom() -> Vec<u8> {
    let words: [u32; 17] = [
        0xE3A00405, // MOV  R0, #0x05000000     ; palette base
        0xE3A0101F, // MOV  R1, #0x1F           ; red
        0xE1C010B2, // STRH R1, [R0, #2]        ; palette[1] = red
        0xE3A00406, // MOV  R0, #0x06000000     ; VRAM (char base 0 / tile 0)
        0xE3A01011, // MOV  R1, #0x11
        0xE1811401, // ORR  R1, R1, R1, LSL #8  ; R1 = 0x1111
        0xE1811801, // ORR  R1, R1, R1, LSL #16 ; R1 = 0x11111111
        0xE3A02008, // MOV  R2, #8
        0xE4801004, // STR  R1, [R0], #4        ; loop: fill tile 0 (32 bytes)
        0xE2522001, // SUBS R2, R2, #1
        0x1AFFFFFC, // BNE  loop
        0xE3A00404, // MOV  R0, #0x04000000
        0xE3A01C01, // MOV  R1, #0x100          ; screen base block 1
        0xE1C010B8, // STRH R1, [R0, #8]        ; BG0CNT
        0xE3A01C01, // MOV  R1, #0x100          ; mode 0 + BG0 on
        0xE1C010B0, // STRH R1, [R0]            ; DISPCNT
        0xEAFFFFFE, // B    .
    ];
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// An IRQ ROM: install a VBlank handler (via the 0x03007FFC pointer) that bumps
/// a counter at IWRAM 0, enable VBlank IRQ (DISPSTAT/IE/IME), then spin. After N
/// frames the counter should be ~N.
fn irq_rom() -> Vec<u8> {
    let words: [u32; 22] = [
        0xE3A00403, // MOV  R0, #0x03000000
        0xE2800C7F, // ADD  R0, R0, #0x7F00
        0xE28000FC, // ADD  R0, R0, #0xFC        ; R0 = 0x03007FFC (handler ptr)
        0xE28F1020, // ADD  R1, PC, #0x20        ; R1 = &handler (0x08000034)
        0xE5801000, // STR  R1, [R0]             ; [0x03007FFC] = handler
        0xE3A00404, // MOV  R0, #0x04000000
        0xE3A01008, // MOV  R1, #0x08
        0xE1C010B4, // STRH R1, [R0, #4]         ; DISPSTAT: V-blank IRQ enable
        0xE2800C02, // ADD  R0, R0, #0x200       ; R0 = 0x04000200
        0xE3A01001, // MOV  R1, #1
        0xE1C010B0, // STRH R1, [R0]             ; IE = V-blank
        0xE1C010B8, // STRH R1, [R0, #8]         ; IME = 1
        0xEAFFFFFE, // spin: B .
        // handler (0x34):
        0xE3A02403, // MOV  R2, #0x03000000      ; counter address
        0xE5923000, // LDR  R3, [R2]
        0xE2833001, // ADD  R3, R3, #1
        0xE5823000, // STR  R3, [R2]
        0xE3A00404, // MOV  R0, #0x04000000
        0xE2800C02, // ADD  R0, R0, #0x200
        0xE3A01001, // MOV  R1, #1
        0xE1C010B2, // STRH R1, [R0, #2]         ; IF: acknowledge V-blank
        0xE12FFF1E, // BX   LR
    ];
    words.iter().flat_map(|w| w.to_le_bytes()).collect()
}

/// Decode the cartridge header (GBATEK): 12-byte ASCII title at 0xA0, 4-byte
/// game code at 0xAC, and verify the header checksum at 0xBD.
fn dump_header(rom: &[u8]) {
    if rom.len() < 0xC0 {
        return;
    }
    let title: String = rom[0xA0..0xAC]
        .iter()
        .take_while(|&&b| b != 0)
        .map(|&b| b as char)
        .collect();
    let code: String = rom[0xAC..0xB0].iter().map(|&b| b as char).collect();
    let mut chk = 0u8;
    for &b in &rom[0xA0..0xBD] {
        chk = chk.wrapping_sub(b);
    }
    chk = chk.wrapping_sub(0x19);
    let ok = chk == rom[0xBD];
    println!("  header: title=\"{title}\" code={code} checksum={} (stored {:02X}, calc {:02X})",
        if ok { "OK" } else { "BAD" }, rom[0xBD], chk);
}

fn main() {
    let (rom, label) = match std::env::args().nth(1) {
        Some(a) if a == "tile" => (tile_rom(), "built-in mode-0 tile test".to_string()),
        Some(a) if a == "irq" => (irq_rom(), "built-in VBlank IRQ test".to_string()),
        Some(path) if !path.is_empty() && path != "-" => {
            (std::fs::read(&path).expect("read rom"), path)
        }
        _ => (test_rom(), "built-in mode-3 test".to_string()),
    };
    let frames: u32 = std::env::args()
        .nth(2)
        .and_then(|s| s.parse().ok())
        .unwrap_or(4);

    // A real BIOS (16 KB) can be supplied via GBA_BIOS; otherwise direct-boot.
    let bios = std::env::var("GBA_BIOS")
        .ok()
        .and_then(|p| std::fs::read(p).ok())
        .unwrap_or_default();

    let is_cart = rom.len() >= 0xC0 && label.ends_with(".gba");
    if is_cart {
        dump_header(&rom);
        println!("  boot: {}", if bios.is_empty() { "direct (no BIOS)" } else { "via BIOS" });
    }

    let distinct_count = |fb: &[u16]| {
        let mut v: Vec<u16> = fb.to_vec();
        v.sort_unstable();
        v.dedup();
        v.len()
    };

    let mut gba = Gba::new(rom, bios);
    // Load a save-state (GBA_LOADSTATE=<file>) to inspect an exact scene. One
    // frame is rendered afterwards so the framebuffer/PNG reflect the state.
    if let Some(path) = std::env::var_os("GBA_LOADSTATE") {
        match std::fs::read(&path) {
            Ok(bytes) if gba.load_state(&bytes) => {
                eprintln!("loaded state {:?} ({} bytes)", path, bytes.len());
            }
            Ok(_) => eprintln!("state {:?} rejected (bad magic/version/size)", path),
            Err(e) => eprintln!("could not read state {:?}: {e}", path),
        }
    }
    if std::env::var_os("GBA_NORENDER").is_some() {
        gba.render_enabled = false;
    }
    if std::env::var_os("GBA_NOBLEND").is_some() {
        gba.bus.ppu.no_blend = true;
    }
    if std::env::var_os("GBA_NOWINDOW").is_some() {
        gba.bus.ppu.no_window = true;
    }
    // Track the most "interesting" frame (most distinct colours) so a single PNG
    // lands on a real rendered screen, not a blank/forced-blank transition frame.
    let mut best_fb: Vec<u16> = Vec::new();
    let mut best_distinct = 0usize;
    let mut best_frame = 0u32;
    let trace = std::env::var("GBA_TRACE").is_ok();
    // Auto-advance menus: hold A/Start in short pulses to reach in-game scenes.
    let autoinput = std::env::var_os("GBA_AUTOINPUT").is_some();
    for f in 0..frames {
        if autoinput {
            let p = f % 24 < 4; // pulse A + Start to advance title and dialogue
            gba.set_button(gba_core::Button::A, p);
            gba.set_button(gba_core::Button::Start, p);
        }
        let fb = gba.run_frame().to_vec();
        let d = distinct_count(&fb);
        if d > best_distinct {
            best_distinct = d;
            best_fb = fb;
            best_frame = f;
        }
        if trace && f < 40 {
            println!("  f{f:>3} cyc={:>9} PC={:08X} DISPCNT={:04X} DISPSTAT={:04X} IE={:04X} IME={} halt={} irqs={}",
                gba.bus.cycles, gba.cpu.r[15], gba.bus.ppu.read_reg16(0), gba.bus.ppu.dispstat(),
                gba.bus.ie, gba.bus.ime, gba.bus.halted as u8, gba.irqs_taken);
        }
    }
    if best_distinct > 1 {
        println!("  best frame: #{best_frame} with {best_distinct} distinct colours (saved to PNG)");
    }
    // GBA_SAVESTATE=<file> writes a save-state of the final frame.
    if let Some(path) = std::env::var_os("GBA_SAVESTATE") {
        let blob = gba.save_state();
        match std::fs::write(&path, &blob) {
            Ok(()) => eprintln!("wrote state {:?} ({} bytes)", path, blob.len()),
            Err(e) => eprintln!("could not write state {:?}: {e}", path),
        }
    }

    if is_cart {
        let pc = gba.cpu.r[15];
        let region = match (pc >> 24) & 0xF {
            0x0 => "BIOS", 0x2 => "EWRAM", 0x3 => "IWRAM",
            0x8..=0xD => "ROM", _ => "other",
        };
        println!("  after {frames} frames: PC={pc:08X} ({region})  DISPCNT={:04X}", gba.bus.ppu.read_reg16(0));
        let iw = |a: u32| u32::from_le_bytes(gba.bus.iwram[(a & 0x7FFF) as usize..][..4].try_into().unwrap());
        println!("  irqs taken={}  IE={:04X} IF={:04X} IME={:X} DISPSTAT={:04X} CPSR={:08X}",
            gba.irqs_taken, gba.bus.ie, gba.bus.if_, gba.bus.ime, gba.bus.ppu.dispstat(), gba.cpu.cpsr);
        println!("  BIOS intr flags [0x03FFFFF8]={:08X}  user IRQ handler [0x03FFFFFC]={:08X}",
            iw(0x7FF8), iw(0x7FFC));
        let rr = |o: u32| gba.bus.ppu.read_reg16(o);
        println!("  BLDCNT={:04X} BLDALPHA={:04X} BLDY={:04X}  WININ={:04X} WINOUT={:04X}",
            rr(0x50), rr(0x52), rr(0x54), rr(0x48), rr(0x4A));
        println!("  WIN0H={:04X} WIN0V={:04X} WIN1H={:04X} WIN1V={:04X}  backdrop(pal0)={:04X}",
            rr(0x40), rr(0x44), rr(0x42), rr(0x46),
            u16::from_le_bytes([gba.bus.ppu.palram[0], gba.bus.ppu.palram[1]]));
        // OAM census: enabled sprites, split by affine / mode.
        let oam = &gba.bus.ppu.oam;
        let (mut normal, mut affine, mut objwin) = (0, 0, 0);
        for i in 0..128 {
            let a0 = u16::from_le_bytes([oam[i * 8], oam[i * 8 + 1]]);
            let aff = a0 & 0x100 != 0;
            let disabled = !aff && a0 & 0x200 != 0;
            if disabled {
                continue;
            }
            match (a0 >> 10) & 3 {
                2 => objwin += 1,
                _ if aff => affine += 1,
                _ => normal += 1,
            }
        }
        println!("  OAM: {normal} normal, {affine} affine, {objwin} obj-window sprites");
        // First few BG palette entries (to spot a washed/faded palette).
        let pe = |i: usize| u16::from_le_bytes([gba.bus.ppu.palram[i * 2], gba.bus.ppu.palram[i * 2 + 1]]);
        println!("  BG pal[0..8]: {:04X} {:04X} {:04X} {:04X} {:04X} {:04X} {:04X} {:04X}",
            pe(0), pe(1), pe(2), pe(3), pe(4), pe(5), pe(6), pe(7));
        // Framebuffer samples (240x160): background, professor centre, text row.
        let fb = &gba.bus.ppu.framebuffer;
        let px = |x: usize, y: usize| fb[y * SCREEN_W + x];
        println!("  fb: bg(120,40)={:04X} centre(120,70)={:04X} text(60,140)={:04X} top(120,10)={:04X}",
            px(120, 40), px(120, 70), px(60, 140), px(120, 10));
        if std::env::var("GBA_REGS").is_ok() {
            for row in 0..4 {
                let r = row * 4;
                println!("    R{:>2}-R{:>2}: {:08X} {:08X} {:08X} {:08X}",
                    r, r + 3, gba.cpu.r[r], gba.cpu.r[r + 1], gba.cpu.r[r + 2], gba.cpu.r[r + 3]);
            }
            // Instruction words around the (Thumb or ARM) execute point.
            let thumb = gba.cpu.cpsr & (1 << 5) != 0;
            let base = if thumb { pc.wrapping_sub(4) } else { pc.wrapping_sub(8) };
            print!("    code @ {base:08X}:");
            for i in 0..6 {
                let a = base + i * 2;
                let h = gba.bus.read16(a, Access::NonSeq);
                print!(" {h:04X}");
            }
            println!();
        }
    }
    println!("steps={}  (~{} instr/frame)", gba.steps, gba.steps / frames.max(1) as u64);
    let counter = u32::from_le_bytes(gba.bus.iwram[0..4].try_into().unwrap());
    println!("IWRAM counter @0x03000000 = {counter}  (IRQs taken)");
    // Prefer the best (most colourful) frame for the PNG; fall back to the final.
    // GBA_LASTFRAME forces the final frame (to inspect a specific moment).
    let fb: Vec<u16> = if best_distinct > 1 && std::env::var_os("GBA_LASTFRAME").is_none() {
        best_fb.clone()
    } else {
        gba.bus.ppu.framebuffer.to_vec()
    };

    // Stats so the user has measurable facts (no self-judging of the image).
    let nonzero = fb.iter().filter(|&&p| p != 0).count();
    let distinct = {
        let mut v: Vec<u16> = fb.to_vec();
        v.sort_unstable();
        v.dedup();
        v.len()
    };
    println!("ran '{label}' for {frames} frames");
    println!(
        "framebuffer {}x{}: {nonzero}/{} non-zero px, {distinct} distinct colours",
        SCREEN_W,
        SCREEN_H,
        fb.len()
    );
    println!("  corners: TL={:04X} TR={:04X} BL={:04X} BR={:04X}", fb[0], fb[SCREEN_W - 1], fb[(SCREEN_H - 1) * SCREEN_W], fb[fb.len() - 1]);
    if std::env::var("GBA_DEBUG").is_ok() {
        let p = &gba.bus.ppu;
        println!("  DISPCNT={:04X} BG0CNT={:04X}", p.read_reg16(0), p.read_reg16(8));
        println!("  tile0 bytes: {:02X?}", &p.vram[0..16]);
        println!("  map row0 (0x800): {:02X?}", &p.vram[0x800..0x810]);
    }

    // RGB555 -> RGB8 and write a PNG.
    let mut rgb = Vec::with_capacity(fb.len() * 3);
    for &p in fb.iter() {
        let r = ((p & 0x1F) << 3) as u8;
        let g = (((p >> 5) & 0x1F) << 3) as u8;
        let b = (((p >> 10) & 0x1F) << 3) as u8;
        rgb.extend_from_slice(&[r, g, b]);
    }
    let out = "gba_frame.png";
    let file = File::create(out).expect("create png");
    let mut enc = png::Encoder::new(BufWriter::new(file), SCREEN_W as u32, SCREEN_H as u32);
    enc.set_color(png::ColorType::Rgb);
    enc.set_depth(png::BitDepth::Eight);
    enc.write_header()
        .unwrap()
        .write_image_data(&rgb)
        .unwrap();
    println!("wrote {out}");
}
