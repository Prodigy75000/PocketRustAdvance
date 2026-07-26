//! Runs the GBA core for a few frames and writes the framebuffer to a PNG.
//!
//! With no argument it runs a built-in, hand-assembled mode-3 test ROM (set
//! DISPCNT to mode 3 + BG2, then fill VRAM with a gradient). With a path it
//! loads that .gba ROM instead.

use std::fs::File;
use std::io::BufWriter;

use gba_core::bus::{Access, Bus};
use gba_core::{Gba, SCREEN_H, SCREEN_W};

/// Write interleaved-stereo `i16` samples as a 16-bit PCM WAV (for offline
/// listening / measurement of the emulated audio).
fn write_wav(path: &std::ffi::OsStr, samples: &[i16], rate: u32) -> std::io::Result<()> {
    use std::io::Write;
    let channels = 2u16;
    let bits = 16u16;
    let block_align = channels * bits / 8;
    let byte_rate = rate * block_align as u32;
    let data_len = (samples.len() * 2) as u32;
    let mut f = BufWriter::new(File::create(path)?);
    f.write_all(b"RIFF")?;
    f.write_all(&(36 + data_len).to_le_bytes())?;
    f.write_all(b"WAVE")?;
    f.write_all(b"fmt ")?;
    f.write_all(&16u32.to_le_bytes())?; // PCM fmt chunk size
    f.write_all(&1u16.to_le_bytes())?; // PCM
    f.write_all(&channels.to_le_bytes())?;
    f.write_all(&rate.to_le_bytes())?;
    f.write_all(&byte_rate.to_le_bytes())?;
    f.write_all(&block_align.to_le_bytes())?;
    f.write_all(&bits.to_le_bytes())?;
    f.write_all(b"data")?;
    f.write_all(&data_len.to_le_bytes())?;
    for &s in samples {
        f.write_all(&s.to_le_bytes())?;
    }
    Ok(())
}

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

/// RetroArch wraps a core save-state in a "RASTATE" container: an 8-byte header
/// then blocks of [4-byte id][4-byte LE size][data]. The core's data is the
/// "MEM " block. Return that if present, else the bytes unchanged.
fn unwrap_rastate(bytes: &[u8]) -> &[u8] {
    if !bytes.starts_with(b"RASTATE") {
        return bytes;
    }
    let mut pos = 8; // "RASTATE" + version byte
    while pos + 8 <= bytes.len() {
        let id = &bytes[pos..pos + 4];
        let size = u32::from_le_bytes(bytes[pos + 4..pos + 8].try_into().unwrap()) as usize;
        let start = pos + 8;
        let end = (start + size).min(bytes.len());
        if id == b"MEM " {
            return &bytes[start..end];
        }
        pos = end;
    }
    bytes
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

    // GBA_ROMFIND=<hex32>: scan the ROM for the first occurrence of this 32-bit
    // little-endian word and dump 32 words from there. Used to compare a copied
    // IWRAM routine against its ROM source (copy-corruption diagnosis).
    if let Ok(h) = std::env::var("GBA_ROMFIND") {
        let needle = u32::from_str_radix(h.trim_start_matches("0x"), 16).unwrap_or(0);
        let nb = needle.to_le_bytes();
        let mut at = None;
        let mut i = 0;
        while i + 4 <= rom.len() {
            if rom[i..i + 4] == nb {
                at = Some(i);
                break;
            }
            i += 4;
        }
        match at {
            Some(off) => {
                println!("ROMFIND {needle:08X} at file-offset {off:06X} (rom 0x{:08X}):", 0x0800_0000 + off);
                for k in 0..32 {
                    let o = off + k * 4;
                    if o + 4 <= rom.len() {
                        let w = u32::from_le_bytes([rom[o], rom[o + 1], rom[o + 2], rom[o + 3]]);
                        println!("  +{:03X}  {w:08X}", k * 4);
                    }
                }
            }
            None => println!("ROMFIND {needle:08X}: not found"),
        }
        return;
    }

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
            Ok(bytes) => {
                let blob = unwrap_rastate(&bytes);
                if gba.load_state(blob) {
                    eprintln!("loaded state {:?} ({} bytes core data)", path, blob.len());
                } else {
                    eprintln!("state {:?} rejected (bad magic/version/size)", path);
                }
            }
            Err(e) => eprintln!("could not read state {:?}: {e}", path),
        }
    }
    if let Ok(a) = std::env::var("GBA_WATCHW") {
        gba.bus.watch_addr = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap_or(0);
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
    // GBA_ITRACE=<n>: single-step the CPU for n instructions and print every
    // non-sequential PC change (a taken branch / exception / return). Used to
    // pinpoint where a boot runs away into the weeds. Prints then exits.
    if let Ok(n) = std::env::var("GBA_ITRACE") {
        let cap: u64 = n.parse().unwrap_or(20000);
        // GBA_BREAK=<hexaddr>: dump all registers each time this instruction runs.
        let brk = std::env::var("GBA_BREAK").ok()
            .and_then(|s| u32::from_str_radix(s.trim_start_matches("0x"), 16).ok());
        let mut prev_exec: u32 = 0;
        for i in 0..cap {
            {
                let back = if gba.cpu.thumb() { 4 } else { 8 };
                let exec = gba.cpu.r[15].wrapping_sub(back);
                if Some(exec) == brk {
                    let r = &gba.cpu.r;
                    println!("  [{i:>6}] BREAK @{exec:08X}  r0={:08X} r1={:08X} r2={:08X} r3={:08X} r4={:08X} r5={:08X} lr={:08X}",
                        r[0], r[1], r[2], r[3], r[4], r[5], r[14]);
                }
            }
            if gba.bus.irq_pending() && gba.cpu.irq_ready() {
                let from = gba.cpu.r[15];
                gba.cpu.take_irq(&mut gba.bus);
                println!("  [{i:>6}] IRQ  from PC~{from:08X} -> {:08X}", gba.cpu.r[15]);
                gba.bus.halted = false;
            }
            let back = if gba.cpu.thumb() { 4 } else { 8 };
            let exec = gba.cpu.r[15].wrapping_sub(back);
            let width = if gba.cpu.thumb() { 2 } else { 4 };
            // A branch: this instruction is not the sequential successor of the
            // last one we executed.
            if prev_exec != 0 && exec != prev_exec.wrapping_add(width) {
                let mode = if gba.cpu.thumb() { "T" } else { "A" };
                println!("  [{i:>6}] {mode} {prev_exec:08X} -> {exec:08X}");
            }
            prev_exec = exec;
            gba.cpu.step(&mut gba.bus);
            // Also advance timers/ppu coarsely so anything gated on them can move.
            if i % 1000 == 0 {
                gba.bus.step_timers();
            }
        }
        return;
    }
    // Track the most "interesting" frame (most distinct colours) so a single PNG
    // lands on a real rendered screen, not a blank/forced-blank transition frame.
    let mut best_fb: Vec<u16> = Vec::new();
    let mut best_distinct = 0usize;
    let mut best_frame = 0u32;
    let trace = std::env::var("GBA_TRACE").is_ok();
    // Auto-advance menus: hold A/Start in short pulses to reach in-game scenes.
    let autoinput = std::env::var_os("GBA_AUTOINPUT").is_some();
    let mut audio: Vec<i16> = Vec::new();
    // GBA_APULOG: per-frame sound state, and flag frames whose audio has a large
    // internal discontinuity (a "blip"), to correlate blips with what the game
    // did to the sound registers / DISPCNT around a screen transition.
    let apulog = std::env::var_os("GBA_APULOG").is_some();
    let mut prev_sound = (0u8, 0u16, 0u16, 0u16);
    let (mut lp, mut lu, mut lr) = (0u64, 0u64, 0u64);
    let mut lirq = 0u64;
    for f in 0..frames {
        if autoinput {
            let p = f % 24 < 4; // pulse A + Start to advance title and dialogue
            gba.set_button(gba_core::Button::A, p);
            gba.set_button(gba_core::Button::Start, p);
        }
        let fb = gba.run_frame().to_vec();
        let a = gba.take_audio();
        if apulog {
            let blip = a
                .chunks_exact(2)
                .map(|s| s[0] as i32)
                .collect::<Vec<_>>()
                .windows(2)
                .map(|w| (w[1] - w[0]).abs())
                .max()
                .unwrap_or(0);
            let sx = gba.bus.apu.read8(0x84);
            let sl = u16::from_le_bytes([gba.bus.apu.read8(0x80), gba.bus.apu.read8(0x81)]);
            let sh = u16::from_le_bytes([gba.bus.apu.read8(0x82), gba.bus.apu.read8(0x83)]);
            let dispcnt = gba.bus.ppu.read_reg16(0);
            let irqd = gba.irqs_taken.wrapping_sub(lirq);
            lirq = gba.irqs_taken;
            let (pops, und, refs) =
                (gba.bus.apu.dbg_pops, gba.bus.apu.dbg_underruns, gba.bus.dbg_fifo_refills);
            let (dp, du, dr) = (pops - lp, und - lu, refs - lr);
            lp = pops;
            lu = und;
            lr = refs;
            let per = gba.bus.ds_timer_period(0);
            let rate = per.map(|p| 16_777_216 / p.max(1)).unwrap_or(0);
            let cur = (sx, sl, sh, dispcnt);
            if cur != prev_sound || blip > 6000 || du > 0 {
                let (s1, _d1, c1, r1) = gba.bus.dma_dbg(1);
                println!(
                    "  f{f:>4} blip={blip:>5} H={sh:04X} DISP={dispcnt:04X} fA={:>2} irq={irqd:>2} T0rate={rate:>5} pops={dp:>4} under={du:>4} refills={dr:>3} | DMA1 sad={s1:07X}->run{r1:07X} ctl={c1:04X}",
                    gba.bus.apu.fifo_a_len(),
                );
                prev_sound = cur;
                if blip > 7000 {
                    // Dump the sample buffer the sound DMA is sourcing, as signed
                    // bytes, to see whether it is plausible PCM or garbage.
                    let (_, _, _, run) = gba.bus.dma_dbg(1);
                    let bytes: Vec<i32> =
                        (0..24).map(|k| gba.bus.read8(run + k, Access::NonSeq) as i8 as i32).collect();
                    println!("      run@{run:07X}: {bytes:?}");
                }
            }
        }
        audio.extend(a);
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
    // Audio stats (measurable, no judgement): sample count, peak, RMS, and how
    // many samples were non-zero. GBA_WAV=<file> also dumps a 16-bit stereo WAV.
    {
        let frames_out = audio.len() / 2;
        let peak = audio.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
        let nonzero = audio.iter().filter(|&&s| s != 0).count();
        let rms = if !audio.is_empty() {
            let sum: f64 = audio.iter().map(|&s| (s as f64).powi(2)).sum();
            (sum / audio.len() as f64).sqrt()
        } else {
            0.0
        };
        println!(
            "  audio: {frames_out} stereo samples ({:.1}/frame), peak {peak}, rms {rms:.1}, {nonzero} non-zero",
            frames_out as f64 / frames.max(1) as f64
        );
        if let Some(path) = std::env::var_os("GBA_WAV") {
            match write_wav(&path, &audio, gba_core::Gba::SAMPLE_RATE) {
                Ok(()) => eprintln!("wrote wav {:?} ({} samples)", path, frames_out),
                Err(e) => eprintln!("could not write wav {:?}: {e}", path),
            }
        }
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
        {
            use gba_core::save::SaveKind;
            let sv = &gba.bus.save;
            let nonzero = sv.data.iter().filter(|&&b| b != 0).count();
            println!("  SAVE: kind={:?} size={} nonzero={nonzero}  data[0..16]={:02X?}",
                sv.kind, sv.data.len(), &sv.data[..sv.data.len().min(16)]);
        }
        // Palette dump (to spot a washed/faded/duplicated palette).
        let pe = |i: usize| u16::from_le_bytes([gba.bus.ppu.palram[i * 2], gba.bus.ppu.palram[i * 2 + 1]]);
        let row = |base: usize| {
            (0..16).map(|i| format!("{:04X}", pe(base + i))).collect::<Vec<_>>().join(" ")
        };
        println!("  BG pal[0..16]:  {}", row(0));
        println!("  BG pal[16..32]: {}", row(16));
        println!("  OBJ pal[0..16]: {}", row(0x100));
        // Search EWRAM/IWRAM for the rug-green 0x5BF4 to find the source palette
        // buffer, and show whether it is paired there or clean (distinct).
        // GBA_DUMP=<hexaddr>: dump 64 bytes of IWRAM/EWRAM as 32-bit words.
        if let Ok(a) = std::env::var("GBA_DUMP") {
            let addr = u32::from_str_radix(a.trim_start_matches("0x"), 16).unwrap_or(0x0300_0000);
            let (name, mem, off) = if addr >= 0x0300_0000 {
                ("IWRAM", &gba.bus.iwram[..], (addr & 0x7FFF) as usize)
            } else {
                ("EWRAM", &gba.bus.ewram[..], (addr & 0x3_FFFF) as usize)
            };
            print!("  {name}@{addr:08X}:");
            for i in 0..16 {
                let o = off + i * 4;
                if o + 4 <= mem.len() {
                    let w = u32::from_le_bytes([mem[o], mem[o + 1], mem[o + 2], mem[o + 3]]);
                    print!(" {w:08X}");
                }
            }
            println!();
        }
        if let Ok(hexs) = std::env::var("GBA_FINDHALF") {
            let needle = u16::from_str_radix(&hexs, 16).unwrap_or(0x5BF4);
            for (name, mem) in [("EWRAM", &gba.bus.ewram[..]), ("IWRAM", &gba.bus.iwram[..])] {
                let mut found = 0;
                let mut o = 0;
                while o + 8 <= mem.len() && found < 4 {
                    let h = u16::from_le_bytes([mem[o], mem[o + 1]]);
                    if h == needle {
                        let hw = |k: usize| u16::from_le_bytes([mem[o + k * 2], mem[o + k * 2 + 1]]);
                        println!("  {name}@{:05X}: {:04X} {:04X} {:04X} {:04X}", o, hw(0), hw(1), hw(2), hw(3));
                        found += 1;
                    }
                    o += 2;
                }
            }
        }
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
    if gba.bus.watch_addr != 0 {
        println!("  watch {:08X}: {} writes", gba.bus.watch_addr, gba.bus.watch_hits.len());
        for (pc, addr, val) in gba.bus.watch_hits.iter().take(24) {
            println!("    PC={pc:08X} w{} [{:07X}] = {val:08X}", addr >> 28, addr & 0x0FFF_FFFF);
        }
    }
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
