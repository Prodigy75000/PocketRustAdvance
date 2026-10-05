// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! High-level emulation of the GBA BIOS SWI functions.
//!
//! On direct boot (no real BIOS image) the CPU has nothing at the SWI vector to
//! run, so instead of vectoring to 0x08 we emulate the documented BIOS calls
//! here. Behavior follows GBATEK's "BIOS Functions" chapter - the register I/O
//! contract, the decompression formats, and the arithmetic helpers. This is a
//! clean-room reimplementation from that specification, not a port of any BIOS.
//!
//! Only the outputs each call documents are produced; the BIOS's incidental
//! clobbering of r0..r3/r12 is not modeled beyond the documented results.

use super::Arm7tdmi;
use crate::bus::{Access, Bus};

const N: Access = Access::NonSeq;

/// Dispatch a SWI by its comment-field number. Returns `true` if it changed the
/// PC (a branch, e.g. SoftReset) and the pipeline must be refilled.
pub fn swi<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, num: u8) -> bool {
    match num {
        0x00 => return soft_reset(cpu, bus),
        0x01 => register_ram_reset(cpu, bus),
        0x02 | 0x03 => bus.set_halted(true), // Halt / Stop
        0x04 => return intr_wait(cpu, bus),
        0x05 => {
            // VBlankIntrWait == IntrWait(discard=1, mask=VBlank). The real BIOS
            // implements it by literally loading these two and falling into 04h,
            // so clobbering them is hardware behaviour, not a shortcut.
            cpu.r[0] = 1;
            cpu.r[1] = 1;
            return intr_wait(cpu, bus);
        }
        0x06 => div(cpu, cpu.r[0] as i32, cpu.r[1] as i32),
        0x07 => div(cpu, cpu.r[1] as i32, cpu.r[0] as i32), // DivArm: args swapped
        0x08 => cpu.r[0] = isqrt(cpu.r[0]),
        0x09 => cpu.r[0] = arctan(cpu.r[0] as i32 as i16 as i32) as u32,
        0x0A => cpu.r[0] = arctan2(cpu.r[0] as i16 as i32, cpu.r[1] as i16 as i32),
        0x0B => cpu_set(cpu, bus),
        0x0C => cpu_fast_set(cpu, bus),
        0x0D => cpu.r[0] = 0xBAAE_187F, // GetBiosChecksum (the real DS/GBA value)
        0x10 => bit_unpack(cpu, bus),
        0x11 | 0x12 => lz77_uncomp(cpu, bus, num == 0x12),
        0x13 => huff_uncomp(cpu, bus),
        0x14 | 0x15 => rl_uncomp(cpu, bus, num == 0x15),
        0x16 | 0x17 => diff8_unfilter(cpu, bus, num == 0x17),
        0x18 => diff16_unfilter(cpu, bus),
        // BgAffineSet/ObjAffineSet/SoundBias/etc.: not needed for boot rendering.
        _ => {}
    }
    false
}

/// SWI 00h SoftReset, per GBATEK, and taken over even when a BIOS IS present.
///
/// **This is what stops the bundled open BIOS showing its boot logo mid-game.**
/// Normmatt's BIOS implements SoftReset by re-entering its own reset path, which
/// replays the boot animation; real hardware does not, it goes straight back to
/// the entry point. Measured: with the open BIOS a SoftReset puts 29% of the
/// following frames' instructions inside the BIOS, against 0% here. gpSP bundles
/// the same BIOS, which is why the owner has seen the same logo there.
///
/// Taking a BIOS call over is normally the wrong move, for the reason written on
/// [`patch_div`]: returning instantly where the BIOS burns a loop gives a timing
/// profile neither a real BIOS nor full HLE has. SoftReset is the case where that
/// does not apply. It happens once, it discards all machine state by definition,
/// and nothing can be timing-coupled to a reset that is about to zero the stacks.
///
/// GBATEK: zero 0x200 bytes at 0x03007E00, set the three stack pointers, clear
/// r0-r12 and the banked LR/SPSR for Supervisor and IRQ, enter System mode, and
/// jump per the flag byte at 0x03007FFA (0 = ROM, else EWRAM).
fn soft_reset<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) -> bool {
    // Read the entry flag BEFORE zeroing, since it lives inside the cleared area.
    let flag = bus.read8(0x0300_7FFA, N);
    let entry = if flag == 0 { 0x0800_0000 } else { 0x0200_0000 };

    for off in (0..0x200u32).step_by(4) {
        bus.write32(0x0300_7E00 + off, 0, N);
    }

    let mut r = [0u32; 16];
    r[13] = 0x0300_7F00; // SP_usr/sys
    r[15] = entry;
    cpu.load_full(
        0x1F,              // System mode, IRQ/FIQ unmasked, ARM state
        r,
        [0; 7],            // FIQ bank
        [0x0300_7FE0, 0],  // SP_svc, LR_svc
        [0; 2],            // Abort bank
        [0x0300_7FA0, 0],  // SP_irq, LR_irq
        [0; 2],            // Undefined bank
        [0; 5],            // every SPSR
    );
    true
}

/// SWI 01h RegisterRamReset: clear the RAM/IO regions selected by the r0 bitmask
/// and force-blank the display, mirroring the BIOS reset.
fn register_ram_reset<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let flags = cpu.r[0];
    let clear = |bus: &mut B, start: u32, len: u32| {
        let mut a = start;
        while a < start + len {
            bus.write32(a, 0, N);
            a += 4;
        }
    };
    if flags & 0x01 != 0 {
        clear(bus, 0x0200_0000, 0x0004_0000); // EWRAM (256K)
    }
    if flags & 0x02 != 0 {
        clear(bus, 0x0300_0000, 0x0000_7E00); // IWRAM, less BIOS scratch at top
    }
    if flags & 0x04 != 0 {
        clear(bus, 0x0500_0000, 0x0000_0400); // palette
    }
    if flags & 0x08 != 0 {
        clear(bus, 0x0600_0000, 0x0001_8000); // VRAM (96K)
    }
    if flags & 0x10 != 0 {
        clear(bus, 0x0700_0000, 0x0000_0400); // OAM
    }
    // The BIOS always leaves the LCD force-blanked after a RAM reset.
    bus.write16(0x0400_0000, 0x0080, N);
}

/// SWI 04h/05h IntrWait: park the CPU until an interrupt in the r1 mask fires.
/// Modeled as: force IME on, then halt - the frame loop wakes us on the IRQ.
/// The BIOS Interrupt Flags halfword, at 0x03007FF8 (GBATEK). Not the IF
/// register: this is a separate copy in IWRAM that the GAME's interrupt handler
/// is required to maintain, by ORing into it whatever it writes to IF.
const BIOS_IF: u32 = 0x0300_7FF8;

/// SWI 04h IntrWait, and 05h VBlankIntrWait through it.
///
/// Waits in Halt until one of the interrupt flags in r1 shows up in [`BIOS_IF`],
/// then clears exactly those flags there and returns. r0 selects whether flags
/// that were ALREADY pending count: 0 returns immediately on an old flag, 1
/// discards the old ones first and waits for a genuinely new one.
///
/// **It cannot block, so it re-executes instead.** A SWI handler here is an
/// ordinary function call that has to return, and there is no way to suspend
/// inside it. When the wait is not yet satisfied this halts AND rewinds R15 to
/// the SWI itself, so the instruction runs again when the next interrupt lifts
/// the halt, and re-tests. The frame loop clears `halted` on `ie & if_` without
/// consulting IME, so a wake cannot be lost.
///
/// Returning true tells the caller R15 was changed and the pipeline must refill.
///
/// **This waits forever if the game's handler never maintains [`BIOS_IF`]**, and
/// that is correct: real hardware and the bundled open BIOS both hang in exactly
/// the same way, which is why GBATEK prints a caution about it. The previous
/// implementation here halted once and returned unconditionally, which papered
/// over any game whose handler we were failing to reach.
fn intr_wait<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) -> bool {
    bus.write16(0x0400_0208, 1, N); // IME = 1, forcefully, per GBATEK

    let want = cpu.r[1] as u16;
    // Discard old flags once per WAIT, not once per pass. The re-execution below
    // re-runs the whole instruction, and SWI 05h re-loads r0 = 1 when it does, so
    // keying this off r0 alone would discard the flag the handler just posted and
    // the wait would never end. Measured: 1616 of 3732 ROMs stopped drawing.
    if cpu.r[0] != 0 && !cpu.in_intr_wait {
        let pending = bus.read16(BIOS_IF, N);
        bus.write16(BIOS_IF, pending & !want, N);
    }

    let pending = bus.read16(BIOS_IF, N);
    if pending & want != 0 {
        bus.write16(BIOS_IF, pending & !want, N);
        cpu.in_intr_wait = false;
        return false; // satisfied; fall through to the instruction after the SWI
    }

    cpu.in_intr_wait = true;
    bus.set_halted(true);
    // Rewind to the SWI. R15 runs two instructions ahead of the one executing,
    // so the SWI itself is at R15 minus one full pipeline.
    let back = if cpu.thumb() { 4 } else { 8 };
    cpu.r[15] = cpu.r[15].wrapping_sub(back);
    true
}

/// SWI 06h/07h Div: signed 32-bit division. r0 = num/den, r1 = num%den,
/// r3 = |num/den|.
///
/// r3 is not optional and not decorative. Measured 100/7 against a real BIOS
/// dump: r0=14, r1=2, r3=14. The bundled open BIOS returns r0 and r1 correctly
/// and leaves r3 untouched, so a game that uses it reads back whatever it
/// happened to be holding. Puppy Luv does exactly that and ends up with a stale
/// pointer in its interrupt-handler table, which it then branches to.
fn div(cpu: &mut Arm7tdmi, number: i32, denom: i32) {
    if denom == 0 {
        divide_by_zero(cpu, number);
        return;
    }
    let q = number.wrapping_div(denom);
    let r = number.wrapping_rem(denom);
    cpu.r[0] = q as u32;
    cpu.r[1] = r as u32;
    cpu.r[3] = q.unsigned_abs();
}

/// The divide-by-zero answer, measured against a real BIOS dump rather than
/// assumed. 0/0 returns r0=1, r1=0, r3=1 and returns normally; N/0 for any
/// nonzero N parks the real BIOS in a loop at 0x03D0 and never comes back.
///
/// Only the 0/0 answer is something a shipped game can depend on, since a game
/// doing N/0 would hang on hardware too. So that is the only case answered
/// here: N/0 leaves the registers alone rather than inventing a value the
/// console never produces, which would mask a bogus divisor coming from a bug
/// of our own.
///
/// Returns true when the call was answered.
fn divide_by_zero(cpu: &mut Arm7tdmi, number: i32) -> bool {
    if number != 0 {
        return false;
    }
    cpu.r[0] = 1;
    cpu.r[1] = 0;
    cpu.r[3] = 1;
    true
}

/// Answer SWI 06h/07h with a zero divisor without vectoring into BIOS code.
///
/// This exists for the BIOS-boot path, which normally runs the BIOS's own
/// instructions. The bundled open BIOS (Normmatt's) has no divide-by-zero guard
/// at all, so 0/0 spins in its normalization loop forever where hardware
/// returns. Blackthorne makes exactly one such call about ten frames into its
/// boot, and the rest of the Blizzard/Interplay port family does the same.
///
/// Scoped as narrowly as it can be: one SWI pair, one operand value, and only
/// the case hardware itself defines. The third-party binary stays untouched.
///
/// Returns true when the call was answered and must not reach the vector.
/// SWI 00h is handled here even with a BIOS loaded: see [`soft_reset`].
pub fn patch_soft_reset<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, num: u8) -> bool {
    num == 0x00 && soft_reset(cpu, bus)
}

pub fn patch_div(cpu: &mut Arm7tdmi, num: u8) -> bool {
    let (number, denom) = match num {
        0x06 => (cpu.r[0] as i32, cpu.r[1] as i32),
        0x07 => (cpu.r[1] as i32, cpu.r[0] as i32), // DivArm: args swapped
        _ => return false,
    };
    if denom == 0 {
        // 0/0 is answered outright, because the open BIOS spins on it forever
        // where hardware returns. N/0 falls through and hangs in the BIOS,
        // which is also what hardware does, so we leave it alone.
        return divide_by_zero(cpu, number);
    }
    // An ordinary divide is NOT taken over. Only r3 is seeded, and then the
    // BIOS runs and does the arithmetic itself.
    //
    // Doing the whole division here instead was measured and rejected: our
    // answer is right (it matches a real BIOS dump on every edge case tried,
    // including i32::MIN / -1), but returning instantly where the BIOS burns a
    // shift-and-subtract loop gives a timing profile that neither a real BIOS
    // nor full HLE has, and Happy Feet lost its title screen to it in both
    // regions. Seeding r3 leaves every cycle where it was.
    //
    // This works because the open BIOS preserves r3 across the call rather than
    // writing it, which is the defect; a BIOS that does write r3 simply
    // overwrites this with the same value, so the seed is inert there.
    cpu.r[3] = number.wrapping_div(denom).unsigned_abs();
    false
}

/// SWI 08h Sqrt: integer square root of an unsigned 32-bit value.
fn isqrt(v: u32) -> u32 {
    if v == 0 {
        return 0;
    }
    let mut x = (v as f64).sqrt() as u32;
    // Correct any floating rounding at the boundary.
    while x.saturating_mul(x) > v {
        x -= 1;
    }
    while (x + 1).saturating_mul(x + 1) <= v {
        x += 1;
    }
    x
}

/// SWI 09h ArcTan: signed 1.14 fixed-point input -> angle. A compact polynomial
/// approximation matching the BIOS's published algorithm closely enough for use.
fn arctan(x: i32) -> u16 {
    // GBATEK's documented approximation.
    let a = x as i64;
    let b = -((a * a) >> 14);
    let mut c = ((0x00A9 * b) >> 14) + 0x0390;
    c = ((c * b) >> 14) + 0x091C;
    c = ((c * b) >> 14) + 0x0FB6;
    c = ((c * b) >> 14) + 0x16AA;
    c = ((c * b) >> 14) + 0x2081;
    c = ((c * b) >> 14) + 0x3651;
    c = ((c * b) >> 14) + 0xA2F9;
    ((a * c) >> 16) as u16
}

/// SWI 0Ah ArcTan2: full-circle angle of (x, y) in BIOS units (0..0xFFFF).
fn arctan2(x: i32, y: i32) -> u32 {
    let r = if y == 0 {
        if x < 0 {
            0x8000
        } else {
            0x0000
        }
    } else if x == 0 {
        if y < 0 {
            0xC000
        } else {
            0x4000
        }
    } else if y.abs() > x.abs() {
        let a = arctan(((x << 14) / y) as i32) as i32;
        if y < 0 {
            0xC000u32.wrapping_add(a as u32)
        } else {
            0x4000u32.wrapping_add(a as u32)
        }
    } else {
        let a = arctan(((y << 14) / x) as i32) as i32;
        if x < 0 {
            0x8000u32.wrapping_add(a as u32)
        } else {
            (a as u32) & 0xFFFF
        }
    };
    r & 0xFFFF
}

/// SWI 0Bh CpuSet: copy or fill in 16- or 32-bit units. r0=src, r1=dst,
/// r2 = count(0..20) | fixed-source(24) | 32-bit(26).
fn cpu_set<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let (mut src, mut dst, ctl) = (cpu.r[0], cpu.r[1], cpu.r[2]);
    let count = ctl & 0x000F_FFFF;
    let fill = ctl & (1 << 24) != 0;
    let word = ctl & (1 << 26) != 0;
    if word {
        src &= !3;
        dst &= !3;
        let mut val = bus.read32(src, N);
        for _ in 0..count {
            if !fill {
                val = bus.read32(src, N);
                src += 4;
            }
            bus.write32(dst, val, N);
            dst += 4;
        }
    } else {
        src &= !1;
        dst &= !1;
        let mut val = bus.read16(src, N);
        for _ in 0..count {
            if !fill {
                val = bus.read16(src, N);
                src += 2;
            }
            bus.write16(dst, val, N);
            dst += 2;
        }
    }
}

/// SWI 0Ch CpuFastSet: 32-bit copy/fill, count rounded up to a multiple of 8.
fn cpu_fast_set<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let (mut src, mut dst, ctl) = (cpu.r[0] & !3, cpu.r[1] & !3, cpu.r[2]);
    let count = ((ctl & 0x000F_FFFF) + 7) & !7;
    let fill = ctl & (1 << 24) != 0;
    let mut val = bus.read32(src, N);
    for _ in 0..count {
        if !fill {
            val = bus.read32(src, N);
            src += 4;
        }
        bus.write32(dst, val, N);
        dst += 4;
    }
}

// --- Decompression -----------------------------------------------------------
//
// The compressed streams share a 32-bit header at the source: bits 0..3 = type,
// bits 4..7 = width, bits 8..31 = decompressed byte length. We decode into a
// scratch buffer and then store it out, honoring the VRAM variants' 16-bit-only
// writes.

fn write_out<B: Bus>(bus: &mut B, dst: u32, data: &[u8], vram: bool) {
    if vram {
        let mut i = 0;
        while i + 1 < data.len() {
            let h = u16::from_le_bytes([data[i], data[i + 1]]);
            bus.write16(dst + i as u32, h, N);
            i += 2;
        }
        if i < data.len() {
            bus.write16(dst + i as u32, data[i] as u16, N);
        }
    } else {
        for (i, &b) in data.iter().enumerate() {
            bus.write8(dst + i as u32, b, N);
        }
    }
}

/// SWI 11h/12h LZ77UnComp: LZSS with 8-flag groups (MSB first). A set flag is a
/// back-reference (len 3..18, disp 1..4096); a clear flag is a literal byte.
fn lz77_uncomp<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, vram: bool) {
    let mut src = cpu.r[0];
    let dst = cpu.r[1];
    let header = bus.read32(src, N);
    src += 4;
    let size = (header >> 8) as usize;
    let mut out: Vec<u8> = Vec::with_capacity(size);
    while out.len() < size {
        let flags = bus.read8(src, N);
        src += 1;
        for bit in 0..8 {
            if out.len() >= size {
                break;
            }
            if flags & (0x80 >> bit) != 0 {
                let b0 = bus.read8(src, N) as usize;
                let b1 = bus.read8(src + 1, N) as usize;
                src += 2;
                let len = (b0 >> 4) + 3;
                let disp = ((b0 & 0xF) << 8 | b1) + 1;
                for _ in 0..len {
                    if out.len() >= size {
                        break;
                    }
                    // A back-reference before the output start reads the memory
                    // preceding the destination on real hardware (uninitialised,
                    // effectively 0). Guard the usize underflow rather than panic.
                    let idx = out.len().wrapping_sub(disp);
                    let byte = out.get(idx).copied().unwrap_or(0);
                    out.push(byte);
                }
            } else {
                out.push(bus.read8(src, N));
                src += 1;
            }
        }
    }
    write_out(bus, dst, &out, vram);
}

/// SWI 14h/15h RLUnComp: run-length. A flag byte's top bit selects a compressed
/// run (len = bits0..6 + 3, one repeated byte) or a literal run (len + 1 bytes).
fn rl_uncomp<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, vram: bool) {
    let mut src = cpu.r[0];
    let dst = cpu.r[1];
    let header = bus.read32(src, N);
    src += 4;
    let size = (header >> 8) as usize;
    let mut out: Vec<u8> = Vec::with_capacity(size);
    while out.len() < size {
        let flag = bus.read8(src, N);
        src += 1;
        if flag & 0x80 != 0 {
            let len = (flag & 0x7F) as usize + 3;
            let byte = bus.read8(src, N);
            src += 1;
            for _ in 0..len {
                if out.len() >= size {
                    break;
                }
                out.push(byte);
            }
        } else {
            let len = (flag & 0x7F) as usize + 1;
            for _ in 0..len {
                if out.len() >= size {
                    break;
                }
                out.push(bus.read8(src, N));
                src += 1;
            }
        }
    }
    write_out(bus, dst, &out, vram);
}

/// SWI 16h/17h Diff8bitUnFilter: reverse a per-byte delta filter (out[i] =
/// out[i-1] + data[i]).
fn diff8_unfilter<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, vram: bool) {
    let mut src = cpu.r[0];
    let dst = cpu.r[1];
    let header = bus.read32(src, N);
    src += 4;
    let size = (header >> 8) as usize;
    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut acc: u8 = 0;
    while out.len() < size {
        acc = acc.wrapping_add(bus.read8(src, N));
        src += 1;
        out.push(acc);
    }
    write_out(bus, dst, &out, vram);
}

/// SWI 18h Diff16bitUnFilter: reverse a per-halfword delta filter.
fn diff16_unfilter<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let mut src = cpu.r[0];
    let dst = cpu.r[1];
    let header = bus.read32(src, N);
    src += 4;
    let size = (header >> 8) as usize; // bytes
    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut acc: u16 = 0;
    while out.len() < size {
        acc = acc.wrapping_add(bus.read16(src, N));
        src += 2;
        out.extend_from_slice(&acc.to_le_bytes());
    }
    write_out(bus, dst, &out, true); // Diff16 always targets 16-bit memory
}


/// SWI 10h BitUnPack: expand packed 1/2/4/8-bit source units into wider
/// destination units, adding a base offset. r2 -> a parameter block:
/// {len:u16 (source bytes), src_w:u8, dst_w:u8, offset:u32 (bit31 = also add
/// the offset to zero units)}.
fn bit_unpack<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let mut src = cpu.r[0];
    let mut dst = cpu.r[1];
    let param = cpu.r[2];
    let len = bus.read16(param, N) as u32;
    let src_w = bus.read8(param + 2, N) as u32;
    let dst_w = bus.read8(param + 3, N) as u32;
    let off_word = bus.read32(param + 4, N);
    let offset = off_word & 0x7FFF_FFFF;
    let zero_off = off_word & 0x8000_0000 != 0;
    if src_w == 0 || dst_w == 0 {
        return;
    }
    let src_mask = (1u32 << src_w) - 1;

    let mut out: u32 = 0; // accumulates one 32-bit destination word
    let mut out_bits = 0u32;
    for _ in 0..len {
        let byte = bus.read8(src, N) as u32;
        src += 1;
        let mut bitpos = 0u32;
        while bitpos < 8 {
            let unit = (byte >> bitpos) & src_mask;
            let val = if unit != 0 || zero_off { unit + offset } else { 0 };
            out |= val << out_bits;
            out_bits += dst_w;
            if out_bits >= 32 {
                bus.write32(dst, out, N);
                dst += 4;
                out = 0;
                out_bits = 0;
            }
            bitpos += src_w;
        }
    }
    if out_bits > 0 {
        bus.write32(dst, out, N);
    }
}

/// SWI 13h HuffUnComp: Huffman decode (GBATEK format).
///
/// Header (4 bytes): bits 0..3 = symbol bit-width (4 or 8), bits 8..31 =
/// decompressed byte length. Then a 1-byte tree-table size, the tree table
/// (8-bit nodes, root first), then the bitstream as little-endian 32-bit words
/// read MSB-first. Each tree node's bits 0..5 give the child-pair offset; bit 7
/// marks the left (bit=0) child a leaf, bit 6 the right (bit=1) child.
fn huff_uncomp<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B) {
    let src = cpu.r[0];
    let dst = cpu.r[1];
    let header = bus.read32(src, N);
    let sym_bits = header & 0x0F;
    let size = (header >> 8) as usize;
    let tree_base = src + 4;
    let tree_size = (bus.read8(tree_base, N) as u32 + 1) * 2; // bytes
    let mut stream = tree_base + tree_size;

    let mut out: Vec<u8> = Vec::with_capacity(size);
    let mut nibble_hi = false;
    let mut pending: u8 = 0;

    let mut word = 0u32;
    let mut bits_left = 0u32;

    while out.len() < size {
        let mut cur = tree_base + 1; // root node
        loop {
            if bits_left == 0 {
                word = bus.read32(stream, N);
                stream += 4;
                bits_left = 32;
            }
            let bit = (word >> 31) & 1;
            word <<= 1;
            bits_left -= 1;

            let node = bus.read8(cur, N) as u32;
            let child = (cur & !1) + (node & 0x3F) * 2 + 2 + bit;
            let leaf_flag = if bit == 0 { 0x80 } else { 0x40 };
            if node & leaf_flag != 0 {
                let data = bus.read8(child, N) as u32;
                if sym_bits == 8 {
                    out.push(data as u8);
                } else {
                    // 4-bit symbols pack low-nibble-first into output bytes.
                    if nibble_hi {
                        out.push(pending | ((data as u8 & 0xF) << 4));
                        nibble_hi = false;
                    } else {
                        pending = data as u8 & 0xF;
                        nibble_hi = true;
                    }
                }
                break;
            }
            cur = child;
        }
    }
    write_out(bus, dst, &out, true);
}
#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::Bus as _;
    use crate::Gba;

    /// An HLE machine: no BIOS image, so SWIs come here.
    fn hle() -> Gba {
        Gba::new(vec![0u8; 0x2000], Vec::new())
    }

    fn bios_if(g: &mut Gba) -> u16 {
        g.bus.read16(BIOS_IF, N)
    }

    /// r0 = 0 means an already-pending flag satisfies the wait at once. The
    /// flags waited for must be cleared on the way out, or the next call returns
    /// immediately on a stale flag and the game free-runs.
    #[test]
    fn intr_wait_returns_at_once_on_a_flag_that_is_already_pending() {
        let mut g = hle();
        g.bus.write16(BIOS_IF, 0x0003, N); // VBlank and HBlank pending
        g.cpu.r[0] = 0;
        g.cpu.r[1] = 0x0001; // wait on VBlank only
        let pc = g.cpu.r[15];
        let moved = swi(&mut g.cpu, &mut g.bus, 0x04);
        assert!(!moved, "a satisfied wait must fall through, not rewind");
        assert!(!g.bus.halted, "a satisfied wait must not halt");
        assert_eq!(bios_if(&mut g), 0x0002, "only the waited-for flag is cleared");
        assert_eq!(g.cpu.r[15], pc, "PC untouched when the wait is satisfied");
    }

    /// r0 = 1 discards what is already pending and waits for something NEW. This
    /// is the mode every VBlankIntrWait uses, so getting it wrong would make a
    /// game run a frame ahead of itself forever.
    #[test]
    fn intr_wait_with_discard_ignores_a_stale_flag_and_waits() {
        let mut g = hle();
        g.bus.write16(BIOS_IF, 0x0001, N);
        g.cpu.r[0] = 1;
        g.cpu.r[1] = 0x0001;
        let pc = g.cpu.r[15];
        let moved = swi(&mut g.cpu, &mut g.bus, 0x04);
        assert!(moved, "an unsatisfied wait must rewind to re-execute");
        assert!(g.bus.halted, "an unsatisfied wait must halt");
        assert_eq!(bios_if(&mut g), 0, "the stale flag is discarded");
        assert_eq!(g.cpu.r[15], pc.wrapping_sub(8), "rewound to the ARM SWI itself");
        assert!(g.cpu.in_intr_wait, "the wait is marked, so a re-run cannot discard again");
    }

    /// The whole point of the rewind: the SWI runs again when the halt lifts, and
    /// completes once the game's handler has posted the flag. Without the r0
    /// clear above, this second call would discard the flag it is waiting for and
    /// wait forever.
    #[test]
    fn a_rewound_intr_wait_completes_once_the_handler_posts_the_flag() {
        let mut g = hle();
        g.bus.write16(BIOS_IF, 0x0001, N);
        g.cpu.r[0] = 1;
        g.cpu.r[1] = 0x0001;
        assert!(swi(&mut g.cpu, &mut g.bus, 0x04), "first pass waits");

        // What a game's interrupt handler is required to do (GBATEK): OR what it
        // wrote to IF into the BIOS flags.
        let posted = bios_if(&mut g) | 0x0001;
        g.bus.write16(BIOS_IF, posted, N);

        let pc = g.cpu.r[15];
        assert!(!swi(&mut g.cpu, &mut g.bus, 0x04), "second pass completes");
        assert_eq!(bios_if(&mut g), 0, "and consumes the flag");
        assert_eq!(g.cpu.r[15], pc, "PC untouched once satisfied");
    }

    /// Thumb rewinds by a Thumb pipeline, not an ARM one. Getting this wrong
    /// lands four bytes early, which is inside the previous instruction.
    #[test]
    fn the_rewind_matches_the_instruction_width() {
        let mut g = hle();
        g.cpu.write_cpsr(g.cpu.cpsr | (1 << 5)); // Thumb
        g.cpu.r[0] = 0;
        g.cpu.r[1] = 0x0001;
        let pc = g.cpu.r[15];
        assert!(swi(&mut g.cpu, &mut g.bus, 0x04));
        assert_eq!(g.cpu.r[15], pc.wrapping_sub(4), "rewound one Thumb pipeline");
    }

    /// SWI 05h is SWI 04h with both arguments forced, which is literally how the
    /// real BIOS implements it.
    #[test]
    fn vblank_intr_wait_is_intr_wait_on_the_vblank_flag() {
        let mut g = hle();
        g.cpu.r[0] = 0xDEAD;
        g.cpu.r[1] = 0xBEEF;
        g.bus.write16(BIOS_IF, 0x0001, N);
        let moved = swi(&mut g.cpu, &mut g.bus, 0x05);
        assert!(moved, "discard mode must wait for a new VBlank");
        assert_eq!(g.cpu.r[1], 1, "r1 forced to the VBlank flag");
        assert_eq!(bios_if(&mut g), 0, "the stale VBlank flag is discarded");
    }


    /// The one that matters, and the one my first cut got wrong.
    ///
    /// SWI 05h re-loads r0 = 1 every time it executes, and the wait re-executes
    /// the whole instruction on each wake. Keying the discard off r0 therefore
    /// threw away the flag the handler had just posted, every pass, forever. It
    /// passed all the single-pass tests and hung 1616 of 3732 ROMs in the corpus
    /// sweep. Drive at least two passes before believing a wait works.
    #[test]
    fn vblank_intr_wait_completes_across_a_re_execution() {
        let mut g = hle();
        g.bus.write16(BIOS_IF, 0x0001, N); // a stale VBlank, which must be discarded

        assert!(swi(&mut g.cpu, &mut g.bus, 0x05), "pass 1 discards and waits");
        assert_eq!(bios_if(&mut g), 0, "the stale flag went");

        // The handler posts a real VBlank while the CPU is halted.
        g.bus.write16(BIOS_IF, 0x0001, N);

        // Wake and re-execute. SWI 05h sets r0 = 1 again here; that must not
        // discard the flag we are waiting for.
        assert!(!swi(&mut g.cpu, &mut g.bus, 0x05), "pass 2 must COMPLETE");
        assert_eq!(bios_if(&mut g), 0, "and consume the flag");
        assert!(!g.cpu.in_intr_wait, "the wait is over");
    }

    /// A second wait after a completed one must discard again, or a game that
    /// stops waiting for a while comes back a frame early forever.
    #[test]
    fn a_fresh_wait_discards_again() {
        let mut g = hle();
        g.bus.write16(BIOS_IF, 0x0001, N);
        assert!(swi(&mut g.cpu, &mut g.bus, 0x05), "first wait parks");
        g.bus.write16(BIOS_IF, 0x0001, N);
        assert!(!swi(&mut g.cpu, &mut g.bus, 0x05), "first wait completes");

        // A stale flag appears, then the game waits again.
        g.bus.write16(BIOS_IF, 0x0001, N);
        assert!(swi(&mut g.cpu, &mut g.bus, 0x05), "the new wait must not accept a stale flag");
        assert_eq!(bios_if(&mut g), 0, "which means discarding it");
    }


    /// SoftReset must not reach the BIOS even when one is loaded. That is the
    /// whole point: Normmatt's BIOS replays its boot animation here, which is the
    /// logo the owner sees mid-game, and real hardware shows nothing.
    #[test]
    fn soft_reset_never_enters_the_bios_even_with_one_loaded() {
        // A BIOS whose SWI vector is an endless loop: if SoftReset reaches it,
        // the machine parks there and never returns to the cartridge.
        let mut bios = vec![0u8; 0x4000];
        bios[8..12].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes()); // 0x08: b .
        let mut rom = vec![0u8; 0x2000];
        rom[0..4].copy_from_slice(&0xEF00_0000u32.to_le_bytes()); // swi #0
        let mut g = crate::Gba::new(rom, bios);
        g.run_frame();
        assert_eq!(g.bios_steps, 0, "SoftReset must never vector into the BIOS");
        assert!(g.cpu.r[15] >= 0x0800_0000, "and must land back in the cartridge");
    }

    /// The entry point is the flag byte at 0x03007FFA, not a constant: 0 is the
    /// cartridge, anything else is EWRAM. Games that stage code in EWRAM and
    /// reset into it depend on this.
    #[test]
    fn soft_reset_honours_the_entry_flag() {
        for (flag, want) in [(0u8, 0x0800_0000u32), (1, 0x0200_0000)] {
            let mut g = hle();
            g.bus.write8(0x0300_7FFA, flag, N);
            soft_reset(&mut g.cpu, &mut g.bus);
            assert_eq!(g.cpu.r[15], want, "flag {flag} should enter at {want:08X}");
        }
    }

    /// GBATEK: 0x200 bytes at 0x03007E00 are cleared, the stacks are reset and
    /// System mode is entered. A game relying on a clean stack after a reset gets
    /// a dirty one otherwise.
    #[test]
    fn soft_reset_clears_the_stack_area_and_resets_the_stacks() {
        let mut g = hle();
        g.bus.write32(0x0300_7E00, 0xDEAD_BEEF, N);
        g.bus.write32(0x0300_7FFC, 0xDEAD_BEEF, N);
        soft_reset(&mut g.cpu, &mut g.bus);
        assert_eq!(g.bus.read32(0x0300_7E00, N), 0, "start of the cleared area");
        assert_eq!(g.bus.read32(0x0300_7FFC, N), 0, "end of the cleared area");
        assert_eq!(g.cpu.r[13], 0x0300_7F00, "SP_sys");
        assert_eq!(g.cpu.cpsr & 0x1F, 0x1F, "System mode");
    }

    /// IntrWait forcefully enables interrupts, and a game that halted with them
    /// masked relies on it: without this the wait could never be satisfied.
    #[test]
    fn intr_wait_forces_interrupts_on() {
        let mut g = hle();
        g.bus.write16(0x0400_0208, 0, N); // IME = 0
        g.cpu.r[0] = 1;
        g.cpu.r[1] = 0x0001;
        swi(&mut g.cpu, &mut g.bus, 0x04);
        assert_eq!(g.bus.read16(0x0400_0208, N) & 1, 1, "IME must be forced on");
    }
}
