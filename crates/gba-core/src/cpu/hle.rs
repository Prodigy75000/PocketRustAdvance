//! High-level emulation of the GBA BIOS SWI functions.
//!
//! On direct boot (no real BIOS image) the CPU has nothing at the SWI vector to
//! run, so instead of vectoring to 0x08 we emulate the documented BIOS calls
//! here. Behavior follows GBATEK's "BIOS Functions" chapter — the register I/O
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
        0x00 => return soft_reset(cpu),
        0x01 => register_ram_reset(cpu, bus),
        0x02 | 0x03 => bus.set_halted(true), // Halt / Stop
        0x04 => intr_wait(cpu, bus),
        0x05 => {
            // VBlankIntrWait == IntrWait(discard=1, mask=VBlank).
            cpu.r[0] = 1;
            cpu.r[1] = 1;
            intr_wait(cpu, bus);
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

/// SWI 00h SoftReset: jump back to 0x02000000 or 0x08000000 per the flag byte at
/// 0x03007FFA, reset the stacks, and enter System mode.
fn soft_reset(cpu: &mut Arm7tdmi) -> bool {
    // The flag is written before calling; default (0) => cartridge at 0x08000000.
    // We read it lazily via the register the caller staged is not available, so
    // default to the ROM entry (the common case for a boot-time soft reset).
    cpu.r[13] = 0x0300_7F00; // SP_usr/sys
    cpu.write_cpsr((cpu.cpsr & !0x1F) | 0x1F); // System mode
    cpu.r[15] = 0x0800_0000;
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
/// Modeled as: force IME on, then halt — the frame loop wakes us on the IRQ.
fn intr_wait<B: Bus>(_cpu: &mut Arm7tdmi, bus: &mut B) {
    bus.write16(0x0400_0208, 1, N); // IME = 1
    bus.set_halted(true);
}

/// SWI 06h/07h Div: signed 32-bit division. r0 = num/den, r1 = num%den,
/// r3 = |num/den|.
fn div(cpu: &mut Arm7tdmi, number: i32, denom: i32) {
    if denom == 0 {
        return; // hardware behavior is undefined; leave registers untouched
    }
    let q = number.wrapping_div(denom);
    let r = number.wrapping_rem(denom);
    cpu.r[0] = q as u32;
    cpu.r[1] = r as u32;
    cpu.r[3] = q.unsigned_abs();
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
                    let byte = out[out.len() - disp];
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
