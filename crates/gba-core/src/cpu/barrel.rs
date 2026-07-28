// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! The barrel shifter.
//!
//! Produces `(value, carry_out)` for the four shift types plus the ARM
//! data-processing immediate operand. The carry-out feeds the C flag for
//! *logical* data-processing ops (arithmetic ops derive C from the ALU instead).
//!
//! The `imm` flag distinguishes the two ways a zero shift amount is encoded:
//! an **immediate** shift field of 0 means "special" (LSR/ASR/ROR #0 actually
//! mean 32, and ROR #0 is RRX), whereas a **register** shift of 0 leaves the
//! value and carry untouched.

/// Logical shift left.
pub fn lsl(v: u32, amt: u32, carry_in: bool) -> (u32, bool) {
    match amt {
        0 => (v, carry_in),
        1..=31 => (v << amt, (v >> (32 - amt)) & 1 != 0),
        32 => (0, v & 1 != 0),
        _ => (0, false),
    }
}

/// Logical shift right. With `imm`, an amount of 0 encodes LSR #32.
pub fn lsr(v: u32, amt: u32, carry_in: bool, imm: bool) -> (u32, bool) {
    match amt {
        0 if imm => (0, v >> 31 != 0), // LSR #32
        0 => (v, carry_in),            // register LSR #0: no-op
        1..=31 => (v >> amt, (v >> (amt - 1)) & 1 != 0),
        32 => (0, v >> 31 != 0),
        _ => (0, false),
    }
}

/// Arithmetic shift right. With `imm`, an amount of 0 encodes ASR #32.
pub fn asr(v: u32, amt: u32, carry_in: bool, imm: bool) -> (u32, bool) {
    let sv = v as i32;
    match amt {
        0 if imm => (((sv >> 31) as u32), v >> 31 != 0), // ASR #32 -> sign fill
        0 => (v, carry_in),
        1..=31 => ((sv >> amt) as u32, (v >> (amt - 1)) & 1 != 0),
        _ => (((sv >> 31) as u32), v >> 31 != 0), // >=32 -> all sign bits
    }
}

/// Rotate right. With `imm`, an amount of 0 encodes RRX (rotate right through
/// carry by one). Register ROR amounts are taken mod 32 (0 -> no-op, and a
/// multiple of 32 keeps the value but sets carry from bit 31).
pub fn ror(v: u32, amt: u32, carry_in: bool, imm: bool) -> (u32, bool) {
    if amt == 0 {
        if imm {
            // RRX: carry_in becomes new bit 31, old bit 0 becomes carry.
            let result = (v >> 1) | ((carry_in as u32) << 31);
            (result, v & 1 != 0)
        } else {
            (v, carry_in)
        }
    } else {
        let a = amt & 31;
        if a == 0 {
            // multiple of 32: value unchanged, carry from bit 31.
            (v, v >> 31 != 0)
        } else {
            (v.rotate_right(a), (v >> (a - 1)) & 1 != 0)
        }
    }
}

/// The ARM data-processing *immediate* operand (bit 25 set): an 8-bit value
/// rotated right by twice the 4-bit rotate field. A zero rotate leaves the
/// carry untouched; any non-zero rotate sets carry from the result's bit 31.
pub fn rotate_immediate(op: u32, carry_in: bool) -> (u32, bool) {
    let imm = op & 0xFF;
    let rot = ((op >> 8) & 0xF) * 2;
    if rot == 0 {
        (imm, carry_in)
    } else {
        let v = imm.rotate_right(rot);
        (v, v >> 31 != 0)
    }
}