// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! 32-bit ARM instruction execution.
//!
//! First format online: **data processing** (the 16 ALU operations). The rest
//! of the formats (loads/stores, multiply, block transfer, branch, PSR
//! transfer) land as their TomHarte vector files are brought in.

use super::barrel;
use super::psr::Flags;
use super::Arm7tdmi;
use crate::bus::{Access, Bus};

/// Execute one already-fetched ARM instruction whose condition has passed.
/// Returns `true` if it wrote R15 (a branch), so the caller flushes+refills the
/// pipeline instead of doing the normal single-word prefetch.
pub fn execute<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let class = (op >> 26) & 0b11;
    match class {
        0b00 => {
            // Branch-and-exchange hides in the data-processing space, matched by
            // its full fixed pattern before anything else.
            if op & 0x0FFF_FFF0 == 0x012F_FF10 {
                return branch_exchange(cpu, op);
            }
            // Multiply and halfword/signed transfers live ONLY in the
            // register-operand space (bit25=0). With bit25=1 the low byte is
            // immediate data, so these signatures must not be probed there.
            if (op >> 25) & 1 == 0 {
                if (op >> 4) & 0xF == 0x9 {
                    // bits 7..4 == 1001: the multiply / swap family.
                    if (op >> 22) & 0x3F == 0 {
                        return multiply(cpu, bus, op); // MUL / MLA
                    }
                    if (op >> 23) & 0x1F == 0b00001 {
                        return multiply_long(cpu, bus, op); // UMULL / UMLAL / SMULL / SMLAL
                    }
                    if (op >> 23) & 0x1F == 0b00010 {
                        return swap(cpu, bus, op); // SWP / SWPB
                    }
                } else if (op >> 4) & 0x9 == 0x9 {
                    // bit7 & bit4 set with a non-zero SH field (bits 6..5): the
                    // halfword / signed byte / signed halfword transfers.
                    return halfword_signed_transfer(cpu, bus, op);
                }
            }
            // Data-processing, unless it's a test-op with S=0 (a PSR transfer,
            // which lives in its own vector file and isn't handled yet).
            let opcode4 = (op >> 21) & 0xF;
            let s = (op >> 20) & 1 == 1;
            // A test-op (0x8..0xB) with S=0 is not data-processing: it's a PSR
            // transfer (MRS/MSR).
            if (0x8..=0xB).contains(&opcode4) && !s {
                return psr_transfer(cpu, op);
            }
            data_processing(cpu, op)
        }
        0b01 => single_data_transfer(cpu, bus, op), // LDR / STR
        0b10 if (op >> 25) & 1 == 1 => branch(cpu, op), // B / BL (bits 27..25 = 101)
        0b10 => block_data_transfer(cpu, bus, op), // LDM / STM (bits 27..25 = 100)
        0b11 if (op >> 24) & 0xF == 0xF => software_interrupt(cpu, bus, op), // SWI
        // Block transfer, multiply, etc. land as their vector files are brought
        // in. Unhandled: leave state untouched (fails the diff honestly rather
        // than silently mutating).
        _ => false,
    }
}

/// LDR / STR: single word/byte transfer with pre/post-indexed, up/down,
/// optionally writeback addressing and an immediate or shifted-register offset.
fn single_data_transfer<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let reg_offset = (op >> 25) & 1 == 1;
    let pre = (op >> 24) & 1 == 1;
    let up = (op >> 23) & 1 == 1;
    let byte = (op >> 22) & 1 == 1;
    let writeback = (op >> 21) & 1 == 1;
    let load = (op >> 20) & 1 == 1;
    let rn = ((op >> 16) & 0xF) as usize;
    let rd = ((op >> 12) & 0xF) as usize;

    let offset = if reg_offset {
        // A shifted register, but with an immediate shift amount only (bit4=0).
        let rm = (op & 0xF) as usize;
        let amount = (op >> 7) & 0x1F;
        let c = cpu.carry();
        let v = cpu.r[rm];
        let (val, _) = match (op >> 5) & 3 {
            0 => barrel::lsl(v, amount, c),
            1 => barrel::lsr(v, amount, c, true),
            2 => barrel::asr(v, amount, c, true),
            _ => barrel::ror(v, amount, c, true),
        };
        val
    } else {
        op & 0xFFF
    };

    let base = cpu.r[rn]; // Rn == 15 reads PC+8
    let offset_addr = if up {
        base.wrapping_add(offset)
    } else {
        base.wrapping_sub(offset)
    };
    // Pre-indexed uses the offset address; post-indexed accesses the base and
    // always writes the offset address back afterwards.
    let addr = if pre { offset_addr } else { base };

    // Either a load into R15 or a base writeback into R15 makes this a branch.
    let mut wrote_pc = false;

    if load {
        let value = if byte {
            bus.read8(addr, Access::NonSeq) as u32
        } else {
            // Word loads present the unaligned address on the bus, then rotate
            // the returned (aligned) word right by the byte offset.
            let raw = bus.read32(addr, Access::NonSeq);
            raw.rotate_right((addr & 3) * 8)
        };
        // Base writeback happens before the load commits, so if Rn == Rd the
        // loaded value wins.
        if !pre || writeback {
            // Writing R15 back as the base is a quirk: the stored value uses the
            // already-incremented PC (base+4), so it lands at offset_addr+4.
            cpu.r[rn] = if rn == 15 {
                offset_addr.wrapping_add(4)
            } else {
                offset_addr
            };
            wrote_pc |= rn == 15;
        }
        if rd == 15 {
            cpu.r[15] = value; // low bits are preserved in R15; the refill aligns fetches
            wrote_pc = true;
        } else {
            cpu.r[rd] = value;
        }
    } else {
        // Storing R15 stores the instruction address + 12 (PC+12 = R15+4).
        let value = if rd == 15 {
            cpu.r[15].wrapping_add(4)
        } else {
            cpu.r[rd]
        };
        if byte {
            bus.write8(addr, value as u8, Access::NonSeq);
        } else {
            // Stores present the unaligned address on the bus (the memory system
            // ignores the low bits); NanoBoyAdvance records it unaligned.
            bus.write32(addr, value, Access::NonSeq);
        }
        if !pre || writeback {
            // Writing R15 back as the base is a quirk: the stored value uses the
            // already-incremented PC (base+4), so it lands at offset_addr+4.
            cpu.r[rn] = if rn == 15 {
                offset_addr.wrapping_add(4)
            } else {
                offset_addr
            };
            wrote_pc |= rn == 15;
        }
    }

    wrote_pc
}

/// SWI: software interrupt — the canonical exception entry. Enters Supervisor
/// mode with the return address in LR_svc, the old CPSR in SPSR_svc, IRQs
/// disabled and ARM state forced, then vectors to 0x08.
fn software_interrupt<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let num = (op >> 16) as u8;
    super::swilog(num, cpu.r[15].wrapping_sub(8), &cpu.r);
    // With an HLE BIOS there is nothing at the vector; emulate the call directly.
    if cpu.hle_bios {
        return super::hle::swi(cpu, bus, num);
    }
    // The bundled open BIOS gets Div wrong twice: it hangs on 0/0, and it never
    // writes r3, which the contract says is abs(quotient). Both are repaired
    // around the BIOS rather than instead of it. See hle::patch_div.
    if super::hle::patch_div(cpu, num) {
        return false;
    }
    // Return address is the instruction after the SWI (R15 is PC+8 here).
    let return_addr = cpu.r[15].wrapping_sub(4);
    let old_cpsr = cpu.cpsr;

    // Switch to Supervisor mode: clears T (ARM state), sets I (mask IRQ), leaves
    // F. write_cpsr banks r13/r14 into the Supervisor set.
    let new_cpsr = (old_cpsr & !0x1F & !(1 << 5)) | 0x13 | (1 << 7);
    cpu.write_cpsr(new_cpsr);

    cpu.set_current_spsr(old_cpsr); // SPSR_svc = CPSR before the exception
    cpu.r[14] = return_addr; // LR_svc
    cpu.r[15] = 0x0000_0008; // SWI vector
    true // branch: refill from the vector
}

/// B / BL: PC-relative branch with a sign-extended 24-bit word offset. BL also
/// stashes the return address (next instruction) in the link register.
fn branch(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let link = (op >> 24) & 1 == 1;
    // 24-bit signed offset, shifted left 2: (<<8 to seat the sign, >>6 nets <<2).
    let offset = (((op & 0x00FF_FFFF) << 8) as i32 >> 6) as u32;
    if link {
        // R15 is PC+8 here, so the instruction after the branch is R15-4.
        cpu.r[14] = cpu.r[15].wrapping_sub(4);
    }
    cpu.r[15] = cpu.r[15].wrapping_add(offset);
    true
}

/// BX: branch to Rn, switching to Thumb when the target's low bit is set. The
/// state bit chooses the instruction width the pipeline then refills with.
fn branch_exchange(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let rn = (op & 0xF) as usize;
    let target = cpu.r[rn];
    if target & 1 != 0 {
        cpu.cpsr |= 1 << 5; // enter Thumb
    } else {
        cpu.cpsr &= !(1 << 5); // stay/return to ARM
    }
    // Clear the state-select low bit; the refill aligns the rest per width.
    cpu.r[15] = target & !1;
    true
}

/// `a + b + carry_in`, returning `(result, carry_out, signed_overflow)` with
/// ARM semantics. Subtraction is expressed as `a + !b + 1` by the callers.
fn addc(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let sum = a as u64 + b as u64 + carry_in as u64;
    let r = sum as u32;
    let carry = sum > 0xFFFF_FFFF;
    let overflow = (!(a ^ b) & (a ^ r)) >> 31 != 0;
    (r, carry, overflow)
}

fn data_processing(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let opcode4 = (op >> 21) & 0xF;
    let s = (op >> 20) & 1 == 1;
    let rn = ((op >> 16) & 0xF) as usize;
    let rd = ((op >> 12) & 0xF) as usize;

    // A register-specified shift (I=0, bit4=1) costs an extra pipeline step, so
    // any R15 read in this instruction sees PC+12 instead of PC+8.
    let reg_shift = (op >> 25) & 1 == 0 && (op >> 4) & 1 == 1;
    let pc_extra = if reg_shift { 4 } else { 0 };
    let read_reg = |cpu: &Arm7tdmi, i: usize| -> u32 {
        if i == 15 {
            cpu.r[15].wrapping_add(pc_extra)
        } else {
            cpu.r[i]
        }
    };

    let cin = cpu.carry();
    let (op2, shifter_c) = operand2(cpu, op, read_reg);
    let a = read_reg(cpu, rn);

    // Compute result plus arithmetic carry/overflow. Logical ops take C from the
    // shifter and leave V untouched.
    let (result, carry, overflow, logical): (u32, bool, bool, bool) = match opcode4 {
        0x0 | 0x8 => (a & op2, shifter_c, false, true),   // AND / TST
        0x1 | 0x9 => (a ^ op2, shifter_c, false, true),   // EOR / TEQ
        0x2 | 0xA => {
            let (r, c, v) = addc(a, !op2, true);          // SUB / CMP
            (r, c, v, false)
        }
        0x3 => {
            let (r, c, v) = addc(op2, !a, true);          // RSB
            (r, c, v, false)
        }
        0x4 | 0xB => {
            let (r, c, v) = addc(a, op2, false);          // ADD / CMN
            (r, c, v, false)
        }
        0x5 => {
            let (r, c, v) = addc(a, op2, cin);            // ADC
            (r, c, v, false)
        }
        0x6 => {
            let (r, c, v) = addc(a, !op2, cin);           // SBC
            (r, c, v, false)
        }
        0x7 => {
            let (r, c, v) = addc(op2, !a, cin);           // RSC
            (r, c, v, false)
        }
        0xC => (a | op2, shifter_c, false, true),         // ORR
        0xD => (op2, shifter_c, false, true),             // MOV
        0xE => (a & !op2, shifter_c, false, true),        // BIC
        0xF => (!op2, shifter_c, false, true),            // MVN
        _ => unreachable!(),
    };

    // TST/TEQ/CMP/CMN (0x8..0xB) never write a destination.
    let is_test = (0x8..=0xB).contains(&opcode4);

    if is_test {
        if rd == 15 && s {
            // TSTP/TEQP/CMPP/CMNP: the ARMv4 "P" forms move SPSR into CPSR
            // (a fast flag/mode restore) instead of just updating the flags.
            // They do not write PC, so this is not a branch.
            restore_or_set_flags(cpu, result, carry, overflow, logical);
        } else {
            set_flags(cpu, result, carry, overflow, logical);
        }
        return false;
    }

    if rd == 15 {
        if s {
            restore_or_set_flags(cpu, result, carry, overflow, logical);
        }
        cpu.r[15] = result;
        return true; // branch: caller refills the pipeline
    }

    cpu.r[rd] = result;
    if s {
        set_flags(cpu, result, carry, overflow, logical);
    }
    false
}

/// Internal (I) cycle count `m` of the ARM7TDMI 32x8 early-terminating
/// multiplier (ARM DDI 0029 datasheet): it consumes the multiplier operand 8
/// bits per cycle, stopping once the remaining high bits are all 0 or all 1.
fn multiply_m(rs: u32) -> u32 {
    let hi = rs & 0xFFFF_FF00;
    if hi == 0 || hi == 0xFFFF_FF00 {
        1
    } else if rs & 0xFFFF_0000 == 0 || rs & 0xFFFF_0000 == 0xFFFF_0000 {
        2
    } else if rs & 0xFF00_0000 == 0 || rs & 0xFF00_0000 == 0xFF00_0000 {
        3
    } else {
        4
    }
}

/// MUL / MLA: 32-bit multiply, optionally accumulating Rn, optionally setting
/// N/Z.
///
/// The C flag is deliberately left untouched. Per the ARM Architecture
/// Reference Manual (DDI 0100), on ARMv4/v4T the C flag is UNPREDICTABLE after
/// MUL/MLA (it only became "unchanged" from ARMv5). Leaving it is spec-valid;
/// the TomHarte vectors happen to encode one specific unpredictable value, which
/// we do not reproduce (doing so would mean reverse-engineering the silicon).
fn multiply<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let accumulate = (op >> 21) & 1 == 1;
    let set_flags = (op >> 20) & 1 == 1;
    let rd = ((op >> 16) & 0xF) as usize;
    let rn = ((op >> 12) & 0xF) as usize;
    let rs = ((op >> 8) & 0xF) as usize;
    let rm = (op & 0xF) as usize;

    // Multiply is multi-cycle, so an R15 operand reads PC+12. Read all operands
    // before writing Rd (the borrow of `cpu` must end before the mutation).
    let rv = |i: usize| if i == 15 { cpu.r[15].wrapping_add(4) } else { cpu.r[i] };
    let (rmv, rsv, rnv) = (rv(rm), rv(rs), rv(rn));
    let m = multiply_m(rsv);

    let mul = rmv.wrapping_mul(rsv);
    let result = if accumulate { mul.wrapping_add(rnv) } else { mul };
    cpu.r[rd] = result;

    // Timing (DDI 0029): MUL = S + mI, MLA = S + (m+1)I.
    bus.tick(if accumulate { m + 1 } else { m });

    if set_flags {
        let mut f = Flags::from_cpsr(cpu.cpsr);
        f.n = result >> 31 != 0;
        f.z = result == 0;
        cpu.cpsr = f.to_cpsr(cpu.cpsr);
    }
    rd == 15 // writing PC branches
}

/// UMULL / UMLAL / SMULL / SMLAL: 32x32 -> 64-bit multiply into RdHi:RdLo,
/// signed or unsigned, optionally accumulating the existing 64-bit value.
fn multiply_long<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let signed = (op >> 22) & 1 == 1;
    let accumulate = (op >> 21) & 1 == 1;
    let set_flags = (op >> 20) & 1 == 1;
    let rdhi = ((op >> 16) & 0xF) as usize;
    let rdlo = ((op >> 12) & 0xF) as usize;
    let rs = ((op >> 8) & 0xF) as usize;
    let rm = (op & 0xF) as usize;

    // Read all operands (incl. the accumulator) before writing the results.
    let rv = |i: usize| if i == 15 { cpu.r[15].wrapping_add(4) } else { cpu.r[i] };
    let (rmv, rsv) = (rv(rm), rv(rs));
    let acc = if accumulate {
        ((rv(rdhi) as u64) << 32) | rv(rdlo) as u64
    } else {
        0
    };
    let m = multiply_m(rsv);

    let mut result: u64 = if signed {
        (rmv as i32 as i64).wrapping_mul(rsv as i32 as i64) as u64
    } else {
        (rmv as u64).wrapping_mul(rsv as u64)
    };
    if accumulate {
        result = result.wrapping_add(acc);
    }
    cpu.r[rdlo] = result as u32;
    cpu.r[rdhi] = (result >> 32) as u32;

    // Timing (DDI 0029): MULL = S + (m+1)I, MLAL = S + (m+2)I. Same C-flag caveat
    // as MUL applies.
    bus.tick(if accumulate { m + 2 } else { m + 1 });

    if set_flags {
        let mut f = Flags::from_cpsr(cpu.cpsr);
        f.n = result >> 63 != 0;
        f.z = result == 0;
        cpu.cpsr = f.to_cpsr(cpu.cpsr);
    }
    rdhi == 15 || rdlo == 15 // writing PC branches
}

/// SWP / SWPB: atomically load [Rn] into Rd and store Rm to [Rn]. The read
/// happens before the write; a word swap rotates the loaded value like LDR.
fn swap<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let byte = (op >> 22) & 1 == 1;
    let rn = ((op >> 16) & 0xF) as usize;
    let rd = ((op >> 12) & 0xF) as usize;
    let rm = (op & 0xF) as usize;

    // SWP with R15 operands reads PC+12 (both the base and the stored source).
    let addr = if rn == 15 { cpu.r[15].wrapping_add(4) } else { cpu.r[rn] };
    let src = if rm == 15 { cpu.r[15].wrapping_add(4) } else { cpu.r[rm] };
    let value = if byte {
        let temp = bus.read8(addr, Access::NonSeq) as u32;
        bus.write8(addr, src as u8, Access::NonSeq);
        temp
    } else {
        let temp = bus.read32(addr, Access::NonSeq).rotate_right((addr & 3) * 8);
        bus.write32(addr, src, Access::NonSeq);
        temp
    };
    cpu.r[rd] = value;
    rd == 15
}

/// MRS / MSR: move a program status register to/from a general register (or an
/// immediate, for MSR). The field-mask bits (19..16) select which byte lanes of
/// the PSR an MSR writes; in User mode only the flags byte is writable.
fn psr_transfer(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let use_spsr = (op >> 22) & 1 == 1;

    if (op >> 21) & 1 == 0 {
        // MRS: Rd = PSR. Reading SPSR in a mode that has none (User/System)
        // returns the CPSR. MRS with Rd == R15 is UNPREDICTABLE (ARM DDI 0100):
        // we write the value without a pipeline flush and do not attempt to
        // reproduce the vectors' specific (undefined) result.
        let rd = ((op >> 12) & 0xF) as usize;
        cpu.r[rd] = if use_spsr && cpu.has_spsr() {
            cpu.current_spsr()
        } else {
            cpu.cpsr
        };
        return false;
    }

    // MSR: source is an immediate (bit25=1) or a register.
    let value = if (op >> 25) & 1 == 1 {
        barrel::rotate_immediate(op, cpu.carry()).0
    } else {
        cpu.r[(op & 0xF) as usize]
    };

    let field = (op >> 16) & 0xF;
    let mut mask = 0u32;
    if field & 1 != 0 {
        mask |= 0x0000_00FF;
    }
    if field & 2 != 0 {
        mask |= 0x0000_FF00;
    }
    if field & 4 != 0 {
        mask |= 0x00FF_0000;
    }
    if field & 8 != 0 {
        mask |= 0xFF00_0000;
    }
    // User mode may only alter the flags byte (the control bits are protected).
    if cpu.cpsr & 0x1F == 0x10 {
        mask &= 0xFF00_0000;
    }

    if use_spsr {
        if cpu.has_spsr() {
            let new = (cpu.current_spsr() & !mask) | (value & mask);
            cpu.set_current_spsr(new);
        }
    } else {
        let new = (cpu.cpsr & !mask) | (value & mask);
        cpu.write_cpsr(new); // re-banks if the mode field changed
    }
    false
}

/// LDRH / STRH / LDRSB / LDRSH: halfword and signed-byte transfers. Same
/// pre/post/up/writeback addressing as the word loads, with either an immediate
/// (split nibble) or register offset, distinguished by the SH field (bits 6..5).
fn halfword_signed_transfer<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let pre = (op >> 24) & 1 == 1;
    let up = (op >> 23) & 1 == 1;
    let imm_offset = (op >> 22) & 1 == 1;
    let writeback = (op >> 21) & 1 == 1;
    let load = (op >> 20) & 1 == 1;
    let rn = ((op >> 16) & 0xF) as usize;
    let rd = ((op >> 12) & 0xF) as usize;
    let sh = (op >> 5) & 3;

    let offset = if imm_offset {
        ((op >> 8) & 0xF) << 4 | (op & 0xF)
    } else {
        cpu.r[(op & 0xF) as usize]
    };
    let base = cpu.r[rn];
    let offset_addr = if up {
        base.wrapping_add(offset)
    } else {
        base.wrapping_sub(offset)
    };
    let addr = if pre { offset_addr } else { base };

    // The base writeback shares the R15-writeback-base quirk with word transfers.
    let writeback_now = !pre || writeback;
    let mut wrote_pc = false;
    let do_writeback = |cpu: &mut Arm7tdmi, wrote_pc: &mut bool| {
        if writeback_now {
            cpu.r[rn] = if rn == 15 {
                offset_addr.wrapping_add(4)
            } else {
                offset_addr
            };
            *wrote_pc |= rn == 15;
        }
    };

    if load {
        let value = match sh {
            1 => {
                // LDRH: zero-extend; an odd address rotates the halfword right 8.
                let raw = bus.read16(addr, Access::NonSeq) as u32;
                raw.rotate_right((addr & 1) * 8)
            }
            2 => bus.read8(addr, Access::NonSeq) as i8 as u32, // LDRSB: sign-extend
            _ => {
                // LDRSH: a 16-bit bus access. An odd address sign-extends the
                // high byte of the fetched halfword instead of the halfword.
                let raw = bus.read16(addr, Access::NonSeq);
                if addr & 1 != 0 {
                    (raw >> 8) as u8 as i8 as u32
                } else {
                    raw as i16 as u32
                }
            }
        };
        do_writeback(cpu, &mut wrote_pc);
        if rd == 15 {
            cpu.r[15] = value;
            wrote_pc = true;
        } else {
            cpu.r[rd] = value;
        }
    } else {
        // Only STRH is a store (SH == 1). Storing R15 stores PC+12.
        let value = if rd == 15 {
            cpu.r[15].wrapping_add(4)
        } else {
            cpu.r[rd]
        };
        bus.write16(addr, value as u16, Access::NonSeq);
        do_writeback(cpu, &mut wrote_pc);
    }

    wrote_pc
}

/// LDM / STM: block transfer of a register list to/from memory.
///
/// Registers are always visited low-to-high at ascending addresses; the P/U
/// bits only choose the lowest address and the writeback value. The S bit
/// selects the User-bank ("^") forms, or — on an LDM that loads R15 — a CPSR
/// restore from SPSR.
fn block_data_transfer<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let pre = (op >> 24) & 1 == 1;
    let up = (op >> 23) & 1 == 1;
    let s = (op >> 22) & 1 == 1;
    let writeback = (op >> 21) & 1 == 1;
    let load = (op >> 20) & 1 == 1;
    let rn = ((op >> 16) & 0xF) as usize;
    let list = op & 0xFFFF;
    let n = list.count_ones();

    let base = cpu.r[rn];

    // Empty list quirk: the ARM7TDMI transfers R15 alone but adjusts the base as
    // if all 16 registers were present (a 0x40 span). R15 sits at the lowest
    // address of that block (the same `start` the normal path would compute).
    if list == 0 {
        let (start, wb_value) = if up {
            (if pre { base.wrapping_add(4) } else { base }, base.wrapping_add(0x40))
        } else {
            let low = base.wrapping_sub(0x40);
            (if pre { low } else { low.wrapping_add(4) }, low)
        };
        let addr = start;
        if load {
            cpu.r[15] = bus.read32(addr, Access::NonSeq);
            if writeback {
                cpu.r[rn] = wb_value;
            }
            return true;
        }
        bus.write32(addr, cpu.r[15].wrapping_add(4), Access::NonSeq);
        if writeback {
            cpu.r[rn] = wb_value;
        }
        return rn == 15;
    }

    // Lowest transfer address, and the value the base is updated to.
    let (start, wb_value) = if up {
        (
            if pre { base.wrapping_add(4) } else { base },
            base.wrapping_add(4 * n),
        )
    } else {
        let low = base.wrapping_sub(4 * n);
        (if pre { low } else { low.wrapping_add(4) }, low)
    };

    let r15_in_list = list & 0x8000 != 0;
    // The "^" user-bank transfer applies unless this is an LDM that loads R15
    // (that case is instead a CPSR restore with ordinary banked registers).
    let user_bank = s && !(load && r15_in_list);

    let mut addr = start;
    let mut wrote_pc = false;

    if load {
        for i in 0..16 {
            if list & (1 << i) == 0 {
                continue;
            }
            let value = bus.read32(addr, Access::NonSeq);
            addr = addr.wrapping_add(4);
            if user_bank {
                cpu.set_user_reg(i, value);
            } else if i == 15 {
                cpu.r[15] = value;
                wrote_pc = true;
            } else {
                cpu.r[i] = value;
            }
        }
        // Writeback is suppressed when the base was itself loaded (the loaded
        // value wins on ARM7TDMI).
        if writeback && list & (1 << rn) == 0 {
            if user_bank && rn != 15 {
                cpu.set_user_reg(rn, wb_value);
            } else {
                cpu.r[rn] = wb_value; // R15 is never banked: a writeback to it branches
                wrote_pc |= rn == 15;
            }
        }
        // LDM with S and R15 in the list restores CPSR from SPSR after the load
        // (so the refill sees the restored mode/T bit). In a mode with no SPSR
        // (User/System) there is nothing to restore.
        if s && r15_in_list && cpu.has_spsr() {
            let spsr = cpu.current_spsr();
            cpu.write_cpsr(spsr);
        }
    } else {
        let mut first = true;
        for i in 0..16 {
            if list & (1 << i) == 0 {
                continue;
            }
            let value = if i == rn && writeback && !first {
                // Base written back mid-list (and not the first entry) stores the
                // *new* value. This wins over both the user-bank form and the
                // R15-stores-PC+12 rule.
                wb_value
            } else if i == 15 {
                cpu.r[15].wrapping_add(4) // stored R15 is PC+12 (never banked)
            } else if user_bank {
                cpu.user_reg(i)
            } else {
                cpu.r[i]
            };
            bus.write32(addr, value, Access::NonSeq);
            addr = addr.wrapping_add(4);
            first = false;
        }
        if writeback {
            if user_bank && rn != 15 {
                cpu.set_user_reg(rn, wb_value);
            } else {
                cpu.r[rn] = wb_value; // R15 is never banked: a writeback to it branches
                wrote_pc |= rn == 15;
            }
        }
    }

    wrote_pc
}

/// Compute the shifter operand and its carry-out. `read_reg` applies the
/// PC-as-operand adjustment for register-specified shifts.
fn operand2(cpu: &Arm7tdmi, op: u32, read_reg: impl Fn(&Arm7tdmi, usize) -> u32) -> (u32, bool) {
    if (op >> 25) & 1 == 1 {
        return barrel::rotate_immediate(op, cpu.carry());
    }
    let rm = (op & 0xF) as usize;
    let shift_type = (op >> 5) & 3;
    let cin = cpu.carry();
    let v = read_reg(cpu, rm);
    let (amount, imm) = if (op >> 4) & 1 == 0 {
        ((op >> 7) & 0x1F, true) // immediate shift amount
    } else {
        let rs = ((op >> 8) & 0xF) as usize;
        (cpu.r[rs] & 0xFF, false) // register shift amount
    };
    match shift_type {
        0 => barrel::lsl(v, amount, cin),
        1 => barrel::lsr(v, amount, cin, imm),
        2 => barrel::asr(v, amount, cin, imm),
        _ => barrel::ror(v, amount, cin, imm),
    }
}

/// `S`-with-Rd=15: restore CPSR from the current mode's SPSR, or — in a mode
/// with no SPSR (User/System) — fall back to a plain flag update.
fn restore_or_set_flags(cpu: &mut Arm7tdmi, result: u32, carry: bool, overflow: bool, logical: bool) {
    if cpu.has_spsr() {
        // Restore CPSR from SPSR, banking r8..r14 if the mode changes.
        let spsr = cpu.current_spsr();
        cpu.write_cpsr(spsr);
    } else {
        set_flags(cpu, result, carry, overflow, logical);
    }
}

fn set_flags(cpu: &mut Arm7tdmi, result: u32, carry: bool, overflow: bool, logical: bool) {
    let mut f = Flags::from_cpsr(cpu.cpsr);
    f.n = result >> 31 != 0;
    f.z = result == 0;
    f.c = carry; // for logical ops, `carry` is already the shifter carry-out
    if !logical {
        f.v = overflow; // logical ops leave V untouched
    }
    cpu.cpsr = f.to_cpsr(cpu.cpsr);
}