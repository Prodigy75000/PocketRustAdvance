//! 16-bit Thumb instruction execution.
//!
//! Thumb is a second, denser encoding whose 19 formats decode down to the same
//! execution primitives as ARM (the barrel shifter, the ALU, the memory model,
//! the pipeline). Most Thumb data operations execute unconditionally and always
//! update the flags. R15 reads as PC+4 here (two halfwords ahead); PC-relative
//! forms word-align it.
//!
//! Each handler returns `true` if it wrote R15 (a branch), so the caller flushes
//! and refills the pipeline.

use super::barrel;
use super::psr::Flags;
use super::Arm7tdmi;
use crate::bus::{Access, Bus};

/// Decode and execute one Thumb instruction.
pub fn execute<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u16) -> bool {
    let op = op as u32;
    match op >> 13 {
        0b000 => {
            if (op >> 11) & 3 == 3 {
                add_subtract(cpu, op) // format 2
            } else {
                move_shifted(cpu, op) // format 1
            }
        }
        0b001 => immediate(cpu, op), // format 3
        0b010 => {
            if (op >> 10) & 0x3F == 0b010000 {
                alu(cpu, op) // format 4
            } else if (op >> 10) & 0x3F == 0b010001 {
                hi_register(cpu, op) // format 5
            } else if (op >> 11) & 0x1F == 0b01001 {
                pc_relative_load(cpu, bus, op) // format 6
            } else if (op >> 9) & 1 == 0 {
                load_store_reg(cpu, bus, op) // format 7
            } else {
                load_store_sign_extended(cpu, bus, op) // format 8
            }
        }
        0b011 => load_store_imm(cpu, bus, op), // format 9
        0b100 => {
            if (op >> 12) & 1 == 0 {
                load_store_halfword(cpu, bus, op) // format 10
            } else {
                sp_relative(cpu, bus, op) // format 11
            }
        }
        0b101 => {
            if (op >> 12) & 1 == 0 {
                load_address(cpu, op) // format 12
            } else if (op >> 8) & 0xF == 0b0000 {
                add_to_sp(cpu, op) // format 13
            } else {
                push_pop(cpu, bus, op) // format 14
            }
        }
        0b110 => {
            if (op >> 12) & 1 == 0 {
                block_transfer(cpu, bus, op) // format 15
            } else if (op >> 8) & 0xF == 0b1111 {
                software_interrupt(cpu, bus, op) // format 17
            } else {
                conditional_branch(cpu, op) // format 16
            }
        }
        // 0b111
        _ => {
            if (op >> 11) & 0x1F == 0b11100 {
                unconditional_branch(cpu, op) // format 18
            } else {
                long_branch_link(cpu, op) // format 19
            }
        }
    }
}

// --- ALU helpers --------------------------------------------------------------

/// `a + b + carry_in`, returning `(result, carry_out, signed_overflow)`.
fn addc(a: u32, b: u32, carry_in: bool) -> (u32, bool, bool) {
    let sum = a as u64 + b as u64 + carry_in as u64;
    let r = sum as u32;
    (r, sum > 0xFFFF_FFFF, (!(a ^ b) & (a ^ r)) >> 31 != 0)
}

fn set_nz(cpu: &mut Arm7tdmi, r: u32) {
    let mut f = Flags::from_cpsr(cpu.cpsr);
    f.n = r >> 31 != 0;
    f.z = r == 0;
    cpu.cpsr = f.to_cpsr(cpu.cpsr);
}

fn set_nz_c(cpu: &mut Arm7tdmi, r: u32, c: bool) {
    let mut f = Flags::from_cpsr(cpu.cpsr);
    f.n = r >> 31 != 0;
    f.z = r == 0;
    f.c = c;
    cpu.cpsr = f.to_cpsr(cpu.cpsr);
}

fn set_nzcv(cpu: &mut Arm7tdmi, r: u32, c: bool, v: bool) {
    let mut f = Flags::from_cpsr(cpu.cpsr);
    f.n = r >> 31 != 0;
    f.z = r == 0;
    f.c = c;
    f.v = v;
    cpu.cpsr = f.to_cpsr(cpu.cpsr);
}

// --- Formats ------------------------------------------------------------------

/// Format 1: move shifted register (LSL/LSR/ASR by immediate).
fn move_shifted(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let amount = (op >> 6) & 0x1F;
    let rs = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let v = cpu.r[rs];
    let c = cpu.carry();
    let (res, carry) = match (op >> 11) & 3 {
        0 => barrel::lsl(v, amount, c),
        1 => barrel::lsr(v, amount, c, true),
        _ => barrel::asr(v, amount, c, true),
    };
    cpu.r[rd] = res;
    set_nz_c(cpu, res, carry);
    false
}

/// Format 2: add/subtract a register or a 3-bit immediate.
fn add_subtract(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let imm = (op >> 10) & 1 == 1;
    let sub = (op >> 9) & 1 == 1;
    let operand = (op >> 6) & 7;
    let rs = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let a = cpu.r[rs];
    let b = if imm { operand } else { cpu.r[operand as usize] };
    let (res, c, v) = if sub {
        addc(a, !b, true)
    } else {
        addc(a, b, false)
    };
    cpu.r[rd] = res;
    set_nzcv(cpu, res, c, v);
    false
}

/// Format 3: move/compare/add/subtract with an 8-bit immediate.
fn immediate(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let rd = ((op >> 8) & 7) as usize;
    let imm = op & 0xFF;
    let a = cpu.r[rd];
    match (op >> 11) & 3 {
        0 => {
            cpu.r[rd] = imm; // MOV
            set_nz(cpu, imm);
        }
        1 => {
            let (res, c, v) = addc(a, !imm, true); // CMP
            set_nzcv(cpu, res, c, v);
        }
        2 => {
            let (res, c, v) = addc(a, imm, false); // ADD
            cpu.r[rd] = res;
            set_nzcv(cpu, res, c, v);
        }
        _ => {
            let (res, c, v) = addc(a, !imm, true); // SUB
            cpu.r[rd] = res;
            set_nzcv(cpu, res, c, v);
        }
    }
    false
}

/// Format 4: ALU operations (Rd = Rd op Rs).
fn alu(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let rs = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let a = cpu.r[rd];
    let b = cpu.r[rs];
    let c = cpu.carry();
    match (op >> 6) & 0xF {
        0x0 => {
            let r = a & b; // AND
            cpu.r[rd] = r;
            set_nz(cpu, r);
        }
        0x1 => {
            let r = a ^ b; // EOR
            cpu.r[rd] = r;
            set_nz(cpu, r);
        }
        0x2 => {
            let (r, carry) = barrel::lsl(a, b & 0xFF, c); // LSL
            cpu.r[rd] = r;
            set_nz_c(cpu, r, carry);
        }
        0x3 => {
            let (r, carry) = barrel::lsr(a, b & 0xFF, c, false); // LSR
            cpu.r[rd] = r;
            set_nz_c(cpu, r, carry);
        }
        0x4 => {
            let (r, carry) = barrel::asr(a, b & 0xFF, c, false); // ASR
            cpu.r[rd] = r;
            set_nz_c(cpu, r, carry);
        }
        0x5 => {
            let (r, carry, v) = addc(a, b, c); // ADC
            cpu.r[rd] = r;
            set_nzcv(cpu, r, carry, v);
        }
        0x6 => {
            let (r, carry, v) = addc(a, !b, c); // SBC
            cpu.r[rd] = r;
            set_nzcv(cpu, r, carry, v);
        }
        0x7 => {
            let (r, carry) = barrel::ror(a, b & 0xFF, c, false); // ROR
            cpu.r[rd] = r;
            set_nz_c(cpu, r, carry);
        }
        0x8 => set_nz(cpu, a & b), // TST
        0x9 => {
            let (r, carry, v) = addc(0, !b, true); // NEG (0 - Rs)
            cpu.r[rd] = r;
            set_nzcv(cpu, r, carry, v);
        }
        0xA => {
            let (r, carry, v) = addc(a, !b, true); // CMP
            set_nzcv(cpu, r, carry, v);
        }
        0xB => {
            let (r, carry, v) = addc(a, b, false); // CMN
            set_nzcv(cpu, r, carry, v);
        }
        0xC => {
            let r = a | b; // ORR
            cpu.r[rd] = r;
            set_nz(cpu, r);
        }
        0xD => {
            let r = a.wrapping_mul(b); // MUL
            cpu.r[rd] = r;
            set_nz(cpu, r); // C is UNPREDICTABLE after MUL (ARM DDI 0100), left as-is
        }
        0xE => {
            let r = a & !b; // BIC
            cpu.r[rd] = r;
            set_nz(cpu, r);
        }
        _ => {
            let r = !b; // MVN
            cpu.r[rd] = r;
            set_nz(cpu, r);
        }
    }
    false
}

/// Format 5: hi-register operations and BX. ADD/MOV do not set flags; only CMP
/// does. ADD/MOV with Rd = PC (or BX) branch.
fn hi_register(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let rs = (((op >> 3) & 7) | ((op >> 3) & 0x8)) as usize; // Rs | (H2 << 3)
    let rd = ((op & 7) | ((op >> 4) & 0x8)) as usize; // Rd | (H1 << 3)
    let a = cpu.r[rd];
    let b = cpu.r[rs];
    match (op >> 8) & 3 {
        0 => {
            let r = a.wrapping_add(b); // ADD (no flags)
            cpu.r[rd] = if rd == 15 { r & !1 } else { r }; // PC stays halfword-aligned
            rd == 15
        }
        1 => {
            let (r, c, v) = addc(a, !b, true); // CMP (flags only)
            set_nzcv(cpu, r, c, v);
            false
        }
        2 => {
            cpu.r[rd] = if rd == 15 { b & !1 } else { b }; // MOV (no flags)
            rd == 15
        }
        _ => {
            // BX: branch to Rs, switching state on bit 0.
            if b & 1 != 0 {
                cpu.cpsr |= 1 << 5;
            } else {
                cpu.cpsr &= !(1 << 5);
            }
            cpu.r[15] = b & !1;
            true
        }
    }
}

/// Format 6: PC-relative load (Rd = [(PC & ~3) + word8*4]).
fn pc_relative_load<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let rd = ((op >> 8) & 7) as usize;
    let addr = (cpu.r[15] & !3).wrapping_add((op & 0xFF) << 2);
    cpu.r[rd] = bus.read32(addr, Access::NonSeq);
    false
}

/// Format 7: load/store word or byte with a register offset.
fn load_store_reg<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let load = (op >> 11) & 1 == 1;
    let byte = (op >> 10) & 1 == 1;
    let ro = ((op >> 6) & 7) as usize;
    let rb = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let addr = cpu.r[rb].wrapping_add(cpu.r[ro]);
    transfer(cpu, bus, addr, rd, load, byte);
    false
}

/// Format 8: load/store sign-extended byte or halfword with a register offset.
fn load_store_sign_extended<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let sh = (op >> 10) & 3; // {S,H}
    let ro = ((op >> 6) & 7) as usize;
    let rb = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let addr = cpu.r[rb].wrapping_add(cpu.r[ro]);
    // The field is {H(bit11), S(bit10)} read as H*2+S: 0=STRH, 1=LDRSB, 2=LDRH,
    // 3=LDRSH.
    cpu.r[rd] = match sh {
        0 => {
            bus.write16(addr, cpu.r[rd] as u16, Access::NonSeq); // STRH
            return false;
        }
        1 => bus.read8(addr, Access::NonSeq) as i8 as u32, // LDRSB
        2 => {
            let raw = bus.read16(addr, Access::NonSeq) as u32; // LDRH
            raw.rotate_right((addr & 1) * 8)
        }
        _ => {
            let raw = bus.read16(addr, Access::NonSeq); // LDRSH
            if addr & 1 != 0 {
                (raw >> 8) as u8 as i8 as u32
            } else {
                raw as i16 as u32
            }
        }
    };
    false
}

/// Format 9: load/store word or byte with a 5-bit immediate offset.
fn load_store_imm<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let byte = (op >> 12) & 1 == 1;
    let load = (op >> 11) & 1 == 1;
    let offset = (op >> 6) & 0x1F;
    let rb = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let addr = cpu.r[rb].wrapping_add(if byte { offset } else { offset << 2 });
    transfer(cpu, bus, addr, rd, load, byte);
    false
}

/// Format 10: load/store halfword with a 5-bit immediate offset.
fn load_store_halfword<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let load = (op >> 11) & 1 == 1;
    let offset = ((op >> 6) & 0x1F) << 1;
    let rb = ((op >> 3) & 7) as usize;
    let rd = (op & 7) as usize;
    let addr = cpu.r[rb].wrapping_add(offset);
    if load {
        let raw = bus.read16(addr, Access::NonSeq) as u32;
        cpu.r[rd] = raw.rotate_right((addr & 1) * 8);
    } else {
        bus.write16(addr, cpu.r[rd] as u16, Access::NonSeq);
    }
    false
}

/// Format 11: SP-relative load/store word.
fn sp_relative<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let load = (op >> 11) & 1 == 1;
    let rd = ((op >> 8) & 7) as usize;
    let addr = cpu.r[13].wrapping_add((op & 0xFF) << 2);
    transfer(cpu, bus, addr, rd, load, false);
    false
}

/// Format 12: load address (Rd = (PC&~3 or SP) + word8*4).
fn load_address(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let sp = (op >> 11) & 1 == 1;
    let rd = ((op >> 8) & 7) as usize;
    let base = if sp { cpu.r[13] } else { cpu.r[15] & !3 };
    cpu.r[rd] = base.wrapping_add((op & 0xFF) << 2);
    false
}

/// Format 13: add a signed 7-bit word offset to SP.
fn add_to_sp(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let offset = (op & 0x7F) << 2;
    if (op >> 7) & 1 == 1 {
        cpu.r[13] = cpu.r[13].wrapping_sub(offset);
    } else {
        cpu.r[13] = cpu.r[13].wrapping_add(offset);
    }
    false
}

/// Format 14: PUSH/POP. PUSH stores the low registers (and optionally LR) below
/// SP; POP loads them (and optionally PC) from SP upward.
fn push_pop<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let load = (op >> 11) & 1 == 1;
    let extra = (op >> 8) & 1 == 1; // LR (push) / PC (pop)
    let list = op & 0xFF;
    let count = list.count_ones() + extra as u32;

    // Empty list (no registers and no LR/PC): the R15/0x40 quirk applies.
    if list == 0 && !extra {
        if load {
            cpu.r[15] = bus.read32(cpu.r[13], Access::NonSeq);
            cpu.r[13] = cpu.r[13].wrapping_add(0x40);
            return true;
        }
        let sp = cpu.r[13].wrapping_sub(0x40);
        bus.write32(sp, cpu.r[15].wrapping_add(2), Access::NonSeq);
        cpu.r[13] = sp;
        return false;
    }

    if load {
        // POP: read upward from SP, then SP += 4*count.
        let mut addr = cpu.r[13];
        for i in 0..8 {
            if list & (1 << i) != 0 {
                cpu.r[i as usize] = bus.read32(addr, Access::NonSeq);
                addr = addr.wrapping_add(4);
            }
        }
        cpu.r[13] = cpu.r[13].wrapping_add(4 * count);
        if extra {
            cpu.r[15] = bus.read32(addr, Access::NonSeq) & !1;
            return true; // POP {..,PC} branches
        }
        false
    } else {
        // PUSH: SP -= 4*count, then store upward from the new SP.
        let start = cpu.r[13].wrapping_sub(4 * count);
        let mut addr = start;
        for i in 0..8 {
            if list & (1 << i) != 0 {
                bus.write32(addr, cpu.r[i as usize], Access::NonSeq);
                addr = addr.wrapping_add(4);
            }
        }
        if extra {
            bus.write32(addr, cpu.r[14], Access::NonSeq);
        }
        cpu.r[13] = start;
        false
    }
}

/// Format 15: LDMIA/STMIA with writeback on Rb.
fn block_transfer<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    let load = (op >> 11) & 1 == 1;
    let rb = ((op >> 8) & 7) as usize;
    let list = op & 0xFF;

    // Empty list quirk: transfer R15 alone and adjust the base by 0x40.
    if list == 0 {
        let base = cpu.r[rb];
        cpu.r[rb] = base.wrapping_add(0x40);
        if load {
            cpu.r[15] = bus.read32(base, Access::NonSeq);
            return true;
        }
        bus.write32(base, cpu.r[15].wrapping_add(2), Access::NonSeq);
        return false;
    }

    let wb_value = cpu.r[rb].wrapping_add(4 * list.count_ones());

    let mut addr = cpu.r[rb];
    let mut first = true;
    for i in 0..8 {
        if list & (1 << i) != 0 {
            let i = i as usize;
            if load {
                cpu.r[i] = bus.read32(addr, Access::NonSeq);
            } else {
                // STMIA storing its own base register stores the old value only
                // if the base is the first entry, otherwise the written-back one.
                let v = if i == rb && !first { wb_value } else { cpu.r[i] };
                bus.write32(addr, v, Access::NonSeq);
            }
            addr = addr.wrapping_add(4);
            first = false;
        }
    }
    // Writeback, unless a load already reloaded the base.
    if !(load && list & (1 << rb) != 0) {
        cpu.r[rb] = wb_value;
    }
    false
}

/// Format 16: conditional branch (8-bit signed halfword offset).
fn conditional_branch(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let cond = super::psr::Cond::decode((op >> 8) & 0xF);
    if !Flags::from_cpsr(cpu.cpsr).eval(cond) {
        return false;
    }
    let offset = ((op & 0xFF) as u8 as i8 as i32 * 2) as u32;
    cpu.r[15] = cpu.r[15].wrapping_add(offset);
    true
}

/// Format 18: unconditional branch (11-bit signed halfword offset).
fn unconditional_branch(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let offset = (((op & 0x7FF) << 21) as i32 >> 20) as u32; // sign-extend, *2
    cpu.r[15] = cpu.r[15].wrapping_add(offset);
    true
}

/// Format 19: long branch with link, in two halves. The prefix seeds LR with the
/// high part of the offset; the suffix computes the target and sets the return
/// address (with its Thumb bit) in LR.
fn long_branch_link(cpu: &mut Arm7tdmi, op: u32) -> bool {
    let offset = op & 0x7FF;
    if (op >> 11) & 1 == 0 {
        // Prefix (H=0): LR = PC + (signed offset << 12).
        let hi = ((offset << 21) as i32 >> 9) as u32;
        cpu.r[14] = cpu.r[15].wrapping_add(hi);
        false
    } else {
        // Suffix (H=1): target = LR + (offset << 1); LR = return | 1.
        let target = cpu.r[14].wrapping_add(offset << 1) & !1;
        let return_addr = cpu.r[15].wrapping_sub(2) | 1;
        cpu.r[15] = target;
        cpu.r[14] = return_addr;
        true
    }
}

/// Format 17: Thumb SWI — identical exception entry to ARM SWI.
fn software_interrupt<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, op: u32) -> bool {
    // With an HLE BIOS there is nothing at the vector; emulate the call directly.
    if cpu.hle_bios {
        return super::hle::swi(cpu, bus, (op & 0xFF) as u8);
    }
    // Return address is the instruction after the SWI (R15 is PC+4 in Thumb).
    let return_addr = cpu.r[15].wrapping_sub(2);
    let old_cpsr = cpu.cpsr;
    // Enter Supervisor mode in ARM state, IRQs masked.
    let new_cpsr = (old_cpsr & !0x1F & !(1 << 5)) | 0x13 | (1 << 7);
    cpu.write_cpsr(new_cpsr);
    cpu.set_current_spsr(old_cpsr);
    cpu.r[14] = return_addr;
    cpu.r[15] = 0x0000_0008;
    true
}

/// Shared word/byte transfer used by the register/immediate/SP forms.
fn transfer<B: Bus>(cpu: &mut Arm7tdmi, bus: &mut B, addr: u32, rd: usize, load: bool, byte: bool) {
    if load {
        cpu.r[rd] = if byte {
            bus.read8(addr, Access::NonSeq) as u32
        } else {
            bus.read32(addr, Access::NonSeq).rotate_right((addr & 3) * 8)
        };
    } else if byte {
        bus.write8(addr, cpu.r[rd] as u8, Access::NonSeq);
    } else {
        bus.write32(addr, cpu.r[rd], Access::NonSeq);
    }
}
