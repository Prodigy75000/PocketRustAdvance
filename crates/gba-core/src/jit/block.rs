// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Block discovery: how far a straight run of instructions reaches.
//!
//! A block is a maximal run starting at one address with no control flow inside
//! it. The run ends AT the first instruction that can write R15, which stays
//! part of the block so whoever executes it performs the control transfer.
//!
//! Only Thumb is handled. That is not a shortcut, it is where the work is: the
//! LeafGreen battle scene runs 76% Thumb, and the parked overworld state runs
//! 100% Thumb. ARM blocks come later and the interpreter covers them meanwhile.
//!
//! ## Why block boundaries can be exact, which was not obvious
//!
//! A recompiler normally checks for interrupts and for the end of its time
//! slice once per block rather than once per instruction, and normally that is
//! an accuracy trade. Here it does not have to be, and the reason is worth
//! writing down because the whole design leans on it.
//!
//! **Interrupts.** The condition is `irq_pending() && irq_ready()`, built from
//! `ie`, `if_` and the CPSR I bit. Inside this core, `if_` is raised by the
//! frame loop at scanline boundaries, never mid-block, so during a block it can
//! only change if the block itself stores to I/O. If the condition is false
//! when the block starts and nothing in the block can change it, it is false at
//! every instruction boundary inside the block. Checking once at the top is
//! then exactly what checking every instruction would have decided.
//!
//! **The time slice.** The interpreter runs an instruction whenever
//! `cycles < target`. If a block's WORST-CASE cost fits in the remaining
//! budget, then every instruction boundary inside it had `cycles < target`, so
//! the interpreter would have run all of them too. Dispatch therefore only runs
//! a block when `target - cycles` exceeds the block's maximum cost, and
//! interprets the short tail of each scanline phase. A phase is around 1000
//! cycles and a block costs tens, so this gives up very little.
//!
//! **Stores are the hole in both arguments**, because a store's address is a
//! register and the region is not known until it runs. An I/O store can raise
//! an interrupt and can start a DMA whose cost is unbounded. The answer is a
//! flag the bus sets when a write lands somewhere that matters, checked by
//! compiled code after each store only: stores are about a tenth of
//! instructions, and the check happens before the next instruction runs, so
//! there is no overshoot to undo.
//!
//! None of that machinery exists yet. This module is the extent calculation it
//! will be built on.

/// Why a block stopped.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum End {
    /// The last instruction can write R15. It is part of the block.
    Branch,
    /// Stopped at the instruction cap without reaching a branch. The next block
    /// starts at the following instruction.
    Cap,
    /// The run left the region it started in, so the next instruction cannot be
    /// read from the same slice.
    Edge,
}

/// A discovered run of Thumb instructions.
#[derive(Clone, Debug)]
pub struct Block {
    /// Address of the first instruction.
    pub start: u32,
    /// The opcodes, in order.
    pub ops: Vec<u16>,
    /// Why the run stopped.
    pub end: End,
}

impl Block {
    /// Bytes spanned. Thumb is fixed width, so this is just the count.
    pub fn byte_len(&self) -> u32 {
        self.ops.len() as u32 * 2
    }

    /// Address one past the last instruction.
    pub fn end_addr(&self) -> u32 {
        self.start.wrapping_add(self.byte_len())
    }
}

/// Longest run we will put in one block.
///
/// Not a tuning knob yet. It bounds the worst-case compile time and the
/// worst-case time-slice overshoot that dispatch has to reserve for, and 64
/// instructions is far above the measured average basic block.
pub const MAX_OPS: usize = 64;

/// Can this Thumb instruction write R15, or otherwise divert control?
///
/// Deliberately conservative: when in doubt this returns true, which costs a
/// shorter block and never costs correctness. Two cases are knowingly
/// over-broad and marked, because narrowing them needs the register decode and
/// that is worth doing only once the emitter can measure the difference.
pub fn diverts(op: u16) -> bool {
    match op >> 12 {
        // Format 5, hi-register ops and BX, at op[15:10] == 010001. BX always
        // diverts; ADD/MOV divert only when Rd is 15 and CMP never writes a
        // register at all. OVER-BROAD: all four are treated as diverting.
        0b0100 if (op >> 10) & 0x3 == 0b01 => true,
        // Format 14, PUSH/POP. Only POP (bit 11) with the R bit (bit 8) loads
        // PC. PUSH with R stores LR and does not divert.
        0b1011 if (op >> 9) & 0x3 == 0b10 => op & 0x0900 == 0x0900,
        // Format 15 is LDMIA/STMIA on low registers only, so it cannot divert.
        // Formats 16 (conditional branch), 17 (SWI) and the undefined 1101 1110
        // encoding all leave the run.
        0b1101 => true,
        // Format 18 (B) and format 19 (BL prefix and suffix). The BL prefix
        // only sets LR, but splitting a BL pair across two blocks would put a
        // half-formed long branch at a boundary. OVER-BROAD on purpose.
        0b1110 | 0b1111 => true,
        _ => false,
    }
}

/// Walk forward from `start`, reading opcodes through `read`, and return the
/// block that begins there.
///
/// `read` returns `None` when the address is outside the region the block
/// started in, which ends the run with [`End::Edge`]. Taking a closure keeps
/// this testable without a bus, and keeps the decision about what counts as
/// "the same region" with the caller that owns the memory map.
pub fn discover<F>(start: u32, mut read: F) -> Block
where
    F: FnMut(u32) -> Option<u16>,
{
    let mut ops = Vec::new();
    let mut at = start;
    loop {
        let Some(op) = read(at) else {
            return Block { start, ops, end: End::Edge };
        };
        ops.push(op);
        if diverts(op) {
            return Block { start, ops, end: End::Branch };
        }
        if ops.len() >= MAX_OPS {
            return Block { start, ops, end: End::Cap };
        }
        at = at.wrapping_add(2);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Build a reader over a halfword array based at `base`.
    fn reader(base: u32, words: &[u16]) -> impl FnMut(u32) -> Option<u16> + '_ {
        move |addr| {
            let off = addr.checked_sub(base)? / 2;
            words.get(off as usize).copied()
        }
    }

    const MOV_R0_1: u16 = 0x2001; // MOV R0, #1   (format 3)
    const ADD_R0_R1: u16 = 0x1808; // ADD R0, R1, R0 (format 2)
    const STR_R0_R1: u16 = 0x6008; // STR R0, [R1]  (format 9)
    const B_BACK: u16 = 0xE7FE; // B .          (format 18)
    const BX_LR: u16 = 0x4770; // BX LR        (format 5)
    const POP_PC: u16 = 0xBD00; // POP {PC}     (format 14, R set)
    const POP_R0: u16 = 0xBC01; // POP {R0}     (format 14, R clear)
    const PUSH_LR: u16 = 0xB500; // PUSH {LR}    (format 14, store)
    const BEQ: u16 = 0xD0FE; // BEQ .        (format 16)
    const SWI: u16 = 0xDF06; // SWI 6        (format 17)
    const BL_HI: u16 = 0xF000; // BL prefix    (format 19)

    #[test]
    fn a_run_ends_at_the_branch_and_includes_it() {
        let code = [MOV_R0_1, ADD_R0_R1, STR_R0_R1, B_BACK, MOV_R0_1];
        let b = discover(0x0800_0000, reader(0x0800_0000, &code));
        assert_eq!(b.end, End::Branch);
        assert_eq!(b.ops, &code[..4], "the branch is the last instruction in the block");
        assert_eq!(b.byte_len(), 8);
        assert_eq!(b.end_addr(), 0x0800_0008, "the next block starts after the branch");
    }

    #[test]
    fn running_off_the_region_ends_the_block_without_a_branch() {
        let code = [MOV_R0_1, ADD_R0_R1];
        let b = discover(0x0300_0000, reader(0x0300_0000, &code));
        assert_eq!(b.end, End::Edge);
        assert_eq!(b.ops.len(), 2, "both readable instructions are kept");
    }

    #[test]
    fn a_long_straight_run_stops_at_the_cap() {
        let code = vec![MOV_R0_1; MAX_OPS * 2];
        let b = discover(0x0800_0000, reader(0x0800_0000, &code));
        assert_eq!(b.end, End::Cap);
        assert_eq!(b.ops.len(), MAX_OPS);
        assert_eq!(b.end_addr(), 0x0800_0000 + MAX_OPS as u32 * 2);
    }

    /// The whole point of `diverts` is which encodings leave the run, so pin
    /// them individually. PUSH and a POP without PC must NOT end a block: they
    /// are the common function prologue and epilogue, and treating them as
    /// branches would cut nearly every function into fragments.
    #[test]
    fn only_the_encodings_that_can_write_r15_divert() {
        for op in [B_BACK, BX_LR, POP_PC, BEQ, SWI, BL_HI] {
            assert!(diverts(op), "{op:04X} can write R15");
        }
        for op in [MOV_R0_1, ADD_R0_R1, STR_R0_R1, POP_R0, PUSH_LR] {
            assert!(!diverts(op), "{op:04X} cannot write R15");
        }
    }

    /// A one-instruction block is legal and common: the target of a branch is
    /// very often another branch.
    #[test]
    fn a_block_can_be_a_single_branch() {
        let b = discover(0x0800_0000, reader(0x0800_0000, &[BX_LR, MOV_R0_1]));
        assert_eq!(b.ops, &[BX_LR]);
        assert_eq!(b.end, End::Branch);
    }

    /// Nothing readable at all. The block has to come back empty rather than
    /// loop or panic, because a runaway PC reaches here.
    #[test]
    fn an_unreadable_start_yields_an_empty_block() {
        let b = discover(0x1000_0000, |_| None);
        assert!(b.ops.is_empty());
        assert_eq!(b.end, End::Edge);
        assert_eq!(b.byte_len(), 0);
        assert_eq!(b.end_addr(), 0x1000_0000, "an empty block cannot advance the PC");
    }
}
