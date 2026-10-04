// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Finding the loop a game spins in while it waits for an interrupt.
//!
//! Pokemon LeafGreen spends **47% of every frame's instructions** in five
//! instructions at 0x080008BE:
//!
//! ```text
//! LDRH R1, [R2, #28]
//! MOV  R0, R3
//! AND  R0, R1
//! CMP  R0, #0
//! BEQ  back
//! ```
//!
//! It is waiting for a flag that the V-blank handler sets. Emulating the spin
//! faithfully is emulating nothing, and skipping it measured **1.75x on the
//! whole frame** with a byte-identical framebuffer over 900 frames.
//!
//! gpSP does this from a hand-built table: `gba_over.h` carries an
//! `idle_loop_target_pc` for 200 games, LeafGreen's being 0x080008B2. A table
//! is not the approach here, for two reasons. It would cover 200 of our 2727
//! ROMs, and gpSP is GPL-2 while this core is GPL-3-or-later, so copying its
//! compiled table is a licence question better not created.
//!
//! ## What makes skipping safe
//!
//! Two independent checks, and the dynamic one is the load-bearing half.
//!
//! **Static, here:** every instruction in the body must be a load or a
//! register operation. No stores, no branches out, no SWI. That is what rules
//! out side effects, so losing an iteration loses nothing.
//!
//! **Dynamic, in the frame loop:** the register file must be *identical* on two
//! consecutive arrivals at the loop head. That is a direct proof the loop made
//! no progress, and it is worth far more than any dataflow analysis I could
//! write here. A counting loop like `ADD R0,#1 / CMP R0,#10 / BNE` is pure by
//! the static test and is caught instantly by this one.
//!
//! **And the skip itself is bounded.** It advances to the end of the current
//! scanline phase and re-tests, never further, so it cannot deadlock and cannot
//! skip past an event. A loop polling VCOUNT, rather than waiting on an
//! interrupt, still resolves correctly: just at scanline granularity.

/// Runtime state for idle-loop skipping.
///
/// Derived state, deliberately NOT serialized. An address carried into a loaded
/// save state could skip code that is no longer a wait loop, and rebuilding it
/// costs one scanline of sampling.
#[derive(Clone)]
pub struct Watch {
    /// R15 as it reads at the loop head being watched, or `u32::MAX` for
    /// nothing. The hot path is one compare against this field, which is why it
    /// holds R15 rather than the PC: no arithmetic per instruction.
    pub probe_r15: u32,
    /// Instructions in one iteration, so "the previous arrival was the previous
    /// iteration" can be checked rather than assumed.
    len: u64,
    /// Register file plus CPSR at the previous arrival, and the step count then.
    snap: [u32; 17],
    last_steps: u64,
    have_snap: bool,
    /// How many times the proof held and the phase was skipped, how many
    /// emulated cycles that skipped, and how many arrivals did NOT prove idle.
    ///
    /// Both counters matter. Lots of misses and no skips means the loop keeps
    /// changing something and this address is the wrong target, which is a
    /// different fault from never finding a loop at all, and from outside the
    /// two look identical.
    pub skips: u64,
    pub skipped_cycles: u64,
    pub misses: u64,
    /// `skips` as of the last productivity check, so a target that has stopped
    /// paying can be dropped and a new one found.
    skips_at_check: u64,
    /// How many targets have been dropped as unproductive. A large number means
    /// the sampler is thrashing between candidates.
    pub retargets: u64,
}

impl Default for Watch {
    fn default() -> Self {
        Watch {
            probe_r15: u32::MAX,
            len: 0,
            snap: [0; 17],
            last_steps: 0,
            have_snap: false,
            skips: 0,
            skipped_cycles: 0,
            misses: 0,
            skips_at_check: 0,
            retargets: 0,
        }
    }
}

impl Watch {
    /// Watch a loop head, given R15 as it reads there and the instruction count
    /// of one iteration. Re-watching the same head keeps progress.
    pub fn watch(&mut self, head_r15: u32, len: u64) {
        if self.probe_r15 != head_r15 {
            self.probe_r15 = head_r15;
            self.have_snap = false;
        }
        self.len = len;
    }

    /// Is anything being watched?
    pub fn watching(&self) -> bool {
        self.probe_r15 != u32::MAX
    }

    /// Forget the target. Called on reset and after loading a save state, which
    /// may be anywhere. Counters survive, because they describe the whole run.
    pub fn forget(&mut self) {
        let keep = (self.skips, self.skipped_cycles, self.misses, self.retargets);
        *self = Watch::default();
        (self.skips, self.skipped_cycles, self.misses, self.retargets) = keep;
    }

    /// An arrival at the watched loop head. Returns true when the loop has
    /// provably made no progress and the rest of the phase can be skipped.
    ///
    /// The proof is ROLLING, re-established on every decision rather than
    /// recorded once. Two things have to hold:
    ///
    /// 1. The previous arrival was the PREVIOUS ITERATION. One iteration is
    ///    exactly `len` instructions, Thumb having no conditional execution
    ///    outside its branches, so the step delta is an exact test for "nothing
    ///    else ran in between". Getting this wrong broke 53 of 80 ROMs: a loop
    ///    like `LDRB R2,[R0] / ADD R0,#1 / CMP R2,#0 / BNE` progresses every
    ///    iteration, but a function CALLED twice with the same arguments arrives
    ///    twice with an identical register file, and comparing arrivals armed it.
    /// 2. The register file and CPSR are unchanged across that iteration. Since
    ///    the body passed the static purity check it has no other effect, so an
    ///    iteration that changed nothing will keep changing nothing until
    ///    something outside the CPU moves.
    ///
    /// Rolling rather than frozen matters, and that was also measured: a first
    /// cut recorded the proven state once and compared against it, and on a
    /// state where the polled word advances once a frame it armed and then
    /// refused every subsequent skip, 14,050 times a frame, ending up slower
    /// than no detection at all.
    pub fn consider(&mut self, steps: u64, r: &[u32; 16], cpsr: u32) -> bool {
        let mut now = [0u32; 17];
        now[..16].copy_from_slice(r);
        now[16] = cpsr;
        // The previous arrival must have been the previous ITERATION. Exactly
        // one iteration, nothing more and nothing LESS.
        //
        // The "nothing less" half is not pedantry, it is the difference between
        // working and freezing. A zero delta means the phase ended at the loop
        // head and the body has not run since, and accepting that as proof looks
        // sound: nothing executed, so nothing changed. It froze 41 of 80 ROMs
        // solid, at zero instructions per frame. The loop has to RUN to observe
        // the thing it is waiting for. Skip the body and it can never see the
        // value change, so the state stays identical forever and the skip
        // justifies itself in a closed circle.
        //
        // So one iteration per scanline phase is the price of being able to
        // notice the world, and it is cheap: five instructions against the
        // hundreds the spin would have cost.
        let delta = steps.wrapping_sub(self.last_steps);
        let one_iteration = self.have_snap && delta == self.len;
        let idle = one_iteration && self.snap == now;
        self.snap = now;
        self.last_steps = steps;
        self.have_snap = true;
        if !idle {
            self.misses += 1;
        }
        idle
    }

    /// Drop the current target if it earned nothing since the last check, so the
    /// sampler can look elsewhere. Called once a frame.
    ///
    /// A productive target has to STICK. Re-sampling unconditionally every frame
    /// measured worse than not re-sampling at all, 679 us/frame against 509,
    /// because the sample lands at the top of the frame where the game is inside
    /// its V-blank handler, and the scan happily finds some other pure loop
    /// there and re-points at it. Half the wait loop then went unskipped.
    pub fn retire_if_idle_target_is_dead(&mut self) {
        if self.watching() && self.skips == self.skips_at_check {
            self.retargets += 1;
            self.forget();
        }
        self.skips_at_check = self.skips;
    }

    /// Account a skip of `cycles` emulated cycles.
    pub fn skipped(&mut self, cycles: u64) {
        self.skips += 1;
        self.skipped_cycles += cycles;
    }
}

/// A backward-branching loop found by [`find_loop`].
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Loop {
    /// Address of the first instruction, the branch target.
    pub head: u32,
    /// Address of the conditional branch that closes the loop.
    pub branch: u32,
}

impl Loop {
    /// Instructions in the body, including the branch.
    pub fn len(&self) -> u32 {
        (self.branch.wrapping_sub(self.head)) / 2 + 1
    }
}

/// How far forward to look for the branch that closes a loop.
const MAX_SCAN: u32 = 32;
/// How far back that branch may reach. A wait loop is a handful of
/// instructions; a long backward branch is ordinary control flow.
const MAX_SPAN: u32 = 64;

/// Where a Thumb conditional branch at `at` goes, if it is one.
///
/// Only format 16. An UNCONDITIONAL backward branch is deliberately not
/// accepted: a loop with no way out is a hang, not an idle wait, and reporting
/// it as idle would paper over a real bug.
pub fn cond_branch_target(op: u16, at: u32) -> Option<u32> {
    if op >> 12 != 0b1101 {
        return None;
    }
    // 0xE is the undefined encoding and 0xF is SWI, neither of which branches.
    if (op >> 8) & 0xF >= 0xE {
        return None;
    }
    let off = ((op & 0xFF) as u8 as i8 as i32) * 2;
    Some((at as i32).wrapping_add(4).wrapping_add(off) as u32)
}

/// Is this instruction a load or a register operation, with no side effect
/// beyond the register file?
///
/// A whitelist, not a blacklist: anything not positively identified returns
/// false. Getting this wrong in the permissive direction means skipping a loop
/// that was doing something, so the default has to be "no".
pub fn pure_in_loop(op: u16) -> bool {
    match op >> 12 {
        // Formats 1 and 2: shifts by immediate, add/sub. Register only.
        0b0000 | 0b0001 => true,
        // Format 3: mov/cmp/add/sub immediate.
        0b0010 | 0b0011 => true,
        0b0100 => {
            if (op >> 10) & 0x3 == 0b00 {
                true // format 4, the ALU ops, register only
            } else {
                // 0100 1xxx is format 6, LDR PC-relative, a load.
                // 0100 01xx is format 5, hi-register ops and BX, which can
                // write R15.
                (op >> 11) & 1 == 1
            }
        }
        0b0101 => {
            if (op >> 9) & 1 == 0 {
                (op >> 11) & 1 == 1 // format 7: L set is LDR/LDRB
            } else {
                // Format 8, bits 11:10 are H:S. Both clear is STRH, a store;
                // the other three are loads.
                (op >> 10) & 0x3 != 0b00
            }
        }
        // Format 9, LDR/STR with immediate offset: bit 11 is L.
        0b0110 | 0b0111 => (op >> 11) & 1 == 1,
        // Format 10, LDRH/STRH.
        0b1000 => (op >> 11) & 1 == 1,
        // Format 11, SP-relative LDR/STR.
        0b1001 => (op >> 11) & 1 == 1,
        // Format 12, ADD Rd, PC/SP, #imm. Writes a register only.
        0b1010 => true,
        // Everything else is rejected, some of it conservatively: 1011 is
        // PUSH/POP and ADD SP, 1100 is LDMIA/STMIA, 1101 is branch and SWI,
        // 111x are the branches. The loads among those are not worth allowing.
        _ => false,
    }
}

/// Look for an idle-shaped loop containing `from`.
///
/// Scans forward for a conditional branch that jumps back to at or before
/// `from`, then requires every instruction from the target through the branch
/// to pass [`pure_in_loop`]. `read` returns `None` for an unreadable address.
///
/// Returning `Some` does NOT mean the loop is idle, only that skipping it would
/// have no side effects. The no-progress proof is the caller's dynamic check.
pub fn find_loop<F>(from: u32, mut read: F) -> Option<Loop>
where
    F: FnMut(u32) -> Option<u16>,
{
    let mut at = from;
    for _ in 0..MAX_SCAN {
        let op = read(at)?;
        if let Some(target) = cond_branch_target(op, at) {
            // Backward, reaching no further than one wait loop's worth, and
            // not past the instruction we sampled.
            if target > from || at.wrapping_sub(target) > MAX_SPAN {
                return None;
            }
            let l = Loop { head: target, branch: at };
            let mut b = target;
            while b < at {
                if !pure_in_loop(read(b)?) {
                    return None;
                }
                b = b.wrapping_add(2);
            }
            return Some(l);
        }
        // A branch, call or SWI before the loop closes means this is ordinary
        // control flow rather than a spin.
        if !pure_in_loop(op) {
            return None;
        }
        at = at.wrapping_add(2);
    }
    None
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reader(base: u32, words: &[u16]) -> impl FnMut(u32) -> Option<u16> + '_ {
        move |addr| {
            let off = addr.checked_sub(base)? / 2;
            words.get(off as usize).copied()
        }
    }

    /// The real LeafGreen wait loop, byte for byte off the cartridge.
    const LG_WAIT: [u16; 5] = [
        0x8B91, // LDRH R1, [R2, #28]
        0x1C18, // ADD  R0, R3, #0
        0x4008, // AND  R0, R1
        0x2800, // CMP  R0, #0
        0xD0FA, // BEQ  -12
    ];

    #[test]
    fn the_real_leafgreen_wait_loop_is_found() {
        let base = 0x0800_08BE;
        let l = find_loop(base, reader(base, &LG_WAIT)).expect("the wait loop");
        assert_eq!(l.head, base, "the branch target is the loop head");
        assert_eq!(l.branch, base + 8);
        assert_eq!(l.len(), 5);
    }

    /// Sampling lands anywhere in the loop, not just on its head, so finding it
    /// from the middle is the normal case rather than an edge one.
    #[test]
    fn the_loop_is_found_from_the_middle_of_its_body() {
        let base = 0x0800_08BE;
        for skip in 0..4u32 {
            let from = base + skip * 2;
            let l = find_loop(from, reader(base, &LG_WAIT))
                .unwrap_or_else(|| panic!("not found from +{skip}"));
            assert_eq!(l.head, base);
        }
    }

    /// A store in the body is the whole reason for the static check: losing an
    /// iteration would lose a write.
    #[test]
    fn a_body_that_stores_is_rejected() {
        let mut code = LG_WAIT;
        code[1] = 0x6008; // STR R0, [R1]
        assert!(find_loop(0x0800_08BE, reader(0x0800_08BE, &code)).is_none());
    }

    /// A call out of the loop is not a spin, whatever else it does.
    #[test]
    fn a_body_that_calls_or_switches_state_is_rejected() {
        for intruder in [
            0xF000u16, // BL prefix
            0x4770,    // BX LR
            0xDF06,    // SWI 6
            0xB500,    // PUSH {LR}
            0xBD00,    // POP {PC}
            0xC801,    // LDMIA R0!, {R0}
        ] {
            let mut code = LG_WAIT;
            code[1] = intruder;
            assert!(
                find_loop(0x0800_08BE, reader(0x0800_08BE, &code)).is_none(),
                "{intruder:04X} must not appear in an idle body"
            );
        }
    }

    /// An unconditional backward branch is a hang, not a wait, and must not be
    /// reported as idle: dressing a real bug up as an optimisation would hide it.
    #[test]
    fn an_unconditional_backward_branch_is_not_idle() {
        assert_eq!(cond_branch_target(0xE7FE, 0x0800_0000), None, "B . is not format 16");
        let code = [0x2001u16, 0xE7FD]; // MOV R0,#1 ; B back
        assert!(find_loop(0x0800_0000, reader(0x0800_0000, &code)).is_none());
    }

    /// A forward conditional branch leaves the run; it is not a loop.
    #[test]
    fn a_forward_branch_is_not_a_loop() {
        let code = [0x2800u16, 0xD002, 0x2001]; // CMP R0,#0 ; BEQ +4 ; MOV R0,#1
        assert!(find_loop(0x0800_0000, reader(0x0800_0000, &code)).is_none());
    }

    /// A long backward branch is ordinary control flow, e.g. the bottom of a
    /// real work loop, and must not be mistaken for a spin.
    #[test]
    fn a_branch_reaching_far_back_is_not_a_wait_loop() {
        // BEQ with offset -128 halfwords, so 256 bytes back, well past MAX_SPAN.
        let at = 0x0800_1000u32;
        let target = cond_branch_target(0xD080, at).expect("conditional");
        assert!(at - target > MAX_SPAN, "the fixture must exceed the span limit");
        let l = find_loop(at, |a| if a == at { Some(0xD080) } else { Some(0x2001) });
        assert_eq!(l, None);
    }

    /// Pin the branch arithmetic directly: Thumb conditional branches are
    /// relative to PC+4 in halfwords, and a sign error here would point the
    /// whole mechanism at the wrong address.
    #[test]
    fn conditional_branch_targets_are_pc_plus_four_in_halfwords() {
        assert_eq!(cond_branch_target(0xD0FA, 0x0800_08C6), Some(0x0800_08BE)); // -6
        assert_eq!(cond_branch_target(0xD000, 0x0800_0000), Some(0x0800_0004)); // 0
        assert_eq!(cond_branch_target(0xD07F, 0x0800_0000), Some(0x0800_0102)); // +127
        // -128 halfwords is 256 bytes back, which from the ROM base lands below it.
        assert_eq!(cond_branch_target(0xD080, 0x0800_0000), Some(0x07FF_FF04));
    }

    /// The whitelist is the only thing standing between this and skipping real
    /// work, so pin both directions of it.
    #[test]
    fn the_whitelist_admits_loads_and_register_ops_only() {
        for op in [
            0x0040u16, // LSL R0, R0, #1   format 1
            0x1808,    // ADD R0, R1, R0   format 2
            0x2001,    // MOV R0, #1       format 3
            0x4008,    // AND R0, R1       format 4
            0x4809,    // LDR R0, [PC,#36] format 6
            0x5808,    // LDR R0, [R1,R2]  format 7 load
            0x5E08,    // LDRSH            format 8 load
            0x6808,    // LDR R0, [R1,#0]  format 9 load
            0x8B91,    // LDRH R1,[R2,#28] format 10 load
            0x9801,    // LDR R0, [SP,#4]  format 11 load
            0xA001,    // ADD R0, PC, #4   format 12
        ] {
            assert!(pure_in_loop(op), "{op:04X} is a load or a register op");
        }
        for op in [
            0x4770u16, // BX LR            format 5
            0x5008,    // STR R0, [R1,R2]  format 7 store
            0x5208,    // STRH             format 8 store
            0x6008,    // STR R0, [R1,#0]  format 9 store
            0x8008,    // STRH R0,[R1,#0]  format 10 store
            0x9001,    // STR R0, [SP,#4]  format 11 store
            0xB500,    // PUSH {LR}        format 14
            0xC001,    // STMIA            format 15
            0xD0FA,    // BEQ              format 16
            0xDF06,    // SWI              format 17
            0xE7FE,    // B                format 18
            0xF000,    // BL prefix        format 19
        ] {
            assert!(!pure_in_loop(op), "{op:04X} is not safe to lose");
        }
    }

    /// The LeafGreen wait loop, driven the way the frame loop drives it: five
    /// instructions per iteration, register file unchanged.
    #[test]
    fn a_genuine_wait_loop_proves_idle_on_the_second_iteration() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let mut r = [0u32; 16];
        r[2] = 0x0300_7FF8;

        assert!(!w.consider(1000, &r, 0x1F), "one iteration cannot prove anything");
        assert!(w.consider(1005, &r, 0x1F), "a second consecutive iteration proves it");
    }

    /// The bug that broke 53 of 80 ROMs. A loop that advances a pointer every
    /// iteration is NOT idle, but a function called twice with the same
    /// arguments arrives twice with an identical register file. Comparing
    /// arrivals accepted it; comparing ITERATIONS must not.
    #[test]
    fn two_separate_calls_with_the_same_arguments_are_not_idle() {
        let mut w = Watch::default();
        w.watch(0x0800_1000, 4);
        let r = [0x0200_0000u32; 16];

        assert!(!w.consider(1000, &r, 0x1F));
        // Called again much later, arguments identical, hundreds of
        // instructions in between.
        assert!(!w.consider(5000, &r, 0x1F), "not consecutive iterations");
        assert!(!w.consider(9000, &r, 0x1F));
    }

    #[test]
    fn a_loop_that_advances_a_register_is_never_idle() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 4);
        let mut r = [0u32; 16];
        for i in 0..32u64 {
            r[0] = i as u32; // a counting loop
            assert!(!w.consider(1000 + i * 4, &r, 0x1F), "a counter advanced at {i}");
        }
    }

    /// The proof has to be re-established every time, not recorded once.
    ///
    /// A first cut froze the proven state and compared against it. On a scene
    /// where the polled word advances once a frame it proved idle, then refused
    /// every later skip, 14,050 times a frame, and came out slower than no
    /// detection at all. Here the polled value changes, then settles again, and
    /// the loop must become skippable once more.
    #[test]
    fn the_proof_recovers_after_the_polled_value_moves() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let mut r = [0u32; 16];

        r[1] = 0x1000;
        assert!(!w.consider(100, &r, 0x1F));
        assert!(w.consider(105, &r, 0x1F), "settled, so skippable");

        r[1] = 0x1001; // the handler moved the word
        assert!(!w.consider(110, &r, 0x1F), "it changed, so not idle this time");
        assert!(w.consider(115, &r, 0x1F), "and it must become skippable again");
    }

    /// Right after a skip the loop head is reached again with NOTHING having
    /// executed in between, and that must NOT count as proof.
    ///
    /// It looks like it should: nothing ran, so nothing changed. Accepting it
    /// froze 41 of 80 ROMs at zero instructions per frame. The loop has to RUN
    /// to observe the value it is waiting for; skip the body and the state stays
    /// identical forever, so the skip justifies itself in a closed circle and
    /// the game never advances again.
    #[test]
    fn a_phase_boundary_with_no_iteration_in_between_is_not_proof() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [0u32; 16];
        assert!(!w.consider(1000, &r, 0x1F));
        assert!(w.consider(1005, &r, 0x1F), "proved by one iteration");
        // The phase ended here. Next phase, same step count, nothing ran.
        assert!(
            !w.consider(1005, &r, 0x1F),
            "the body must run again before another skip can be justified"
        );
        assert!(w.consider(1010, &r, 0x1F), "and after one iteration it may skip");
    }

    /// But a handler that runs between two arrivals DOES intervene, even if it
    /// leaves the register file as it found it, because it can have changed the
    /// memory the loop is polling.
    #[test]
    fn a_handler_that_restores_every_register_still_breaks_the_proof() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [0u32; 16];
        assert!(!w.consider(1000, &r, 0x1F));
        assert!(w.consider(1005, &r, 0x1F));
        // 40 instructions ran and restored everything: neither len nor zero.
        assert!(!w.consider(1045, &r, 0x1F), "something ran, so re-prove it");
    }

    /// An interrupt taken mid-loop inserts instructions, so the step delta stops
    /// matching. Declining that time is correct and costs one iteration.
    #[test]
    fn an_interrupt_in_the_middle_of_an_iteration_is_not_idle() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [0u32; 16];
        assert!(!w.consider(1000, &r, 0x1F));
        assert!(!w.consider(1200, &r, 0x1F), "a handler ran in between");
        assert!(w.consider(1205, &r, 0x1F), "back to back again");
    }

    /// The flags live in CPSR and are the loop's actual condition, so a loop
    /// whose comparison result changes has made progress even with identical
    /// general registers.
    #[test]
    fn a_change_confined_to_the_flags_counts_as_progress() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [7u32; 16];
        assert!(!w.consider(1000, &r, 0x4000_001F)); // Z set
        assert!(!w.consider(1005, &r, 0x0000_001F), "the condition changed");
        assert!(w.consider(1010, &r, 0x0000_001F), "and once settled it may skip");
    }

    /// Moving to a different loop head has to drop the old snapshot, or the new
    /// loop inherits a proof it never earned.
    #[test]
    fn watching_a_new_head_discards_the_old_snapshot() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [0u32; 16];
        w.consider(1000, &r, 0x1F);
        assert!(w.consider(1005, &r, 0x1F));
        w.watch(0x0900_0000, 5);
        assert!(!w.consider(1010, &r, 0x1F), "a new head needs its own two iterations");
    }

    /// `forget` runs on reset and after a state load. It must stop watching,
    /// since skipping at an address the new state does not spin at would skip
    /// real code, while keeping the counters that describe the run.
    #[test]
    fn forgetting_stops_watching_but_keeps_the_counters() {
        let mut w = Watch::default();
        w.watch(0x0800_08C2, 5);
        let r = [0u32; 16];
        w.consider(1000, &r, 0x1F);
        assert!(w.consider(1005, &r, 0x1F));
        w.skipped(500);
        w.forget();
        assert!(!w.watching(), "nothing is watched after a load");
        assert_eq!(w.probe_r15, u32::MAX);
        assert_eq!(w.skips, 1, "the counters describe the whole run");
        assert_eq!(w.skipped_cycles, 500);
    }

    /// Nothing readable. A runaway PC reaches here and must not hang or panic.
    #[test]
    fn an_unreadable_address_finds_nothing() {
        assert_eq!(find_loop(0x1000_0000, |_| None), None);
    }
}
