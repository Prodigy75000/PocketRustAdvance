// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! The ARM7TDMI: an ARMv4T core with a 3-stage pipeline and two instruction
//! sets (32-bit ARM, 16-bit Thumb).
//!
//! Module layout mirrors the natural seams of the chip:
//!   - [`psr`]    : CPSR/SPSR, modes, condition-code evaluation.
//!   - [`barrel`] : the barrel shifter (carry-out semantics shared by ARM+Thumb).
//!   - [`arm`]    : 32-bit ARM decode+execute (data-processing online; rest TODO).
//!   - `thumb`    : 16-bit Thumb decode+execute (19 formats) — TODO
//!
//! Everything reaches memory through [`crate::bus::Bus`], never a concrete map,
//! so the whole CPU runs under the TomHarte harness before the rest of the GBA
//! exists.
//!
//! ## Pipeline model
//!
//! The chip has a 3-stage pipeline; we model the two prefetched words the
//! TomHarte vectors expose as `pipeline[0]` (the instruction to execute now) and
//! `pipeline[1]` (the next, already fetched). `r[15]` holds the address of the
//! word to be fetched next, which is why a reading instruction sees R15 as
//! `PC + 8` (ARM) — the executing instruction sits two words behind the fetch.
//!
//! Each [`step`](Arm7tdmi::step):
//!   1. executes `pipeline[0]`;
//!   2. on a normal (non-branch) instruction, prefetches one word at `r[15]`,
//!      shifts `pipeline[1] -> pipeline[0]`, stores the fetch in `pipeline[1]`,
//!      and advances `r[15]` by 4;
//!   3. on a branch (any write to R15), flushes and refills both slots.

pub mod arm;
pub mod barrel;
pub mod hle;
pub mod psr;
pub mod thumb;

use crate::bus::{Access, Bus};
use psr::{Cond, Flags, Mode};

/// GBA_SWILOG=<max lines>: trace every BIOS call, with the caller and the
/// argument registers. A game that runs under one BIOS image and hangs under
/// another diverges at a single SWI, and diffing two traces is what names it.
/// From outside, "the BIOS returned something different" and "the game took a
/// different branch" look exactly alike.
///
/// Capped because Div and the decompression calls run in the thousands per
/// frame; the default 2000 lines covers a boot.
pub(crate) fn swilog(num: u8, caller: u32, r: &[u32; 16]) {
    static LIMIT: std::sync::OnceLock<u64> = std::sync::OnceLock::new();
    let limit = *LIMIT.get_or_init(|| match std::env::var("GBA_SWILOG") {
        Ok(v) => v.parse().unwrap_or(2000),
        Err(_) => 0,
    });
    if limit == 0 {
        return;
    }
    static SEEN: std::sync::atomic::AtomicU64 = std::sync::atomic::AtomicU64::new(0);
    let n = SEEN.fetch_add(1, std::sync::atomic::Ordering::Relaxed);
    if n < limit {
        eprintln!(
            "  SWI {num:02X} from {caller:08X} r0={:08X} r1={:08X} r2={:08X} r3={:08X}",
            r[0], r[1], r[2], r[3]
        );
    }
}

/// R13 (SP), R14 (LR), R15 (PC) have their usual conventional indices.
pub const SP: usize = 13;
pub const LR: usize = 14;
pub const PC: usize = 15;

/// Maps a mode's low 5 bits to a 0..6 bank class for r13/r14:
/// User/System share bank 0; FIQ=1, Supervisor=2, Abort=3, IRQ=4, Undefined=5.
fn bank_class(mode: u32) -> usize {
    match mode & 0x1F {
        0x11 => 1, // FIQ
        0x13 => 2, // Supervisor
        0x17 => 3, // Abort
        0x12 => 4, // IRQ
        0x1B => 5, // Undefined
        _ => 0,    // User / System
    }
}

/// The ARM7TDMI register/status state, with full register banking.
///
/// `r` holds the *active* view r0..r15. The r8..r14 that a mode banks away are
/// stored in the save areas below and swapped in/out by [`Arm7tdmi::write_cpsr`]
/// whenever the mode changes. r0..r7 and r15 are never banked.
pub struct Arm7tdmi {
    /// The 16 currently-visible registers r0..r15 (r15 == PC, see pipeline note).
    pub r: [u32; 16],
    /// Current Program Status Register (NZCV + control bits + mode).
    pub cpsr: u32,
    /// The two prefetched pipeline words: [execute slot, fetch slot].
    pub pipeline: [u32; 2],
    /// When set, SWIs are emulated at the CPU level (HLE BIOS) instead of
    /// vectoring to 0x08. Enabled on direct boot when no real BIOS is present.
    pub hle_bios: bool,

    /// Saved r8..r12: [0] = the bank shared by every non-FIQ mode, [1] = FIQ.
    r8_12: [[u32; 5]; 2],
    /// Saved r13/r14 per bank class (see [`bank_class`]).
    r13_14: [[u32; 2]; 6],
    /// SPSRs for [FIQ, Supervisor, Abort, IRQ, Undefined].
    spsr_bank: [u32; 5],
}

impl Default for Arm7tdmi {
    fn default() -> Self {
        Arm7tdmi {
            r: [0; 16],
            cpsr: Mode::System as u32,
            pipeline: [0; 2],
            hle_bios: false,
            r8_12: [[0; 5]; 2],
            r13_14: [[0; 2]; 6],
            spsr_bank: [0; 5],
        }
    }
}

impl Arm7tdmi {
    pub fn new() -> Self {
        Self::default()
    }

    /// Serialize the full register/banked state for a save-state.
    pub fn serialize(&self, w: &mut crate::state::Writer) {
        for &v in &self.r {
            w.u32(v);
        }
        w.u32(self.cpsr);
        w.u32(self.pipeline[0]);
        w.u32(self.pipeline[1]);
        w.bool(self.hle_bios);
        for bank in &self.r8_12 {
            for &v in bank {
                w.u32(v);
            }
        }
        for bank in &self.r13_14 {
            w.u32(bank[0]);
            w.u32(bank[1]);
        }
        for &v in &self.spsr_bank {
            w.u32(v);
        }
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        for v in &mut self.r {
            *v = r.u32();
        }
        self.cpsr = r.u32();
        self.pipeline[0] = r.u32();
        self.pipeline[1] = r.u32();
        self.hle_bios = r.bool();
        for bank in &mut self.r8_12 {
            for v in bank {
                *v = r.u32();
            }
        }
        for bank in &mut self.r13_14 {
            bank[0] = r.u32();
            bank[1] = r.u32();
        }
        for v in &mut self.spsr_bank {
            *v = r.u32();
        }
    }

    /// The C flag (carry), needed as a shifter/ALU carry-in.
    pub fn carry(&self) -> bool {
        self.cpsr & (1 << 29) != 0
    }

    /// Whether the T (Thumb) bit is set.
    pub fn thumb(&self) -> bool {
        self.cpsr & (1 << 5) != 0
    }

    /// Whether the current mode has a saved PSR. User and System do not, which
    /// makes the `S`-with-Rd=15 "restore CPSR from SPSR" behavior fall back to a
    /// plain flag update.
    pub fn has_spsr(&self) -> bool {
        !matches!(self.cpsr & 0x1F, 0x10 | 0x1F)
    }

    /// Read register `i` as seen from *User* mode, regardless of the current
    /// mode. Used by the S-bit ("^") forms of LDM/STM, which transfer the user
    /// bank even while a privileged mode is active.
    pub fn user_reg(&self, i: usize) -> u32 {
        match i {
            8..=12 if self.cpsr & 0x1F == 0x11 => self.r8_12[0][i - 8], // FIQ shadows r8..r12
            13 | 14 if bank_class(self.cpsr) != 0 => self.r13_14[0][i - 13],
            _ => self.r[i],
        }
    }

    /// Write register `i` in the User bank (companion to [`Arm7tdmi::user_reg`]).
    pub fn set_user_reg(&mut self, i: usize, v: u32) {
        match i {
            8..=12 if self.cpsr & 0x1F == 0x11 => self.r8_12[0][i - 8] = v,
            13 | 14 if bank_class(self.cpsr) != 0 => self.r13_14[0][i - 13] = v,
            _ => self.r[i] = v,
        }
    }

    /// The SPSR of the current mode (0 in User/System, which have none).
    pub fn current_spsr(&self) -> u32 {
        match self.cpsr & 0x1F {
            0x11 => self.spsr_bank[0],
            0x13 => self.spsr_bank[1],
            0x17 => self.spsr_bank[2],
            0x12 => self.spsr_bank[3],
            0x1B => self.spsr_bank[4],
            _ => 0,
        }
    }

    /// Write the current mode's SPSR (no-op in User/System).
    pub fn set_current_spsr(&mut self, v: u32) {
        match self.cpsr & 0x1F {
            0x11 => self.spsr_bank[0] = v,
            0x13 => self.spsr_bank[1] = v,
            0x17 => self.spsr_bank[2] = v,
            0x12 => self.spsr_bank[3] = v,
            0x1B => self.spsr_bank[4] = v,
            _ => {}
        }
    }

    /// Write CPSR, banking r8..r14 out/in if the mode changed. All PSR restores
    /// and mode switches must go through here so the visible register file stays
    /// consistent with the new mode.
    pub fn write_cpsr(&mut self, new: u32) {
        // Every valid mode has M[4] == 1 (ARM DDI 0029, Table 2-2); the mode
        // field's bit 4 is architecturally 1.
        let new = new | 0x10;
        let old_mode = self.cpsr & 0x1F;
        let new_mode = new & 0x1F;
        if old_mode != new_mode {
            self.save_bank(old_mode);
            self.cpsr = new;
            self.load_bank(new_mode);
        } else {
            self.cpsr = new;
        }
    }

    /// Copy the active r8..r14 into `mode`'s save areas.
    fn save_bank(&mut self, mode: u32) {
        let g = if mode & 0x1F == 0x11 { 1 } else { 0 };
        self.r8_12[g].copy_from_slice(&self.r[8..13]);
        let c = bank_class(mode);
        self.r13_14[c] = [self.r[13], self.r[14]];
    }

    /// Load `mode`'s banked r8..r14 into the active view.
    fn load_bank(&mut self, mode: u32) {
        let g = if mode & 0x1F == 0x11 { 1 } else { 0 };
        self.r[8..13].copy_from_slice(&self.r8_12[g]);
        let c = bank_class(mode);
        self.r[13] = self.r13_14[c][0];
        self.r[14] = self.r13_14[c][1];
    }

    /// Load the full banked register state (used by the TomHarte harness). The
    /// arrays follow the vector format: `r_usr` is the User/System view of
    /// r0..r15, `r_fiq` is FIQ r8..r14, and the rest are r13/r14 per mode.
    #[allow(clippy::too_many_arguments)]
    pub fn load_full(
        &mut self,
        cpsr: u32,
        r_usr: [u32; 16],
        r_fiq: [u32; 7],
        r_svc: [u32; 2],
        r_abt: [u32; 2],
        r_irq: [u32; 2],
        r_und: [u32; 2],
        spsr_bank: [u32; 5],
    ) {
        self.cpsr = cpsr;
        self.r[0..8].copy_from_slice(&r_usr[0..8]);
        self.r[15] = r_usr[15];

        self.r8_12[0].copy_from_slice(&r_usr[8..13]); // non-FIQ r8..r12
        self.r8_12[1].copy_from_slice(&r_fiq[0..5]); // FIQ r8..r12
        self.r13_14[0] = [r_usr[13], r_usr[14]];
        self.r13_14[1] = [r_fiq[5], r_fiq[6]];
        self.r13_14[2] = r_svc;
        self.r13_14[3] = r_abt;
        self.r13_14[4] = r_irq;
        self.r13_14[5] = r_und;
        self.spsr_bank = spsr_bank;

        self.load_bank(cpsr & 0x1F); // activate the current mode's r8..r14
    }

    /// Whether an instruction with condition `cond` should execute now.
    pub fn cond_passes(&self, cond: Cond) -> bool {
        Flags::from_cpsr(self.cpsr).eval(cond)
    }

    /// Execute one instruction (the word in `pipeline[0]`), then advance the
    /// pipeline. See the module-level pipeline note.
    pub fn step<B: Bus>(&mut self, bus: &mut B) {
        let op = self.pipeline[0];
        let thumb = self.thumb();

        // The fetch stage prefetches the next instruction *first* (a sequential
        // fetch at R15, which points two instructions ahead), before the
        // instruction's own data accesses. R15 is left untouched for execute, so
        // reads of R15 still see the pipeline-ahead value. On a branch this
        // prefetch is the in-flight fetch that completes before the flush.
        let fetch_addr = self.r[15];
        let (fetched, width) = if thumb {
            (bus.read16(fetch_addr, Access::Seq) as u32, 2)
        } else {
            (bus.read32(fetch_addr, Access::Seq), 4)
        };

        let branched = if thumb {
            thumb::execute(self, bus, op as u16)
        } else {
            let cond = Cond::decode(op >> 28);
            if Flags::from_cpsr(self.cpsr).eval(cond) {
                arm::execute(self, bus, op)
            } else {
                false
            }
        };

        if branched {
            self.refill(bus);
        } else {
            self.pipeline[0] = self.pipeline[1];
            self.pipeline[1] = fetched;
            // Normally the fetch advances PC by one instruction. But if the
            // instruction itself wrote R15 without branching (e.g. MRS Rd=15,
            // which is unpredictable and does not flush), that write stands.
            if self.r[15] == fetch_addr {
                self.r[15] = fetch_addr.wrapping_add(width);
            }
        }
    }

    /// Fill the pipeline from the current PC (used at boot / after a manual PC
    /// set). Same two-fetch flush a branch performs.
    pub fn reload_pipeline<B: Bus>(&mut self, bus: &mut B) {
        self.refill(bus);
    }

    /// Whether IRQs are currently enabled on the CPU (the CPSR I bit is clear).
    pub fn irq_ready(&self) -> bool {
        self.cpsr & (1 << 7) == 0
    }

    /// Enter the IRQ exception: banks into IRQ mode with the return address in
    /// LR_irq, the old CPSR in SPSR_irq, IRQs masked and ARM state forced, then
    /// vectors to 0x18. Return is via `SUBS PC, LR, #4`, hence the LR = A + 4.
    pub fn take_irq<B: Bus>(&mut self, bus: &mut B) {
        let width = if self.thumb() { 2 } else { 4 };
        let return_addr = self.r[15].wrapping_sub(2 * width).wrapping_add(4);
        let old = self.cpsr;
        let new = (old & !0x1F & !(1 << 5)) | 0x12 | (1 << 7); // IRQ mode, ARM, I set
        self.write_cpsr(new);
        self.set_current_spsr(old);
        self.r[14] = return_addr;
        self.r[15] = 0x0000_0018;
        self.refill(bus);
    }

    /// Flush the pipeline after a branch: two fresh fetches at the new PC, in
    /// whichever instruction width the current state selects.
    ///
    /// The prefetches are aligned to the instruction width, but the ARM7TDMI
    /// leaves the target's sub-word low bits in R15 itself — the final register
    /// is `target + step`, not `(target & align) + step`.
    fn refill<B: Bus>(&mut self, bus: &mut B) {
        let target = self.r[15];
        if self.thumb() {
            let aligned = target & !1;
            self.pipeline[0] = bus.read16(aligned, Access::NonSeq) as u32;
            self.pipeline[1] = bus.read16(aligned.wrapping_add(2), Access::Seq) as u32;
            self.r[15] = target.wrapping_add(4);
        } else {
            let aligned = target & !3;
            self.pipeline[0] = bus.read32(aligned, Access::NonSeq);
            self.pipeline[1] = bus.read32(aligned.wrapping_add(4), Access::Seq);
            self.r[15] = target.wrapping_add(8);
        }
    }
}