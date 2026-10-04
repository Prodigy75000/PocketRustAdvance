// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! The recompiler.
//!
//! **Status: scaffolding. No code is generated yet and nothing in the emulator
//! calls into this module, so the interpreter is still the only execution
//! engine.** That is deliberate: the two pieces here are the ones that decide
//! whether the approach is viable at all, and both are testable on their own.
//!
//! ## Why a recompiler, in numbers
//!
//! Measured on a LeafGreen battle scene, CPU and bus are 69% of frame time and
//! cost about 40 host cycles per emulated instruction. The decomposition is the
//! reason this is codegen rather than more interpreter tuning:
//!
//! ```text
//! instruction fetch read    12.1%
//! instruction fetch charge   ~9%
//! Thumb decode tree         below the noise floor, effectively free
//! everything else           ~75%   handler bodies: r[] traffic, the barrel
//!                                  shifter, computing four flags, storing CPSR
//! ```
//!
//! The fetch side is only about a fifth of the time, so caching decoded blocks
//! and replaying handlers, which is what a "cached interpreter" does, caps out
//! near 1.25x. The ~75% is the work of interpreting each instruction, and that
//! is what native code collapses: an emulated `ADD` becomes one host `add` plus
//! host flags.
//!
//! ## What is NOT being traded
//!
//! Per-access bus timing stays. Compiled code calls the real bus for every data
//! access, charging the same cycles through the same path, because that
//! precision is load-bearing here: it is what fixed the GTA Advance teardown
//! race, and the compatibility corpus sits at 99.1%. Owner decision, taken
//! explicitly with the cost stated.
//!
//! Instruction fetches are the part that can get cheaper without losing
//! exactness, because the translator knows at compile time what a block's fetch
//! sequence is.

pub static mut BP_HIST: [u64; 72] = [0; 72];
pub static mut BP_RUN: u64 = 0;
pub static mut BP_PREV: u32 = 0;
pub static mut BP_DIVERTED: bool = false;
pub static mut BP_BLOCKS: u64 = 0;
pub static mut BP_INSTR: u64 = 0;
pub static mut BP_ARM: u64 = 0;

pub mod block;
pub mod code;
