// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! The CPU/memory boundary.
//!
//! This trait is the single most important architectural decision in the core:
//! the ARM7TDMI never touches a concrete GBA memory map. It talks only to a
//! `Bus`. That buys us two independent implementations of the same CPU:
//!
//!   - the **TomHarte harness** (`gba-runner`) backs the bus with flat RAM and
//!     no MMIO, so every one of the SingleStepTests JSON vectors can drive the
//!     CPU in complete isolation, before a PPU or timer exists;
//!   - the **real GBA bus** (`GbaBus`, added later) implements the 8 memory
//!     regions, WAITCNT wait-states, the cartridge prefetch buffer, and open-bus
//!     reads.
//!
//! Same `Arm7tdmi`, two `Bus` impls. If the memory map were baked into the CPU
//! this door would be shut.

/// Access width, threaded through so the bus can model region-specific timing
/// (e.g. 16-bit-only regions incur an extra access for a 32-bit read) and the
/// prefetch buffer. The CPU itself is width-agnostic beyond sizing the value.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Width {
    Byte,
    Half,
    Word,
}

/// Whether an instruction/data fetch is Sequential or Non-sequential — the S/N
/// cycle distinction the GBA's wait-state model is built on. The CPU knows this
/// from its access pattern; the bus turns it into a cycle count.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Access {
    Seq,
    NonSeq,
}

/// Everything the ARM7TDMI needs from the outside world.
///
/// Reads/writes are little-endian. Addresses are full 32-bit; region decoding
/// and mirroring live entirely on the bus side. `tick` is how the CPU hands
/// back cycles it isn't itself accounting for (internal/`I` cycles); the S/N
/// cost of a memory access is charged by the bus inside the access itself.
pub trait Bus {
    fn read8(&mut self, addr: u32, access: Access) -> u8;
    fn read16(&mut self, addr: u32, access: Access) -> u16;
    fn read32(&mut self, addr: u32, access: Access) -> u32;

    fn write8(&mut self, addr: u32, val: u8, access: Access);
    fn write16(&mut self, addr: u32, val: u16, access: Access);
    fn write32(&mut self, addr: u32, val: u32, access: Access);

    /// Charge `n` internal cycles (the `I` cycles of MUL, shifts by register,
    /// etc.). Memory-access cycles are charged inside the read/write calls.
    fn tick(&mut self, n: u32);

    /// HLE BIOS halt hook: the Halt / IntrWait SWIs park the CPU until an IRQ.
    /// Default no-op so the TomHarte harness bus need not model it.
    fn set_halted(&mut self, _halted: bool) {}
}