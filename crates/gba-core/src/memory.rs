// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! The real GBA memory bus: the 8 regions, their mirroring, cycle costs, and
//! I/O dispatch. Implements [`crate::bus::Bus`] so the CPU drives it exactly as
//! it drove the test harness. Region layout and mirroring follow GBATEK.

use crate::apu::Apu;
use crate::bus::{Access, Bus};
use crate::ppu::Ppu;
use crate::save::Save;

/// Serial transfer durations, in system cycles, for eight bits at each of the
/// two Normal-mode shift rates: 16777216 / 256000 * 8 and 16777216 / 2000000
/// * 8. A 32-bit transfer costs four of them.
/// Cycles a Multi-Player transfer occupies, from SIOCNT's baud field and the
/// number of units on the bus. A free function so both ends compute the
/// duration the same way: the parent sets the busy window when it clocks and
/// the child sets it when the clock lands, and a mismatch would drift the two
/// games' idea of how long a transfer takes.
fn multi_cycles(cnt: u16, units: u8) -> u32 {
    let slot = (units as usize).clamp(1, crate::cable::MAX_UNITS) - 1;
    // Hardware timing, deliberately. A floor here to hide the network round trip
    // was tried on device and the game rejected it: see the note above
    // TRANSFER_CYCLES in cable.rs for why no value can work. The wall-clock wait
    // is handled by stretching the frame on both devices instead.
    crate::cable::TRANSFER_CYCLES[(cnt & 3) as usize][slot]
}

const CYC_256KHZ_8BIT: u32 = 524;
const CYC_2MHZ_8BIT: u32 = 67;

/// Which slice an instruction-fetch window points into.
///
/// The BIOS is deliberately absent, and this was MEASURED rather than assumed.
/// Region 0 reads back its own contents only to code executing inside it, so a
/// BIOS window has to be torn down whenever `exec_in_bios` flips. Doing that
/// costs one conditional store in [`GbaBus::set_fetch_pc`], which runs on every
/// fetch, and that store cost more than the 7,000 cold window installs per
/// frame it saved: 600 us/frame against 574. BIOS code is only 3.3% of fetches.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum CodeWin {
    Ewram,
    Iwram,
    Rom,
}

pub struct GbaBus {
    pub bios: Box<[u8]>,   // 16 KB
    /// Whether the CPU is currently fetching instructions from inside the BIOS.
    /// The BIOS region is readable ONLY to code executing in it; see
    /// `set_fetch_pc`, which maintains this from the fetch address so that
    /// exception entry is covered by the same rule.
    exec_in_bios: bool,
    /// Where the CPU last fetched an instruction from, and whether that fetch
    /// was a halfword. Together they reconstruct what is sitting on the bus,
    /// which is what an unmapped address reads back.
    last_fetch: u32,
    last_fetch_thumb: bool,
    /// Which exception last vectored into the BIOS, so the value it leaves
    /// behind on the way out is the right one of the two.
    bios_entry_irq: bool,
    /// What the BIOS leaves on the bus. A read of region 0 from outside the
    /// BIOS returns THIS rather than the BIOS contents. Starts at the value
    /// GBATEK documents as left over after the boot sequence.
    ///
    /// In the save state, appended at the end so older blobs still load. The
    /// first cut left it out on the theory that the next BIOS entry would
    /// rebuild it within a frame. That is wrong for exactly the games this
    /// protection matters to: they read region 0 before the next entry and get
    /// the boot-time value, so the state does not resume where it was saved.
    bios_prefetch: u32,
    pub ewram: Box<[u8]>,  // 256 KB
    pub iwram: Box<[u8]>,  // 32 KB
    pub rom: Box<[u8]>,    // up to 32 MB
    /// Cartridge backup: SRAM / Flash (region 0xE-0xF) or EEPROM (region 0xD).
    pub save: Save,
    pub ppu: Ppu,
    pub apu: Apu,
    /// Catch-all for I/O registers not yet modeled (DMA/timers/IRQ/sound).
    io: Box<[u8]>, // 0x400
    /// Cartridge motion sensors, when the cartridge has any. Detected from the
    /// game code, so an ordinary cart never sees any of this.
    pub sensors: crate::sensor::Sensors,
    /// The wireless adapter, when the cartridge is one of the 43 titles that
    /// expect one. `None` leaves every serial path exactly as it was, which is
    /// what keeps the no-cable behaviour byte-identical for every other game.
    pub rfu: Option<crate::rfu::Rfu>,
    /// The link cable, when a netplay session is live and the cartridge does
    /// not use the wireless adapter instead. `None` leaves every serial path
    /// exactly as it was, so the offline behaviour of all 2729 licensed titles
    /// is untouched by this feature existing.
    pub cable: Option<Box<dyn crate::cable::LinkCable>>,
    /// A Multi-Player transfer is in flight over the cable. Distinguishes a
    /// cable completion from an adapter one, which clear different bits.
    cable_busy: bool,
    /// The words a child's transfer will land, held until its duration is up.
    /// `None` on a parent, which does not know them until it collects the
    /// children's replies at completion.
    cable_words: Option<[u16; crate::cable::MAX_UNITS]>,
    /// Transfers completed over the cable, and transfers the cable gave up on.
    /// Both counted because a silent cable and a broken one look identical from
    /// outside, which is how two wireless adapter bugs stayed invisible for a
    /// whole device session.
    pub cable_transfers: u64,
    pub cable_failures: u64,
    /// Give-ups where the peer was still present, the subset of
    /// `cable_failures` that is ours rather than the player's. A late word is a
    /// network artifact and the game is handed zeros for it; a peer that has
    /// gone is told the truth instead.
    pub cable_late: u64,
    /// What this unit presented when the in-flight transfer STARTED.
    ///
    /// Latched, because a game may write SIOMLT_SEND again while the transfer
    /// runs and hardware would not carry the new value. It also means a parent
    /// always knows its own slot, so a give-up never has to make one up: the
    /// old code overwrote it with FFFF, claiming the unit running the transfer
    /// was not there.
    cable_own: u16,
    /// Rolling hash of every word every completed transfer landed, in order.
    ///
    /// The two ends see the SAME four words for the same transfer, so at an equal
    /// `cable_transfers` the hashes must match. That is the one question the
    /// counters could not answer after the second device run: the wire was
    /// provably healthy and the game still refused the data, and nothing we
    /// logged could say whether the words themselves agreed. FNV-1a, and the
    /// order is part of it, so a pair delivered out of sequence diverges too.
    pub cable_wordsum: u64,
    /// The hash of the LAST window of `CABLE_MARK_EVERY` transfers, and the count
    /// it closed at.
    ///
    /// Windowed rather than cumulative, because a cumulative hash answers the
    /// wrong question: one bad word early makes every later checkpoint differ, so
    /// 46 diverged checkpoints and 1 diverged checkpoint look identical. A window
    /// says WHERE, and distinguishes a single corrupted transfer from two streams
    /// that have been offset against each other since the start.
    ///
    /// The running hash alone turned out not to be comparable between devices: the
    /// heartbeat fires on a timer, so the two ends print it at different transfer
    /// counts, and an order-dependent hash at different counts must differ. In one
    /// whole 8460-transfer run exactly ONE sample pair lined up, at zero. A
    /// checkpoint at a fixed count is the same number on both ends or the data
    /// diverged, with nothing to line up by hand.
    pub cable_mark_at: u64,
    pub cable_mark_sum: u64,
    /// The running hash of the window in progress.
    cable_window_sum: u64,
    /// The first few transfers' words, for printing once.
    pub cable_trace: Vec<(u64, u16, u16)>,
    /// `cycles` as of the last `step_serial`, so serial timing is a delta the
    /// same way the timers are.
    serial_cycles: u64,
    /// Cycles left before the in-flight serial transfer completes. Unlike the
    /// no-cable path, an adapter transfer is paced: the game polls the start
    /// bit and does an SO/SI handshake in between, so completing instantly
    /// collapses a sequence the game is relying on.
    serial_pending: u32,
    /// Decoded WAITCNT, kept in step with `io[0x204]` by `write`.
    waits: Waits,
    /// Cycles the Game Pak prefetch unit has been free to run ahead since the
    /// last ROM access, in units of bus cycles. Derived state, not serialized:
    /// worst case after a load is one under-credited fetch.
    prefetch_credit: u64,
    /// Instruction-fetch window: the address range the CPU is currently
    /// fetching code from, and which slice backs it. A fetch inside the window
    /// skips the general region decode entirely, which is one subtract and one
    /// compare instead of a sixteen-way match plus the GPIO test.
    ///
    /// It holds a RANGE and a slice selector, never a copy of the code, so
    /// self-modifying code needs no invalidation here: a write to IWRAM lands
    /// in the same slice the next fetch reads. All three backing slices are
    /// fixed-length for the life of the bus (a state load fills them in place),
    /// so the window cannot outlive what it points at.
    ///
    /// Derived state, deliberately NOT serialized: a stale window would be a
    /// correctness bug, and rebuilding it costs one cold call.
    code_lo: u32,
    /// Window size in bytes, already reduced so a 4-byte read at the last
    /// in-window address stays in bounds. Zero means "no window", which the
    /// unsigned compare rejects without a separate flag.
    code_span: u32,
    code_win: CodeWin,
    /// KEYINPUT (0x4000130): bits are active-low, 1 = released.
    pub keyinput: u16,
    /// Interrupt controller: enable mask, request flags, master enable.
    pub ie: u16,
    pub if_: u16,
    pub ime: u16,
    /// The four hardware timers, and the cycle count they were last advanced to.
    timers: [Timer; 4],
    timer_cycles: u64,
    /// Latched (running) DMA source/dest/count per channel.
    dma_src: [u32; 4],
    dma_dst: [u32; 4],
    dma_count: [u32; 4],
    /// True while a transfer is running, so a DMA that writes into the DMA
    /// control registers queues the newly enabled channel instead of nesting.
    /// Transient (always false at a frame boundary), so it stays out of the
    /// save-state and `format_version` is unaffected.
    dma_active: bool,
    /// Channels enabled from inside a running transfer, drained after it ends.
    dma_pending: u8,
    /// Free-running cycle counter; drives the frame/scanline pacing.
    pub cycles: u64,
    /// CPU halted (by the HLE Halt / IntrWait SWIs) until the next IRQ.
    pub halted: bool,
    /// Debug write-watchpoint: current CPU PC, watched address (0 = off), and a
    /// capped log of (pc, value, width) writes that hit it.
    pub cur_pc: u32,
    pub cur_mode: u32,
    /// Set by any DATA access (and by DMA). A load or store interrupts the
    /// CPU's sequential fetch stream, so the next opcode fetch is charged at
    /// the non-sequential cost; this is the implicit N in the ARM7TDMI's
    /// documented LDR = 1S+1N+1I and STR = 2N timings, and mGBA charges the
    /// same delta (its LOAD/STORE_POST_BODY). Only fetches from the Game Pak
    /// bus are affected in practice: IWRAM/BIOS are 1-cycle either way and
    /// EWRAM's non-sequential and sequential costs are equal. A data access on
    /// the cart bus additionally tears down the prefetch unit.
    data_break: bool,
    /// Real frame counter (incremented by run_frame at line 0) so debug watch
    /// hits can be stamped with the true frame/scanline instead of a
    /// cycle-derived estimate that drifts with DMA overshoot.
    pub frame_no: u32,
    /// Bus cycle, and matching audio-clock position, at the start of the current
    /// scanline. The APU runs on its own exact clock that the frame loop only
    /// advances once per line, so without these a mid-line event is attributed to
    /// a line boundary and lands up to 1232 cycles away from where it really is.
    pub line_cycle_base: u64,
    pub audio_line_base: u64,
    pub watch_addr: u32,
    /// Bytes covered by the watchpoint, starting at `watch_addr`. Defaults to 4.
    /// This is NOT cosmetic: a 4-byte window on 0x04000004 reports zero writes
    /// while DISPSTAT is visibly clobbered, because the stores are 32-bit and
    /// start at 0x04000000. Watch a block, then read the addresses back.
    pub watch_len: u32,
    /// When non-zero, only record watch hits made by this PC. A buffer that is
    /// written by several unrelated routines drowns the one you care about.
    pub watch_pc: u32,
    pub watch_hits: Vec<(u32, u32, u32, u32)>,
    /// Every write that hit the watch block, counted even after `watch_hits` has
    /// rolled. `watch_hits` keeps the MOST RECENT writes, because the interesting
    /// one is nearly always the last: an init sequence floods the first hundred.
    pub watch_total: u64,
    /// Debug counter: sound-FIFO DMA refills (4-word transfers) since boot.
    pub dbg_fifo_refills: u64,
    /// Debug counter: times the PPU raised the V-blank IRQ request (IF bit 0).
    pub dbg_vbl_raised: u64,
    /// Debug counter: IRQs actually taken by the CPU, indexed by IF bit.
    pub dbg_irq_src: [u64; 16],
}

#[derive(Clone, Copy, Default)]
struct Timer {
    reload: u16,
    counter: u16,
    control: u16,
    subcycle: u32,
}

/// Add `ticks` to a timer counter, reloading on overflow. Returns how many times
/// it overflowed (for cascade + IRQ).
fn timer_add(counter: &mut u16, reload: u16, ticks: u32) -> u32 {
    if ticks == 0 {
        return 0;
    }
    let val = *counter as u32 + ticks;
    if val <= 0xFFFF {
        *counter = val as u16;
        return 0;
    }
    let period = 0x1_0000 - reload as u32; // ticks per overflow after the first
    let extra = val - 0x1_0000;
    *counter = (reload as u32 + extra % period) as u16;
    1 + extra / period
}

/// Minimal HLE stand-in for the BIOS IRQ handler, installed when no real BIOS is
/// provided. It saves scratch registers, loads the user handler pointer from
/// [0x03FFFFFC] (mirror of 0x03007FFC), calls it, restores, and returns.
const HLE_IRQ: &[(usize, u32)] = &[
    (0x18, 0xEA00_0000), // b 0x20
    (0x20, 0xE92D_500F), // stmfd sp!, {r0-r3, r12, lr}
    (0x24, 0xE3A0_0301), // mov r0, #0x04000000
    (0x28, 0xE28F_E000), // add lr, pc, #0
    (0x2C, 0xE510_F004), // ldr pc, [r0, #-4]
    (0x30, 0xE8BD_500F), // ldmfd sp!, {r0-r3, r12, lr}
    (0x34, 0xE25E_F004), // subs pc, lr, #4
];

/// WAITCNT (0x4000204) decoded into cycle counts, recomputed whenever the
/// register is written and after a state load. Not serialized: it is derived
/// entirely from `io[0x204]`, which is, so the save-state format is unchanged.
#[derive(Clone, Copy)]
struct Waits {
    /// Cost of a 16-bit access to each ROM wait-state region, [WS0, WS1, WS2],
    /// non-sequential and sequential.
    rom_n: [u64; 3],
    rom_s: [u64; 3],
    sram: u64,
    /// WAITCNT bit 14. Games set it (Mario Kart writes 0x4497, Fire Emblem
    /// 0x45B4) and it is the difference between code from ROM costing a couple
    /// of cycles and costing one.
    prefetch: bool,
}

impl Default for Waits {
    fn default() -> Self {
        Self::from_reg(0)
    }
}

impl Waits {
    fn from_reg(w: u16) -> Self {
        // GBATEK: the wait values are cycle counts added to the 1-cycle access.
        const N: [u64; 4] = [4, 3, 2, 8];
        const S0: [u64; 2] = [2, 1];
        const S1: [u64; 2] = [4, 1];
        const S2: [u64; 2] = [8, 1];
        let w = w as usize;
        Waits {
            rom_n: [
                1 + N[(w >> 2) & 3],
                1 + N[(w >> 5) & 3],
                1 + N[(w >> 8) & 3],
            ],
            rom_s: [
                1 + S0[(w >> 4) & 1],
                1 + S1[(w >> 7) & 1],
                1 + S2[(w >> 10) & 1],
            ],
            sram: 1 + N[w & 3],
            prefetch: w & 0x4000 != 0,
        }
    }
}

/// Eight words of the bundled open BIOS, and what they become.
///
/// Normmatt's IntrWait keeps the I/O base in r2 and parks 0x208 in r12. Real
/// hardware's BIOS keeps the base in r12, and so does mGBA's hand-written HLE
/// BIOS, so games were written expecting it there: Elevator Action Old and New
/// ends its interrupt handler with `strneh r0, [r12, #-8]`, which is the BIOS
/// interrupt check flags at 0x03FFFFF8 only if r12 holds 0x04000000. With 0x208
/// in it the store lands on 0x200 instead, which is BIOS ROM and read-only, so
/// the flag never appears and IntrWait never returns. The cartridge spins in the
/// BIOS forever while its handler keeps running and looking perfectly healthy.
///
/// The second, same shape one level down: the loop keeps the flag POINTER in r4,
/// and the BIOS interrupt prologue saves only {r0-r3, r12, lr}, so a game handler
/// that clobbers r4 leaves IntrWait reading a ROM address forever. Harobots does
/// exactly that.
///
/// The repair swaps which register holds which constant. r2 and r12 trade places
/// in the halt loop, and the three stores that index off them swap operands to
/// match. Instruction for instruction it is the same routine: same control flow,
/// same cycle count, same register footprint on return. Only the value a halted
/// IntrWait leaves in r12 changes, from 0x208 to 0x04000000.
///
/// Rewriting the routine outright was tried first and rejected. A faithful
/// reimplementation of IntrWait fixed the same games and broke Tennis no
/// Ouji-sama 2004 in both regional builds, which boots on Normmatt's version
/// and on a real BIOS dump. Swapping two registers cannot do that.
///
/// `(offset, expected, replacement)`.
#[rustfmt::skip]
const OPEN_BIOS_R12: &[(usize, u32, u32)] = &[
    (0x488, 0xE3A0_2301, 0xE3A0_C301), // mov r2, #0x04000000   -> mov r12, #0x04000000
    (0x48C, 0xE3A0_CF82, 0xE3A0_2F82), // mov r12, #0x208        -> mov r2, #0x208
    (0x49C, 0xE5C2_5301, 0xE5CC_5301), // strb r5, [r2, #0x301]  -> strb r5, [r12, #0x301]
    (0x4A0, 0xE182_70BC, 0xE18C_70B2), // strh r7, [r2, r12]     -> strh r7, [r12, r2]
    (0x4BC, 0xE182_60BC, 0xE18C_60B2), // strh r6, [r2, r12]     -> strh r6, [r12, r2]
    (0x4CC, 0xE182_60BC, 0xE18C_60B2), // strh r6, [r2, r12]     -> strh r6, [r12, r2]
    // And the flag pointer itself, for the same reason one level down. The BIOS
    // interrupt prologue saves {r0-r3, r12, lr} and NOTHING else, so a game's
    // handler is free to clobber r4. Normmatt's halt loop keeps the check-flag
    // pointer in r4 across the halt, so one such handler leaves it reading some
    // address in ROM forever. Harobots does exactly that: measured mid-hang, r4
    // held 0x08003205 where the loop needed 0x03FFFFFF. Re-based on r12, which
    // survives, and which is 8 above the flag word anyway.
    (0x4A4, 0xE154_30B7, 0xE15C_30B8), // ldrh r3, [r4, #-7]     -> ldrh r3, [r12, #-8]
    (0x4B8, 0xE144_30B7, 0xE14C_30B8), // strh r3, [r4, #-7]     -> strh r3, [r12, #-8]
];

/// Where the RL-decompression repair lands. The bundled open BIOS stops using
/// its image at 0x243C and the remaining 7 KB is zero, so there is room to put
/// four instructions somewhere the original never executes.
const RL_STUB: usize = 0x2440;

/// `RLUnCompWram` (SWI 0x14) reads its header with `ldr r2, [r0], #4` and does
/// not word-align `r0` first. On an ARM7 a misaligned LDR ROTATES the word it
/// loaded, so a source that is not a multiple of four yields a header with the
/// right type nibble and a garbage size.
///
/// Mortal Kombat Deadly Alliance passes `src % 4 == 2` on every one of its
/// twenty calls, every time with a perfectly good header sitting at that byte
/// offset. Measured on the Europe build: the block is 2048 bytes, and the
/// rotation turns that into 6,579,200. The decompressor then writes until it
/// has filled EWRAM several times over and never returns, which is the Midway
/// logo hanging forever at a 99% BIOS share.
///
/// The fix is one instruction's worth of behaviour, and the image itself says
/// so: `RLUnCompVram` next door at 0x0FBC does `bic r0, r0, #3` before the very
/// same load, and only the Wram entry is missing it. mGBA agrees and is the
/// reference here, since it runs these games and we did not:
///
/// ```text
/// remaining = (load32(source & 0xFFFFFFFC) & 0xFFFFFF00) >> 8;   // masked
/// source += 4;                                                   // NOT masked
/// ```
///
/// Note which pointer gets masked. The header is read from the aligned address,
/// but the source advances from the ORIGINAL, so the compressed stream still
/// starts at `src + 4`. Masking r0 outright shifts the whole stream two bytes
/// and corrupts everything it decodes. The stub keeps them separate, and for an
/// already-aligned source it is behaviour-identical to the instruction it
/// replaces.
///
/// `(offset, expected, replacement)`.
#[rustfmt::skip]
const OPEN_BIOS_RL: &[(usize, u32, u32)] = &[
    // ldr r2, [r0], #4  ->  b RL_STUB
    (0x0F18, 0xE490_2004, 0xEA00_0548),
    // The stub, into guaranteed-zero space.
    (RL_STUB,        0, 0xE3C0_2003), // bic r2, r0, #3     (aligned copy for the header)
    (RL_STUB + 0x4,  0, 0xE592_2000), // ldr r2, [r2]       (unrotated, so the size is real)
    (RL_STUB + 0x8,  0, 0xE280_0004), // add r0, r0, #4     (advance from the ORIGINAL src)
    (RL_STUB + 0xC,  0, 0xEAFF_FAB2), // b 0x0F1C           (back into the routine)
];

/// Apply [`OPEN_BIOS_RL`], under the same all-or-nothing rule as the r12
/// repair: a real BIOS dump, a different build of the open BIOS or the HLE stub
/// is left untouched, and the four stub words must be zero before anything is
/// written over them.
fn align_the_rl_header_read(bios: &mut [u8]) {
    let word = |b: &[u8], o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    if !OPEN_BIOS_RL.iter().all(|&(o, want, _)| o + 4 <= bios.len() && word(bios, o) == want) {
        return;
    }
    for &(o, _, patched) in OPEN_BIOS_RL {
        bios[o..o + 4].copy_from_slice(&patched.to_le_bytes());
    }
}

/// Apply [`OPEN_BIOS_R12`], but only to a BIOS image that is byte-for-byte the
/// one it was measured against.
///
/// Every word is checked before any word is written, so a real BIOS dump, a
/// different build of the open BIOS, or the HLE stub is left exactly as it came
/// in. The image on disk is never touched; this edits the copy in memory.
fn hold_io_base_in_r12(bios: &mut [u8]) {
    let word = |b: &[u8], o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);
    if !OPEN_BIOS_R12.iter().all(|&(o, want, _)| o + 4 <= bios.len() && word(bios, o) == want) {
        return;
    }
    for &(o, _, patched) in OPEN_BIOS_R12 {
        bios[o..o + 4].copy_from_slice(&patched.to_le_bytes());
    }
}

impl GbaBus {
    pub fn new(rom: Vec<u8>, bios: Vec<u8>) -> Self {
        let mut b = vec![0u8; 16 * 1024];
        let n = bios.len().min(b.len());
        b[..n].copy_from_slice(&bios[..n]);
        if bios.is_empty() {
            for &(off, word) in HLE_IRQ {
                b[off..off + 4].copy_from_slice(&word.to_le_bytes());
            }
        }
        hold_io_base_in_r12(&mut b);
        align_the_rl_header_read(&mut b);
        let save = Save::detect(&rom);
        let sensors = crate::sensor::Sensors::new(&rom);
        // GBA_RFU=0 detaches the adapter from a cart that has one, and
        // GBA_RFU=1 attaches one to a cart that does not. Both directions are
        // needed to tell "the game behaves differently with an adapter" from
        // "the game behaves the same either way", which is the only way to
        // know detection actually worked.
        let attached = match std::env::var("GBA_RFU").as_deref() {
            Ok("0") => false,
            Ok(_) => true,
            Err(_) => crate::rfu::detect(&rom),
        };
        let rfu = attached.then(crate::rfu::Rfu::new);
        GbaBus {
            sensors,
            rfu,
            cable: None,
            cable_busy: false,
            cable_words: None,
            cable_transfers: 0,
            cable_failures: 0,
            cable_late: 0,
            cable_own: 0,
            cable_wordsum: 0xcbf2_9ce4_8422_2325, // FNV-1a offset basis
            cable_mark_at: 0,
            cable_mark_sum: 0xcbf2_9ce4_8422_2325,
            cable_window_sum: 0xcbf2_9ce4_8422_2325,
            cable_trace: Vec::new(),
            serial_cycles: 0,
            serial_pending: 0,
            bios: b.into_boxed_slice(),
            exec_in_bios: false,
            last_fetch: 0,
            last_fetch_thumb: false,
            bios_entry_irq: false,
            bios_prefetch: 0xE129_F000,
            ewram: vec![0; 256 * 1024].into_boxed_slice(),
            iwram: vec![0; 32 * 1024].into_boxed_slice(),
            rom: rom.into_boxed_slice(),
            save,
            ppu: Ppu::new(),
            apu: Apu::new(),
            io: vec![0; 0x400].into_boxed_slice(),
            waits: Waits::default(),
            prefetch_credit: 0,
            code_lo: 0,
            code_span: 0,
            code_win: CodeWin::Rom,
            keyinput: 0x03FF,
            ie: 0,
            if_: 0,
            ime: 0,
            timers: [Timer::default(); 4],
            timer_cycles: 0,
            dma_src: [0; 4],
            dma_dst: [0; 4],
            dma_count: [0; 4],
            dma_active: false,
            dma_pending: 0,
            cycles: 0,
            halted: false,
            cur_pc: 0,
            cur_mode: 0,
            data_break: false,
            frame_no: 0,
            line_cycle_base: 0,
            audio_line_base: 0,
            watch_addr: 0,
            watch_len: 4,
            watch_pc: 0,
            watch_hits: Vec::new(),
            watch_total: 0,
            dbg_fifo_refills: 0,
            dbg_vbl_raised: 0,
            dbg_irq_src: [0; 16],
        }
    }

    pub fn serialize(&self, w: &mut crate::state::Writer) {
        w.u16(self.keyinput);
        w.u16(self.ie);
        w.u16(self.if_);
        w.u16(self.ime);
        w.u64(self.cycles);
        w.u64(self.timer_cycles);
        w.bool(self.halted);
        for i in 0..4 {
            w.u32(self.dma_src[i]);
            w.u32(self.dma_dst[i]);
            w.u32(self.dma_count[i]);
        }
        for t in &self.timers {
            w.u16(t.reload);
            w.u16(t.counter);
            w.u16(t.control);
            w.u32(t.subcycle);
        }
        w.bytes(&self.ewram);
        w.bytes(&self.iwram);
        w.bytes(&self.io);
        self.save.serialize(w);
        self.ppu.serialize(w);
        self.apu.serialize(w);
        self.sensors.serialize(w);
        // The bus state behind the BIOS read protection. Small, but a state
        // that omits it does not resume faithfully: a game that reads region 0
        // before the next BIOS entry sees the boot-time value instead of the
        // one it actually had. Babar to the Rescue is the measured case, and it
        // restarted into the BIOS boot animation on load.
        w.u32(self.bios_prefetch);
        w.u32(self.last_fetch);
        w.bool(self.last_fetch_thumb);
        w.bool(self.exec_in_bios);
        w.bool(self.bios_entry_irq);
        // The cartridge clock, appended last so a state written before it
        // existed still loads (the reader gates on remaining()).
        self.sensors.serialize_rtc(w);
    }

    pub fn deserialize(&mut self, r: &mut crate::state::Reader) {
        self.keyinput = r.u16();
        self.ie = r.u16();
        self.if_ = r.u16();
        self.ime = r.u16();
        self.cycles = r.u64();
        self.timer_cycles = r.u64();
        self.halted = r.bool();
        for i in 0..4 {
            self.dma_src[i] = r.u32();
            self.dma_dst[i] = r.u32();
            self.dma_count[i] = r.u32();
        }
        for t in &mut self.timers {
            t.reload = r.u16();
            t.counter = r.u16();
            t.control = r.u16();
            t.subcycle = r.u32();
        }
        r.bytes_into(&mut self.ewram);
        r.bytes_into(&mut self.iwram);
        r.bytes_into(&mut self.io);
        self.waits = Waits::from_reg(self.io_u16(0x204)); // derived, not stored
        self.save.deserialize(r);
        self.ppu.deserialize(r);
        // The APU block was appended after the state format shipped; only read it
        // when the blob actually carries it, so pre-APU states still load.
        if r.remaining() > 8 {
            self.apu.deserialize(r);
        }
        // Appended after the format shipped, like the APU block before it, so a
        // state written by an older build still loads: the sensors simply start
        // from their reset values, which is one frame of staleness at worst.
        if r.remaining() >= 17 {
            self.sensors.deserialize(r);
        }
        // Appended after the sensors, same rule: a state from a build without
        // it still loads, and the bus simply starts from its reset values.
        if r.remaining() >= 11 {
            self.bios_prefetch = r.u32();
            self.last_fetch = r.u32();
            self.last_fetch_thumb = r.bool();
            self.exec_in_bios = r.bool();
            self.bios_entry_irq = r.bool();
        }
        // The cartridge clock, appended last under the same rule.
        if r.remaining() >= 25 {
            self.sensors.deserialize_rtc(r);
        }
    }

    // --- Timers ---------------------------------------------------------------

    /// Advance all four timers by the cycles elapsed since the last call,
    /// handling prescalers, cascade, overflow reload and timer IRQs.
    pub fn step_timers(&mut self) {
        let delta = (self.cycles - self.timer_cycles) as u32;
        self.timer_cycles = self.cycles;
        let mut prev_overflows = 0u32;
        for ch in 0..4 {
            let ctrl = self.timers[ch].control;
            if ctrl & 0x80 == 0 {
                prev_overflows = 0;
                continue;
            }
            let ticks = if ch > 0 && ctrl & 4 != 0 {
                prev_overflows // cascade / count-up
            } else {
                let period = [1u32, 64, 256, 1024][(ctrl & 3) as usize];
                self.timers[ch].subcycle += delta;
                let t = self.timers[ch].subcycle / period;
                self.timers[ch].subcycle %= period;
                t
            };
            let reload = self.timers[ch].reload;
            let overflows = timer_add(&mut self.timers[ch].counter, reload, ticks);
            if overflows > 0 && ctrl & 0x40 != 0 {
                self.if_ |= 1 << (3 + ch);
            }
            prev_overflows = overflows;
        }
    }

    /// Debug: a DMA channel's live (latched) source and its control-register
    /// snapshot, plus the current per-channel running source. For diagnosing the
    /// sound FIFO DMA (channels 1/2).
    pub fn dma_dbg(&self, ch: usize) -> (u32, u32, u16, u32) {
        let base = 0xB0 + ch as u32 * 12;
        (self.io_u32(base) & 0x0FFF_FFFF, self.io_u32(base + 4) & 0x0FFF_FFFF, self.io_u16(base + 10), self.dma_src[ch])
    }

    /// The overflow period (in system cycles) of timer `ch`, or `None` if it is
    /// stopped or in cascade mode. Used by the APU to schedule Direct Sound FIFO
    /// pops at the audio rate the game programmed.
    pub fn ds_timer_period(&self, ch: usize) -> Option<u32> {
        let t = &self.timers[ch];
        if t.control & 0x80 == 0 || (ch > 0 && t.control & 4 != 0) {
            return None;
        }
        let prescaler = [1u32, 64, 256, 1024][(t.control & 3) as usize];
        Some((0x1_0000 - t.reload as u32) * prescaler)
    }

    // --- DMA ------------------------------------------------------------------

    fn io_u16(&self, off: u32) -> u16 {
        u16::from_le_bytes([self.io[off as usize], self.io[off as usize + 1]])
    }
    fn io_u32(&self, off: u32) -> u32 {
        let o = off as usize;
        u32::from_le_bytes([self.io[o], self.io[o + 1], self.io[o + 2], self.io[o + 3]])
    }

    /// Run enabled DMA channels whose start-timing matches (1=VBlank, 2=HBlank).
    pub fn trigger_dma(&mut self, timing: u16) {
        for ch in 0..4 {
            let control = self.io_u16(0xB0 + ch as u32 * 12 + 10);
            if control & 0x8000 != 0 && (control >> 12) & 3 == timing {
                self.run_dma_guarded(ch);
            }
        }
    }

    /// Run a channel, then drain any channel that channel enabled.
    ///
    /// A transfer writes through the normal bus, so a DMA whose destination is
    /// the DMA control block re-enters `start_dma` and, unguarded, recurses once
    /// per transferred word. Hardware cannot do that: DMA is a state machine with
    /// four fixed-priority channels, and enabling a channel from inside a running
    /// transfer just schedules it. Unguarded this overflowed the stack outright,
    /// which is worse than a hang because a stack overflow aborts the process and
    /// `catch_unwind` cannot see it: it silently truncated the 6118-ROM sweep at
    /// whatever ROM it happened to reach (263 ROMs never ran). Kaisertal (Europe)
    /// (Demo) is the cheap repro.
    fn run_dma_guarded(&mut self, ch: usize) {
        if self.dma_active {
            self.dma_pending |= 1 << ch;
            return;
        }
        self.dma_active = true;
        self.run_dma(ch);
        // Lowest channel number first, matching hardware DMA priority. Bounded:
        // a channel whose destination is its OWN control register re-arms itself
        // every pass, so an unbounded drain would trade the stack overflow for a
        // hang inside a single store. Hardware spends real cycles per transfer
        // and the rest of the machine keeps advancing; we cannot yield from here,
        // so we cap the drain and let the frame loop pick the channel up again on
        // its next timing trigger.
        let mut drained = 0;
        while self.dma_pending != 0 && drained < 32 {
            let c = self.dma_pending.trailing_zeros() as usize;
            self.dma_pending &= !(1 << c);
            self.run_dma(c);
            drained += 1;
        }
        self.dma_pending = 0;
        self.dma_active = false;
    }

    /// Cycles one DMA bus access costs, per region and width. The same
    /// WAITCNT-derived costs the CPU pays, minus the prefetch unit: DMA owns
    /// the bus outright, there is no prefetcher running ahead of it and the
    /// CPU core is stalled for the duration, so the raw N/S cost is the whole
    /// story. EWRAM, PALRAM and VRAM have no N/S distinction.
    fn dma_access_cycles(&self, addr: u32, word: bool, seq: bool) -> u64 {
        match (addr >> 24) & 0xF {
            0x2 => if word { 6 } else { 3 },       // EWRAM (16-bit bus, 2 WS)
            0x5 | 0x6 => if word { 2 } else { 1 }, // PALRAM / VRAM (16-bit bus)
            region @ 0x8..=0xD => {
                let ws = ((region - 8) >> 1) as usize;
                let halves = if word { 2 } else { 1 };
                let mut total = 0;
                for i in 0..halves {
                    // Only the first half of a 32-bit access can be
                    // non-sequential; the second always follows on.
                    total += if seq || i > 0 { self.waits.rom_s[ws] } else { self.waits.rom_n[ws] };
                }
                total
            }
            0xE | 0xF => self.waits.sram,
            _ => 1, // BIOS / IWRAM / I/O / OAM
        }
    }

    fn run_dma(&mut self, ch: usize) {
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        let word = control & 0x400 != 0;
        let size = if word { 4u32 } else { 2 };
        let dst_ctrl = (control >> 5) & 3;
        let src_ctrl = (control >> 7) & 3;
        let mut src = self.dma_src[ch];
        let mut dst = self.dma_dst[ch];
        let step = |a: u32, c: u16| match c {
            1 => a.wrapping_sub(size),
            2 => a,
            _ => a.wrapping_add(size),
        };
        // EEPROM lives in region 0xD and is driven bit-serially by DMA; the
        // transfer length tells the chip its command and address width.
        let ee_src = self.save.is_eeprom() && (src >> 24) == 0xD;
        let ee_dst = self.save.is_eeprom() && (dst >> 24) == 0xD;
        if ee_src || ee_dst {
            self.save.eeprom_set_dma_len(self.dma_count[ch]);
        }
        for n in 0..self.dma_count[ch] {
            if ee_src || ee_dst {
                let v = if ee_src {
                    self.save.eeprom_read_bit() as u32
                } else {
                    self.read16_raw(src & !1) as u32
                };
                if ee_dst {
                    self.save.eeprom_write_bit(v as u8);
                } else {
                    self.write(dst & !1, v, 2);
                }
            } else if word {
                let v = self.read32_raw(src & !3);
                self.write(dst & !3, v, 4);
            } else {
                let v = self.read16_raw(src & !1) as u32;
                self.write(dst & !1, v, 2);
            }
            // Hardware DMA timing is 2N + 2(n-1)S + xI: the first transfer
            // pays the non-sequential cost on both ends, every later one the
            // sequential cost. The old model charged a flat `size` cycles per
            // transfer regardless of region, which made big EWRAM/ROM blits
            // run at roughly double speed. GTA Advance's cutscene streamer is
            // phase-locked to those blits: undercharging them let the main
            // pass stage its next code chunk AFTER the per-frame V-count
            // promote instead of before it, so a promote stayed pending across
            // the scene switch and stamped a stale 64-byte head over the
            // freshly loaded rasteriser at 0x03000100.
            let seq = n > 0;
            self.cycles += self.dma_access_cycles(src, word, seq)
                + self.dma_access_cycles(dst, word, seq);
            src = step(src, src_ctrl);
            dst = step(dst, dst_ctrl);
        }
        // Internal overhead: 2I, or 4I when both ends sit on the Game Pak bus.
        self.cycles += if (8..=0xD).contains(&((src >> 24) & 0xF))
            && (8..=0xD).contains(&((dst >> 24) & 0xF)) { 4 } else { 2 };
        // A DMA that used the cart bus leaves the CPU's prefetch buffer empty.
        if matches!((src >> 24) & 0xF, 0x8..=0xF) || matches!((dst >> 24) & 0xF, 0x8..=0xF) {
            self.prefetch_credit = 0;
        }
        self.data_break = true;
        self.dma_src[ch] = src;
        if control & 0x4000 != 0 {
            self.if_ |= 1 << (8 + ch); // DMA complete IRQ
        }
        let repeat = control & 0x200 != 0 && (control >> 12) & 3 != 0;
        if repeat {
            let cl = self.io_u16(base + 8) as u32;
            let max = if ch == 3 { 0x1_0000 } else { 0x4000 };
            self.dma_count[ch] = if cl == 0 { max } else { cl };
            self.dma_dst[ch] = if dst_ctrl == 3 {
                self.io_u32(base + 4) & 0x0FFF_FFFF // dest reload
            } else {
                dst
            };
        } else {
            self.dma_dst[ch] = dst;
            let cleared = control & 0x7FFF; // clear the enable bit
            self.io[base as usize + 10] = cleared as u8;
            self.io[base as usize + 11] = (cleared >> 8) as u8;
        }
    }

    /// Latch a DMA channel's registers when its enable bit is set, running it
    /// immediately if its timing is "immediate".
    fn start_dma(&mut self, ch: usize) {
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        if control & 0x8000 == 0 {
            return;
        }
        // A sound-FIFO channel's re-arm moves the read pointer back to the start
        // of the game's PCM buffer, so the APU has to be caught up to this exact
        // cycle first or refills land on the wrong side of the boundary.
        if (ch == 1 || ch == 2) && (control >> 12) & 3 == 3 {
            self.catch_up_audio(ch);
        }
        self.dma_src[ch] = self.io_u32(base) & 0x0FFF_FFFF;
        self.dma_dst[ch] = self.io_u32(base + 4) & 0x0FFF_FFFF;
        let cl = self.io_u16(base + 8) as u32;
        let max = if ch == 3 { 0x1_0000 } else { 0x4000 };
        self.dma_count[ch] = if cl == 0 { max } else { cl };
        if (control >> 12) & 3 == 0 {
            self.run_dma_guarded(ch); // immediate
        }
    }

    /// Advance the APU to the exact cycle the CPU has reached inside this
    /// scanline, and service any FIFO that has drained by then.
    ///
    /// The frame loop normally advances the audio clock once per line, after the
    /// line's CPU work, so every mid-line event is attributed to the line
    /// boundary. For a sound-FIFO DMA re-arm that is not good enough: the re-arm
    /// resets the source pointer, so the question of which FIFO refills fall
    /// before it and which after decides where in the game's PCM buffer the next
    /// read lands. A whole line is 1.3 Direct Sound samples, so rounding the
    /// re-arm to a line boundary moves the read by one refill, which is 16 bytes
    /// past the end of a buffer with no slack. See
    /// `the_sound_fifo_is_serviced_at_the_cycle_the_dma_is_rearmed`.
    ///
    /// Only `ch` is serviced, never the other sound channel. Soccer Kid re-arms
    /// both of them back to back, and topping up the second one here would read
    /// through the stale source pointer it is about to replace.
    fn catch_up_audio(&mut self, ch: usize) {
        let within = (self.cycles.saturating_sub(self.line_cycle_base))
            .min(crate::ppu::CYCLES_PER_LINE as u64);
        let target = self.audio_line_base + within;
        let periods = [self.ds_timer_period(0), self.ds_timer_period(1)];
        self.apu.generate(periods, target);
        self.refill_fifo(ch);
    }

    /// Top up the Direct Sound FIFOs from their sound DMA channels. Hardware
    /// requests a refill when a FIFO drops to <= 4 words (16 bytes); DMA1/DMA2 in
    /// "special" timing service FIFO_A/FIFO_B respectively, always 4 words wide.
    pub fn refill_fifos(&mut self) {
        for ch in [1usize, 2] {
            self.refill_fifo(ch);
        }
    }

    /// Top up one Direct Sound FIFO if its channel is streaming and it has
    /// drained to the half-empty mark hardware requests a transfer at.
    fn refill_fifo(&mut self, ch: usize) {
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        if control & 0x8000 == 0 || (control >> 12) & 3 != 3 {
            return; // channel off, or not sound-FIFO timing
        }
        let dst = self.io_u32(base + 4) & 0x07FF_FFFF;
        let level = match dst {
            0x0400_00A0 => self.apu.fifo_a_len(),
            0x0400_00A4 => self.apu.fifo_b_len(),
            _ => return,
        };
        if level <= 16 {
            self.run_sound_dma(ch, dst);
        }
    }

    /// Transfer one FIFO request: 4 words from the (incrementing) source into the
    /// fixed FIFO port. The word count and destination-fixed behaviour are forced
    /// by the hardware regardless of the channel's programmed count/dest-control.
    fn run_sound_dma(&mut self, ch: usize, dst: u32) {
        self.dbg_fifo_refills += 1;
        let base = 0xB0 + ch as u32 * 12;
        let control = self.io_u16(base + 10);
        let src_ctrl = (control >> 7) & 3;
        let mut src = self.dma_src[ch];
        for n in 0..4 {
            let v = self.read32_raw(src & !3);
            self.write(dst, v, 4);
            // Same real bus costs as any other DMA (the destination is the
            // FIFO port, a 1-cycle I/O access; the source pays its region).
            self.cycles += self.dma_access_cycles(src, true, n > 0) + 1;
            src = match src_ctrl {
                1 => src.wrapping_sub(4),
                2 => src,
                _ => src.wrapping_add(4),
            };
        }
        self.cycles += 2; // internal DMA overhead
        if matches!((src >> 24) & 0xF, 0x8..=0xF) {
            self.prefetch_credit = 0;
        }
        self.data_break = true;
        self.dma_src[ch] = src;
        if control & 0x4000 != 0 {
            self.if_ |= 1 << (8 + ch); // DMA-complete IRQ
        }
    }

    /// True when the CPU should take an IRQ: master-enabled and some enabled
    /// source is requesting. The CPU still gates on its own CPSR I bit.
    pub fn irq_pending(&self) -> bool {
        self.ime & 1 != 0 && (self.ie & self.if_) != 0
    }

    /// Raise the LCD interrupts for `line` according to DISPSTAT's enable bits.
    pub fn raise_ppu_irqs(&mut self, line: u16) {
        let stat = self.ppu.dispstat();
        if line == 160 && stat & 0x08 != 0 {
            self.if_ |= 1 << 0; // V-blank
            self.dbg_vbl_raised += 1;
        }
        if stat & 0x10 != 0 {
            self.if_ |= 1 << 1; // H-blank (every line, approximate timing)
        }
        if line == stat >> 8 && stat & 0x20 != 0 {
            self.if_ |= 1 << 2; // V-counter match
        }
    }

    /// Charge one CPU bus access, deciding fetch-vs-data from the address.
    /// A data access on the Game Pak bus aborts the prefetch unit, and the
    /// opcode fetch that follows it restarts at the non-sequential cost; that
    /// is a documented consequence of the cart bus having a single owner.
    /// The opcode fetch is recognised as the access at (or pipeline-adjacent
    /// to) the address `set_fetch_pc` recorded this step.
    fn charge(&mut self, addr: u32, word: bool, a: Access) {
        let cart = matches!((addr >> 24) & 0xF, 0x8..=0xF);
        let is_fetch = addr.wrapping_sub(self.last_fetch) <= 4;
        let mut seq = a == Access::Seq;
        if !is_fetch {
            if cart {
                // Data on the cart bus also kills the prefetcher outright.
                self.prefetch_credit = 0;
            }
            self.data_break = true;
        } else if self.data_break {
            seq = false;
            self.data_break = false;
        }
        self.cycles += self.access_cycles(addr, word, seq);
    }

    /// Cycles for one access, honouring WAITCNT and whether the access is
    /// sequential.
    ///
    /// This used to be a flat 5 cycles for every 16-bit ROM access and 8 for
    /// every 32-bit one, with the `Access` the CPU already passes thrown away.
    /// That charges a sequential opcode fetch the price of a random one, which
    /// is wrong in itself whatever it nets out to.
    ///
    /// Do NOT read it as a throughput win. Measured on Mario Kart Super Circuit
    /// over 1800 frames, instructions per frame are 41866 before and 41755
    /// after under the open BIOS, and 40350 against 40284 on a direct boot: the
    /// cheaper sequential fetches and the newly charged load/store N and I
    /// cycles very nearly cancel. Host wall-clock is unchanged too, 1662 ms
    /// against 1658 ms. What changes is the SHAPE of the timing, which is what
    /// GTA Advance's teardown race turns on, not the total.
    ///
    /// ROM is one 16-bit bus, so a 32-bit access is two of them: the first pays
    /// N or S depending on how we arrived, the second is always sequential.
    fn access_cycles(&mut self, addr: u32, word: bool, seq: bool) -> u64 {
        match (addr >> 24) & 0xF {
            0x2 => if word { 6 } else { 3 },       // EWRAM (16-bit bus, 2 WS)
            0x5 | 0x6 => if word { 2 } else { 1 }, // PALRAM / VRAM (16-bit bus)
            region @ 0x8..=0xD => {
                // 8/9 = WS0, A/B = WS1, C/D = WS2.
                let ws = ((region - 8) >> 1) as usize;
                let halves = if word { 2 } else { 1 };
                let mut total = 0;
                for i in 0..halves {
                    // Only the first half of a 32-bit access can be
                    // non-sequential; the second always follows on.
                    let sequential = seq || i > 0;
                    total += self.rom_half(ws, sequential);
                }
                total
            }
            0xE | 0xF => {
                self.prefetch_credit = 0;
                self.waits.sram
            }
            _ => {
                // The ROM bus is idle during this access, so the prefetch unit
                // gets to run. This is most of where prefetching pays: code in
                // ROM that touches IWRAM or I/O buys its next opcodes for free.
                self.credit_prefetch(1);
                1 // BIOS / IWRAM / I/O / OAM
            }
        }
    }

    /// One 16-bit ROM access, through the prefetch unit.
    ///
    /// The unit holds 8 halfwords and fills at the sequential rate whenever the
    /// CPU is not using the Game Pak bus. So a sequential fetch is nearly free
    /// if the unit has had time to run ahead, and costs the full sequential
    /// wait if it has not. Modelling it as a flat 1 cycle instead would be too
    /// generous: in a tight loop of sequential code with no spare cycles the
    /// prefetcher cannot fill faster than it is drained, and real hardware gets
    /// no benefit there either.
    ///
    /// A non-sequential access is a jump, which empties the buffer.
    fn rom_half(&mut self, ws: usize, seq: bool) -> u64 {
        let s = self.waits.rom_s[ws];
        if !seq || !self.waits.prefetch {
            self.prefetch_credit = 0;
            return if seq { s } else { self.waits.rom_n[ws] };
        }
        if self.prefetch_credit >= s {
            // Already fetched ahead: hand it over in one cycle.
            self.prefetch_credit -= s;
            1
        } else {
            self.prefetch_credit = 0;
            s
        }
    }

    /// Give the prefetch unit `n` cycles of bus time, capped at the 8-halfword
    /// buffer so an idle stretch cannot bank unlimited free fetches.
    fn credit_prefetch(&mut self, n: u64) {
        if !self.waits.prefetch {
            return;
        }
        let cap = 8 * self.waits.rom_s[0];
        self.prefetch_credit = (self.prefetch_credit + n).min(cap);
    }

    // --- Reads: composed little-endian from raw bytes (no side effects) --------

    /// What is sitting on the bus: the last instruction the CPU fetched. An
    /// unmapped address reads this back. A Thumb fetch only drove sixteen
    /// lines, so the halfword appears in both halves of the word, which is the
    /// behaviour mGBA models in `GBALoadBad` for every region but IWRAM, OAM
    /// and the BIOS.
    fn open_bus(&self) -> u32 {
        let a = self.last_fetch;
        // Executing in unmapped space already (a runaway PC): there is nothing
        // to report and reading it back here would recurse.
        if (a >> 24) & 0xF == 0 && a >= 0x4000 {
            return self.bios_prefetch;
        }
        if self.last_fetch_thumb {
            let h = self.read16_raw(a & !1) as u32;
            h | (h << 16)
        } else {
            self.read32_raw(a & !3)
        }
    }

    fn read8_raw(&self, addr: u32) -> u8 {
        match (addr >> 24) & 0xF {
            // Above the 16 KB BIOS, region 0 is simply NOT MAPPED, and unmapped
            // is open bus: whatever the CPU last put on it. It is emphatically
            // not the BIOS, neither mirrored into it (which is what this used
            // to do) nor the BIOS's own stale prefetch (which is what the first
            // cut of the read protection did). Mario vs Donkey Kong reads
            // 0x0000F026 and Harry Potter Quidditch 0x00005498, and handing
            // either of them a BIOS value hangs the game.
            0x0 if addr >= 0x4000 => (self.open_bus() >> (8 * (addr & 3))) as u8,
            // Inside the BIOS, read-protected: see `set_fetch_pc`.
            0x0 if !self.exec_in_bios => (self.bios_prefetch >> (8 * (addr & 3))) as u8,
            0x0 => *self.bios.get((addr & 0x3FFF) as usize).unwrap_or(&0),
            0x2 => self.ewram[(addr & 0x3_FFFF) as usize],
            0x3 => self.iwram[(addr & 0x7FFF) as usize],
            0x4 => self.io_read8(addr),
            0x5 => self.ppu.read_pal8(addr),
            0x6 => self.ppu.read_vram8(addr),
            0x7 => self.ppu.read_oam8(addr),
            0x8..=0xD => {
                let o = (addr & 0x01FF_FFFF) as usize;
                // GPIO half-words, when the cart has the hardware AND the game
                // has switched the port to readable. Otherwise this falls
                // through to ROM, which is zero-filled here, exactly as the
                // write-only mode is specified to read back.
                if (0xC4..0xCA).contains(&o) {
                    if let Some(v) = self.sensors.gpio_read((o & !1) as u32) {
                        return if o & 1 == 0 { v as u8 } else { (v >> 8) as u8 };
                    }
                }
                *self.rom.get(o).unwrap_or(&0)
            }
            // The tilt ADC sits in the top half of the SRAM window on the carts
            // that have it; those all save to EEPROM, so nothing is displaced.
            0xE | 0xF => match self.sensors.tilt_read(addr) {
                Some(v) => v,
                None => self.save.read(addr),
            },
            _ => 0,
        }
    }

    /// Fast path for the large linear regions (BIOS/EWRAM/IWRAM/ROM): the region
    /// slice and the masked offset. `None` for the small side-effect regions
    /// (I/O, PAL/VRAM/OAM, SRAM), which stay on the byte-compose path.
    #[inline]
    fn linear_region(&self, addr: u32) -> Option<(&[u8], usize)> {
        match (addr >> 24) & 0xF {
            // Only while executing inside it; otherwise the byte path below
            // synthesises the protected value.
            0x0 if self.exec_in_bios && addr < 0x4000 => Some((&self.bios, (addr & 0x3FFF) as usize)),
            0x2 => Some((&self.ewram, (addr & 0x3_FFFF) as usize)),
            0x3 => Some((&self.iwram, (addr & 0x7FFF) as usize)),
            // The GPIO port lives inside the ROM window at 0x080000C4..C9, so
            // a cart that has one must drop off the linear fast path there.
            // Gated on the cart actually having the hardware, so every other
            // game pays a single predictable compare.
            0x8..=0xD if self.sensors.has_gpio() && (addr & 0x01FF_FFFF) < 0x100 => None,
            0x8..=0xD => Some((&self.rom, (addr & 0x01FF_FFFF) as usize)),
            _ => None,
        }
    }

    /// Read a halfword through the fetch window, or `None` if the address is
    /// outside it. The unsigned subtract makes one compare do both the region
    /// test and the bounds test.
    #[inline]
    fn code_fetch16(&self, addr: u32) -> Option<u16> {
        let o = addr.wrapping_sub(self.code_lo);
        if o >= self.code_span {
            return None;
        }
        let c = self.code_slice().get(o as usize..o as usize + 2)?;
        Some(u16::from_le_bytes([c[0], c[1]]))
    }

    /// Word companion to [`GbaBus::code_fetch16`].
    #[inline]
    fn code_fetch32(&self, addr: u32) -> Option<u32> {
        let o = addr.wrapping_sub(self.code_lo);
        if o >= self.code_span {
            return None;
        }
        let c = self.code_slice().get(o as usize..o as usize + 4)?;
        Some(u32::from_le_bytes([c[0], c[1], c[2], c[3]]))
    }

    /// The slice an installed fetch window points into.
    #[inline]
    fn code_slice(&self) -> &[u8] {
        match self.code_win {
            CodeWin::Rom => &self.rom,
            CodeWin::Iwram => &self.iwram,
            CodeWin::Ewram => &self.ewram,
        }
    }

    /// Point the fetch window at whatever backs `addr`, or clear it.
    ///
    /// Cold: this runs when execution crosses from one region into another (a
    /// ROM game calling into an IWRAM routine, an IRQ vectoring into the BIOS),
    /// which is thousands of times less often than a fetch.
    #[cold]
    fn install_code_window(&mut self, addr: u32) {
        let (win, base, len) = match (addr >> 24) & 0xF {
            0x2 => (CodeWin::Ewram, addr & !0x3_FFFF, self.ewram.len()),
            0x3 => (CodeWin::Iwram, addr & !0x7FFF, self.iwram.len()),
            // A cart with a GPIO port keeps the first 0x100 bytes of the ROM
            // window off the linear path, because the port sits at 0x080000C4
            // and those halfwords are not ROM reads. Such a cart gets no window
            // at all rather than a window that starts past the header.
            //
            // Both cleverer versions were tried and both were SLOWER than no
            // window at all: a shifted-slice variant of this enum and a bias
            // field each cost about 2% on every game, to win a few percent on
            // the handful of carts with a port. The match in `code_slice` is on
            // the hot path and a fourth arm is not free.
            0x8..=0xD if !self.sensors.has_gpio() => {
                (CodeWin::Rom, addr & !0x01FF_FFFF, self.rom.len())
            }
            _ => {
                self.code_span = 0;
                return;
            }
        };
        // A window must admit a 4-byte read at its last in-window address.
        if len < 4 {
            self.code_span = 0;
            return;
        }
        self.code_win = win;
        self.code_lo = base;
        self.code_span = (len - 3) as u32;
    }

    #[inline]
    fn read16_raw(&self, addr: u32) -> u16 {
        // The instruction-fetch/data hot path: one region dispatch, one slice
        // read, instead of two per-byte dispatches.
        if let Some((mem, o)) = self.linear_region(addr) {
            if o + 2 <= mem.len() {
                return u16::from_le_bytes([mem[o], mem[o + 1]]);
            }
        }
        u16::from_le_bytes([self.read8_raw(addr), self.read8_raw(addr + 1)])
    }
    /// Side-effect-free halfword read, for code the emulator wants to INSPECT
    /// rather than execute. No cycle charge and no EEPROM/GPIO handling, so it
    /// must never stand in for a real access.
    ///
    /// Returns `None` outside the regions code can run from, which is what lets
    /// the idle-loop analysis stop at the edge of a region instead of walking
    /// into open bus and reading the last fetch back as an opcode.
    pub fn code_peek16(&self, addr: u32) -> Option<u16> {
        match (addr >> 24) & 0xF {
            0x0 | 0x2 | 0x3 | 0x8..=0xD => Some(self.read16_raw(addr)),
            _ => None,
        }
    }

    /// Debug-only word read (no cycle cost, no side effects).
    pub fn read32_dbg(&self, addr: u32) -> u32 {
        self.read32_raw(addr)
    }

    #[inline]
    fn read32_raw(&self, addr: u32) -> u32 {
        // The bus always returns the word-aligned value; the CPU rotates it for
        // an unaligned LDR. Reading the raw (unaligned) bytes instead corrupts
        // e.g. Pokemon's palette-fade loop (it LDRs colours from a halfword
        // offset), which halved the palette and garbled sprites.
        let addr = addr & !3;
        if let Some((mem, o)) = self.linear_region(addr) {
            if o + 4 <= mem.len() {
                return u32::from_le_bytes([mem[o], mem[o + 1], mem[o + 2], mem[o + 3]]);
            }
        }
        u32::from_le_bytes([
            self.read8_raw(addr),
            self.read8_raw(addr + 1),
            self.read8_raw(addr + 2),
            self.read8_raw(addr + 3),
        ])
    }

    fn io_read8(&self, addr: u32) -> u8 {
        let off = addr & 0x3FF;
        match off {
            0x000..=0x05F => (self.ppu.read_reg16(off & !1) >> ((off & 1) * 8)) as u8,
            0x060..=0x0A7 => self.apu.read8(off),
            0x130 => self.keyinput as u8,
            0x131 => (self.keyinput >> 8) as u8,
            0x100..=0x10F => {
                let ch = ((off - 0x100) / 4) as usize;
                let t = &self.timers[ch];
                match (off - 0x100) % 4 {
                    0 => t.counter as u8,
                    1 => (t.counter >> 8) as u8,
                    2 => t.control as u8,
                    _ => (t.control >> 8) as u8,
                }
            }
            0x200 => self.ie as u8,
            0x201 => (self.ie >> 8) as u8,
            0x202 => self.if_ as u8,
            0x203 => (self.if_ >> 8) as u8,
            0x208 => self.ime as u8,
            0x209 => (self.ime >> 8) as u8,
            _ => self.io[off as usize],
        }
    }

    // --- Writes: width-aware so display-memory quirks are honored --------------

    fn write(&mut self, addr: u32, val: u32, width: u32) {
        // 0x10000000-0xFFFFFFFF is unused address space (the upper region-decode
        // nibble is not mapped): writes there are ignored on hardware. Bailing
        // here is essential - otherwise a wild/uninitialised pointer store (which
        // real hardware harmlessly drops) would alias down into a real region and
        // corrupt it. Motocross Maniacs et al. store to such a scratch pointer and
        // would otherwise clobber their own copied IRQ handler in IWRAM.
        if addr >= 0x1000_0000 {
            return;
        }
        // ARM7TDMI force-aligns store addresses to the access width: STR writes at
        // addr & ~3, STRH at addr & ~1 (unlike LDR, a store does NOT rotate - the
        // register value just lands at the aligned address). Without this, a word
        // store to an unaligned address splits across the aligned slot's neighbour
        // and corrupts it. DBZ Legacy of Goku registers a DMA2 IRQ handler with an
        // unaligned STR to its ISR table; the raw-address write mangled the entry's
        // top byte, so the first DMA2 IRQ vectored into garbage and ran away. (The
        // load side of this was fixed earlier in read32_raw.)
        let addr = match width {
            4 => addr & !3,
            2 => addr & !1,
            _ => addr,
        };
        if self.watch_addr != 0
            && addr >= self.watch_addr
            && addr < self.watch_addr + self.watch_len.max(4)
            && (self.watch_pc == 0 || self.watch_pc == self.cur_pc)
        {
            self.watch_total += 1;
            // Stash width in the top nibble (palette values are 16-bit, so free).
            self.watch_hits.push(((self.frame_no << 8) | (self.ppu.read_reg16(6) as u32 & 0xFF), (self.cur_mode << 28) | self.cur_pc, (width << 28) | addr, val));
            if self.watch_hits.len() > 512 {
                self.watch_hits.drain(..256);
            }
        }
        match (addr >> 24) & 0xF {
            0x2 => write_le(&mut self.ewram, (addr & 0x3_FFFF) as usize, val, width),
            0x3 => write_le(&mut self.iwram, (addr & 0x7FFF) as usize, val, width),
            0x4 => self.io_write(addr, val, width),
            0x5 => self.display_write(DisplayRegion::Pal, addr, val, width),
            0x6 => self.display_write(DisplayRegion::Vram, addr, val, width),
            0x7 => self.display_write(DisplayRegion::Oam, addr, val, width),
            // SRAM/Flash: an 8-bit bus, so only the low byte is written per byte.
            0xE | 0xF => {
                if !self.sensors.tilt_write(addr, val as u8) {
                    self.save.write(addr, val as u8);
                }
            }
            // ROM is read-only, except for the cartridge GPIO port. GBATEK: the
            // ROM bus takes 16- and 32-bit writes only, STRB is ignored.
            0x8..=0xD if width >= 2 => {
                let o = addr & 0x01FF_FFFF;
                if (0xC4..0xCA).contains(&o) {
                    self.sensors.gpio_write(o & !1, val as u16);
                    if width == 4 {
                        self.sensors.gpio_write((o & !1) + 2, (val >> 16) as u16);
                    }
                }
            }
            _ => {} // BIOS / ROM are read-only
        }
    }

    fn io_write(&mut self, addr: u32, val: u32, width: u32) {
        let off = addr & 0x3FF;
        // A game power-cycles an attached wireless adapter through RCNT, and
        // that is an edge against the PREVIOUS value, so sample it before the
        // store lands. Cheap enough to do unconditionally rather than branch.
        let touches_rcnt = off < 0x136 && off + width > 0x134;
        let prev_rcnt = if touches_rcnt { self.io_u16(0x134) } else { 0 };
        let touches_siocnt = off < 0x12A && off + width > 0x128;
        let prev_siocnt = if touches_siocnt { self.io_u16(0x128) } else { 0 };
        // SIOMLT_SEND, which in Multi-Player mode is this unit's word. A child's
        // reply is answered by its transport the instant the parent's clock
        // arrives, so the cable has to be told on the STORE rather than at the
        // next transfer, or it answers with the previous word.
        let touches_mlt_send = off < 0x12C && off + width > 0x12A;
        for i in 0..width {
            let o = off + i;
            let byte = (val >> (i * 8)) as u8;
            match o {
                0x000..=0x05F => {
                    // Merge the byte into its 16-bit register.
                    let reg = o & !1;
                    let cur = self.ppu.read_reg16(reg);
                    let merged = if o & 1 == 0 {
                        (cur & 0xFF00) | byte as u16
                    } else {
                        (cur & 0x00FF) | ((byte as u16) << 8)
                    };
                    self.ppu.write_reg16(reg, merged);
                }
                0x060..=0x0A7 => self.apu.write8(o, byte),
                0x130 | 0x131 => {} // KEYINPUT read-only
                0x200 => self.ie = (self.ie & 0xFF00) | byte as u16,
                0x201 => self.ie = (self.ie & 0x00FF) | ((byte as u16) << 8),
                // IF is write-1-to-clear (interrupt acknowledge).
                0x202 => self.if_ &= !(byte as u16),
                0x203 => self.if_ &= !((byte as u16) << 8),
                0x208 => self.ime = (self.ime & 0xFF00) | byte as u16,
                0x209 => self.ime = (self.ime & 0x00FF) | ((byte as u16) << 8),
                0x100..=0x10F => {
                    let ch = ((o - 0x100) / 4) as usize;
                    let t = &mut self.timers[ch];
                    match (o - 0x100) % 4 {
                        0 => t.reload = (t.reload & 0xFF00) | byte as u16,
                        1 => t.reload = (t.reload & 0x00FF) | ((byte as u16) << 8),
                        2 => {
                            let was_on = t.control & 0x80 != 0;
                            t.control = (t.control & 0xFF00) | byte as u16;
                            if t.control & 0x80 != 0 && !was_on {
                                t.counter = t.reload; // enable reloads the counter
                                t.subcycle = 0;
                            }
                        }
                        _ => t.control = (t.control & 0x00FF) | ((byte as u16) << 8),
                    }
                }
                // HALTCNT: bit 7 clear = Halt, set = Stop. Either way the CPU
                // parks until an interrupt it has enabled arrives.
                //
                // This was unhandled, and only the HLE SWI path ever set
                // `halted`. In the configuration we actually ship a BIOS is
                // loaded, so Halt and IntrWait run the BIOS's own code, which
                // writes here, and we ignored it. The CPU therefore never
                // parked: it spun inside the BIOS wait loop for the whole idle
                // part of every frame. The emulated game kept correct time, so
                // nothing looked wrong, but the host burned real cycles
                // emulating a spin that does nothing.
                0x301 => {
                    self.io[o as usize] = byte;
                    self.halted = true;
                }
                // A DMA control high byte carries the enable bit, and hardware
                // starts a transfer on its 0 -> 1 EDGE only. Storing a 1 over a 1
                // does nothing at all, so the previous state has to be read
                // before the byte lands.
                0xBB | 0xC7 | 0xD3 | 0xDF => {
                    let was_enabled = self.io[o as usize] & 0x80 != 0;
                    self.io[o as usize] = byte;
                    if !was_enabled {
                        self.start_dma(((o - 0xBB) / 12) as usize);
                    }
                }
                _ => {
                    self.io[o as usize] = byte;
                }
            }
        }
        // WAITCNT decides what every ROM access costs, so re-decode it as soon as
        // it lands rather than re-reading the register on each access.
        if off < 0x206 && off + width > 0x204 {
            self.waits = Waits::from_reg(self.io_u16(0x204));
        }
        // SIOCNT's start bit is in the LOW byte but the mode select is in the
        // HIGH byte, so this has to run after the whole store has landed rather
        // than per byte the way the DMA enable does.
        // An attached adapter drives SIOCNT bit 2 (SI) as a ready/busy
        // handshake against bit 3 (SO), and bit 2 is a status bit the game
        // cannot write at all. We used to store whatever the game wrote
        // verbatim, so the game never saw the adapter acknowledge anything and
        // gave up: Emerald got through the handshake, issued one command, then
        // power-cycled the adapter and started over, forever. Masking is
        // deliberately scoped to carts that have an adapter rather than
        // applied to every game, to keep the blast radius at 43 ROMs.
        if touches_siocnt && self.rfu.is_some() {
            let written = self.io_u16(0x128);
            let mut cnt = (written & 0x7F8B) | (prev_siocnt & 0x0004);
            let so_rose = cnt & 0x0008 != 0 && prev_siocnt & 0x0008 == 0;
            let so_fell = cnt & 0x0008 == 0 && prev_siocnt & 0x0008 != 0;
            if cnt & 0x0001 != 0 {
                // The game drives the clock. SO going high says the game is
                // busy, and the adapter answers by dropping SI to ready.
                if so_rose {
                    cnt &= !0x0004;
                }
            } else {
                // The adapter drives the clock, and SI just follows SO.
                if so_rose {
                    cnt |= 0x0004;
                } else if so_fell {
                    cnt &= !0x0004;
                }
            }
            self.set_io16(0x128, cnt);
        }
        // The read-only half of SIOCNT in Multi-Player mode. SI (bit 2), SD
        // (bit 3), the ID (bits 4-5) and the error flag (bit 6) are driven by
        // the cable, not by the game, and a game reads them to find out whether
        // it is the parent and whether its peer is there. We used to store
        // whatever the game wrote, which with a cable attached would let it
        // declare itself parent and both units would then drive the clock.
        if touches_siocnt && self.cable.is_some() && self.multi_mode() {
            // Selecting Multi-Player mode is the port being set up, so anything
            // in flight from before belongs to no protocol this game is running.
            if prev_siocnt & 0x3000 != 0x2000 {
                self.cable.as_mut().unwrap().flush();
                self.cable_words = None;
                self.cable_busy = false;
                self.serial_pending = 0;
            }
            let written = self.io_u16(0x128);
            // Bit 7 is Start on a parent and a read-only Busy on a child, so a
            // child's write to it is dropped along with the status bits.
            let keep = if self.cable.as_ref().is_some_and(|c| c.id() == 0) { 0x007C } else { 0x00FC };
            self.set_io16(0x128, (written & !keep) | (prev_siocnt & keep));
            self.drive_multi_status();
        }
        if touches_mlt_send {
            let word = self.io_u16(0x12A);
            if let Some(cable) = self.cable.as_mut() {
                cable.set_output(word);
            }
        }
        if touches_siocnt {
            self.sio_transfer();
        }
        // Driving SD high while it is an output resets the wireless adapter.
        // The test is gpSP's: SD is an output in the new value and was low in
        // the old one, which in practice is SD's rising edge. Emerald walks
        // RCNT 8000 -> 80A0 -> 80A2 at boot and this is the 80A2 step.
        if touches_rcnt && self.rfu.is_some() {
            let rcnt = self.io_u16(0x134);
            // Only while RCNT selects general-purpose mode (bit 15 set, bit 14
            // clear). Outside it those low bits are not pins at all, so acting
            // on them would let a game in serial mode power-cycle the adapter
            // by accident.
            let gpio_mode = rcnt & 0xC000 == 0x8000;
            if gpio_mode && rcnt & 0x20 != 0 && prev_rcnt & 0x02 == 0 {
                if std::env::var_os("GBA_RFULOG").is_some() {
                    eprintln!("  RFU reset  RCNT {prev_rcnt:04X} -> {rcnt:04X}");
                }
                self.rfu.as_mut().unwrap().reset();
            }
        }
        // GBA_SIOLOG: every store that lands on the serial block, with the mode
        // decoded. A game hung waiting on a serial IRQ looks exactly like one
        // that never asked for a transfer, and only the register trace tells the
        // two apart.
        if off < 0x136 && off + width > 0x120 && std::env::var_os("GBA_SIOLOG").is_some() {
            let cnt = self.io_u16(0x128);
            let rcnt = self.io_u16(0x134);
            let mode = if rcnt & 0x8000 != 0 {
                if rcnt & 0x4000 != 0 { "JOYBUS" } else { "GPIO" }
            } else {
                match cnt & 0x3000 {
                    0x0000 => "NORMAL8",
                    0x1000 => "NORMAL32",
                    0x2000 => "MULTI",
                    _ => "UART",
                }
            };
            eprintln!(
                "  SIO w{width} @{off:03X} SIOCNT={cnt:04X} RCNT={rcnt:04X} mode={mode} start={} irq={} send={:04X} multi={:04X},{:04X},{:04X},{:04X}",
                (cnt >> 7) & 1,
                (cnt >> 14) & 1,
                self.io_u16(0x12A),
                self.io_u16(0x120), self.io_u16(0x122), self.io_u16(0x124), self.io_u16(0x126),
            );
        }
    }

    /// Which serial mode the game has the port in, decoded the way
    /// [`GbaBus::write_io`]'s trace decodes it.
    ///
    /// Worth a heartbeat field of its own, because "we linked game X and nothing
    /// happened" has two completely different causes and no counter separates
    /// them. The cable only carries Multi-Player transfers: a game that picks
    /// NORMAL32 is asking for a mode nothing is attached to, which looks
    /// identical from the counters to a game that never tried to link at all.
    pub fn sio_mode(&self) -> &'static str {
        let cnt = self.io_u16(0x128);
        let rcnt = self.io_u16(0x134);
        if rcnt & 0x8000 != 0 {
            if rcnt & 0x4000 != 0 {
                "JOYBUS"
            } else {
                "GPIO"
            }
        } else {
            match cnt & 0x3000 {
                0x0000 => "NORMAL8",
                0x1000 => "NORMAL32",
                0x2000 => "MULTI",
                _ => "UART",
            }
        }
    }

    /// Advance the serial port by the cycles since the last call.
    ///
    /// Two jobs. It completes a paced transfer, clearing the start bit and
    /// raising the serial IRQ once the word has had time to move. And it gives
    /// an attached adapter the chance to take the clock and push an event back
    /// at the game, which is the one place the device, not the game, is master.
    pub fn step_serial(&mut self) {
        let delta = (self.cycles - self.serial_cycles) as u32;
        self.serial_cycles = self.cycles;

        if self.serial_pending > 0 {
            if self.serial_pending > delta {
                self.serial_pending -= delta;
            } else {
                self.serial_pending = 0;
                if self.cable_busy {
                    self.multi_finish();
                } else {
                    let cnt = self.io_u16(0x128);
                    // Clear start, and set SI to mark the device busy: that is
                    // the handshake the adapter runs between words.
                    self.set_io16(0x128, (cnt & !0x0080) | 0x0004);
                    if cnt & 0x4000 != 0 {
                        self.if_ |= 1 << 7;
                    }
                }
            }
        }

        // A child's transfers are started by the parent, so they arrive rather
        // than being asked for. Collected once per scanline: that adds at most
        // 73 us to the parent's wait, against a transfer the game has budgeted
        // 343 us for, and costs one test per scanline when no cable is attached.
        if self.cable.is_some() && self.multi_mode() {
            // Hardware drives SI, SD and the ID continuously, not only when the
            // game writes SIOCNT. A game that set up Multi-Player mode before
            // the session started would otherwise read SD low for as long as it
            // only polls, and conclude it has no peer.
            self.drive_multi_status();
        }
        if self.cable.is_some() && !self.cable_busy && self.multi_mode() {
            let id = self.cable.as_ref().map_or(0, |c| c.id());
            let units = self.cable.as_ref().map_or(1, |c| c.units());
            let clocked = (id != 0).then(|| self.cable.as_mut().unwrap().child_clock()).flatten();
            if let Some((parent_word, own_word)) = clocked {
                let mut words = [crate::cable::ABSENT; crate::cable::MAX_UNITS];
                words[0] = parent_word;
                words[id as usize] = own_word;
                self.cable_words = Some(words);
                self.cable_busy = true;
                let cnt = self.io_u16(0x128);
                // Busy goes up for the transfer's real duration. A game that
                // polls the bit mid-transfer has to see it set, and a child's
                // only evidence a transfer is happening is this plus the IRQ.
                self.set_io16(0x128, cnt | 0x0080);
                self.serial_pending = multi_cycles(cnt, units);
            }
        }

        if self.rfu.is_none() {
            return;
        }
        let cnt = self.io_u16(0x128);
        let game_is_slave = cnt & 0x0001 == 0;
        let so_si_clear = cnt & 0x000C == 0;
        let armed = cnt & 0x0080 != 0;
        let pushed = self
            .rfu
            .as_mut()
            .unwrap()
            .step(delta, game_is_slave, so_si_clear, armed);
        if let Some(word) = pushed {
            if std::env::var_os("GBA_RFULOG").is_some() {
                eprintln!("  RFU rx {word:08X}  (adapter holds the clock)");
            }
            self.set_io16(0x120, word as u16);
            self.set_io16(0x122, (word >> 16) as u16);
            self.set_io16(0x128, cnt & !0x0080);
            if cnt & 0x4000 != 0 {
                self.if_ |= 1 << 7;
            }
        }
    }

    /// Drop the cable and any transfer in flight over it.
    pub fn detach_cable(&mut self) {
        self.cable = None;
        self.cable_words = None;
        // A transfer that was in flight has to stop counting down, or the next
        // completion lands on a port with nothing attached and clears bits a
        // cable owns.
        if self.cable_busy {
            self.cable_busy = false;
            self.serial_pending = 0;
        }
    }

    /// Is the port in Multi-Player mode? RCNT bit 15 has to be clear or the
    /// pins are not serial pins at all, and SIOCNT bits 12-13 select the mode.
    pub(crate) fn multi_mode(&self) -> bool {
        self.io_u16(0x134) & 0x8000 == 0 && self.io_u16(0x128) & 0x3000 == 0x2000
    }

    /// Drive the bits of SIOCNT the cable owns rather than the game: SI says
    /// which end of the chain we are, SD says the bus is whole, and the ID is
    /// our position on it. All three are read-only to the game, and all three
    /// are what a game reads to decide whether it is the parent and whether its
    /// peer has arrived.
    fn drive_multi_status(&mut self) {
        let Some((id, units)) = self.cable.as_ref().map(|c| (c.id() as u16, c.units())) else {
            return;
        };
        if !self.multi_mode() {
            return;
        }
        let mut cnt = (self.io_u16(0x128) & !0x003C) | (id << 4);
        if id != 0 {
            cnt |= 0x0004; // SI low on the parent, high on a child
        }
        if units >= 2 {
            cnt |= 0x0008; // SD: every unit is present and ready
        }
        self.set_io16(0x128, cnt);
    }

    /// Start a Multi-Player transfer over the cable.
    ///
    /// Only the parent can: on hardware the start bit is read-only for a child,
    /// so a child that writes it is dropping the write, not queueing a transfer
    /// it will wait for forever.
    fn multi_start(&mut self, cnt: u16) {
        let Some((id, units)) = self.cable.as_ref().map(|c| (c.id(), c.units())) else {
            return;
        };
        if id != 0 {
            self.set_io16(0x128, cnt & !0x0080);
            return;
        }
        let own = self.io_u16(0x12A);
        self.cable_own = own;
        self.cable.as_mut().unwrap().parent_start(own);
        // Every slot reads FFFF while the transfer is in flight, which is what
        // a game polling mid-transfer sees on hardware.
        for slot in 0..crate::cable::MAX_UNITS as u32 {
            self.set_io16(0x120 + slot * 2, crate::cable::ABSENT);
        }
        self.cable_busy = true;
        self.cable_words = None;
        self.serial_pending = multi_cycles(cnt, units);
    }

    /// Land a Multi-Player transfer: every unit's word, the IDs, and the IRQ.
    fn multi_finish(&mut self) {
        self.cable_busy = false;
        use crate::cable::MultiResult;
        let outcome = match self.cable_words.take() {
            Some(w) => MultiResult::Landed(w),
            // A parent collects the children's words HERE rather than at the
            // start, so the network round trip overlaps the transfer's own
            // duration instead of being added to it. Only a parent gets here
            // with nothing in hand: a child's transfer is created BY the arriving
            // clock, so it always already holds the words.
            None => match self.cable.as_mut() {
                Some(c) => c.parent_result(),
                None => MultiResult::Gone,
            },
        };
        let words = match outcome {
            MultiResult::Landed(w) => Some(w),
            _ => None,
        };
        if let Some(w) = words {
            self.cable_transfers += 1;
            if self.cable_transfers % crate::cable::CABLE_MARK_EVERY == 0 {
                self.cable_mark_at = self.cable_transfers;
                self.cable_mark_sum = self.cable_window_sum;
                self.cable_window_sum = 0xcbf2_9ce4_8422_2325; // basis again
            }
            for half in w {
                for byte in half.to_be_bytes() {
                    self.cable_wordsum ^= byte as u64;
                    self.cable_wordsum = self.cable_wordsum.wrapping_mul(0x100_0000_01b3);
                    self.cable_window_sum ^= byte as u64;
                    self.cable_window_sum = self.cable_window_sum.wrapping_mul(0x100_0000_01b3);
                }
            }
            // The head of the stream, verbatim, on both ends. A hash says the
            // two disagree; these say what the words actually were, which is the
            // only way to tell a shifted stream from a corrupted one.
            if self.cable_transfers <= crate::cable::CABLE_TRACE_HEAD
                && std::env::var_os("GBA_NOSIOTRACE").is_none()
            {
                self.cable_trace.push((self.cable_transfers, w[0], w[1]));
            }
        } else {
            self.cable_failures += 1;
        }
        let units = self.cable.as_ref().map_or(1, |c| c.units()) as usize;
        let own_slot = self.cable.as_ref().map_or(0, |c| c.id()) as usize;
        let mut landed = match outcome {
            MultiResult::Landed(w) => w,
            // A late word is not an absent unit, and this is where we used to
            // say it was. ABSENT is FFFF, which every game reads as "no GBA in
            // that slot", so filling with it because one word missed its
            // deadline tells a game whose partner is sitting right there that
            // the cable came out. Zeros instead, the filler gpSP has shipped on
            // this exact path for years: a protocol that checksums its frames
            // rejects them and retries, where FFFF ends the link on the spot.
            MultiResult::Late => {
                self.cable_late += 1;
                let mut w = [crate::cable::ABSENT; crate::cable::MAX_UNITS];
                for word in w.iter_mut().take(units) {
                    *word = 0;
                }
                w
            }
            MultiResult::Gone => [crate::cable::ABSENT; crate::cable::MAX_UNITS],
        };
        // Our own word is never unknown: we latched it when the transfer
        // started, and hardware shows every unit its own slot whatever the far
        // end did.
        landed[own_slot] = match outcome {
            MultiResult::Landed(w) => w[own_slot],
            _ => self.cable_own,
        };
        for (slot, word) in landed.iter().enumerate() {
            self.set_io16(0x120 + slot as u32 * 2, *word);
        }
        // Clear busy, and raise the error flag only for a transfer that failed
        // while a unit was supposed to BE there. It used to go up for a late word
        // too, which sends a game down its link-error path when its own retry path
        // was available.
        //
        // The `units` test is the other half of the fix for the achievement
        // corruption on 2026-10-07. Once the bus reports a single unit, a transfer
        // is not failing at all: a lone GBA in Multi-Player mode clocks, reads
        // FFFF for the slots nobody is driving, and raises nothing. Holding the
        // error flag up sixty times a second for minutes is not a state hardware
        // can be in, and the game kept being driven through a path it was never
        // written for.
        let mut cnt = self.io_u16(0x128) & !0x00C0;
        if outcome == MultiResult::Gone && units >= 2 {
            cnt |= 0x0040;
        }
        self.set_io16(0x128, cnt);
        self.drive_multi_status();
        if std::env::var_os("GBA_SIOLOG").is_some() {
            eprintln!(
                "  CABLE {} {:04X} {:04X} {:04X} {:04X}{}",
                match outcome {
                    MultiResult::Landed(_) => "ok  ",
                    MultiResult::Late => "late",
                    MultiResult::Gone => "gone",
                },
                landed[0], landed[1], landed[2], landed[3],
                if cnt & 0x4000 != 0 { "  irq" } else { "" },
            );
        }
        if cnt & 0x4000 != 0 {
            self.if_ |= 1 << 7;
        }
    }

    /// Complete a serial transfer with nothing on the other end of the cable.
    ///
    /// The SIO registers used to be plain storage, so the start/busy bit stayed
    /// set forever and any game that polls it waiting for the transfer to finish
    /// spun there forever. Hardware always finishes: a lone GBA drives SD high in
    /// Multi-Player mode, transfers, and reports the absent players as FFFF.
    ///
    /// Completion is immediate rather than paced at the selected baud rate. That
    /// matches how this core already runs DMA, and it is safe because IF is only
    /// examined at instruction boundaries by the frame loop, so raising the IRQ
    /// from inside the store cannot re-enter a handler. It will have to become
    /// real timing when there is an actual link partner to stay in step with,
    /// since then the two ends have to agree on when a transfer lands.
    fn sio_transfer(&mut self) {
        let cnt = self.io_u16(0x128);
        if cnt & 0x0080 == 0 {
            return; // start/busy not set: nothing to do
        }
        if cnt & 0x3000 == 0x2000 {
            if self.cable.is_some() && self.io_u16(0x134) & 0x8000 == 0 {
                self.multi_start(cnt);
                return;
            }
            // Multi-Player. SIOMULTI0-3 reset to FFFF on start, then each unit's
            // own send data lands in its own slot. We are the parent (ID 0) and
            // alone, so slots 1-3 stay FFFF, which is how a game sees "no peer".
            let send = self.io_u16(0x12A);
            self.set_io16(0x120, send);
            self.set_io16(0x122, 0xFFFF);
            self.set_io16(0x124, 0xFFFF);
            self.set_io16(0x126, 0xFFFF);
            // Clear SI (bit 2, parent), ID (4-5), error (6) and start (7);
            // set SD (bit 3), which a lone unit in Multi-Player mode does drive.
            self.set_io16(0x128, (cnt & !0x00F4) | 0x0008);
        } else if self.rfu.is_some() && cnt & 0x0001 != 0 && self.serial_pending == 0 {
            // Normal mode into a wireless adapter, with us driving the clock.
            // The word goes to the device and its answer comes straight back,
            // but completion is PACED rather than immediate: the game polls
            // the start bit and runs an SO/SI handshake between words, and
            // finishing inside the store collapses a sequence it depends on.
            let sent = self.io_u32(0x120);
            let reply = self.rfu.as_mut().unwrap().transfer(sent);
            if std::env::var_os("GBA_RFULOG").is_some() {
                let rfu = self.rfu.as_ref().unwrap();
                eprintln!(
                    "  RFU tx {sent:08X} -> {reply:08X}  state={} cmds={}",
                    rfu.state_name(),
                    rfu.commands
                );
            }
            self.set_io16(0x120, reply as u16);
            self.set_io16(0x122, (reply >> 16) as u16);
            // Eight bits at the selected baud, four times over for a 32-bit
            // word. Emerald picks 256 KHz, so a word is about two scanlines.
            let mut pending = if cnt & 0x0002 != 0 { CYC_2MHZ_8BIT } else { CYC_256KHZ_8BIT };
            if cnt & 0x1000 != 0 {
                pending *= 4;
            }
            self.serial_pending = pending;
            // The start bit deliberately stays set; `step_serial` clears it
            // and raises the IRQ when the transfer has had time to happen.
            return;
        } else {
            // Normal mode. Bit 0 selects the shift clock: with an EXTERNAL clock
            // we are the slave, and with nothing plugged in there is no clock, so
            // no bits move and the start bit legitimately stays set. Completing
            // it anyway invents a reply: Derby Stallion Advance and JGTO Golf
            // Master Mobile both probe for the Mobile Adapter GB that way, and a
            // fake completion sent them down the adapter path and blanked them
            // (late 41 -> 2 and 257 -> 1) when they had been fine before.
            if cnt & 0x0001 == 0 {
                return;
            }
            // An idle line reads high, so all-ones shifts in.
            if cnt & 0x1000 != 0 {
                self.set_io16(0x120, 0xFFFF); // SIODATA32_L
                self.set_io16(0x122, 0xFFFF); // SIODATA32_H
            } else {
                self.io[0x12A] = 0xFF; // SIODATA8
            }
            // Clear start (bit 7), set SI state (bit 2) = High/None.
            self.set_io16(0x128, (cnt & !0x0080) | 0x0004);
        }
        if cnt & 0x4000 != 0 {
            self.if_ |= 1 << 7; // serial IRQ on completion
        }
    }

    fn set_io16(&mut self, off: u32, val: u16) {
        self.io[off as usize] = val as u8;
        self.io[off as usize + 1] = (val >> 8) as u8;
    }

    fn display_write(&mut self, region: DisplayRegion, addr: u32, val: u32, width: u32) {
        match region {
            DisplayRegion::Pal => {
                if width == 1 {
                    self.ppu.write_pal8(addr, val as u8);
                } else {
                    write_le(&mut self.ppu.palram, (addr & 0x3FF) as usize, val, width);
                }
            }
            DisplayRegion::Vram => {
                if width == 1 {
                    self.ppu.write_vram8(addr, val as u8);
                } else {
                    let i = Ppu::vram_index(addr);
                    write_le(&mut self.ppu.vram, i, val, width);
                }
            }
            DisplayRegion::Oam => {
                if width == 1 {
                    // OAM ignores byte writes.
                } else {
                    write_le(&mut self.ppu.oam, (addr & 0x3FF) as usize, val, width);
                }
            }
        }
    }
}

enum DisplayRegion {
    Pal,
    Vram,
    Oam,
}

/// Write the low `width` bytes of `val` little-endian into `mem` at `off`,
/// wrapping within the slice.
fn write_le(mem: &mut [u8], off: usize, val: u32, width: u32) {
    let width = width as usize;
    // Fast path: the whole value fits without wrapping (the common case).
    if off + width <= mem.len() {
        for i in 0..width {
            mem[off + i] = (val >> (i * 8)) as u8;
        }
        return;
    }
    let len = mem.len();
    for i in 0..width {
        mem[(off + i) % len] = (val >> (i * 8)) as u8;
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn bus() -> GbaBus {
        GbaBus::new(vec![0; 0x100], Vec::new())
    }

    /// A load or store interrupts the CPU's sequential fetch stream, so the
    /// opcode fetch after it pays the non-sequential cost. This is the implicit
    /// 1N in the ARM7TDMI's documented LDR = 1S+1N+1I / STR = 2N timings, and
    /// it is what GTA Advance's scene teardown is paced by: without it the
    /// teardown outran the V-count IRQ whose callback retires the cutscene
    /// streamer's pending buffer swap, and the stale swap then stamped 64 bytes
    /// of pixels over the freshly loaded IWRAM rasteriser at 0x03000100.
    #[test]
    fn a_data_access_makes_the_next_opcode_fetch_non_sequential() {
        let mut b = bus();
        // Default WAITCNT: ROM 16-bit N = 5 cycles, S = 3, prefetch off.
        b.set_fetch_pc(0x0800_0000, false);
        b.read32(0x0800_0000, Access::NonSeq);
        let c0 = b.cycles;
        b.set_fetch_pc(0x0800_0004, false);
        b.read32(0x0800_0004, Access::Seq);
        assert_eq!(b.cycles - c0, 6, "straight-line ARM opcode fetch: S+S = 3+3");
        let c1 = b.cycles;
        b.read32(0x0200_0000, Access::NonSeq);
        assert_eq!(b.cycles - c1, 6, "EWRAM word data access: two 16-bit halves at 2 waits");
        let c2 = b.cycles;
        b.set_fetch_pc(0x0800_0008, false);
        b.read32(0x0800_0008, Access::Seq);
        assert_eq!(b.cycles - c2, 8, "opcode fetch after a data access: N+S = 5+3");
        // And the stream is sequential again afterwards.
        let c3 = b.cycles;
        b.set_fetch_pc(0x0800_000C, false);
        b.read32(0x0800_000C, Access::Seq);
        assert_eq!(b.cycles - c3, 6, "one broken fetch, not a permanent penalty");
    }

    /// DMA transfers pay the real per-region bus cycles (2N + 2(n-1)S + 2I),
    /// not a flat rate. A 4-word immediate DMA3 from ROM to EWRAM at default
    /// wait-states costs: reads 8+6+6+6, writes 6*4, plus 2 internal cycles;
    /// the enabling I/O write itself is 1 cycle. The old flat model charged
    /// 4 cycles per word and made big blits run at double speed, which is half
    /// of how GTA Advance's teardown won a race it must lose.
    #[test]
    fn dma_is_charged_real_bus_cycles() {
        let mut b = bus();
        b.set_fetch_pc(0x0800_0000, false);
        b.write32(0x0400_00D4, 0x0800_0000, Access::NonSeq); // DMA3SAD
        b.write32(0x0400_00D8, 0x0200_0000, Access::NonSeq); // DMA3DAD
        let c0 = b.cycles;
        b.write32(0x0400_00DC, 0x8400_0004, Access::NonSeq); // enable, 32-bit, 4 words
        assert_eq!(b.cycles - c0, 1 + (8 + 6 + 6 + 6) + 4 * 6 + 2);
    }

    /// The BIOS region must read back its contents only to code executing
    /// inside it. Everywhere else it reads what the BIOS left on the bus.
    ///
    /// Legends of Wrestling II and both European Tetris Worlds builds all boot
    /// on this and all three died without it: each calls through a pointer that
    /// is legitimately null at that moment and decides what to do next from a
    /// byte it reads at a low address. With the BIOS exposed, that byte is one
    /// of OUR BIOS image's bytes, and reading a zero there sends the game down
    /// a path it was never meant to take.
    #[test]
    fn the_bios_reads_back_only_to_code_running_inside_it() {
        // A BIOS whose first two words are recognisable and are NOT either of
        // the values a protected read is supposed to return.
        let mut img = vec![0u8; 16 * 1024];
        img[0..4].copy_from_slice(&0x1111_1111u32.to_le_bytes());
        img[4..8].copy_from_slice(&0x2222_2222u32.to_le_bytes());
        let mut b = GbaBus::new(vec![0; 0x100], img);

        // Boot leaves the CPU fetching from the cartridge.
        b.set_fetch_pc(0x0800_0000, false);
        assert_eq!(
            b.read32(0, Access::NonSeq),
            0xE129_F000,
            "from outside, region 0 must return what the BIOS left on the bus"
        );
        assert_eq!(b.read8(1, Access::NonSeq), 0xF0, "byte reads select from that word");
        assert_eq!(b.read16(2, Access::NonSeq), 0xE129, "and so do halfword reads");

        // Executing inside the BIOS, it reads normally.
        b.set_fetch_pc(4, false);
        assert_eq!(b.read32(0, Access::NonSeq), 0x1111_1111, "from inside, real contents");
        assert_eq!(b.read32(4, Access::NonSeq), 0x2222_2222);

        // Leaving re-protects it, and what it leaves behind is the documented
        // constant rather than anything out of our own image.
        b.set_fetch_pc(0x0300_0000, false);
        assert_eq!(b.read32(0, Access::NonSeq), 0xE3A0_2004, "an SWI leaves this");
        assert_eq!(b.read32(4, Access::NonSeq), 0xE3A0_2004);

        // An IRQ leaves the other one. Vector in at 0x18, then back out.
        b.set_fetch_pc(0x18, false);
        b.set_fetch_pc(0x0300_0000, false);
        assert_eq!(b.read32(0, Access::NonSeq), 0xE25E_F004, "an IRQ leaves this");
    }

    /// Above the 16 KB BIOS, region 0 is unmapped, and unmapped is open bus:
    /// the last instruction the CPU fetched, NOT the BIOS and not the BIOS's
    /// prefetch. Mario vs Donkey Kong reads 0x0000F026 and Harry Potter
    /// Quidditch World Cup 0x00005498; handing either of them a BIOS value
    /// hangs the game, which is how this was found.
    #[test]
    fn unmapped_region_zero_is_open_bus_not_the_bios() {
        let mut img = vec![0u8; 16 * 1024];
        img[0x3026..0x302A].copy_from_slice(&0x4444_4444u32.to_le_bytes());
        // ROM carrying a recognisable word where the CPU will be fetching.
        let mut rom = vec![0u8; 0x100];
        rom[0x40..0x44].copy_from_slice(&0x7777_7777u32.to_le_bytes());
        let mut b = GbaBus::new(rom, img);

        b.set_fetch_pc(0x0800_0040, false);
        // 0xF026 masked into the BIOS would be 0x3026, which is what this used
        // to return. It must not.
        assert_eq!(
            b.read32(0x0000_F026, Access::NonSeq),
            0x7777_7777,
            "unmapped must read the CPU's own last fetch, not a BIOS mirror"
        );
        assert_ne!(b.read32(0x0000_F026, Access::NonSeq), 0x4444_4444);

        // A Thumb fetch only drove sixteen lines, so the halfword appears in
        // both halves.
        b.set_fetch_pc(0x0800_0040, true);
        assert_eq!(b.read32(0x0000_F026, Access::NonSeq), 0x7777_7777);
    }

    /// Build a 16 KB image carrying only the six words the r12 repair targets.
    fn open_bios_shaped(words: &[(usize, u32, u32)]) -> Vec<u8> {
        let mut b = vec![0u8; 16 * 1024];
        for &(o, original, _) in words {
            b[o..o + 4].copy_from_slice(&original.to_le_bytes());
        }
        b
    }

    /// The two branches in the RL repair must land exactly where they are meant
    /// to, decoded the way the CPU decodes them.
    ///
    /// This exists because the first cut of the table had the branch back
    /// hand-computed as `0xEAFFFEB2` when it should have been `0xEAFFFAB2`, a
    /// slip of one hex digit that sent it to 0x1F1C instead of 0x0F1C. It still
    /// stopped Mortal Kombat hanging, so every behavioural check I had said
    /// PASS, and it was only caught by diffing the patched image against a
    /// known-good one. Encoded branch offsets need decoding, not eyeballing.
    #[test]
    fn the_rl_repair_branches_land_where_they_are_aimed() {
        // ARM B: offset is a signed 24-bit word count, relative to PC + 8.
        let target = |at: usize, word: u32| -> usize {
            assert_eq!(word >> 24, 0xEA, "expected an unconditional B at {at:#X}");
            let off = ((word & 0x00FF_FFFF) << 8) as i32 >> 8; // sign-extend 24 -> 32
            (at as i64 + 8 + (off as i64) * 4) as usize
        };
        let word_at = |o: usize| OPEN_BIOS_RL.iter().find(|&&(x, _, _)| x == o).unwrap().2;

        assert_eq!(
            target(0x0F18, word_at(0x0F18)),
            RL_STUB,
            "the call site must branch into the stub"
        );
        assert_eq!(
            target(RL_STUB + 0xC, word_at(RL_STUB + 0xC)),
            0x0F1C,
            "the stub must branch back to the instruction after the one it replaced"
        );

        // The stub must sit in space the original image never uses, and the
        // table must say so, or the all-or-nothing guard is meaningless.
        for &(o, want, _) in OPEN_BIOS_RL.iter().filter(|&&(o, _, _)| o >= RL_STUB) {
            assert_eq!(want, 0, "stub word at {o:#X} must be expected-zero");
        }
    }

    /// The RL repair, like the r12 one, only touches the image it was measured
    /// against, and it must leave the four stub words alone unless they really
    /// are zero.
    #[test]
    fn the_rl_repair_only_touches_the_bios_it_was_measured_against() {
        let word = |b: &[u8], o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);

        let mut good = vec![0u8; 16 * 1024];
        for &(o, original, _) in OPEN_BIOS_RL {
            good[o..o + 4].copy_from_slice(&original.to_le_bytes());
        }
        align_the_rl_header_read(&mut good);
        for &(o, _, patched) in OPEN_BIOS_RL {
            assert_eq!(word(&good, o), patched, "word at {o:#X} should have been rewritten");
        }

        // Somebody else's BIOS, or a build whose stub space is already in use.
        let mut occupied = vec![0u8; 16 * 1024];
        for &(o, original, _) in OPEN_BIOS_RL {
            occupied[o..o + 4].copy_from_slice(&original.to_le_bytes());
        }
        occupied[RL_STUB + 4..RL_STUB + 8].copy_from_slice(&0xE1A0_0000u32.to_le_bytes());
        let before = occupied.clone();
        align_the_rl_header_read(&mut occupied);
        assert_eq!(occupied, before, "an image whose stub space is in use must be untouched");

        let mut zeros = vec![0u8; 16 * 1024];
        align_the_rl_header_read(&mut zeros);
        assert!(zeros.iter().all(|&x| x == 0), "an unrecognised BIOS must be untouched");
    }

    /// The r12 repair must rewrite an image it recognises and refuse every other
    /// one, including an image that matches in all but one word.
    ///
    /// Recognising too little silently un-fixes Elevator Action Old and New and
    /// the Bubble Bobble Old and New family. Recognising too much rewrites six
    /// words of somebody's real BIOS dump, which is far worse.
    #[test]
    fn the_r12_repair_only_touches_the_bios_it_was_measured_against() {
        let word = |b: &[u8], o: usize| u32::from_le_bytes([b[o], b[o + 1], b[o + 2], b[o + 3]]);

        let mut good = open_bios_shaped(OPEN_BIOS_R12);
        hold_io_base_in_r12(&mut good);
        for &(o, original, patched) in OPEN_BIOS_R12 {
            assert_eq!(
                word(&good, o),
                patched,
                "word at {o:#X} should have been rewritten from {original:#X}"
            );
        }

        // One word off is a different BIOS, and none of it may be touched.
        let mut near = open_bios_shaped(OPEN_BIOS_R12);
        let spoiled = OPEN_BIOS_R12[3].0;
        near[spoiled..spoiled + 4].copy_from_slice(&0xE1A0_0000u32.to_le_bytes());
        let before = near.clone();
        hold_io_base_in_r12(&mut near);
        assert_eq!(
            near, before,
            "a BIOS that differs anywhere must be left byte-identical"
        );

        // And an unrelated image, e.g. a real dump or the HLE stub.
        let mut zeros = vec![0u8; 16 * 1024];
        hold_io_base_in_r12(&mut zeros);
        assert!(zeros.iter().all(|&x| x == 0), "an unrecognised BIOS must be untouched");
    }

    /// The rewritten words must be the same instructions with r2 and r12 swapped,
    /// not merely different bytes: same opcode, same condition, same operand
    /// count. A wrong encoding here executes as some other instruction inside a
    /// BIOS we cannot single-step, and the symptom would be a hang with no clue.
    #[test]
    fn the_r12_repair_swaps_registers_and_changes_nothing_else() {
        for &(off, original, patched) in OPEN_BIOS_R12 {
            assert_eq!(
                original & 0xFFF0_0000,
                patched & 0xFFF0_0000,
                "condition and instruction class must survive at {off:#X}"
            );
            assert_ne!(original, patched, "every listed word must actually change");
        }
        // The two constants keep their values and only trade destination register.
        assert_eq!(OPEN_BIOS_R12[0].1 & 0xFFFF_0FFF, OPEN_BIOS_R12[0].2 & 0xFFFF_0FFF);
        assert_eq!(OPEN_BIOS_R12[1].1 & 0xFFFF_0FFF, OPEN_BIOS_R12[1].2 & 0xFFFF_0FFF);
        assert_eq!((OPEN_BIOS_R12[0].2 >> 12) & 0xF, 12, "the I/O base must land in r12");
        assert_eq!((OPEN_BIOS_R12[1].2 >> 12) & 0xF, 2, "and 0x208 in r2");
    }

    /// ROM access cost has to come from WAITCNT and from whether the access is
    /// sequential. It used to be a flat 5 cycles for every 16-bit ROM read and
    /// 8 for every 32-bit one, with the `Access` the CPU already passes thrown
    /// away, which charged a sequential opcode fetch the price of a random one.
    /// Mario Kart Super Circuit ran its own race clock at half speed as a
    /// result: 4.99 seconds of in-game time per 10 seconds of real time.
    #[test]
    fn rom_cost_follows_waitcnt_and_sequentiality() {
        let mut b = bus();

        // Reset value: WS0 is 4 wait states non-sequential, 2 sequential.
        assert_eq!(b.access_cycles(0x0800_0000, false, false), 5, "default N");
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 3, "default S");

        // What Mario Kart actually asks for: WS0 N = 3 waits, S = 1 wait.
        b.write(0x0400_0204, 0x4497, 2);
        assert_eq!(b.access_cycles(0x0800_0000, false, false), 4, "N = 1 + 3");
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 2, "S = 1 + 1");
        // ROM is a 16-bit bus, so a 32-bit access is two halves: N then S.
        assert_eq!(b.access_cycles(0x0800_0000, true, false), 6, "32-bit = N + S");

        // WS1 and WS2 have their own wait fields and their own regions, and
        // their sequential tables are not the same as WS0's. Happy Feet's
        // 0x4014 is a value that tells the three apart: WS0 S = 1 wait,
        // WS1 S = 4, WS2 S = 8.
        b.write(0x0400_0204, 0x4014, 2);
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 2, "WS0 S = 1 + 1");
        assert_eq!(b.access_cycles(0x0A00_0000, false, true), 5, "WS1 S = 1 + 4");
        assert_eq!(b.access_cycles(0x0C00_0000, false, true), 9, "WS2 S = 1 + 8");
    }

    /// The prefetch unit only pays off when the Game Pak bus was actually idle
    /// long enough to have run ahead. Charging a flat 1 cycle for every
    /// sequential fetch instead would be too generous: in a tight loop of
    /// sequential ROM code there are no spare cycles, and real hardware gets no
    /// benefit there either.
    #[test]
    fn prefetch_only_pays_when_the_bus_was_idle() {
        let mut b = bus();
        b.write(0x0400_0204, 0x4497, 2); // prefetch enabled (bit 14), WS0 S = 2 cycles

        // Straight back-to-back sequential fetches: nothing has run ahead.
        b.prefetch_credit = 0;
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 2, "no credit, full S");

        // Now give the unit some idle bus time, as an internal CPU cycle or an
        // access somewhere other than the cartridge would.
        b.credit_prefetch(8);
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 1, "credited, near free");

        // A jump empties the buffer, so the next sequential fetch pays again.
        b.credit_prefetch(8);
        b.access_cycles(0x0800_0000, false, false); // non-sequential = branch
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 2, "buffer flushed by a jump");

        // With prefetch disabled the credit must never apply.
        b.write(0x0400_0204, 0x0497, 2);
        b.credit_prefetch(64);
        assert_eq!(b.access_cycles(0x0800_0000, false, true), 2, "bit 14 clear, no benefit");
    }

    #[test]
    fn unused_high_addresses_do_not_alias_into_ram() {
        // Addresses >= 0x10000000 are unused space (the upper region-decode nibble
        // is not mapped); writes there must be dropped, NOT folded into a real
        // region. Before the fix, 0xE3A00057 aliased to IWRAM 0x0057 via a 4-bit
        // region mask and clobbered whatever lived there (e.g. a copied IRQ
        // handler), turning a harmless wild-pointer store into a boot crash.
        let mut b = bus();
        b.write32(0x0300_0054, 0xE28C_C004, Access::NonSeq); // real IWRAM word
        b.write32(0xE3A0_0057, 0x0000_0000, Access::NonSeq); // wild pointer store
        assert_eq!(
            b.read32(0x0300_0054, Access::NonSeq),
            0xE28C_C004,
            "a store to unused space (0xE3A00057) must not touch IWRAM"
        );
    }

    #[test]
    fn unaligned_word_store_force_aligns() {
        // ARM7TDMI STR ignores the low address bits: a word store to 0x...3 lands
        // at the word-aligned slot (0x...0), it does NOT straddle into the next
        // word. DBZ Legacy of Goku registers an IRQ handler this way; the raw
        // (unaligned) write corrupted the neighbouring ISR-table entry.
        let mut b = bus();
        b.write32(0x0300_0010, 0xAAAA_AAAA, Access::NonSeq); // neighbour slot
        b.write32(0x0300_000F, 0x0800_996D, Access::NonSeq); // unaligned -> 0x...0C
        assert_eq!(
            b.read32(0x0300_000C, Access::NonSeq),
            0x0800_996D,
            "unaligned word store must land at the aligned slot"
        );
        assert_eq!(
            b.read32(0x0300_0010, Access::NonSeq),
            0xAAAA_AAAA,
            "unaligned word store must not corrupt the next word"
        );
        // Halfword stores force-align to & ~1 the same way.
        b.write16(0x0300_0021, 0x1234, Access::NonSeq); // -> 0x...20
        assert_eq!(b.read16(0x0300_0020, Access::NonSeq), 0x1234);
    }

    #[test]
    fn dma_writing_dma_registers_queues_instead_of_recursing() {
        // A transfer goes through the normal bus, so a DMA whose destination is
        // the DMA control block re-enters start_dma. Unguarded that recursed once
        // per transferred word and overflowed the stack, which aborts the process
        // outright rather than panicking. Kaisertal (Europe) (Demo) does it.
        //
        // DMA3 writes one word into DMA0's SAD..CNT block, arming DMA0 to copy a
        // marker into IWRAM. DMA0 must run exactly once, after DMA3 finishes.
        let mut b = bus();
        b.write32(0x0300_0100, 0xFEED_FACE, Access::NonSeq); // DMA0's future source
        b.write32(0x0300_0200, 0x0000_0000, Access::NonSeq); // DMA0's future dest

        // Pre-load DMA0's SAD/DAD; DMA3 supplies the count+control word.
        b.write32(0x0400_00B0, 0x0300_0100, Access::NonSeq); // DMA0SAD
        b.write32(0x0400_00B4, 0x0300_0200, Access::NonSeq); // DMA0DAD

        // The word DMA3 stores over DMA0CNT (count in the low half, control in
        // the high half): 1 word, enable + 32-bit + immediate. It has to be one
        // aligned word because stores force-align, so writing CNT_H alone at
        // 0xBA would slide down to 0xB8 and land in CNT_L.
        b.write32(0x0300_0300, 0x8400_0001, Access::NonSeq);

        b.write32(0x0400_00D4, 0x0300_0300, Access::NonSeq); // DMA3SAD
        b.write32(0x0400_00D8, 0x0400_00B8, Access::NonSeq); // DMA3DAD = DMA0CNT
        b.write16(0x0400_00DC, 1, Access::NonSeq); // DMA3CNT_L = 1
        b.write16(0x0400_00DE, 0x8400, Access::NonSeq); // enable + 32-bit + immediate

        assert_eq!(
            b.read32(0x0300_0200, Access::NonSeq),
            0xFEED_FACE,
            "the DMA armed from inside a transfer must still run"
        );
        assert!(!b.dma_active, "the guard must be released once the drain ends");
        assert_eq!(b.dma_pending, 0, "nothing may be left queued");
    }

    #[test]
    fn the_sound_fifo_is_serviced_at_the_cycle_the_dma_is_rearmed() {
        // The frame loop advances the APU's clock once per scanline, after the
        // line's CPU work, so by default every mid-line event is attributed to a
        // line boundary. Re-arming a Direct Sound FIFO channel cannot tolerate
        // that: the re-arm moves the read pointer back to the start of the game's
        // PCM buffer, so which FIFO refills happen before it and which after
        // decides where the next read lands. A scanline is 1232 cycles, about 1.3
        // Direct Sound samples, so rounding the re-arm to a line boundary moves
        // the read by a whole 16-byte refill.
        //
        // Golden Nugget Casino and Caesars Palace Advance mix exactly 608 bytes
        // of PCM per two frames into a buffer with no slack, and ARM code sits
        // immediately after it. Reading 16 bytes past the end plays that code as
        // 8-bit samples: a loud tick several times a second, which is what the
        // owner heard on hardware.
        let mut b = bus();
        b.write32(0x0400_00BC, 0x0300_0000, Access::NonSeq); // DMA1SAD
        b.write32(0x0400_00C0, 0x0400_00A0, Access::NonSeq); // DMA1DAD = FIFO_A
        b.write16(0x0400_00C4, 0, Access::NonSeq); // DMA1CNT_L

        // The CPU is partway through a scanline when the re-arm store executes.
        b.line_cycle_base = b.cycles;
        b.audio_line_base = b.apu.cycle();
        b.cycles += 1024;

        // enable | 32-bit | repeat | timing = special (sound FIFO)
        b.write16(0x0400_00C6, 0x8000 | 0x400 | 0x200 | (3 << 12), Access::NonSeq);

        let advanced = b.apu.cycle() - b.audio_line_base;
        assert!(
            advanced >= 512,
            "the APU must be caught up to the re-arm's own cycle, not left at the              start of the line; it advanced {advanced} cycles of the 1024 the CPU had run"
        );
        assert!(
            advanced <= crate::ppu::CYCLES_PER_LINE as u64,
            "and it must not run past the line it is inside ({advanced} cycles)"
        );
    }

    #[test]
    fn rewriting_a_running_dma_control_does_not_restart_it() {
        // Hardware starts a DMA on the 0 -> 1 EDGE of the enable bit. Storing a 1
        // over a 1 does nothing at all.
        //
        // Grand Theft Auto Advance tears its sound DMA down in two steps at
        // 0x08033418: first DMA1CNT_H &= 0xC5FF, which clears the repeat and
        // start-timing bits but deliberately LEAVES enable set, then
        // DMA1CNT_H &= 0x7FFF to drop enable. Restarting on any write with enable
        // set made step one latch the channel's stale registers with its timing
        // now reading as "immediate", and run a 0x4000-word transfer whose
        // destination walked up from the sound FIFO through the entire I/O block.
        // That cleared DISPCNT and DISPSTAT's V-blank IRQ enable, after which the
        // game waited forever in VBlankIntrWait for an interrupt it could no
        // longer receive. Confirming a save name was a permanent black screen.
        let mut b = bus();
        b.write32(0x0300_0000, 0xCAFE_F00D, Access::NonSeq); // the source word

        b.write32(0x0400_00BC, 0x0300_0000, Access::NonSeq); // DMA1SAD
        b.write32(0x0400_00C0, 0x0300_1000, Access::NonSeq); // DMA1DAD
        b.write16(0x0400_00C4, 1, Access::NonSeq); // DMA1CNT_L = 1 word
        // enable | IRQ | 32-bit | repeat | timing = V-blank. V-blank timing means
        // it does NOT fire on the store, and repeat means enable stays set.
        b.write16(0x0400_00C6, 0x8000 | 0x4000 | 0x400 | 0x200 | (1 << 12), Access::NonSeq);
        assert_eq!(
            b.read32(0x0300_1000, Access::NonSeq),
            0,
            "a V-blank-timed channel must not transfer when it is armed"
        );

        // Step one of the teardown, exactly as the game writes it.
        let control = b.io_u16(0xC6);
        assert_eq!(control & 0x8000, 0x8000, "a repeating channel keeps enable set");
        b.write16(0x0400_00C6, control & 0xC5FF, Access::NonSeq);

        assert_eq!(
            b.read32(0x0300_1000, Access::NonSeq),
            0,
            "rewriting the control register of an already-enabled channel must not              start a transfer; only a 0 -> 1 edge on the enable bit does"
        );
    }

    #[test]
    fn self_rearming_dma_terminates() {
        // The pathological shape: a channel whose destination IS its own control
        // register, with a source word that sets the enable bit again. Unguarded
        // this recursed once per transferred word and overflowed the stack, which
        // aborts the process outright (uncatchable by catch_unwind, and how it
        // silently truncated a 6118-ROM sweep). Kaisertal (Europe) (Demo) does it.
        //
        // Since DMA became edge-triggered the store cannot re-arm the channel at
        // all, because its enable bit is already set while the transfer runs, so
        // the shape is now closed twice over. That is what the source assertion
        // below pins: one transfer, so the source advanced by exactly one word.
        // The cross-channel arming that the guard still protects is covered by
        // `dma_writing_dma_registers_queues_instead_of_recursing`.
        let mut b = bus();
        b.write32(0x0300_0300, 0x8400_0001, Access::NonSeq); // count 1, enable+32bit+immediate
        b.write32(0x0400_00B0, 0x0300_0300, Access::NonSeq); // DMA0SAD
        b.write32(0x0400_00B4, 0x0400_00B8, Access::NonSeq); // DMA0DAD = its own CNT
        b.write32(0x0400_00B8, 0x8400_0001, Access::NonSeq); // count + enable, fires now
        assert!(!b.dma_active, "the guard must be released");
        assert_eq!(b.dma_pending, 0, "the drain must leave nothing queued");
        assert_eq!(
            b.dma_src[0], 0x0300_0304,
            "the channel must have run exactly once: a store of 1 over an enable              bit that is already 1 is not an edge and starts nothing"
        );
    }

    /// A peer that has gone leaves a LONE GBA, not a broken one.
    ///
    /// The second half of the 2026-10-07 achievement corruption. While a peer was
    /// gone the bus went on reporting two units, so SD said a partner was present
    /// and ready while every transfer came back FFFF with the error flag, sixty
    /// times a second. No cable can do that: pull one out and SD drops. Mario Kart
    /// Super Circuit, driven through that impossible state for four minutes, wrote
    /// enough nonsense into its own tables to unlock a third of its achievement
    /// set in one go.
    ///
    /// So once the bus is down to one unit, a transfer is an ordinary lone-unit
    /// transfer: absent slots, and nothing raised.
    #[test]
    fn a_transfer_with_no_peer_left_is_lone_not_failed() {
        struct Lone;
        impl crate::cable::LinkCable for Lone {
            fn units(&self) -> u8 {
                1 // the peer stopped answering, so the bus is down to us
            }
            fn id(&self) -> u8 {
                0
            }
            fn set_output(&mut self, _word: u16) {}
            fn parent_start(&mut self, _own: u16) {}
            fn parent_result(&mut self) -> crate::cable::MultiResult {
                crate::cable::MultiResult::Gone
            }
            fn child_clock(&mut self) -> Option<(u16, u16)> {
                None
            }
            fn flush(&mut self) {}
        }

        let mut b = bus();
        b.cable = Some(Box::new(Lone));
        b.write16(0x0400_012A, 0xBEEF, Access::NonSeq);
        b.write16(0x0400_0128, 0x2000 | 0x0080 | 0x0003, Access::NonSeq);
        b.cycles += 40_000;
        b.step_serial();

        let cnt = b.io_u16(0x128);
        assert_eq!(cnt & 0x0040, 0, "a lone unit's transfer is not an error");
        assert_eq!(cnt & 0x0008, 0, "and SD is low: there is no partner to be ready");
        assert_eq!(b.io_u16(0x120), 0xBEEF, "our own word still lands in our slot");
        assert_eq!(b.io_u16(0x122), crate::cable::ABSENT, "and nobody drives the rest");
    }

    /// The mode the heartbeat reports has to come from both registers, because
    /// RCNT's bit 15 overrides SIOCNT entirely. Reading only SIOCNT would report
    /// MULTI for a cartridge sitting on GPIO, which is every RTC and rumble game.
    #[test]
    fn the_serial_mode_is_decoded_from_both_registers() {
        let mut b = bus();
        b.write16(0x0400_0134, 0x0000, Access::NonSeq); // RCNT: serial pins
        b.write16(0x0400_0128, 0x2000, Access::NonSeq);
        assert_eq!(b.sio_mode(), "MULTI", "the one mode the cable carries");
        b.write16(0x0400_0128, 0x1000, Access::NonSeq);
        assert_eq!(b.sio_mode(), "NORMAL32", "and the one it does not");
        b.write16(0x0400_0134, 0x8000, Access::NonSeq);
        assert_eq!(b.sio_mode(), "GPIO", "RCNT bit 15 outranks SIOCNT's mode bits");
    }

    /// A cable whose parent transfers end however a test says they do.
    struct Scripted(crate::cable::MultiResult, u16);

    impl crate::cable::LinkCable for Scripted {
        fn units(&self) -> u8 {
            2 // the peer is in the session in BOTH cases; only answering differs
        }
        fn id(&self) -> u8 {
            0 // the parent: the only end that can give up on a transfer
        }
        fn set_output(&mut self, word: u16) {
            self.1 = word;
        }
        fn parent_start(&mut self, _own: u16) {}
        fn parent_result(&mut self) -> crate::cable::MultiResult {
            self.0
        }
        fn child_clock(&mut self) -> Option<(u16, u16)> {
            None
        }
        fn flush(&mut self) {}
    }

    /// Run one Multi-Player transfer to completion against `outcome`, and give
    /// back the four slots and SIOCNT.
    fn one_transfer(outcome: crate::cable::MultiResult) -> ([u16; 4], u16) {
        let mut b = bus();
        b.cable = Some(Box::new(Scripted(outcome, 0)));
        b.write16(0x0400_012A, 0xBEEF, Access::NonSeq); // our word for this transfer
        // Multi-Player at 115200, which is the baud Pokemon links at, plus start.
        b.write16(0x0400_0128, 0x2000 | 0x0080 | 0x0003, Access::NonSeq);
        // The slots read FFFF while it is in flight, so a test that forgot to let
        // the transfer finish would see absence for a different reason.
        assert!(b.cable_busy, "the transfer must still be running here");
        b.cycles += 40_000; // longer than any entry in the transfer table
        b.step_serial();
        assert!(!b.cable_busy, "and finished after its duration");
        (
            [
                b.io_u16(0x120),
                b.io_u16(0x122),
                b.io_u16(0x124),
                b.io_u16(0x126),
            ],
            b.io_u16(0x128),
        )
    }

    /// A word that misses its deadline must NOT be reported as an absent unit.
    ///
    /// This is the bug the first device sessions ended on. `ABSENT` is FFFF,
    /// which a game reads as "no GBA in that slot", so filling every slot with it
    /// AND raising the error flag told Ruby twice over that the cable had come
    /// out, when the peer was present, answering, and one word was late. Ruby
    /// goes straight to "transmission error" on that.
    #[test]
    fn a_late_peer_is_not_reported_as_an_absent_one() {
        let (slots, cnt) = one_transfer(crate::cable::MultiResult::Late);
        assert_eq!(slots[0], 0xBEEF, "our own word is never unknown: we latched it");
        assert_ne!(
            slots[1], crate::cable::ABSENT,
            "the peer is in the session, so its slot must not claim it is gone"
        );
        assert_eq!(slots[1], 0, "zeros, which a protocol that checksums rejects");
        assert_eq!(
            slots[2], crate::cable::ABSENT,
            "slots past the unit count are genuinely empty"
        );
        assert_eq!(slots[3], crate::cable::ABSENT);
        assert_eq!(cnt & 0x0040, 0, "and no error flag: the game keeps its retry path");
        assert_eq!(cnt & 0x0080, 0, "busy clears either way");
    }

    /// A peer that has actually stopped answering gets the opposite treatment,
    /// because here both signals are true.
    #[test]
    fn a_peer_that_has_gone_is_reported_absent_with_the_error_flag() {
        let (slots, cnt) = one_transfer(crate::cable::MultiResult::Gone);
        assert_eq!(slots[0], 0xBEEF, "the parent still knows its own word");
        assert_eq!(slots[1], crate::cable::ABSENT, "nothing on the far end");
        assert_eq!(cnt & 0x0040, 0x0040, "error flag: the link really has failed");
    }

    /// The counters have to separate the two, because one is our transport
    /// missing a deadline and the other is the player's cable or app going away.
    #[test]
    fn late_and_gone_are_counted_apart() {
        let mut b = bus();
        b.cable = Some(Box::new(Scripted(crate::cable::MultiResult::Late, 0)));
        b.write16(0x0400_0128, 0x2000 | 0x0080 | 0x0003, Access::NonSeq);
        b.cycles += 40_000;
        b.step_serial();
        assert_eq!(b.cable_late, 1);
        assert_eq!(b.cable_failures, 1, "a late transfer is still a transfer we lost");
        assert_eq!(b.cable_transfers, 0, "and never counted as one that landed");

        let mut b = bus();
        b.cable = Some(Box::new(Scripted(crate::cable::MultiResult::Gone, 0)));
        b.write16(0x0400_0128, 0x2000 | 0x0080 | 0x0003, Access::NonSeq);
        b.cycles += 40_000;
        b.step_serial();
        assert_eq!(b.cable_late, 0, "a peer that has gone is not late");
        assert_eq!(b.cable_failures, 1);
    }

    #[test]
    fn multiplayer_transfer_completes_with_no_peer() {
        // The SIO registers were plain storage, so the start/busy bit stayed set
        // and a game polling it hung. A lone GBA still completes the transfer.
        let mut b = bus();
        b.write16(0x0400_012A, 0xBEEF, Access::NonSeq); // SIOMLT_SEND
        // SIOCNT: Multi-Player (bit13), IRQ enable (bit14), start (bit7).
        b.write16(0x0400_0128, 0x2000 | 0x4000 | 0x0080, Access::NonSeq);

        let cnt = b.read16(0x0400_0128, Access::NonSeq);
        assert_eq!(cnt & 0x0080, 0, "start/busy must clear when the transfer ends");
        assert_eq!(cnt & 0x0004, 0, "SI low: a lone unit is the parent");
        assert_eq!(cnt & 0x0008, 0x0008, "SD high: Multi-Player mode drives it");
        assert_eq!(cnt & 0x0030, 0, "multi-player ID 0 (parent)");
        assert_eq!(cnt & 0x0040, 0, "no error flag");

        assert_eq!(
            b.read16(0x0400_0120, Access::NonSeq),
            0xBEEF,
            "our own send data lands in our own slot"
        );
        for (i, off) in [0x0400_0122u32, 0x0400_0124, 0x0400_0126].iter().enumerate() {
            assert_eq!(
                b.read16(*off, Access::NonSeq),
                0xFFFF,
                "absent player {} reads FFFF",
                i + 1
            );
        }
        assert!(b.if_ & (1 << 7) != 0, "completion raises the serial IRQ");
    }

    #[test]
    fn normal_mode_master_transfer_completes_and_shifts_in_ones() {
        // Normal mode, internal clock (bit 0 = master), 32-bit, IRQ disabled.
        let mut b = bus();
        b.write16(0x0400_0128, 0x1000 | 0x0080 | 0x0001, Access::NonSeq);
        let cnt = b.read16(0x0400_0128, Access::NonSeq);
        assert_eq!(cnt & 0x0080, 0, "start must clear");
        assert_eq!(cnt & 0x0004, 0x0004, "SI reads High/None with no opponent");
        assert_eq!(b.read32(0x0400_0120, Access::NonSeq), 0xFFFF_FFFF);
        assert_eq!(b.if_ & (1 << 7), 0, "IRQ disabled, so none raised");

        // 8-bit variant lands in SIODATA8.
        let mut b = bus();
        b.write16(0x0400_0128, 0x0080 | 0x0001, Access::NonSeq);
        assert_eq!(b.read8(0x0400_012A, Access::NonSeq), 0xFF);
    }

    #[test]
    fn normal_mode_slave_does_not_invent_a_reply() {
        // External clock with nothing plugged in means no clock, so no bits move
        // and the start bit stays set, exactly as on hardware. Completing it
        // anyway fabricates a peer: that is how Derby Stallion Advance and JGTO
        // Golf Master Mobile, which both probe for the Mobile Adapter GB this
        // way, got pushed down the adapter path and blanked.
        let mut b = bus();
        b.set_io16(0x120, 0x1234);
        // 32-bit, start, IRQ enable, bit 0 clear = external clock (slave).
        b.write16(0x0400_0128, 0x1000 | 0x0080 | 0x4000, Access::NonSeq);
        assert_eq!(
            b.read16(0x0400_0128, Access::NonSeq) & 0x0080,
            0x0080,
            "a slave with no clock stays busy"
        );
        assert_eq!(
            b.read16(0x0400_0120, Access::NonSeq),
            0x1234,
            "no data may be shifted in"
        );
        assert_eq!(b.if_ & (1 << 7), 0, "and no completion IRQ");
    }

    #[test]
    fn sio_does_nothing_without_the_start_bit() {
        // Writing mode/baud bits alone must not fake a transfer or an IRQ.
        let mut b = bus();
        b.set_io16(0x120, 0x1234);
        b.write16(0x0400_0128, 0x2000 | 0x4000, Access::NonSeq); // multi + IRQ, no start
        assert_eq!(
            b.read16(0x0400_0120, Access::NonSeq),
            0x1234,
            "SIOMULTI0 untouched with no transfer"
        );
        assert_eq!(b.if_ & (1 << 7), 0, "no IRQ without a transfer");
    }

    #[test]
    fn timer_overflow_raises_irq() {
        let mut b = bus();
        b.write16(0x0400_0100, 0xFFFE, Access::NonSeq); // reload
        b.write16(0x0400_0102, 0x00C0, Access::NonSeq); // enable + IRQ, prescaler 1
        b.timer_cycles = b.cycles; // ignore setup cycles
        b.cycles += 4;
        b.step_timers();
        assert!(b.if_ & (1 << 3) != 0, "timer0 overflow should request IRQ");
        assert_eq!(b.timers[0].counter, 0xFFFE);
    }

    #[test]
    fn dma_immediate_copy() {
        let mut b = bus();
        for i in 0..8u32 {
            b.write32(0x0300_0000 + i * 4, 0x1000 + i, Access::NonSeq);
        }
        b.write32(0x0400_00B0, 0x0300_0000, Access::NonSeq); // SAD
        b.write32(0x0400_00B4, 0x0300_0100, Access::NonSeq); // DAD
        b.write16(0x0400_00B8, 8, Access::NonSeq); // count
        b.write16(0x0400_00BA, 0x8000 | 0x400, Access::NonSeq); // enable, 32-bit, immediate
        for i in 0..8u32 {
            assert_eq!(b.read32(0x0300_0100 + i * 4, Access::NonSeq), 0x1000 + i);
        }
        // Enable bit self-clears after a non-repeating transfer.
        assert_eq!(b.io_u16(0xBA) & 0x8000, 0);
    }
}

impl Bus for GbaBus {
    fn read8(&mut self, addr: u32, a: Access) -> u8 {
        self.charge(addr, false, a);
        self.read8_raw(addr)
    }
    fn read16(&mut self, addr: u32, a: Access) -> u16 {
        self.charge(addr, false, a);
        // Direct EEPROM access (e.g. a game polling write-ready).
        if self.save.is_eeprom() && (addr >> 24) == 0xD {
            return self.save.eeprom_read_bit() as u16;
        }
        self.read16_raw(addr)
    }
    fn read32(&mut self, addr: u32, a: Access) -> u32 {
        self.charge(addr, true, a);
        self.read32_raw(addr)
    }
    fn write8(&mut self, addr: u32, val: u8, a: Access) {
        self.charge(addr, false, a);
        self.write(addr, val as u32, 1);
    }
    fn write16(&mut self, addr: u32, val: u16, a: Access) {
        self.charge(addr, false, a);
        if self.save.is_eeprom() && (addr >> 24) == 0xD {
            self.save.eeprom_write_bit(val as u8);
            return;
        }
        self.write(addr, val as u32, 2);
    }
    fn write32(&mut self, addr: u32, val: u32, a: Access) {
        self.charge(addr, true, a);
        self.write(addr, val, 4);
    }
    /// Instruction fetch. Same cycle charge as the data path, deliberately
    /// through the very same [`GbaBus::charge`] call so the split cannot move a
    /// single cycle; what it skips is the region decode on the READ.
    ///
    /// Measured on a LeafGreen battle scene: the fetch read alone is 12.1% of
    /// CPU time (593 to 665 us/frame when doubled), and 96.6% of fetches land
    /// in one of the three windowed regions (75.3% ROM, 21.3% IWRAM, and EWRAM
    /// which this scene never executes from).
    fn fetch16(&mut self, addr: u32, a: Access) -> u16 {
        self.charge(addr, false, a);
        // Carried over from the data path, which this used to go through: a
        // cart that saves to EEPROM answers reads in the top ROM window from
        // the EEPROM state machine, not from ROM.
        if self.save.is_eeprom() && (addr >> 24) == 0xD {
            return self.save.eeprom_read_bit() as u16;
        }
        if let Some(h) = self.code_fetch16(addr) {
            return h;
        }
        self.install_code_window(addr);
        match self.code_fetch16(addr) {
            Some(h) => h,
            None => self.read16_raw(addr),
        }
    }

    /// Word-width companion to [`GbaBus::fetch16`]. Masks the address exactly
    /// as [`GbaBus::read32_raw`] does: the bus always returns the word-aligned
    /// value and the CPU rotates it.
    fn fetch32(&mut self, addr: u32, a: Access) -> u32 {
        self.charge(addr, true, a);
        let addr = addr & !3;
        if let Some(w) = self.code_fetch32(addr) {
            return w;
        }
        self.install_code_window(addr);
        match self.code_fetch32(addr) {
            Some(w) => w,
            None => self.read32_raw(addr),
        }
    }

    fn tick(&mut self, n: u32) {
        self.cycles += n as u64;
        // Internal CPU cycles (shifts, multiplies, branches) leave the Game Pak
        // bus free, which is exactly when the prefetch unit earns its keep.
        self.credit_prefetch(n as u64);
    }
    fn set_halted(&mut self, halted: bool) {
        self.halted = halted;
    }
    /// BIOS read protection. Region 0 reads back its contents only to code
    /// executing inside it; from anywhere else the hardware returns the last
    /// opcode the BIOS fetched, which is what `bios_prefetch` holds.
    ///
    /// Legends of Wrestling II is the reason this exists. It calls through a
    /// null object pointer: `ldr r1,[0x03000004]` yields 0, and the game then
    /// reads the byte at 0xC3 to decide whether to make an indirect call. With
    /// the BIOS readable that byte is 0, the one-shot test passes, and the call
    /// goes through the null pointer to the word at address 0, which is the
    /// BIOS reset branch. The PC runs away into unmapped space inside the first
    /// frame and never comes back.
    fn set_fetch_pc(&mut self, addr: u32, thumb: bool) {
        self.last_fetch = addr;
        self.last_fetch_thumb = thumb;
        let now_in_bios = addr < 0x4000;
        // Vectoring in. 0x18 is the IRQ entry and 0x08 the SWI entry, and the
        // two leave different words behind.
        match addr {
            0x18 => self.bios_entry_irq = true,
            0x08 => self.bios_entry_irq = false,
            _ => {}
        }
        if self.exec_in_bios && !now_in_bios {
            // Just left the BIOS, so stamp what it leaves behind on the bus.
            //
            // This is a CONSTANT on purpose, and the constant is the real
            // BIOS's, not ours. Reading our own image's last opcode is the
            // mechanically faithful thing and it produces values no hardware
            // ever shows, because the bundled open BIOS is a clean-room
            // rewrite whose literal pools sit in different places. Mario vs
            // Donkey Kong hangs on one of them (it saw 0x00001524, a word out
            // of a literal pool) and runs on any of the three values GBATEK
            // documents. mGBA hardcodes this same value for the same reason.
            //
            // GBATEK lists 0xE25EF004 after an IRQ (the `subs pc, lr, #4` that
            // ends the handler) and 0xE3A02004 after an SWI (a `mov r2, #4` in
            // its epilogue), so which one it is depends on how the BIOS was
            // entered. A running game leaves through the IRQ path every frame,
            // so defaulting to the SWI value for both was wrong most of the
            // time.
            self.bios_prefetch = if self.bios_entry_irq { 0xE25E_F004 } else { 0xE3A0_2004 };
        }
        self.exec_in_bios = now_in_bios;
    }
}