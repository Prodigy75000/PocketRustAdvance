// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! gba-core: a clean-room Game Boy Advance emulator, written from scratch in
//! Rust.
//!
//! The CPU ([`cpu::Arm7tdmi`]) is validated against the TomHarte ARM7TDMI
//! vectors through the [`bus::Bus`] trait. [`GbaBus`] is the real system bus it
//! drives; [`Gba`] wires the two together and runs frames.

pub mod apu;
pub mod bus;
pub mod cable;
pub mod cpu;
pub mod memory;
pub mod jit;
pub mod ppu;
pub mod rfu;
pub mod rtc;
pub mod save;
pub mod sensor;
pub mod state;

pub use memory::GbaBus;
pub use ppu::{Ppu, SCREEN_H, SCREEN_W, TOTAL_LINES};

use cpu::Arm7tdmi;

/// GBA buttons, in KEYINPUT bit order.
#[derive(Clone, Copy)]
pub enum Button {
    A = 0,
    B = 1,
    Select = 2,
    Start = 3,
    Right = 4,
    Left = 5,
    Up = 6,
    Down = 7,
    R = 8,
    L = 9,
}

/// A whole console: the CPU plus the system bus.
/// Audio output rate, and the single source of truth for it: [`Gba::SAMPLE_RATE`]
/// re-exports this and the libretro layer's AV-info reads that, so the two cannot
/// drift. It must equal `16_777_216 / apu::CYCLES_PER_SAMPLE`.
const SAMPLE_RATE: u64 = 65_536;

pub struct Gba {
    pub cpu: Arm7tdmi,
    pub bus: GbaBus,
    /// Diagnostics: how many IRQs the CPU has taken since boot.
    pub irqs_taken: u64,
    /// Diagnostics: CPU instructions executed since boot.
    pub steps: u64,
    /// Diagnostics: how many of `steps` were executed inside the 16 KB BIOS.
    ///
    /// A cartridge that crashes and restarts through the BIOS looks perfectly
    /// healthy to any colour-based liveness check, because the boot animation
    /// draws and animates: Maniac Racers Advance sits on the Normmatt logo
    /// forever and scores 16 distinct colours, the same as a game resting on a
    /// dark title screen. This says WHO is executing, which a colour count
    /// cannot. A healthy cartridge enters the BIOS only for SWIs.
    pub bios_steps: u64,
    /// Diagnostics: cycles the CPU spent halted, i.e. the game had finished its
    /// work for that stretch and was waiting for an interrupt.
    ///
    /// This is a SPEED HEADROOM measure and the only one that scales past a
    /// handful of titles. A game that keeps up finishes its frame and idles; a
    /// game the emulated CPU is starving never idles at all. Finding Mario Kart
    /// running at half speed needed the owner's save state and an on-screen race
    /// timer, which does not generalise to thousands of ROMs.
    ///
    /// Read it as a screen, not a verdict: a game that busy-polls DISPSTAT
    /// instead of halting shows zero idle while being perfectly healthy.
    pub halt_cycles: u64,
    /// When false, scanline rendering is skipped (for profiling CPU vs PPU).
    pub render_enabled: bool,
    /// A fixed-rate audio clock: it advances by exactly one scanline's worth of
    /// cycles per rendered line, independent of `bus.cycles` (which our timing
    /// model inflates whenever DMA runs). Direct Sound is paced off this so a
    /// heavy graphics-DMA frame can't over-pump the FIFO and run the sample-buffer
    /// pointer off the end (which played garbage during screen transitions).
    audio_clock: u64,
    /// Debug: when set, `run_frame` reports the first time the CPU executes from
    /// unused address space (>= 0x10000000 = definitely a crashed/runaway PC),
    /// printing the branch that jumped there. Off by default (one cheap compare
    /// per instruction when enabled, nothing otherwise).
    /// Idle-loop skipping: see [`jit::idle`]. Costs one compare per instruction
    /// while off, and saves 47% of the instructions in Pokemon.
    pub idle: jit::idle::Watch,
    /// Set false to disable idle-loop skipping outright, for A/B measurement and
    /// for bisecting a game that misbehaves.
    pub idle_skip: bool,
    pub trap_unused: bool,
    /// Debug: print the executing PC at the top of every scanline this frame.
    pub linepc: bool,
    trap_prev: u32,
    trap_from: u32,
    trapped: bool,
    /// The last 64 taken branches before the trap fired, oldest first. A runaway
    /// PC is never interesting at the point it lands: the question is always what
    /// called what to get there, and the single "prev branch from" address above
    /// answers it only when the jump is one hop deep. GTA Advance took four.
    pub trap_ring: Vec<(u32, u32)>,
}

impl Gba {
    /// Create a console with a cartridge ROM and optional BIOS. With no BIOS the
    /// CPU is booted directly into the cartridge (the post-BIOS register state).
    pub fn new(rom: Vec<u8>, bios: Vec<u8>) -> Self {
        let has_bios = !bios.is_empty();
        let mut bus = GbaBus::new(rom, bios);
        let mut cpu = Arm7tdmi::new();

        if has_bios && std::env::var_os("GBA_FULLBOOT").is_some() {
            // Full BIOS boot (reset vector, Supervisor, IRQ/FIQ masked): runs the
            // whole BIOS boot sequence + logo. Opt-in via GBA_FULLBOOT - a few
            // titles depend on the boot-time state it sets up (Pitfall Mayan
            // Adventure, Super Robot Taisen A, ...) that fast-boot skips.
            cpu.load_full(0x13 | (1 << 7) | (1 << 6), [0; 16], [0; 7], [0; 2], [0; 2], [0; 2], [0; 2], [0; 5]);
        } else if has_bios {
            // Fast/direct boot WITH the BIOS still loaded (DEFAULT): jump straight
            // to the cartridge with post-BIOS register state, skipping the BIOS
            // boot animation, but leave the BIOS image in place so SWIs run the
            // BIOS's own code and BIOS-ROM reads work. gpSP-style - no ~2 s boot
            // logo, and it rescues more games than full-boot (incl. titles that
            // loop the boot). hle_bios stays false.
            let mut r = [0u32; 16];
            r[13] = 0x0300_7F00;
            r[15] = 0x0800_0000;
            let svc = [0x0300_7FE0, 0];
            let irq = [0x0300_7FA0, 0];
            cpu.load_full(0x1F, r, [0; 7], svc, [0, 0], irq, [0; 2], [0; 5]);
        } else {
            // Direct boot: System mode at the cartridge entry, standard stacks.
            let mut r = [0u32; 16];
            r[13] = 0x0300_7F00; // user/system SP
            r[15] = 0x0800_0000; // cartridge entry
            let svc = [0x0300_7FE0, 0]; // SP_svc
            let irq = [0x0300_7FA0, 0]; // SP_irq
            cpu.load_full(0x1F, r, [0; 7], svc, [0, 0], irq, [0; 2], [0; 5]);
            cpu.hle_bios = true; // no real BIOS: emulate SWIs
        }
        cpu.reload_pipeline(&mut bus);

        Gba {
            cpu,
            bus,
            irqs_taken: 0,
            steps: 0,
            bios_steps: 0,
            halt_cycles: 0,
            render_enabled: true,
            audio_clock: 0,
            trap_unused: false,
            linepc: false,
            trap_prev: 0,
            trap_from: 0,
            idle: jit::idle::Watch::default(),
            idle_skip: true,
            trapped: false,
            trap_ring: Vec::new(),
        }
    }

    /// The audio sample rate reported to the front-end.
    pub const SAMPLE_RATE: u32 = SAMPLE_RATE as u32;

    /// Drain this frame's interleaved-stereo samples for the front-end.
    pub fn take_audio(&mut self) -> Vec<i16> {
        self.bus.apu.take_output()
    }

    /// Serialize the whole machine to a save-state blob. The ROM and BIOS are
    /// not included (the front-end already has them); everything else is.
    pub fn save_state(&self) -> Vec<u8> {
        let mut w = state::Writer::default();
        w.u32(state::MAGIC);
        w.u8(state::VERSION);
        self.cpu.serialize(&mut w);
        self.bus.serialize(&mut w);
        w.buf
    }

    /// Restore a save-state produced by [`save_state`] on the same cartridge.
    /// Returns false if the blob is not a valid state for this build.
    pub fn load_state(&mut self, data: &[u8]) -> bool {
        let mut r = state::Reader::new(data);
        let version = {
            if r.u32() != state::MAGIC {
                return false;
            }
            r.u8()
        };
        if !(state::OLDEST_VERSION..=state::VERSION).contains(&version) {
            return false;
        }
        // Components read this to migrate a field that gained meaning after the
        // state was written, rather than trusting a byte nothing ever set.
        r.version = version;
        self.cpu.deserialize(&mut r);
        self.bus.deserialize(&mut r);
        // The loaded state may be anywhere, and an armed loop head carried over
        // from before it would skip code the new state does not spin at. The
        // watch costs one scanline to rebuild.
        self.idle.forget();
        // Align the fixed audio clock to the APU's restored position (its own
        // clock domain, independent of bus.cycles) so audio resumes seamlessly
        // and deterministically. A pre-APU state leaves the APU at 0, which is
        // fine: audio simply restarts from 0 rather than bursting a backlog.
        self.audio_clock = self.bus.apu.cycle();
        !r.failed
    }

    /// Run one full frame (228 scanlines) and return the RGB555 framebuffer.
    pub fn run_frame(&mut self) -> &[u16] {
        self.run_frame_polling(&mut || Vec::new())
    }

    /// Run one frame, asking `poll` for inbound adapter packets **while the
    /// adapter is holding the clock**, not only at the frame boundary.
    ///
    /// Draining the transport once per frame and then running 16 ms with
    /// nothing arriving makes the receive queue peak at the frame edge: a peer
    /// streaming a trade can put more blocks on the wire during one of our
    /// frames than the queue can hold, and the overflow is then arithmetic
    /// rather than bad luck. Measured on hardware: a host reached the queue
    /// ceiling of 16 and discarded 14 blocks during a trade that only survived
    /// because the game retried.
    ///
    /// The reference polls in two places, once a frame and again whenever the
    /// adapter is mid-wait, and this is the second of those. `poll` must not
    /// reach back into this core; the front-end's netpacket state is separate
    /// from it for exactly this reason.
    pub fn run_frame_polling<F>(&mut self, poll: &mut F) -> &[u16]
    where
        F: FnMut() -> Vec<(u16, Vec<u8>)>,
    {
        for line in 0..TOTAL_LINES {
            if line == 0 {
                self.bus.frame_no = self.bus.frame_no.wrapping_add(1);
                // The adapter re-announces a hosted room about twice a second
                // and ages out peers it has stopped hearing from. Driven from
                // here rather than from the front-end so the offline runner
                // behaves the same way.
                if let Some(rfu) = self.bus.rfu.as_mut() {
                    rfu.frame_update();
                }
            }
            // Cable pacing, charged and checked before any of this scanline's
            // emulated work.
            //
            // A scanline boundary is the only place a child can wait. Blocking
            // where the game is mid-transfer would leave it holding one word
            // while several clocks arrive, and it would answer them all with
            // that word: a corrupted trade over a wire that looks perfect. The
            // cable's own park predicate adds the other half, that its queue is
            // empty, so a parked child has always serviced the last transfer and
            // its game has armed the next word.
            //
            // Charging here and asking immediately after means the budget being
            // tested is the one this scanline will spend, not the one the last
            // one did. A parent's `advance` never holds; it is how a parent knows
            // when to put its position on the wire.
            if self.bus.cable.is_some() && self.bus.multi_mode() {
                let per_line = ppu::CYCLES_PER_LINE as u32;
                self.bus.cable.as_mut().unwrap().advance(per_line);
                // The transport only receives when something pumps it, and while
                // this core is held nothing else will: the packet that releases
                // it arrives through `poll`.
                while self.bus.cable.as_mut().unwrap().hold() {
                    for (from, bytes) in poll() {
                        if let Some(rfu) = self.bus.rfu.as_mut() {
                            rfu.net_receive(&bytes, from);
                        }
                    }
                }
            }
            self.bus.line_cycle_base = self.bus.cycles;
            self.bus.audio_line_base = self.audio_clock;
            self.bus.ppu.begin_line(line);
            if self.linepc {
                let back = if self.cpu.thumb() { 4 } else { 8 };
                eprintln!("LINEPC L{line:<3} pc={:08X} m{:X} halted={}",
                    self.cpu.r[15].wrapping_sub(back), self.cpu.cpsr & 0xF, self.bus.halted as u8);
            }
            // Look for a wait loop once per scanline. Per instruction would
            // cost more than it saves; a loop that runs thousands of times a
            // frame is found within a line or two either way.
            // A target that is paying sticks. Once a frame, drop one that is
            // not, which lets the sampler follow a game that changes its wait
            // loop without ever re-pointing away from a good target.
            if self.idle_skip {
                if line == 0 {
                    self.idle.retire_if_idle_target_is_dead();
                }
                if !self.idle.watching() {
                    self.idle_sample();
                }
            }
            self.bus.raise_ppu_irqs(line as u16);
            self.bus.step_timers();
            self.bus.step_serial();
            // Only while the adapter is waiting on the air. Outside that window
            // the game is driving transfers itself and an early delivery buys
            // nothing, so this costs one check per scanline when idle.
            // A cable widens this to every scanline. A child's reply is
            // produced by its transport, so this is not about the reply: it is
            // the only thing that makes the transport RUN, and until it does the
            // peer's parent is blocked. 228 polls a frame puts the worst-case
            // delivery delay at 73 us against a transfer the game has budgeted
            // 343 us for.
            //
            // **Deliberately NOT gated on Multi-Player mode, which it was.**
            // That gate looked like free economy and was a bug: a child answers
            // a clock from its TRANSPORT, which is correct and mode-independent,
            // but only if something is pumping. The two players reach the link
            // menu seconds apart, so a parent clocks while its peer's game has
            // not selected the mode yet, and with the gate in place nobody on
            // that device was listening. The parent then waited out its whole
            // timeout and raised a transfer error, which Pokemon reports as a
            // bad cable. One give-up is enough: the protocol frame carries a
            // checksum, so a single FFFF poisons it.
            if self.bus.cable.is_some()
                || self.bus.rfu.as_ref().map_or(false, |r| r.awaiting_event())
            {
                for (from, bytes) in poll() {
                    if let Some(rfu) = self.bus.rfu.as_mut() {
                        rfu.net_receive(&bytes, from);
                    }
                }
            }
            if line == 160 {
                self.bus.trigger_dma(1); // V-blank DMA
            }
            if line < SCREEN_H as u32 {
                self.bus.trigger_dma(2); // H-blank DMA (approximate timing)
            }
            // Each scanline runs in two phases so DISPSTAT's H-blank status bit
            // is set for the part of the line that actually is H-blank. Running
            // all 1232 cycles in one go left bit 1 reading 0 always, and any
            // game that waits on H-blank by polling DISPSTAT spun there forever.
            let line_start = self.bus.cycles;
            // One branch in the loop body instead of three. These are armed from
            // the front-end before a run, so per scanline is often enough.
            let dbg = self.trap_unused || self.bus.watch_addr != 0;
            for phase in 0..2 {
                let target = line_start
                    + if phase == 0 { ppu::HDRAW_CYCLES as u64 } else { ppu::CYCLES_PER_LINE as u64 };
                if phase == 1 {
                    self.bus.ppu.enter_hblank();
                }
                while self.bus.cycles < target {
                    // Take a pending IRQ at the instruction boundary; taking one also
                    // wakes the CPU from a HLE Halt / IntrWait.
                    if self.bus.irq_pending() && self.cpu.irq_ready() {
                        self.take_irq_now(line);
                    }
                    if dbg {
                        self.debug_step();
                    }
                    // Leaving Halt is NOT the same condition as taking an
                    // interrupt: hardware wakes on any enabled-and-requested
                    // interrupt, whether or not IME lets the CPU vector to the
                    // handler. Gating the wake on IME would park a game forever
                    // the moment it halted with interrupts masked.
                    if self.bus.halted && (self.bus.ie & self.bus.if_) != 0 {
                        self.bus.halted = false;
                    }
                    if self.bus.halted {
                        // Parked waiting for an interrupt: skip to the end of the line
                        // (the next scanline may raise the IRQ that wakes us).
                        self.halt_cycles += target.saturating_sub(self.bus.cycles);
                        self.bus.cycles = target;
                        break;
                    }
                    // The game may be spinning on a flag only an interrupt
                    // handler sets. One compare against a cached R15; see
                    // `idle_hit` for what has to be true before it can fire.
                    if self.cpu.r[15] == self.idle.probe_r15 && self.idle_hit(target) {
                        break;
                    }
                    // One compare on the hot path, with the pipeline offset folded
                    // into the bound: R15 runs 8 bytes (ARM) or 4 (Thumb) ahead of
                    // the instruction being executed, so R15 < 0x4008 is exactly
                    // "executing inside the BIOS" for ARM and over-counts Thumb by
                    // at most the four bytes either side of the BIOS ceiling.
                    if self.cpu.r[15] < 0x4008 {
                        self.bios_steps += 1;
                    }
                    self.cpu.step(&mut self.bus);
                    self.steps += 1;
                }
            }
            if line < SCREEN_H as u32 && self.render_enabled {
                self.bus.ppu.render_line(line as usize);
            }
            // Generate this line's audio (one stereo sample per 512 cycles) from
            // the APU state as it stands after the line's CPU work, then let the
            // sound DMA top up any Direct Sound FIFO that has drained. Paced off
            // the fixed audio clock, not bus.cycles, so DMA can't inflate the rate.
            self.audio_clock = self.bus.audio_line_base + ppu::CYCLES_PER_LINE as u64;
            let periods = [self.bus.ds_timer_period(0), self.bus.ds_timer_period(1)];
            self.bus.apu.generate(periods, self.audio_clock);
            self.bus.refill_fifos();
        }

        &self.bus.ppu.framebuffer
    }

    /// The per-step bookkeeping that only an armed debug feature needs: the
    /// runaway-PC trap ring (`GBA_TRAP`) and the watchpoint's PC/mode capture
    /// (`GBA_WATCHW`). Reached through a single flag tested once per step.
    ///
    /// This used to be two separate `if`s inside the loop, which looks free:
    /// each is one compare against a flag that is normally off. It is not the
    /// branches that cost, it is the code. Measured on a LeafGreen battle scene
    /// (`out/lg-battle.state`) over interleaved A/B pairs with identical
    /// instructions per frame, this shape is worth **6.3%** of CPU time, 628 to
    /// 589 us/frame, against an 8.8% ceiling from deleting the checks outright.
    /// Taken one at a time none of them measures as expensive and one measures
    /// as negative, because what actually moves is the size of the loop body.
    ///
    /// A two-loop split (one fast, one with the bookkeeping, chosen per phase)
    /// was tried first and came out **29% SLOWER**, so the gain is not simply
    /// "fewer instructions in the loop" and the shape has to be measured rather
    /// than reasoned about.
    ///
    /// Note the ordering changed with the merge: the watchpoint capture now runs
    /// before the halt check rather than after it, so a halted step updates
    /// `cur_pc`. Nothing observes that, because a halted CPU makes no bus access
    /// and the capture exists only to label one.
    #[inline(never)]
    fn debug_step(&mut self) {
        if self.trap_unused && !self.trapped {
            self.trap_check();
        }
        // The executing instruction sits two fetches behind R15.
        if self.bus.watch_addr != 0 {
            let back = if self.cpu.thumb() { 4 } else { 8 };
            self.bus.cur_pc = self.cpu.r[15].wrapping_sub(back);
            self.bus.cur_mode = self.cpu.cpsr & 0xF;
        }
    }

    /// An arrival at the watched loop head.
    ///
    /// Armed, this abandons the rest of the scanline phase: the clock still
    /// advances to exactly where it would have, so the emulated time is
    /// unchanged and only the pointless spinning is skipped. Returns true to
    /// break the caller's loop.
    ///
    /// The skip reaches the end of the PHASE and no further, then the loop is
    /// re-entered and re-tested. That bound is what makes this safe rather than
    /// lucky: it cannot deadlock, it cannot run past an interrupt, and a loop
    /// polling something that changes with time still resolves, at scanline
    /// granularity.
    ///
    /// Not yet armed, this is where the no-progress proof is collected.
    #[inline(never)]
    fn idle_hit(&mut self, target: u64) -> bool {
        if !self.idle.consider(self.steps, &self.cpu.r, self.cpu.cpsr) {
            return false;
        }
        self.idle.skipped(target.saturating_sub(self.bus.cycles));
        self.bus.cycles = target;
        true
    }

    /// Look for a wait loop around wherever the CPU currently is.
    ///
    /// Thumb only. Every idle loop measured so far is Thumb, and the ARM case
    /// costs a second decoder for no evidence of benefit yet.
    fn idle_sample(&mut self) {
        if !self.cpu.thumb() {
            return;
        }
        let exec = self.cpu.r[15].wrapping_sub(4);
        // Borrow the bus immutably for the scan, then let it go before touching
        // the watch state.
        let bus = &self.bus;
        let found = jit::idle::find_loop(exec, |a| bus.code_peek16(a));
        if let Some(l) = found {
            // R15 reads two halfwords ahead of the instruction executing, so
            // this is what the hot compare will see at the loop head.
            self.idle.watch(l.head.wrapping_add(4), l.len() as u64);
        }
    }

    /// Take a pending IRQ. Out of line on purpose: an IRQ fires a handful of
    /// times per frame, so the source histogram and the optional trace are pure
    /// code size sitting in a loop that runs 83,000 times per frame, and the
    /// size of that loop body is worth measurable throughput.
    #[inline(never)]
    fn take_irq_now(&mut self, line: u32) {
        let pend = self.bus.ie & self.bus.if_;
        for b in 0..16 {
            if pend & (1 << b) != 0 {
                self.bus.dbg_irq_src[b] += 1;
            }
        }
        if self.linepc {
            eprintln!("IRQTAKE L{line} cyc_in_line={} pend={:04X} from={:08X}",
                self.bus.cycles - self.bus.line_cycle_base, pend,
                self.cpu.r[15].wrapping_sub(if self.cpu.thumb() {4} else {8}));
        }
        self.cpu.take_irq(&mut self.bus);
        self.bus.halted = false;
        self.irqs_taken += 1;
    }

    /// `GBA_TRAP`: watch for the PC running away into unmapped space, keeping a
    /// ring of the branches that led there. Out of line for the same reason
    /// [`Gba::take_irq_now`] is.
    #[inline(never)]
    fn trap_check(&mut self) {
                    let back = if self.cpu.thumb() { 4 } else { 8 };
                    let exec = self.cpu.r[15].wrapping_sub(back);
                    let width = if self.cpu.thumb() { 2 } else { 4 };
                    if exec >= 0x1000_0000 {
                        eprintln!(
                            "TRAP: PC jumped into unused space: {:08X} -> {exec:08X} (prev branch from {:08X}, irqs={})",
                            self.trap_prev, self.trap_from, self.irqs_taken
                        );
                        let r = &self.cpu.r;
                        eprintln!(
                            "TRAP: r0-r7  {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X} {:08X}",
                            r[0], r[1], r[2], r[3], r[4], r[5], r[6], r[7]
                        );
                        eprintln!(
                            "TRAP: r8-r15 {:08X} {:08X} {:08X} {:08X} {:08X} sp={:08X} lr={:08X} pc={:08X}",
                            r[8], r[9], r[10], r[11], r[12], r[13], r[14], r[15]
                        );
                        eprintln!("TRAP: the {} branches that led there:", self.trap_ring.len());
                        for (from, to) in self.trap_ring.iter() {
                            eprintln!("TRAP:   {from:08X} -> {to:08X}");
                        }
                        self.trapped = true;
                    } else {
                        if self.trap_prev != 0 && exec != self.trap_prev.wrapping_add(width) {
                            self.trap_from = self.trap_prev; // last in-range branch source
                            self.trap_ring.push((self.trap_prev, exec));
                            if self.trap_ring.len() > 64 {
                                self.trap_ring.remove(0);
                            }
                        }
                        self.trap_prev = exec;
                    }
    }

    /// Which motion sensor the cartridge carries, if any. The front-end asks
    /// this so it only turns the device's sensors on for a game that reads them.
    pub fn cart_sensor(&self) -> crate::sensor::CartSensor {
        self.bus.sensors.kind
    }

    /// Feed the accelerometer, in m/s^2. Used by the tilt carts (Yoshi
    /// Topsy-Turvy, Yoshi's Universal Gravitation, Koro Koro Puzzle).
    pub fn set_accelerometer(&mut self, x: f32, y: f32, z: f32) {
        self.bus.sensors.set_accelerometer(x, y, z);
    }

    /// True when the cartridge carries a real-time clock, so a front-end knows
    /// whether it needs to push the wall clock at all.
    pub fn cart_has_rtc(&self) -> bool {
        self.bus.sensors.rtc.present
    }

    /// True when the cartridge has a rumble motor, so a front-end knows whether
    /// to ask for the host's rumble interface at all.
    pub fn cart_has_rumble(&self) -> bool {
        self.bus.sensors.has_rumble
    }

    /// Is the cartridge driving its motor right now? Read once per frame and
    /// hand to the host; the game toggles it far faster than a phone's motor
    /// can follow, so this is a level, not an event.
    pub fn rumble_on(&mut self) -> bool {
        self.bus.sensors.take_rumble()
    }

    /// Push the wall clock, in seconds since the Unix epoch, **already shifted
    /// into the player's local time**.
    ///
    /// The conversion is the front-end's job on purpose: this core carries no
    /// timezone database and should not grow one, while the Android side
    /// already knows the user's zone. Push it every frame; a cartridge clock
    /// latches on demand and a stale value shows up as a game whose day never
    /// turns.
    pub fn set_rtc_unix_time(&mut self, secs: i64) {
        self.bus.sensors.rtc.unix_time = secs;
    }

    /// Diagnostics: how many times the game has latched the clock. Zero after a
    /// run separates "the clock is wrong" from "the game never asked", which
    /// look the same from outside.
    pub fn rtc_latches(&self) -> u64 {
        self.bus.sensors.rtc.latches
    }

    /// Is a wireless adapter attached to this cartridge?
    pub fn cart_has_rfu(&self) -> bool {
        self.bus.rfu.is_some()
    }

    /// Commands the game has sent the adapter, times it has power-cycled it,
    /// and where the adapter currently is. Same purpose as `rtc_latches`:
    /// zero commands separates "the adapter is wrong" from "the game never
    /// asked", and resets climbing while commands stay put is specifically a
    /// handshake that never completes.
    pub fn rfu_stats(&self) -> Option<(u64, u64, &'static str)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.commands, r.resets, r.state_name()))
    }

    /// Attach a link cable. The serial port then models a real Multi-Player
    /// bus instead of a lone unit with nothing plugged in.
    ///
    /// Deliberately NOT done from the cartridge the way the wireless adapter is:
    /// a cable is a property of the session, not of the game, so it goes on when
    /// a netplay session starts and comes off when it stops.
    pub fn connect_cable(&mut self, cable: Box<dyn crate::cable::LinkCable>) {
        self.bus.cable = Some(cable);
    }

    /// Take the cable away, leaving the port exactly as it is with nothing
    /// attached.
    pub fn disconnect_cable(&mut self) {
        self.bus.detach_cable();
    }

    pub fn cable_attached(&self) -> bool {
        self.bus.cable.is_some()
    }

    /// Does this cartridge use the wireless adapter? The two devices sit on the
    /// same port, so a session attaches one or the other, never both.
    pub fn rfu_attached(&self) -> bool {
        self.bus.rfu.is_some()
    }

    /// Transfers the cable completed, and transfers it gave up on. Zero
    /// completions separates "the cable is wrong" from "the game never asked",
    /// which is the distinction every silent failure in the adapter work turned
    /// out to need.
    pub fn cable_stats(&self) -> (u64, u64) {
        (self.bus.cable_transfers, self.bus.cable_failures)
    }

    /// Rolling hash of the words every completed transfer landed. Both ends must
    /// agree at an equal transfer count, which is the only way to tell a healthy
    /// wire carrying the wrong data from one carrying the right data.
    pub fn cable_wordsum(&self) -> u64 {
        self.bus.cable_wordsum
    }

    /// The hash at the last exact checkpoint, as `(transfer count, hash)`. The
    /// pair both devices can be compared on without lining anything up by hand.
    pub fn cable_mark(&self) -> (u64, u64) {
        (self.bus.cable_mark_at, self.bus.cable_mark_sum)
    }

    /// Tell the adapter which peer the frontend thinks we are. The device id
    /// it advertises is derived from this, so without it two devices running
    /// the same ROM advertise the same id and refuse to join each other.
    pub fn rfu_set_self_id(&mut self, id: u16) {
        if let Some(rfu) = self.bus.rfu.as_mut() {
            rfu.set_self_id(id);
        }
    }

    /// Hand the adapter a packet that arrived from `from`, the frontend id of
    /// the peer that sent it.
    pub fn rfu_net_receive(&mut self, buf: &[u8], from: u16) {
        if let Some(rfu) = self.bus.rfu.as_mut() {
            rfu.net_receive(buf, from);
        }
    }

    /// Take the packets the adapter wants put on the wire.
    pub fn rfu_take_outbox(&mut self) -> Vec<crate::rfu::OutPacket> {
        match self.bus.rfu.as_mut() {
            Some(rfu) => rfu.take_outbox(),
            None => Vec::new(),
        }
    }

    /// Is the adapter hosting a room or attached to one?
    pub fn rfu_in_session(&self) -> bool {
        self.bus.rfu.as_ref().is_some_and(|r| r.in_session())
    }

    /// Peers heard from, sessions formed, and packets dropped for want of
    /// queue space. A drop in the middle of a trade shifts every later block,
    /// so it is counted rather than swallowed.
    pub fn rfu_session_stats(&self) -> Option<(u64, u64, u64)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.peers_seen, r.connections, r.dropped))
    }

    /// Bitmasks of every command code the game has issued, and of the ones we
    /// do not implement. Two numbers that say exactly which of the protocol a
    /// game drives, which cannot be learned any other way on a device.
    pub fn rfu_command_masks(&self) -> Option<(u64, u64)> {
        self.bus.rfu.as_ref().map(|r| (r.cmd_seen, r.unknown_seen))
    }

    /// Connection accounting: asked, no-peer, sent, nacked, requests received,
    /// requests refused. "The connect failed" has several distinct causes that
    /// need opposite fixes, and none of them errors on a device.
    /// What the adapter reported while holding the clock (timeout, event,
    /// disconnection) and how many blocks actually crossed the air.
    /// Clients attached right now, peers heard right now, and why clients
    /// have left: added, timed out, told to go, wiped by a restart.
    pub fn rfu_client_life(&self) -> Option<(u32, u32, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            let (clients, peers) = r.live_counts();
            (clients, peers, r.cl_added, r.cl_timeout, r.cl_told, r.cl_wiped)
        })
    }

    pub fn rfu_wait_stats(&self) -> Option<(u64, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            (r.resp_timeout, r.resp_data, r.resp_disc, r.blocks_out, r.blocks_in)
        })
    }

    /// Retransmits performed, and blocks the game asked to send that did not
    /// go out: no client attached, or a length past what the medium carries.
    pub fn rfu_tx_stats(&self) -> Option<(u64, u64, u64)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.tx_rtx, r.tx_no_client, r.tx_too_long))
    }

    /// The arrival side of the link: queued, rejected, dropped by role, and
    /// the deepest any queue has been.
    pub fn rfu_rx_stats(&self) -> Option<(u64, u64, u64, u64, u32)> {
        self.bus.rfu.as_ref().map(|r| {
            (
                r.rx_queued,
                r.rx_reject,
                r.drop_host,
                r.drop_client,
                r.queue_max,
            )
        })
    }

    /// What the flash save has been asked to do. A non-zero chip-erase count on
    /// a Gen 3 game means the save was wiped by a COMMAND rather than the
    /// buffer having been reallocated, and those two are impossible to tell
    /// apart from the file: both are all 0xFF.
    pub fn flash_stats(&self) -> Option<(u64, u64, u64, u64, u32, u8)> {
        let s = &self.bus.save;
        Some((
            s.flash_chip_erases,
            s.flash_sector_erases,
            s.flash_programs,
            s.flash_bank_sets,
            s.flash_last_sector,
            s.flash_bank_now,
        ))
    }

    /// Why a client lost its link: the game asked, the host told it, or the
    /// host went silent. The mirror of `rfu_client_life`, which only covers how
    /// a HOST loses a client.
    pub fn rfu_client_left(&self) -> Option<(u64, u64, u64)> {
        self.bus
            .rfu
            .as_ref()
            .map(|r| (r.cl_left_self, r.cl_left_told, r.cl_left_silent))
    }

    pub fn rfu_connect_stats(&self) -> Option<(u64, u64, u64, u64, u64, u64)> {
        self.bus.rfu.as_ref().map(|r| {
            (
                r.conn_asked,
                r.conn_no_peer,
                r.conn_sent,
                r.conn_nacked,
                r.req_got,
                r.req_refused,
            )
        })
    }

    /// Feed the gyroscope, in rad/s. Used by WarioWare Twisted, which senses
    /// rotation about Z only.
    pub fn set_gyroscope(&mut self, x: f32, y: f32, z: f32) {
        self.bus.sensors.set_gyroscope(x, y, z);
    }

    /// Set a button's pressed state (KEYINPUT is active-low).
    pub fn set_button(&mut self, button: Button, pressed: bool) {
        let bit = 1u16 << (button as u16);
        if pressed {
            self.bus.keyinput &= !bit;
        } else {
            self.bus.keyinput |= bit;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A cart with a GPIO port keeps the first 0x100 bytes of its ROM window
    /// off the linear fast path, because the port lives at 0x080000C4 and those
    /// halfwords are not ROM reads. The fetch window therefore starts past the
    /// header, and it has to point at a correspondingly shifted slice: moving
    /// the start without moving the slice reads 0x100 bytes too early.
    ///
    /// That was a real bug and only a framebuffer hash over 80 ROMs caught it,
    /// on the single solar cart that happened to be in the sample. This makes
    /// it a unit test so the next person does not need the same luck.
    ///
    /// The same program is run on a cart WITHOUT a port, so the assertion is
    /// that the two paths agree rather than that one of them returns 7.
    #[test]
    fn a_gpio_cart_fetches_rom_from_the_right_offset() {
        fn run(code: &[u8; 4]) -> (u32, bool) {
            let mut rom = vec![0u8; 0x4000];
            rom[0xAC..0xB0].copy_from_slice(code);
            // MOV R0, #7 ; B .   at 0x08001000, well past the header.
            rom[0x1000..0x1004].copy_from_slice(&0xE3A0_0007u32.to_le_bytes());
            rom[0x1004..0x1008].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
            let mut gba = Gba::new(rom, Vec::new());
            gba.render_enabled = false;
            let gpio = gba.cart_sensor() == crate::sensor::CartSensor::Gyro;
            gba.cpu.r[0] = 0;
            gba.cpu.r[15] = 0x0800_1000;
            gba.cpu.reload_pipeline(&mut gba.bus);
            for _ in 0..4 {
                gba.cpu.step(&mut gba.bus);
            }
            (gba.cpu.r[0], gpio)
        }

        let (plain, plain_gpio) = run(b"AAAA");
        let (gyro, gyro_gpio) = run(b"RAAA");
        assert!(!plain_gpio, "the control cart must have no GPIO port");
        assert!(gyro_gpio, "a game code starting R is a gyro cart, which has one");
        assert_eq!(plain, 7, "plain cart: ROM fetch at 0x08001000");
        assert_eq!(gyro, plain, "a GPIO cart must fetch the same instruction");
    }

    /// The instruction fetch has its own path on the bus now, with a window
    /// that skips the region decode. The window holds an address RANGE and a
    /// slice selector, never a copy of the code, and that is the whole reason
    /// self-modifying code needs no invalidation. Games copy routines into
    /// IWRAM and rewrite them, so prove it: run an instruction, overwrite it in
    /// place, run it again, and the second result must differ.
    ///
    /// A fetch cache that copied opcodes would pass the first half and fail the
    /// second, which is exactly the regression a translation cache can
    /// introduce later.
    #[test]
    fn code_rewritten_in_iwram_executes_the_new_opcode() {
        use crate::bus::{Access, Bus};

        fn run_from(gba: &mut Gba, at: u32) -> u32 {
            gba.cpu.r[0] = 0;
            gba.cpu.r[15] = at;
            gba.cpu.reload_pipeline(&mut gba.bus);
            for _ in 0..4 {
                gba.cpu.step(&mut gba.bus);
            }
            gba.cpu.r[0]
        }

        let mut gba = Gba::new(vec![0u8; 0x200], Vec::new());
        gba.render_enabled = false;
        // MOV R0, #1 ; B .
        gba.bus.write32(0x0300_0000, 0xE3A0_0001, Access::NonSeq);
        gba.bus.write32(0x0300_0004, 0xEAFF_FFFE, Access::NonSeq);
        assert_eq!(run_from(&mut gba, 0x0300_0000), 1);

        // Overwrite the instruction where it sits.
        gba.bus.write32(0x0300_0000, 0xE3A0_0002, Access::NonSeq);
        assert_eq!(
            run_from(&mut gba, 0x0300_0000),
            2,
            "the fetch must see the rewritten instruction, not a cached copy"
        );

        // IWRAM is 32 KB mirrored across its 16 MB region, so the same code has
        // to execute from a mirror. This catches a window installed with the
        // wrong base, which a correct-looking in-range test cannot.
        assert_eq!(
            run_from(&mut gba, 0x0300_8000),
            2,
            "an IWRAM mirror must fetch the same bytes"
        );
    }

    /// The per-step debug bookkeeping moved behind a single flag computed once
    /// per scanline instead of two compares per instruction, which is worth
    /// about 6% of CPU time. That makes the arming itself the thing that can
    /// silently break: a flag that never turns on leaves every debug feature
    /// looking present and doing nothing.
    ///
    /// Two runs of the same ROM, which branches into unmapped space. Unarmed it
    /// must NOT trap, so this cannot pass by the trap firing unconditionally.
    #[test]
    fn an_armed_trap_still_catches_a_runaway_pc() {
        // MOV R0, #0x1000_0000 ; BX R0
        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xE3A0_0201u32.to_le_bytes());
        rom[4..8].copy_from_slice(&0xE12F_FF10u32.to_le_bytes());

        let mut off = Gba::new(rom.clone(), Vec::new());
        off.render_enabled = false;
        off.run_frame();
        assert!(!off.trapped, "the trap is opt-in; it must stay quiet unarmed");

        let mut on = Gba::new(rom, Vec::new());
        on.trap_unused = true;
        on.render_enabled = false;
        on.run_frame();
        assert!(on.trapped, "an armed trap must still catch a PC in unmapped space");
    }

    /// Same risk for the watchpoint half of that flag: it labels a write with
    /// the PC that made it, and a flag that never arms would report nothing
    /// while looking armed.
    ///
    /// The ROM is one instruction branching to itself, so the executing PC is
    /// 0x08000000 for the whole frame and the expected value is exact rather
    /// than "wherever the frame happened to end".
    #[test]
    fn an_armed_watchpoint_still_learns_the_pc() {
        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes()); // B .

        let mut off = Gba::new(rom.clone(), Vec::new());
        off.render_enabled = false;
        off.run_frame();
        assert_eq!(off.bus.cur_pc, 0, "no watch set, so nothing to label");

        let mut on = Gba::new(rom, Vec::new());
        on.bus.watch_addr = 0x0300_0000;
        on.render_enabled = false;
        on.run_frame();
        assert_eq!(
            on.bus.cur_pc, 0x0800_0000,
            "an armed watchpoint must still track the executing PC"
        );
    }

    /// Two cores wired together must actually move a word in both directions.
    ///
    /// The desk harness: the register model is exercised the way a game drives
    /// it (mode, word, start bit) rather than through the cable's own API, so
    /// this fails if the masking, the pacing, the slot assignment or the IRQ is
    /// wrong, and it needs no network and no second device. Every link bug
    /// PocketRust shipped was reproducible this way.
    #[test]
    fn two_cores_exchange_a_word_over_the_cable() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes()); // B .
        let (parent_end, child_end) = crate::cable::local_pair();
        let mut parent = Gba::new(rom.clone(), Vec::new());
        let mut child = Gba::new(rom, Vec::new());
        parent.connect_cable(Box::new(parent_end));
        child.connect_cable(Box::new(child_end));
        parent.render_enabled = false;
        child.render_enabled = false;

        // Both games enter Multi-Player mode at 115200 baud with the serial
        // interrupt enabled, which is what Pokemon's link code does.
        const MULTI: u16 = 0x2000 | 0x4000 | 3;
        for g in [&mut parent, &mut child] {
            g.bus.write16(0x0400_0134, 0x0000, N); // RCNT: these are serial pins
            g.bus.write16(0x0400_0128, MULTI, N);
        }

        // Before any transfer, each game has to be able to read which end it is
        // and whether its peer is there.
        assert_eq!(
            parent.bus.read16(0x0400_0128, N) & 0x003C,
            0x0008,
            "the parent reads SI low, ID 0 and SD high"
        );
        assert_eq!(
            child.bus.read16(0x0400_0128, N) & 0x003C,
            0x001C,
            "the child reads SI high, ID 1 and SD high"
        );

        // The child presents its word, then the parent clocks.
        child.bus.write16(0x0400_012A, 0xB9A0, N);
        parent.bus.write16(0x0400_012A, 0x8FFF, N);
        parent.bus.write16(0x0400_0128, MULTI | 0x0080, N);
        assert_eq!(
            parent.bus.read16(0x0400_0128, N) & 0x0080,
            0x0080,
            "the transfer is paced, so it is still busy the instant it starts"
        );

        parent.run_frame();
        assert_eq!(parent.bus.read16(0x0400_0120, N), 0x8FFF, "parent slot 0 is its own word");
        assert_eq!(parent.bus.read16(0x0400_0122, N), 0xB9A0, "parent slot 1 is the child's");
        assert_eq!(parent.bus.read16(0x0400_0124, N), 0xFFFF, "no third unit");
        let cnt = parent.bus.read16(0x0400_0128, N);
        assert_eq!(cnt & 0x0080, 0, "busy clears when the transfer lands");
        assert_eq!(cnt & 0x0040, 0, "and it did not land as an error");
        assert_ne!(parent.bus.if_ & 0x80, 0, "a completed transfer raises the serial IRQ");
        assert_eq!(parent.cable_stats(), (1, 0));

        // The child is driven by the parent's clock, not by its own game.
        child.run_frame();
        assert_eq!(child.bus.read16(0x0400_0120, N), 0x8FFF, "the child sees the parent's word");
        assert_eq!(child.bus.read16(0x0400_0122, N), 0xB9A0, "and its own in its own slot");
        assert_eq!(child.bus.read16(0x0400_0128, N) & 0x0080, 0, "the child is idle again");
        assert_ne!(child.bus.if_ & 0x80, 0, "both ends interrupt on the same transfer");
        assert_eq!(child.cable_stats(), (1, 0));
    }

    /// A transfer has to occupy the time its baud rate says, and no more.
    ///
    /// The lone-unit path completes inside the store, which is safe because there
    /// is nobody to stay in step with. Over a cable it is not: a game can poll the
    /// busy bit, and both ends compute this window from the same table, so a
    /// transfer that completed instantly would let a parent clock faster than its
    /// peer can answer.
    ///
    /// **A floor was added here to hide the network round trip, and the game
    /// rejected it on device.** Nine transfers have to fit in one 280896-cycle
    /// frame, so each must stay under 1.86 ms, while hiding an 8 ms round trip
    /// needs over 8 ms. The wall-clock wait is handled by stretching the frame on
    /// both devices instead. So this asserts hardware timing: 5755 cycles at
    /// 115200 baud with two units, which is five scanlines of 1232.
    #[test]
    fn a_transfer_occupies_the_time_its_baud_rate_says() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (parent_end, child_end) = crate::cable::local_pair();
        let mut gba = Gba::new(rom, Vec::new());
        gba.connect_cable(Box::new(parent_end));
        let mut child = child_end;
        crate::cable::LinkCable::set_output(&mut child, 0x4321);

        const MULTI: u16 = 0x2000 | 3; // Multi-Player, 115200 baud
        gba.bus.write16(0x0400_0134, 0x0000, N);
        gba.bus.write16(0x0400_0128, MULTI, N);
        gba.bus.write16(0x0400_012A, 0x1234, N);
        gba.bus.write16(0x0400_0128, MULTI | 0x0080, N);

        let mut lines = 0;
        while gba.bus.read16(0x0400_0128, N) & 0x0080 != 0 {
            gba.bus.cycles += 1232; // one scanline
            gba.bus.step_serial();
            lines += 1;
            assert!(lines <= 300, "the transfer never completed");
        }
        assert_eq!(lines, 5, "5755 cycles is five scanlines of 1232");
        // Nine of these have to fit in a frame, which is the constraint that
        // killed the floor. Asserted in absolute numbers on purpose.
        assert!(5755 * 9 < 280_896, "nine transfers must fit one frame");
    }

    /// A child that reaches the link menu late must not inherit the transfers
    /// its peer sent while it was still elsewhere.
    ///
    /// Both players press A seconds apart, so a parent clocks handshake attempts
    /// for a while before the other game is listening. Delivering those once the
    /// child arrives would hand it interrupts for a protocol it has not started
    /// and make it report a backlog that looks like a device falling behind.
    /// Selecting Multi-Player mode is the port being set up, so that is where
    /// they go.
    #[test]
    fn selecting_multi_player_mode_drops_what_was_in_flight_before_it() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (parent_end, child_end) = crate::cable::local_pair();
        let mut parent = Gba::new(rom.clone(), Vec::new());
        let mut child = Gba::new(rom, Vec::new());
        parent.connect_cable(Box::new(parent_end));
        child.connect_cable(Box::new(child_end));
        parent.render_enabled = false;
        child.render_enabled = false;

        const MULTI: u16 = 0x2000 | 0x4000 | 3;
        parent.bus.write16(0x0400_0134, 0x0000, N);
        parent.bus.write16(0x0400_0128, MULTI, N);

        // Three clocks while the child is still in its own menus, not linking.
        for w in [0xB9A0u16, 0xB9A0, 0xB9A0] {
            parent.bus.write16(0x0400_012A, w, N);
            parent.bus.write16(0x0400_0128, MULTI | 0x0080, N);
            parent.run_frame();
        }
        assert_eq!(parent.cable_stats().0, 3, "the parent really did clock three times");

        // Now the child arrives and selects Multi-Player mode.
        child.bus.write16(0x0400_0134, 0x0000, N);
        child.bus.write16(0x0400_0128, MULTI, N);
        child.bus.if_ = 0;
        child.run_frame();
        assert_eq!(
            child.cable_stats(),
            (0, 0),
            "none of the three may land: they belong to a protocol this game had not begun"
        );
        assert_eq!(child.bus.if_ & 0x80, 0, "and no interrupt for a transfer it never took part in");

        // A transfer clocked after it arrived does land.
        parent.bus.write16(0x0400_012A, 0x8FFF, N);
        child.bus.write16(0x0400_012A, 0x1234, N);
        parent.bus.write16(0x0400_0128, MULTI | 0x0080, N);
        parent.run_frame();
        child.run_frame();
        assert_eq!(child.cable_stats(), (1, 0), "the cable is not broken, only drained");
        assert_eq!(child.bus.read16(0x0400_0120, N), 0x8FFF);
    }

    /// A whole Pokemon link frame, nine transfers, driven the way the game
    /// drives it.
    ///
    /// One transfer proves the plumbing; a sequence proves it keeps working.
    /// This is the shape gpSP documents for the Pokemon protocol: a checksum
    /// word followed by eight data words, with each side writing its next word
    /// from the interrupt of the previous transfer. It catches what a single
    /// exchange cannot: a sequence number that stops matching, a busy bit that
    /// is not clear in time for the next clock, a child queue that drifts
    /// behind, and an interrupt that fires once and then stops.
    #[test]
    fn a_nine_transfer_pokemon_frame_runs_end_to_end() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (parent_end, child_end) = crate::cable::local_pair();
        let mut parent = Gba::new(rom.clone(), Vec::new());
        let mut child = Gba::new(rom, Vec::new());
        parent.connect_cable(Box::new(parent_end));
        child.connect_cable(Box::new(child_end));
        parent.render_enabled = false;
        child.render_enabled = false;

        const MULTI: u16 = 0x2000 | 0x4000 | 3;
        for g in [&mut parent, &mut child] {
            g.bus.write16(0x0400_0134, 0x0000, N);
            g.bus.write16(0x0400_0128, MULTI, N);
        }

        // Transfer 0 is the checksum word, 1..=8 are the frame's data.
        let parent_words: [u16; 9] = [0x0000, 0x8FFF, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E];
        let child_words: [u16; 9] = [0x0000, 0xB9A0, 0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E];

        for i in 0..9 {
            // Each side presents its word, exactly as its serial interrupt
            // handler would.
            parent.bus.write16(0x0400_012A, parent_words[i], N);
            child.bus.write16(0x0400_012A, child_words[i], N);
            parent.bus.if_ = 0;
            child.bus.if_ = 0;

            parent.bus.write16(0x0400_0128, MULTI | 0x0080, N);
            parent.run_frame();
            child.run_frame();

            assert_eq!(
                (parent.bus.read16(0x0400_0120, N), parent.bus.read16(0x0400_0122, N)),
                (parent_words[i], child_words[i]),
                "transfer {i}: the parent must see both words"
            );
            assert_eq!(
                (child.bus.read16(0x0400_0120, N), child.bus.read16(0x0400_0122, N)),
                (parent_words[i], child_words[i]),
                "transfer {i}: and the child must see the same pair, in the same slots"
            );
            assert_eq!(
                parent.bus.read16(0x0400_0128, N) & 0x00C0,
                0,
                "transfer {i}: neither busy nor an error once it has landed"
            );
            assert_ne!(parent.bus.if_ & 0x80, 0, "transfer {i}: the parent interrupts");
            assert_ne!(child.bus.if_ & 0x80, 0, "transfer {i}: so does the child");
        }

        assert_eq!(parent.cable_stats(), (9, 0), "nine completed, none lost");
        assert_eq!(child.cable_stats(), (9, 0));
        // The instrument the device run needed: both ends hash the words they
        // landed, in order, so an equal count with unequal hashes means a healthy
        // wire carried the wrong data. Tested here so the number means something
        // when it comes off a phone.
        assert_eq!(
            parent.cable_wordsum(),
            child.cable_wordsum(),
            "the two ends must have landed the same words in the same order"
        );
    }

    /// The core's pacing hook: charge every scanline, and pump the transport
    /// while held.
    ///
    /// The second half is the one that can fail silently. A held child is waiting
    /// for a packet that only arrives when something calls the frontend's poll,
    /// and if the hold loop does not do that, the only thing that can release it
    /// is its own patience running out. The link would then work, slowly, and
    /// with a desync at the end of every wait, which is close to the hardest
    /// possible symptom to attribute.
    #[test]
    fn a_held_core_charges_every_scanline_and_pumps_while_it_waits() {
        use crate::bus::{Access::NonSeq as N, Bus};
        use std::cell::RefCell;
        use std::rc::Rc;

        #[derive(Default)]
        struct Script {
            advances: Vec<u32>,
            holds_left: u32,
            asks: u32,
        }

        struct ScriptedCable(Rc<RefCell<Script>>);

        impl crate::cable::LinkCable for ScriptedCable {
            fn units(&self) -> u8 {
                2
            }
            fn id(&self) -> u8 {
                1 // a child: the only role that is ever held
            }
            fn set_output(&mut self, _word: u16) {}
            fn parent_start(&mut self, _own: u16) {}
            fn parent_result(&mut self) -> Option<[u16; crate::cable::MAX_UNITS]> {
                None
            }
            fn child_clock(&mut self) -> Option<(u16, u16)> {
                None
            }
            fn flush(&mut self) {}
            fn advance(&mut self, cycles: u32) {
                self.0.borrow_mut().advances.push(cycles);
            }
            fn hold(&mut self) -> bool {
                let mut s = self.0.borrow_mut();
                s.asks += 1;
                if s.holds_left == 0 {
                    return false;
                }
                s.holds_left -= 1;
                true
            }
        }

        // Returns what the cable saw, how often the transport was pumped, and the
        // emulated cycles the frame cost.
        let run = |holds: u32| -> (Script, u32, u64) {
            let script = Rc::new(RefCell::new(Script { holds_left: holds, ..Script::default() }));
            let mut rom = vec![0u8; 0x200];
            rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
            let mut gba = Gba::new(rom, Vec::new());
            gba.render_enabled = false;
            gba.connect_cable(Box::new(ScriptedCable(Rc::clone(&script))));
            gba.bus.write16(0x0400_0134, 0x0000, N);
            gba.bus.write16(0x0400_0128, 0x2000 | 3, N); // Multi-Player, 115200

            let polls = Rc::new(RefCell::new(0u32));
            let p = Rc::clone(&polls);
            let before = gba.bus.cycles;
            gba.run_frame_polling(&mut || {
                *p.borrow_mut() += 1;
                Vec::new()
            });
            let seen = std::mem::take(&mut *script.borrow_mut());
            let pumped = *polls.borrow();
            let spent = gba.bus.cycles - before;
            (seen, pumped, spent)
        };

        let (s, polls, cycles) = run(5);
        // The control: the same core, nothing held. A frame costs slightly more
        // than 280896 cycles because the last instruction of each half-scanline
        // can overshoot its target, so the number to compare against is this run,
        // not the nominal frame length.
        let (_, control_polls, control_cycles) = run(0);
        assert_eq!(s.advances.len(), 228, "one charge per scanline of the frame");
        assert!(
            s.advances.iter().all(|&c| c == 1232),
            "and each charge is a scanline of 1232 cycles"
        );
        assert_eq!(s.asks, 228 + 5, "asked once per scanline, plus once per hold");
        assert_eq!(
            polls, control_polls + 5,
            "the transport must be pumped inside the hold, or nothing can release it"
        );
        assert_eq!(
            cycles, control_cycles,
            "and holding costs wall time, never emulated time"
        );
    }

    /// Nine transfers inside ONE emulated frame, at the interval the game uses.
    ///
    /// This is the case connected mode actually runs and the one nothing tested.
    /// `a_nine_transfer_pokemon_frame_runs_end_to_end` drives nine transfers
    /// across nine FRAMES, which is the handshake's cadence, not the trade's. A
    /// trade issues all nine inside a single 280896-cycle frame, 28672 cycles
    /// apart, which is gpSP's measured figure for the Pokemon protocol
    /// (`SLAVE_IRQ_CYCLES_C`), and every link failure seen on hardware has been
    /// at the moment the game switches from the first cadence to the second.
    ///
    /// Both cores are stepped a scanline at a time rather than by `run_frame`,
    /// so the test can be the games' two interrupt handlers: each side checks
    /// the words the transfer landed and arms its next one, which is what makes
    /// the child's answers fresh. The CPU does not run, so this is the register
    /// model, the busy pacing, the cable and the interrupt, not the ROM.
    ///
    /// It fails if a transfer outlasts the gap the protocol leaves for it, if
    /// the child's engine cannot keep up at nine per frame, if a word lands in
    /// the wrong slot, or if the interrupt stops arriving part way through.
    #[test]
    fn nine_transfers_land_inside_one_emulated_frame() {
        use crate::bus::{Access::NonSeq as N, Bus};

        // Absolute numbers, not expressions over the constants they check.
        const GAP: u64 = 28672; // gpSP's SLAVE_IRQ_CYCLES_C
        const LINE: u64 = 1232;
        const LINES: u64 = 228;
        const MULTI: u16 = 0x2000 | 0x4000 | 3;

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (parent_end, child_end) = crate::cable::local_pair();
        let mut parent = Gba::new(rom.clone(), Vec::new());
        let mut child = Gba::new(rom, Vec::new());
        parent.connect_cable(Box::new(parent_end));
        child.connect_cable(Box::new(child_end));
        for g in [&mut parent, &mut child] {
            g.render_enabled = false;
            g.bus.write16(0x0400_0134, 0x0000, N);
            g.bus.write16(0x0400_0128, MULTI, N);
        }

        // One checksum word and eight data words on each side, the frame shape
        // gpSP documents.
        let parent_words: [u16; 9] =
            [0xA55A, 0x8FFF, 0x0102, 0x0304, 0x0506, 0x0708, 0x090A, 0x0B0C, 0x0D0E];
        let child_words: [u16; 9] =
            [0x5AA5, 0xB9A0, 0x1112, 0x1314, 0x1516, 0x1718, 0x191A, 0x1B1C, 0x1D1E];

        // The child's handler armed its first word before the parent clocked.
        child.bus.write16(0x0400_012A, child_words[0], N);

        let (mut issued, mut parent_done, mut child_done) = (0usize, 0usize, 0usize);
        let mut next_due = 0u64;
        let mut ninth_landed_at = None;

        for line in 0..LINES {
            if issued < 9 && line * LINE >= next_due {
                assert_eq!(
                    parent.bus.read16(0x0400_0128, N) & 0x0080,
                    0,
                    "transfer {issued} is due at line {line} and the previous one is still busy"
                );
                parent.bus.write16(0x0400_012A, parent_words[issued], N);
                parent.bus.write16(0x0400_0128, MULTI | 0x0080, N);
                issued += 1;
                next_due += GAP;
            }

            for g in [&mut parent, &mut child] {
                g.bus.cycles += LINE;
                g.bus.step_serial();
            }

            // Each game's serial interrupt handler: read the slots, arm the
            // next word. The child's store here is the one a parked child never
            // makes, which is the hazard
            // a_clock_that_beats_the_handler_is_held_until_the_game_arms
            // pins down.
            if parent.bus.if_ & 0x80 != 0 {
                parent.bus.if_ &= !0x80;
                let i = parent_done;
                assert_eq!(
                    (parent.bus.read16(0x0400_0120, N), parent.bus.read16(0x0400_0122, N)),
                    (parent_words[i], child_words[i]),
                    "parent transfer {i} landed the wrong pair"
                );
                parent_done += 1;
                if parent_done == 9 {
                    ninth_landed_at = Some(parent.bus.cycles);
                }
            }
            if child.bus.if_ & 0x80 != 0 {
                child.bus.if_ &= !0x80;
                let i = child_done;
                assert_eq!(
                    (child.bus.read16(0x0400_0120, N), child.bus.read16(0x0400_0122, N)),
                    (parent_words[i], child_words[i]),
                    "child transfer {i} landed the wrong pair"
                );
                child_done += 1;
                if child_done < 9 {
                    child.bus.write16(0x0400_012A, child_words[child_done], N);
                }
            }
        }

        assert_eq!(issued, 9, "the parent could not issue nine transfers in a frame");
        assert_eq!(
            (parent_done, child_done),
            (9, 9),
            "both ends have to complete all nine inside the frame"
        );
        assert_eq!(parent.cable_stats(), (9, 0), "nine completed, none lost");
        assert_eq!(child.cable_stats(), (9, 0));
        // The instrument the device run needed: both ends hash the words they
        // landed, in order, so an equal count with unequal hashes means a healthy
        // wire carried the wrong data. Tested here so the number means something
        // when it comes off a phone.
        assert_eq!(
            parent.cable_wordsum(),
            child.cable_wordsum(),
            "the two ends must have landed the same words in the same order"
        );
        assert_eq!(
            parent.bus.read16(0x0400_0128, N) & 0x00C0,
            0,
            "the parent ends the frame idle and without an error"
        );
        // The frame is 280896 cycles. Asserted against the absolute number so
        // this moves only when hardware does.
        let landed = ninth_landed_at.expect("the ninth transfer never landed");
        assert!(
            landed <= 280_896,
            "the ninth transfer landed at cycle {landed}, past the end of the frame"
        );
    }

    /// A child cannot clock the bus. The start bit is read-only for it on
    /// hardware, and a game that writes it anyway must not be left polling a
    /// busy bit that nothing is ever going to clear.
    #[test]
    fn a_child_cannot_start_a_transfer() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (_parent_end, child_end) = crate::cable::local_pair();
        let mut child = Gba::new(rom, Vec::new());
        child.connect_cable(Box::new(child_end));
        child.render_enabled = false;

        child.bus.write16(0x0400_0134, 0x0000, N);
        child.bus.write16(0x0400_0128, 0x2003, N);
        child.bus.write16(0x0400_012A, 0x1234, N);
        child.bus.write16(0x0400_0128, 0x2083, N);
        assert_eq!(
            child.bus.read16(0x0400_0128, N) & 0x0080,
            0,
            "the write to the start bit is dropped, not queued"
        );
        child.run_frame();
        assert_eq!(child.cable_stats(), (0, 0), "and no transfer happened");
    }

    /// Taking the cable away has to leave the port exactly as it is with
    /// nothing plugged in, because that is what every other game in the library
    /// sees and it is the path the whole corpus was swept against.
    #[test]
    fn a_detached_cable_leaves_a_lone_unit_behind() {
        use crate::bus::{Access::NonSeq as N, Bus};

        let mut rom = vec![0u8; 0x200];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let (parent_end, _child_end) = crate::cable::local_pair();
        let mut gba = Gba::new(rom, Vec::new());
        gba.connect_cable(Box::new(parent_end));
        gba.render_enabled = false;
        gba.bus.write16(0x0400_0134, 0x0000, N);
        gba.bus.write16(0x0400_0128, 0x2003, N);
        gba.bus.write16(0x0400_012A, 0xCAFE, N);
        gba.bus.write16(0x0400_0128, 0x2083, N);
        gba.disconnect_cable();
        assert!(!gba.cable_attached());

        // The lone-unit path completes inside the store, so the slots are right
        // immediately and nothing is left counting down.
        gba.bus.write16(0x0400_0128, 0x2083, N);
        assert_eq!(gba.bus.read16(0x0400_0120, N), 0xCAFE, "its own word echoes back");
        assert_eq!(gba.bus.read16(0x0400_0122, N), 0xFFFF, "no peer");
        let cnt = gba.bus.read16(0x0400_0128, N);
        assert_eq!(cnt & 0x0080, 0, "and the start bit does not stay set");
        assert_eq!(cnt & 0x0008, 0x0008, "a lone unit drives SD");
    }

    /// Every single-register load spends one internal cycle moving the loaded
    /// value into the register file (LDR = 1S+1N+1I on the ARM7TDMI). Run the
    /// same Thumb program from IWRAM twice, once with eight loads and once
    /// with eight register adds, and the loads must cost exactly their data
    /// access (1 cycle in IWRAM) plus that internal cycle more. IWRAM code has
    /// no sequential/non-sequential distinction, so this isolates the I-cycle.
    /// A frame must offer the adapter a chance to pull packets MID-FRAME, not
    /// only at the frame boundary.
    ///
    /// Draining the transport once per frame and then running 16 ms with nothing
    /// arriving makes the receive queue peak at the frame edge. Measured on two
    /// devices during a trade: the host reached the queue ceiling of 16 and
    /// discarded 14 blocks, and the trade only survived because the game
    /// retried. The reference polls in two places, once a frame and again
    /// whenever the adapter is mid-wait; this is the second.
    #[test]
    fn a_frame_polls_the_network_while_the_adapter_waits() {
        fn rom_coded(code: &[u8; 4]) -> Vec<u8> {
            let mut rom = vec![0u8; 0x200];
            rom[0xAC..0xB0].copy_from_slice(code);
            rom
        }

        // No adapter in the cart: the hook is never called, so a cart without
        // one pays nothing for this.
        let mut plain = Gba::new(rom_coded(b"ZZZZ"), Vec::new());
        assert!(plain.bus.rfu.is_none(), "no adapter on a plain cart");
        let mut calls = 0u32;
        plain.run_frame_polling(&mut || {
            calls += 1;
            Vec::new()
        });
        assert_eq!(calls, 0, "nothing to poll for");

        // FireRed. Wake the adapter, then hand it the clock with a WAIT, which
        // is the state where the game is sitting on the air waiting for a peer.
        let mut gba = Gba::new(rom_coded(b"BPRE"), Vec::new());
        let rfu = gba.bus.rfu.as_mut().expect("FireRed has an adapter");
        rfu.transfer(0x0000_494E);
        rfu.transfer(0x1234_5678);
        rfu.transfer(0xB0BB_8001);
        assert_eq!(rfu.state_name(), "idle", "handshake complete");
        // Host a room first. A WAIT with no session resolves immediately as a
        // disconnection, which is correct and would end the wait before the
        // first scanline check.
        rfu.transfer(0x9966_0019); // HOST_START
        rfu.transfer(0);
        assert_eq!(rfu.state_name(), "hosting");
        rfu.transfer(0x9966_0027); // WAIT, no payload
        rfu.transfer(0);           // the ack hands the clock over
        assert!(rfu.awaiting_event(), "the adapter now holds the clock");

        let mut polls = 0u32;
        gba.run_frame_polling(&mut || {
            polls += 1;
            Vec::new()
        });
        assert!(
            polls > 0,
            "a waiting adapter is offered the network during the frame"
        );
    }

    #[test]
    fn a_load_costs_one_internal_cycle_on_top_of_its_data_access() {
        use crate::bus::{Access, Bus};
        fn run(body_op: u16) -> u64 {
            use crate::bus::{Access, Bus};
            // ROM (ARM): ldr r0, [pc]; bx r0  -> Thumb code at 0x03000005.
            let mut rom = vec![0u8; 0x100];
            for (i, w) in [0xE59F_0000u32, 0xE12F_FF10, 0x0300_0005].iter().enumerate() {
                rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
            }
            let mut gba = Gba::new(rom, Vec::new());
            // Thumb at 0x03000004: ldr r2, =0x03000100 ; 8x body ; b .
            let prog: [u16; 11] = [
                0x4A06, // ldr r2, [pc, #0x18]  (literal at 0x03000020)
                body_op, body_op, body_op, body_op, body_op, body_op, body_op, body_op,
                0xE7FE, // b .
                0x0000,
            ];
            for (i, h) in prog.iter().enumerate() {
                let a = 0x0300_0004 + i as u32 * 2;
                gba.bus.write16(a, *h, Access::NonSeq);
            }
            gba.bus.write32(0x0300_0020, 0x0300_0100, Access::NonSeq);
            let setup_cycles = gba.bus.cycles;
            // 2 ARM instructions + ldr r2 + 8 body instructions = 11 steps.
            for _ in 0..11 {
                gba.cpu.step(&mut gba.bus);
            }
            gba.bus.cycles - setup_cycles
        }
        let loads = run(0x6811); // ldr r1, [r2]
        let adds = run(0x1C11); // adds r1, r2, #0
        assert_eq!(
            loads - adds,
            8 * (1 + 1),
            "each load must pay its 1-cycle IWRAM data access plus 1 internal cycle"
        );
    }

    #[test]
    fn save_state_roundtrip_is_deterministic() {
        // A small ROM: enough for the CPU to run deterministically for a while.
        let rom = vec![0u8; 0x2000];
        let mut a = Gba::new(rom.clone(), Vec::new());
        for _ in 0..8 {
            a.run_frame();
        }
        let blob = a.save_state();

        // Restore into a fresh machine and confirm it resumes identically.
        let mut b = Gba::new(rom, Vec::new());
        assert!(b.load_state(&blob), "state should load");
        assert_eq!(a.cpu.r, b.cpu.r, "registers restored");
        assert_eq!(a.cpu.cpsr, b.cpu.cpsr);
        assert_eq!(a.bus.cycles, b.bus.cycles);

        // Advancing both from the same point stays bit-identical.
        for _ in 0..4 {
            a.run_frame();
            b.run_frame();
        }
        assert_eq!(a.cpu.r, b.cpu.r, "diverged after resume");
        assert_eq!(&a.bus.ppu.framebuffer[..], &b.bus.ppu.framebuffer[..]);

        // A garbage blob is rejected, not panicked on.
        assert!(!b.load_state(&[1, 2, 3]));
    }

    /// A state written before SOUNDBIAS was modelled carries a zero nobody wrote,
    /// and honouring it clips the whole negative half of the audio to silence.
    /// The version byte is what separates "nobody wrote it" from "the game chose
    /// it", so both readings are pinned here.
    #[test]
    fn an_old_state_gets_the_soundbias_reset_value_and_a_new_one_keeps_its_own() {
        use crate::bus::{Access, Bus};
        let rom = vec![0u8; 0x2000];
        let mut a = Gba::new(rom.clone(), Vec::new());
        a.run_frame();
        // Deliberately park a zero in SOUNDBIAS, which is what an old state holds.
        a.bus.write16(0x0400_0088, 0, Access::Seq);
        assert_eq!(a.bus.read16(0x0400_0088, Access::Seq), 0, "the write landed");
        let current = a.save_state();
        assert_eq!(current[4], state::VERSION, "the header carries the version");

        let mut old = current.clone();
        old[4] = 1;

        let mut b = Gba::new(rom.clone(), Vec::new());
        assert!(b.load_state(&old), "a version-1 state must still load");
        assert_eq!(
            b.bus.read16(0x0400_0088, Access::Seq),
            0x0200,
            "an old state's SOUNDBIAS must be replaced with the reset value"
        );

        let mut c = Gba::new(rom, Vec::new());
        assert!(c.load_state(&current));
        assert_eq!(
            c.bus.read16(0x0400_0088, Access::Seq),
            0,
            "a current state's SOUNDBIAS is the game's own choice and must survive"
        );
    }

    /// The CPU must be able to see DISPSTAT's H-blank flag (bit 1) go high and
    /// come back down within a frame. Games wait on H-blank by polling this bit
    /// as often as by taking the IRQ, and while it was hard-wired to 0 they
    /// spun forever (Konami Krazy Racers never left its boot loop).
    ///
    /// The ROM is six ARM instructions: count the polls where bit 1 is set (r1)
    /// against the total polls (r3). Both bounds matter. r1 == 0 is the bug this
    /// fixes; r1 == r3 would be a flag stuck on, which breaks the other half of
    /// the games (the ones that wait for H-blank to *end*).
    #[test]
    fn hblank_flag_is_visible_to_the_cpu_and_clears_again() {
        let code: [u32; 8] = [
            0xE3A0_0404, // mov  r0, #0x04000000
            0xE3A0_1000, // mov  r1, #0            ; polls that saw H-blank
            0xE3A0_3000, // mov  r3, #0            ; polls total
            0xE1D0_20B4, // ldrh r2, [r0, #4]      ; DISPSTAT
            0xE312_0002, // tst  r2, #2
            0x1281_1001, // addne r1, r1, #1
            0xE283_3001, // add  r3, r3, #1
            0xEAFF_FFFA, // b    -> the ldrh
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let mut gba = Gba::new(rom, Vec::new());
        gba.run_frame();

        let (seen, total) = (gba.cpu.r[1], gba.cpu.r[3]);
        assert!(total > 0, "the poll loop should have run at all");
        assert!(seen > 0, "H-blank flag never read as set in a whole frame");
        assert!(seen < total, "H-blank flag never read as clear: stuck on");
    }

    /// Build a ROM that divides `number` by zero through SWI 06h and then parks.
    fn div_by_zero_rom(number: u32) -> Vec<u8> {
        let code: [u32; 4] = [
            0xE3A0_0000 | number, // mov r0, #number
            0xE3A0_1000,          // mov r1, #0
            0xEF06_0000,          // swi 0x06        Div
            0xEAFF_FFFE,          // b .
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        rom
    }

    /// A stand-in BIOS in which every vector is an infinite loop. Any SWI that
    /// actually reaches 0x08 never returns, so "did the core answer this call
    /// itself?" becomes observable without shipping a real BIOS image into the
    /// test.
    fn hanging_bios() -> Vec<u8> {
        0xEAFF_FFFEu32.to_le_bytes().repeat(0x4000 / 4)
    }

    /// Measured against a real BIOS dump: 0/0 returns r0=1, r1=0, r3=1, and
    /// every other divide by zero hangs the BIOS forever. The bundled open BIOS
    /// hangs on 0/0 as well, which is what kept Blackthorne and the rest of the
    /// Blizzard/Interplay ports from booting in the shipping configuration.
    ///
    /// Both bounds are checked. 0/0 must be answered without reaching the
    /// vector, and N/0 must still reach it, because widening the interception
    /// to N/0 would invent a result the console never produces.
    #[test]
    fn zero_over_zero_is_answered_without_entering_the_bios() {
        for bios in [Vec::new(), hanging_bios()] {
            let mut gba = Gba::new(div_by_zero_rom(0), bios.clone());
            for _ in 0..2 {
                gba.run_frame();
            }
            let tag = if bios.is_empty() { "HLE" } else { "BIOS" };
            assert_eq!(gba.cpu.r[0], 1, "{tag}: 0/0 quotient");
            assert_eq!(gba.cpu.r[1], 0, "{tag}: 0/0 remainder");
            assert_eq!(gba.cpu.r[3], 1, "{tag}: 0/0 abs quotient");
            assert!(
                gba.cpu.r[15] >= 0x0800_0000,
                "{tag}: should have returned to the cartridge, not parked in the BIOS"
            );
        }
    }

    /// A stand-in BIOS whose every vector returns immediately without touching
    /// r0-r3. It models the defect exactly: a BIOS that runs the call and
    /// leaves r3 alone.
    fn returning_bios() -> Vec<u8> {
        0xE1B0_F00Eu32.to_le_bytes().repeat(0x4000 / 4) // movs pc, lr
    }

    /// Div returns three registers, not two: r0 = quotient, r1 = remainder and
    /// r3 = abs(quotient). Measured 100/7 against a real BIOS dump: 14, 2, 14.
    ///
    /// r3 is the one the bundled open BIOS forgets, and forgetting it is not
    /// visibly wrong: r0 and r1 are right, so the game computes correctly until
    /// it uses r3, and then it has whatever it was holding. Puppy Luv puts that
    /// stale value in its interrupt-handler table and branches to it.
    ///
    /// Note what this does NOT assert. r0 stays 100 here, because the stand-in
    /// BIOS never divides and we deliberately do not divide for it: the real
    /// BIOS is left to do the arithmetic and spend the cycles, and only r3 is
    /// seeded. Taking the whole call over instead was measured and cost Happy
    /// Feet its title screen in both regions, so "we did not compute r0" is a
    /// property worth pinning rather than an omission.
    #[test]
    fn divide_seeds_r3_without_taking_over_the_call() {
        let code: [u32; 9] = [
            0xE3A0_0064, // mov r0, #100
            0xE3A0_1007, // mov r1, #7
            0xE3A0_30FF, // mov r3, #0xFF     sentinel: untouched means not set
            0xEF06_0000, // swi 0x06          Div
            0xE3A0_4403, // mov r4, #0x03000000
            0xE584_0000, // str r0, [r4]
            0xE584_1004, // str r1, [r4, #4]
            0xE584_3008, // str r3, [r4, #8]
            0xEAFF_FFFE, // b .
        ];
        let mut rom = vec![0u8; 0x2000];
        for (i, w) in code.iter().enumerate() {
            rom[i * 4..i * 4 + 4].copy_from_slice(&w.to_le_bytes());
        }
        let mut gba = Gba::new(rom, returning_bios());
        for _ in 0..2 {
            gba.run_frame();
        }
        assert_eq!(gba.cpu.r[3], 14, "r3 must be seeded with abs(quotient)");
        assert_eq!(gba.cpu.r[0], 100, "the divide itself is left to the BIOS");
    }

    #[test]
    fn nonzero_over_zero_still_reaches_the_bios() {
        let mut gba = Gba::new(div_by_zero_rom(5), hanging_bios());
        for _ in 0..2 {
            gba.run_frame();
        }
        assert!(
            gba.cpu.r[15] < 0x0800_0000,
            "N/0 must not be intercepted: hardware hangs here, so we must too"
        );
    }

    /// `bios_steps` must count BIOS instructions and ONLY BIOS instructions.
    ///
    /// This is the signal that tells a crashed cartridge apart from a working
    /// one, and a colour count cannot: Maniac Racers Advance sits on the
    /// Normmatt boot logo forever and scores the same 16 distinct colours as a
    /// game on a dark title screen. Both halves are asserted separately, so a
    /// bound that is too wide (counting cartridge code) or an increment that
    /// never fires both fail.
    #[test]
    fn bios_steps_counts_the_bios_and_nothing_else() {
        // A stand-in BIOS: an endless loop parked at the SWI vector, so a
        // cartridge that calls a SWI never comes back and every instruction
        // after that one is BIOS.
        let mut bios = vec![0u8; 0x4000];
        bios[8..12].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes()); // 0x08: b .

        // Cartridge A never leaves its own code: `b .` at the entry point.
        let mut rom = vec![0u8; 0x2000];
        rom[0..4].copy_from_slice(&0xEAFF_FFFEu32.to_le_bytes());
        let mut a = Gba::new(rom, bios.clone());
        a.run_frame();
        assert!(a.steps > 1000, "the cartridge should have run, got {}", a.steps);
        assert_eq!(
            a.bios_steps, 0,
            "a cartridge that never calls a SWI must not count as BIOS"
        );

        // Cartridge B vectors into the BIOS on its first instruction and stays.
        // Any SWI the core does not handle itself will do; SWI 08h (Sqrt) is one.
        // It CANNOT be SWI 00h, which this test used to use: SoftReset is now
        // taken over in `hle::soft_reset` even when a BIOS is loaded, so it never
        // reaches the vector. That is the point of the interception, not a
        // regression in what this test is actually about, which is `bios_steps`.
        let mut rom = vec![0u8; 0x2000];
        rom[0..4].copy_from_slice(&0xEF08_0000u32.to_le_bytes()); // swi #8 (Sqrt)
        let mut b = Gba::new(rom, bios);
        b.run_frame();
        assert!(
            b.bios_steps > 1000,
            "a cartridge stuck in the BIOS must count as BIOS, got {} of {}",
            b.bios_steps,
            b.steps
        );
        assert!(
            b.bios_steps < b.steps,
            "the cartridge ran its own SWI first, so not every step is BIOS"
        );
    }
}