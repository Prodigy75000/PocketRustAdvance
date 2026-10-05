# LOCKSTEP: holding two emulated GBA clocks together over a real network

Status: design on paper, nothing implemented. This is the plan for the one
problem left on the link cable after the 2026-10 device rounds: the transport
is proven (8827 transfers, both ends agreeing to the digit, zero loss), the
role election works, Pokemon Ruby's handshake completes end to end, and the
game then dies at the handshake-to-connected transition with "transmission
error" while every cable counter reads healthy. The transport is innocent.
The two emulated clocks are not being held together, and that is the whole
remaining problem.

Everything here is phrased against the code as it reads today at:

- `crates/gba-core/src/cable.rs` (wire protocol, `LinkCable` trait, `local_pair`)
- `crates/gba-core/src/memory.rs` (register model, `multi_start` / `multi_finish`, where emulated time is charged)
- `crates/gba-core/src/lib.rs` (`run_frame_polling`, the per-scanline pump)
- `crates/gba-libretro/src/netpacket.rs` (`NetCable`, the blocking `parent_result` loop)

and against mGBA's lockstep, surveyed below, at
`TrophyHubCoresMirror/mgba/src/gba/sio/lockstep.c` and friends.

## 0. The arithmetic that frames everything

Measured between the owner's two devices on his Wi-Fi: RTT 6.0 ms min, 8.1 ms
mean, 10.7 ms max, 0% loss, `WIFI_MODE_FULL_LOW_LATENCY` held. Pokemon's
connected mode runs ~9 transfers per video frame; gpSP models the slave side
of exactly this (`gpsp/serial_proto.c`):

```c
#define SLAVE_IRQ_CYCLES_C   28672   // Aproximately every 1.7ms, ~9.5 per frame.
#define SLAVE_IRQ_CYCLES_H  280064   // Aproximately every frame (16.66ms).
```

So in connected mode the master clocks every ~28672 cycles, nine and a bit
per 280896-cycle frame, and the protocol frame is 1 checksum word plus 8 data
words, so one bad word poisons the whole frame. A networked transfer cannot
complete without the peer's word, so each one costs a round trip: nine round
trips, ~72 ms of wall time, inside what the game believes is one 16.7 ms
frame. Something has to stretch, and both devices have to stretch equally.

Two fixes were tried on device and reverted, and both failed the same way:
each used a signal visible on ONE device to decide a question about TWO.

1. A floor on the transfer duration (12 ms, so the round trip hides inside
   emulated time). The game rejected it instantly: nine transfers must fit
   one 280896-cycle frame, so each must be under 1.86 ms, while hiding an
   8 ms round trip needs over 8 ms. No value satisfies both. The arithmetic
   is now recorded in the long comment above `TRANSFER_CYCLES` in `cable.rs`.
2. Blocking the child the moment its game wrote SIOMLT_SEND. Measured: child
   at 6.6 fps against a parent at 56.7. That write means "the next word is
   ready", not "I am waiting"; the game does a whole frame of other work
   after it. A readiness signal is not a waiting signal.

Neither is proposed again here. The design below puts the pacing signal on
the wire, where a two-device question has to live.

## 1. The invariant

### Quantities that exist in the code today

- `GbaBus::cycles` (u64, `memory.rs`): the emulated clock, charged by the CPU,
  DMA and the serial busy window. One scanline is `ppu::CYCLES_PER_LINE` =
  1232 cycles; one frame is 228 scanlines = 280896 cycles.
- `GbaBus::frame_no` (u32, `memory.rs:180`): incremented at line 0 of
  `Gba::run_frame_polling`. It exists, but it is NOT the unit this design
  uses; section 5 shows why a frame is too coarse.
- `CableProto::own_output` (`cable.rs`): the word this unit presents on the
  bus, refreshed by every store to SIOMLT_SEND via `LinkCable::set_output`.
- `CableProto::child_input` (`cable.rs`): transfers a parent clocked into us,
  not yet collected by the serial engine; its depth is `lag()`.
- `serial_pending` / `cable_busy` (`memory.rs`): the in-flight transfer's
  remaining emulated duration, from `multi_cycles(cnt, units)` =
  `TRANSFER_CYCLES[baud][units-1]`, 5755 cycles for Pokemon's two units at
  115200 baud.

### New quantities this design adds

- A session cycle counter per cable end: `local_cycles: u32`, advanced by a
  new `LinkCable::advance(cycles)` called once per scanline from
  `run_frame_polling` (delta 1232), zeroed on `CableProto::reset` and on
  `flush` (the Multi-Player mode-select edge that already drops in-flight
  state in `GbaBus::write`'s `touches_siocnt` branch).
- Every outgoing packet carries the sender's `local_cycles` (section 2).
- On the child, a signed budget `B`, integrated purely from wire deltas:
  on each valid parent packet carrying time `t`,
  `B += wrapping_sub(t, last_parent_time) clamped to >= 0`, capped at `+H`;
  on each scanline the child emulates, `B -= 1232`. `B` starts at `+H` when
  pacing engages. `H = 280896`, one frame.

### The invariant

> **While its game is in Multi-Player mode with a cable attached, the child
> never emulates more than H = 280896 cycles past the parent's most recently
> reported emulated position.** Formally: `B >= 0` before every scanline,
> where `B = H + (sum of parent cycle deltas heard) - (sum of child scanline
> deltas emulated since pacing engaged)`.

The parent is never budget-paced. It is already wall-paced by the one
mechanism that cannot lie: `NetCable::parent_result` blocks until the child's
reply arrives, so in connected mode the parent's emulated progress slows to
the round trip automatically. The invariant then transmits that slowness to
the child, because the parent's progress is the only thing that feeds `B`.
During the handshake the parent is not blocking, its progress runs at 60 fps,
`B` stays topped up, and the child never throttles. Nothing anywhere decides
"are we in connected mode"; the pace emerges from the parent's measured
progress. That is the entire trick.

A budget, not an absolute comparison, on purpose: the two cores power on
seconds apart, so their absolute `cycles` values share no epoch. Integrating
deltas needs no shared anchor, and the failure direction is safe: a lost or
late packet can only slow the child down, never let it run ahead.

### One amendment to the agreed direction

The direction agreed with the owner was "parent's emulated FRAME count on the
wire; a child throttles at a FRAME boundary only while it is ahead". The
survey below keeps the shape (parent progress on the wire, child-only
throttle, one-sided) but has to refine the unit and the granularity, because
the frame is provably too coarse:

- The master's inter-clock gap in connected mode is 28672 cycles, about 23
  scanlines. A frame is 280896. If the child may only stop at frame
  boundaries, then while the parent grinds through one 72 ms frame emitting
  nine clocks, a parked child faces exactly the two bad options: stay parked
  across several clocks and answer them all from one `own_output` (the
  stale-word corruption this document exists to prevent, section 4), or wake
  per clock and run a whole frame each time, which runs the child at up to
  nine frames per parent frame and starves its game down to one transfer per
  frame, the same failure signature as the reverted SIOMLT_SEND block, with
  the sign flipped back.
- The pacing quantum must therefore be at most the inter-clock gap:
  1232 <= 28672 holds, 280896 does not. The throttle point is the top of the
  scanline loop, and the wire unit is cycles, which frames can be derived
  from but not the reverse.

## 2. What goes on the wire

### Today (version 2)

From `cable.rs`: `FAMILY = *b"CBL"`, `WIRE_VERSION = b'2'`,
`MAGIC = 0x4342_4C32` ("CBL2"), `PACKET_LEN = 9`, and three tags:

| off | len | field | notes |
|----:|----:|-------|-------|
| 0   | 3   | FAMILY `"CBL"` | family and version split on purpose, so a version mismatch is visible instead of reading as no-peer |
| 3   | 1   | WIRE_VERSION `'2'` | mismatch counts `version_mismatch` and latches refusal |
| 4   | 1   | tag: `TAG_CLOCK`=1, `TAG_REPLY`=2, `TAG_GAVE_UP`=3 | |
| 5   | 1   | seq | pairing; parent consumes only the reply carrying its awaited seq |
| 6   | 2   | word, big-endian | |
| 8   | 1   | lag | reply only: child backlog at answer time |

### Proposed (version 3)

| off | len | field | change |
|----:|----:|-------|--------|
| 0   | 3   | FAMILY `"CBL"` | unchanged |
| 3   | 1   | WIRE_VERSION `'3'` | **bumped** |
| 4   | 1   | tag, now also `TAG_TICK` = 4 | **new tag** |
| 5   | 1   | seq (0 for TICK) | unchanged |
| 6   | 2   | word (0 for TICK and GAVE_UP) | unchanged |
| 8   | 1   | lag (reply only, else 0) | unchanged |
| 9   | 4   | `sender_cycles`, u32 big-endian | **new field**: the sender's session cycle counter at emission |

`PACKET_LEN` 9 -> 13, `MAGIC` -> `0x4342_4C33` ("CBL3"), `put()` gains the
time argument. Meaning per tag:

- `TAG_CLOCK`: parent's position when it clocked this exchange. This is the
  packet that feeds the child's budget during transfers; each clock carries
  roughly one inter-clock gap of credit by construction.
- `TAG_TICK`: parent only, emitted once per emulated frame (every 280896
  cycles of `advance()` accumulation) whenever the session is active,
  regardless of mode. It is the 60 Hz floor that keeps the child's budget fed
  while the parent's game is in menus, naming screens, or anywhere it is not
  clocking, so pacing never freezes a child just because the link went quiet.
  One 13-byte datagram per frame is noise. It is the same job as mGBA's
  SIO_EV_HARD_SYNC, which fires every 0x80000 cycles even with no transfer.
- `TAG_REPLY` / `TAG_GAVE_UP`: sender's (child's) position, carried for
  diagnostics only. It paces nobody, but it lets the PARENT's heartbeat print
  the measured skew, so one device's logcat can answer "how far ahead was the
  child", which until now needed both logs side by side.

Duplicate handling note: the cached-reply replay path (`last_reply` in
`on_packet`) replays the reply byte-for-byte, so a replayed reply carries the
original timestamp. Correct: a retransmit is not new progress. On the budget
side, out-of-order or duplicate parent times clamp to a zero delta rather
than going negative.

### Does this need a wire version bump? Yes, twice over.

The layout changes (9 -> 13 bytes), and the semantics change: a v3 child
paired with a v2 parent would never hear a tick and would starve-disengage
forever, which is a silently degraded session, the exact failure the version
byte exists to prevent. Version 2's own doc comment already committed to
this: it was bumped "for a change with no layout in it at all: how a transfer
is paced... A semantic agreement counts as the wire." The existing machinery
then does the right thing unmodified: `on_packet` counts `version_mismatch`,
latches, and `cable_units()` reports a one-unit bus, so a mixed pair plainly
refuses to link instead of corrupting. There is still no negotiation, and
there still should not be: both devices come off the same APK.

## 3. Where each side may block, traced through the real call path

### The parent (unchanged by this design)

```
retro_run                                    crates/gba-libretro/src/lib.rs:697
  Gba::run_frame_polling(drain_inbound)      lib.rs:857 -> gba-core/src/lib.rs:237
    per scanline: bus.step_timers(); bus.step_serial()     memory.rs
      serial_pending counts down; on expiry with cable_busy:
      GbaBus::multi_finish()                 memory.rs:1699
        cable_words is None on a parent, so:
        cable.parent_result()                -> NetCable::parent_result, netpacket.rs
          loop { poll_receive(); take_reply(seq);
                 retry at RETRANSMIT_AFTER=40ms; sleep POLL_INTERVAL=250us }
          until reply, or EXCHANGE_TIMEOUT=150ms, or open circuit
          (FAILURES_BEFORE_OPEN_CIRCUIT=3 fails fast when the peer is gone)
```

The transfer was started earlier by the game's SIOCNT store:
`GbaBus::write -> sio_transfer -> multi_start -> cable.parent_start`, where
`NetCable::parent_start` calls `begin_exchange` and flushes the outbox
immediately, "so the round trip overlaps the transfer's own duration instead
of being added to it" (`multi_finish`'s own comment). The parent is therefore
ALLOWED to block mid-frame, inside `step_serial`, bounded by the timeout, and
it keeps pumping the network while blocked (`poll_receive` inside the loop).
This is where the parent's wall-time stretch already happens today, 9 round
trips per connected frame, and it is correct.

### The child, today

The child never blocks anywhere. Clocks are answered synchronously inside
`np_receive` from `CableProto::on_packet` ("the peer that sent it is BLOCKED
waiting for the answer", netpacket.rs module doc), queued in `child_input`,
and collected at most one scanline later by the per-scanline block in
`step_serial` (`memory.rs:1588`), which raises the busy window and, 5755
cycles later, `multi_finish` lands the words and the IRQ. Because nothing
ever slows the child's frame loop, it runs at 60 fps wall while the parent
runs at 14, which is the divergence that kills the connected transition.

### The child, with this design: one new hold point

At the top of the scanline body in `run_frame_polling`, immediately before
the existing pump block (the one whose comment reads "**Deliberately NOT
gated on Multi-Player mode, which it was**", gba-core/src/lib.rs:287):

```
if bus.cable attached, this end is a child (id != 0), and bus.multi_mode():
    while cable.hold() {                 // hold(): paced && B <= 0 && lag == 0
        for (from, bytes) in poll() { .. }   // the pump keeps running: this
                                             // is what answers the parent
        sleep(POLL_INTERVAL);
        on starvation (no parent packet for ~150 ms): hold_starved(); break;
    }
cable.advance(1232);                     // both ends; parent's impl also
                                         // emits TAG_TICK every 280896
```

Rules the hold point obeys:

- It sits BETWEEN scanlines, never inside `step_serial`, never inside
  `np_receive`. Blocking in `np_receive` is forbidden for the same reason the
  hot-spin was removed from `parent_result` (`POLL_INTERVAL`'s comment): the
  frontend's receive path takes the host's queue lock, and the callback runs
  re-entrantly under `poll_receive`.
- The pump stays unconditional and mode-free, exactly as the lib.rs:287
  comment demands. The HOLD is gated on the child's own `multi_mode()`, and
  that asymmetry is deliberate and safe: the pump answers the PEER (a
  two-device duty, always on), while the hold slows OURSELVES (self-regarding
  only). A wrong local `multi_mode` can only make the child run at full
  speed, which is today's behaviour, never corrupt a word.
- `hold()` is false the instant `child_input` is non-empty, so a pending
  clock always gets its scanline. In practice the clock packet that queued it
  also carried ~28672 cycles of budget, so the child has about 23 scanlines
  of credit to collect it, run the busy window, and let the game's IRQ
  handler arm the next word.
- The hold loop fails open: starvation (parent silent while we are parked),
  session teardown (`units() == 1`), or a latched version refusal all
  disengage pacing and return the child to free running, where the game's own
  link-error path takes over via failed transfers. A dead parent must never
  be able to freeze a child indefinitely.

MUST-NOT-block list, explicitly: `set_output`, `child_clock`,
`on_packet`/`np_receive`, `multi_finish` on a child (its `cable_words` are
already present), and anything holding the `with_net` borrow across
`poll_receive`.

## 4. The queued-clock hazard, and what actually prevents a stale `own_output` answering twice

The hazard, restated in code terms: `CableProto::on_packet` answers
`TAG_CLOCK` from `self.own_output` immediately. `own_output` is refreshed
only when the child's GAME stores SIOMLT_SEND, which Pokemon does in the
serial IRQ handler of the previous transfer. A child whose emulation is
parked while clocks K and K+1 both arrive answers K+1 with the word armed for
K. Both ends' counters stay green, the checksum word is wrong, and the game
reports a cable fault that no instrument contradicts. It is the worst
available failure shape because it is invisible.

Three layers prevent it, two of which already exist:

1. **At most one clock can ever be outstanding** (exists). The parent's
   `parent_result` blocks until the reply for `awaiting` arrives or is
   abandoned, so the parent cannot emit clock K+1 until it has consumed the
   answer to K. Retransmits of K are answered from the `last_reply` cache
   byte-for-byte ("recomputing it would answer a stale clock with a fresh
   word, which is the corruption the pairing exists to stop"), so duplicates
   cannot consume a fresh word either. The multi-clock pileup therefore
   requires the parent to have completed exchange K, emulated at least its
   own handler plus protocol gap (>= ~28672 cycles), and clocked K+1, all
   while the child emulated nothing. That is only possible if the child is
   parked across the whole sequence, which brings in layer 2.
2. **The park predicate makes "parked implies armed" an invariant** (new).
   The child parks only when `B <= 0 && lag == 0`. Work backwards from a
   parked child: the last clock K was collected (lag 0), and collecting it
   plus running the busy window plus the game's handler costs at most
   1232 + 5755 + handler cycles, while the budget delta that clock K itself
   carried is the parent's own inter-clock progress, >= 5755 + the parent's
   handler + the protocol's deliberate slave-servicing margin (gpSP's 28672).
   The game's handler fits inside the master's gap BY THE GAME'S OWN DESIGN,
   because that is precisely the contract a physical cable imposes: the
   master paces so the slave's IRQ handler finishes before the next clock.
   So the child always reaches the handler's SIOMLT_SEND store before its
   budget can run out, and a parked child always holds a freshly armed
   `own_output`. Clock K+1 arriving at a parked child is answered correctly,
   queues, carries fresh budget, and the child wakes and services it; by
   induction every answer is armed. The design does not shrink the race to
   "unlikely"; it inherits hardware's own guarantee and keeps the child
   inside the window where that guarantee applies. A game that violated it
   (rewriting SIOMLT_SEND later than one master gap) would corrupt on real
   hardware too.
3. **If it happens anyway, it is counted** (new). `on_packet` increments a
   new `stale_risk` counter whenever a `TAG_CLOCK` is accepted while
   `child_input` is non-empty, i.e. the previous transfer was never collected
   by the serial engine before the next clock arrived. That is the exact
   precondition of the stale answer (the one case it over-counts is a game
   that legitimately never rewrites the register, which is why this is a
   counter and not a gate). It rides the once-a-second heartbeat in
   `retro_run` next to `cable_done`/`cable_lost`, so the worst failure shape
   stops being invisible on a phone. Today this condition occurs silently;
   `CHILD_QUEUE_MAX = 8` will happily hold a pileup.

Rejected alternative, recorded so it is not proposed again: gating the answer
on an `own_output` dirty flag ("answer K only after a store since the answer
to K-1") encodes the contract more directly, but deadlocks any game that
legitimately presents one word across many transfers without rewriting the
register. A pacing invariant plus a counter beats a correctness gate that can
hang a conforming game.

## 5. How the handshake stays at 60 fps while connected mode paces

The suspicious question first, because both reverted attempts died on it:
**what signal distinguishes the phases, and who can see it?**

Answer: there is no phase signal, by design. The only control input is the
parent's emulated progress, produced on the parent (the one device that has
it), carried to the child on every `TAG_CLOCK` and on the once-per-frame
`TAG_TICK`. The child compares it against its own progress, which it also
owns. Every quantity is measured on the device that owns it and crosses the
wire to be compared; nothing is inferred about the other side from local
state. The arithmetic then produces the two behaviours:

- Handshake: one transfer per frame (gpSP's `SLAVE_IRQ_CYCLES_H`, ~280064
  cycles). The parent pays one RTT per frame, largely overlapped with the
  transfer window and frame slack, and runs at or near 60 fps. Budget income
  to the child is ~280896 cycles per 16.7 ms against spend of the same.
  `B` hovers near its cap; the child never holds. Counts stay level, nothing
  throttles, which matches what the devices already measured (both at 59.5
  to 60 through "Start link with 2 players").
- Connected: nine transfers per frame. `parent_result` blocks ~RTT per
  transfer, nine times per frame, so the parent's progress rate falls to
  roughly `16.7 / (16.7 + 9 * 8.1 - overlap)` of real time, ~14 fps. Budget
  income falls to the same rate, the child spends its initial `+H` within one
  frame and from then on holds whenever income pauses, pinned to the parent's
  pace within one frame. Both devices at ~14 fps, which the owner has
  explicitly accepted: that is what lockstep means, and normal play is
  untouched because pacing only engages with a cable attached and the game in
  Multi-Player mode.
- Link goes quiet (menus, pauses between protocol phases): the parent stops
  clocking but keeps ticking at its frame rate, so the child does not freeze.
  This is the case a transfer-driven signal alone would get wrong, and it is
  why `TAG_TICK` exists.

The two residual local gates, named and defended rather than hidden: the
child's own `multi_mode()` enables self-pacing, and the starvation timeout
disables it. Both only ever fail toward "child runs free", which is the
status quo, never toward a stale word or a frozen screen.

## 6. Desk test plan

**Status, 2026-10-05: tests 1 and 2 have landed, before any pacing code.**

- `nine_transfers_land_inside_one_emulated_frame` (`6af1c8c`): nine transfers
  28672 cycles apart inside one 280896-cycle frame, both cores stepped a
  scanline at a time with the test playing both interrupt handlers. **Proven to
  fail by putting the reverted 12 ms floor back**, where it stops at "transfer 1
  is due at line 24 and the previous one is still busy", the game's own
  complaint reproduced on the desk in 30 ms. The nine-FRAME test stays green
  under that same change, which is why it was never evidence about this.
- `a_parked_child_answers_two_clocks_with_the_word_armed_for_the_first` and
  `a_child_that_collects_each_transfer_is_never_counted_as_at_risk`
  (`7043c7a`): the hazard pinned down against today's code, plus the
  `stale_risk` counter of section 4 layer 3, on the heartbeat as
  `cable_stale_risk`. The first asserts the stale answer rather than a
  corrected one on purpose: pacing removes the parked child, it does not change
  what this layer can answer while parked. The correctness half, `stale_risk`
  staying zero across a paced run, is test 7 and still to come.

What remains from this plan: `DelayedWire`, `PacedTestCable`, and tests 3
through 7, all of which need the pacing surface to exist.


Correction first, and then a correction to the correction. The brief for this
document named an existing test `a_nine_transfer_pokemon_frame_runs_end_to_end`
and it was absent from HEAD, which is what the survey above reported. The
history says why: it was WRITTEN in `9fb3a42` ("drive a whole nine-transfer
link frame on the desk") and DELETED in `d201ee7`, the commit that introduced
the 12 ms transfer floor. `d201ee7`'s message accounts for two tests it
rewrote and does not mention this one at all, so the deletion went unrecorded.

It has been restored on top of this document and passes unchanged, so the
scaffold below exists again rather than needing to be built. Two things about
it that matter more than its return:

- **It does not prove what its name suggests.** It drives one transfer per
  `run_frame()`, nine frames for nine transfers, so it never tests nine
  transfers inside ONE frame, which is the only case connected mode cares
  about. Test 1 below is still to build.
- **It would not have caught the floor either.** Re-applying
  `max(201_327)` to `multi_cycles` on today's code leaves it green, measured
  rather than assumed, because one transfer per frame fits under the floor
  comfortably. So the comforting story, that the desk test contradicted the
  design and was removed, is false. Why it was deleted is undetermined.

What exists, after the restore:

- `cable.rs` tests: a `Wire` harness around two `CableProto`s with manual
  `deliver`/`drop_all`, "deliberately meaner than the real one: nothing is
  delivered until a test says so".
- `gba-core/src/lib.rs` tests: `two_cores_exchange_a_word_over_the_cable`
  drives TWO FULL CORES over `local_pair()` through the register model (RCNT,
  SIOCNT, SIOMLT_SEND, start bit) for ONE transfer,
  `a_nine_transfer_pokemon_frame_runs_end_to_end` (restored) drives nine of
  them one per frame, plus
  `a_transfer_occupies_the_time_its_baud_rate_says`,
  `selecting_multi_player_mode_drops_what_was_in_flight_before_it`, and
  `a_child_cannot_start_a_transfer`.

So the nine-in-one-frame test is the first thing to BUILD, and the three
device rounds that went on heuristics are the argument for building it before
writing any pacing code.

### Harness pieces

1. `DelayedWire` (new, `cable.rs` tests): the existing `Wire` plus a delivery
   queue keyed on a tick counter: `send_at(tick + rtt_ticks)`. Time is pump
   ticks, never wall clock, so tests are deterministic and instant. An
   `rtt_ticks` of 0 reproduces today's `Wire`.
2. `PacedTestCable` (new): implements `LinkCable` including the new
   `advance`/`hold` surface over a `CableProto` pair joined by `DelayedWire`,
   so the budget arithmetic is testable with no sockets, no `NET` global and
   no sleeps. The production hold loop takes its timeout as poll-iterations
   through the trait, so tests can set "starve after N polls" instead of
   150 ms.
3. The two-core scaffold copied from `two_cores_exchange_a_word_over_the_cable`:
   both cores in Multi-Player at 115200 baud with the serial IRQ enabled,
   `const MULTI: u16 = 0x2000 | 0x4000 | 3`.

### The tests, each with the failure it exists to catch

1. `nine_transfers_land_inside_one_emulated_frame` (create it; the restored
   `a_nine_transfer_pokemon_frame_runs_end_to_end` is the nine-frames case and
   stays as it is): script the gpSP frame shape, 1 checksum word
   plus 8 data words, parent writing SIOMLT_SEND and the start bit at 28672
   cycle intervals, child answering by writing its next word after each
   serial IRQ (poll `if_` bit 7 the way the existing test polls registers).
   Assert all nine land, in order, with the right word in the right SIOMULTI
   slot on both ends, and that both cores end the sequence inside two emulated
   frames. Fails on: ordering bugs, slot assignment, busy-window pacing, IRQ
   delivery. Run it under `DelayedWire` at rtt 0 AND at a simulated 8 ms.
2. `a_child_parked_across_two_clocks_never_answers_the_same_word_twice`: the
   hazard, nailed shut. Parent completes exchange K; child core is then NOT
   run at all (simulating a parked child) while the parent runs far enough to
   issue K+1. Assert the design's outcome: either the child's budget machinery
   forces scanlines before K+1 is answered, or `stale_risk` increments.
   **Prove it can fail** by running it against the pre-change transport,
   where the second answer IS the same word: it must be red before the
   mechanism lands and green after. That is the test the last three device
   rounds needed.
3. `the_child_never_runs_past_the_horizon`: feed the child parent-time
   packets up to position T, run frames; assert its session cycles never
   exceed T + 280896 (the literal, not the constant `H`, per the
   assert-absolute-numbers rule) and that `hold()` is true at the boundary.
   Then deliver one tick and assert exactly ~280896 more cycles unlock.
   Fails on: budget sign errors, wrap handling, cap handling.
4. `a_level_handshake_never_holds_the_child`: ticks every 280896 of parent
   time, child consuming at the same rate with realistic jitter; assert zero
   holds across 60 frames. Catches the design over-throttling the phase that
   must stay at 60 fps, which is where attempt 2 died.
5. `a_silent_parent_releases_the_child`: stop feeding packets while the child
   is held; assert it resumes free-running after the injected starvation
   bound, and that a subsequent parent packet re-engages cleanly. Catches the
   frozen-child failure.
6. `a_v2_peer_is_refused_not_paced`: a 9-byte version-2 packet against the
   13-byte build: `version_mismatch` latches, `units()` reports 1, budget
   never engages. Extends the existing
   `a_peer_on_a_different_cable_version_is_refused_rather_than_guessed_at`.
7. Property run: test 1 swept over injected RTTs {0, 2, 8, 12 ms} and a
   deterministic jitter pattern, asserting zero `stale_risk`, zero give-ups,
   and words correct to the digit. This is the desk version of the owner's
   device rounds, and it must exist BEFORE the next APK goes out.

Anti-tautology check: every test above asserts game-visible words, absolute
cycle numbers, or a counter that the pre-change code demonstrably fails;
none asserts the implementation's own constants back at itself.

## 7. What this design cannot do, and what would falsify it

### Cannot do, by construction

- **Connected-mode speed.** Both devices run at roughly
  `1000 / (16.7 + 9 * RTT_ms - overlap)` fps; at the measured 8.1 ms mean
  that is ~14 fps for the duration of a trade or battle. Accepted by the
  owner; not a defect, the definition of lockstep. Protocols that clock more
  than 9 per frame scale worse linearly.
- **RTT spikes past 150 ms** (`EXCHANGE_TIMEOUT`) still raise SIOCNT's error
  flag via `multi_finish`'s `words.is_none()` path and poison a protocol
  frame; the game runs its own link-error screen. The design narrows failure
  to genuine network faults; it does not survive them.
- **Three and four units.** Out of scope, as the transport already enforces
  (`cable_units()` caps at 2, `cable_extra_peers` counts the excess).
- **Fast forward.** Out of scope by owner instruction; no tolerance or
  buffering exists for a peer above 60 fps, deliberately.
- **Savestates and resets during a session.** The budget and seq state are
  session-local and unserialized; loading a state or `retro_reset` mid-link
  desyncs the pair (mGBA ships a 0x1F0-byte lockstep savestate blob for this;
  we deliberately do not). Also note `retro_reset` history here: 964c583.
- **Cartridge RTC honesty under stretch.** The S-3511A reads host time via
  the frontend, so a stretched session advances RTC ~4x per emulated second.
  No known link protocol reads the RTC mid-exchange; named as a watch item,
  not handled.

### Falsifiers on hardware, named by counter or symptom

- `stale_risk > 0` on the child's heartbeat during a paced session: layer 2's
  invariant is wrong in practice. That counter existing is half the point.
- Both heartbeats at ~14 fps, `stale_risk` 0, `cable_lost` 0, counts agreeing
  to the digit, and Ruby STILL errors at the same transition: pacing was not
  the missing ingredient, and the bug is in the transition itself; next stop
  is `GBA_SIOLOG` word traces on both ends diffed seq by seq.
- Child heartbeat shows ~0 hold time while the parent sits at 14 fps: the
  budget is not engaging; check `cable_badver`, tick receipt count, and the
  child's `multi_mode()` gate.
- Child visibly frozen while its game is NOT linking: the `multi_mode()` gate
  or starvation release is broken; that is a frozen-child bug, fail-open was
  the requirement.
- Handshake measurably below ~59 fps on either device: the design is
  over-throttling the level phase (test 4's failure escaping to hardware).

New heartbeat fields to carry all of this: `cable_stale`, `cable_holds`,
`cable_hold_us` (sum and max), `cable_ticks` (sent on the parent, heard on
the child), and the latest peer-skew estimate from reply timestamps.

## Appendix A: the `LinkCable` seam the design must fit through

The trait as it reads today in `crates/gba-core/src/cable.rs` (doc comments
abridged to their first lines; signatures verbatim):

```rust
/// The serial engine's view of the cable.
///
/// One instance per core. Everything that can block or touch a socket is behind
/// this trait, so the register model in `memory.rs` is the same code on a desk
/// and on a phone.
pub trait LinkCable {
    /// Units on the bus including this one. 1 means the peer has gone.
    fn units(&self) -> u8;
    /// This unit's Multi-Player ID. 0 is the parent.
    fn id(&self) -> u8;
    /// Present `word` as this unit's SIOMLT_SEND.
    fn set_output(&mut self, word: u16);
    /// Parent: begin clocking `own` out to the children.
    fn parent_start(&mut self, own: u16);
    /// Parent: every unit's word for the transfer [`LinkCable::parent_start`]
    /// began, blocking until the children answer.
    fn parent_result(&mut self) -> Option<[u16; MAX_UNITS]>;
    /// Child: the parent clocked a transfer into us, as `(parent's word, the
    /// word we answered with)`.
    fn child_clock(&mut self) -> Option<(u16, u16)>;
    /// Throw away transfers in flight, because the game just selected
    /// Multi-Player mode and anything from before that is not addressed to the
    /// protocol it is about to run.
    fn flush(&mut self);
}
```

The design adds two provided methods so `LocalCable` and every existing test
compile untouched:

```rust
    /// Charge emulated time to the cable's pacing clock. Called once per
    /// scanline by run_frame_polling. Default: no pacing.
    fn advance(&mut self, _cycles: u32) {}
    /// Should the serial engine pause wall time before the next scanline?
    /// True only on a paced child whose budget is spent and whose queue is
    /// empty. Default: never.
    fn hold(&self) -> bool { false }
    /// The hold loop starved waiting for the parent; disengage pacing.
    fn hold_starved(&mut self) {}
```

Housekeeping found while reading, for whoever implements: the doc comment at
`cable.rs:102` links `[`LinkCable::wait_for_clock`]`, a method that does not
exist on the trait today (this design's `hold` is its descendant; fix the
link when the method lands). And `LAG_HEALTHY = 0`'s comment says "zero means
drained" while `on_packet` pushes before computing `lag()`, so a child that
is keeping up actually reports 1 in every reply; nothing compares against the
constant today, but the next reader will trip on it.

## Appendix B: what mGBA actually does (the survey the owner asked for)

Files read: `src/gba/sio/lockstep.c` (1063 lines) and its header,
`src/core/lockstep.c` + `include/mgba/core/lockstep.h`, `src/gb/sio/lockstep.c`.

### It is in-process, full stop

**mGBA's lockstep is built for cooperating threads in one process with
effectively zero latency. There is no network transport for it anywhere in
the tree.** `grep -rln lockstep` over the mirror returns exactly eight files:
the three core/sio implementations, their headers, and Qt's
`MultiplayerController`, which runs N emulator windows in ONE process and
syncs them with a shared `GBASIOLockstepCoordinator` guarded by a `Mutex`.
"Sleep" is `mCoreThreadWaitFromThread` (park this core's thread), "wake" is
`mCoreThreadStopWaiting`; see `mLockstepThreadUserSleep/Wake` in
`src/core/lockstep.c`. No sockets, no serialization of events for a wire, no
tolerance for delivery delay. **Anyone porting it expecting the network part
to be solved will find there is no network part.** It is a model for the
STATE MACHINE and for where blocking belongs, and nothing else.

The constants prove it was never meant for 8 ms links: `LOCKSTEP_INTERVAL`
is 4096 cycles (~244 us). A secondary may run at most that far ahead before
it parks (`_lockstepEvent`: a secondary with an empty queue and
`nextEvent <= LOCKSTEP_INTERVAL` sleeps). Replayed over a network, that is a
synchronization round every quarter millisecond, ~68 per frame; at 8 ms each
that is half a second of wall time per emulated frame. The GB variant is the
same shape at `LOCKSTEP_INCREMENT = 512`.

### How it advances time, mapped onto our vocabulary

- Shared clock: the coordinator holds `cycle`; each player carries
  `cycleOffset`, and `GBASIOLockstepTime(player) = mTimingCurrentTime(...) -
  player->cycleOffset` puts everyone on one timebase. Our equivalent is the
  session cycle counter plus wire deltas; we cannot share memory, so we
  integrate instead of subtracting.
- Who may run: a secondary runs until it is `LOCKSTEP_INTERVAL` ahead of the
  shared clock, then sleeps; player 0 advances the shared clock in its
  scheduler callback and wakes everyone. Our equivalent: the child's budget
  horizon H, except H is a full frame because every top-up costs a datagram
  rather than a mutex release.
- Transfers are events with timestamps: `GBASIOLockstepDriverStart` (the
  SIOCNT start bit, our `multi_start`) stamps `SIO_EV_TRANSFER_START` with
  `finishCycle = timestamp + GBASIOTransferCycles(...)` (their
  `TRANSFER_CYCLES` analogue, the table our own cycle counts were taken
  from), queues it to the secondaries, and then, critically:
- **The equivalent of "the parent cannot continue until the child answers":
  `GBASIOLockstepCoordinatorWaitOnPlayers` parks the PRIMARY'S WHOLE CORE
  THREAD at transfer START.** Each secondary, on reaching the event's
  timestamp in its own emulated timeline, stages its SIOMLT_SEND into
  `coordinator->multiData` (`_setData` inside the `SIO_EV_TRANSFER_START`
  case of `_lockstepEvent`), schedules its own completion at `finishCycle`,
  acks (`GBASIOLockstepCoordinatorAckPlayer` clears its bit in
  `coordinator->waiting`, wakes the primary when the mask empties, and parks
  the secondary in the same breath). Both ends then run the busy window in
  their own timelines and read the staged words at
  `GBASIOLockstepDriverFinishMultiplayer`.
- The stale-word hazard does not exist for mGBA because a transfer cannot
  HAPPEN until every core has emulated its way to the moment of transfer and
  staged the word it holds AT THAT MOMENT. The word is captured at the right
  point in each timeline by stopping the world. The residue when it goes
  wrong anyway is `"MULTI did not receive data. Are we running behind?"`
  followed by `memset(data, 0xFF, ...)`, their version of our
  `cable_failures` path.
- `_hardSync` fires every `HARD_SYNC_INTERVAL = 0x80000` cycles (~31 ms) even
  with no transfer, so idle cores cannot drift unboundedly. Our `TAG_TICK` is
  the same idea pointed the other way (feeding instead of fencing).
- One more borrowable idea: their driver serializes lockstep state into
  savestates (`GBASIOLockstepSerializedState`, 0x1F0 bytes, versioned). We
  deliberately do not, and section 7 owns that as a stated limitation.

Our design is the latency-tolerant relaxation of exactly this machine: where
mGBA pins every core to the transfer instant (exact, affordable at zero
latency), we pin the child inside a one-frame window behind the parent's
reported position and let the hardware contract (the master's inter-clock
gap exceeds the slave's servicing time) cover the slack, paying one round
trip per transfer instead of one mutex handoff.

### Why not gpSP's approach instead

gpSP never synchronizes clocks at all: `serial_proto.c` recognizes ~16 games
by code, fakes the master locally at a fixed IRQ rate, and swaps whole 1+8
protocol frames asynchronously "with some relaxed rules on latency". That is
why it links Pokemon over real networks at 60 fps, and why Mario Kart has
never linked on it. PocketRustAdvance's cable exists to be a cable, not a
per-game protocol library (the first paragraph of `cable.rs` is this
argument), so the comparison is noted and declined.

### Considered and rejected for now: input-exchange netplay

The only known route to 60 fps linking over a real network is to stop
networking the cable entirely: run BOTH cores on BOTH devices joined by
`local_pair()`, exchange only controller input (one small packet per frame,
one RTT of input delay), and rely on byte-identical determinism, with state
hashes to detect divergence. The savestate contract makes our cores unusually
good candidates, and the idle-skip numbers say one phone can run two cores.
But it is a different, much larger project: it replicates whole cores rather
than fitting through the `LinkCable` seam, needs divergence recovery, and
doubles emulation load. The owner has accepted 14 fps lockstep; this is
recorded so the 60 fps question has a filed answer, not as a proposal.
