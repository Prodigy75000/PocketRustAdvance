# PocketRustAdvance against gpSP

Last measured 2026-10-02 at `57f4d9c`. gpSP is the GBA core Trophy Hub ships
today; this core is reachable only behind a debug preference that defaults off.

This is the head-to-head the owner asked for when this work started: not an
argument that the core is good, but a list of what a swap would gain and what it
would cost, with the losses first-class. Where a number is unmeasured it says so.

## The short version

A swap today would gain motion-control support, which gpSP does not have at all,
and seven titles gpSP cannot boot. It would cost rumble, and the link cable,
which is the real one. The real-time clock was the serious item on this list
until 2026-10-02 and is now implemented.

**No title that gpSP boots is dead here, and the compatibility list is closed.**
Both of the two that were went on 2026-10-02: Hikaru no Go 2 boots and plays
(`d98365e`), and Grand Theft Auto Advance went in two steps, the DMA enable edge
(`d98365e`) and then the ARM7TDMI bus cycles we were not charging (`b59bf8e`).
GTA now reaches Portland gameplay. **The owner confirmed it on hardware and
cross-checked the same scenes against mGBA and gpSP.**

The compatibility side of this page was rewritten on 2026-10-02. Three of the
five regressions went away in one commit (`e1af4e6`, BIOS read protection), and
so did a title neither core could run.

Speed is NOT on that list. An earlier draft of this page called it a risk and the
owner corrected it: both cores sit at the app's 600 fps fast-forward cap on his
device, so the four-times gap measured below never reaches a user.

## Compatibility

2727 licensed ROMs, 1800 frames each, booting through the bundled open BIOS,
which is the shipping configuration. **244 rows flagged by the automated cut,
of which a handful are real**; see the caveat below, which is why this section
quotes titles rather than a percentage.

Read "flagged" as "worth a human look", not "broken". The automated cut is a
screen and it has been wrong in both directions: on 2026-09-30 it called seven
working games dead because the window was 10 seconds rather than 30, it could not
see a cartridge that had crashed into the BIOS boot animation at all, and the
auto-input it uses to skip title screens manufactured failures in twelve more.
Every verdict below is the owner on hardware, not the sweep.

### gpSP cannot boot these, this core can

| title | gpSP |
|---|---|
| Diddy Kong Pilot (2005 prototype) | does not boot |
| Bubble Bobble Old and New (USA, Europe, Japan) | black screen |
| Elevator Action Old and New (Japan) | black screen |
| Harobots Robo Hero Battling (Japan) | white screen |
| everGirl (USA) | black screen in gameplay with audio screeching |
| Hello Kitty Collection - Miracle Fashion Maker (Japan) | does not run |
| Legends of Wrestling II (USA, Europe) | does not boot |

everGirl is a qualified win: it reaches gameplay here with a sprite flicker on
the protagonist, so it is better rather than correct.

### Open compatibility issues, in full

This is the whole list, and it is short. Every entry is the owner on hardware;
nothing here comes from the automated cut.

**Broken here, works on gpSP: none.** Grand Theft Auto Advance headed this
table until 2026-10-02. It now plays: boots, takes a save name, plays the intro
cutscene and runs gameplay in Portland. **Owner-confirmed on hardware, with the
same scenes cross-checked against mGBA and gpSP.** USA and Europe, under the
bundled open BIOS and a real BIOS dump.

One thing to know if a GTA save state is ever used to judge this: every state
captured before `b59bf8e` is poisoned. The corruption happened earlier in the
scene than the crash, so the state's own work RAM already contains the damaged
code and it will still die on load. Judge a timing fix from a fresh run.

Hikaru no Go 2 used to head this table as a white screen dead in all three BIOS
modes. It boots to its title screen and into the kana name entry as of `d98365e`.

**What GTA Advance's second fault actually was.** Not a code generator: the
game's cutscene streamer and its software rasteriser SHARE the IWRAM address
0x03000100 across a scene change (the heap hands the freed stream buffer to the
rasteriser). The streamer leaves one buffer swap pending every frame, retired by
a V-count-IRQ callback at line 50. On hardware the scene teardown is still
short of its code DMA when that IRQ lands, so the stale swap hits the dead
buffer and the rasteriser loads intact. Our CPU ran the teardown stretch a few
thousand cycles too fast (missing load/store broken-fetch N cycles, missing
internal cycles, flat-rate DMA timing), finished the DMA one scanline before
the IRQ, and the stale swap stamped 64 prologue-less bytes over the fresh code;
the first call after the mission card then popped fourteen registers that were
never pushed and ran off through the BIOS SWI table. The owner's two save
states were captured after that stamp, so they still crash by construction;
the fix is visible from a cold boot.

The fix is the missing hardware cycle accounting itself, not a tuned margin:
after it the code DMA lands 22 scanlines after the swap retires. Corpus sweep
at 1800 frames (dumps/_dmatime.tsv vs dumps/_catchup2.tsv): flagged 244 vs
233, but every one of the 42 flag flips is a title caught mid-fade by the
fixed 30-frame sampling window after the phase shift; all 42 render a full
screen within 100 frames of the cutoff (scratch/reg_recheck2.txt), and 31
formerly flagged titles flipped clean by the same mechanism. Zero titles
actually broke.

**Broken here, not known to work anywhere (1 title)**

| title | here |
|---|---|
| Blades of Thunder (USA) | hangs on garbage VRAM. It walks an allocator free list whose head is zero; the list was never built, and until 2026-10-02 an over-permissive BIOS read was handing it an accidental escape. |

**Cosmetic, gameplay unaffected (1 title, 2 ROMs)**

| title | here |
|---|---|
| Babar to the Rescue (USA, Europe) | one dialogue line renders as two stray glyphs, and a transition shows an undrawn bitmap page. Narrowed to number formatting: the broken line contains "99 99 100" and the pure-text line after it is perfect. |

**Broken on both cores, so not a swap risk**

One dump of Crash and Spyro (Season of Ice plus Huge Adventure, USA; the other
four Crash and Spyro dumps are healthy) and the first title in 2 Game Pack Hot
Wheels. Madden NFL 06 runs on both with the same obscured in-game menu.

Mortal Kombat Deadly Alliance and Tournament Edition used to be in this
paragraph. **All three ROMs now boot** (`4c159ad`): the open BIOS's
`RLUnCompWram` never word-aligned its header read, so a misaligned source got an
ARM7-rotated size and a 2048-byte block decompressed as 6,579,200, filling EWRAM
forever. gpSP still fails them; mGBA runs them, which is why they were worth
chasing.

**Never smoked, flagged by the BIOS-crash signal, about 13 ROMs**

2 Games in 1 Finding Nemo plus The Incredibles, 3 Games in One Super Breakout
plus Millipede plus Lunar Lander (USA and Europe), Disney Princess (five
language builds), Famista Advance (Japan), Gadget Racers and Penny Racers,
Minna no Shiiku Series 2 (Japan), NHL Hitz 20-03 (USA), Scooby-Doo 2 Monsters
Unleashed (Europe). Candidates, not failures: the same signal has flagged
Super Bust-A-Move, Madden NFL 06, the Powerpuff Girls pair and Babar, all four
of which the owner has since played without trouble.

### What the automated number is worth, which is not much

**The sweep flags 244 of 2727 rows and that is not a failure count.** It pulses
A and Start forever, which walks a game into pause menus and soft resets no
player visits, and it scores a frame by counting colours, which calls a dark
title screen dead. Six of the 233 are titles the owner has personally cleared.
Over 2026-10-01 and 02 it pointed at healthy games three separate times and
missed three real regressions that only owner device reports caught.

Quote the titles above. Do not quote a pass rate.

## Features

Verified by reading both cores, not by running them.

| | this core | gpSP |
|---|---|---|
| Tilt sensor (Yoshi Topsy-Turvy, Koro Koro Puzzle) | yes, verified on device | **no** |
| Gyro sensor (WarioWare Twisted, Mawaru) | yes, verified on device | **no** |
| Rumble (Drill Dozer, Twisted) | **no** | yes, with a core option |
| Real-time clock (Pokemon berries and tides, Boktai) | yes, as of `a50f8a5` | yes |
| Solar sensor (Boktai) | no | no |

gpSP has no sensor emulation whatsoever: the only matches for gyro, tilt or
accelerometer anywhere in its tree are in the generic libretro-common support
files, none in its own emulation code.

**This is the strongest argument for a swap and it is not a quality argument.**
Motion-gated achievements in sets for the sensor cartridges are unobtainable on
gpSP at any level of skill, because the input they require does not exist. A swap
does not make those games play better, it makes them completable. Nothing else on
this page changes what a player can finish.

**The real-time clock was the strongest argument against, and it closed on
2026-10-02 (`a50f8a5`).** It is worth recording why it mattered: unlike a black
screen its absence was invisible. Pokemon Ruby, Sapphire and Emerald boot and
play perfectly well without a clock and quietly never grow a berry or change a
tide, which is a worse failure mode than a game that does not start, on some of
the most played titles there are.

The chip is a Seiko S-3511A on the same GPIO port as the gyro, found by the
Nintendo RTC library's version string in the ROM, which hits exactly 30 of the
2727 licensed ROMs and no false positives. All seven clock carts checked latch
the clock while booting.

**Owner-confirmed on device**: Sapphire's wall clock was set at 9:02 and read
9:05 three real minutes later, advancing on its own with no input. Still
outstanding is a berry, which is the only test of elapsed time surviving a save
and reload.

The clock follows the **host wall clock**, pushed every frame, not emulated time.
So it neither pauses while the app is suspended nor runs fast in fast-forward: it
reads real time whenever a frame happens to run, which is how a cartridge whose
crystal keeps going with the console switched off behaves.

The one piece still missing for these cartridges is the **solar sensor**, which
neither core has. It is why both Boktai rows sit flagged in the sweep.

## Performance

Mario Kart Super Circuit, 1800 frames, this desktop (Ryzen 9 9950X), rendering
enabled in all three:

```text
  this core   1660 ms    18x realtime
  mGBA         517 ms    58x realtime     3.2x faster than us
  gpSP         341 ms    88x realtime     4.9x faster than us
```

Our figure was re-measured on 2026-10-02 and the previous 1374 ms was stale: the
same binary measures 1660 ms today, and so does the commit before the cycle
accounting changed, so this is not a regression from that work. The mGBA and gpSP
numbers have NOT been re-measured on the current machine, so read the ratios as
approximate.

**That ratio does not reach the user, and an earlier draft of this page was wrong
to call it a risk.** The owner's measurement on device: both cores run at 600 fps
in fast-forward on a Galaxy S25 Ultra, which is the app's cap. Both are pinned to
the ceiling, so the four-times difference is invisible from inside the product.

At 1x there is no plausible Android device that struggles with a GBA, so normal
play is not a differentiator for either core. The only place the gap could ever
surface is the fast-forward ceiling on much weaker hardware, where gpSP would
still hit the cap and this core might fall short of it. That is a ceiling on a
convenience feature, not a playability problem, and nobody has reported it.

Worth knowing rather than worth fixing. This core is a plain interpreter and
gpSP is a dynamic recompiler built for ARM handhelds, so the headroom gap is
real and structural; it simply sits above the range anyone can perceive.

## Accuracy

Where there is evidence rather than assertion:

- Advance Wars 2 work RAM at frame 30 is **byte-for-byte identical to mGBA** over
  every byte mGBA exposes, and differs from gpSP in 426 bytes of 262144. Method
  in `tools/gbaread.rs`.
- Semi-transparent sprites (OBJ mode 1) were missing here entirely until
  2026-10-01 and are present in gpSP, which rendered Super Bust-A-Move's greyed
  menu option correctly while this core did not. Fixing it changed 284 of 2727
  ROMs. **That is parity reached, not an advantage.**
- The RetroAchievements memory map is more complete here: rcheevos registers all
  three GBA regions from our descriptors, and logs "Could not map region starting
  at $048000" for gpSP, which publishes only two. Double-edged, see below.
- **The bitmap modes did not composite at all. Fixed over 2026-10-01/02.** They
  wrote pixels straight into the framebuffer, so they ignored the BG2 enable bit,
  the colour effects and sprites entirely. Three bugs the owner found on hardware
  in one evening, all the same root cause:

  - Iridion II drew a buffer it was mid-rewrite because BLDY never reached the
    picture. The game fades to black with BLDY at 16 and we showed it at full
    brightness. **gpSP was correct, this core was not.**
  - Hello Kitty Collection held a flat green backdrop for 53 frames because the
    fade never reached the backdrop either. mGBA shows the same green and covers
    it immediately, which is what pinned it down.
  - Spider-Man 2 lost every line of dialogue and Harry Potter Quidditch World Cup
    its entire language menu and splash animation, because sprites were never
    drawn in these modes. Both run mode 3 with OAM full.

  The bitmap path now builds the same compositor the tiled modes use, so the
  picture is BG2 with a priority, sprites and windows and blending all apply, and
  the two renderers share one resolve step. Corpus unchanged at 30 flagged.

- **DMA started on any write that left the enable bit set, not on its rising
  edge. Fixed 2026-10-02 in `d98365e`.** Hardware starts a transfer when enable
  goes 0 to 1; storing a 1 over a 1 does nothing. A game that read-modify-writes
  a running channel therefore re-ran the whole transfer with whatever its
  registers happened to hold. GTA Advance does exactly that while tearing down
  its sound DMA, and the phantom transfer sprayed 0x4000 words up through the
  whole I/O block from the sound FIFO, clearing DISPCNT and DISPSTAT's V-blank
  IRQ enable. mGBA tests the same edge in `GBADMAWriteCNT_HI`. Four other titles
  improved with it, and the corpus showed no regression.

- **A sound FIFO was refilled at scanline granularity rather than at the cycle
  its DMA was re-armed. Fixed 2026-10-02 in `ebda31a`.** The frame loop advances
  the APU once per line, so a re-arm executed mid-line was accounted up to 1232
  cycles away, which is 1.3 Direct Sound samples and enough to move a 16-byte
  FIFO refill to the wrong side of it. Golden Nugget Casino and Caesars Palace
  Advance mix exactly 608 bytes of PCM per two frames with no slack and ARM code
  immediately after the buffer, so the stray refill played that code as 8-bit
  samples: a loud tick several times a second, which the owner heard on hardware
  and gpSP does not have. This one was introduced by the DMA edge fix above and
  found by owner device report, not by the corpus.

- Ghost Rider and Kao the Kangaroo show garbage in the same transitions on gpSP
  and are clean here, so this class of bug runs in both directions.
- **The BIOS region was readable from anywhere. Fixed 2026-10-02 in `e1af4e6`.**
  Hardware exposes it only to code fetching from inside it, and returns the last
  BIOS opcode to everyone else. Twenty-two titles improved, because a startling
  number of GBA games dereference a pointer that is null at that moment and use
  whatever comes back. Legends of Wrestling II is the extreme case: it calls
  through the null pointer into the BIOS reset branch and the PC walks off into
  unmapped space in the first frame. **gpSP cannot run it either**, so this is a
  gain rather than parity, which is unusual for this list.

## Risks a swap would carry

1. ~~No real-time clock~~ - implemented 2026-10-02 (`a50f8a5`) and confirmed
   ticking on device; a berry is the one check still outstanding.
2. **A more complete memory map exposes racy achievement sets.** Advance Wars 2
   awards "Shiny Design Room" seconds after boot here. That one is a flaw in RA
   set 1768, reproduced by the owner on Linkboy and ticketed, and it is not caused
   by anything this core gets wrong. But gpSP's narrower map hides a class of set
   bug that this core will surface, and false unlocks reach a player's account and
   cannot be taken back.
3. ~~One title falls short of gameplay where gpSP reaches it~~ - cleared
   2026-10-02 with the cycle-accounting fixes; no known title now regresses
   against gpSP.

## What replacing gpSP actually needs

In the order I would do it:

1. **Rumble.** Small, and it is the other half of the Twisted cartridge already
   emulated here.
2. **Link cable and RFU.** Not a defect, but it is the actual reason gpSP is the
   incumbent: it was chosen for multiplayer, not for compatibility. Until this
   core serves that, a swap trades a working feature for everything else on this
   page. The serial registers are modelled with no cable attached (`1a58d70`),
   which is the floor to build on, not the feature.
3. Solar, for Boktai. Neither core has it, so it is a shared gap and not part of
   the promotion case.

~~GTA Advance's second fault~~ and ~~the real-time clock~~ were both on this
list until 2026-10-02. Compatibility is no longer on it at all, and the clock is
done pending a hardware check.

Performance work is deliberately absent. The margin is structural but sits above
the range anyone can perceive, so chasing it would buy nothing a player notices.

Items 1 and 3 are both cartridge hardware on a port this core already models. The
honest estimate is that the feature gap is smaller than the compatibility gap
looks, and that the clock is most of the remaining distance.
