# PocketRustAdvance against gpSP

Last measured 2026-10-02 at `4c159ad`. gpSP is the GBA core Trophy Hub ships
today; this core is reachable only behind a debug preference that defaults off.

This is the head-to-head the owner asked for when this work started: not an
argument that the core is good, but a list of what a swap would gain and what it
would cost, with the losses first-class. Where a number is unmeasured it says so.

## The short version

A swap today would gain motion-control support, which gpSP does not have at all,
and seven titles gpSP cannot boot. It would cost real-time clock support,
rumble, and two titles gpSP runs. The clock is the serious one: it is the only
item on the list that silently degrades games that otherwise look fine.

The compatibility side of this page was rewritten on 2026-10-02. Three of the
five regressions went away in one commit (`e1af4e6`, BIOS read protection), and
so did a title neither core could run.

Speed is NOT on that list. An earlier draft of this page called it a risk and the
owner corrected it: both cores sit at the app's 600 fps fast-forward cap on his
device, so the four-times gap measured below never reaches a user.

## Compatibility

2727 licensed ROMs, 1800 frames each, booting through the bundled open BIOS,
which is the shipping configuration. **225 rows flagged by the automated cut,
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

**Broken here, works on gpSP (2 titles, 3 ROMs)**

| title | here |
|---|---|
| Hikaru no Go 2 (Japan) | white screen, dead in all three BIOS modes. A loop in IWRAM at `0x03000B0C` pinned from frame 10 to 1800, interrupts never enabled. Hikaru no Go 1 is fine. |
| Grand Theft Auto Advance (USA, Europe) | black screen on confirming a save name. NOT the save: there are zero EEPROM transactions. It writes garbage into the I/O registers (0xEFEF and friends into DISPCNT, DISPSTAT and IE), which clears its own V-blank IRQ enable, and then waits in VBlankIntrWait for the interrupt it just disabled. |

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

**The sweep flags 223 of 2727 rows and that is not a failure count.** It pulses
A and Start forever, which walks a game into pause menus and soft resets no
player visits, and it scores a frame by counting colours, which calls a dark
title screen dead. Six of the 223 are titles the owner has personally cleared.
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
| Real-time clock (Pokemon berries and tides, Boktai) | **no** | yes |
| Solar sensor (Boktai) | no | no |

gpSP has no sensor emulation whatsoever: the only matches for gyro, tilt or
accelerometer anywhere in its tree are in the generic libretro-common support
files, none in its own emulation code.

**This is the strongest argument for a swap and it is not a quality argument.**
Motion-gated achievements in sets for the sensor cartridges are unobtainable on
gpSP at any level of skill, because the input they require does not exist. A swap
does not make those games play better, it makes them completable. Nothing else on
this page changes what a player can finish.

**The real-time clock is the strongest argument against.** It is absent here and
present in gpSP, and unlike a black screen its absence is invisible: Pokemon
Ruby, Sapphire and Emerald boot and play fine without it and quietly never grow a
berry or change tide. That is a worse failure mode than a game that does not
start, and those are among the most played GBA titles there are.

## Performance

Mario Kart Super Circuit, 1800 frames, this desktop (Ryzen 9 9950X), rendering
enabled in all three:

```text
  this core   1374 ms    22x realtime
  mGBA         517 ms    58x realtime     2.7x faster than us
  gpSP         341 ms    88x realtime     4.0x faster than us
```

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

1. **No real-time clock.** Silent degradation in heavily played titles.
2. **A more complete memory map exposes racy achievement sets.** Advance Wars 2
   awards "Shiny Design Room" seconds after boot here. That one is a flaw in RA
   set 1768, reproduced by the owner on Linkboy and ticketed, and it is not caused
   by anything this core gets wrong. But gpSP's narrower map hides a class of set
   bug that this core will surface, and false unlocks reach a player's account and
   cannot be taken back.
3. **Two titles regress**, down from five.

## What replacing gpSP actually needs

In the order I would do it:

1. **Real-time clock.** The only item that makes a popular game quietly worse.
   Shares the GPIO port already built for the gyro, so the hardware plumbing
   exists; this is the S3511 protocol on top of it.
2. **The two regressions**, down from five. Hikaru no Go 2 is pinned to a
   single loop address and GTA Advance to a single EEPROM write, so both are
   narrow rather than open-ended.
3. **Rumble.** Small, and it is the other half of the Twisted cartridge already
   emulated here.
4. Solar, for Boktai. Neither core has it, so it is a shared gap and not part of
   the promotion case.

Performance work is deliberately absent. The margin is structural but sits above
the range anyone can perceive, so chasing it would buy nothing a player notices.

Items 1 and 3 are both cartridge hardware on a port this core already models. The
honest estimate is that the feature gap is smaller than the compatibility gap
looks, and that the clock is most of the remaining distance.
