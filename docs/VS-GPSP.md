# PocketRustAdvance against gpSP

Last measured 2026-10-02 at `e1af4e6`. gpSP is the GBA core Trophy Hub ships
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

### gpSP runs these, this core cannot

| title | here |
|---|---|
| Hikaru no Go 2 (Japan) | white screen, dead in all three BIOS modes |
| Grand Theft Auto Advance (USA, Europe) | black screen on confirming a save name |

Three entries left this table on 2026-10-02 and all three were the same bug.
**Tetris Worlds** (both European builds and the USA one), **Hagane no
Renkinjutsushi Omoide no Sonata** and the **Frogger family** were every one of
them reading the BIOS region from outside the BIOS, which we allowed and
hardware does not. They now draw: Tetris Worlds at 228 colours from 1, Frogger
Adventures 2 at 2037 from 16. Awaiting the owner on hardware.

Hikaru no Go 2 is the one real regression left, and it is narrow rather than
vague: a pinned loop in IWRAM at `0x03000B0C` from frame 10 through frame 1800,
interrupts never enabled, no background ever turned on. Hikaru no Go 1 runs
here without trouble.

### Neither core runs these

Crash and Spyro Superpack (identical white screen after picking a game from the
collection), Mortal Kombat Deadly Alliance and Tournament Edition (both stick on
the Midway logo), the first title in 2 Game Pack Hot Wheels. Madden NFL 06 runs
on both and has the same obscured in-game menu on both.

**Net: eight titles gained, two lost.** That is no longer a wash. It is still
not the main argument for a swap, because two of the eight are prototypes or
collections rather than games anyone is waiting for, but compatibility has
stopped being a reason against.

Not yet smoked on hardware, flagged by the BIOS-share column rather than the
colour count, and several of them moved on 2026-10-02: Motocross Maniacs
Advance (USA + Japan), Disney Princess (five language builds), Gadget Racers and
Penny Racers, Banjo-Pilot (USA + Europe), Konami Collector's Series Arcade
Classics, Minna no Shiiku Series 2, and Hagane no Renkinjutsushi Meisou no
Rondo, a second Hagane title that was missing from this page entirely.

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
