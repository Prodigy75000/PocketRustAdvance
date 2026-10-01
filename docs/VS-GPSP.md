# PocketRustAdvance against gpSP

Last measured 2026-10-01 at `e38dc6a`. gpSP is the GBA core Trophy Hub ships
today; this core is reachable only behind a debug preference that defaults off.

This is the head-to-head the owner asked for when this work started: not an
argument that the core is good, but a list of what a swap would gain and what it
would cost, with the losses first-class. Where a number is unmeasured it says so.

## The short version

A swap today would gain motion-control support, which gpSP does not have at all,
and six titles gpSP cannot boot. It would cost real-time clock support, rumble,
five titles gpSP runs, and roughly a factor of four in CPU speed. The clock is
the serious one: it is the only item on the list that silently degrades games
that otherwise look fine.

## Compatibility

2727 licensed ROMs, 1800 frames each, booting through the bundled open BIOS,
which is the shipping configuration. **30 flagged, 98.90% pass.**

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

everGirl is a qualified win: it reaches gameplay here with a sprite flicker on
the protagonist, so it is better rather than correct.

### gpSP runs these, this core cannot

| title | here |
|---|---|
| Hikaru no Go 2 (Japan) | white screen, dead in all three BIOS modes |
| Tetris Worlds (Europe) | black screen; the USA build hangs on the press-start splash |
| Hagane no Renkinjutsushi Omoide no Sonata (Japan) | branches into BIOS reset code |
| Frogger Adventures 2, Journey, Mahou no Kuni (5 ROMs) | stuck in the BIOS |
| Grand Theft Auto Advance (USA, Europe) | black screen on confirming a save name |

### Neither core runs these

Crash and Spyro Superpack (identical white screen after picking a game from the
collection), Mortal Kombat Deadly Alliance and Tournament Edition (both stick on
the Midway logo), the first title in 2 Game Pack Hot Wheels, Legends of Wrestling
II. Madden NFL 06 runs on both and has the same obscured in-game menu on both.

**Net: six titles gained, five lost.** That is close to a wash and is not on its
own a reason to switch.

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

**Treat that as a lower bound on the gap, not an upper one.** This core is a
plain interpreter. gpSP is built around a dynamic recompiler whose reason for
existing is ARM handhelds, and a Windows x86 build is the configuration where it
is least ahead. On the hardware that matters the margin is likely wider.

What is known on device: both cores hold 60 fps on the owner's Galaxy S25 Ultra.
**Low-end performance is entirely unmeasured**, and it is the gap in this
document I would close first. The Galaxy Tab A9+ already attached to the bench is
the obvious instrument.

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

## Risks a swap would carry

1. **No real-time clock.** Silent degradation in heavily played titles.
2. **Speed.** Four times slower than gpSP on the one platform measured, and the
   platform that matters is the one not measured.
3. **A more complete memory map exposes racy achievement sets.** Advance Wars 2
   awards "Shiny Design Room" seconds after boot here. That one is a flaw in RA
   set 1768, reproduced by the owner on Linkboy and ticketed, and it is not caused
   by anything this core gets wrong. But gpSP's narrower map hides a class of set
   bug that this core will surface, and false unlocks reach a player's account and
   cannot be taken back.
4. **Five titles regress.**

## What replacing gpSP actually needs

In the order I would do it:

1. **Real-time clock.** The only item that makes a popular game quietly worse.
   Shares the GPIO port already built for the gyro, so the hardware plumbing
   exists; this is the S3511 protocol on top of it.
2. **Low-end device performance measurement.** Cheap, and it either removes risk 2
   from this page or reframes the whole project.
3. **The five regressions**, three of which are one engine (Frogger) and one of
   which is already narrowed to a specific BIOS interaction (Hagane).
4. **Rumble.** Small, and it is the other half of the Twisted cartridge already
   emulated here.
5. Solar, for Boktai. Neither core has it, so it is a shared gap and not part of
   the promotion case.

Items 1 and 4 are both cartridge hardware on a port this core already models. The
honest estimate is that the feature gap is smaller than the compatibility gap
looks, and that the unmeasured risk is performance.
