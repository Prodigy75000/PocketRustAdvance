<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# PocketRustAdvance

A clean-room Game Boy Advance emulator core, written from scratch in Rust.

> **README in progress.** This is a skeleton — the intro/voice is still being written.

## Status

<!-- TODO: write the overview. Bullet facts to draw from: -->
- ARM7TDMI CPU validated against the TomHarte SingleStepTests vectors (all 42 files pass; the only gaps are officially-`UNPREDICTABLE` cases).
- PPU: bitmap modes 3/4/5, tile modes 0–2, affine backgrounds and sprites, windows, blending.
- APU: the four PSG channels plus both Direct Sound FIFO channels.
- Timers, DMA, IRQs, and BIOS SWIs.
- Save types: SRAM, Flash (64/128 KB), EEPROM (512 B / 8 KB).
- Save-states.
- Ships as a libretro core.

## Clean-room policy

This core is implemented **only** from official documentation (the ARM7TDMI
manuals and GBATEK) and black-box conformance vectors. **No other emulator's
source is read or referenced.** See [`docs/README.md`](docs/README.md) for the
reference material (the documents themselves are not committed — they are
third-party copyrighted works).

## Building

Requires a Rust toolchain.

```sh
# headless runner: run a ROM for N frames and dump a PNG frame
cargo run --release -p gba-runner --bin gba -- <rom.gba> <frames>

# tests
cargo test -p gba-core
```

The libretro core is `gba-libretro` (builds to `libgbacore_libretro`).

## Layout

- `crates/gba-core` — the emulator (CPU, PPU, APU, memory/bus, DMA, timers, save, state)
- `crates/gba-libretro` — libretro front-end (bundles an open-source BIOS replacement)
- `crates/gba-runner` — headless dev runner + the TomHarte conformance harness
- `docs/` — index of the clean-room reference documentation

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE).

The bundled `crates/gba-libretro/open_gba_bios.bin` is an open-source BIOS
replacement, redistributable under its own terms.
<!-- TODO: name/credit the open BIOS (Normmatt / Cult-of-GBA) + its license. -->
