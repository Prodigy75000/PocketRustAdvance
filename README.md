<!-- SPDX-License-Identifier: GPL-3.0-or-later -->
# PocketRustAdvance

A clean-room Game Boy Advance emulator core written from scratch in Rust.
No C, no bindings, and no lifted code, just the hardware modeled from the docs.
The core passes the full ARM7TDMI conformance suite and runs most commercial Game Boy Advance titles.

## Status

| Component | State |
|-----------|-------|
| CPU (ARM7TDMI) | ✅ complete ARM + Thumb instruction sets, all modes, banking, exceptions |
| CPU accuracy | ✅ all 42 TomHarte SingleStepTests files pass (~2.1M vectors); the only gaps are officially `UNPREDICTABLE` cases (ARMv4 multiply carry) |
| PPU | ✅ bitmap modes 3/4/5, tile modes 0-2, affine backgrounds and sprites, windows, alpha / brightness blending |
| Audio (APU) | ✅ the four PSG channels (2 pulse, wave, noise) plus both Direct Sound FIFO channels, stereo |
| Timers | ✅ all four, prescaler and cascade |
| DMA | ✅ all four channels; immediate, VBlank, HBlank, and sound-FIFO timing |
| Interrupts / BIOS | ✅ full interrupt controller; BIOS SWIs run through a bundled open BIOS (with an HLE fallback) |
| Cartridge saves | ✅ SRAM, Flash (64 / 128 KB), EEPROM (512 B / 8 KB), battery-backed |
| Save states | ✅ full machine state, deterministic round-trip |

Compatibility: boots and plays the large majority of the commercial Game Boy
Advance library. The remaining misses are a handful of edge cases (a few
copy-protected titles, some special-hardware carts) rather than whole classes of
game.

## How it is built

The ARM7TDMI is validated instruction by instruction against the TomHarte
SingleStepTests vectors *before* the rest of the console exists: the CPU has to
pass ~2.1 million single-step cases through the same `Bus` trait the real system
later plugs into. Behavior comes only from the official ARM manuals (ARM7TDMI
data sheet + TRMs, the ARM ARM) and GBATEK; no other emulator's source is read
or referenced. Where the vectors and the spec disagree, the spec wins, the
ARMv4 multiply carry flag is officially `UNPREDICTABLE`, so it is left that way.

## Layout

```
crates/
  gba-core/      the emulator library (no I/O deps)
    src/cpu/     ARM + Thumb decoder, execution, barrel shifter, HLE BIOS
    src/{memory, ppu, apu, save, state}.rs
  gba-runner/    headless dev runner + the TomHarte conformance harness
  gba-libretro/  libretro core (bundles an open-source BIOS)
docs/            index of the clean-room reference documentation
```

## Running

The dev runner is headless: it runs a ROM for N frames and writes a PNG of the
framebuffer (actual play is through the libretro core, below).

```sh
# Run a ROM for 600 frames and dump gba_frame.png
cargo run --release -p gba-runner --bin gba -- path/to/rom.gba 600

# Run the accuracy + unit test suite
cargo test --release

# TomHarte ARM7TDMI conformance (vectors downloaded to tests/vectors/)
cargo run --release -p gba-runner --bin tomharte
```

## libretro core

```sh
# Native build -> target/release/libgbacore_libretro.{so,dll,dylib}
cargo build --release -p gba-libretro
```

To cross-compile the Android `.so`, point `CARGO_TARGET_AARCH64_LINUX_ANDROID_LINKER`
at your NDK's `aarch64-linux-android21-clang` and build with
`--target aarch64-linux-android`.

## License

GPL-3.0-or-later. See [`LICENSE`](LICENSE).

The bundled `crates/gba-libretro/open_gba_bios.bin` is an open-source, freely
redistributable BIOS replacement (Normmatt's, not Nintendo's) and keeps its own
license.
