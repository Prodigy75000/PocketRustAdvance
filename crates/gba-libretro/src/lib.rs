//! libretro entry points for the GBA core.
//!
//! Thin by design: once `pocketrust_common::libretro` exposes the shared
//! `retro_*` scaffolding over a `Core` trait, this crate just implements the
//! trait for `gba_core::Gba` and re-exports the boilerplate. Empty until the
//! core produces frames.
