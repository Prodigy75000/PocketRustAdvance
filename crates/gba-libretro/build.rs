// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Stamp the build with the commit it came from.
//!
//! jniLibs and the shared Drive APK slot are mutable directories with no
//! version in them, and several agents deploy into them independently. On
//! 2026-09-22 that produced an APK that was simultaneously newer than the
//! tablet and older than the tree, and the only way to tell which core was in
//! which artifact was comparing ELF section sizes. A stamp makes the artifact
//! self-describing, which is the only fix that does not need everyone to
//! coordinate timing.
//!
//! The stamp rides in `library_version`, so it shows up in the front-end's
//! core-info line as well as in `strings -a libgbacore_libretro.so | grep build=`.

use std::process::Command;

fn main() {
    // Rebuild the stamp when HEAD moves or the index changes, otherwise cargo
    // would happily reuse a stale one and the stamp would lie.
    for p in ["../../.git/HEAD", "../../.git/index"] {
        println!("cargo:rerun-if-changed={p}");
    }

    let git = |args: &[&str]| {
        Command::new("git")
            .args(args)
            .output()
            .ok()
            .filter(|o| o.status.success())
            .map(|o| String::from_utf8_lossy(&o.stdout).trim().to_string())
    };

    // No git, or not a checkout: say so rather than emitting a hash that cannot
    // be looked up. An honest "local" beats a plausible-looking wrong commit.
    let stamp = match git(&["rev-parse", "--short", "HEAD"]) {
        Some(hash) if !hash.is_empty() => {
            // Uncommitted changes mean the hash does not name these bytes.
            let dirty = git(&["status", "--porcelain", "--untracked-files=no"])
                .map(|s| !s.is_empty())
                .unwrap_or(true);
            if dirty {
                format!("{hash}-dirty")
            } else {
                hash
            }
        }
        _ => "local".to_string(),
    };
    println!("cargo:rustc-env=PRA_BUILD={stamp}");
}
