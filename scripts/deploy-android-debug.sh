#!/usr/bin/env bash
#
# One-liner: build the arm64 PocketRustAdvance libretro core from the current
# working tree, drop it into the TrophyHubAndroid debug app's jniLibs, and
# assemble the debug APK (optionally install it to an attached device).
#
#   scripts/deploy-android-debug.sh            # build core -> jniLibs -> assembleDebug
#   scripts/deploy-android-debug.sh install    # ... then adb install -r the APK
#
# Why this exists: the core and the app (TrophyHubAndroid) are separate repos in
# the TrophyHub umbrella, so getting a core change onto a device is a fixed
# three-step dance. This pins it so no one rediscovers it. Only arm64-v8a ships
# (see the app's abiFilters); the two devices are arm64.
#
# The .so files under the app's jniLibs are gitignored and untracked there, so
# this drops a build artifact rather than writing to another repo's history.
#
# AUTHORIZATION BOUNDARY (TH-Android, 2026-09-22). assembleDebug and installDebug
# are standing authorization in TrophyHubAndroid, which is why this script ends
# in a gradle call rather than handing back. RELEASE is NOT: assembleRelease,
# bundleRelease and bundle-deploy.mjs go through the owner manually and must
# never be added here. Whoever runs this should also say what they built and
# when, because jniLibs and the Drive APK slot have several writers and an
# unattributed artifact is how two near-misses happened that day.
#
# This core is DEBUG-ONLY for now: it is still being brought up against the
# commercial library and must not go into a Play AAB while gpSP is the shipping
# GBA core. The app keeps libgpsp_libretro.so alongside it.
#
# Save states: the format is versioned (gba_core::state::VERSION, magic "PRAS")
# and the serialize output is a binding byte-identical contract across every
# in-house core, so a change to the state layout has to bump the version and be
# mirrored anywhere else this core ships.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ANDROID="$(cd "$REPO/../TrophyHubAndroid" && pwd)"
TRIPLE="aarch64-linux-android"
ABI="arm64-v8a"
SO="libgbacore_libretro.so"
# Unlike FamiRust this crate is part of the umbrella workspace, so its artifacts
# land in TrophyHub/target, not a repo-local target/. The umbrella's
# .cargo/config.toml supplies the NDK r27.1 linker and the 16 KB max-page-size
# link arg that Android 15+ requires.
TARGET_DIR="${CARGO_TARGET_DIR:-$REPO/../target}"

echo "[0/3] cargo test (a bad core takes GBA with it)"
( cd "$REPO" && cargo test --release -p gba-core )

echo "[1/3] cargo build --release -p gba-libretro --target $TRIPLE"
( cd "$REPO" && cargo build --release -p gba-libretro --target "$TRIPLE" )

SRC="$TARGET_DIR/$TRIPLE/release/$SO"
DST="$ANDROID/app/src/main/jniLibs/$ABI/$SO"
[ -f "$SRC" ] || { echo "ERROR: built core not found at $SRC" >&2; exit 1; }

# Android 15+ rejects native libs that are not 16 KB aligned, and the failure
# shows up at install time rather than at build time, so check it here.
if command -v readelf >/dev/null 2>&1; then
  if readelf -lW "$SRC" | awk '/LOAD/ {print $NF}' | grep -qv '0x4000'; then
    echo "ERROR: $SO has a PT_LOAD segment that is not 16 KB aligned" >&2
    exit 1
  fi
fi

echo "[2/3] cp core -> $DST"
cp "$SRC" "$DST"
# Keep a copy here so "what is on the device" can be answered from this repo
# alone, without reaching into the app repo.
mkdir -p "$REPO/out/release"
cp "$SRC" "$REPO/out/release/libgbacore_libretro.android-arm64.so"

echo "[3/3] ./gradlew :app:assembleDebug"
( cd "$ANDROID" && ./gradlew :app:assembleDebug )

APK="$ANDROID/app/build/outputs/apk/debug/app-debug.apk"
echo "APK: $APK"

# Installing the APK is NOT enough to test this core, and the failure is silent:
# you get a working GBA game running on the wrong emulator. The app's
# coreSlotForPlatform maps "gba" to gpSP unconditionally, and the swap to this
# core happens later from a preference that defaults OFF, so a fresh install
# packages this .so and never dlopen's it. Flagged by TH-Android 2026-09-22
# after it nearly invalidated a smoke test.
cat <<'NOTE'

  ------------------------------------------------------------------
  THE CORE IS NOT LIVE UNTIL YOU FLIP THE TOGGLE. On the device:

      Settings -> Consoles -> Game Boy Advance
        -> "Use PocketRust Advance core (beta)"

  It defaults OFF, and it applies on the NEXT ROM LOAD, not immediately.
  Without it the app runs gpSP and any result you get is gpSP's.
  ------------------------------------------------------------------

NOTE

# The debug APK always lives at this fixed Drive slot, replacing the current one,
# so a phone/tablet can pull it without a cable (Drive for Desktop syncs it up).
DRIVE_SLOT="${TROPHYHUB_DEBUG_APK:-/g/My Drive/Trophy Hub/TrophyHub-debug.apk}"
if [ -d "$(dirname "$DRIVE_SLOT")" ]; then
  cp "$APK" "$DRIVE_SLOT"
  echo "Drive: $DRIVE_SLOT (replaced)"
else
  echo "note: Drive slot dir missing, skipped ($DRIVE_SLOT)"
fi

if [ "${1:-}" = "install" ]; then
  echo "adb install -r (device must be authorized for USB debugging)"
  adb install -r "$APK"
fi
