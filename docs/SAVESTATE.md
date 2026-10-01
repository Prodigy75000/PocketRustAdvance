# Save-state format — platform-agnostic serialization (READ BEFORE TOUCHING `retro_serialize`)

**This core:** PocketRustAdvance already ships save-states. **Audit the existing
`retro_serialize` against the rules below** before it's relied on for
cross-platform netplay; bump `format_version` if a fix changes the byte layout.
FamiRust's `save.rs` byte-cursor is the reference implementation.

Trophy Hub's in-house cores must produce save-states that transfer across
platforms (iOS ↔ Android ↔ Desktop) for cross-engine netplay. The durable way is
**not** matching build binaries — it's making serialize output identical *by
construction*.

**The invariant:** `retro_serialize` output is a pure function of emulator
**state**, identical byte-for-byte on every target triple and build config.

**Rules:**
1. No native memory layout — encode field by field. Never `memcpy` a struct,
   `transmute`, or `bincode`/`derive` over layout-dependent types.
2. Fixed width + little-endian (`to_le_bytes`). Never serialize `usize`/`isize`
   (cast to a fixed width). `bool`→`u8`. Avoid floats in state.
3. Fixed field order — no `HashMap`/`HashSet` iteration in the stream.
4. Versioned header: core magic + `format_version: u16`; bump on any layout
   change; reject unknown/newer versions cleanly (no panic).
5. Validate input — reject truncated/oversized/wrong-magic rather than read OOB.
6. Deterministic emulator (no wall-clock / host-seeded RNG / float nondeterminism
   leaking into state).
7. `retro_serialize_size()` stable + equal across platforms per `format_version`
   (the netplay handshake keys on it — keep `format_version` honest).

**Required test — the golden-bytes test:** build a known state, serialize, assert
the **exact** byte string. Target-independence makes those bytes identical
everywhere, so this one host-side test guarantees cross-platform agreement. Add
round-trip (`serialize→unserialize→serialize` byte-identical) + reject tests
(truncated / oversized / wrong-magic / newer-version → `false`, no panic).

**Canonical spec (private resources hub, authoritative if this drifts):**
`../TrophyHubResources/specs/play/IN_HOUSE_CORE_SAVESTATE_SPEC.md`
