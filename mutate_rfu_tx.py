"""Prove each new transmit-path test can fail.

Every mutation here restores the behaviour the fix removed, or breaks the thing
the test claims to pin. A mutation that does not APPLY looks exactly like a test
that cannot fail, so each one is checked for its expected hit count and the run
aborts if the text was not found.
"""
import io, subprocess, sys

RFU = "crates/gba-core/src/rfu.rs"

MUTATIONS = [
    (
        "RTX_WAIT stops retransmitting (the bug, restored)",
        "                if self.cmd == CMD_RTX_WAIT {\n                    self.tx_rtx += 1;\n                } else {",
        "                if false {\n                    self.tx_rtx += 1;\n                } else {",
        1,
        "a_bare_retransmit_sends_the_last_block_again",
    ),
    (
        "host length bounded by this command again (the bug, restored)",
        "                        if blen > TX_MAX_HOST {",
        "                        if blen > (self.len.saturating_sub(1) as usize) * 4 {",
        1,
        "a_host_may_ask_to_send_more_than_it_just_handed_over",
    ),
    (
        "host medium limit widened to the whole length field",
        "const TX_MAX_HOST: usize = 90;",
        "const TX_MAX_HOST: usize = 127;",
        1,
        "a_block_past_what_the_medium_carries_is_refused_and_counted",
    ),
    (
        "an empty room is no longer counted",
        "                        if !any {\n                            self.tx_no_client += 1;\n                        }",
        "                        if !any {}",
        1,
        "a_host_sending_into_an_empty_room_is_counted_not_lost",
    ),
    (
        "sending while idle is quietly accepted",
        "_ => return Err(1),",
        "_ => return Ok(0),",
        2,
        "sending_while_neither_hosting_nor_attached_is_an_error",
    ),
    (
        "client medium limit widened past its slot",
        "const TX_MAX_CLIENT: usize = 16;",
        "const TX_MAX_CLIENT: usize = 90;",
        1,
        "a_client_block_past_its_slot_is_refused_and_counted",
    ),
]

with io.open(RFU, encoding="utf-8", newline="") as f:
    ORIG = f.read()

fails = []
for name, old, new, count, test in MUTATIONS:
    if ORIG.count(old) != count:
        sys.exit("MUTATION DID NOT APPLY: %s (wanted %d, found %d)" % (name, count, ORIG.count(old)))
    with io.open(RFU, "w", encoding="utf-8", newline="") as f:
        f.write(ORIG.replace(old, new, count))
    r = subprocess.run(
        ["cargo", "test", "-p", "gba-core", "rfu::tests::"],
        capture_output=True, text=True,
    )
    out = r.stdout + r.stderr
    killed = (r.returncode != 0)
    # Did the test we NAMED do the killing, or did something else?
    named = ("%s ... FAILED" % test) in out or ("%s stdout" % test) in out
    status = "KILLED" if killed else "SURVIVED"
    by = "by its own test" if named else "but NOT by %s" % test
    print("%-9s %-55s %s" % (status, name, by))
    if not killed or not named:
        fails.append(name)

with io.open(RFU, "w", encoding="utf-8", newline="") as f:
    f.write(ORIG)
print("\nrestored. unkilled or mis-attributed: %d" % len(fails))
for f_ in fails:
    print("  " + f_)
sys.exit(1 if fails else 0)
