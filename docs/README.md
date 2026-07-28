# PocketRustAdvance — Reference Documentation

Official ARM specifications for the ARM7TDMI (the GBA's CPU core, an **ARMv4T**
part), plus the community hardware reference for the rest of the console.
These are the authoritative inputs for clean-room implementation: read behavior
from the spec, never from another emulator's source.

> **The document files themselves are not committed** — they are copyrighted by
> ARM Ltd. / Martin Korth and only referenced here. Download them into this
> `docs/` folder from the **Sources** links at the bottom if you want them
> locally; the repository ships only this index.

## Official ARM documentation

| File | ARM doc # | Pages | What it's for |
|------|-----------|-------|---------------|
| `ARM7TDMI_Data_Sheet_DDI0029.pdf` | ARM DDI 0029G (Rev 3) | 284 | The classic ARM7TDMI data sheet. Per-instruction encoding, instruction-set detail, and cycle timing (incl. multiply early-termination timing). The reference most GBA CPU work leans on. |
| `ARM7TDMI_Technical_Reference_Manual_DDI0210.pdf` | ARM DDI 0210 | 286 | ARM7TDMI Technical Reference Manual. Bus/signal behavior, memory interface, cycle-by-cycle operation. |
| `ARM7TDMI-S_Technical_Reference_Manual_DDI0234.pdf` | ARM DDI 0234B | 242 | ARM7TDMI-S (synthesizable) TRM. Companion to DDI 0210; useful cross-check on documented behavior. |
| `ARM_Architecture_Reference_Manual_DDI0100.pdf` | ARM DDI 0100 (ARM ARM) | 1138 | The ISA spec. Definitive per-instruction semantics; marks which behavior belongs to which architecture version (v4/v4T vs v5+). |

## Community hardware reference

| File | Provenance | What it's for |
|------|-----------|---------------|
| `GBATEK_gba_hardware_reference.htm` | GBATEK by Martin Korth (`problemkaputt.de/gbatek.htm`) | De-facto GBA hardware reference: memory map, PPU, DMA, timers, I/O registers, cart/BIOS. **Reverse-engineered documentation**, not an official Nintendo/ARM spec — there is no official GBA hardware doc. It describes observed hardware behavior (like a datasheet), not any emulator's implementation. Flagged here so provenance is explicit; treat it the way you treat the TomHarte vectors — a black-box description of the real chip, not source to copy. |

## The multiply carry flag — settled by the official spec

The architect's open question ("does any official doc specify the `MUL`/`MLA`
carry precisely?") is answered directly and definitively by the ARM
Architecture Reference Manual (DDI 0100). Verbatim, from three instruction
pages:

> **MUL** (p. 594): "The MUL instruction is defined to leave the C flag
> unchanged in ARMv5 and above. In earlier versions of the architecture, the
> value of the C flag was UNPREDICTABLE after a MUL instruction."

> **MULS** (p. 231): "The MULS instruction is defined to leave the C flag
> unchanged in ARM architecture version 5 and above. In earlier versions of the
> architecture, the value of the C flag was UNPREDICTABLE after a MULS
> instruction."

> **MLAS** (p. 217): "The MLAS instruction is defined to leave the C flag
> unchanged in ARMv5 and above. In earlier versions of the architecture, the
> value of the C flag was UNPREDICTABLE after an MLAS instruction."

The GBA's ARM7TDMI is **ARMv4T** — an "earlier version." So the C flag after
`MUL`/`MLA` is officially **UNPREDICTABLE**, with **no specified value**. The
"leave C unchanged, documented as unpredictable" implementation is exactly
spec-correct. The residual gap against NBA-generated vectors is not a bug: it is
the one implementation-specific, officially-undefined bit, and closing it would
mean reverse-engineering NBA's internals through its outputs. Correctly left
alone.

(The DDI 0029 data sheet documents multiply *timing* — the m-cycle
early-termination based on the multiplier operand — precisely, and that part is
worth implementing from the datasheet. It does not define the carry value; on
this the ARM ARM above is the last word.)

## Sources

- ARM7TDMI Data Sheet (DDI 0029G): `https://ww1.microchip.com/downloads/en/DeviceDoc/DDI0029G_7TDMI_R3_trm.pdf` (official ARM PDF, mirrored by Microchip/Atmel)
- ARM7TDMI TRM (DDI 0210): `https://documentation-service.arm.com/static/5f4786a179ff4c392c0ff819` (ARM documentation service)
- ARM7TDMI-S TRM (DDI 0234B): `https://documentation-service.arm.com/static/5e8e13a9fd977155116a3368` (ARM documentation service)
- ARM Architecture Reference Manual (DDI 0100): `https://documentation-service.arm.com/static/5f8dacc8f86e16515cdb865a` (ARM documentation service)
- GBATEK: `https://problemkaputt.de/gbatek.htm`
