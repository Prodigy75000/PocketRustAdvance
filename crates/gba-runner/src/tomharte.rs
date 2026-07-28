// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! TomHarte ARM7TDMI conformance runner.
//!
//! Loads a SingleStepTests `.json.bin` file, runs each vector through a real
//! [`gba_core::cpu::Arm7tdmi`], and diffs the resulting register/flag/pipeline
//! state against the expected `final` state.
//!
//! Memory is modeled by [`TxBus`], a *transaction-replay* bus: the vectors don't
//! ship an initial memory image, they ship the exact ordered list of bus
//! transactions the step must produce. Each CPU read is answered from the next
//! transaction's data; each write is checked against it. This is why the
//! [`gba_core::bus::Bus`] trait tags every access — the same tags let a later
//! mode also assert cycle/access exactness.
//!
//! Usage: `tomharte [path-to.json.bin]`
//!   (defaults to tests/vectors/arm_data_proc_immediate.json.bin)

mod harte;

use std::panic::{catch_unwind, AssertUnwindSafe};
use std::path::PathBuf;

use gba_core::bus::{Access, Bus};
use gba_core::cpu::Arm7tdmi;
use harte::{Test, Tx};

/// Replays a vector's transaction list as the CPU's memory. Reads return the
/// recorded data; writes are verified. Records any deviation.
struct TxBus {
    txs: Vec<Tx>,
    idx: usize,
    errors: Vec<String>,
}

impl TxBus {
    fn new(txs: Vec<Tx>) -> Self {
        TxBus {
            txs,
            idx: 0,
            errors: Vec::new(),
        }
    }

    fn next(&mut self) -> Option<Tx> {
        let t = self.txs.get(self.idx).copied();
        self.idx += 1;
        t
    }

    // Note: a transaction's `kind` is code-fetch (0) vs data-access (1), NOT
    // read vs write — the direction is implied by the instruction. So we verify
    // address (and data, for writes) but not kind.
    fn do_read(&mut self, addr: u32, _size: u32) -> u32 {
        match self.next() {
            None => {
                self.errors.push(format!("unexpected read @ {addr:08X} (no more txns)"));
                0
            }
            Some(t) => {
                if t.addr != addr {
                    self.errors
                        .push(format!("read addr {addr:08X} != expected {:08X}", t.addr));
                }
                t.data
            }
        }
    }

    fn do_write(&mut self, addr: u32, size: u32, val: u32) {
        match self.next() {
            None => self.errors.push(format!("unexpected write @ {addr:08X} (no more txns)")),
            Some(t) => {
                if t.addr != addr {
                    self.errors
                        .push(format!("write addr {addr:08X} != expected {:08X}", t.addr));
                }
                // Compare only the meaningful low bytes for sub-word writes.
                let mask = if size >= 4 { u32::MAX } else { (1u32 << (size * 8)) - 1 };
                if (t.data & mask) != (val & mask) {
                    self.errors
                        .push(format!("write data {val:08X} != expected {:08X}", t.data));
                }
            }
        }
    }

    /// True once every recorded transaction has been consumed exactly.
    fn fully_consumed(&self) -> bool {
        self.idx == self.txs.len()
    }
}

impl Bus for TxBus {
    fn read8(&mut self, addr: u32, _a: Access) -> u8 {
        self.do_read(addr, 1) as u8
    }
    fn read16(&mut self, addr: u32, _a: Access) -> u16 {
        self.do_read(addr, 2) as u16
    }
    fn read32(&mut self, addr: u32, _a: Access) -> u32 {
        self.do_read(addr, 4)
    }
    fn write8(&mut self, addr: u32, val: u8, _a: Access) {
        self.do_write(addr, 1, val as u32)
    }
    fn write16(&mut self, addr: u32, val: u16, _a: Access) {
        self.do_write(addr, 2, val as u32)
    }
    fn write32(&mut self, addr: u32, val: u32, _a: Access) {
        self.do_write(addr, 4, val)
    }
    fn tick(&mut self, _n: u32) {}
}

/// Run one vector; return `Ok(())` on an exact match or `Err(reason)`.
fn run_test(test: &Test) -> Result<(), String> {
    let mut cpu = Arm7tdmi::new();
    // Load the full banked register file; the CPU derives the active view and
    // re-banks itself on any mode change during the step.
    let init = &test.initial;
    cpu.load_full(
        init.cpsr,
        init.r,
        init.r_fiq,
        init.r_svc,
        init.r_abt,
        init.r_irq,
        init.r_und,
        init.spsr,
    );
    cpu.pipeline = init.pipeline;

    let mut bus = TxBus::new(test.transactions.clone());

    // A stray panic (e.g. an unimplemented path) fails just this vector.
    let stepped = catch_unwind(AssertUnwindSafe(|| cpu.step(&mut bus)));
    if stepped.is_err() {
        return Err("panic during step".into());
    }

    let f = &test.final_;
    // Compare against the expected active view (banks overlaid per final mode).
    let expected = f.active_regs();
    let mut diffs = Vec::new();
    for i in 0..16 {
        if cpu.r[i] != expected[i] {
            diffs.push(format!("R{i}={:08X}!={:08X}", cpu.r[i], expected[i]));
        }
    }
    if cpu.cpsr != f.cpsr {
        diffs.push(format!("CPSR={:08X}!={:08X}", cpu.cpsr, f.cpsr));
    }
    if cpu.pipeline != f.pipeline {
        diffs.push(format!(
            "pipe=[{:08X},{:08X}]!=[{:08X},{:08X}]",
            cpu.pipeline[0], cpu.pipeline[1], f.pipeline[0], f.pipeline[1]
        ));
    }
    if !bus.fully_consumed() {
        diffs.push(format!("txns {}/{} consumed", bus.idx, bus.txs.len()));
    }
    diffs.extend(bus.errors);

    if diffs.is_empty() {
        Ok(())
    } else {
        Err(diffs.join(", "))
    }
}

fn main() {
    let path = std::env::args().nth(1).map(PathBuf::from).unwrap_or_else(|| {
        let mut p = PathBuf::from(env!("CARGO_MANIFEST_DIR"));
        p.push("../../tests/vectors/arm_data_proc_immediate.json.bin");
        p
    });

    let buf = match std::fs::read(&path) {
        Ok(b) => b,
        Err(e) => {
            eprintln!("cannot read {}: {e}", path.display());
            std::process::exit(2);
        }
    };
    let tests = match harte::parse(&buf) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("parse error: {e}");
            std::process::exit(2);
        }
    };

    // Silence per-panic backtraces; we count them as failures ourselves.
    std::panic::set_hook(Box::new(|_| {}));

    let mut passed = 0usize;
    let mut samples: Vec<String> = Vec::new();
    for test in &tests {
        match run_test(test) {
            Ok(()) => passed += 1,
            Err(reason) => {
                if samples.len() < 12 {
                    samples.push(format!("  op {:08X}: {reason}", test.opcode));
                }
            }
        }
    }

    let total = tests.len();
    println!("{}", path.file_name().unwrap().to_string_lossy());
    println!("  {passed}/{total} passed ({:.2}%)", 100.0 * passed as f64 / total as f64);
    if passed != total {
        println!("  sample failures:");
        for s in &samples {
            println!("{s}");
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn txbus_replays_reads_in_order() {
        let txs = vec![Tx { kind: 0, size: 4, addr: 0x0800_0000, data: 0xCAFEBABE, cycle: 0, access: 0 }];
        let mut b = TxBus::new(txs);
        assert_eq!(b.read32(0x0800_0000, Access::Seq), 0xCAFEBABE);
        assert!(b.fully_consumed());
        assert!(b.errors.is_empty());
    }
}