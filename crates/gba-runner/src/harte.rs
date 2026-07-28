// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Parser for the SingleStepTests ARM7TDMI `.json.bin` binary vector format.
//!
//! Layout (little-endian), mirrored from the repo's `transcode_json.py`:
//!   file:  [magic u32 = 0xD33DBAE0][num_tests u32] then `num_tests` tests
//!   test:  [full_sz i32] initial-state, final-state, transactions, opcode-block
//!   state: [full_sz i32][pad u32] then 40 u32:
//!            R[0..16] R_fiq[16..23] R_svc[23..25] R_abt[25..27]
//!            R_irq[27..29] R_und[29..31] CPSR[31] SPSR[32..37]
//!            pipeline[37..39] access[39]
//!   txs:   [full_sz i32][magic i32 = 3][count i32] then count * 6 u32
//!            (kind, size, addr, data, cycle, access)
//!   op:    [full_sz i32][pad u32][opcode u32][base_addr u32]
//!
//! Each sub-block is prefixed by its own byte size, so we advance the cursor by
//! those sizes and never guess.

pub const FILE_MAGIC: u32 = 0xD33D_BAE0;

#[derive(Clone)]
pub struct State {
    /// The User/System register bank for r0..r15. In privileged modes the
    /// active r8..r14 come from the banked arrays below, not from here.
    pub r: [u32; 16],
    /// FIQ bank for r8..r14 (7 registers).
    pub r_fiq: [u32; 7],
    /// r13/r14 banks for the remaining privileged modes.
    pub r_svc: [u32; 2],
    pub r_abt: [u32; 2],
    pub r_irq: [u32; 2],
    pub r_und: [u32; 2],
    pub cpsr: u32,
    /// SPSR banks in the order [fiq, svc, abt, irq, und].
    pub spsr: [u32; 5],
    pub pipeline: [u32; 2],
    #[allow(dead_code)]
    pub access: u32,
}

impl State {
    /// The SPSR of `cpsr`'s current mode, or 0 for User/System (no SPSR).
    pub fn spsr_for_mode(&self) -> u32 {
        match self.cpsr & 0x1F {
            0x11 => self.spsr[0], // FIQ
            0x13 => self.spsr[1], // Supervisor
            0x17 => self.spsr[2], // Abort
            0x12 => self.spsr[3], // IRQ
            0x1B => self.spsr[4], // Undefined
            _ => 0,
        }
    }

    /// The register file as the CPU actually sees it in `cpsr`'s mode, with the
    /// banked r8..r14 overlaid onto the User/System base.
    pub fn active_regs(&self) -> [u32; 16] {
        let mut r = self.r;
        match self.cpsr & 0x1F {
            0x11 => {
                // FIQ banks r8..r14.
                r[8..15].copy_from_slice(&self.r_fiq);
            }
            0x13 => {
                r[13] = self.r_svc[0];
                r[14] = self.r_svc[1];
            }
            0x17 => {
                r[13] = self.r_abt[0];
                r[14] = self.r_abt[1];
            }
            0x12 => {
                r[13] = self.r_irq[0];
                r[14] = self.r_irq[1];
            }
            0x1B => {
                r[13] = self.r_und[0];
                r[14] = self.r_und[1];
            }
            _ => {} // User / System: no banking
        }
        r
    }
}

#[derive(Clone, Copy)]
pub struct Tx {
    pub kind: u32, // 0 = read, 1 = write (opcode fetches are reads)
    pub size: u32, // bytes
    pub addr: u32,
    pub data: u32,
    #[allow(dead_code)]
    pub cycle: u32,
    #[allow(dead_code)]
    pub access: u32,
}

pub struct Test {
    pub initial: State,
    pub final_: State,
    pub transactions: Vec<Tx>,
    pub opcode: u32,
    pub base_addr: u32,
}

struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
}

impl<'a> Reader<'a> {
    fn u32_at(&self, off: usize) -> u32 {
        u32::from_le_bytes(self.buf[off..off + 4].try_into().unwrap())
    }
    fn i32_at(&self, off: usize) -> i32 {
        i32::from_le_bytes(self.buf[off..off + 4].try_into().unwrap())
    }

    fn state(&mut self) -> State {
        let full_sz = self.i32_at(self.pos) as usize;
        let base = self.pos + 8; // skip full_sz + pad
        let v = |i: usize| self.u32_at(base + i * 4);
        let mut r = [0u32; 16];
        for (i, slot) in r.iter_mut().enumerate() {
            *slot = v(i);
        }
        let st = State {
            r,
            r_fiq: [v(16), v(17), v(18), v(19), v(20), v(21), v(22)],
            r_svc: [v(23), v(24)],
            r_abt: [v(25), v(26)],
            r_irq: [v(27), v(28)],
            r_und: [v(29), v(30)],
            cpsr: v(31),
            spsr: [v(32), v(33), v(34), v(35), v(36)],
            pipeline: [v(37), v(38)],
            access: v(39),
        };
        self.pos += full_sz;
        st
    }

    fn transactions(&mut self) -> Vec<Tx> {
        let full_sz = self.i32_at(self.pos) as usize;
        let count = self.i32_at(self.pos + 8) as usize;
        let mut txs = Vec::with_capacity(count);
        let mut off = self.pos + 12;
        for _ in 0..count {
            txs.push(Tx {
                kind: self.u32_at(off),
                size: self.u32_at(off + 4),
                addr: self.u32_at(off + 8),
                data: self.u32_at(off + 12),
                cycle: self.u32_at(off + 16),
                access: self.u32_at(off + 20),
            });
            off += 24;
        }
        self.pos += full_sz;
        txs
    }

    fn opcode_block(&mut self) -> (u32, u32) {
        let full_sz = self.i32_at(self.pos) as usize;
        let opcode = self.u32_at(self.pos + 8);
        let base_addr = self.u32_at(self.pos + 12);
        self.pos += full_sz;
        (opcode, base_addr)
    }

    fn test(&mut self) -> Test {
        self.pos += 4; // outer full_sz (we advance via the sub-block sizes)
        let initial = self.state();
        let final_ = self.state();
        let transactions = self.transactions();
        let (opcode, base_addr) = self.opcode_block();
        Test {
            initial,
            final_,
            transactions,
            opcode,
            base_addr,
        }
    }
}

/// Parse an entire `.json.bin` file into its test vectors.
pub fn parse(buf: &[u8]) -> Result<Vec<Test>, String> {
    if buf.len() < 8 {
        return Err("file too small".into());
    }
    let magic = u32::from_le_bytes(buf[0..4].try_into().unwrap());
    if magic != FILE_MAGIC {
        return Err(format!("bad magic {magic:08X}"));
    }
    let num = u32::from_le_bytes(buf[4..8].try_into().unwrap()) as usize;
    let mut rd = Reader { buf, pos: 8 };
    let mut tests = Vec::with_capacity(num);
    for _ in 0..num {
        tests.push(rd.test());
    }
    Ok(tests)
}