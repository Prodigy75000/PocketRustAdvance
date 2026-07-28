// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Program Status Register (CPSR/SPSR) and processor modes.

/// The seven ARM7TDMI operating modes, encoded by the low 5 bits of the PSR.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    User = 0b10000,
    Fiq = 0b10001,
    Irq = 0b10010,
    Supervisor = 0b10011,
    Abort = 0b10111,
    Undefined = 0b11011,
    System = 0b11111,
}

impl Mode {
    pub fn from_bits(bits: u32) -> Option<Mode> {
        Some(match bits & 0x1F {
            0b10000 => Mode::User,
            0b10001 => Mode::Fiq,
            0b10010 => Mode::Irq,
            0b10011 => Mode::Supervisor,
            0b10111 => Mode::Abort,
            0b11011 => Mode::Undefined,
            0b11111 => Mode::System,
            _ => return None,
        })
    }
}

/// Condition field of every ARM instruction (bits 31..28) and of Thumb branch.
/// Evaluated against the NZCV flags.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Cond {
    Eq, Ne, Cs, Cc, Mi, Pl, Vs, Vc,
    Hi, Ls, Ge, Lt, Gt, Le, Al, Nv,
}

impl Cond {
    pub fn decode(nibble: u32) -> Cond {
        use Cond::*;
        match nibble & 0xF {
            0x0 => Eq, 0x1 => Ne, 0x2 => Cs, 0x3 => Cc,
            0x4 => Mi, 0x5 => Pl, 0x6 => Vs, 0x7 => Vc,
            0x8 => Hi, 0x9 => Ls, 0xA => Ge, 0xB => Lt,
            0xC => Gt, 0xD => Le, 0xE => Al, _ => Nv,
        }
    }
}

/// The condition-code flags, held as bits 31..28 of the CPSR. Broken out so the
/// ALU can set them without reasoning about the rest of the PSR.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Flags {
    pub n: bool,
    pub z: bool,
    pub c: bool,
    pub v: bool,
}

impl Flags {
    /// Extract NZCV from a full CPSR word (bits 31..28).
    pub fn from_cpsr(cpsr: u32) -> Flags {
        Flags {
            n: cpsr & (1 << 31) != 0,
            z: cpsr & (1 << 30) != 0,
            c: cpsr & (1 << 29) != 0,
            v: cpsr & (1 << 28) != 0,
        }
    }

    /// Write NZCV back into a CPSR word, preserving every other bit.
    pub fn to_cpsr(&self, cpsr: u32) -> u32 {
        let mut c = cpsr & 0x0FFF_FFFF;
        if self.n {
            c |= 1 << 31;
        }
        if self.z {
            c |= 1 << 30;
        }
        if self.c {
            c |= 1 << 29;
        }
        if self.v {
            c |= 1 << 28;
        }
        c
    }

    /// Evaluate a condition code against the current flags. This is pure and
    /// fully unit-testable on its own — the first green test in the suite.
    pub fn eval(&self, cond: Cond) -> bool {
        use Cond::*;
        match cond {
            Eq => self.z,
            Ne => !self.z,
            Cs => self.c,
            Cc => !self.c,
            Mi => self.n,
            Pl => !self.n,
            Vs => self.v,
            Vc => !self.v,
            Hi => self.c && !self.z,
            Ls => !self.c || self.z,
            Ge => self.n == self.v,
            Lt => self.n != self.v,
            Gt => !self.z && (self.n == self.v),
            Le => self.z || (self.n != self.v),
            Al => true,
            Nv => false, // "never" on ARMv4 (later repurposed; unused here)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn signed_comparisons_follow_n_v() {
        // GE is true when N == V; LT when they differ. This is the classic
        // pitfall (unsigned CS/CC vs signed GE/LT) so pin it early.
        let ge = Flags { n: true, v: true, ..Default::default() };
        assert!(ge.eval(Cond::Ge));
        assert!(!ge.eval(Cond::Lt));

        let lt = Flags { n: true, v: false, ..Default::default() };
        assert!(lt.eval(Cond::Lt));
        assert!(!lt.eval(Cond::Ge));
    }

    #[test]
    fn unsigned_higher_needs_c_and_not_z() {
        let hi = Flags { c: true, z: false, ..Default::default() };
        assert!(hi.eval(Cond::Hi));
        assert!(!hi.eval(Cond::Ls));
    }

    #[test]
    fn al_always_nv_never() {
        let f = Flags::default();
        assert!(f.eval(Cond::Al));
        assert!(!f.eval(Cond::Nv));
    }
}