// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! Executable memory for the recompiler.
//!
//! This is the one piece of the JIT that is pure platform plumbing, and it is
//! deliberately first: if a target cannot hand us memory we are allowed to
//! execute, nothing else in the recompiler matters. Nothing else in this fleet
//! has ever needed it, so there was no in-house pattern to copy.
//!
//! `gba-core` has a single path dependency and no external crates, which is a
//! property worth keeping, so the system calls are declared by hand rather than
//! pulled in through `libc` or `windows-sys`.
//!
//! ## Protection
//!
//! The slab is mapped read-write-execute for its whole life. That is what gpSP
//! does and it is what makes a lazily compiled block cheap: a block is written
//! and then jumped into, with no syscall in between. The stricter shape,
//! writing under RW and flipping to RX, costs a syscall per block and is worth
//! revisiting only if a target refuses RWX. Android app processes allow it,
//! their own runtime needs it, which is why gpSP's dynarec works there at all.

/// A fixed-size slab of executable memory, filled by bumping a cursor.
///
/// Compiled blocks are never freed individually. When the slab fills, the whole
/// thing is reset and everything is recompiled, which is also the simplest
/// correct answer to invalidation.
pub struct CodeBuffer {
    ptr: *mut u8,
    len: usize,
    used: usize,
}

impl CodeBuffer {
    /// Map `len` bytes of executable memory. Returns `None` if the platform
    /// refuses, which a caller must treat as "run the interpreter" rather than
    /// as a fatal error.
    pub fn new(len: usize) -> Option<Self> {
        if len == 0 {
            return None;
        }
        let ptr = unsafe { sys::map(len) };
        if ptr.is_null() {
            return None;
        }
        Some(CodeBuffer { ptr, len, used: 0 })
    }

    /// Bytes written so far.
    pub fn used(&self) -> usize {
        self.used
    }

    /// Total capacity in bytes.
    pub fn capacity(&self) -> usize {
        self.len
    }

    /// Forget every block. The caller must drop all pointers into the slab
    /// first; nothing here can check that for it.
    pub fn reset(&mut self) {
        self.used = 0;
    }

    /// Append machine code and return its entry address, or `None` if the slab
    /// is full. The pointer stays valid until [`CodeBuffer::reset`].
    pub fn push(&mut self, code: &[u8]) -> Option<*const u8> {
        if code.len() > self.len - self.used {
            return None;
        }
        // SAFETY: used + code.len() <= len, the mapping is writable for its
        // whole life, and the ranges cannot overlap because `code` is a
        // caller-owned buffer while this mapping is private to us.
        let at = unsafe {
            let at = self.ptr.add(self.used);
            core::ptr::copy_nonoverlapping(code.as_ptr(), at, code.len());
            at
        };
        self.used += code.len();
        // Must happen before anything jumps to `at`.
        unsafe { sync_icache(at, code.len()) };
        Some(at)
    }
}

impl Drop for CodeBuffer {
    fn drop(&mut self) {
        // SAFETY: ptr/len are exactly what `map` returned, and this is the only
        // owner, so nothing can still be executing out of the slab.
        unsafe { sys::unmap(self.ptr, self.len) }
    }
}

// The slab owns its mapping outright, so moving it between threads is sound. It
// is deliberately not Sync: two threads bumping one cursor would interleave
// half-written blocks.
unsafe impl Send for CodeBuffer {}

#[cfg(windows)]
mod sys {
    use core::ffi::c_void;

    extern "system" {
        fn VirtualAlloc(addr: *mut c_void, size: usize, typ: u32, protect: u32) -> *mut c_void;
        fn VirtualFree(addr: *mut c_void, size: usize, typ: u32) -> i32;
    }

    const MEM_COMMIT: u32 = 0x0000_1000;
    const MEM_RESERVE: u32 = 0x0000_2000;
    const MEM_RELEASE: u32 = 0x0000_8000;
    const PAGE_EXECUTE_READWRITE: u32 = 0x40;

    pub unsafe fn map(size: usize) -> *mut u8 {
        VirtualAlloc(
            core::ptr::null_mut(),
            size,
            MEM_COMMIT | MEM_RESERVE,
            PAGE_EXECUTE_READWRITE,
        ) as *mut u8
    }

    pub unsafe fn unmap(ptr: *mut u8, _size: usize) {
        // MEM_RELEASE wants a zero size and the base VirtualAlloc returned.
        VirtualFree(ptr as *mut c_void, 0, MEM_RELEASE);
    }
}

#[cfg(unix)]
mod sys {
    use core::ffi::c_void;

    extern "C" {
        fn mmap(
            addr: *mut c_void,
            len: usize,
            prot: i32,
            flags: i32,
            fd: i32,
            off: i64,
        ) -> *mut c_void;
        fn munmap(addr: *mut c_void, len: usize) -> i32;
    }

    const PROT_READ: i32 = 1;
    const PROT_WRITE: i32 = 2;
    const PROT_EXEC: i32 = 4;
    const MAP_PRIVATE: i32 = 2;
    // The one constant that genuinely differs across the unixes we build for.
    #[cfg(not(target_vendor = "apple"))]
    const MAP_ANONYMOUS: i32 = 0x20;
    #[cfg(target_vendor = "apple")]
    const MAP_ANONYMOUS: i32 = 0x1000;

    pub unsafe fn map(size: usize) -> *mut u8 {
        let p = mmap(
            core::ptr::null_mut(),
            size,
            PROT_READ | PROT_WRITE | PROT_EXEC,
            MAP_PRIVATE | MAP_ANONYMOUS,
            -1,
            0,
        );
        // mmap reports failure as MAP_FAILED, which is -1 rather than null.
        if p as isize == -1 {
            core::ptr::null_mut()
        } else {
            p as *mut u8
        }
    }

    pub unsafe fn unmap(ptr: *mut u8, size: usize) {
        munmap(ptr as *mut c_void, size);
    }
}

/// Make freshly written bytes visible to the instruction fetcher.
///
/// aarch64 needs this and x86-64 does not, and that asymmetry is the classic
/// first bug of a new ARM recompiler: the data cache has to be cleaned to the
/// point of unification and the stale instruction-cache lines invalidated, or
/// the core can execute whatever happened to be in that memory before. It
/// presents as random corruption on the device while the desktop stays clean,
/// so it is written out here before any emitter exists to need it.
#[cfg(target_arch = "aarch64")]
unsafe fn sync_icache(ptr: *mut u8, len: usize) {
    // CTR_EL0 reports line sizes as log2 words: bits 19..16 D-cache, bits 3..0
    // I-cache. Reading it is unprivileged.
    let ctr: u64;
    core::arch::asm!("mrs {}, ctr_el0", out(reg) ctr, options(nomem, nostack));
    let dline = 4usize << ((ctr >> 16) & 0xF);
    let iline = 4usize << (ctr & 0xF);

    let start = ptr as usize;
    let end = start + len;

    let mut a = start & !(dline - 1);
    while a < end {
        core::arch::asm!("dc cvau, {}", in(reg) a, options(nostack, preserves_flags));
        a += dline;
    }
    core::arch::asm!("dsb ish", options(nostack, preserves_flags));

    let mut a = start & !(iline - 1);
    while a < end {
        core::arch::asm!("ic ivau, {}", in(reg) a, options(nostack, preserves_flags));
        a += iline;
    }
    core::arch::asm!("dsb ish", "isb", options(nostack, preserves_flags));
}

/// x86-64 keeps its instruction cache coherent with stores, so there is nothing
/// to do. Named and documented rather than left implicit, because "it worked on
/// the desktop" is exactly how the aarch64 path above gets forgotten.
#[cfg(not(target_arch = "aarch64"))]
unsafe fn sync_icache(_ptr: *mut u8, _len: usize) {}

#[cfg(test)]
mod tests {
    use super::*;

    /// Proof of life, and the gate for the whole recompiler: emit a function
    /// that returns 42, call it, and check. If this fails on a target, the JIT
    /// has to stay off there and the interpreter carries it.
    #[test]
    fn emitted_code_actually_runs() {
        let mut buf = CodeBuffer::new(4096).expect("executable memory");

        #[cfg(target_arch = "x86_64")]
        let code: &[u8] = &[
            0xB8, 0x2A, 0x00, 0x00, 0x00, // mov eax, 42
            0xC3, // ret
        ];
        #[cfg(target_arch = "aarch64")]
        let code: &[u8] = &[
            0x40, 0x05, 0x80, 0x52, // mov w0, #42
            0xC0, 0x03, 0x5F, 0xD6, // ret
        ];

        let entry = buf.push(code).expect("room for the function");
        // SAFETY: `entry` points at the bytes just written, which form a
        // complete function for this architecture taking no arguments and
        // returning u32 in the C ABI's return register.
        let f: extern "C" fn() -> u32 = unsafe { core::mem::transmute(entry) };
        assert_eq!(f(), 42, "emitted code did not run");
    }

    /// A second block must not land on top of the first, and the slab must
    /// refuse rather than overrun.
    #[test]
    fn two_blocks_coexist_and_a_full_slab_refuses() {
        let mut buf = CodeBuffer::new(4096).expect("executable memory");

        #[cfg(target_arch = "x86_64")]
        let (a, b) = (
            [0xB8, 0x01, 0x00, 0x00, 0x00, 0xC3].as_slice(), // mov eax, 1; ret
            [0xB8, 0x02, 0x00, 0x00, 0x00, 0xC3].as_slice(), // mov eax, 2; ret
        );
        #[cfg(target_arch = "aarch64")]
        let (a, b) = (
            [0x20, 0x00, 0x80, 0x52, 0xC0, 0x03, 0x5F, 0xD6].as_slice(), // mov w0,#1; ret
            [0x40, 0x00, 0x80, 0x52, 0xC0, 0x03, 0x5F, 0xD6].as_slice(), // mov w0,#2; ret
        );

        let fa = buf.push(a).expect("first block");
        let fb = buf.push(b).expect("second block");
        assert_ne!(fa, fb, "two blocks must not share an address");
        // SAFETY: as above, both ranges hold complete functions.
        let (fa, fb): (extern "C" fn() -> u32, extern "C" fn() -> u32) =
            unsafe { (core::mem::transmute(fa), core::mem::transmute(fb)) };
        assert_eq!((fa(), fb()), (1, 2), "a later block overwrote an earlier one");
    }

    #[test]
    fn the_cursor_accounts_exactly() {
        // The OS rounds the mapping up to a page, so ask for a tiny capacity:
        // what is under test is the accounting, not the page size.
        let mut buf = CodeBuffer::new(8).expect("executable memory");
        assert_eq!(buf.capacity(), 8);
        assert!(buf.push(&[0x90; 8]).is_some(), "an exact fit must be accepted");
        assert_eq!(buf.used(), 8);
        assert!(buf.push(&[0x90]).is_none(), "a full slab must refuse");
        buf.reset();
        assert_eq!(buf.used(), 0);
        assert!(buf.push(&[0x90; 8]).is_some(), "reset must free the whole slab");
    }
}
