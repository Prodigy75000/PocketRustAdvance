//! Minimal binary (de)serialization for save-states. No external deps: each
//! component writes its fields in a fixed order via [`Writer`] and reads them
//! back via [`Reader`]. Reads are bounds-checked; a short/corrupt buffer sets
//! the reader's `failed` flag instead of panicking, so `load_state` can reject.

pub const MAGIC: u32 = 0x5052_4153; // "PRAS"
pub const VERSION: u8 = 1;

#[derive(Default)]
pub struct Writer {
    pub buf: Vec<u8>,
}

impl Writer {
    pub fn u8(&mut self, v: u8) {
        self.buf.push(v);
    }
    pub fn u16(&mut self, v: u16) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u32(&mut self, v: u32) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u64(&mut self, v: u64) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn u128(&mut self, v: u128) {
        self.buf.extend_from_slice(&v.to_le_bytes());
    }
    pub fn i32(&mut self, v: i32) {
        self.u32(v as u32);
    }
    pub fn bool(&mut self, v: bool) {
        self.u8(v as u8);
    }
    /// A length-prefixed byte block.
    pub fn bytes(&mut self, b: &[u8]) {
        self.u32(b.len() as u32);
        self.buf.extend_from_slice(b);
    }
}

pub struct Reader<'a> {
    buf: &'a [u8],
    pos: usize,
    pub failed: bool,
}

impl<'a> Reader<'a> {
    pub fn new(buf: &'a [u8]) -> Self {
        Reader { buf, pos: 0, failed: false }
    }
    /// Bytes not yet consumed. Used to make newly-appended state blocks (e.g. the
    /// APU, added after the format shipped) optional, so older states still load.
    pub fn remaining(&self) -> usize {
        self.buf.len().saturating_sub(self.pos)
    }
    fn take(&mut self, n: usize) -> &[u8] {
        if self.pos + n > self.buf.len() {
            self.failed = true;
            return &[];
        }
        let s = &self.buf[self.pos..self.pos + n];
        self.pos += n;
        s
    }
    pub fn u8(&mut self) -> u8 {
        self.take(1).first().copied().unwrap_or(0)
    }
    pub fn u16(&mut self) -> u16 {
        let b = self.take(2);
        if b.len() == 2 {
            u16::from_le_bytes([b[0], b[1]])
        } else {
            0
        }
    }
    pub fn u32(&mut self) -> u32 {
        let b = self.take(4);
        if b.len() == 4 {
            u32::from_le_bytes([b[0], b[1], b[2], b[3]])
        } else {
            0
        }
    }
    pub fn u64(&mut self) -> u64 {
        let b = self.take(8);
        if b.len() == 8 {
            u64::from_le_bytes(b.try_into().unwrap())
        } else {
            0
        }
    }
    pub fn u128(&mut self) -> u128 {
        let b = self.take(16);
        if b.len() == 16 {
            u128::from_le_bytes(b.try_into().unwrap())
        } else {
            0
        }
    }
    pub fn i32(&mut self) -> i32 {
        self.u32() as i32
    }
    pub fn bool(&mut self) -> bool {
        self.u8() != 0
    }
    /// Read a length-prefixed byte block, copying into `dst` (only the bytes
    /// that fit; a size mismatch is tolerated so states survive minor changes).
    pub fn bytes_into(&mut self, dst: &mut [u8]) {
        let n = self.u32() as usize;
        let src = self.take(n);
        let m = src.len().min(dst.len());
        dst[..m].copy_from_slice(&src[..m]);
    }
    /// Read a length-prefixed byte block as an owned vector.
    pub fn bytes_vec(&mut self) -> Vec<u8> {
        let n = self.u32() as usize;
        self.take(n).to_vec()
    }
}
