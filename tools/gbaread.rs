// Minimal libretro host: load a core DLL, load a ROM, retro_unserialize a state,
// then read RETRO_MEMORY_SYSTEM_RAM. Purpose: use the real Mesen-S core as an
// oracle to read WRAM at a captured moment, so our clean-room core's divergence
// can be diffed byte-for-byte. No external crates (kernel32 via #[link]).
//
// Standalone dev tool, NOT part of the cargo workspace (Windows-only; loads an
// external libretro core). Build with: `rustc -O tools/mesenread.rs -o mesenread`.
// Generic: works with any libretro core DLL, not just Mesen.
//
// usage: gbaread <core.dll> <rom> <state|RESET> <hexaddr>... [--frames N] [--dumpram <path>]
//
// Copied from SuperRust/tools/mesenread.rs, which says it is generic across
// libretro cores, plus --dumpram. Used on the GBA to diff our work RAM against
// mgba_libretro.dll and gpsp_libretro.dll, both of which sit in
// TrophyHubDesktop/runtime-deps/cores_windows.
use std::ffi::c_void;
use std::os::raw::{c_char, c_uint};

#[link(name = "kernel32")]
extern "system" {
    fn LoadLibraryA(name: *const c_char) -> *mut c_void;
    fn GetProcAddress(module: *mut c_void, name: *const c_char) -> *mut c_void;
}

#[repr(C)]
struct GameInfo {
    path: *const c_char,
    data: *const c_void,
    size: usize,
    meta: *const c_char,
}

type EnvFn = extern "C" fn(c_uint, *mut c_void) -> bool;

// System/save dir handed to the core if it asks. Null-terminated.
static SYSDIR: &[u8] = b"C:\\Users\\User\\AppData\\Local\\Temp\\claude\0";

static PIXFMT: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static NONBLACK: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

extern "C" fn env_cb(cmd: c_uint, data: *mut c_void) -> bool {
    match cmd & 0xffff {
        10 => unsafe {
            // SET_PIXEL_FORMAT: record it so video_refresh can be interpreted.
            if !data.is_null() {
                PIXFMT.store(*(data as *const u32), std::sync::atomic::Ordering::Relaxed);
            }
            true
        },
        9 | 31 => unsafe {                // GET_SYSTEM_DIRECTORY / GET_SAVE_DIRECTORY
            if data.is_null() {
                return false;
            }
            *(data as *mut *const c_char) = SYSDIR.as_ptr() as *const c_char;
            true
        }
        _ => false, // everything else: not handled -> core uses defaults
    }
}

static FRAME_W: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static FRAME_H: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);
static mut FRAME_RGB: Vec<u8> = Vec::new();
static VRFRAME: std::sync::atomic::AtomicUsize = std::sync::atomic::AtomicUsize::new(0);

extern "C" fn vr_cb(d: *const c_void, w: c_uint, h: c_uint, pitch: usize) {
    use std::sync::atomic::Ordering::Relaxed;
    if d.is_null() {
        return; // duplicate frame; keep last count
    }
    let fmt = PIXFMT.load(Relaxed); // 1 = XRGB8888, 2 = RGB565, 0 = 0RGB1555
    let mut nb = 0usize;
    let mut rgb = Vec::with_capacity(w as usize * h as usize * 3);
    unsafe {
        for y in 0..h as usize {
            if fmt == 1 {
                let row = (d as *const u8).add(y * pitch) as *const u32;
                for x in 0..w as usize {
                    let p = *row.add(x);
                    if p & 0x00FF_FFFF != 0 { nb += 1; }
                    rgb.push(((p >> 16) & 0xFF) as u8);
                    rgb.push(((p >> 8) & 0xFF) as u8);
                    rgb.push((p & 0xFF) as u8);
                }
            } else {
                let row = (d as *const u8).add(y * pitch) as *const u16;
                for x in 0..w as usize {
                    let p = *row.add(x);
                    if p != 0 { nb += 1; }
                    // 0RGB1555
                    let (r, g, b) = (((p >> 10) & 0x1F) as u8, ((p >> 5) & 0x1F) as u8, (p & 0x1F) as u8);
                    rgb.push((r << 3) | (r >> 2));
                    rgb.push((g << 3) | (g >> 2));
                    rgb.push((b << 3) | (b >> 2));
                }
            }
        }
        // --vrlog: one line per delivered frame with a colour count and a
        // horizontal-edge count. A frame of garbage spikes BOTH at once, which
        // ordinary artwork does not, so a whole run can be scanned for a single
        // bad frame instead of sampled.
        if std::env::args().any(|a| a == "--vrlog") {
            let (mut edges, mut cols) = (0usize, std::collections::HashSet::new());
            for y in 0..h as usize {
                for x in 0..w as usize {
                    let i = (y * w as usize + x) * 3;
                    let px = (rgb[i], rgb[i + 1], rgb[i + 2]);
                    cols.insert(px);
                    if x > 0 {
                        let j = i - 3;
                        if px != (rgb[j], rgb[j + 1], rgb[j + 2]) { edges += 1; }
                    }
                }
            }
            let n = VRFRAME.fetch_add(1, Relaxed);
            println!("VRLOG {} {} {}", n, cols.len(), edges);
        }
        FRAME_W.store(w as usize, Relaxed);
        FRAME_H.store(h as usize, Relaxed);
        FRAME_RGB = rgb;
    }
    NONBLACK.store(nb, Relaxed);
}

fn crc32(data: &[u8]) -> u32 {
    let mut c: u32 = 0xFFFF_FFFF;
    for &b in data {
        c ^= b as u32;
        for _ in 0..8 {
            c = if c & 1 != 0 { 0xEDB8_8320 ^ (c >> 1) } else { c >> 1 };
        }
    }
    !c
}
fn adler32(data: &[u8]) -> u32 {
    let (mut a, mut b) = (1u32, 0u32);
    for &x in data {
        a = (a + x as u32) % 65521;
        b = (b + a) % 65521;
    }
    (b << 16) | a
}
fn png_chunk(out: &mut Vec<u8>, kind: &[u8; 4], data: &[u8]) {
    out.extend_from_slice(&(data.len() as u32).to_be_bytes());
    let mut cd = kind.to_vec();
    cd.extend_from_slice(data);
    out.extend_from_slice(&cd);
    out.extend_from_slice(&crc32(&cd).to_be_bytes());
}
fn write_png(path: &str, rgb: &[u8], w: usize, h: usize) {
    // Raw scanlines with filter byte 0, wrapped in one stored zlib stream.
    let mut raw = Vec::with_capacity(h * (1 + w * 3));
    for y in 0..h {
        raw.push(0);
        raw.extend_from_slice(&rgb[y * w * 3..(y + 1) * w * 3]);
    }
    let mut z = vec![0x78u8, 0x01];
    let mut i = 0;
    while i < raw.len() {
        let n = (raw.len() - i).min(65535);
        z.push(if i + n >= raw.len() { 1 } else { 0 });
        z.extend_from_slice(&(n as u16).to_le_bytes());
        z.extend_from_slice(&(!(n as u16)).to_le_bytes());
        z.extend_from_slice(&raw[i..i + n]);
        i += n;
    }
    z.extend_from_slice(&adler32(&raw).to_be_bytes());
    let mut png = vec![0x89, 0x50, 0x4E, 0x47, 0x0D, 0x0A, 0x1A, 0x0A];
    let mut ihdr = Vec::new();
    ihdr.extend_from_slice(&(w as u32).to_be_bytes());
    ihdr.extend_from_slice(&(h as u32).to_be_bytes());
    ihdr.extend_from_slice(&[8, 2, 0, 0, 0]); // 8-bit RGB
    png_chunk(&mut png, b"IHDR", &ihdr);
    png_chunk(&mut png, b"IDAT", &z);
    png_chunk(&mut png, b"IEND", &[]);
    std::fs::write(path, &png).expect("write png");
}
extern "C" fn as_cb(_l: i16, _r: i16) {}
extern "C" fn asb_cb(_d: *const i16, frames: usize) -> usize {
    frames
}
extern "C" fn ip_cb() {}
// Frame counter + a Start-press window, so the oracle can drive the same input
// our core got. RETRO_DEVICE_JOYPAD=1, RETRO_DEVICE_ID_JOYPAD_START=3.
static FRAME: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
static PRESS_FROM: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(9999);
static PRESS_LEN: std::sync::atomic::AtomicU32 = std::sync::atomic::AtomicU32::new(0);
// --autoinput replays gba-runner's GBA_AUTOINPUT exactly: A and Start held for
// frames 0..4 of every 24. Without the identical pattern the oracle walks a
// different path through the menus and the work-RAM diff is meaningless.
static AUTOINPUT: std::sync::atomic::AtomicBool = std::sync::atomic::AtomicBool::new(false);
extern "C" fn is_cb(_port: c_uint, dev: c_uint, _idx: c_uint, id: c_uint) -> i16 {
    use std::sync::atomic::Ordering::Relaxed;
    if AUTOINPUT.load(Relaxed) && dev == 1 && (id == 3 || id == 8) {
        // RETRO_DEVICE_ID_JOYPAD_START = 3, _A = 8.
        return if FRAME.load(Relaxed) % 24 < 4 { 1 } else { 0 };
    }
    if dev == 1 && id == 3 {
        let f = FRAME.load(Relaxed);
        let from = PRESS_FROM.load(Relaxed);
        if f >= from && f < from + PRESS_LEN.load(Relaxed) {
            return 1;
        }
    }
    0
}

unsafe fn sym(lib: *mut c_void, name: &[u8]) -> *mut c_void {
    let p = GetProcAddress(lib, name.as_ptr() as *const c_char);
    if p.is_null() {
        panic!("missing symbol: {}", std::str::from_utf8(&name[..name.len() - 1]).unwrap());
    }
    p
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 4 {
        eprintln!("usage: mesenread <core.dll> <rom> <state> <hexaddr>...");
        std::process::exit(2);
    }
    let dll = std::ffi::CString::new(args[1].clone()).unwrap();
    let rom = std::fs::read(&args[2]).expect("read rom");
    let state = if args[3] == "RESET" { Vec::new() } else { std::fs::read(&args[3]).expect("read state") };
    // --run N: after unserialize, step N frames and print the watched addrs each
    // frame, to trace the *healthy* trajectory the game takes on real silicon.
    let run_n: usize = args
        .iter()
        .position(|a| a == "--run")
        .and_then(|i| args.get(i + 1))
        .and_then(|s| s.parse().ok())
        .unwrap_or(0);
    let addrs: Vec<usize> = args[4..]
        .iter()
        .take_while(|a| *a != "--run")
        .filter_map(|h| usize::from_str_radix(h.trim_start_matches("0x"), 16).ok())
        .collect();

    unsafe {
        let lib = LoadLibraryA(dll.as_ptr());
        if lib.is_null() {
            panic!("LoadLibraryA failed for {}", args[1]);
        }
        let set_env: extern "C" fn(EnvFn) = std::mem::transmute(sym(lib, b"retro_set_environment\0"));
        let set_vr: extern "C" fn(extern "C" fn(*const c_void, c_uint, c_uint, usize)) =
            std::mem::transmute(sym(lib, b"retro_set_video_refresh\0"));
        let set_as: extern "C" fn(extern "C" fn(i16, i16)) =
            std::mem::transmute(sym(lib, b"retro_set_audio_sample\0"));
        let set_asb: extern "C" fn(extern "C" fn(*const i16, usize) -> usize) =
            std::mem::transmute(sym(lib, b"retro_set_audio_sample_batch\0"));
        let set_ip: extern "C" fn(extern "C" fn()) =
            std::mem::transmute(sym(lib, b"retro_set_input_poll\0"));
        let set_is: extern "C" fn(extern "C" fn(c_uint, c_uint, c_uint, c_uint) -> i16) =
            std::mem::transmute(sym(lib, b"retro_set_input_state\0"));
        let init: extern "C" fn() = std::mem::transmute(sym(lib, b"retro_init\0"));
        let load: extern "C" fn(*const GameInfo) -> bool =
            std::mem::transmute(sym(lib, b"retro_load_game\0"));
        let unser: extern "C" fn(*const c_void, usize) -> bool =
            std::mem::transmute(sym(lib, b"retro_unserialize\0"));
        let ser_size: extern "C" fn() -> usize =
            std::mem::transmute(sym(lib, b"retro_serialize_size\0"));
        let mem_data: extern "C" fn(c_uint) -> *mut c_void =
            std::mem::transmute(sym(lib, b"retro_get_memory_data\0"));
        let mem_size: extern "C" fn(c_uint) -> usize =
            std::mem::transmute(sym(lib, b"retro_get_memory_size\0"));
        let run: extern "C" fn() = std::mem::transmute(sym(lib, b"retro_run\0"));

        set_env(env_cb);
        set_vr(vr_cb);
        set_as(as_cb);
        set_asb(asb_cb);
        set_ip(ip_cb);
        set_is(is_cb);
        init();

        let path_c = std::ffi::CString::new(args[2].clone()).unwrap();
        let info = GameInfo {
            path: path_c.as_ptr(),
            data: rom.as_ptr() as *const c_void,
            size: rom.len(),
            meta: std::ptr::null(),
        };
        if !load(&info) {
            panic!("retro_load_game failed");
        }
        println!("[mesen] ROM loaded ({} bytes)", rom.len());
        println!("[mesen] retro_serialize_size = {}, state file = {}", ser_size(), state.len());

        // --findsig HEXBYTES: locate a byte pattern in the raw state blob (Mesen
        // stores VRAM contiguously), print the offset, and dump the region + the
        // +0x700-byte window (BG3 tilemap rows 28-31 relative to row 0).
        if let Some(sh) = args.iter().position(|a| a == "--findsig").and_then(|i| args.get(i + 1)) {
            let sig: Vec<u8> = sh.as_bytes().chunks(2).filter_map(|c| u8::from_str_radix(std::str::from_utf8(c).ok()?, 16).ok()).collect();
            let mut offs = vec![];
            let mut j = 0;
            while let Some(k) = state[j..].windows(sig.len()).position(|w| w == sig.as_slice()) {
                offs.push(j + k); j += k + 1;
            }
            println!("[find] {} occurrence(s): {:?}", offs.len(), offs.iter().map(|o| format!("{o:#X}")).collect::<Vec<_>>());
            for &o in offs.iter().take(2) {
                // Row 0 word $4C00 is at `o`; rows 28-31 word $4F80 is +0x700 bytes.
                for (label, delta) in [("row0 @sig", 0usize), ("rows28-31 @+0x700", 0x700)] {
                    println!("  {label} (state off {:#X}):", o + delta);
                    for r in 0..(if delta==0 {2} else {8}) {
                        let base = o + delta + r * 32;
                        let cells: Vec<String> = (0..16).map(|i| {
                            let b = base + i*2;
                            format!("{:04X}", state[b] as u16 | ((state[b+1] as u16) << 8))
                        }).collect();
                        println!("    {}", cells.join(" "));
                    }
                }
            }
            return;
        }

        // state path "RESET" runs from power-on instead of a captured state, to
        // measure a game's own output geometry (overscan) without needing a save.
        if args[3] == "RESET" {
            let n: usize = args.iter().position(|a| a == "--frames").and_then(|i| args.get(i + 1)).and_then(|s| s.parse().ok()).unwrap_or(400);
            if args.iter().any(|a| a == "--autoinput") {
                AUTOINPUT.store(true, std::sync::atomic::Ordering::Relaxed);
            }
            // Per-frame watch in RESET mode: print each watched SYSTEM_RAM word
            // whenever it changes, so a healthy core's trajectory around a scene
            // change can be compared event-for-event against ours.
            let sram2 = mem_data(2) as *const u8;
            let ss2 = mem_size(2);
            let mut prev: Vec<u32> = vec![0xDEAD_BEEF; addrs.len()];
            for i in 0..n {
                FRAME.store(i as u32, std::sync::atomic::Ordering::Relaxed);
                run();
                if !sram2.is_null() && ss2 > 4 && !addrs.is_empty() {
                    let w = std::slice::from_raw_parts(sram2, ss2);
                    for (k, &a) in addrs.iter().enumerate() {
                        let off = a & (ss2 - 1) & !3;
                        let v = (w[off] as u32) | ((w[off+1] as u32)<<8) | ((w[off+2] as u32)<<16) | ((w[off+3] as u32)<<24);
                        if v != prev[k] {
                            println!("[watch] f{i:<4} {a:#010X} = {v:08X}");
                            prev[k] = v;
                        }
                    }
                }
            }
            println!("[mesen] ran {n} frames from reset");
            // --serialize <path>: write the full core savestate after the run, so
            // regions libretro does not expose (GBA EWRAM) can be mined offline.
            if let Some(path) = args.iter().position(|a| a == "--serialize").and_then(|i| args.get(i + 1)) {
                let ser: extern "C" fn(*mut c_void, usize) -> bool =
                    std::mem::transmute(sym(lib, b"retro_serialize "));
                let sz = ser_size();
                let mut buf = vec![0u8; sz];
                if ser(buf.as_mut_ptr() as *mut c_void, sz) {
                    std::fs::write(path, &buf).expect("write state");
                    println!("[mesen] serialized {sz} bytes to {path}");
                } else {
                    println!("[mesen] retro_serialize FAILED");
                }
            }
        } else if !unser(state.as_ptr() as *const c_void, state.len()) {
            panic!("retro_unserialize FAILED (version/size mismatch?)");
        } else {
            println!("[mesen] state unserialized OK");
        }

        // Probe every memory id so we can find where Mesen exposes the 64KB VRAM.
        println!("[mesen] memory-id sizes:");
        for id in 0u32..8 {
            let sz = mem_size(id);
            let dp = mem_data(id);
            println!("  id {id}: size={sz} (0x{sz:X}) ptr={}", if dp.is_null() { "null" } else { "ok" });
        }

        let sram = mem_data(2); // RETRO_MEMORY_SYSTEM_RAM
        let ssize = mem_size(2);
        if sram.is_null() || ssize == 0 {
            panic!("no SYSTEM_RAM exposed");
        }
        let wram = std::slice::from_raw_parts(sram as *const u8, ssize);
        // --dumpram <path>: write the whole SYSTEM_RAM region to a file, so it can
        // be diffed byte-for-byte against our own core's dump (GBA_RAMDUMP in
        // gba-runner). Added for the GBA, where "works on another core and not on
        // ours" is almost always a handful of bytes of work RAM and guessing which
        // ones wastes a day.
        if let Some(path) = args.iter().position(|a| a == "--dumpram").and_then(|i| args.get(i + 1)) {
            std::fs::write(path, wram).expect("write ram dump");
            println!("[oracle] wrote {ssize} bytes of SYSTEM_RAM to {path}");
        }
        println!("[mesen] SYSTEM_RAM = {} bytes (0x{:X})\n", ssize, ssize);

        // RETRO_MEMORY_VIDEO_RAM = 3. Dump the BG3 tilemap ($4C00-$4FFF word) so
        // rows 25-31 can be compared against our garbage. --vram START:END (word
        // hex) overrides the region.
        let vptr = mem_data(3);
        let vsize = mem_size(3);
        if !vptr.is_null() && vsize > 0 {
            let vram = std::slice::from_raw_parts(vptr as *const u8, vsize);
            println!("[mesen] VIDEO_RAM = {} bytes (0x{:X})", vsize, vsize);
            let (mut ws, mut we) = (0x4C00usize, 0x5000usize);
            if let Some(spec) = args.iter().position(|a| a == "--vram").and_then(|i| args.get(i + 1)) {
                let p: Vec<&str> = spec.split(':').collect();
                if let [s, e] = p.as_slice() {
                    ws = usize::from_str_radix(s.trim_start_matches("0x"), 16).unwrap_or(ws);
                    we = usize::from_str_radix(e.trim_start_matches("0x"), 16).unwrap_or(we);
                }
            }
            println!("  BG3 tilemap words ${ws:04X}..${we:04X} (entry = tile|pal|prio):");
            for row_w in (ws..we).step_by(16) {
                let cells: Vec<String> = (0..16)
                    .map(|i| {
                        let w = row_w + i;
                        let e = vram[(w * 2) & (vsize - 1)] as u16
                            | ((vram[(w * 2 + 1) & (vsize - 1)] as u16) << 8);
                        format!("{:04X}", e)
                    })
                    .collect();
                println!("  {row_w:04X}: {}", cells.join(" "));
            }
        } else {
            println!("[mesen] VIDEO_RAM not exposed (mem id 3 empty)");
        }

        println!("Mesen WRAM at requested addresses:");
        for &a in &addrs {
            if a < ssize {
                println!("  ${:04X} = {:02X}", a, wram[a]);
            } else {
                println!("  ${:04X} = <out of range>", a);
            }
        }
        // --png PATH: run a couple frames off the state (to produce a video frame)
        // and save Mesen's actual render -- the ground-truth picture for the state.
        if let Some(p) = args.iter().position(|a| a == "--png").and_then(|i| args.get(i + 1)).cloned() {
            for _ in 0..2 { run(); }
            use std::sync::atomic::Ordering::Relaxed;
            let (w, h) = (FRAME_W.load(Relaxed), FRAME_H.load(Relaxed));
            let rgb = &*std::ptr::addr_of!(FRAME_RGB);
            if w > 0 && !rgb.is_empty() {
                write_png(&p, rgb, w, h);
                println!("[mesen] wrote frame {p} ({w}x{h})");
            } else {
                println!("[mesen] no frame captured");
            }
            return;
        }

        if run_n > 0 {
            use std::sync::atomic::Ordering::Relaxed;
            // --press FROM:LEN sets a Start-press window (default: frames 5..13).
            let (mut pf, mut pl) = (5u32, 8u32);
            if let Some(spec) = args.iter().position(|a| a == "--press").and_then(|i| args.get(i + 1)) {
                let p: Vec<&str> = spec.split(':').collect();
                if let [f, l] = p.as_slice() {
                    pf = f.parse().unwrap_or(5);
                    pl = l.parse().unwrap_or(8);
                }
            }
            PRESS_FROM.store(pf, Relaxed);
            PRESS_LEN.store(pl, Relaxed);
            println!("\n[mesen] stepping {run_n} frames, Start held frames {pf}..{}:", pf + pl);
            print!("frame ");
            for &a in &addrs {
                print!(" ${:04X}", a);
            }
            println!();
            for f in 0..run_n {
                FRAME.store(f as u32, Relaxed);
                run();
                let w = std::slice::from_raw_parts(mem_data(2) as *const u8, ssize);
                print!("{f:5} ");
                for &a in &addrs {
                    print!("  {:02X}", w[a]);
                }
                println!("   nonblack={}", NONBLACK.load(Relaxed));
            }
            return;
        }

        // Context dumps around the interesting pages.
        for &(base, len) in &[(0x0000usize, 0x100usize), (0x0700, 0x100), (0x0800, 0x100)] {
            println!("\n  --- WRAM ${:04X}..${:04X} ---", base, base + len);
            for row in (0..len).step_by(16) {
                let bytes: Vec<String> =
                    (0..16).map(|i| format!("{:02X}", wram[base + row + i])).collect();
                println!("  {:04X}  {}", base + row, bytes.join(" "));
            }
        }
    }
}
