// Measure the AUDIO a libretro core produces: sample rate, peak, RMS, and how
// much of the output is clipped. Purpose: answer "is our core quieter than
// gpSP?" with a number instead of an impression, by running the same ROM from
// reset with the same input through gpsp_libretro.dll, mgba_libretro.dll and
// (via gba-runner) our own core.
//
// Standalone dev tool, NOT part of the cargo workspace (Windows-only; loads an
// external libretro core). Build with: `rustc -O tools/gbaudio.rs -o gbaudio`.
// Env/callback handling copied from tools/gbaread.rs.
//
// usage: gbaudio <core.dll> <rom> [--frames N] [--skip N] [--wav <path>]
//                [--noinput]
//
//   --frames N   frames to run (default 1200, about 20 seconds)
//   --skip N     ignore the first N frames of audio, so a boot logo's silence
//                does not drag the RMS down (default 300)
//   --noinput    do not hold A+Start; by default they are held for frames 0..4
//                of every 24, matching gba-runner's GBA_AUTOINPUT, so both
//                cores walk the same path through the title screens
use std::ffi::c_void;
use std::os::raw::{c_char, c_uint};
use std::sync::atomic::Ordering::Relaxed;
use std::sync::atomic::{AtomicBool, AtomicU32};

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

#[repr(C)]
struct Geometry {
    base_width: c_uint,
    base_height: c_uint,
    max_width: c_uint,
    max_height: c_uint,
    aspect_ratio: f32,
}

#[repr(C)]
struct Timing {
    fps: f64,
    sample_rate: f64,
}

#[repr(C)]
struct AvInfo {
    geometry: Geometry,
    timing: Timing,
}

type EnvFn = extern "C" fn(c_uint, *mut c_void) -> bool;

static SYSDIR: &[u8] = b"C:\\Users\\User\\AppData\\Local\\Temp\\claude\0";

extern "C" fn env_cb(cmd: c_uint, data: *mut c_void) -> bool {
    match cmd & 0xffff {
        9 | 31 => unsafe {
            // GET_SYSTEM_DIRECTORY / GET_SAVE_DIRECTORY
            if data.is_null() {
                return false;
            }
            *(data as *mut *const c_char) = SYSDIR.as_ptr() as *const c_char;
            true
        },
        _ => false,
    }
}

extern "C" fn vr_cb(_d: *const c_void, _w: c_uint, _h: c_uint, _p: usize) {}

// Audio capture. Both entry points are used by real cores: gpSP batches, some
// cores emit one frame at a time, so both have to land in the same buffer or a
// core's output silently reads as zero.
static mut AUDIO: Vec<i16> = Vec::new();

extern "C" fn as_cb(l: i16, r: i16) {
    unsafe {
        let a = &mut *std::ptr::addr_of_mut!(AUDIO);
        a.push(l);
        a.push(r);
    }
}

extern "C" fn asb_cb(d: *const i16, frames: usize) -> usize {
    unsafe {
        let a = &mut *std::ptr::addr_of_mut!(AUDIO);
        if !d.is_null() {
            a.extend_from_slice(std::slice::from_raw_parts(d, frames * 2));
        }
    }
    frames
}

extern "C" fn ip_cb() {}

static FRAME: AtomicU32 = AtomicU32::new(0);
static AUTOINPUT: AtomicBool = AtomicBool::new(true);

extern "C" fn is_cb(_port: c_uint, dev: c_uint, _idx: c_uint, id: c_uint) -> i16 {
    // RETRO_DEVICE_ID_JOYPAD_START = 3, _A = 8.
    if AUTOINPUT.load(Relaxed) && dev == 1 && (id == 3 || id == 8) {
        return if FRAME.load(Relaxed) % 24 < 4 { 1 } else { 0 };
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

fn write_wav(path: &str, samples: &[i16], rate: u32) {
    let bytes = samples.len() * 2;
    let mut w: Vec<u8> = Vec::with_capacity(44 + bytes);
    w.extend_from_slice(b"RIFF");
    w.extend_from_slice(&((36 + bytes) as u32).to_le_bytes());
    w.extend_from_slice(b"WAVEfmt ");
    w.extend_from_slice(&16u32.to_le_bytes());
    w.extend_from_slice(&1u16.to_le_bytes()); // PCM
    w.extend_from_slice(&2u16.to_le_bytes()); // stereo
    w.extend_from_slice(&rate.to_le_bytes());
    w.extend_from_slice(&(rate * 4).to_le_bytes()); // byte rate
    w.extend_from_slice(&4u16.to_le_bytes()); // block align
    w.extend_from_slice(&16u16.to_le_bytes()); // bits
    w.extend_from_slice(b"data");
    w.extend_from_slice(&(bytes as u32).to_le_bytes());
    for s in samples {
        w.extend_from_slice(&s.to_le_bytes());
    }
    std::fs::write(path, &w).expect("write wav");
}

fn flag(args: &[String], name: &str) -> Option<String> {
    args.iter().position(|a| a == name).and_then(|i| args.get(i + 1)).cloned()
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    if args.len() < 3 {
        eprintln!("usage: gbaudio <core.dll> <rom> [--frames N] [--skip N] [--wav <path>] [--noinput]");
        std::process::exit(2);
    }
    let frames: u32 = flag(&args, "--frames").and_then(|s| s.parse().ok()).unwrap_or(1200);
    let skip: u32 = flag(&args, "--skip").and_then(|s| s.parse().ok()).unwrap_or(300);
    let wav = flag(&args, "--wav");
    if args.iter().any(|a| a == "--noinput") {
        AUTOINPUT.store(false, Relaxed);
    }

    let dll = std::ffi::CString::new(args[1].clone()).unwrap();
    let rom = std::fs::read(&args[2]).expect("read rom");

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
        let av: extern "C" fn(*mut AvInfo) = std::mem::transmute(sym(lib, b"retro_get_system_av_info\0"));
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

        let mut avi: AvInfo = std::mem::zeroed();
        av(&mut avi);
        let rate = avi.timing.sample_rate;
        println!("core {} rate {rate:.1} Hz fps {:.3}", args[1], avi.timing.fps);

        // Run the skipped prefix first and throw its audio away, so the window
        // measured is the same span of emulated time for every core even though
        // each emits a different number of samples per frame.
        for _ in 0..skip {
            run();
            FRAME.fetch_add(1, Relaxed);
        }
        let a = &mut *std::ptr::addr_of_mut!(AUDIO);
        a.clear();
        for _ in skip..frames {
            run();
            FRAME.fetch_add(1, Relaxed);
        }

        let audio: &[i16] = &*std::ptr::addr_of!(AUDIO);
        report(audio, rate, frames - skip);
        if let Some(p) = wav {
            write_wav(&p, audio, rate.round() as u32);
            eprintln!("wrote {p} ({} stereo samples)", audio.len() / 2);
        }
    }
}

fn report(audio: &[i16], rate: f64, frames: u32) {
    if audio.is_empty() {
        println!("  NO AUDIO: the core emitted nothing through either callback");
        return;
    }
    let peak = audio.iter().map(|s| s.unsigned_abs()).max().unwrap_or(0);
    let sum: f64 = audio.iter().map(|&s| (s as f64) * (s as f64)).sum();
    let rms = (sum / audio.len() as f64).sqrt();
    // A sample sitting on a rail is the signature of a core that is mixing
    // louder than i16 can hold. Count it, because "louder" and "clipped" are
    // different verdicts and both show up as a high peak.
    let railed = audio.iter().filter(|&&s| s == 32767 || s == -32768).count();
    let db = |v: f64| if v <= 0.0 { f64::NEG_INFINITY } else { 20.0 * (v / 32768.0).log10() };
    println!(
        "  {} stereo samples over {frames} frames ({:.1}/frame, expected {:.1})",
        audio.len() / 2,
        audio.len() as f64 / 2.0 / frames as f64,
        rate / 59.7275
    );
    println!(
        "  peak {peak} ({:.1} dBFS)   rms {rms:.1} ({:.1} dBFS)   railed {railed} ({:.4}%)",
        db(peak as f64),
        db(rms),
        railed as f64 * 100.0 / audio.len() as f64
    );
}
