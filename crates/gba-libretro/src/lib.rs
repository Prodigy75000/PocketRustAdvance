// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Prodigy75000

//! libretro front-end ABI for gba-core.
//!
//! Exposes the standard `retro_*` C entry points so the GBA core can be loaded
//! by any libretro host (RetroArch, Trophy Hub's libretro host, ...). The core
//! runs single-threaded, so all state lives in a thread-local `State`.
//!
//! The core boots through a BIOS: the bundled open-source BIOS by default (no
//! install needed, matches gpSP), or a real `gba_bios.bin` from the frontend's
//! system dir when present (also serves titles that read the BIOS ROM directly).
//! It produces an RGB555 framebuffer, expanded to XRGB8888 for the host. Audio
//! and save-states are wired.

#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use gba_core::save::SaveKind;
use gba_core::sensor::CartSensor;
use gba_core::{Button, Gba, SCREEN_H, SCREEN_W};
mod netpacket;

use std::cell::UnsafeCell;
use std::ffi::{c_char, c_uint, c_void};
use std::ptr;

// --- libretro C types we need -------------------------------------------------

type retro_environment_t = Option<unsafe extern "C" fn(u32, *mut c_void) -> bool>;
type retro_video_refresh_t = Option<unsafe extern "C" fn(*const c_void, u32, u32, usize)>;
type retro_audio_sample_batch_t = Option<unsafe extern "C" fn(*const i16, usize) -> usize>;
type retro_input_poll_t = Option<unsafe extern "C" fn()>;
type retro_input_state_t = Option<unsafe extern "C" fn(u32, u32, u32, u32) -> i16>;

#[repr(C)]
struct retro_system_info {
    library_name: *const c_char,
    library_version: *const c_char,
    valid_extensions: *const c_char,
    need_fullpath: bool,
    block_extract: bool,
}

#[repr(C)]
struct retro_game_geometry {
    base_width: u32,
    base_height: u32,
    max_width: u32,
    max_height: u32,
    aspect_ratio: f32,
}

#[repr(C)]
struct retro_system_timing {
    fps: f64,
    sample_rate: f64,
}

#[repr(C)]
struct retro_system_av_info {
    geometry: retro_game_geometry,
    timing: retro_system_timing,
}

#[repr(C)]
struct retro_game_info {
    path: *const c_char,
    data: *const c_void,
    size: usize,
    meta: *const c_char,
}

// Environment command + pixel-format constants we use.
const RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY: u32 = 9;
const RETRO_ENVIRONMENT_SET_PIXEL_FORMAT: u32 = 10;
/// SET_MEMORY_MAPS is `36 | EXPERIMENTAL`, not a bare 36.
///
/// It was a bare 36 here, so the host never recognised the call and the map was
/// never published. Nothing failed: the front-end fell back to laying the
/// console's regions over whatever retro_get_memory_data returned for
/// SYSTEM_RAM, which for a GBA cannot be right because IWRAM and EWRAM are not
/// contiguous. RetroAchievements then evaluated against real memory at wrong
/// offsets and awarded the owner three achievements he had not earned.
///
/// Measured on device: rc_libretro laid out 360448 bytes of regions over a
/// 262144-byte buffer, so every region after the first pointed somewhere wrong.
const RETRO_ENVIRONMENT_SET_MEMORY_MAPS: u32 = 36 | 0x1_0000;
/// GET_SENSOR_INTERFACE is 25 | EXPERIMENTAL. The bare 25 is a different
/// command entirely, so it must not be used as a fallback: writing a
/// retro_sensor_interface into the wrong command's payload corrupts it
/// silently.
const RETRO_ENVIRONMENT_GET_SENSOR_INTERFACE: u32 = 25 | 0x1_0000;

const RETRO_SENSOR_ACCELEROMETER_ENABLE: u32 = 0;
const RETRO_SENSOR_GYROSCOPE_ENABLE: u32 = 2;
const RETRO_SENSOR_ACCELEROMETER_X: u32 = 0;
const RETRO_SENSOR_ACCELEROMETER_Y: u32 = 1;
const RETRO_SENSOR_ACCELEROMETER_Z: u32 = 2;
const RETRO_SENSOR_GYROSCOPE_X: u32 = 3;
const RETRO_SENSOR_GYROSCOPE_Y: u32 = 4;
const RETRO_SENSOR_GYROSCOPE_Z: u32 = 5;

/// GET_RUMBLE_INTERFACE. Plain command number, no experimental bit: rumble has
/// been in the stable libretro API for years, unlike the sensor interface above
/// which carries 0x10000 and is easy to copy the shape of by mistake.
const RETRO_ENVIRONMENT_GET_RUMBLE_INTERFACE: u32 = 23;
/// The front-end's logger. Plain 27, no experimental bit.
const RETRO_ENVIRONMENT_GET_LOG_INTERFACE: u32 = 27;
/// Is the front end running the core faster than real time? Plain, no
/// experimental bit. A front end that does not implement it leaves our bool
/// untouched, which is why it is initialised to false before every call.
const RETRO_ENVIRONMENT_GET_FASTFORWARDING: u32 = 65;

/// Shortest gap between two DRAWN frames. Must stay BELOW a 60 Hz frame
/// (16_667 us) or ordinary front-end pacing jitter drops a real frame; a test
/// pins that relationship, because a timing test cannot catch it reliably.
const FRAME_DRAW_MIN_US: u128 = 14_000;
const RETRO_LOG_INFO: u32 = 1;

/// libretro's logger is printf-shaped, so this is a variadic pointer and every
/// call goes through a literal "%s" with one argument. Passing a message as
/// the format string itself would let a game-supplied percent sign read
/// arbitrary varargs.
type RetroLogPrintf = unsafe extern "C" fn(level: u32, fmt: *const c_char, ...);

#[repr(C)]
#[derive(Clone, Copy)]
struct retro_log_callback {
    log: Option<RetroLogPrintf>,
}

/// Write one line to the front-end's log, if it gave us one.
///
/// This exists because the wireless adapter failures are all SILENT: a session
/// bound to the wrong core, two devices minting the same id, a connect that is
/// never sent. None of them error, and on a phone there is no GBA_RFULOG to
/// fall back on, so without this the only evidence is the absence of packets.
fn log_line(logger: Option<RetroLogPrintf>, msg: &str) {
    let Some(log) = logger else { return };
    let Ok(c) = std::ffi::CString::new(msg) else { return };
    // SAFETY: the front-end owns the callback for the core's lifetime, and the
    // format string is a literal with exactly one matching argument.
    unsafe { log(RETRO_LOG_INFO, c"%s".as_ptr(), c.as_ptr()) };
}

const RETRO_RUMBLE_STRONG: u32 = 0;
const RETRO_RUMBLE_WEAK: u32 = 1;

type retro_set_rumble_state_t =
    Option<unsafe extern "C" fn(port: c_uint, effect: c_uint, strength: u16) -> bool>;

#[repr(C)]
#[derive(Clone, Copy)]
struct retro_rumble_interface {
    set_rumble_state: retro_set_rumble_state_t,
}

type retro_set_sensor_state_t =
    Option<unsafe extern "C" fn(port: c_uint, action: c_uint, rate: c_uint) -> bool>;
type retro_sensor_get_input_t = Option<unsafe extern "C" fn(port: c_uint, id: c_uint) -> f32>;

#[repr(C)]
#[derive(Clone, Copy)]
struct retro_sensor_interface {
    set_sensor_state: retro_set_sensor_state_t,
    get_sensor_input: retro_sensor_get_input_t,
}
const RETRO_PIXEL_FORMAT_XRGB8888: i32 = 1;

const RETRO_MEMDESC_SYSTEM_RAM: u64 = 1 << 2;
const RETRO_MEMDESC_SAVE_RAM: u64 = 1 << 3;

/// One entry of the address-space table published through SET_MEMORY_MAPS.
/// `select` = 0 means the front-end matches an address explicitly against
/// `[start, start + len)`, which is what we want: the GBA regions are plain
/// disjoint blocks with no mirroring to describe.
#[repr(C)]
struct retro_memory_descriptor {
    flags: u64,
    ptr: *mut c_void,
    offset: usize,
    start: usize,
    select: usize,
    disconnect: usize,
    len: usize,
    addrspace: *const c_char,
}

#[repr(C)]
struct retro_memory_map {
    descriptors: *const retro_memory_descriptor,
    num_descriptors: c_uint,
}

/// Bundled open-source GBA BIOS (Normmatt's clean-room reimplementation - the
/// same freely-redistributable image gpSP ships). We boot through it by default
/// so no BIOS install is needed and behaviour matches gpSP. It cannot serve
/// titles that read the *real* BIOS ROM's bytes directly (a reimplementation has
/// different bytes at those offsets); for those, a real `gba_bios.bin` in the
/// system directory overrides this.
static OPEN_BIOS: &[u8] = include_bytes!("../open_gba_bios.bin");

/// Reported as the libretro library version, and the only self-description the
/// shipped .so carries. Concatenated at compile time so it survives stripping:
/// it is referenced by an exported function, so the linker keeps it.
const LIBRARY_VERSION: &std::ffi::CStr = match std::ffi::CStr::from_bytes_with_nul(
    concat!("0.1.0 build=", env!("PRA_BUILD"), "\0").as_bytes(),
) {
    Ok(v) => v,
    Err(_) => c"0.1.0 build=unknown",
};

const RETRO_MEMORY_SAVE_RAM: u32 = 0;
const RETRO_MEMORY_SYSTEM_RAM: u32 = 2;

// Device + button ids.
const RETRO_DEVICE_JOYPAD: u32 = 1;
const RETRO_DEVICE_ID_JOYPAD_B: u32 = 0;
const RETRO_DEVICE_ID_JOYPAD_SELECT: u32 = 2;
const RETRO_DEVICE_ID_JOYPAD_START: u32 = 3;
const RETRO_DEVICE_ID_JOYPAD_UP: u32 = 4;
const RETRO_DEVICE_ID_JOYPAD_DOWN: u32 = 5;
const RETRO_DEVICE_ID_JOYPAD_LEFT: u32 = 6;
const RETRO_DEVICE_ID_JOYPAD_RIGHT: u32 = 7;
const RETRO_DEVICE_ID_JOYPAD_A: u32 = 8;
const RETRO_DEVICE_ID_JOYPAD_L: u32 = 10;
const RETRO_DEVICE_ID_JOYPAD_R: u32 = 11;

// --- Core state ---------------------------------------------------------------

struct State {
    gba: Option<Gba>,
    rom: Vec<u8>,    // kept so retro_reset can rebuild the machine
    bios: Vec<u8>,   // resolved once at load, reused by retro_reset
    frame: Vec<u32>, // XRGB8888, SCREEN_W*SCREEN_H
    env: retro_environment_t,
    video: retro_video_refresh_t,
    audio_batch: retro_audio_sample_batch_t,
    input_poll: retro_input_poll_t,
    input_state: retro_input_state_t,
    /// The front-end's sensor interface, present only when this cartridge has
    /// a sensor AND the front-end offered one.
    sensors: Option<retro_sensor_interface>,
    rumble: Option<retro_rumble_interface>,
    /// Last level handed to the host, so an unchanged level is not re-sent 60
    /// times a second.
    rumble_last: bool,
    /// Whether the front-end accepted the memory map. See
    /// `retro_get_memory_data`: when it did not, we must not answer SYSTEM_RAM.
    map_published: bool,
    /// The front-end's logger, when it offered one.
    log: Option<RetroLogPrintf>,
    /// Last reported adapter session counters, so a line is logged only when
    /// something actually changes rather than sixty times a second.
    rfu_last: Option<(u64, u64, u64, u64, &'static str, u64, u64)>,
    flash_last: Option<(u64, u64, u64, u64, u32, u8)>,
    /// When the last frame was actually drawn. In fast forward the core runs
    /// many frames per displayed one, so compositing every frame spends most of
    /// its time on pictures nobody sees.
    last_drawn: Option<std::time::Instant>,
    /// Frames run, for a heartbeat line. Two emulators on two phones do NOT
    /// share a clock the way two GBAs do, so if one runs below full speed it
    /// under-drains its receive queue while the peer keeps sending at its own
    /// rate. That shows up as a queue pinned at its ceiling, and it is cured by
    /// speed rather than by a bigger buffer, so the frame rate has to be
    /// measurable on the device itself.
    frames: u64,
}

impl State {
    const fn new() -> State {
        State {
            gba: None,
            rom: Vec::new(),
            bios: Vec::new(),
            frame: Vec::new(),
            env: None,
            video: None,
            audio_batch: None,
            input_poll: None,
            input_state: None,
            sensors: None,
            rumble: None,
            rumble_last: false,
            map_published: false,
            log: None,
            rfu_last: None,
            flash_last: None,
            last_drawn: None,
            frames: 0,
        }
    }
}

/// Trophy Hub's libretro host registers callbacks (env, video, audio, input) on
/// its main thread but runs frames on a dedicated emulation thread. So the state
/// must be a single process-global, not thread-local - otherwise `retro_run`
/// sees a fresh empty state with null callbacks (black screen, no audio). This
/// mirrors how C libretro cores keep their state in plain `static`s.
struct GlobalState(UnsafeCell<State>);

// SAFETY: libretro serializes every call into the core - `retro_run`, the
// `retro_set_*` registrations and load/unload never overlap - so there is never
// concurrent access to the single STATE instance.
unsafe impl Sync for GlobalState {}

static STATE: GlobalState = GlobalState(UnsafeCell::new(State::new()));

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    // SAFETY: see `GlobalState` - accesses are serialized by the frontend.
    unsafe { f(&mut *STATE.0.get()) }
}

/// Expand a GBA RGB555 pixel (0bbbbbggggggrrrrr) into host XRGB8888.
#[inline]
fn rgb555_to_xrgb8888(p: u16) -> u32 {
    let r5 = (p & 0x1F) as u32;
    let g5 = ((p >> 5) & 0x1F) as u32;
    let b5 = ((p >> 10) & 0x1F) as u32;
    let r = (r5 << 3) | (r5 >> 2);
    let g = (g5 << 3) | (g5 >> 2);
    let b = (b5 << 3) | (b5 >> 2);
    (r << 16) | (g << 8) | b
}

// --- Required libretro entry points ------------------------------------------

#[no_mangle]
pub extern "C" fn retro_api_version() -> u32 {
    1
}

#[no_mangle]
pub extern "C" fn retro_init() {
    with_state(|s| s.frame = vec![0u32; SCREEN_W * SCREEN_H]);
}

#[no_mangle]
pub extern "C" fn retro_deinit() {
    with_state(|s| *s = State::new());
}

#[no_mangle]
pub unsafe extern "C" fn retro_get_system_info(info: *mut retro_system_info) {
    if info.is_null() {
        return;
    }
    (*info).library_name = c"PocketRustAdvance".as_ptr();
    // Carries the commit (see build.rs) so an artifact can be identified
    // without comparing ELF section sizes:
    //   strings -a libgbacore_libretro.so | grep build=
    // It also lands in the front-end's core-info line. "-dirty" means the hash
    // does not name these bytes; "local" means there was no git to ask.
    (*info).library_version = LIBRARY_VERSION.as_ptr();
    (*info).valid_extensions = c"gba|agb|bin".as_ptr();
    (*info).need_fullpath = false;
    (*info).block_extract = false;
}

#[no_mangle]
pub unsafe extern "C" fn retro_get_system_av_info(info: *mut retro_system_av_info) {
    if info.is_null() {
        return;
    }
    (*info).geometry = retro_game_geometry {
        base_width: SCREEN_W as u32,
        base_height: SCREEN_H as u32,
        max_width: SCREEN_W as u32,
        max_height: SCREEN_H as u32,
        aspect_ratio: SCREEN_W as f32 / SCREEN_H as f32,
    };
    (*info).timing = retro_system_timing {
        fps: 16_777_216.0 / 280_896.0, // ~59.7275 Hz
        // Must come from the core, never a literal: this used to be a hardcoded
        // 32768 and the APU's rate is a tuning decision that lives there. A
        // mismatch is not a small bug, the host would pace and pitch every game
        // wrong while both numbers looked individually reasonable.
        sample_rate: gba_core::Gba::SAMPLE_RATE as f64,
    };
}

#[no_mangle]
pub extern "C" fn retro_set_environment(cb: retro_environment_t) {
    with_state(|s| s.env = cb);
    // Offer the wireless adapter over the frontend netplay here, NOT from
    // retro_load_game: the frontend brings a session up around the load, and
    // an interface registered after that is never started. The host's own
    // transport notes record melonDS-DS hitting exactly that and silently
    // falling back to in-process loopback. The frontend keeps the struct, so
    // it has to be a &'static.
    if let Some(env) = cb {
        unsafe {
            env(
                netpacket::RETRO_ENVIRONMENT_SET_NETPACKET_INTERFACE,
                &netpacket::CALLBACK as *const _ as *mut c_void,
            );
            let mut lg = retro_log_callback { log: None };
            if env(
                RETRO_ENVIRONMENT_GET_LOG_INTERFACE,
                &mut lg as *mut retro_log_callback as *mut c_void,
            ) {
                with_state(|s| s.log = lg.log);
            }
        }
    }
}

#[no_mangle]
pub extern "C" fn retro_set_video_refresh(cb: retro_video_refresh_t) {
    with_state(|s| s.video = cb);
}
#[no_mangle]
pub extern "C" fn retro_set_audio_sample(_cb: *const c_void) {}
#[no_mangle]
pub extern "C" fn retro_set_audio_sample_batch(cb: retro_audio_sample_batch_t) {
    with_state(|s| s.audio_batch = cb);
}
#[no_mangle]
pub extern "C" fn retro_set_input_poll(cb: retro_input_poll_t) {
    with_state(|s| s.input_poll = cb);
}
#[no_mangle]
pub extern "C" fn retro_set_input_state(cb: retro_input_state_t) {
    with_state(|s| s.input_state = cb);
}
#[no_mangle]
pub extern "C" fn retro_set_controller_port_device(_port: u32, _device: u32) {}

/// Resolve the BIOS to boot through: a real `gba_bios.bin` from the frontend's
/// system directory if the host provides one (this additionally fixes titles
/// that read live data straight out of the real BIOS ROM), otherwise the bundled
/// open BIOS. No BIOS install is required for the common case.
unsafe fn resolve_bios(env: retro_environment_t) -> Vec<u8> {
    if let Some(env) = env {
        let mut dir: *const c_char = ptr::null();
        let ok = env(RETRO_ENVIRONMENT_GET_SYSTEM_DIRECTORY, &mut dir as *mut *const c_char as *mut c_void);
        if ok && !dir.is_null() {
            if let Ok(dir) = std::ffi::CStr::from_ptr(dir).to_str() {
                if let Ok(data) = std::fs::read(format!("{dir}/gba_bios.bin")) {
                    if data.len() >= 0x4000 {
                        return data; // real BIOS present → prefer it
                    }
                }
            }
        }
    }
    OPEN_BIOS.to_vec() // bundled open BIOS (default; no install needed)
}

#[no_mangle]
pub extern "C" fn retro_reset() {
    with_state(|s| {
        if !s.rom.is_empty() {
            // **A reset is a power cycle, not a new cartridge.** Rebuilding the
            // machine runs save detection again, which allocates a BLANK save
            // (all 0xFF, the erased state of flash), and the front-end then
            // flushes that over the player's .sav. The save has to be carried
            // across by hand.
            //
            // This destroyed two real saves before it was found, and it looked
            // like a wireless-adapter bug for hours because the owner reset
            // after each failed Union Room session: the reset was doing the
            // damage, not the trade. It is also invisible in the moment, since
            // the game keeps running on the blank save and only the next flush
            // writes the file.
            //
            // A blank save and a chip-erased one are byte-identical, so the
            // file cannot tell you which happened. Guarded on the length
            // matching so a change in save detection can never paste a buffer
            // of the wrong size into the new machine.
            let save = s.gba.as_ref().map(|g| g.bus.save.data.clone());
            s.gba = Some(Gba::new(s.rom.clone(), s.bios.clone()));
            if let (Some(gba), Some(save)) = (s.gba.as_mut(), save) {
                if gba.bus.save.data.len() == save.len() {
                    gba.bus.save.data = save;
                }
            }
            machine_rebuilt(s);
        }
    });
}

/// Everything that must be re-done whenever the machine is rebuilt.
///
/// The memory map hands the front-end RAW POINTERS into this Gba's RAM, so
/// building a new one frees the buffers the front-end is still holding. Any path
/// that replaces the machine without coming back through here leaves the
/// front-end reading freed heap.
///
/// That is not hypothetical. `retro_reset` used to rebuild the machine and not
/// re-publish, and RetroAchievements read the freed pages: the owner booted
/// Yoshi Topsy-Turvy and was awarded three achievements he had not earned,
/// including one for finishing a world, from walking through menus. rc_client
/// resets the core itself when hardcore is toggled, so RA could trigger the very
/// corruption it then read.
///
/// False unlocks are written to a player's account server-side and we cannot
/// take them back, which is why this is one function called from both paths
/// rather than two call sites that have to be remembered separately.
fn machine_rebuilt(s: &mut State) {
    publish_memory_map(s);
    enable_cart_sensors(s);
    enable_cart_rumble(s);
}

#[no_mangle]
pub unsafe extern "C" fn retro_load_game(info: *const retro_game_info) -> bool {
    if info.is_null() || (*info).data.is_null() {
        return false;
    }
    let rom = std::slice::from_raw_parts((*info).data as *const u8, (*info).size).to_vec();

    with_state(|s| {
        // Ask the host for 32-bit XRGB8888 video.
        if let Some(env) = s.env {
            let mut fmt = RETRO_PIXEL_FORMAT_XRGB8888;
            env(
                RETRO_ENVIRONMENT_SET_PIXEL_FORMAT,
                &mut fmt as *mut i32 as *mut c_void,
            );
        }
        s.rom = rom.clone();
        // Boot through a BIOS (real hardware behaviour): a system-dir gba_bios.bin
        // if the host has one, else the bundled open BIOS. Fixes titles that read
        // the BIOS ROM / depend on BIOS-initialised state.
        s.bios = resolve_bios(s.env);
        s.gba = Some(Gba::new(rom, s.bios.clone()));
        machine_rebuilt(s);
    });
    true
}

/// Ask the front-end for its sensor interface, and switch on only the sensor
/// this cartridge actually has.
///
/// WarioWare Twisted carries a Z-axis gyro; Yoshi Topsy-Turvy, Yoshi's
/// Universal Gravitation and Koro Koro Puzzle carry a two-axis accelerometer.
/// Which one is decided by the game code, so an ordinary cartridge never turns
/// a sensor on and never costs the device the power of running one.
fn enable_cart_rumble(s: &mut State) {
    s.rumble = None;
    s.rumble_last = false;
    let Some(env) = s.env else { return };
    let Some(gba) = s.gba.as_ref() else { return };
    if !gba.cart_has_rumble() {
        return;
    }
    let mut iface = retro_rumble_interface { set_rumble_state: None };
    let ok = unsafe {
        env(
            RETRO_ENVIRONMENT_GET_RUMBLE_INTERFACE,
            &mut iface as *mut retro_rumble_interface as *mut c_void,
        )
    };
    // As with the sensors, a front-end may answer true and leave the struct
    // empty, so the pointer is what decides.
    if ok && iface.set_rumble_state.is_some() {
        s.rumble = Some(iface);
    }
}

fn enable_cart_sensors(s: &mut State) {
    s.sensors = None;
    let Some(env) = s.env else { return };
    let Some(gba) = s.gba.as_ref() else { return };
    let action = match gba.cart_sensor() {
        CartSensor::None => return,
        CartSensor::Gyro => RETRO_SENSOR_GYROSCOPE_ENABLE,
        CartSensor::Tilt => RETRO_SENSOR_ACCELEROMETER_ENABLE,
    };
    let mut iface = retro_sensor_interface {
        set_sensor_state: None,
        get_sensor_input: None,
    };
    let ok = unsafe {
        env(
            RETRO_ENVIRONMENT_GET_SENSOR_INTERFACE,
            &mut iface as *mut retro_sensor_interface as *mut c_void,
        )
    };
    // A front-end may answer true and still leave the struct empty, so the
    // pointers are checked rather than the return value.
    if !ok || iface.get_sensor_input.is_none() {
        return;
    }
    if let Some(set_state) = iface.set_sensor_state {
        // 60 Hz: the game samples once a frame, so asking for more only costs
        // battery.
        unsafe { set_state(0, action, 60) };
    }
    s.sensors = Some(iface);
}

/// Publish the address space so the front-end can read emulated memory by GBA
/// address. RetroAchievements needs this: without it rc_libretro_memory_init
/// fails, and the Trophy Hub host parks on "waiting for core memory map"
/// forever rather than evaluating a single achievement. Reported from a device
/// on 2026-09-22 against Knights' Kingdom, where gpSP resolved the title fine
/// and we hung.
///
/// rcheevos maps the GBA in an order that is not the obvious one: IWRAM is at
/// RA address 0 and EWRAM follows it, so the descriptors are matched by their
/// real addresses rather than by the order they appear here.
///
/// ```text
/// RA 0x000000  32 KB   real 0x03000000   IWRAM
/// RA 0x008000  256 KB  real 0x02000000   EWRAM
/// RA 0x048000  64 KB   real 0x0E000000   cartridge SRAM
/// ```
///
/// The host deep-copies the table and the addrspace strings, so these may be
/// locals; only `ptr` has to outlive the call, and it points into the Gba that
/// was just constructed. Called on every load, because a new Gba means new
/// buffers and the previous pointers are dangling.
fn publish_memory_map(s: &mut State) {
    let Some(env) = s.env else { return };
    let Some(gba) = s.gba.as_mut() else { return };

    let mut descs = vec![
        retro_memory_descriptor {
            flags: RETRO_MEMDESC_SYSTEM_RAM,
            ptr: gba.bus.iwram.as_mut_ptr() as *mut c_void,
            offset: 0,
            start: 0x0300_0000,
            select: 0,
            disconnect: 0,
            len: gba.bus.iwram.len(),
            addrspace: c"IWRAM".as_ptr(),
        },
        retro_memory_descriptor {
            flags: RETRO_MEMDESC_SYSTEM_RAM,
            ptr: gba.bus.ewram.as_mut_ptr() as *mut c_void,
            offset: 0,
            start: 0x0200_0000,
            select: 0,
            disconnect: 0,
            len: gba.bus.ewram.len(),
            addrspace: c"EWRAM".as_ptr(),
        },
    ];

    // Only SRAM and Flash live at 0x0E000000. EEPROM is not memory mapped at
    // all, it is clocked in a bit at a time through region 0xD, so publishing
    // it at the SRAM address would hand the achievement runtime bytes that are
    // real but mean something else.
    //
    // This descriptor was removed on 2026-10-01 and put straight back. The
    // theory was that an unwritten save reads 0xFF and false-unlocks a "collect
    // them all" achievement, which Advance Wars 2 was doing within 110 ms of
    // load. The owner falsified it on hardware: mGBA publishes this same region
    // and fills fresh flash with 0xFF exactly as we do, and does NOT unlock it.
    // Withholding the descriptor here did not stop the unlock either, which
    // proves the condition reads WORK RAM. RetroAchievements documents this
    // region for the GBA, so sets are entitled to read it and hiding it would
    // only break the honest ones silently.
    if matches!(
        gba.bus.save.kind,
        SaveKind::Sram | SaveKind::Flash64 | SaveKind::Flash128
    ) && !gba.bus.save.data.is_empty()
    {
        descs.push(retro_memory_descriptor {
            flags: RETRO_MEMDESC_SAVE_RAM,
            ptr: gba.bus.save.data.as_mut_ptr() as *mut c_void,
            offset: 0,
            start: 0x0E00_0000,
            select: 0,
            disconnect: 0,
            len: gba.bus.save.data.len(),
            addrspace: c"SRAM".as_ptr(),
        });
    }

    let map = retro_memory_map {
        descriptors: descs.as_ptr(),
        num_descriptors: descs.len() as c_uint,
    };
    s.map_published = unsafe {
        env(
            RETRO_ENVIRONMENT_SET_MEMORY_MAPS,
            &map as *const retro_memory_map as *mut c_void,
        )
    };
}

#[no_mangle]
pub extern "C" fn retro_unload_game() {
    // This frees the RAM the published memory map points at. The front-end is
    // expected to stop reading it once the game is unloaded (Trophy Hub's host
    // gates getMemoryMap on game_loaded), and there is no way to retract a map,
    // so there is nothing to publish here. Noted because the lifetime is the
    // dangerous part of this interface. See machine_rebuilt.
    with_state(|s| s.gba = None);
}

#[no_mangle]
pub extern "C" fn retro_run() {
    with_state(|s| {
        // Poll input and translate the joypad.
        if let Some(poll) = s.input_poll {
            unsafe { poll() };
        }
        if let (Some(gba), Some(input)) = (&mut s.gba, s.input_state) {
            let pressed = |id: u32| unsafe { input(0, RETRO_DEVICE_JOYPAD, 0, id) != 0 };
            gba.set_button(Button::A, pressed(RETRO_DEVICE_ID_JOYPAD_A));
            gba.set_button(Button::B, pressed(RETRO_DEVICE_ID_JOYPAD_B));
            gba.set_button(Button::Start, pressed(RETRO_DEVICE_ID_JOYPAD_START));
            gba.set_button(Button::Select, pressed(RETRO_DEVICE_ID_JOYPAD_SELECT));
            gba.set_button(Button::Up, pressed(RETRO_DEVICE_ID_JOYPAD_UP));
            gba.set_button(Button::Down, pressed(RETRO_DEVICE_ID_JOYPAD_DOWN));
            gba.set_button(Button::Left, pressed(RETRO_DEVICE_ID_JOYPAD_LEFT));
            gba.set_button(Button::Right, pressed(RETRO_DEVICE_ID_JOYPAD_RIGHT));
            gba.set_button(Button::L, pressed(RETRO_DEVICE_ID_JOYPAD_L));
            gba.set_button(Button::R, pressed(RETRO_DEVICE_ID_JOYPAD_R));
        }

        // Feed the cartridge's sensor from the device, once per frame, just
        // before the game gets a chance to sample it.
        if let (Some(gba), Some(iface)) = (&mut s.gba, s.sensors) {
            if let Some(get) = iface.get_sensor_input {
                unsafe {
                    match gba.cart_sensor() {
                        CartSensor::Gyro => gba.set_gyroscope(
                            get(0, RETRO_SENSOR_GYROSCOPE_X),
                            get(0, RETRO_SENSOR_GYROSCOPE_Y),
                            get(0, RETRO_SENSOR_GYROSCOPE_Z),
                        ),
                        CartSensor::Tilt => gba.set_accelerometer(
                            get(0, RETRO_SENSOR_ACCELEROMETER_X),
                            get(0, RETRO_SENSOR_ACCELEROMETER_Y),
                            get(0, RETRO_SENSOR_ACCELEROMETER_Z),
                        ),
                        CartSensor::None => {}
                    }
                }
            }
        }

        // The wireless adapter, if a session is live. Inbound first so a
        // packet that arrived during the last frame is visible to this one,
        // then outbound so anything the game just produced leaves immediately
        // rather than waiting a frame.
        //
        // `drain_inbound` calls the frontend's poll, which re-enters us
        // through the receive callback; that callback only touches the
        // netpacket module's own state, never `State`, which is why this is
        // safe to do from inside `with_state`.
        // The link cable goes on when a session starts and comes off when it
        // stops, because a cable is a property of the SESSION rather than of the
        // cartridge. A cart that expects the wireless adapter keeps that
        // instead: the two are different devices on the same port, and no game
        // uses both, so attaching the cable only where there is no adapter keeps
        // this change away from all 43 adapter titles.
        if let Some(gba) = &mut s.gba {
            let want = netpacket::is_active() && !gba.rfu_attached();
            if want != gba.cable_attached() {
                if want {
                    gba.connect_cable(Box::new(netpacket::NetCable));
                } else {
                    gba.disconnect_cable();
                }
                log_line(
                    s.log,
                    &format!(
                        "[cable] {} self_id={}",
                        if want { "attached" } else { "detached" },
                        netpacket::self_id()
                    ),
                );
            }
        }

        if netpacket::is_active() {
            if let Some(gba) = &mut s.gba {
                // Cheap and idempotent, and it has to land before the game
                // hosts a room: the adapter derives its advertised device id
                // from this, and two devices that both think they are peer 0
                // advertise the same id and will not join each other.
                gba.rfu_set_self_id(netpacket::self_id());
                for (from, bytes) in netpacket::drain_inbound() {
                    gba.rfu_net_receive(&bytes, from);
                }
            }
        }

        // Rumble. The game toggles GPIO bit 3 far faster than a phone motor can
        // follow, so this is a level rather than an event, and it is only sent
        // when it changes: a front-end that queues every call would otherwise
        // get 60 a second.
        if let (Some(gba), Some(iface)) = (&mut s.gba, s.rumble) {
            let on = gba.rumble_on();
            if on != s.rumble_last {
                s.rumble_last = on;
                if let Some(set) = iface.set_rumble_state {
                    let strength = if on { u16::MAX } else { 0 };
                    unsafe {
                        set(0, RETRO_RUMBLE_STRONG, strength);
                        set(0, RETRO_RUMBLE_WEAK, strength);
                    }
                }
            }
        }

        // The cartridge clock, pushed every frame. A cartridge clock latches on
        // demand, so a stale value shows up as a game whose day never turns.
        if let Some(gba) = &mut s.gba {
            if gba.cart_has_rtc() {
                gba.set_rtc_unix_time(host_local_unix_time());
            }
        }

        // Fast-forward frameskip.
        //
        // The core already had `render_enabled` and nothing outside the offline
        // runner ever set it, so in fast forward we composited all 160 lines and
        // converted 38400 pixels for every frame, about nine in ten of which are
        // never shown. Measured on the owner's save state: rendering costs
        // 279 us of a 901 us frame, so skipping it is worth about +38%.
        //
        // The cadence is driven by REAL elapsed time rather than a fixed ratio,
        // so it self-tunes: the picture updates about 60 times a second whether
        // the core is running at 4x or 10x, instead of a fixed divisor that
        // either wastes work at low speeds or stutters at high ones.
        // Driven ENTIRELY by elapsed real time, and deliberately NOT by asking
        // the front end whether it is fast-forwarding.
        //
        // This first shipped gated on RETRO_ENVIRONMENT_GET_FASTFORWARDING and
        // did nothing at all, because this host does not implement that command:
        // the device log says `Unhandled env cmd=65` for us and for
        // GenesisPlusGX, melonDS DS and Beetle PCE alike. An unanswered env
        // query leaves our bool false, which is indistinguishable from "running
        // at normal speed", so the feature was inert and the measured speed did
        // not move.
        //
        // The clock needs no cooperation and subsumes the query anyway: at 60 Hz
        // every frame arrives more than 14 ms after the last drawn one and so is
        // drawn, and at 6x nearly all of them arrive sooner and are skipped. The
        // threshold sits below 16.67 ms on purpose, so ordinary frame-pacing
        // jitter at normal speed can never drop a frame.
        let now = std::time::Instant::now();
        let draw = s
            .last_drawn
            .map_or(true, |t| now.duration_since(t).as_micros() >= FRAME_DRAW_MIN_US);
        if draw {
            s.last_drawn = Some(now);
        }
        if let Some(gba) = &mut s.gba {
            gba.render_enabled = draw;
        }

        // Run one frame and expand the RGB555 framebuffer to XRGB8888.
        if let Some(gba) = &mut s.gba {
            // Hand the frame a way to pull packets mid-flight. This closure
            // only touches the netpacket module's own state, never `State`,
            // which is what makes it safe to call with the core borrowed.
            let fb = if netpacket::is_active() {
                gba.run_frame_polling(&mut netpacket::drain_inbound)
            } else {
                gba.run_frame()
            };
            // A skipped frame leaves `s.frame` holding the last drawn picture,
            // which is re-sent below. Deliberately NOT passing a null frame to
            // signal a duplicate: that is valid libretro, but it depends on the
            // front end handling it, and re-sending a buffer we already own
            // cannot go wrong.
            if draw {
                for (dst, &px) in s.frame.iter_mut().zip(fb.iter()) {
                    *dst = rgb555_to_xrgb8888(px);
                }
            }
        }

        // Anything the adapter produced during that frame goes out now. Doing
        // it after the frame rather than before means a reply leaves in the
        // same frame the game asked for it, instead of sitting for 16 ms.
        if netpacket::is_active() {
            if let Some(gba) = &mut s.gba {
                for pkt in gba.rfu_take_outbox() {
                    netpacket::send(pkt.to, &pkt.bytes);
                }
            }
        }

        // Report what the adapter is doing, but only when it changes. Every
        // way this feature fails is silent, so without a line here the only
        // evidence on a phone is the absence of packets, which is exactly how
        // two separate bugs stayed invisible for a whole device session.
        if let Some(gba) = &s.gba {
            if let (Some((cmds, _resets, state)), Some((peers, conns, dropped))) =
                (gba.rfu_stats(), gba.rfu_session_stats())
            {
                let (seen, unknown) = gba.rfu_command_masks().unwrap_or((0, 0));
                // The command count moves every frame on a busy link, so it is
                // deliberately NOT part of the change test: including it would
                // log sixty lines a second and bury the transitions that matter.
                let cs = gba.rfu_connect_stats().unwrap_or((0, 0, 0, 0, 0, 0));
                let ws = gba.rfu_wait_stats().unwrap_or((0, 0, 0, 0, 0));
                let cl = gba.rfu_client_life().unwrap_or((0, 0, 0, 0, 0, 0));
                let tx = gba.rfu_tx_stats().unwrap_or((0, 0, 0));
                let rx = gba.rfu_rx_stats().unwrap_or((0, 0, 0, 0, 0));
                let cl2 = gba.rfu_client_left().unwrap_or((0, 0, 0));
                let now = (
                    cs.0 + cs.1 + cs.2 + cs.3 + cs.4 + cs.5 + ws.1 + ws.2 + ws.3 + ws.4
                        + cl.0 as u64 + cl.2 + cl.3 + cl.4 + cl.5
                        + tx.0 + tx.1 + tx.2
                        + rx.1 + rx.2 + rx.3 + rx.4 as u64
                        + cl2.0 + cl2.1 + cl2.2,
                    peers, conns, dropped, state, seen, unknown,
                );
                if s.rfu_last != Some(now) {
                    s.rfu_last = Some(now);
                    log_line(
                        s.log,
                        &format!(
                            "[rfu] state={state} self_id={} commands={cmds} peers_seen={peers} connections={conns} dropped={dropped} cmds_seen={seen:016X} unknown={unknown:016X} connect(asked={} nopeer={} sent={} nacked={}) incoming(got={} refused={}) wait(timeout={} data={} disc={}) blocks(out={} in={}) tx(rtx={} noclient={} toolong={}) rx(queued={} reject={} qmax={}) drop(host={} client={}) NOW(clients={} peers={}) left(added={} timeout={} told={} wiped={}) ileft(self={} told={} silent={})",
                            netpacket::self_id(),
                            cs.0, cs.1, cs.2, cs.3, cs.4, cs.5,
                            ws.0, ws.1, ws.2, ws.3, ws.4,
                            tx.0, tx.1, tx.2,
                            rx.0, rx.1, rx.4, rx.2, rx.3,
                            cl.0, cl.1, cl.2, cl.3, cl.4, cl.5,
                            cl2.0, cl2.1, cl2.2
                        ),
                    );
                }
            }
        }
        // The flash save, which a trade destroyed on both devices at once. A
        // fresh buffer and a chip-erased one are both all 0xFF, so the file
        // cannot say which happened and this can. Logged on change only.
        if let Some(gba) = &s.gba {
            if let Some(f) = gba.flash_stats() {
                if s.flash_last != Some(f) {
                    s.flash_last = Some(f);
                    log_line(
                        s.log,
                        &format!(
                            "[save] flash chip_erase={} sector_erase={} program={} bank_set={} last_sector={:#06X} bank={}",
                            f.0, f.1, f.2, f.3, f.4, f.5
                        ),
                    );
                }
            }
        }
        // Heartbeat, once a second of emulated time. The logcat timestamp on
        // consecutive lines gives the real elapsed time, so 60 emulated frames
        // taking longer than a second is visible by subtraction.
        s.frames += 1;
        if s.frames % 60 == 0 {
            // Idle-loop skipping reported alongside, because on device the only
            // question that matters is whether it found the game's wait loop.
            // Zero skips with a watched address is a different fault from never
            // finding one, and from outside the two look the same.
            let idle = s
                .gba
                .as_ref()
                .map(|g| {
                    // Audio FIFO health rides along because on a device that
                    // cannot keep up, audio starves before the frame rate visibly
                    // drops, and an underrun holds a stale PCM sample. Without it,
                    // "it sounded rough on the tablet" cannot be told apart from a
                    // bug in the mixer. Both counters are cumulative, like the
                    // skip counts, so consecutive heartbeats give the rate.
                    // The cable rides along for the same reason, and only while
                    // one is attached: a cable that is moving words and a cable
                    // that is silently never asked look identical on screen, so
                    // the completed count is the one number that tells them
                    // apart. `lost` is give-ups and `extra` is peers this cable
                    // cannot carry.
                    let cable = if g.cable_attached() {
                        let (done, lost, extra) = netpacket::cable_stats();
                        format!(" cable_done={done} cable_lost={lost} cable_extra={extra}")
                    } else {
                        String::new()
                    };
                    format!(
                        " idle_pc={:08X} skips={} skipped_cycles={} ds_pops={} ds_underruns={}{cable}",
                        g.idle.probe_r15.wrapping_sub(4),
                        g.idle.skips,
                        g.idle.skipped_cycles,
                        g.bus.apu.dbg_pops,
                        g.bus.apu.dbg_underruns
                    )
                })
                .unwrap_or_default();
            log_line(s.log, &format!("[perf] emulated_frames={}{idle}", s.frames));
        }
        if let Some(video) = s.video {
            unsafe {
                video(
                    s.frame.as_ptr() as *const c_void,
                    SCREEN_W as u32,
                    SCREEN_H as u32,
                    SCREEN_W * 4,
                );
            }
        }

        // Feed this frame's audio (interleaved stereo i16), at the rate reported
        // in retro_get_system_av_info, which is how the host paces us to real
        // time.
        if let (Some(gba), Some(audio)) = (&mut s.gba, s.audio_batch) {
            let samples = gba.take_audio();
            if !samples.is_empty() {
                unsafe { audio(samples.as_ptr(), samples.len() / 2) };
            }
        }
    });
}

// --- Save RAM: exposed so the front-end persists it to disk ------------------

#[no_mangle]
pub extern "C" fn retro_get_memory_data(id: u32) -> *mut c_void {
    with_state(|s| {
        let Some(gba) = s.gba.as_mut() else {
            return ptr::null_mut();
        };
        match id {
            // EWRAM is what every other GBA core reports as system RAM, and
            // some front-ends (and the cheat engine) ask for it instead of
            // walking the memory map.
            //
            // ONLY answered when the front-end took the memory map. If it did
            // not, anything laying the console's regions over this buffer builds
            // a map that is confidently wrong, because the GBA's two work RAMs
            // are not contiguous and cannot be described by one pointer. Null
            // makes that failure loud (RetroAchievements declines to start)
            // instead of silent (it evaluates against the wrong addresses and
            // unlocks things the player did not earn). A missing feature is
            // recoverable; a false unlock is written to their account and is
            // not ours to take back.
            RETRO_MEMORY_SYSTEM_RAM if s.map_published => {
                gba.bus.ewram.as_mut_ptr() as *mut c_void
            }
            RETRO_MEMORY_SAVE_RAM if !gba.bus.save.data.is_empty() => {
                gba.bus.save.data.as_mut_ptr() as *mut c_void
            }
            _ => ptr::null_mut(),
        }
    })
}
#[no_mangle]
pub extern "C" fn retro_get_memory_size(id: u32) -> usize {
    with_state(|s| {
        let Some(gba) = s.gba.as_ref() else { return 0 };
        match id {
            RETRO_MEMORY_SYSTEM_RAM if s.map_published => gba.bus.ewram.len(),
            RETRO_MEMORY_SAVE_RAM => gba.bus.save.data.len(),
            _ => 0,
        }
    })
}

// --- Save states: not implemented yet ----------------------------------------
#[no_mangle]
pub extern "C" fn retro_serialize_size() -> usize {
    with_state(|s| s.gba.as_ref().map(|g| g.save_state().len()).unwrap_or(0))
}
#[no_mangle]
pub unsafe extern "C" fn retro_serialize(data: *mut c_void, size: usize) -> bool {
    with_state(|s| {
        let gba = match s.gba.as_ref() {
            Some(g) => g,
            None => return false,
        };
        let blob = gba.save_state();
        if data.is_null() || blob.len() > size {
            return false;
        }
        std::ptr::copy_nonoverlapping(blob.as_ptr(), data as *mut u8, blob.len());
        true
    })
}
#[no_mangle]
pub unsafe extern "C" fn retro_unserialize(data: *const c_void, size: usize) -> bool {
    with_state(|s| {
        let gba = match s.gba.as_mut() {
            Some(g) => g,
            None => return false,
        };
        if data.is_null() {
            return false;
        }
        let slice = std::slice::from_raw_parts(data as *const u8, size);
        gba.load_state(slice)
    })
}
#[no_mangle]
pub extern "C" fn retro_cheat_reset() {}
#[no_mangle]
pub extern "C" fn retro_cheat_set(_index: u32, _enabled: bool, _code: *const c_char) {}
#[no_mangle]
pub unsafe extern "C" fn retro_load_game_special(
    _game_type: u32,
    _info: *const retro_game_info,
    _num: usize,
) -> bool {
    false
}
#[no_mangle]
pub extern "C" fn retro_get_region() -> u32 {
    0 // RETRO_REGION_NTSC
}

#[cfg(test)]
mod tests {
    use super::*;

    static mut CAPTURED: Option<Vec<(u64, usize, usize, usize)>> = None;
    /// (cmd, data pointer) of the netpacket interface offer, if the core made
    /// one. See `offers_the_netpacket_interface_to_the_frontend`.
    static mut FAKE_FF: bool = false;
    static mut NETPACKET_OFFER: Option<(u32, *mut c_void)> = None;

    /// The core keeps ONE global `State`, and `CAPTURED` below is global too, so
    /// every test that touches it must hold this for its WHOLE body, not just
    /// across the load: the other test swaps the environment callback after
    /// loading, which is enough to make this one capture nothing.
    ///
    /// Cargo runs tests in threads inside one process. Without the lock they
    /// trample each other: `publishes_the_memory_map_rcheevos_needs` passed for
    /// months because it happened to be the only test loading a ROM, and adding a
    /// second one made BOTH fail. A test that only passes when it runs alone is
    /// not a test.
    ///
    /// Poisoning is ignored on purpose. A panic in one test should fail that
    /// test, not cascade into every other one and hide which was the real
    /// failure.
    static CORE_LOCK: std::sync::Mutex<()> = std::sync::Mutex::new(());

    unsafe extern "C" fn fake_env(cmd: u32, data: *mut c_void) -> bool {
        // Assert the LITERAL value, not the constant. The previous test matched
        // on our own symbol, so a wrong constant looked correct in the test and
        // was never sent to any real front-end. A test that agrees with the bug
        // is worse than no test, because it is evidence of the wrong thing.
        assert_eq!(
            RETRO_ENVIRONMENT_SET_MEMORY_MAPS, 0x1_0024,
            "SET_MEMORY_MAPS is 36 | EXPERIMENTAL; a bare 36 is silently ignored"
        );
        assert_eq!(
            RETRO_ENVIRONMENT_GET_SENSOR_INTERFACE, 0x1_0019,
            "GET_SENSOR_INTERFACE is 25 | EXPERIMENTAL"
        );
        assert_eq!(
            RETRO_ENVIRONMENT_GET_RUMBLE_INTERFACE, 23,
            "GET_RUMBLE_INTERFACE is a PLAIN 23, with no EXPERIMENTAL bit;              copying the sensor interface's shape and ORing 0x10000 asks for a              different command and the front-end writes a different struct back"
        );
        if cmd == 78 {
            NETPACKET_OFFER = Some((cmd, data));
            return true;
        }
        if cmd == RETRO_ENVIRONMENT_GET_FASTFORWARDING {
            // The LITERAL, for the same reason as the asserts above: a wrong
            // constant would make the core read some other front-end state and
            // skip frames for reasons unrelated to speed.
            assert_eq!(RETRO_ENVIRONMENT_GET_FASTFORWARDING, 65);
            *(data as *mut bool) = FAKE_FF;
            return true;
        }
        if cmd == RETRO_ENVIRONMENT_SET_MEMORY_MAPS {
            let map = &*(data as *const retro_memory_map);
            let descs =
                std::slice::from_raw_parts(map.descriptors, map.num_descriptors as usize);
            CAPTURED = Some(
                descs
                    .iter()
                    .map(|d| (d.flags, d.start, d.len, d.ptr as usize))
                    .collect(),
            );
        }
        false
    }

    /// An environment callback that REFUSES the memory map, as a front-end too
    /// old to know the command would.
    unsafe extern "C" fn refusing_env(cmd: u32, _data: *mut c_void) -> bool {
        if cmd == RETRO_ENVIRONMENT_SET_MEMORY_MAPS {
            return false;
        }
        false
    }

    /// Resolve a real address exactly the way rcheevos does, so the published
    /// table can be checked against the algorithm that will actually read it.
    ///
    /// Transcribed from `rc_libretro_memory_get_descriptor` in
    /// `TrophyHubLibretroHost/third_party/rcheevos/src/rc_libretro.c`: with
    /// `select` zero it matches the address against `start..start + len` and
    /// nothing else, so order in the table does not matter and a gap is simply
    /// unmapped.
    fn rcheevos_resolve(
        descs: &[(u64, usize, usize, usize)],
        real: usize,
    ) -> Option<(usize, usize)> {
        descs
            .iter()
            .find(|&&(_, start, len, _)| real >= start && real < start + len)
            .map(|&(_, start, _, ptr)| (ptr, real - start))
    }

    /// Every byte rcheevos asks for must land in the buffer it expects.
    ///
    /// RetroAchievements addresses a GBA as one flat space and rcheevos turns
    /// that into three real ranges (consoleinfo.c, RC_CONSOLE_GAMEBOY_ADVANCE):
    ///
    /// ```text
    ///   RA 0x000000..0x007FFF  ->  0x03000000   32 KB   IWRAM
    ///   RA 0x008000..0x047FFF  ->  0x02000000  256 KB   EWRAM
    ///   RA 0x048000..0x057FFF  ->  0x0E000000   64 KB   cartridge SRAM
    /// ```
    ///
    /// If any of those resolves to the wrong buffer or the wrong offset, an
    /// achievement reads bytes that are real and mean something else, and the
    /// result is a FALSE UNLOCK against the owner's account rather than a
    /// visible failure. The boundaries are checked as well as the bases,
    /// because an off-by-one length is exactly what aliases one region into
    /// the next.
    #[test]
    fn the_memory_map_resolves_the_way_rcheevos_reads_it() {
        let _core = CORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let descs = load(b"FLASH_V126");
        let find = |name: &str, start: usize| -> (usize, usize, usize) {
            let d = descs
                .iter()
                .find(|&&(_, s, _, _)| s == start)
                .unwrap_or_else(|| panic!("no descriptor at {start:#X} for {name}"));
            (d.1, d.2, d.3)
        };
        let (_, iwram_len, iwram) = find("IWRAM", 0x0300_0000);
        let (_, ewram_len, ewram) = find("EWRAM", 0x0200_0000);

        assert_eq!(iwram_len, 32 * 1024, "IWRAM is 32 KB");
        assert_eq!(ewram_len, 256 * 1024, "EWRAM is 256 KB");
        let (_, sram_len, sram) = find("SRAM", 0x0E00_0000);
        assert!(sram_len >= 64 * 1024, "rcheevos reads 64 KB of save RAM");

        for (label, real, want_ptr, want_off) in [
            ("IWRAM base", 0x0300_0000usize, iwram, 0usize),
            ("IWRAM last", 0x0300_7FFF, iwram, 0x7FFF),
            ("EWRAM base", 0x0200_0000, ewram, 0),
            ("EWRAM last", 0x0203_FFFF, ewram, 0x3_FFFF),
            ("SRAM base", 0x0E00_0000, sram, 0),
            ("SRAM last", 0x0E00_FFFF, sram, 0xFFFF),

        ] {
            let got = rcheevos_resolve(&descs, real)
                .unwrap_or_else(|| panic!("{label} ({real:#X}) resolved to nothing"));
            assert_eq!(got, (want_ptr, want_off), "{label} resolved to the wrong place");
        }

        // One past the end of IWRAM must NOT fall into another region: that is
        // the aliasing an over-long descriptor would cause.
        assert!(
            rcheevos_resolve(&descs, 0x0300_8000).is_none(),
            "IWRAM must stop at 32 KB, not run on into the next descriptor"
        );
    }

    /// A ROM carrying the save-type marker the detector looks for.
    fn rom_with(marker: &[u8]) -> Vec<u8> {
        let mut rom = vec![0u8; 0x2000];
        rom[0x1000..0x1000 + marker.len()].copy_from_slice(marker);
        rom
    }

    fn load(marker: &[u8]) -> Vec<(u64, usize, usize, usize)> {
        unsafe {
            retro_unload_game();
            CAPTURED = None;
            retro_set_environment(Some(fake_env));
            let rom = rom_with(marker);
            let info = retro_game_info {
                path: ptr::null(),
                data: rom.as_ptr() as *const c_void,
                size: rom.len(),
                meta: ptr::null(),
            };
            assert!(retro_load_game(&info));
            CAPTURED.take().expect("core published no memory map")
        }
    }

    /// Running faster than real time must stop compositing frames nobody sees,
    /// and running at normal speed must never drop one.
    ///
    /// The cadence is driven by the CLOCK, not by asking the front end whether
    /// it is fast-forwarding. The first version did ask, and did nothing at
    /// all: this host answers `Unhandled env cmd=65`, which leaves the bool
    /// false, which is indistinguishable from normal speed. The measured speed
    /// on device did not move and the feature looked implemented.
    ///
    /// Pinned from BOTH directions, because the dangerous failure is not a slow
    /// core, it is one that stops drawing when it should be drawing.
    #[test]
    fn frames_are_skipped_only_when_running_ahead_of_real_time() {
        let _core = CORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        unsafe {
            retro_unload_game();
            retro_set_environment(Some(fake_env));
            let rom = rom_with(b"FLASH1M_V103");
            let info = retro_game_info {
                path: ptr::null(),
                data: rom.as_ptr() as *const c_void,
                size: rom.len(),
                meta: ptr::null(),
            };
            assert!(retro_load_game(&info));

            // Asserted against the LITERAL 60 Hz frame time, not a symbol, and
            // checked directly rather than through timing: a threshold at or
            // above a frame interval drops real frames on device, and the
            // 17 ms sleep below has too little margin to catch it once an
            // unoptimised retro_run is added to the gap.
            assert!(
                FRAME_DRAW_MIN_US < 16_667,
                "the draw interval must sit below a 60 Hz frame, is {FRAME_DRAW_MIN_US} us"
            );

            let drawn = || with_state(|s| s.gba.as_ref().map_or(false, |g| g.render_enabled));

            // Spaced like a 60 Hz front end: every frame must be drawn. A
            // threshold at or above 16.67 ms would make this flaky and cost a
            // real dropped frame on device, which is why it sits at 14.
            for _ in 0..5 {
                std::thread::sleep(std::time::Duration::from_millis(17));
                retro_run();
                assert!(drawn(), "a frame arriving 17 ms after the last must be drawn");
            }

            // Back to back, far faster than real time.
            //
            // Asserted as the INVARIANT the rule actually promises, at most one
            // draw per 14 ms, with the elapsed time measured rather than
            // assumed. A fixed skip ratio would be testing how fast this
            // machine runs frames: an unoptimised build takes milliseconds per
            // frame and legitimately draws more of them. Two earlier versions of
            // this assertion failed for exactly that reason.
            retro_run();
            let t0 = std::time::Instant::now();
            let mut draws = 0u128;
            for _ in 0..20 {
                retro_run();
                if drawn() {
                    draws += 1;
                }
            }
            let budget = t0.elapsed().as_millis() / 14 + 2;
            assert!(
                draws <= budget,
                "at most one draw per 14 ms: {draws} draws in {} ms (budget {budget})",
                t0.elapsed().as_millis()
            );
            retro_unload_game();
        }
    }

    /// A reset must not erase the cartridge save.
    ///
    /// Rebuilding the machine re-runs save detection, which allocates a blank
    /// all-0xFF buffer, and the front-end flushes whatever
    /// `retro_get_memory_data` points at to the .sav file. So a reset used to
    /// destroy the player's save, silently, with the damage only landing on the
    /// next flush. Two real saves were lost to this on device.
    ///
    /// Pinned with a non-0xFF pattern specifically, because a blank save and an
    /// erased one are both 0xFF and a test written with 0xFF would pass either
    /// way.
    #[test]
    fn a_reset_keeps_the_cartridge_save() {
        let _core = CORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        unsafe {
            retro_unload_game();
            retro_set_environment(Some(fake_env));
            let rom = rom_with(b"FLASH1M_V103");
            let info = retro_game_info {
                path: ptr::null(),
                data: rom.as_ptr() as *const c_void,
                size: rom.len(),
                meta: ptr::null(),
            };
            assert!(retro_load_game(&info));

            let size = retro_get_memory_size(RETRO_MEMORY_SAVE_RAM);
            assert!(size >= 0x2_0000, "a 1 Mbit flash save, got {size:#X}");

            // Write a pattern the way a game would see it, through the live
            // buffer the front-end is handed.
            let before: Vec<u8> = (0..size).map(|i| (i % 251) as u8).collect();
            let ptr = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM) as *mut u8;
            assert!(!ptr.is_null());
            std::ptr::copy_nonoverlapping(before.as_ptr(), ptr, size);

            retro_reset();

            // The front-end may hold a NEW pointer after a rebuild, so ask
            // again rather than reusing the old one.
            let after_ptr = retro_get_memory_data(RETRO_MEMORY_SAVE_RAM) as *const u8;
            assert!(!after_ptr.is_null(), "still a save after the reset");
            assert_eq!(
                retro_get_memory_size(RETRO_MEMORY_SAVE_RAM),
                size,
                "the save did not change size"
            );
            let after = std::slice::from_raw_parts(after_ptr, size);
            assert_eq!(after, &before[..], "the save survived the reset");
            retro_unload_game();
        }
    }

    /// The wireless adapter is only reachable if the core OFFERS the netpacket
    /// interface, and it has to do so from `retro_set_environment`.
    ///
    /// This is pinned by a test because the failure is completely silent and
    /// cost real time on device: the frontend brings a session up, reports a
    /// healthy LAN match, and the core simply never gets `start()`, so both
    /// players sit in an empty room with no error anywhere. Worse, the host
    /// does NOT re-run `set_environment` on a plain ROM reload (only on a
    /// fresh core load), so the one log line that proves it is easy to miss
    /// and easy to misread as absent.
    #[test]
    fn offers_the_netpacket_interface_to_the_frontend() {
        let _core = CORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        unsafe {
            NETPACKET_OFFER = None;
            retro_set_environment(Some(fake_env));
            let (cmd, data) = NETPACKET_OFFER.take().expect(
                "the core must offer the netpacket interface from retro_set_environment",
            );
            // The literal, not our constant: a test phrased against the symbol
            // it is checking agrees with any bug in it. 78 is plain, with no
            // experimental bit.
            assert_eq!(cmd, 78);
            assert!(!data.is_null(), "the frontend is handed a real struct");
            // And the struct must carry the three callbacks a session needs.
            let cb = &*(data as *const netpacket::RetroNetpacketCallback);
            assert!(cb.has_start_receive_stop(), "start / receive / stop are all set");
        }
    }

    /// Without this map the Trophy Hub host's rc_libretro_memory_init fails and
    /// it parks on "waiting for core memory map" forever. rcheevos matches the
    /// descriptors by REAL address, and its GBA layout puts IWRAM at RA address
    /// 0 with EWRAM after it, so both blocks have to be published with their
    /// true bases rather than in RA order.
    ///
    /// One test, not three, because the core keeps its state in a process
    /// global and cargo runs tests on parallel threads.
    #[test]
    fn publishes_the_memory_map_rcheevos_needs() {
        let _core = CORE_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let sram = load(b"SRAM_V113");
        let iwram = sram.iter().find(|d| d.1 == 0x0300_0000).expect("IWRAM descriptor");
        let ewram = sram.iter().find(|d| d.1 == 0x0200_0000).expect("EWRAM descriptor");
        assert_eq!(iwram.2, 32 * 1024, "IWRAM is 32 KB");
        assert_eq!(ewram.2, 256 * 1024, "EWRAM is 256 KB");
        assert_eq!(iwram.0, RETRO_MEMDESC_SYSTEM_RAM);
        assert_eq!(ewram.0, RETRO_MEMDESC_SYSTEM_RAM);

        // A cart with SRAM is memory mapped at 0x0E000000 and RetroAchievements
        // documents it as a GBA region, so it is published.
        let save = sram.iter().find(|d| d.1 == 0x0E00_0000).expect("SRAM descriptor");
        assert_eq!(save.0, RETRO_MEMDESC_SAVE_RAM);
        // And the front-end must be able to persist it to disk.
        assert!(
            retro_get_memory_size(RETRO_MEMORY_SAVE_RAM) > 0,
            "withholding the descriptor must not stop saves being written"
        );
        assert!(!retro_get_memory_data(RETRO_MEMORY_SAVE_RAM).is_null());

        // EEPROM is NOT memory mapped: it is clocked in through region 0xD. If
        // it were published at the SRAM address the achievement runtime would
        // read real bytes that mean something else, which unlocks value
        // achievements against garbage instead of failing visibly. A wrong
        // mapping is worse than a missing one.
        let eeprom = load(b"EEPROM_V122");
        assert!(
            eeprom.iter().all(|d| d.1 != 0x0E00_0000),
            "EEPROM must not be published as SRAM at 0x0E000000"
        );
        assert!(
            eeprom.iter().any(|d| d.1 == 0x0300_0000),
            "work RAM is still published for an EEPROM cart"
        );

        // Resetting rebuilds the machine, which FREES the RAM these pointers
        // refer to, so the map has to be published again or the front-end reads
        // freed heap. That is the bug which awarded the owner three
        // achievements he had not earned on Yoshi Topsy-Turvy, and rc_client
        // resets the core itself when hardcore is toggled, so RetroAchievements
        // can trigger the corruption it then reads. False unlocks reach a
        // player's account and cannot be taken back.
        //
        // Folded into this test rather than given its own, because the core
        // keeps its state in a process global and cargo runs tests on parallel
        // threads: a second stateful test races this one.
        unsafe {
            CAPTURED = None;
            retro_reset();
            let after = CAPTURED.take().expect("reset published no memory map");
            assert_eq!(after.len(), eeprom.len(), "reset changed the regions");
            assert!(after.iter().all(|d| d.3 != 0), "reset published a null pointer");
            for (a, b) in after.iter().zip(eeprom.iter()) {
                assert_eq!((a.0, a.1, a.2), (b.0, b.1, b.2), "region changed on reset");
            }
        }

        // A front-end that REFUSES the map must not then be offered SYSTEM_RAM.
        // Laying the GBA's regions over one buffer cannot be correct, because
        // IWRAM and EWRAM are not contiguous, and a confidently wrong map is
        // what unlocked achievements the owner had not earned. Null makes the
        // failure loud instead.
        unsafe {
            retro_set_environment(Some(refusing_env));
            let rom = rom_with(b"SRAM_V113");
            let info = retro_game_info {
                path: ptr::null(),
                data: rom.as_ptr() as *const c_void,
                size: rom.len(),
                meta: ptr::null(),
            };
            assert!(retro_load_game(&info));
            assert!(
                retro_get_memory_data(RETRO_MEMORY_SYSTEM_RAM).is_null(),
                "SYSTEM_RAM must be withheld when the map was refused"
            );
            assert_eq!(retro_get_memory_size(RETRO_MEMORY_SYSTEM_RAM), 0);
            // Save RAM is unaffected: it is a real region on its own terms.
            assert!(!retro_get_memory_data(RETRO_MEMORY_SAVE_RAM).is_null());
            // Put the accepting env back for anything after this.
            retro_set_environment(Some(fake_env));
        }
    }
}

/// The host's wall clock in **local** time, as seconds since the Unix epoch.
///
/// The GBA clock reports a broken-down local date, so a core that pushes UTC
/// puts a player's in-game day out by their whole timezone offset. Boktai is
/// built around day and night and Pokemon grows berries and turns tides on it.
///
/// This crate carries no dependencies, so the offset comes from the C runtime
/// that every target already links. Only the first nine members of `struct tm`
/// are read, which are `int` and in this order on bionic, the Windows CRT and
/// glibc alike; glibc's extra members sit after them. The broken-down local
/// date is then folded back into seconds with the core's own civil-date
/// arithmetic rather than `mktime`, which keeps the whole conversion inside
/// code this repo tests.
#[repr(C)]
struct CTm {
    sec: i32,
    min: i32,
    hour: i32,
    mday: i32,
    mon: i32,
    year: i32,
    wday: i32,
    yday: i32,
    isdst: i32,
}

extern "C" {
    fn localtime(time: *const i64) -> *const CTm;
}

fn host_local_unix_time() -> i64 {
    let utc = std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .map(|d| d.as_secs() as i64)
        .unwrap_or(gba_core::rtc::DEFAULT_UNIX_TIME);
    let tm = unsafe { localtime(&utc as *const i64) };
    if tm.is_null() {
        return utc;
    }
    let tm = unsafe { &*tm };
    let days = gba_core::rtc::days_from_civil(
        i64::from(tm.year) + 1900,
        (tm.mon + 1) as u32,
        tm.mday as u32,
    );
    days * 86_400 + i64::from(tm.hour) * 3600 + i64::from(tm.min) * 60 + i64::from(tm.sec)
}

#[cfg(test)]
mod rtc_tests {
    /// The local-time conversion reads a C `struct tm` through a hand-written
    /// layout, and the failure mode of getting that wrong is silent: a garbage
    /// year or hour that still looks like a number. Every real timezone is
    /// within 14 hours of UTC, and DST moves it by at most one more, so a
    /// correct conversion can never be a day away.
    #[test]
    fn the_host_local_time_is_a_plausible_offset_from_utc() {
        let utc = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs() as i64;
        let local = super::host_local_unix_time();
        let offset = local - utc;
        assert!(
            offset.abs() <= 15 * 3600,
            "local time is {offset} seconds from UTC, which is not a timezone;              the struct tm layout is probably wrong"
        );
        // And it must land on a whole minute offset, which every zone does.
        assert_eq!(offset % 60, 0, "timezone offsets are whole minutes");
    }
}
