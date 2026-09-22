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
use gba_core::{Button, Gba, SCREEN_H, SCREEN_W};
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
const RETRO_ENVIRONMENT_SET_MEMORY_MAPS: u32 = 36;
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

/// Bundled open-source GBA BIOS (Normmatt's clean-room reimplementation — the
/// same freely-redistributable image gpSP ships). We boot through it by default
/// so no BIOS install is needed and behaviour matches gpSP. It cannot serve
/// titles that read the *real* BIOS ROM's bytes directly (a reimplementation has
/// different bytes at those offsets); for those, a real `gba_bios.bin` in the
/// system directory overrides this.
static OPEN_BIOS: &[u8] = include_bytes!("../open_gba_bios.bin");

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
        }
    }
}

/// Trophy Hub's libretro host registers callbacks (env, video, audio, input) on
/// its main thread but runs frames on a dedicated emulation thread. So the state
/// must be a single process-global, not thread-local — otherwise `retro_run`
/// sees a fresh empty state with null callbacks (black screen, no audio). This
/// mirrors how C libretro cores keep their state in plain `static`s.
struct GlobalState(UnsafeCell<State>);

// SAFETY: libretro serializes every call into the core — `retro_run`, the
// `retro_set_*` registrations and load/unload never overlap — so there is never
// concurrent access to the single STATE instance.
unsafe impl Sync for GlobalState {}

static STATE: GlobalState = GlobalState(UnsafeCell::new(State::new()));

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    // SAFETY: see `GlobalState` — accesses are serialized by the frontend.
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
    (*info).library_version = c"0.1.0".as_ptr();
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
        sample_rate: 32_768.0,         // GBA rate; no audio produced yet
    };
}

#[no_mangle]
pub extern "C" fn retro_set_environment(cb: retro_environment_t) {
    with_state(|s| s.env = cb);
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
            s.gba = Some(Gba::new(s.rom.clone(), s.bios.clone()));
        }
    });
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
        publish_memory_map(s);
    });
    true
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
    // real but mean something else. A wrong mapping is worse than a missing
    // one: it unlocks value achievements against garbage instead of failing
    // visibly.
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
    unsafe {
        env(
            RETRO_ENVIRONMENT_SET_MEMORY_MAPS,
            &map as *const retro_memory_map as *mut c_void,
        );
    }
}

#[no_mangle]
pub extern "C" fn retro_unload_game() {
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

        // Run one frame and expand the RGB555 framebuffer to XRGB8888.
        if let Some(gba) = &mut s.gba {
            let fb = gba.run_frame();
            for (dst, &px) in s.frame.iter_mut().zip(fb.iter()) {
                *dst = rgb555_to_xrgb8888(px);
            }
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

        // Feed this frame's audio (interleaved stereo i16). Silent for now, but
        // emitted at the correct rate so the host paces us to real time.
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
            RETRO_MEMORY_SYSTEM_RAM => gba.bus.ewram.as_mut_ptr() as *mut c_void,
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
            RETRO_MEMORY_SYSTEM_RAM => gba.bus.ewram.len(),
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

    static mut CAPTURED: Option<Vec<(u64, usize, usize)>> = None;

    unsafe extern "C" fn fake_env(cmd: u32, data: *mut c_void) -> bool {
        if cmd == RETRO_ENVIRONMENT_SET_MEMORY_MAPS {
            let map = &*(data as *const retro_memory_map);
            let descs =
                std::slice::from_raw_parts(map.descriptors, map.num_descriptors as usize);
            CAPTURED = Some(descs.iter().map(|d| (d.flags, d.start, d.len)).collect());
        }
        false
    }

    /// A ROM carrying the save-type marker the detector looks for.
    fn rom_with(marker: &[u8]) -> Vec<u8> {
        let mut rom = vec![0u8; 0x2000];
        rom[0x1000..0x1000 + marker.len()].copy_from_slice(marker);
        rom
    }

    fn load(marker: &[u8]) -> Vec<(u64, usize, usize)> {
        unsafe {
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
        let sram = load(b"SRAM_V113");
        let iwram = sram.iter().find(|d| d.1 == 0x0300_0000).expect("IWRAM descriptor");
        let ewram = sram.iter().find(|d| d.1 == 0x0200_0000).expect("EWRAM descriptor");
        assert_eq!(iwram.2, 32 * 1024, "IWRAM is 32 KB");
        assert_eq!(ewram.2, 256 * 1024, "EWRAM is 256 KB");
        assert_eq!(iwram.0, RETRO_MEMDESC_SYSTEM_RAM);
        assert_eq!(ewram.0, RETRO_MEMDESC_SYSTEM_RAM);

        // A cart with SRAM is memory mapped at 0x0E000000 and should be offered.
        let save = sram.iter().find(|d| d.1 == 0x0E00_0000).expect("SRAM descriptor");
        assert_eq!(save.0, RETRO_MEMDESC_SAVE_RAM);

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
    }
}
