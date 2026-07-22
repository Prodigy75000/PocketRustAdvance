//! libretro front-end ABI for gba-core.
//!
//! Exposes the standard `retro_*` C entry points so the GBA core can be loaded
//! by any libretro host (RetroArch, Trophy Hub's libretro host, ...). The core
//! runs single-threaded, so all state lives in a thread-local `State`.
//!
//! The core boots directly into the cartridge with an HLE BIOS (no BIOS image
//! required) and produces an RGB555 framebuffer, which we expand to XRGB8888
//! for the host. Audio and save-states are not wired yet.

#![allow(non_camel_case_types)]
#![allow(clippy::missing_safety_doc)]

use gba_core::{Button, Gba, SCREEN_H, SCREEN_W};
use std::cell::RefCell;
use std::ffi::{c_char, c_void};
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
const RETRO_ENVIRONMENT_SET_PIXEL_FORMAT: u32 = 10;
const RETRO_PIXEL_FORMAT_XRGB8888: i32 = 1;

const RETRO_MEMORY_SAVE_RAM: u32 = 0;

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
            frame: Vec::new(),
            env: None,
            video: None,
            audio_batch: None,
            input_poll: None,
            input_state: None,
        }
    }
}

thread_local! {
    static STATE: RefCell<State> = const { RefCell::new(State::new()) };
}

fn with_state<R>(f: impl FnOnce(&mut State) -> R) -> R {
    STATE.with(|s| f(&mut s.borrow_mut()))
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

#[no_mangle]
pub extern "C" fn retro_reset() {
    with_state(|s| {
        if !s.rom.is_empty() {
            s.gba = Some(Gba::new(s.rom.clone(), Vec::new()));
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
        // Direct boot with the HLE BIOS (no BIOS image needed).
        s.gba = Some(Gba::new(rom, Vec::new()));
    });
    true
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
    if id != RETRO_MEMORY_SAVE_RAM {
        return ptr::null_mut();
    }
    with_state(|s| match &mut s.gba {
        Some(gba) if !gba.bus.save.data.is_empty() => {
            gba.bus.save.data.as_mut_ptr() as *mut c_void
        }
        _ => ptr::null_mut(),
    })
}
#[no_mangle]
pub extern "C" fn retro_get_memory_size(id: u32) -> usize {
    if id != RETRO_MEMORY_SAVE_RAM {
        return 0;
    }
    with_state(|s| s.gba.as_ref().map(|g| g.bus.save.data.len()).unwrap_or(0))
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
