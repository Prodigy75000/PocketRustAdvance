//! Bulk compatibility sweep: boot every ROM under a directory headless and
//! record a cheap health signal per ROM, so we can gauge core maturity across
//! thousands of games and surface the hard failures (no-boot / blank screen /
//! panic) for follow-up.
//!
//! Usage: sweep <rom-dir> [frames=250] [out.tsv=sweep.tsv] [workers=cores-2]
//!
//! Each ROM is HLE direct-booted (the shipping libretro config: no BIOS), run
//! for `frames` frames with the same A+Start auto-input pulse the interactive
//! runner uses, and scored by the peak distinct-colour count of any frame (the
//! boot signal), plus CPU steps and audio activity. A per-ROM panic is caught
//! and recorded rather than aborting the sweep. Results are appended to the TSV
//! as they complete, so the run is crash-safe and resumable (re-running skips
//! ROMs already present in the output).

use std::fs::{File, OpenOptions};
use std::io::{BufRead, BufReader, Write};
use std::panic::{self, AssertUnwindSafe};
use std::path::{Path, PathBuf};
use std::sync::atomic::{AtomicUsize, Ordering};
use std::sync::Mutex;

use gba_core::{Button, Gba};

fn collect_gba(root: &Path, out: &mut Vec<PathBuf>) {
    let Ok(entries) = std::fs::read_dir(root) else { return };
    for e in entries.flatten() {
        let p = e.path();
        if p.is_dir() {
            collect_gba(&p, out);
        } else if p.extension().map(|x| x.eq_ignore_ascii_case("gba")).unwrap_or(false) {
            out.push(p);
        }
    }
}

fn distinct(fb: &[u16]) -> usize {
    let mut v: Vec<u16> = fb.to_vec();
    v.sort_unstable();
    v.dedup();
    v.len()
}

/// Boot one ROM and return (peak distinct colours, frame of that peak, CPU
/// steps, non-zero audio samples). Panics propagate to the caller's catch.
/// With `bios` empty this is HLE direct-boot (the shipping config); pass a real
/// 16 KB BIOS (via GBA_BIOS) to measure how many failures are BIOS-dependent.
fn run_one(rom: Vec<u8>, bios: &[u8], frames: u32) -> (usize, u32, u64, usize) {
    let mut gba = Gba::new(rom, bios.to_vec());
    let (mut best, mut best_frame) = (0usize, 0u32);
    // `late` = peak distinct colours over the final ~30 frames. Unlike the
    // all-frames peak, this is immune to the BIOS boot-logo animation (which
    // makes every cart colourful early), so it reflects whether the *game* is
    // actually drawing when we stop, not just that something flashed at boot.
    let late_from = frames.saturating_sub(30);
    let mut late = 0usize;
    for f in 0..frames {
        let p = f % 24 < 4; // pulse A+Start to advance logos / title / dialogue
        gba.set_button(Button::A, p);
        gba.set_button(Button::Start, p);
        let d = distinct(gba.run_frame());
        if d > best {
            best = d;
            best_frame = f;
        }
        if f >= late_from && d > late {
            late = d;
        }
        let _ = gba.take_audio();
    }
    (best, best_frame, gba.steps, late)
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let root = PathBuf::from(args.get(1).expect("usage: sweep <rom-dir> [frames] [out.tsv] [workers]"));
    let frames: u32 = args.get(2).and_then(|s| s.parse().ok()).unwrap_or(250);
    let out_path = args.get(3).cloned().unwrap_or_else(|| "sweep.tsv".into());
    let workers: usize = args.get(4).and_then(|s| s.parse().ok()).unwrap_or_else(|| {
        std::thread::available_parallelism().map(|n| n.get().saturating_sub(2).max(1)).unwrap_or(4)
    });

    // Optional real/open BIOS (GBA_BIOS=<path>); empty = HLE direct-boot.
    let bios = std::env::var("GBA_BIOS").ok().and_then(|p| std::fs::read(p).ok()).unwrap_or_default();

    let mut roms = Vec::new();
    if root.extension().map(|e| e == "txt").unwrap_or(false) {
        // Listfile mode: one ROM path per line (lets us sweep an exact subset).
        if let Ok(f) = File::open(&root) {
            for line in BufReader::new(f).lines().map_while(Result::ok) {
                let p = line.trim();
                if !p.is_empty() {
                    roms.push(PathBuf::from(p));
                }
            }
        }
    } else {
        collect_gba(&root, &mut roms);
    }
    roms.sort();
    let total_found = roms.len();

    // Resume: skip ROMs already recorded (first TSV column is the path).
    let mut done = std::collections::HashSet::new();
    if let Ok(f) = File::open(&out_path) {
        for line in BufReader::new(f).lines().map_while(Result::ok) {
            if let Some(p) = line.split('\t').next() {
                done.insert(p.to_string());
            }
        }
    }
    let todo: Vec<PathBuf> = roms.into_iter().filter(|p| !done.contains(&p.to_string_lossy().to_string())).collect();
    eprintln!(
        "sweep: {} ROMs found, {} already done, {} to run, {} frames, {} workers, bios={}",
        total_found, done.len(), todo.len(), frames, workers,
        if bios.is_empty() { "HLE" } else { "yes" }
    );

    // Capture each panic's location + message (per worker thread) instead of
    // printing a backtrace, so the catch below can record it for clustering.
    thread_local! {
        static LAST_PANIC: std::cell::RefCell<String> = const { std::cell::RefCell::new(String::new()) };
    }
    panic::set_hook(Box::new(|info| {
        let loc = info.location().map(|l| format!("{}:{}", l.file(), l.line())).unwrap_or_default();
        let msg = info
            .payload()
            .downcast_ref::<&str>()
            .copied()
            .or_else(|| info.payload().downcast_ref::<String>().map(|s| s.as_str()))
            .unwrap_or("?");
        LAST_PANIC.with(|p| *p.borrow_mut() = format!("{} | {}", loc, msg));
    }));

    let writer = Mutex::new(OpenOptions::new().create(true).append(true).open(&out_path).expect("open out"));
    let next = AtomicUsize::new(0);
    let completed = AtomicUsize::new(0);
    let start = std::time::Instant::now();

    std::thread::scope(|s| {
        for _ in 0..workers {
            s.spawn(|| loop {
                let i = next.fetch_add(1, Ordering::Relaxed);
                if i >= todo.len() {
                    break;
                }
                let path = &todo[i];
                // Columns: path, status, peak_distinct, best_frame, steps, late_distinct, note
                let line = match std::fs::read(path) {
                    Err(e) => format!("{}\tREADERR\t0\t0\t0\t0\t{}\n", path.display(), e),
                    Ok(rom) => match panic::catch_unwind(AssertUnwindSafe(|| run_one(rom, &bios, frames))) {
                        Ok((d, bf, steps, late)) => {
                            format!("{}\tOK\t{}\t{}\t{}\t{}\t\n", path.display(), d, bf, steps, late)
                        }
                        Err(_) => {
                            let msg = LAST_PANIC.with(|p| p.borrow().clone());
                            format!("{}\tPANIC\t0\t0\t0\t0\t{}\n", path.display(), msg)
                        }
                    },
                };
                {
                    let mut w = writer.lock().unwrap();
                    let _ = w.write_all(line.as_bytes());
                    let _ = w.flush();
                }
                let c = completed.fetch_add(1, Ordering::Relaxed) + 1;
                if c % 100 == 0 {
                    let rate = c as f64 / start.elapsed().as_secs_f64();
                    eprintln!("  {}/{}  ({:.1} rom/s, ETA {:.0}s)", c, todo.len(), rate, (todo.len() - c) as f64 / rate);
                }
            });
        }
    });
    eprintln!("sweep done: {} ROMs in {:.0}s -> {}", todo.len(), start.elapsed().as_secs_f64(), out_path);
}
