pub mod sdr;
pub mod dsp {
    pub mod caf;
    pub mod cancel;
    pub mod decimate;
    pub mod fft;
    pub mod tropical;
}
pub mod math {
    pub mod adelic;
}
pub mod tracking {
    pub mod bank;
    pub mod ekf;
    pub mod jem;
}
pub mod db {
    pub mod flights;
    pub mod towers;
}
pub mod ui {
    pub mod dashboard;
}

use clap::Parser;
use crossterm::{
    cursor,
    event::{self, DisableMouseCapture, EnableMouseCapture, Event, KeyCode},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use num_complex::Complex;
use ratatui::{
    backend::{Backend, CrosstermBackend, TestBackend},
    Terminal,
};
use std::error::Error;
use std::io;
use std::time::{Duration, Instant};
use std::collections::VecDeque;
use rayon::prelude::*;

use db::towers::TowerDatabase;
use dsp::cancel::NlmsCanceler;
use dsp::decimate::DigitalDownConverter;
use dsp::fft::FftEngine;
use sdr::{SdrSource, SimulationSdrSource, SoapySdrSource};
use tracking::bank::TrackingBank;
use ui::dashboard::Dashboard;

#[derive(Parser, Debug)]
#[command(
    author,
    version,
    about = "Passive Radar (Forward-Scatter) DSP Pipeline"
)]
struct Args {
    /// Ingestion mode: 'sim' for simulated aircraft, 'sdr' for physical hardware
    #[arg(short, long, default_value = "sim")]
    mode: String,

    /// Target FM radio frequency in MHz (if omitted, the system will auto-tune to the optimal tower group)
    #[arg(short, long)]
    freq: Option<f64>,


    /// SDR input sample rate in MSPS (default 2.048 MHz)
    #[arg(short, long, default_value_t = 2.048)]
    rate: f64,

    /// Optional LNA gain in dB (defaults to 32.0)
    #[arg(long)]
    lna: Option<f64>,

    /// Optional VGA gain in dB (defaults to 30.0)
    #[arg(long)]
    vga: Option<f64>,


    /// Enable compatibility mode (ASCII borders, simpler canvas symbols) for terminals without full Unicode support
    #[arg(long, default_value_t = false)]
    compat: bool,

    /// Path to a test script text file for E2E testing
    #[arg(long)]
    test_script: Option<String>,

    /// Directory path where frame dumps are written
    #[arg(long)]
    test_out: Option<String>,

    /// Override terminal width for testing
    #[arg(long)]
    width: Option<u16>,

    /// Override terminal height for testing
    #[arg(long)]
    height: Option<u16>,

    /// Disable GPU-accelerated FFTs
    #[arg(long, default_value_t = false)]
    disable_gpu: bool,
}

#[derive(Debug, Clone)]
enum ScriptCommand {
    Key(String),
    Tick(usize),
    Dump(String),
}

enum AppBackend {
    Crossterm(CrosstermBackend<io::Stdout>),
    Test(TestBackend),
}

impl Backend for AppBackend {
    fn draw<'a, I>(&mut self, content: I) -> io::Result<()>
    where
        I: Iterator<Item = (u16, u16, &'a ratatui::buffer::Cell)>,
    {
        match self {
            AppBackend::Crossterm(b) => b.draw(content),
            AppBackend::Test(b) => b.draw(content),
        }
    }

    fn hide_cursor(&mut self) -> io::Result<()> {
        match self {
            AppBackend::Crossterm(b) => b.hide_cursor(),
            AppBackend::Test(b) => b.hide_cursor(),
        }
    }

    fn show_cursor(&mut self) -> io::Result<()> {
        match self {
            AppBackend::Crossterm(b) => b.show_cursor(),
            AppBackend::Test(b) => b.show_cursor(),
        }
    }

    fn get_cursor(&mut self) -> io::Result<(u16, u16)> {
        match self {
            AppBackend::Crossterm(b) => b.get_cursor(),
            AppBackend::Test(b) => b.get_cursor(),
        }
    }

    fn set_cursor(&mut self, x: u16, y: u16) -> io::Result<()> {
        match self {
            AppBackend::Crossterm(b) => b.set_cursor(x, y),
            AppBackend::Test(b) => b.set_cursor(x, y),
        }
    }

    fn clear(&mut self) -> io::Result<()> {
        match self {
            AppBackend::Crossterm(b) => b.clear(),
            AppBackend::Test(b) => b.clear(),
        }
    }

    fn size(&self) -> io::Result<ratatui::layout::Rect> {
        match self {
            AppBackend::Crossterm(b) => b.size(),
            AppBackend::Test(b) => b.size(),
        }
    }

    fn window_size(&mut self) -> io::Result<ratatui::backend::WindowSize> {
        match self {
            AppBackend::Crossterm(b) => b.window_size(),
            AppBackend::Test(b) => b.window_size(),
        }
    }

    fn flush(&mut self) -> io::Result<()> {
        match self {
            AppBackend::Crossterm(b) => b.flush(),
            AppBackend::Test(b) => b.flush(),
        }
    }
}

fn handle_key_code(
    code: KeyCode,
    paused: &std::sync::Arc<std::sync::atomic::AtomicBool>,
    speed_factor: &std::sync::Arc<std::sync::atomic::AtomicU32>,
    step_requested: &std::sync::Arc<std::sync::atomic::AtomicBool>,
    dashboard: &mut Dashboard,
    tracking_bank: &TrackingBank,
    should_exit: &mut bool,
) {
    match code {
        KeyCode::Char('q') | KeyCode::Char('Q') => {
            *should_exit = true;
        }
        KeyCode::Char(' ') => {
            let is_paused = paused.load(std::sync::atomic::Ordering::SeqCst);
            paused.store(!is_paused, std::sync::atomic::Ordering::SeqCst);
        }
        KeyCode::Char('+') | KeyCode::Char('=') => {
            let current_speed = speed_factor.load(std::sync::atomic::Ordering::SeqCst);
            let new_speed = (current_speed + 25).min(1000);
            speed_factor.store(new_speed, std::sync::atomic::Ordering::SeqCst);
        }
        KeyCode::Char('-') => {
            let current_speed = speed_factor.load(std::sync::atomic::Ordering::SeqCst);
            let new_speed = current_speed.saturating_sub(25).max(25);
            speed_factor.store(new_speed, std::sync::atomic::Ordering::SeqCst);
        }
        KeyCode::Char('s') | KeyCode::Char('S') => {
            if paused.load(std::sync::atomic::Ordering::SeqCst) {
                step_requested.store(true, std::sync::atomic::Ordering::SeqCst);
            }
        }
        KeyCode::Up => {
            let mut sorted_targets: Vec<&tracking::bank::TrackedTarget> = tracking_bank.targets.iter().collect();
            sorted_targets.sort_by_key(|t| {
                let s = match t.state {
                    crate::tracking::bank::TrackState::Active => 0u32,
                    crate::tracking::bank::TrackState::Coasting => 1,
                    crate::tracking::bank::TrackState::Suspect => 2,
                    crate::tracking::bank::TrackState::Terminated => 3,
                };
                (s, t.id)
            });
            // Apply same OFFLINE filter as the display
            sorted_targets.retain(|t| {
                if t.state == crate::tracking::bank::TrackState::Terminated {
                    return t.terminated_at.map_or(true, |ta| ta.elapsed().as_secs_f64() <= 30.0);
                }
                true
            });

            let num_targets = sorted_targets.len();
            if num_targets > 0 {
                let current_idx = dashboard.selected_target_id.and_then(|id| {
                    sorted_targets.iter().position(|t| t.id == id)
                });
                let next_idx = match current_idx {
                    Some(idx) => (idx + num_targets - 1) % num_targets,
                    None => num_targets - 1,
                };
                dashboard.selected_target_id = Some(sorted_targets[next_idx].id);
            }
        }
        KeyCode::Down => {
            let mut sorted_targets: Vec<&tracking::bank::TrackedTarget> = tracking_bank.targets.iter().collect();
            sorted_targets.sort_by_key(|t| {
                let s = match t.state {
                    crate::tracking::bank::TrackState::Active => 0u32,
                    crate::tracking::bank::TrackState::Coasting => 1,
                    crate::tracking::bank::TrackState::Suspect => 2,
                    crate::tracking::bank::TrackState::Terminated => 3,
                };
                (s, t.id)
            });
            // Apply same OFFLINE filter as the display
            sorted_targets.retain(|t| {
                if t.state == crate::tracking::bank::TrackState::Terminated {
                    return t.terminated_at.map_or(true, |ta| ta.elapsed().as_secs_f64() <= 30.0);
                }
                true
            });

            let num_targets = sorted_targets.len();
            if num_targets > 0 {
                let current_idx = dashboard.selected_target_id.and_then(|id| {
                    sorted_targets.iter().position(|t| t.id == id)
                });
                let next_idx = match current_idx {
                    Some(idx) => (idx + 1) % num_targets,
                    None => 0,
                };
                dashboard.selected_target_id = Some(sorted_targets[next_idx].id);
            }
        }
        KeyCode::Esc => {
            dashboard.selected_target_id = None;
        }
        KeyCode::Char('l') | KeyCode::Char('L') => {
            dashboard.visible_panels.logs = !dashboard.visible_panels.logs;
        }
        KeyCode::Char('t') | KeyCode::Char('T') => {
            dashboard.visible_panels.towers = !dashboard.visible_panels.towers;
        }
        KeyCode::Char('w') | KeyCode::Char('W') => {
            dashboard.visible_panels.waterfall = !dashboard.visible_panels.waterfall;
        }
        _ => {}
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.disable_gpu {
        crate::dsp::fft::DISABLE_GPU.store(true, std::sync::atomic::Ordering::SeqCst);
    }

    let input_rate = args.rate * 1e6;

    // 1. Initialize Tower Database and cross-reference geographic coordinates
    let db_path = "towers.json";
    println!("Loading Transmitter Tower Database...");
    let tower_db = TowerDatabase::load_or_create(db_path)?;

    // Determine target frequency (auto-tune if omitted)
    let target_freq = match args.freq {
        Some(f) => f * 1e6,
        None => {
            let (optimal_freq, optimal_towers) = tower_db.find_optimal_tuning(input_rate);
            println!(
                "Auto-Tuning: Identified optimal center frequency {:.3} MHz covering {} active tower(s)",
                optimal_freq / 1e6,
                optimal_towers.len()
            );
            optimal_freq
        }
    };

    // Find active towers within SDR bandwidth
    let mut active_towers = Vec::new();
    for tower in &tower_db.towers {
        let f_offset = tower.frequency_hz - target_freq;
        if f_offset.abs() <= input_rate / 2.0 {
            let enu = tower_db.get_tower_enu(tower);
            println!(
                "Match Found! Active Illuminator: {} (Callsign: {})",
                tower.name, tower.callsign
            );
            println!(
                "Coordinates: Lat={:.4}, Lon={:.4}, alt={:.1}m (ENU: East={:.1}km, North={:.1}km)",
                tower.latitude,
                tower.longitude,
                tower.elevation_m,
                enu[0] / 1000.0,
                enu[1] / 1000.0
            );
            active_towers.push((tower.clone(), enu));
        }
    }

    // Instantiate flight lookup engine (OpenSky API matching)
    let (flight_log_tx, flight_log_rx) = std::sync::mpsc::channel();
    let flight_engine = db::flights::FlightLookupEngine::new(
        tower_db.receiver.latitude,
        tower_db.receiver.longitude,
        tower_db.receiver.elevation_m,
        args.mode == "sim",
        flight_log_tx,
    );

    // 2. Instantiate selected SDR source
    let mut sdr: Box<dyn SdrSource> = match args.mode.as_str() {
        "sdr" => {
            println!("Initializing physical SoapySDR hardware source...");
            Box::new(SoapySdrSource::new(target_freq, input_rate, args.lna, args.vga))
        }
        _ => {
            println!("Initializing high-fidelity simulation source...");
            Box::new(SimulationSdrSource::new(target_freq, input_rate))
        }
    };

    sdr.start()?;

    // 3. Setup DSP Pipeline stages
    // Target baseband rate after 256x decimation (2.048 MSPS / 256 = 8 kHz)
    let decimation_factor = 256;
    let baseband_rate = input_rate / decimation_factor as f64;
    println!(
        "Decimating input stream: {:.3} MHz -> {:.1} kHz",
        input_rate / 1e6,
        baseband_rate / 1e3
    );

    struct TowerChannel {
        tower: db::towers::TransmitterTower,
        tower_pos: [f64; 3],
        ddc: DigitalDownConverter,
        dc_blocker: dsp::cancel::DcBlocker,
        clutter_filter: NlmsCanceler,
        fft_engine: FftEngine,
        wavelet_canceller: dsp::tropical::TropicalWaveletCanceller,
        decimated_buf: Vec<Complex<f32>>,
        dc_blocked_buf: Vec<Complex<f32>>,
        cancelled_buf: Vec<Complex<f32>>,
    }

    let fft_size = 8192;
    let fft_step = 1024;

    let mut channels = Vec::new();
    for (tower, enu) in active_towers {
        let offset = (tower.frequency_hz - target_freq) + 75.0;
        channels.push(TowerChannel {
            tower: tower.clone(),
            tower_pos: enu,
            ddc: DigitalDownConverter::new(offset, input_rate),
            dc_blocker: dsp::cancel::DcBlocker::new(0.99),
            clutter_filter: NlmsCanceler::new(32, 0.05, 8),
            fft_engine: FftEngine::new(fft_size),
            wavelet_canceller: dsp::tropical::TropicalWaveletCanceller::new(fft_size),
            decimated_buf: Vec::with_capacity(2048),
            dc_blocked_buf: Vec::with_capacity(2048),
            cancelled_buf: Vec::with_capacity(2048),
        });
    }

    if channels.is_empty() {
        println!("No towers in database found in bandwidth. Falling back to default tuned channel at {:.1} MHz", target_freq / 1e6);
        channels.push(TowerChannel {
            tower: db::towers::TransmitterTower {
                name: "Default Tuned Channel".to_string(),
                callsign: "DFLT".to_string(),
                frequency_hz: target_freq,
                latitude: tower_db.receiver.latitude,
                longitude: tower_db.receiver.longitude,
                elevation_m: tower_db.receiver.elevation_m,
                erp_watts: 50_000.0,
            },
            tower_pos: [0.0, 0.0, 0.0],
            ddc: DigitalDownConverter::new(75.0, input_rate),
            dc_blocker: dsp::cancel::DcBlocker::new(0.99),
            clutter_filter: NlmsCanceler::new(32, 0.05, 8),
            fft_engine: FftEngine::new(fft_size),
            wavelet_canceller: dsp::tropical::TropicalWaveletCanceller::new(fft_size),
            decimated_buf: Vec::with_capacity(2048),
            dc_blocked_buf: Vec::with_capacity(2048),
            cancelled_buf: Vec::with_capacity(2048),
        });
    }

    let tower_name = channels[0].tower.name.clone();
    let tower_pos = channels[0].tower_pos;

    // 4. Setup EKF Tracking Bank and UI Dashboard
    let mut tracking_bank = TrackingBank::new();
    tracking_bank.mode = args.mode.clone();
    tracking_bank.load_disk_fingerprints();
    let mut dashboard = Dashboard::new(target_freq, input_rate, 75.0, args.mode.clone());
    dashboard.tower_name = tower_name;
    dashboard.tower_pos = tower_pos;
    dashboard.active_towers = channels
        .iter()
        .map(|c| (c.tower.callsign.clone(), c.tower_pos))
        .collect();

    // Auto-detect compatibility mode if terminal locale is not UTF-8
    let mut compat_mode = args.compat;
    if !compat_mode {
        let lang = std::env::var("LANG").unwrap_or_default().to_lowercase();
        let lc_all = std::env::var("LC_ALL").unwrap_or_default().to_lowercase();
        if (!lang.is_empty() && !lang.contains("utf-8") && !lang.contains("utf8"))
            || (!lc_all.is_empty() && !lc_all.contains("utf-8") && !lc_all.contains("utf8"))
        {
            compat_mode = true;
        }
    }
    dashboard.compat_mode = compat_mode;

    let test_mode = args.test_script.is_some();

    let mut commands = Vec::new();
    if let Some(ref script_path) = args.test_script {
        let content = std::fs::read_to_string(script_path)?;
        for line in content.lines() {
            let line = line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }
            let parts: Vec<&str> = line.splitn(2, ' ').collect();
            if parts.len() == 2 {
                let cmd = parts[0].to_uppercase();
                let arg = parts[1].trim();
                match cmd.as_str() {
                    "KEY" => commands.push(ScriptCommand::Key(arg.to_string())),
                    "TICK" => {
                        if let Ok(count) = arg.parse::<usize>() {
                            commands.push(ScriptCommand::Tick(count));
                        }
                    }
                    "DUMP" => commands.push(ScriptCommand::Dump(arg.to_string())),
                    _ => {}
                }
            }
        }
    }

    // 5. Initialize Terminal raw screen for Ratatui
    // Redirect stderr to /dev/null during TUI to prevent C library debug output
    // (SoapySDR, FFTW, etc.) from bleeding through and corrupting the display.
    let saved_stderr = if !test_mode {
        use std::os::unix::io::AsRawFd;
        let saved = unsafe { libc::dup(2) };
        let devnull = std::fs::File::open("/dev/null").ok();
        if let Some(ref dn) = devnull {
            unsafe { libc::dup2(dn.as_raw_fd(), 2) };
        }
        Some(saved)
    } else {
        None
    };

    let backend = if test_mode {
        AppBackend::Test(TestBackend::new(args.width.unwrap_or(120), args.height.unwrap_or(40)))
    } else {
        enable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(stdout, EnterAlternateScreen, EnableMouseCapture, cursor::Hide)?;
        AppBackend::Crossterm(CrosstermBackend::new(stdout))
    };
    let mut terminal = Terminal::new(backend)?;

    // 6. Main thread loops with background SDR Ingestion Thread
    let (sdr_tx, sdr_rx) = std::sync::mpsc::sync_channel::<Vec<Complex<f32>>>(64);
    let mut sdr_source = sdr;
    let sdr_tx_clone = sdr_tx.clone();
    let sdr_alive = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let sdr_alive_clone = sdr_alive.clone();

    // Initialize thread-safe control states
    let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let speed_factor = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(100)); // speed * 100
    let step_requested = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));

    let paused_clone = paused.clone();
    let speed_factor_clone = speed_factor.clone();
    let step_requested_clone = step_requested.clone();

    // Spawn ingestion thread
    std::thread::spawn(move || {
        let mut local_buf = vec![Complex::new(0.0, 0.0); 16384];
        let mut last_speed = -1.0;
        loop {
            // Read paused/speed state
            let is_paused = paused_clone.load(std::sync::atomic::Ordering::SeqCst);
            let speed_val = if test_mode {
                100000.0
            } else {
                speed_factor_clone.load(std::sync::atomic::Ordering::SeqCst) as f64 / 100.0
            };
            
            if speed_val != last_speed {
                let _ = sdr_source.set_speed_factor(speed_val);
                last_speed = speed_val;
            }

            if is_paused {
                if step_requested_clone.load(std::sync::atomic::Ordering::SeqCst) {
                    step_requested_clone.store(false, std::sync::atomic::Ordering::SeqCst);
                    match sdr_source.read(&mut local_buf) {
                        Ok(len) => {
                            if len > 0 && sdr_tx_clone.send(local_buf[0..len].to_vec()).is_err() {
                                break;
                            }
                        }
                        Err(_) => {
                            sdr_alive_clone.store(false, std::sync::atomic::Ordering::SeqCst);
                            break;
                        }
                    }
                } else {
                    std::thread::sleep(Duration::from_millis(10));
                }
            } else {
                match sdr_source.read(&mut local_buf) {
                    Ok(len) => {
                        if len > 0 && sdr_tx_clone.send(local_buf[0..len].to_vec()).is_err() {
                            break; // Main thread exited
                        }
                    }
                    Err(_) => {
                        sdr_alive_clone.store(false, std::sync::atomic::Ordering::SeqCst);
                        break;
                    }
                }
            }
        }
        let _ = sdr_source.stop();
        sdr_alive_clone.store(false, std::sync::atomic::Ordering::SeqCst);
    });

    let mut last_ui_render = Instant::now();
    let startup_time = Instant::now();

    let mut script_idx = 0;
    let mut current_tick_remaining = 0;
    let mut should_exit_after_render = false;

    let mut primary_ref_buf: VecDeque<Complex<f32>> = VecDeque::new();
    let mut primary_surv_buf: VecDeque<Complex<f32>> = VecDeque::new();
    let mut caf_engine = crate::dsp::caf::CafEngine::new(fft_size);
    let mut caf_frame_counter: u32 = 0;

    'main_loop: loop {
        let mut pending_dump = None;

        if test_mode {
            while script_idx < commands.len() && current_tick_remaining == 0 {
                match &commands[script_idx] {
                    ScriptCommand::Key(key_name) => {
                        let key_code = match key_name.to_lowercase().as_str() {
                            "space" => KeyCode::Char(' '),
                            "+" | "=" => KeyCode::Char('+'),
                            "-" => KeyCode::Char('-'),
                            "up" => KeyCode::Up,
                            "down" => KeyCode::Down,
                            "esc" => KeyCode::Esc,
                            "l" => KeyCode::Char('l'),
                            "t" => KeyCode::Char('t'),
                            "w" => KeyCode::Char('w'),
                            "s" => KeyCode::Char('s'),
                            "q" => KeyCode::Char('q'),
                            other => {
                                if other.len() == 1 {
                                    KeyCode::Char(other.chars().next().unwrap())
                                } else {
                                    KeyCode::Null
                                }
                            }
                        };
                        let mut should_exit = false;
                        handle_key_code(
                            key_code,
                            &paused,
                            &speed_factor,
                            &step_requested,
                            &mut dashboard,
                            &tracking_bank,
                            &mut should_exit,
                        );
                        if should_exit {
                            should_exit_after_render = true;
                        }
                        script_idx += 1;
                    }
                    ScriptCommand::Dump(filename) => {
                        pending_dump = Some(filename.clone());
                        script_idx += 1;
                    }
                    ScriptCommand::Tick(count) => {
                        current_tick_remaining = *count;
                        script_idx += 1;
                    }
                }
            }

            if script_idx >= commands.len() && current_tick_remaining == 0 {
                should_exit_after_render = true;
            }

            current_tick_remaining = current_tick_remaining.saturating_sub(1);
        } else {
            // Handle keystrokes (non-blocking poll)
            if event::poll(Duration::from_millis(1))? {
                if let Event::Key(key) = event::read()? {
                    let mut should_exit = false;
                    handle_key_code(
                        key.code,
                        &paused,
                        &speed_factor,
                        &step_requested,
                        &mut dashboard,
                        &tracking_bank,
                        &mut should_exit,
                    );
                    if should_exit {
                        break 'main_loop;
                    }
                }
            }
        }

        // Process all queued SDR blocks (limit to at most 8 blocks per iteration to prevent UI starvation/freeze)
        let mut got_data = false;
        let mut processed_blocks = 0;
        let is_paused_val = paused.load(std::sync::atomic::Ordering::SeqCst);
        let has_step_val = step_requested.load(std::sync::atomic::Ordering::SeqCst);

        while processed_blocks < 64 {
            let block = if test_mode && (!is_paused_val || has_step_val) {
                match sdr_rx.recv_timeout(Duration::from_millis(5000)) {
                    Ok(b) => b,
                    Err(_) => break,
                }
            } else if !test_mode && processed_blocks == 0 {
                match sdr_rx.recv_timeout(Duration::from_millis(500)) {
                    Ok(b) => b,
                    Err(_) => break,
                }
            } else {
                match sdr_rx.try_recv() {
                    Ok(b) => b,
                    Err(_) => break,
                }
            };
            got_data = true;
            processed_blocks += 1;

            let clip_count = block
                .iter()
                .filter(|c| c.re.abs() >= 0.99 || c.im.abs() >= 0.99)
                .count();
            let block_clipping_rate = clip_count as f32 / block.len() as f32;
            dashboard.clipping_rate = 0.9 * dashboard.clipping_rate + 0.1 * block_clipping_rate;

            let results: Vec<(f32, f32, f32)> = channels
                .par_iter_mut()
                .map(|chan| {
                    chan.ddc.process_block(&block, &mut chan.decimated_buf);
                    chan.dc_blocker.process_block(&chan.decimated_buf, &mut chan.dc_blocked_buf);
                    chan.clutter_filter.process_block(&chan.dc_blocked_buf, &mut chan.cancelled_buf);
                    
                    let p_in: f32 = chan.dc_blocked_buf.iter().map(|c| c.norm_sqr()).sum();
                    let p_out: f32 = chan.cancelled_buf.iter().map(|c| c.norm_sqr()).sum();
                    let decimated_len = chan.decimated_buf.len() as f32;

                    chan.fft_engine.feed(&chan.cancelled_buf);

                    (p_in, p_out, decimated_len)
                })
                .collect();

            let mut primary_cancelled = Vec::new();
            if !results.is_empty() {
                let (p_in, p_out, decimated_len) = results[0];
                primary_cancelled = channels[0].cancelled_buf.clone();
                let block_carrier_rms = (p_in / decimated_len).sqrt();
                let block_cancel_db = 10.0 * (p_in / p_out.max(1e-10)).log10();
                
                dashboard.carrier_rms = 0.9 * dashboard.carrier_rms + 0.1 * block_carrier_rms;
                dashboard.cancellation_ratio_db = 0.9 * dashboard.cancellation_ratio_db + 0.1 * block_cancel_db;

                primary_ref_buf.extend(channels[0].dc_blocked_buf.iter());
                primary_surv_buf.extend(channels[0].cancelled_buf.iter());
            }

            // Pull overlap FFT frames if available in lockstep
            while channels.iter().all(|chan| chan.fft_engine.has_frame()) {
                // Compute time delta since last update for EKF tracking propagation using constant sample-based delta
                let dt = fft_step as f64 / baseband_rate;

                let primary_freq = channels[0].tower.frequency_hz;

                // Compute CAF matrix every 4th FFT frame (~2-4 Hz) to save 40 FFTs/frame
                caf_frame_counter += 1;
                if primary_ref_buf.len() >= fft_size && primary_surv_buf.len() >= fft_size && (caf_frame_counter % 4 == 0) {
                    let max_delay = 40;
                    // make_contiguous() ensures we can slice the VecDeque
                    let ref_slice = primary_ref_buf.make_contiguous();
                    let surv_slice = primary_surv_buf.make_contiguous();
                    let caf_res = caf_engine.compute(
                        &ref_slice[0..fft_size],
                        &surv_slice[0..fft_size],
                        max_delay,
                    );
                    dashboard.update_caf(caf_res);
                }

                // Pre-collect active targets EKF states to avoid thread-borrowing issues in par_iter_mut
                let active_targets: Vec<[f64; 6]> = tracking_bank
                    .targets
                    .iter()
                    .filter(|t| t.state != tracking::bank::TrackState::Terminated)
                    .map(|t| t.ekf.state)
                    .collect();

                // Parallelize FFT extraction, notching, and peak detection across channels
                let fft_results: Vec<(Vec<(f32, f32)>, Option<Vec<f32>>)> = channels
                    .par_iter_mut()
                    .map(|chan| {
                        if let Some(magnitude) = chan.fft_engine.next_frame(fft_step) {
                            let n_bins = magnitude.len();
                            let mut active_dopplers = Vec::new();
                            let lambda = crate::sdr::C / chan.tower.frequency_hz;

                            for state in &active_targets {
                                let x = state[0];
                                let y = state[1];
                                let z = state[2];
                                let vx = state[3];
                                let vy = state[4];
                                let vz = state[5];

                                let dx = x - chan.tower_pos[0];
                                let dy = y - chan.tower_pos[1];
                                let dz = z - chan.tower_pos[2];
                                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                                let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

                                let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
                                let dot_r = (vx * x + vy * y + vz * z) / r_r;
                                let pred_doppler = -(dot_t + dot_r) / lambda;
                                active_dopplers.push(pred_doppler);
                            }

                            // Apply non-linear Tropical Wavelet spur notching to magnitudes
                            // notch_stationary_spurs returns the background so we don't recompute it
                            let mut magnitude_clean = magnitude.clone();
                            let background = chan.wavelet_canceller.notch_stationary_spurs(
                                &mut magnitude_clean,
                                &active_dopplers,
                                baseband_rate,
                            );

                            // Scan for local maxima relative to the local background envelope
                            let mut peaks = Vec::new();
                            for i in 5..(n_bins - 5) {
                                let f = (i as f64 - n_bins as f64 / 2.0) * (baseband_rate / n_bins as f64);
                                // Filter out zero-Doppler zenith/DC clutter leakage (+/- 8 Hz)
                                if f.abs() < 8.0 {
                                    continue;
                                }

                                let val = magnitude_clean[i];
                                let bg_val = background[i].max(1e-6);
                                // Check if local peak
                                if val > magnitude_clean[i - 1]
                                    && val > magnitude_clean[i + 1]
                                    && val > magnitude_clean[i - 2]
                                    && val > magnitude_clean[i + 2]
                                {
                                    // Check peak threshold (e.g. 5.8x local background noise floor)
                                    if val > bg_val * 5.8 {
                                        let snr_db = (val / bg_val).max(1.0).log10() * 10.0;
                                        peaks.push((f as f32, snr_db));
                                    }
                                }
                            }

                            // Sort by SNR descending and truncate to top 15 peaks to cap workload
                            peaks.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
                            peaks.truncate(30);

                            let mut db_magnitude = None;
                            // If this is the primary channel, calculate DB magnitude
                            if chan.tower.frequency_hz == primary_freq {
                                let mut db_mag = vec![0.0; n_bins];
                                for i in 0..n_bins {
                                    let bg_val = background[i].max(1e-6);
                                    db_mag[i] = (magnitude_clean[i] / bg_val).max(1.0).log10() * 10.0;
                                }
                                db_magnitude = Some(db_mag);
                            }

                            (peaks, db_magnitude)
                        } else {
                            (Vec::new(), None)
                        }
                    })
                    .collect();

                let mut tower_peaks_store = vec![Vec::new(); channels.len()];
                let mut primary_db_magnitude = None;

                for (idx, (peaks, db_mag_opt)) in fft_results.into_iter().enumerate() {
                    tower_peaks_store[idx] = peaks;
                    if let Some(db_mag) = db_mag_opt {
                        primary_db_magnitude = Some(db_mag);
                    }
                }

                if let Some(db_mag) = primary_db_magnitude {
                    dashboard.add_spectrum(db_mag);
                }

                // Drain processed samples from the primary reference/surveillance buffers
                // VecDeque::drain at front is O(drained) not O(remaining)
                if primary_ref_buf.len() >= fft_step {
                    primary_ref_buf.drain(0..fft_step);
                }
                if primary_surv_buf.len() >= fft_step {
                    primary_surv_buf.drain(0..fft_step);
                }

                // Construct towers_data slice parameter
                let mut towers_data = Vec::new();
                for (idx, chan) in channels.iter().enumerate() {
                    if idx < tower_peaks_store.len() {
                        towers_data.push((
                            chan.tower.name.clone(),
                            chan.tower_pos,
                            chan.tower.frequency_hz,
                            &tower_peaks_store[idx][..],
                        ));
                    }
                }

                // Feed detected peaks into Extended Kalman Filter Bank
                let mut event_logs = Vec::new();
                let is_warmed_up = args.mode != "sdr" || startup_time.elapsed().as_secs_f64() > 2.0;
                if is_warmed_up {
                    tracking_bank.update_multitower(
                        &towers_data,
                        dt,
                        &primary_cancelled,
                        &mut event_logs,
                    );
                    for log in event_logs {
                        dashboard.add_log(log);
                    }
                } else {
                    let elapsed = startup_time.elapsed().as_secs_f64();
                    // Throttle log insertion to avoid duplicate prints in the 100Hz loop
                    if event_logs.is_empty() && ((elapsed * 2.0) as usize).is_multiple_of(2) {
                        let log_msg = format!(
                            "Warmup: NLMS clutter filters converging... ({:.1}s/2.0s)",
                            elapsed
                        );
                        if dashboard.logs.last() != Some(&log_msg) {
                            dashboard.add_log(log_msg);
                        }
                    }
                }

                // Cross-reference trajectories with flight data
                for target in &mut tracking_bank.targets {
                    if target.state == tracking::bank::TrackState::Active {
                        if let Some(flight) =
                            flight_engine.match_flight(&target.ekf.state, args.mode == "sim")
                        {
                            target.classification = flight;
                        }
                    }
                }
            }
        }

        // Render dashboard UI at ~30 FPS (33ms) to support decoupled refresh rates
        if test_mode || last_ui_render.elapsed() >= Duration::from_millis(33) {
            // Drain background flight lookup logs
            while let Ok(log) = flight_log_rx.try_recv() {
                dashboard.add_log(log);
            }
            dashboard.sdr_alive = sdr_alive.load(std::sync::atomic::Ordering::SeqCst);
            dashboard.paused = paused.load(std::sync::atomic::Ordering::SeqCst);
            dashboard.speed_factor = speed_factor.load(std::sync::atomic::Ordering::SeqCst) as f64 / 100.0;
            dashboard.candidates = tracking_bank.candidates.clone();
            // Auto-clear selection if the target no longer exists in the bank
            if let Some(sel_id) = dashboard.selected_target_id {
                if !tracking_bank.targets.iter().any(|t| t.id == sel_id) {
                    dashboard.selected_target_id = None;
                }
            }
            terminal.draw(|f| {
                let size = f.size();
                dashboard.render(f, size, &tracking_bank.targets, &tracking_bank.transients);
            })?;
            last_ui_render = Instant::now();
        }

        if let Some(filename) = pending_dump {
            if let AppBackend::Test(ref tb) = terminal.backend() {
                let buffer = tb.buffer();
                let mut rendered_text = String::new();
                for y in 0..buffer.area.height {
                    for x in 0..buffer.area.width {
                        let cell = buffer.get(x, y);
                        rendered_text.push_str(cell.symbol());
                    }
                    rendered_text.push('\n');
                }
                if let Some(ref out_dir) = args.test_out {
                    std::fs::create_dir_all(out_dir)?;
                    let dump_path = std::path::Path::new(out_dir).join(&filename);
                    std::fs::write(&dump_path, &rendered_text)?;
                }
            }
        }

        if should_exit_after_render {
            break 'main_loop;
        }

        // Throttle loop CPU usage if no data was received
        if !got_data {
            std::thread::sleep(Duration::from_millis(1));
        }
    }

    // 7. Cleanup raw terminal mode and restore screen
    if !test_mode {
        disable_raw_mode()?;
        let mut stdout = io::stdout();
        execute!(
            stdout,
            LeaveAlternateScreen,
            DisableMouseCapture,
            cursor::Show
        )?;

        // Restore stderr so post-TUI output works normally
        if let Some(saved_fd) = saved_stderr {
            unsafe { libc::dup2(saved_fd, 2) };
            unsafe { libc::close(saved_fd) };
        }
    }

    println!("Passive Radar shutdown complete.");
    Ok(())
}
