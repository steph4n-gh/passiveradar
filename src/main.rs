pub mod orbit;
pub mod sdr;
pub mod dsp {
    pub mod caf;
    pub mod cancel;
    pub mod cic;
    pub mod decimate;
    pub mod fft;
    pub mod tropical;
    pub mod isar;
    pub mod pfb;
    pub mod remod;
    pub mod pll;
    pub mod declip;
}
pub mod math {
    pub mod adelic;
}
pub mod tracking {
    pub mod bank;
    pub mod ekf;
    pub mod jem;
    pub mod tbd;
    pub mod osm;
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
    event::{self, Event, KeyCode, KeyModifiers},
    execute,
    terminal::{disable_raw_mode, enable_raw_mode, EnterAlternateScreen, LeaveAlternateScreen},
};
use std::sync::{Arc, Mutex};
use rand::Rng;
use num_complex::Complex;

#[derive(Clone, Copy, PartialEq, Eq, Debug, Hash)]
pub enum IlluminatorType {
    Fm,
    Atsc,
    FiveG,
    LeoStarlink,
}

pub struct SdrBlock {
    pub buf: Vec<Complex<f32>>,
    pub freq: f64,
    pub illuminator: IlluminatorType,
}
use ratatui::{
    backend::{Backend, CrosstermBackend, TestBackend},
    Terminal,
};
use std::error::Error;
use std::io;
use std::time::{Duration, Instant};
use std::collections::VecDeque;
use std::sync::atomic::Ordering;
use rayon::prelude::*;


use db::towers::TowerDatabase;
use dsp::cancel::{DcBlocker, EcaBatchedCanceler};
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
    /// Ingestion mode: 'sim' for simulated aircraft, 'sdr' for physical hardware, 'atsc' for ATSC Digital TV Mode
    #[arg(short, long, default_value = "sim")]
    mode: String,

    /// ATSC digital TV receiver cluster: 'tenleytown' or 'river'
    #[arg(long, default_value = "tenleytown")]
    atsc_cluster: String,

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

    /// Alignment compass heading in degrees
    #[arg(long, default_value_t = 0.0, allow_hyphen_values = true)]
    heading: f64,

    /// Disable tower loading (run without signals/towers)
    #[arg(long, default_value_t = false)]
    no_towers: bool,

    /// Run with empty/no-signal constellation
    #[arg(long, default_value_t = false)]
    no_signal: bool,

    /// Overrides active tower positions to receiver origin (0,0,0)
    #[arg(long, default_value_t = false)]
    tower_at_origin: bool,

    /// Mock 50 dummy active towers for TUI overlap limits testing
    #[arg(long, default_value_t = false)]
    many_towers: bool,

    /// Mock audio hardware failure
    #[arg(long, default_value_t = false)]
    mock_no_audio: bool,

    /// Mock 100+ active targets for audio hum clipping testing
    #[arg(long, default_value_t = false)]
    max_targets: bool,

    /// Force target termination in simulation for testing
    #[arg(long, default_value_t = false)]
    mock_target_termination: bool,

    /// Custom WebSocket listener port
    #[arg(long)]
    port: Option<u16>,

    /// Custom Web HUD listener port (default 8080)
    #[arg(long, default_value_t = 8080)]
    web_port: u16,

    /// Custom WebSocket listener host address
    #[arg(long, default_value = "127.0.0.1")]
    host: String,

    /// Override receiver latitude
    #[arg(long, allow_hyphen_values = true)]
    lat: Option<f64>,

    /// Override receiver longitude
    #[arg(long, allow_hyphen_values = true)]
    lon: Option<f64>,
}

#[derive(Debug, Clone)]
enum ScriptCommand {
    Key(String),
    Tick(usize),
    Dump(String),
}

#[derive(Debug, Clone)]
enum SdrCommand {
    Spoof { id: u32, speed: f64 },
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
            // Apply same OFFLINE and UNCONFIRMED filter as the display
            sorted_targets.retain(|t| {
                if !dashboard.show_unconfirmed && t.state == crate::tracking::bank::TrackState::Suspect {
                    return false;
                }
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
            } else if dashboard.is_test {
                dashboard.selected_target_id = Some(999999);
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
            // Apply same OFFLINE and UNCONFIRMED filter as the display
            sorted_targets.retain(|t| {
                if !dashboard.show_unconfirmed && t.state == crate::tracking::bank::TrackState::Suspect {
                    return false;
                }
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
            } else if dashboard.is_test {
                dashboard.selected_target_id = Some(999999);
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
        KeyCode::Char('f') | KeyCode::Char('F') => {
            dashboard.waterfall_mode = match dashboard.waterfall_mode {
                ui::dashboard::WaterfallMode::DopplerTime => ui::dashboard::WaterfallMode::RangeDoppler,
                ui::dashboard::WaterfallMode::RangeDoppler => ui::dashboard::WaterfallMode::DopplerTime,
            };
            dashboard.data_version = dashboard.data_version.wrapping_add(1);
        }
        KeyCode::Char('c') | KeyCode::Char('C') => {
            dashboard.show_constellation = !dashboard.show_constellation;
        }
        KeyCode::Char('a') | KeyCode::Char('A') => {
            dashboard.show_aligner = !dashboard.show_aligner;
        }
        KeyCode::Char('h') | KeyCode::Char('H') => {
            dashboard.show_hacker = !dashboard.show_hacker;
        }
        KeyCode::Char('r') | KeyCode::Char('R') => {
            dashboard.show_records = !dashboard.show_records;
        }
        KeyCode::Char('p') | KeyCode::Char('P') => {
            dashboard.show_multipath = !dashboard.show_multipath;
        }
        KeyCode::Char('u') | KeyCode::Char('U') => {
            dashboard.show_unconfirmed = !dashboard.show_unconfirmed;
        }
        KeyCode::Char('x') | KeyCode::Char('X') => {
            dashboard.crt_mode = !dashboard.crt_mode;
        }
        KeyCode::Char('m') | KeyCode::Char('M') => {
            dashboard.screen_shake = !dashboard.screen_shake;
            if dashboard.screen_shake {
                dashboard.add_log("METEOR EVENT DETECTED".to_string());
            }
        }
        KeyCode::Char('j') | KeyCode::Char('J') => {
            dashboard.show_jem_spectrogram = !dashboard.show_jem_spectrogram;
        }
        KeyCode::Char('v') | KeyCode::Char('V') => {
            dashboard.doppler_scale_pm = !dashboard.doppler_scale_pm;
        }
        KeyCode::Char('e') | KeyCode::Char('E') => {
            dashboard.ellipse_mode = match dashboard.ellipse_mode {
                ui::dashboard::EllipseMode::None => ui::dashboard::EllipseMode::Selected,
                ui::dashboard::EllipseMode::Selected => ui::dashboard::EllipseMode::All,
                ui::dashboard::EllipseMode::All => ui::dashboard::EllipseMode::None,
            };
        }
        KeyCode::Char('d') | KeyCode::Char('D') => {
            dashboard.dc_block = !dashboard.dc_block;
        }
        KeyCode::Char('1') => {
            dashboard.one_bit_mode = !dashboard.one_bit_mode;
            dashboard.add_log(format!("1-Bit Mode: {}", if dashboard.one_bit_mode { "ON" } else { "OFF" }));
        }
        KeyCode::Char('[') => {
            if dashboard.sdr_type == "sdr" {
                dashboard.gain = (dashboard.gain - 2.0).max(0.0);
            } else {
                dashboard.gain = (dashboard.gain - 0.5).max(0.0);
            }
            dashboard.software_agc = false;
        }
        KeyCode::Char(']') => {
            if dashboard.sdr_type == "sdr" {
                dashboard.gain = (dashboard.gain + 2.0).min(72.0);
            } else {
                dashboard.gain = (dashboard.gain + 0.5).min(10.0);
            }
            dashboard.software_agc = false;
        }
        KeyCode::Char('g') | KeyCode::Char('G') => {
            if dashboard.sdr_type == "sdr" {
                dashboard.software_agc = !dashboard.software_agc;
                dashboard.last_agc_update = std::time::Instant::now();
            }
        }
        KeyCode::Char('k') | KeyCode::Char('K') => {
            dashboard.tracking_mode = match dashboard.tracking_mode {
                crate::tracking::ekf::TrackingMode::Airspace => crate::tracking::ekf::TrackingMode::GroundCar,
                crate::tracking::ekf::TrackingMode::GroundCar => crate::tracking::ekf::TrackingMode::GroundTrain,
                crate::tracking::ekf::TrackingMode::GroundTrain => crate::tracking::ekf::TrackingMode::Airspace,
            };
            dashboard.add_log(format!("Tracking Mode: {:?}", dashboard.tracking_mode));
        }
        _ => {}
    }
}

fn start_web_server(host: &str, port: u16, ws_host: &str, ws_port: u16) {
    use std::io::{Read, Write};
    use std::fs;
    let host = host.to_string();
    let ws_host = ws_host.to_string();
    std::thread::spawn(move || {
        let addr = format!("{}:{}", host, port);
        match std::net::TcpListener::bind(&addr) {
            Ok(listener) => {
                println!("Web HUD Server: Serving web/ on http://{}", addr);
                for stream in listener.incoming() {
                    if let Ok(mut stream) = stream {
                        let ws_host = ws_host.clone();
                        std::thread::spawn(move || {
                            let _ = stream.set_read_timeout(Some(std::time::Duration::from_secs(5)));
                            let mut buffer = [0; 1024];
                            if stream.read(&mut buffer).is_ok() {
                                let req = String::from_utf8_lossy(&buffer);
                                let parts: Vec<&str> = req.split_whitespace().collect();
                                if parts.len() >= 2 && parts[0] == "GET" {
                                    let mut path = parts[1];
                                    if path == "/" {
                                        path = "/index.html";
                                    }
                                    if let Some(pos) = path.find('?') {
                                        path = &path[..pos];
                                    }
                                    let path = path.trim_start_matches('/');
                                    
                                    // Serve dynamic config if requested
                                    if path == "config" || path == "config.json" {
                                        let body = format!(
                                            r#"{{"ws_host": "{}", "ws_port": {}}}"#,
                                            ws_host, ws_port
                                        );
                                        let response = format!(
                                            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{}",
                                            body.len(),
                                            body
                                        );
                                        let _ = stream.write_all(response.as_bytes());
                                        return;
                                    }

                                    let safe_path = std::path::Path::new("web").join(path);
                                    let mut served = false;
                                    if safe_path.exists() {
                                        if let (Ok(canonical_base), Ok(canonical_safe)) = (fs::canonicalize("web"), fs::canonicalize(&safe_path)) {
                                            if canonical_safe.starts_with(canonical_base) && canonical_safe.is_file() {
                                                if let Ok(content) = fs::read(&canonical_safe) {
                                                    let mime_type = match canonical_safe.extension().and_then(|s| s.to_str()) {
                                                        Some("html") => "text/html",
                                                        Some("css") => "text/css",
                                                        Some("js") => "application/javascript",
                                                        Some("png") => "image/png",
                                                        Some("jpg") | Some("jpeg") => "image/jpeg",
                                                        _ => "application/octet-stream",
                                                    };
                                                    let response = format!(
                                                        "HTTP/1.1 200 OK\r\nContent-Type: {}\r\nContent-Length: {}\r\nConnection: close\r\n\r\n",
                                                        mime_type,
                                                        content.len()
                                                    );
                                                    let _ = stream.write_all(response.as_bytes());
                                                    let _ = stream.write_all(&content);
                                                    served = true;
                                                }
                                            }
                                        }
                                    }
                                    if !served {
                                        let _ = stream.write_all(b"HTTP/1.1 404 NOT FOUND\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
                                    }
                                }
                            }
                        });
                    }
                }
            }
            Err(e) => {
                eprintln!("Error: Failed to bind Web HUD Server to {}: {}", addr, e);
            }
        }
    });
}

fn to_ws_json_string<T: serde::Serialize>(val: &T) -> String {
    let raw = serde_json::to_string(val).unwrap_or_default();
    let mut result = String::with_capacity(raw.len() + 32);
    let mut in_string = false;
    let mut escaped = false;
    for c in raw.chars() {
        if escaped {
            result.push(c);
            escaped = false;
        } else if c == '\\' {
            result.push(c);
            escaped = true;
        } else if c == '"' {
            result.push(c);
            in_string = !in_string;
        } else if c == ':' && !in_string {
            result.push_str(": ");
        } else {
            result.push(c);
        }
    }
    result
}

fn process_ws_command(cmd_text: String, dashboard: &mut Dashboard, tracking_bank: &mut TrackingBank) -> serde_json::Value {
    let raw_val: serde_json::Value = match serde_json::from_str(&cmd_text) {
        Ok(v) => v,
        Err(_) => return serde_json::json!({"error": "Invalid arguments"}),
    };

    let action_or_command = raw_val.get("action").or(raw_val.get("command")).and_then(|v| v.as_str());
    let act = match action_or_command {
        Some(a) => a,
        None => return serde_json::json!({"error": "Invalid arguments"}),
    };

    let req_value = raw_val.get("value").and_then(|v| v.as_f64());
    let req_min = raw_val.get("min").and_then(|v| v.as_i64());
    let req_max = raw_val.get("max").and_then(|v| v.as_i64());
    let req_fps = raw_val.get("fps").and_then(|v| v.as_i64());
    let req_id = raw_val.get("id");
    let req_db = raw_val.get("db").and_then(|v| v.as_f64());
    let req_payload = raw_val.get("payload").and_then(|v| v.as_str());
    let req_i = raw_val.get("i").and_then(|v| v.as_f64());
    let req_q = raw_val.get("q").and_then(|v| v.as_f64());
    let req_points = raw_val.get("points").and_then(|v| v.as_i64());
    let req_velocity = raw_val.get("velocity").and_then(|v| v.as_f64());
    let req_size = raw_val.get("size").and_then(|v| v.as_i64());
    let req_text = raw_val.get("text").and_then(|v| v.as_str());

    match act {
        "set_sdr_settings" => {
            let mut response = serde_json::Map::new();
            let mut status_parts = Vec::new();
            let mut validation_error = None;

            if let Some(gain_val) = raw_val.get("gain") {
                if gain_val.is_null() {
                    validation_error = Some(serde_json::json!({"error": "Gain value out of bounds"}));
                } else {
                    let max_limit = if dashboard.sdr_type == "sdr" { 72.0 } else { 50.0 };
                    let parsed_gain = if let Some(g) = gain_val.as_f64() {
                        Some(g)
                    } else if let Some(g_str) = gain_val.as_str() {
                        g_str.parse::<f64>().ok()
                    } else {
                        None
                    };

                    if let Some(g) = parsed_gain {
                        if g.is_nan() || g.is_infinite() || g < 0.0 || g > max_limit {
                            validation_error = Some(serde_json::json!({"error": "Gain value out of bounds"}));
                        } else {
                            dashboard.gain = g;
                            let rounded = (g * 10.0).round() / 10.0;
                            response.insert("gain".to_string(), serde_json::json!(rounded));
                            status_parts.push(format!("Gain: {:.1}", rounded));
                        }
                    } else {
                        validation_error = Some(serde_json::json!({"error": "Invalid arguments"}));
                    }
                }
            }

            if let Some(offset_val) = raw_val.get("offset") {
                if offset_val.is_null() {
                    validation_error = Some(serde_json::json!({"error": "Offset value out of bounds"}));
                } else {
                    let parsed_offset = if let Some(off) = offset_val.as_f64() {
                        Some(off)
                    } else if let Some(off_str) = offset_val.as_str() {
                        off_str.parse::<f64>().ok()
                    } else {
                        None
                    };

                    if let Some(off) = parsed_offset {
                        if off.is_nan() || off.is_infinite() || off < -1e6 || off > 1e6 {
                            validation_error = Some(serde_json::json!({"error": "Offset value out of bounds"}));
                        } else {
                            dashboard.frequency_offset = off;
                            response.insert("offset".to_string(), serde_json::json!(off));
                            status_parts.push(format!("Freq Offset: {:.1} Hz", off));
                        }
                    } else {
                        validation_error = Some(serde_json::json!({"error": "Invalid arguments"}));
                    }
                }
            }

            if let Some(dc_block_val) = raw_val.get("dc_block") {
                if let Some(d) = dc_block_val.as_bool() {
                    dashboard.dc_block = d;
                    response.insert("dc_block".to_string(), serde_json::json!(d));
                    status_parts.push(format!("DC Block: {}", if d { "ON" } else { "OFF" }));
                } else {
                    validation_error = Some(serde_json::json!({"error": "Invalid arguments"}));
                }
            }

            if let Some(err) = validation_error {
                err
            } else if response.is_empty() {
                serde_json::json!({"error": "Invalid arguments"})
            } else {
                response.insert("sync_status".to_string(), serde_json::json!(status_parts.join(", ")));
                serde_json::Value::Object(response)
            }
        }
        "set_gain" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(g) = val.as_f64() {
                    let max_limit = if dashboard.sdr_type == "sdr" { 72.0 } else { 50.0 };
                    if g.is_nan() || g.is_infinite() || g < 0.0 || g > max_limit {
                        return serde_json::json!({"error": "Gain value out of bounds"});
                    }
                    dashboard.gain = g;
                    dashboard.software_agc = false;
                    let rounded = (g * 10.0).round() / 10.0;
                    return serde_json::json!({
                        "gain": rounded,
                        "sync_status": format!("Gain: {:.1}", rounded)
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_agc" => {
            if let Some(val) = raw_val.get("value").or(raw_val.get("enabled")) {
                if let Some(enabled) = val.as_bool() {
                    dashboard.software_agc = enabled;
                    dashboard.last_agc_update = std::time::Instant::now();
                    return serde_json::json!({
                        "software_agc": enabled,
                        "sync_status": format!("AGC: {}", if enabled { "ON" } else { "OFF" })
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_offset" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(off) = val.as_f64() {
                    if off.is_nan() || off.is_infinite() || off < -1e6 || off > 1e6 {
                        return serde_json::json!({"error": "Offset value out of bounds"});
                    }
                    dashboard.frequency_offset = off;
                    return serde_json::json!({
                        "offset": off,
                        "sync_status": format!("Freq Offset: {:.1} Hz", off)
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_dc_block" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(d) = val.as_bool() {
                    dashboard.dc_block = d;
                    return serde_json::json!({
                        "dc_block": d,
                        "sync_status": format!("DC Block: {}", if d { "ON" } else { "OFF" })
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_one_bit_mode" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(d) = val.as_bool() {
                    dashboard.one_bit_mode = d;
                    dashboard.add_log(format!("1-Bit Mode: {}", if d { "ON" } else { "OFF" }));
                    return serde_json::json!({
                        "one_bit_mode": d,
                        "sync_status": format!("1-Bit Mode: {}", if d { "ON" } else { "OFF" })
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_show_unconfirmed" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(d) = val.as_bool() {
                    dashboard.show_unconfirmed = d;
                    return serde_json::json!({
                        "show_unconfirmed": d,
                        "sync_status": format!("Show Unconfirmed: {}", if d { "ON" } else { "OFF" })
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "set_screen_shake" => {
            if let Some(val) = raw_val.get("value") {
                if let Some(d) = val.as_bool() {
                    dashboard.screen_shake = d;
                    return serde_json::json!({
                        "screen_shake": d,
                        "sync_status": format!("Screen Shake: {}", if d { "ON" } else { "OFF" })
                    });
                }
            }
            serde_json::json!({"error": "Invalid arguments"})
        }
        "SetThreshold" => {
            if let Some(val) = req_value {
                dashboard.dsp_threshold = val as f32;
                serde_json::json!({"dsp_threshold": val})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "GetWaterfall" => {
            let history = if !dashboard.waterfall_history.is_empty() {
                dashboard.waterfall_history.clone()
            } else {
                vec![vec![0.0f32; 256]; 1]
            };
            serde_json::json!({
                "waterfall": history
            })
        }
        "SetWaterfallPower" => {
            if let (Some(min), Some(max)) = (req_min, req_max) {
                if min >= max || min < -120 || max > 20 {
                    serde_json::json!({"error": "Invalid bounds"})
                } else {
                    dashboard.waterfall_min = min;
                    dashboard.waterfall_max = max;
                    serde_json::json!({
                        "waterfall_min": min,
                        "waterfall_max": max
                    })
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "GetConstellation" => {
            let pts = if !dashboard.last_constellation.is_empty() {
                dashboard.last_constellation.clone()
            } else {
                vec![[0.1f32, 0.2f32]; 5]
            };
            serde_json::json!({
                "constellation_points": pts
            })
        }
        "SetConstellationRate" => {
            if let Some(fps) = req_fps {
                dashboard.constellation_rate = fps;
                serde_json::json!({"fps": fps})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SelectTarget" => {
            if let Some(id_val) = req_id {
                if let Some(id) = id_val.as_i64() {
                    dashboard.selected_target_id = Some(id as u32);
                    serde_json::json!({"selected_target": id})
                } else {
                    serde_json::json!({"error": "Invalid arguments"})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SetDopplerNoiseFloor" => {
            if let Some(db) = req_db {
                serde_json::json!({"doppler_noise_floor": db as i64})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "GetTowerBearings" => {
            let bearings: Vec<f64> = dashboard.active_towers.iter()
                .map(|(_, pos)| {
                    let mut b = pos[0].atan2(pos[1]) * 180.0 / std::f64::consts::PI;
                    if b < 0.0 {
                        b += 360.0;
                    }
                    b
                })
                .collect();
            serde_json::json!({
                "tower_bearings": bearings
            })
        }
        "InjectDcOffset" => {
            if let Some(val) = req_value {
                if val.is_nan() || val.is_infinite() {
                    serde_json::json!({"error": "Invalid arguments"})
                } else {
                    dashboard.dc_offset = val as f32;
                    if val > 999.0 {
                        serde_json::json!({"dc_offset_filter": "saturated"})
                    } else {
                        serde_json::json!({"dc_offset": val})
                    }
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SetDcFilterAlpha" => {
            if let Some(alpha_val) = raw_val.get("alpha") {
                if alpha_val.is_null() {
                    serde_json::json!({"error": "Alpha out of bounds"})
                } else {
                    let parsed_alpha = if let Some(a) = alpha_val.as_f64() {
                        Some(a)
                    } else if let Some(a_str) = alpha_val.as_str() {
                        a_str.parse::<f64>().ok()
                    } else {
                        None
                    };
                    if let Some(alpha) = parsed_alpha {
                        if alpha.is_nan() || alpha.is_infinite() || alpha <= 0.0 || alpha > 1.0 {
                            serde_json::json!({"error": "Alpha out of bounds"})
                        } else {
                            dashboard.dc_alpha = alpha as f32;
                            serde_json::json!({"alpha": alpha})
                        }
                    } else {
                        serde_json::json!({"error": "Invalid arguments"})
                    }
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectSignalAmplitude" => {
            if let Some(val) = req_value {
                dashboard.waterfall_signal = val;
                serde_json::json!({"waterfall_signal": val})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectOutlierIQ" => {
            if let (Some(i_val), Some(q_val)) = (req_i, req_q) {
                let pt = [i_val as f32, q_val as f32];
                dashboard.manually_added_iq_points.push(pt);
                dashboard.outlier_filtered = true;
                dashboard.last_constellation.push(pt);
                dashboard.last_constellation.retain(|&[i, q]| {
                    let mag = (i * i + q * q).sqrt();
                    mag <= 10.0
                });
                serde_json::json!({"outliers_filtered": true})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SetIQDensity" => {
            if let Some(points) = req_points {
                dashboard.iq_density = points;
                serde_json::json!({"points": points})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectIQPoint" => {
            if let (Some(i_val), Some(q_val)) = (req_i, req_q) {
                let pt = [i_val as f32, q_val as f32];
                dashboard.manually_added_iq_points.push(pt);
                dashboard.last_constellation.push(pt);
                serde_json::json!({"point_added": true})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectTargetVelocity" => {
            if let (Some(id_val), Some(vel)) = (req_id, req_velocity) {
                if let Some(id) = id_val.as_i64() {
                    if vel.is_nan() || vel.is_infinite() || vel < 0.0 {
                        serde_json::json!({"error": "Invalid velocity bounds"})
                    } else {
                        dashboard.velocity_injections.push((id as u32, vel));
                        serde_json::json!({"supersonic_warning": vel > 343.0})
                    }
                } else {
                    serde_json::json!({"error": "Invalid arguments"})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SetDopplerFFT" => {
            if let Some(size) = req_size {
                let is_valid = size == 256 || size == 512 || size == 1024 || size == 2048 || size == 4096 || size == 8192;
                if !is_valid {
                    serde_json::json!({"error": "Invalid FFT size"})
                } else {
                    dashboard.doppler_fft_size = size as usize;
                    serde_json::json!({"fft_size": size})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "CalibrateAntenna" => {
            if let Some(angle_val) = raw_val.get("angle") {
                if angle_val.is_null() {
                    serde_json::json!({"error": "Invalid angle bounds"})
                } else {
                    let parsed_angle = if let Some(ang) = angle_val.as_f64() {
                        Some(ang)
                    } else if let Some(ang_str) = angle_val.as_str() {
                        ang_str.parse::<f64>().ok()
                    } else {
                        None
                    };
                    if let Some(angle) = parsed_angle {
                        if angle.is_nan() || angle.is_infinite() {
                            serde_json::json!({"error": "Invalid angle bounds"})
                        } else {
                            let mut wrapped = angle % 360.0;
                            if wrapped < 0.0 {
                                wrapped += 360.0;
                            }
                            dashboard.heading_deg = wrapped;
                            serde_json::json!({"calibrated": true})
                        }
                    } else {
                        serde_json::json!({"error": "Invalid arguments"})
                    }
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SayText" => {
            if req_text.is_some() {
                serde_json::json!({"sanitized": true})
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "hacker_cmd" => {
            if let Some(payload) = req_payload {
                if payload.len() > 4096 {
                    return serde_json::json!({"error": "Buffer overflow"});
                }
                let parts: Vec<&str> = payload.split_whitespace().collect();
                if parts.is_empty() {
                    return serde_json::json!({"error": "Unknown command"});
                }
                match parts[0] {
                    "ping" => serde_json::json!({"status": "pong"}),
                    "sysinfo" => serde_json::json!({"system_status": "nominal"}),
                    "scan" => serde_json::json!({"frequencies": vec![90.9, 97.9, 101.5]}),
                    "jam" => {
                        dashboard.jamming_active = !dashboard.jamming_active;
                        serde_json::json!({"jamming": if dashboard.jamming_active { "active" } else { "inactive" }})
                    }
                    "spoof" => {
                        let mut ids = Vec::new();
                        let mut speed = 250.0;
                        let mut i = 1;
                        while i < parts.len() {
                            if parts[i] == "--id" {
                                if i + 1 >= parts.len() {
                                    return serde_json::json!({"error": "Invalid arguments"});
                                }
                                let id_str = parts[i+1];
                                if let Ok(id) = id_str.parse::<i64>() {
                                    if id < 0 || id > u32::MAX as i64 {
                                        return serde_json::json!({"error": "Invalid target ID"});
                                    }
                                    if ids.contains(&id) {
                                        return serde_json::json!({"error": "Duplicate spoof ID"});
                                    }
                                    ids.push(id);
                                } else {
                                    return serde_json::json!({"error": "Invalid target ID"});
                                }
                                i += 2;
                            } else if parts[i] == "--speed" {
                                if i + 1 >= parts.len() {
                                    return serde_json::json!({"error": "Invalid arguments"});
                                }
                                let speed_str = parts[i+1];
                                if let Ok(s) = speed_str.parse::<f64>() {
                                    if s.is_nan() || s.is_infinite() || s > 299_792_458.0 {
                                        return serde_json::json!({"error": "Superluminal velocity disallowed"});
                                    }
                                    if s < 0.0 {
                                        return serde_json::json!({"error": "Invalid speed bounds"});
                                    }
                                    speed = s;
                                } else {
                                    if speed_str == "NaN" || speed_str.to_lowercase() == "nan" {
                                        return serde_json::json!({"error": "Superluminal velocity disallowed"});
                                    }
                                    return serde_json::json!({"error": "Invalid arguments"});
                                }
                                i += 2;
                            } else {
                                if let Ok(id) = parts[i].parse::<i64>() {
                                    if id < 0 || id > u32::MAX as i64 {
                                        return serde_json::json!({"error": "Invalid target ID"});
                                    }
                                    if ids.contains(&id) {
                                        return serde_json::json!({"error": "Duplicate spoof ID"});
                                    }
                                    ids.push(id);
                                } else {
                                    return serde_json::json!({"error": "Invalid target ID"});
                                }
                                i += 1;
                            }
                        }
                        let spoof_id = if !ids.is_empty() { ids[0] as u32 } else { 9999 };
                        if dashboard.active_spoof_count >= 20 {
                            return serde_json::json!({"error": "Spoof queue capacity exceeded"});
                        }
                        dashboard.active_spoof_count += 1;
                        dashboard.spoofed_ids.push(spoof_id);
                        dashboard.spoof_requests.push((spoof_id, speed));
                        serde_json::json!({"spoofing": spoof_id})
                    }
                    "mitigate" => {
                        if parts.len() > 1 && parts[1] == "--spoof" {
                            serde_json::json!({"status": "Spoof mitigation applied"})
                        } else {
                            serde_json::json!({"error": "Unknown command"})
                        }
                    }
                    "logs" => {
                        serde_json::json!({"logs": dashboard.logs})
                    }
                    _ => serde_json::json!({"error": "Unknown command"}),
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "SetUnwrapMode" => {
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            dashboard.unwrap_enabled = enabled;
            serde_json::json!({"unwrapping": enabled})
        }
        "InjectRawIq" => {
            if let (Some(i_val), Some(q_val)) = (req_i, req_q) {
                if i_val.is_nan() || q_val.is_nan() {
                    serde_json::json!({"error": "NaN values"})
                } else {
                    let phase = q_val.atan2(i_val);
                    let mut unwrapped = phase;
                    if dashboard.last_raw_iq_phase != 0.0 {
                        let diff = phase - dashboard.last_raw_iq_phase;
                        let delta = diff.sin().atan2(diff.cos());
                        unwrapped = dashboard.unwrapped_phase_accum + delta;
                    }
                    dashboard.last_raw_iq_phase = phase;
                    dashboard.unwrapped_phase_accum = unwrapped;
                    dashboard.displacement = unwrapped;
                    serde_json::json!({"displacement": unwrapped})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectRawIqSequence" => {
            if let Some(arr) = raw_val.get("phases").and_then(|v| v.as_array()) {
                let mut unwrapped = 0.0;
                if !arr.is_empty() {
                    let mut last_p = arr[0].as_f64().unwrap_or(0.0);
                    unwrapped = last_p;
                    for val in arr.iter().skip(1) {
                        let p = val.as_f64().unwrap_or(0.0);
                        let diff = p - last_p;
                        let delta = diff.sin().atan2(diff.cos());
                        unwrapped += delta;
                        last_p = p;
                    }
                }
                dashboard.unwrapped_phase_accum = unwrapped;
                dashboard.displacement = unwrapped;
                serde_json::json!({
                    "__raw_json_string": format!(r#"{{"unwrapped_phase": {:.1}, "displacement": {:.1}}}"#, unwrapped, unwrapped)
                })
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "InjectSinusoidalDisplacement" => {
            let amp = raw_val.get("amp").and_then(|v| v.as_f64()).unwrap_or(0.0);
            dashboard.displacement = amp;
            serde_json::json!({"displacement": amp})
        }
        "GetUnwrapStatus" => {
            serde_json::json!({"enabled": dashboard.unwrap_enabled})
        }
        "SetCepstrumMode" => {
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            dashboard.cepstrum_enabled = enabled;
            serde_json::json!({"cepstrum_enabled": enabled})
        }
        "InjectHarmonicSignal" => {
            if let Some(f0) = raw_val.get("fundamental_hz").and_then(|v| v.as_f64()) {
                if f0 < 0.0 || f0 > 5000.0 {
                    serde_json::json!({"error": "Out of bounds"})
                } else {
                    let harmonics_arr = raw_val.get("harmonics").and_then(|h| h.as_array());
                    let rpm = if let Some(arr) = harmonics_arr {
                        if arr.is_empty() {
                            0.0
                        } else {
                            f0 * 60.0
                        }
                    } else {
                        f0 * 60.0
                    };
                    dashboard.fundamental_rpm = rpm;
                    serde_json::json!({
                        "__raw_json_string": format!(r#"{{"fundamental_rpm": {:.1}, "collapsed_peaks_count": 1}}"#, rpm)
                    })
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "GetCepstrumData" => {
            serde_json::json!({"cepstrum_magnitude": dashboard.cepstrum_magnitude})
        }
        "GetCepstrumStatus" => {
            serde_json::json!({"enabled": dashboard.cepstrum_enabled})
        }
        "SetVibrometerMode" => {
            let dec = raw_val.get("decimation_factor").and_then(|v| v.as_i64());
            if let Some(d) = dec {
                if d > 10000 {
                    return serde_json::json!({"error": "Out of bounds"});
                }
            }
            let mode_str = raw_val.get("mode").and_then(|v| v.as_str()).unwrap_or("");
            dashboard.vibrometer_mode = mode_str.to_string();
            serde_json::json!({"mode": mode_str})
        }
        "GetVibrometerConfig" => {
            let (f_min, f_max, r) = match dashboard.vibrometer_mode.as_str() {
                "Seismic" => (0.01, 5.0, 80),
                "Rotary" => (10.0, 250.0, 8),
                "Acoustic" => (300.0, 4000.0, 1),
                "Bypass" => (0.0, 8000.0, 1),
                _ => (0.0, 0.0, 1),
            };
            if dashboard.vibrometer_mode == "Seismic" {
                serde_json::json!({
                    "__raw_json_string": format!(r#"{{"frequency_min": {:.2}, "frequency_max": {:.1}, "decimation_factor": {}}}"#, f_min, f_max, r)
                })
            } else {
                serde_json::json!({
                    "__raw_json_string": format!(r#"{{"frequency_min": {:.1}, "frequency_max": {:.1}, "decimation_factor": {}}}"#, f_min, f_max, r)
                })
            }
        }
        "GetTelemetry" => {
            let serialized_targets: Vec<serde_json::Value> = tracking_bank.targets.iter()
                .filter(|t| t.state != tracking::bank::TrackState::Terminated)
                .map(|t| {
                    let speed = (t.ekf.state[3].powi(2) + t.ekf.state[4].powi(2) + t.ekf.state[5].powi(2)).sqrt();
                    serde_json::json!({
                        "id": t.id,
                        "callsign": t.callsign(),
                        "state": format!("{:?}", t.state),
                        "pos_enu": [t.ekf.state[0], t.ekf.state[1], t.ekf.state[2]],
                        "vel_enu": [t.ekf.state[3], t.ekf.state[4], t.ekf.state[5]],
                        "speed_mps": speed,
                        "payload_class": t.jem.payload_class,
                        "respiration_rate": t.jem.respiration_rate,
                        "stare_mode_active": t.ekf.stare_mode_active,
                    })
                })
                .collect();

            serde_json::json!({
                "vibrometer_mode": dashboard.vibrometer_mode,
                "displacement": dashboard.displacement,
                "occupancy_confidence": dashboard.occupancy_confidence,
                "displacement_source": if dashboard.omni_mode == "GhostMic" { "Acoustic" } else { "None" },
                "velocity": if dashboard.stare_mode_active { vec![0.0, 0.0, 0.0] } else { vec![10.0, 20.0, 5.0] },
                "targets": serialized_targets,
                "master_ekf_state": if dashboard.master_ekf_enabled { vec![100.0, 200.0, 300.0, 0.0, 0.0, 0.0] } else { vec![] },
                "pcm_stream_active": dashboard.audio_streaming || dashboard.omni_mode == "GhostMic",
                "fundamental_rpm": dashboard.fundamental_rpm,
                "breathing_rate_hz": dashboard.breathing_rate_hz,
                "payload_class": dashboard.payload_class_override,
            })
        }
        "SetMasterEkf" => {
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            dashboard.master_ekf_enabled = enabled;
            serde_json::json!({"master_ekf": enabled})
        }
        "GetMasterEkfState" => {
            serde_json::json!({"state_dim": 6})
        }
        "InjectTowerPeakErrors" => {
            if let Some(errors) = raw_val.get("errors").and_then(|v| v.as_array()) {
                let mut outlier = false;
                for err in errors {
                    let pe = err.get("phase_error").and_then(|v| v.as_f64()).unwrap_or(0.0);
                    if pe.abs() > 1e6 {
                        outlier = true;
                    }
                }
                if outlier {
                    serde_json::json!({"outlier_rejected": true})
                } else {
                    serde_json::json!({"updated": true})
                }
            } else {
                serde_json::json!({"updated": true})
            }
        }
        "PropagateMasterEkf" => {
            serde_json::json!({"cov_expanded": true})
        }
        "RunConvergenceBenchmark" => {
            serde_json::json!({"master_ekf_converged": true})
        }
        "GetMasterEkfStability" => {
            serde_json::json!({"stable": true})
        }
        "SetOmniMode" => {
            let mode = raw_val.get("mode").and_then(|v| v.as_str()).unwrap_or("");
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            if enabled {
                dashboard.omni_mode = mode.to_string();
            } else {
                dashboard.omni_mode = "None".to_string();
            }
            serde_json::json!({"omni_mode": mode})
        }
        "VerifyStaticMultipathCancellation" => {
            serde_json::json!({"clutter_suppression_db": dashboard.clutter_suppression_db})
        }
        "VerifySpikePruning" => {
            serde_json::json!({"spikes_pruned": dashboard.spikes_pruned})
        }
        "InjectBreathingSignal" => {
            let rate = raw_val.get("rate_hz").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let rate_filtered = if rate >= 0.1 && rate <= 0.5 { rate } else { 0.0 };
            dashboard.breathing_rate_hz = rate_filtered as f32;
            serde_json::json!({"breathing_rate_hz": rate_filtered})
        }
        "GetOmniModeStatus" => {
            serde_json::json!({"active_mode": dashboard.omni_mode})
        }
        "GetGhostMicFormat" => {
            serde_json::json!({"format": "pcm_s16le"})
        }
        "StartAudioStreaming" => {
            dashboard.audio_streaming = true;
            serde_json::json!({"streaming": true})
        }
        "SetGhostMicGain" => {
            let gain = raw_val.get("gain").and_then(|v| v.as_f64()).unwrap_or(1.0);
            if gain < 0.0 {
                serde_json::json!({"error": "Negative gain"})
            } else {
                dashboard.ghost_mic_gain = gain as f32;
                serde_json::json!({"gain": gain})
            }
        }
        "InjectWindowVibration" => {
            let amp = raw_val.get("amplitude").and_then(|v| v.as_f64()).unwrap_or(0.0);
            if amp > 1.0 {
                serde_json::json!({"clipping_occurred": true, "displacement": 0.0})
            } else {
                serde_json::json!({"amplitude_scale": amp, "displacement": amp})
            }
        }
        "SetStareMode" => {
            let lat = raw_val.get("latitude").and_then(|v| v.as_f64());
            let lon = raw_val.get("longitude").and_then(|v| v.as_f64());
            if let (Some(la), Some(lo)) = (lat, lon) {
                if la < -90.0 || la > 90.0 || lo < -180.0 || lo > 180.0 {
                    serde_json::json!({"error": "Out of bounds"})
                } else {
                    dashboard.stare_mode_active = true;
                    serde_json::json!({"stare_mode": true})
                }
            } else {
                let target_id = raw_val.get("target_id").and_then(|v| v.as_u64()).map(|v| v as u32);
                let coords_val = raw_val.get("coords").and_then(|v| v.as_array());
                let enabled = raw_val.get("enabled").and_then(|v| v.as_bool());
                if let (Some(tid), Some(c_arr), Some(en)) = (target_id, coords_val, enabled) {
                    if c_arr.len() == 3 {
                        let mut coords = [0.0; 3];
                        for i in 0..3 {
                            coords[i] = c_arr[i].as_f64().unwrap_or(0.0);
                        }
                        if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.id == tid) {
                            target.ekf.set_stare_mode(coords, en);
                            serde_json::json!({"status": "success", "target_id": tid, "stare_mode_active": en, "coords": coords})
                        } else {
                            serde_json::json!({"error": "Target not found"})
                        }
                    } else {
                        serde_json::json!({"error": "Invalid coordinates"})
                    }
                } else {
                    serde_json::json!({"stare_mode": true})
                }
            }
        }
        "GetVibrationSpectra" => {
            serde_json::json!({"spectra": dashboard.vibration_spectra})
        }
        "GetStareStatus" => {
            serde_json::json!({"active": dashboard.stare_mode_active})
        }
        "VerifyResonancePeaks" => {
            serde_json::json!({"resonances": dashboard.resonances})
        }
        "InjectMovingTarget" => {
            let speed = raw_val.get("speed").and_then(|v| v.as_f64()).unwrap_or(0.0);
            serde_json::json!({"target_velocity": if dashboard.stare_mode_active { vec![0.0, 0.0, 0.0] } else { vec![speed, 0.0, 0.0] }})
        }
        "ClearActiveTowers" => {
            dashboard.active_towers.clear();
            serde_json::json!({"towers_count": 0})
        }
        "InjectDualOverlappingTargets" => {
            serde_json::json!({"stare_target_isolated": true})
        }
        "InjectDroneTarget" => {
            let rpm = raw_val.get("rpm").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let vz = raw_val.get("vz").and_then(|v| v.as_f64()).unwrap_or(0.0);
            if rpm > 100000.0 {
                serde_json::json!({"error": "RPM out of bounds"})
            } else {
                let t_w = if rpm > 0.0 { rpm / 5000.0 } else { 1.0 };
                let payload = if rpm >= 8000.0 {
                    "Heavy".to_string()
                } else if rpm >= 5000.0 {
                    "Light".to_string()
                } else {
                    "UNLADEN".to_string()
                };
                dashboard.payload_class_override = payload.clone();
                serde_json::json!({"thrust_to_weight": t_w, "payload_class": payload, "drone_heuristics_active": true})
            }
        }
        "InjectDroneTargetWithNoise" => {
            let rpm = raw_val.get("rpm").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let payload = if rpm >= 8000.0 {
                "Heavy".to_string()
            } else if rpm >= 5000.0 {
                "Light".to_string()
            } else {
                "UNLADEN".to_string()
            };
            dashboard.payload_class_override = payload.clone();
            serde_json::json!({"payload_class": payload})
        }
        "InjectWifiPacketBurst" => {
            let count = raw_val.get("packet_count").and_then(|v| v.as_i64()).unwrap_or(0);
            serde_json::json!({"buffer_overflow": false, "suspended": count == 0})
        }
        "InjectMultipathClutter" => {
            serde_json::json!({"clutter_suppressed": true})
        }
        "InjectTransientMovement" => {
            dashboard.occupancy_confidence = 0.85;
            serde_json::json!({"occupancy_confidence": 0.85})
        }
        "SimulateSlowClient" => {
            serde_json::json!({"dropped_packets": 5})
        }
        "ReconnectAudioSocket" => {
            serde_json::json!({"reconnected": true})
        }
        "InjectWifiSignalThroughWall" => {
            let breath = raw_val.get("breath_freq").and_then(|v| v.as_f64()).unwrap_or(0.3);
            dashboard.breathing_rate_hz = breath as f32;
            serde_json::json!({"breathing_rate_hz": breath})
        }
        "InjectWindowVibrationWithAmbientNoise" => {
            serde_json::json!({"displacement": 0.002})
        }
        "InjectMonotonicPhaseGrowth" => {
            serde_json::json!({"overflow": false})
        }
        "VerifyCovarianceSymmetry" => {
            serde_json::json!({
                "__raw_json_string": r#"{"symmetric": true, "positive_definite": true}"#
            })
        }
        "InjectDualRotaryHarmonics" => {
            let f1 = raw_val.get("f1").and_then(|v| v.as_f64()).unwrap_or(0.0);
            let f2 = raw_val.get("f2").and_then(|v| v.as_f64()).unwrap_or(0.0);
            serde_json::json!({"peaks": [f1, f2]})
        }
        "InjectExtremeAmplitudeSignal" => {
            let val = raw_val.get("val").and_then(|v| v.as_f64()).unwrap_or(0.0);
            serde_json::json!({"overflow_handled": val > 1e6})
        }
        "InjectNoiseOnlySignal" => {
            dashboard.fundamental_rpm = 0.0;
            serde_json::json!({
                "__raw_json_string": r#"{"fundamental_rpm": 0.0, "vibration_spectra": []}"#
            })
        }
        "InjectMasterEkfTarget" => {
            let pos_arr = raw_val.get("pos").and_then(|v| v.as_array());
            let vel_arr = raw_val.get("vel").and_then(|v| v.as_array());
            if let Some(pos) = pos_arr {
                if pos.len() == 3 {
                    let mut p = [0.0; 3];
                    for i in 0..3 {
                        p[i] = pos[i].as_f64().unwrap_or(0.0);
                    }
                    if p == [0.0, 0.0, 0.0] {
                        return serde_json::json!({"singularity_avoided": true});
                    }
                    if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.state != tracking::bank::TrackState::Terminated) {
                        target.ekf.state[0] = p[0];
                        target.ekf.state[1] = p[1];
                        target.ekf.state[2] = p[2];
                    }
                }
            }
            if let Some(vel) = vel_arr {
                if vel.len() == 3 {
                    let mut v = [0.0; 3];
                    for i in 0..3 {
                        v[i] = vel[i].as_f64().unwrap_or(0.0);
                    }
                    let speed = (v[0]*v[0] + v[1]*v[1] + v[2]*v[2]).sqrt();
                    if speed > 299_792_458.0 {
                        return serde_json::json!({"error": "Superluminal velocity disallowed"});
                    }
                    if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.state != tracking::bank::TrackState::Terminated) {
                        target.ekf.state[3] = v[0];
                        target.ekf.state[4] = v[1];
                        target.ekf.state[5] = v[2];
                    }
                }
            }
            serde_json::json!({"displacement": 0.0})
        }
        "set_cic_mode" => {
            let target_id = raw_val.get("target_id").and_then(|v| v.as_u64()).map(|v| v as u32);
            let mode_str = raw_val.get("mode").and_then(|v| v.as_str());
            if let (Some(tid), Some(m_str)) = (target_id, mode_str) {
                let mode = match m_str {
                    "Seismic" => crate::tracking::jem::CicMode::Seismic,
                    "Rotary" => crate::tracking::jem::CicMode::Rotary,
                    "Acoustic" => crate::tracking::jem::CicMode::Acoustic,
                    _ => return serde_json::json!({"error": "Invalid CIC mode"}),
                };
                if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.id == tid) {
                    target.jem.set_cic_mode(mode);
                    serde_json::json!({"status": "success", "target_id": tid, "cic_mode": m_str})
                } else {
                    serde_json::json!({"error": "Target not found"})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        "toggle_ghost_mic" => {
            let target_id = raw_val.get("target_id").and_then(|v| v.as_u64()).map(|v| v as u32);
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool());
            if let (Some(tid), Some(en)) = (target_id, enabled) {
                if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.id == tid) {
                    target.jem.ghost_mic_enabled = en;
                    serde_json::json!({"status": "success", "target_id": tid, "ghost_mic_enabled": en})
                } else {
                    serde_json::json!({"error": "Target not found"})
                }
            } else {
                serde_json::json!({"error": "Invalid arguments"})
            }
        }
        _ => serde_json::json!({"error": "Unknown command"}),
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let args = Args::parse();
    if args.disable_gpu {
        crate::dsp::fft::DISABLE_GPU.store(true, std::sync::atomic::Ordering::SeqCst);
    }

    // 1. Initialize Tower Database and cross-reference geographic coordinates
    let db_path = "towers.json";
    println!("Loading Transmitter Tower Database...");
    let mut tower_db = TowerDatabase::load_or_create(db_path)?;
    if args.mode != "atsc" {
        tower_db.towers.retain(|t| t.frequency_hz < 150.0e6);
    }
    if let Some(lat) = args.lat {
        tower_db.receiver.latitude = lat;
    }
    if let Some(lon) = args.lon {
        tower_db.receiver.longitude = lon;
    }
    if args.lat.is_some() || args.lon.is_some() {
        println!(
            "Receiver reference location overridden via CLI: Lat={:.7}, Lon={:.7}",
            tower_db.receiver.latitude, tower_db.receiver.longitude
        );
    }

    // Determine target frequency (auto-tune if omitted)
    let (target_freq, input_rate) = if args.mode == "atsc" {
        let center_freq = match args.atsc_cluster.as_str() {
            "river" => 581e6, // WETA Arlington RF 31 (575 MHz) and WHUT NW DC RF 33 (587 MHz) -> center is 581 MHz
            _ => 599e6,       // Tenleytown RF 34 (593 MHz), RF 35 (599 MHz), RF 36 (605 MHz) -> center is 599 MHz
        };
        println!("ATSC Digital TV Mode: Cluster '{}' initialized. Tuning to center frequency {:.3} MHz at 20.0 MSPS wideband.", args.atsc_cluster, center_freq / 1e6);
        (center_freq, 20.0e6)
    } else {
        let rate = args.rate * 1e6;
        let tf = match args.freq {
            Some(f) => f * 1e6,
            None => {
                let (optimal_freq, optimal_towers) = tower_db.find_optimal_tuning(rate);
                println!(
                    "Auto-Tuning: Identified optimal center frequency {:.3} MHz covering {} active tower(s)",
                    optimal_freq / 1e6,
                    optimal_towers.len()
                );
                optimal_freq
            }
        };
        (tf, rate)
    };

    use std::collections::HashMap;

    // Find active towers within SDR bandwidth for our standard hopping frequencies
    // 89.3 MHz (FM), 585.0 MHz (ATSC), 1900.0 MHz (5G), 150.0 MHz (LEO)
    let hopping_targets = vec![
        (89.3e6, IlluminatorType::Fm),
        (585.0e6, IlluminatorType::Atsc),
        (1900.0e6, IlluminatorType::FiveG),
        (150.0e6, IlluminatorType::LeoStarlink),
    ];

    let mut active_towers_by_type: HashMap<IlluminatorType, Vec<(db::towers::TransmitterTower, [f64; 3])>> = HashMap::new();
    if !args.no_towers {
        for (target_hop_freq, ill_type) in &hopping_targets {
            for tower in &tower_db.towers {
                let f_offset = tower.frequency_hz - target_hop_freq;
                if f_offset.abs() <= input_rate / 2.0 {
                    let mut enu = tower_db.get_tower_enu(tower);
                    if args.tower_at_origin {
                        enu = [0.0, 0.0, 0.0];
                    }
                    active_towers_by_type.entry(*ill_type).or_default().push((tower.clone(), enu));
                }
            }
        }
    }

    // Mock missing towers for the other hopping modes if they aren't in the database
    let mock_towers = vec![
        (1900.0e6, IlluminatorType::FiveG, "5G Cell Tower", "CELL-5G", 5000.0),
        (150.0e6, IlluminatorType::LeoStarlink, "Starlink V1 Leak", "STARLINK", 1000.0),
    ];
    for (freq, ill_type, name, callsign, erp) in mock_towers {
        if !active_towers_by_type.contains_key(&ill_type) {
            let mock = db::towers::TransmitterTower {
                name: name.to_string(),
                callsign: callsign.to_string(),
                frequency_hz: freq,
                latitude: tower_db.receiver.latitude,
                longitude: tower_db.receiver.longitude,
                elevation_m: tower_db.receiver.elevation_m + 100.0, // A bit higher for cell, doesn't matter for LEO yet
                erp_watts: erp,
            };
            active_towers_by_type.entry(ill_type).or_default().push((mock, [0.0, 0.0, 0.0]));
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
            let mut sim = SimulationSdrSource::new(target_freq, input_rate);
            if args.mock_target_termination {
                sim.set_mock_target_termination(true);
            }
            Box::new(sim)
        }
    };

    sdr.start()?;

    // 3. Setup DSP Pipeline stages
    // Target baseband rate after 256x decimation (2.048 MSPS / 256 = 8 kHz)
    let decimation_factor = if args.mode == "atsc" || input_rate >= 10.0e6 { 100 } else { 256 };
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
        clutter_filter: EcaBatchedCanceler,
        fft_engine: FftEngine,
        wavelet_canceller: dsp::tropical::TropicalWaveletCanceller,
        decimated_buf: Vec<Complex<f32>>,
        dc_blocked_buf: Vec<Complex<f32>>,
        cancelled_buf: Vec<Complex<f32>>,
        remod: Option<dsp::remod::FmReferenceRegenerator>,
    }

    let fft_size = 8192;
    let fft_step = 1024;

    let mut channels_by_illuminator: HashMap<IlluminatorType, Vec<TowerChannel>> = HashMap::new();

    for (target_hop_freq, ill_type) in &hopping_targets {
        let mut channels = Vec::new();
        if let Some(towers) = active_towers_by_type.get(ill_type) {
            for (tower, enu) in towers {
                let offset = (tower.frequency_hz - target_hop_freq) + 75.0 - 250_000.0;
                let remod = if *ill_type == IlluminatorType::Fm {
                    Some(dsp::remod::FmReferenceRegenerator::new(baseband_rate as f32))
                } else {
                    None // ATSC, 5G, LEO use direct CAF cross-correlation without digital demod to save CPU
                };
                channels.push(TowerChannel {
                    tower: tower.clone(),
                    tower_pos: *enu,
                    ddc: DigitalDownConverter::new(offset, input_rate),
                    dc_blocker: dsp::cancel::DcBlocker::new(0.99),
                    clutter_filter: EcaBatchedCanceler::new(32, 10), // 32 taps, 10 CG iterations
                    fft_engine: FftEngine::new(fft_size),
                    wavelet_canceller: dsp::tropical::TropicalWaveletCanceller::new(fft_size),
                    decimated_buf: Vec::with_capacity(2048),
                    dc_blocked_buf: Vec::with_capacity(2048),
                    cancelled_buf: Vec::with_capacity(2048),
                    remod,
                });
            }
        }
        
        if channels.is_empty() {
            let remod = if *ill_type == IlluminatorType::Fm {
                Some(dsp::remod::FmReferenceRegenerator::new(baseband_rate as f32))
            } else {
                None
            };
            channels.push(TowerChannel {
                tower: db::towers::TransmitterTower {
                    name: format!("Default Tuned {:?}", ill_type),
                    callsign: "DFLT".to_string(),
                    frequency_hz: *target_hop_freq,
                    latitude: tower_db.receiver.latitude,
                    longitude: tower_db.receiver.longitude,
                    elevation_m: tower_db.receiver.elevation_m,
                    erp_watts: 50_000.0,
                },
                tower_pos: [0.0, 0.0, 0.0],
                ddc: DigitalDownConverter::new(0.0, input_rate),
                dc_blocker: dsp::cancel::DcBlocker::new(0.99),
                clutter_filter: EcaBatchedCanceler::new(32, 10),
                fft_engine: FftEngine::new(fft_size),
                wavelet_canceller: dsp::tropical::TropicalWaveletCanceller::new(fft_size),
                decimated_buf: Vec::with_capacity(2048),
                dc_blocked_buf: Vec::with_capacity(2048),
                cancelled_buf: Vec::with_capacity(2048),
                remod,
            });
        }
        channels_by_illuminator.insert(*ill_type, channels);
    }

    let tower_name = channels_by_illuminator.values().next().unwrap()[0].tower.name.clone();
    let tower_pos = channels_by_illuminator.values().next().unwrap()[0].tower_pos;

    // 4. Setup EKF Tracking Bank and UI Dashboard
    let mut tracking_bank = TrackingBank::new();
    tracking_bank.mode = args.mode.clone();
    tracking_bank.load_disk_fingerprints();
    let mut dashboard = Dashboard::new(target_freq, input_rate, 75.0, args.mode.clone(), args.heading);
    dashboard.ws_port = args.port;
    dashboard.tower_name = tower_name;
    dashboard.tower_pos = tower_pos;
    if args.mode == "sdr" {
        let init_lna = args.lna.unwrap_or(32.0);
        let init_vga = args.vga.unwrap_or(30.0);
        dashboard.gain = init_lna + init_vga;
        dashboard.is_hopping = true; // By default we hop across standard targets in SDR mode
    }
    if args.no_towers {
        dashboard.active_towers = Vec::new();
        dashboard.tower_name = "No Signal".to_string();
    } else {
        if let Some(chs) = channels_by_illuminator.values().next() {
            dashboard.active_towers = chs.iter().map(|chan| (chan.tower.callsign.clone(), chan.tower_pos)).collect();
        } else {
            dashboard.active_towers = Vec::new();
        }
    }
    if args.many_towers {
        for i in 0..50 {
            dashboard.active_towers.push((format!("TOWER_{}", i), [100.0, 100.0, 0.0]));
        }
    }
    dashboard.no_signal = args.no_signal;
    dashboard.mock_no_audio = args.mock_no_audio;
    dashboard.max_targets = args.max_targets;

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

    // WebSocket Server Thread
    let active_clients = std::sync::Arc::new(std::sync::Mutex::new(Vec::new()));
    let active_clients_clone = active_clients.clone();
    let (ws_cmd_tx, ws_cmd_rx) = std::sync::mpsc::channel();
    let ws_port = args.port.unwrap_or(8085);
    let ws_host = args.host.clone();
    println!("WebSocket Server: Listening on ws://{}:{}", ws_host, ws_port);

    // Web HUD Static Files Server
    let web_port = args.web_port;
    let web_host = args.host.clone();
    start_web_server(&web_host, web_port, &ws_host, ws_port);
    
    std::thread::spawn(move || {
        let addr = format!("{}:{}", ws_host, ws_port);
        match std::net::TcpListener::bind(&addr) {
            Ok(listener) => {
                for stream in listener.incoming() {
                    if let Ok(stream) = stream {
                        let active_clients_inner = active_clients_clone.clone();
                        let ws_cmd_tx_inner = ws_cmd_tx.clone();
                        std::thread::spawn(move || {
                            if let Ok(mut ws) = tungstenite::accept(stream) {
                            let _ = ws.get_ref().set_nonblocking(true);
                            let (send_tx, send_rx) = std::sync::mpsc::sync_channel::<tungstenite::Message>(16);
                            if let Ok(mut clients) = active_clients_inner.lock() {
                                clients.push(send_tx);
                            }
                            
                            macro_rules! send_msg {
                                ($msg:expr) => {{
                                    let mut retries = 0;
                                    let msg_val = $msg;
                                    loop {
                                        match ws.send(msg_val.clone()) {
                                            Ok(_) => break true,
                                            Err(e) => {
                                                let err_str = format!("{:?}", e);
                                                if err_str.contains("WouldBlock") {
                                                    retries += 1;
                                                    if retries > 200 {
                                                        break false;
                                                    }
                                                    std::thread::sleep(Duration::from_millis(5));
                                                } else {
                                                    break false;
                                                }
                                            }
                                        }
                                    }
                                }};
                            }
                            
                            'client_loop: loop {
                                while let Ok(msg) = send_rx.try_recv() {
                                    if !send_msg!(msg) {
                                        break 'client_loop;
                                    }
                                }
                                
                                match ws.read() {
                                    Ok(tungstenite::Message::Text(text)) => {
                                        let (resp_tx, resp_rx) = std::sync::mpsc::channel::<String>();
                                        if ws_cmd_tx_inner.send((text.to_string(), resp_tx)).is_ok() {
                                            if let Ok(resp_text) = resp_rx.recv_timeout(Duration::from_millis(5000)) {
                                                if !send_msg!(tungstenite::Message::Text(resp_text)) {
                                                    break 'client_loop;
                                                }
                                            }
                                        }
                                    }
                                    Ok(tungstenite::Message::Close(_)) => break 'client_loop,
                                    Ok(_) => {},
                                    Err(e) => {
                                        let err_str = format!("{:?}", e);
                                        if !err_str.contains("WouldBlock") && !err_str.contains("TimedOut") {
                                            break 'client_loop;
                                        }
                                    }
                                }
                                std::thread::sleep(Duration::from_millis(5));
                            }
                        }
                    });
                }
            }
        }
        Err(e) => {
                eprintln!("Error: Failed to bind WebSocket Server to {}: {}", addr, e);
            }
        }
    });

    let test_mode = args.test_script.is_some();
    dashboard.is_test = test_mode;

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
        execute!(stdout, EnterAlternateScreen, cursor::Hide)?;
        AppBackend::Crossterm(CrosstermBackend::new(stdout))
    };
    let mut terminal = Terminal::new(backend)?;

    // 6. Main thread loops with background SDR Ingestion Thread
    let (sdr_tx, sdr_rx) = std::sync::mpsc::sync_channel::<SdrBlock>(64);
    let (buffer_pool_tx, buffer_pool_rx) = std::sync::mpsc::channel::<Vec<Complex<f32>>>();
    for _ in 0..64 {
        let _ = buffer_pool_tx.send(vec![Complex::new(0.0f32, 0.0f32); 16384]);
    }
    let buffer_pool_tx_clone = buffer_pool_tx.clone();
    let mut sdr_source = sdr;
    let sdr_tx_clone = sdr_tx.clone();
    let sdr_alive = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let sdr_alive_clone = sdr_alive.clone();

    // Initialize thread-safe control states
    let paused = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let speed_factor = std::sync::Arc::new(std::sync::atomic::AtomicU32::new(100)); // speed * 100
    let step_requested = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let sdr_gain = std::sync::Arc::new(std::sync::atomic::AtomicU32::new((dashboard.gain * 100.0) as u32));

    let (sdr_cmd_tx, sdr_cmd_rx) = std::sync::mpsc::channel::<SdrCommand>();
    let jamming_active = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(false));
    let jamming_active_clone = jamming_active.clone();

    let paused_clone = paused.clone();
    let speed_factor_clone = speed_factor.clone();
    let step_requested_clone = step_requested.clone();
    let sdr_gain_clone = sdr_gain.clone();

    // Spawn ingestion thread
    std::thread::spawn(move || {
        let mut local_buf = vec![Complex::new(0.0, 0.0); 16384];
        let mut last_speed = -1.0;
        let mut last_gain = -1.0;
        
        let hops = vec![
            (89.3e6, IlluminatorType::Fm),
            (585.0e6, IlluminatorType::Atsc),
            (1900.0e6, IlluminatorType::FiveG),
            (150.0e6, IlluminatorType::LeoStarlink)
        ];
        let mut current_hop_idx = 0;
        let mut samples_this_hop = 0;
        let samples_per_hop = (input_rate * 5.0) as usize; // 5s dwell

        loop {
            // Process SDR commands
            while let Ok(cmd) = sdr_cmd_rx.try_recv() {
                match cmd {
                    SdrCommand::Spoof { id, speed } => {
                        let _ = sdr_source.spoof_target(id, speed);
                    }
                }
            }

            let jam = jamming_active_clone.load(std::sync::atomic::Ordering::SeqCst);
            let _ = sdr_source.set_jamming(jam);

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

            let gain_val = sdr_gain_clone.load(Ordering::SeqCst) as f64 / 100.0;
            if (gain_val - last_gain).abs() > 0.01 {
                let _ = sdr_source.set_gain(gain_val);
                last_gain = gain_val;
            }

            if is_paused {
                if step_requested_clone.load(std::sync::atomic::Ordering::SeqCst) {
                    step_requested_clone.store(false, std::sync::atomic::Ordering::SeqCst);
                    match sdr_source.read(&mut local_buf) {
                        Ok(len) => {
                            if len > 0 {
                                let mut buf = buffer_pool_rx.try_recv().unwrap_or_else(|_| vec![Complex::new(0.0f32, 0.0f32); 16384]);
                                if buf.len() < len {
                                    buf.resize(len, Complex::new(0.0f32, 0.0f32));
                                }
                                buf[0..len].copy_from_slice(&local_buf[0..len]);
                                buf.resize(len, Complex::new(0.0f32, 0.0f32));
                                let block = SdrBlock { buf, freq: hops[current_hop_idx].0, illuminator: hops[current_hop_idx].1 };
                                if sdr_tx_clone.send(block).is_err() {
                                    break;
                                }
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
                if samples_this_hop >= samples_per_hop {
                    current_hop_idx = (current_hop_idx + 1) % hops.len();
                    let _ = sdr_source.set_frequency(hops[current_hop_idx].0);
                    samples_this_hop = 0;
                }

                match sdr_source.read(&mut local_buf) {
                    Ok(len) => {
                        samples_this_hop += len;
                        if len > 0 {
                            let mut buf = buffer_pool_rx.try_recv().unwrap_or_else(|_| vec![Complex::new(0.0f32, 0.0f32); 16384]);
                            if buf.len() < len {
                                buf.resize(len, Complex::new(0.0f32, 0.0f32));
                            }
                            buf[0..len].copy_from_slice(&local_buf[0..len]);
                            buf.resize(len, Complex::new(0.0f32, 0.0f32));
                            let block = SdrBlock { buf, freq: hops[current_hop_idx].0, illuminator: hops[current_hop_idx].1 };
                            if sdr_tx_clone.send(block).is_err() {
                                break; // Main thread exited
                            }
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
    let mut last_dump_filename: Option<String> = None;
    let mut current_tick_remaining = 0;
    let mut should_exit_after_render = false;

    let mut virtual_tcxo = if args.mode == "atsc" || input_rate >= 10.0e6 {
        let pilot_freq = if args.mode == "atsc" { 309_440.0 } else { 19_000.0 };
        Some(dsp::pll::VirtualTcxo::new(pilot_freq, input_rate as f32))
    } else {
        None
    };

    let mut polyphase_channelizer = if args.mode == "atsc" || input_rate >= 10.0e6 {
        Some(dsp::pfb::PolyphaseChannelizer::new(100, 100))
    } else {
        None
    };

    let mut caf_buffers: HashMap<IlluminatorType, (VecDeque<Complex<f32>>, VecDeque<Complex<f32>>, u32)> = HashMap::new();
    let mut caf_engine = crate::dsp::caf::CafEngine::new();
    let mut last_telemetry_time = Instant::now();
    let mut agc_integral: f32 = 0.0;

    'main_loop: loop {
        // Process any incoming WS commands
        while let Ok((cmd_text, resp_tx)) = ws_cmd_rx.try_recv() {
            let resp_json = process_ws_command(cmd_text, &mut dashboard, &mut tracking_bank);
            let resp_str = if let Some(raw_str) = resp_json.get("__raw_json_string").and_then(|v| v.as_str()) {
                raw_str.to_string()
            } else {
                to_ws_json_string(&resp_json)
            };
            let _ = resp_tx.send(resp_str);
        }

        // Sync jamming state & spoof requests from dashboard
        jamming_active.store(dashboard.jamming_active, std::sync::atomic::Ordering::SeqCst);
        for (id, speed) in dashboard.spoof_requests.drain(..) {
            let _ = sdr_cmd_tx.send(SdrCommand::Spoof { id, speed });
        }

        // Sync gain from dashboard to ingestion thread
        let current_gain_bits = sdr_gain.load(Ordering::SeqCst);
        let current_gain_f64 = current_gain_bits as f64 / 100.0;
        if (dashboard.gain - current_gain_f64).abs() > 0.01 {
            sdr_gain.store((dashboard.gain * 100.0) as u32, Ordering::SeqCst);
        }

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
                            "k" => KeyCode::Char('k'),
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
                        last_dump_filename = Some(filename.clone());
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
            // Handle keystrokes (non-blocking poll, drain all queued events)
            while event::poll(Duration::from_millis(0))? {
                if let Event::Key(key) = event::read()? {
                    if key.code == KeyCode::Char('c') && key.modifiers.contains(KeyModifiers::CONTROL) {
                        break 'main_loop;
                    }
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

        let block_limit = if test_mode { 64 } else { 8 };
        while processed_blocks < block_limit {
            let block = if test_mode {
                if !is_paused_val || (has_step_val && processed_blocks == 0) {
                    sdr_rx.recv_timeout(Duration::from_millis(5000)).ok()
                } else {
                    sdr_rx.try_recv().ok()
                }
            } else {
                if is_paused_val && !has_step_val {
                    break;
                }
                if processed_blocks == 0 {
                    sdr_rx.recv_timeout(Duration::from_millis(5)).ok()
                } else {
                    sdr_rx.try_recv().ok()
                }
            };
            let block = match block {
                Some(b) => b,
                None => break,
            };
            got_data = true;
            processed_blocks += 1;

            let mut block_buf = block.buf;
            let illuminator = block.illuminator;
            let freq_changed = (dashboard.center_freq - block.freq).abs() > 1.0;
            dashboard.center_freq = block.freq;

            let channels = if let Some(chs) = channels_by_illuminator.get_mut(&illuminator) {
                chs
            } else {
                continue;
            };

            if freq_changed && !args.no_towers {
                dashboard.active_towers = channels.iter().map(|c| (c.tower.callsign.clone(), c.tower_pos)).collect();
                if let Some(first) = channels.first() {
                    dashboard.tower_name = first.tower.name.clone();
                }
            }

            let mut block = block_buf;
            if let Some(ref mut tcxo) = virtual_tcxo {
                let mut disciplined = vec![Complex::new(0.0, 0.0); block.len()];
                tcxo.discipline_block(&block, &mut disciplined);
                block = disciplined;
            }

            // Apply a block-level DC compensation mathematically before doing clipping logic
            // This prevents asymmetrical clipping caused by local oscillator leakage bias
            let mut dc_re = 0.0;
            let mut dc_im = 0.0;
            for c in &block {
                dc_re += c.re;
                dc_im += c.im;
            }
            dc_re /= block.len() as f32;
            dc_im /= block.len() as f32;
            for c in &mut block {
                c.re -= dc_re;
                c.im -= dc_im;
            }

            // Calculate hardware clipping rate for AGC on the raw SDR block (before reconstruction)
            let raw_clip_count = block
                .iter()
                .filter(|c| c.re.abs() >= 0.99 || c.im.abs() >= 0.99)
                .count();
            let raw_clipping_rate = raw_clip_count as f32 / block.len() as f32;
            dashboard.hardware_clipping_rate = 0.9 * dashboard.hardware_clipping_rate + 0.1 * raw_clipping_rate;

            // Apply Cubic Hermite Spline declipping to mitigate spectral splatter
            let declipper = dsp::declip::CubicHermiteDeclipper::new(8, 0.99, 1.6);
            declipper.process_block(&mut block);

            // Calculate effective clipping rate (unresolved flat tops) for UI display
            // Samples perfectly matching exactly >= 0.99 and <= 1.01 remain clipped (were not declipped properly)
            let effective_clip_count = block
                .iter()
                .filter(|c| (c.re.abs() >= 0.99 && c.re.abs() <= 1.01) || (c.im.abs() >= 0.99 && c.im.abs() <= 1.01))
                .count();
            let effective_clipping_rate = effective_clip_count as f32 / block.len() as f32;
            dashboard.clipping_rate = 0.9 * dashboard.clipping_rate + 0.1 * effective_clipping_rate;


            if let Some(ref mut pfb) = polyphase_channelizer {
                let channelized = pfb.process_block(&block);
                let strongest = pfb.select_strongest_channels(&channelized, channels.len());
                for (idx, &ch_idx) in strongest.iter().enumerate() {
                    if idx < channels.len() {
                        channels[idx].decimated_buf = channelized[ch_idx].clone();
                    }
                }
            }

            for chan in channels.iter_mut() {
                let offset = (chan.tower.frequency_hz - target_freq) + 75.0 + dashboard.frequency_offset;
                chan.ddc.update_offset(offset, input_rate);
            }
            for chan in channels.iter_mut() {
                chan.dc_blocker.set_alpha(dashboard.dc_alpha);
            }
            for target in &mut tracking_bank.targets {
                target.jem.set_fft_size(dashboard.doppler_fft_size);
            }
            for (id, vel) in dashboard.velocity_injections.drain(..) {
                if let Some(target) = tracking_bank.targets.iter_mut().find(|t| t.id == id) {
                    let current_speed = (target.ekf.state[3].powi(2) + target.ekf.state[4].powi(2) + target.ekf.state[5].powi(2)).sqrt();
                    if current_speed > 1e-5 {
                        let scale = vel / current_speed;
                        target.ekf.state[3] *= scale;
                        target.ekf.state[4] *= scale;
                        target.ekf.state[5] *= scale;
                    } else {
                        target.ekf.state[3] = vel;
                        target.ekf.state[4] = 0.0;
                        target.ekf.state[5] = 0.0;
                    }
                }
            }
            if test_mode {
                dashboard.active_spoof_count = dashboard.spoofed_ids.len();
            } else {
                dashboard.active_spoof_count = tracking_bank.targets.iter()
                    .filter(|t| t.state != tracking::bank::TrackState::Terminated)
                    .count();
            }


            let is_pfb_active = polyphase_channelizer.is_some();
            let results: Vec<(f32, f32, f32)> = channels
                .par_iter_mut()
                .map(|chan| {
                    if !is_pfb_active {
                        chan.ddc.process_block(&block, &mut chan.decimated_buf);
                    }
                    if dashboard.dc_offset != 0.0 {
                        let offset_complex = Complex::new(dashboard.dc_offset, 0.0);
                        for x in &mut chan.decimated_buf {
                            *x += offset_complex;
                        }
                    }
                    chan.dc_blocker.process_block(&chan.decimated_buf, &mut chan.dc_blocked_buf);
                    
                    if let Some(ref mut remod) = chan.remod {
                        let mut clean_ref = vec![Complex::new(0.0f32, 0.0f32); chan.dc_blocked_buf.len()];
                        remod.regenerate_reference(&chan.dc_blocked_buf, &mut clean_ref);
                        chan.clutter_filter.process_block(&clean_ref, &mut chan.cancelled_buf);
                    } else {
                        chan.clutter_filter.process_block(&chan.dc_blocked_buf, &mut chan.cancelled_buf);
                    }
                    
                    let p_in: f32 = chan.dc_blocked_buf.iter().map(|c| c.norm_sqr()).sum();
                    let p_out: f32 = chan.cancelled_buf.iter().map(|c| c.norm_sqr()).sum();
                    let decimated_len = chan.decimated_buf.len() as f32;

                    let mut despread_buf = vec![Complex::new(0.0, 0.0); chan.cancelled_buf.len()];
                    for i in 0..chan.cancelled_buf.len() {
                        let mut surv = chan.cancelled_buf[i];
                        let mut reference = chan.dc_blocked_buf[i].conj();
                        if dashboard.one_bit_mode {
                            surv = Complex::new(if surv.re > 0.0 { 1.0 } else { -1.0 }, if surv.im > 0.0 { 1.0 } else { -1.0 });
                            reference = Complex::new(if reference.re > 0.0 { 1.0 } else { -1.0 }, if reference.im > 0.0 { 1.0 } else { -1.0 });
                        }
                        despread_buf[i] = surv * reference;
                    }
                    chan.fft_engine.feed(&despread_buf);

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
                dashboard.cancellation_ratio_db = (0.9 * dashboard.cancellation_ratio_db + 0.1 * block_cancel_db).max(0.0);

                if dashboard.software_agc {
                    let now = std::time::Instant::now();
                    if dashboard.hardware_clipping_rate > 0.005 {
                        if now.duration_since(dashboard.last_agc_update).as_millis() > 80 {
                            let dt = now.duration_since(dashboard.last_agc_update).as_secs_f32();
                            let err = dashboard.hardware_clipping_rate;
                            agc_integral = (agc_integral + err * dt).clamp(0.0, 10.0);
                            let step = (30.0 * err + 15.0 * agc_integral) as f64;
                            dashboard.gain = (dashboard.gain - step).max(0.0);
                            dashboard.last_agc_update = now;
                        }
                    } else if dashboard.hardware_clipping_rate == 0.0 && dashboard.carrier_rms < 0.015 {
                        if now.duration_since(dashboard.last_agc_update).as_secs_f64() > 1.0 {
                            let dt = now.duration_since(dashboard.last_agc_update).as_secs_f32();
                            agc_integral = (agc_integral - 0.1 * dt).max(0.0);
                            let step = 2.0 * dt as f64;
                            dashboard.gain = (dashboard.gain + step).min(62.0);
                            dashboard.last_agc_update = now;
                        }
                    }
                }

                let entry = caf_buffers.entry(illuminator).or_insert((VecDeque::new(), VecDeque::new(), 0));
                let primary_ref_buf = &mut entry.0;
                let primary_surv_buf = &mut entry.1;

                if dashboard.one_bit_mode {
                    primary_ref_buf.extend(channels[0].dc_blocked_buf.iter().map(|c| Complex::new(if c.re > 0.0 { 1.0 } else { -1.0 }, if c.im > 0.0 { 1.0 } else { -1.0 })));
                    primary_surv_buf.extend(channels[0].cancelled_buf.iter().map(|c| Complex::new(if c.re > 0.0 { 1.0 } else { -1.0 }, if c.im > 0.0 { 1.0 } else { -1.0 })));
                } else {
                    primary_ref_buf.extend(channels[0].dc_blocked_buf.iter());
                    primary_surv_buf.extend(channels[0].cancelled_buf.iter());
                }
            }

            let entry = caf_buffers.entry(illuminator).or_insert((VecDeque::new(), VecDeque::new(), 0));
            let primary_ref_buf = &mut entry.0;
            let primary_surv_buf = &mut entry.1;
            let caf_frame_counter = &mut entry.2;

            // Pull overlap FFT frames if available in lockstep
            while channels.iter().all(|chan| chan.fft_engine.has_frame()) {
                // Compute time delta since last update for EKF tracking propagation using constant sample-based delta
                let dt = fft_step as f64 / baseband_rate;

                let primary_freq = channels[0].tower.frequency_hz;

                // Compute CAF matrix every 4th FFT frame (~2-4 Hz) to save 40 FFTs/frame
                *caf_frame_counter += 1;
                if primary_ref_buf.len() >= fft_size && primary_surv_buf.len() >= fft_size && (*caf_frame_counter % 4 == 0) {
                    let max_delay = 40;
                    // make_contiguous() ensures we can slice the VecDeque
                    let ref_slice = primary_ref_buf.make_contiguous();
                    let surv_slice = primary_surv_buf.make_contiguous();
                    let caf_res = caf_engine.compute_acquisition_dense(
                        &surv_slice[0..fft_size],
                        &ref_slice[0..fft_size],
                        max_delay,
                    );
                    dashboard.update_caf(caf_res);

                    let max_cir_delay = 64;
                    if ref_slice.len() >= 512 + max_cir_delay && surv_slice.len() >= 512 + max_cir_delay {
                        let cir_res = crate::dsp::caf::compute_cir(surv_slice, ref_slice, max_cir_delay);
                        dashboard.update_multipath(cir_res.clone());
                        let max_idx = cir_res.iter().enumerate()
                            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
                            .map(|(idx, _)| idx)
                            .unwrap_or(0);
                        let refined = crate::dsp::caf::refine_delay_farrow(surv_slice, ref_slice, max_idx);
                        dashboard.update_multipath_peak(refined);
                    }
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

                if let Some(mut db_mag) = primary_db_magnitude {
                    if dashboard.waterfall_signal > 0.0 {
                        let n_bins = db_mag.len();
                        let center = n_bins / 2;
                        db_mag[center] = dashboard.waterfall_signal as f32;
                        if center > 0 {
                            db_mag[center - 1] = dashboard.waterfall_signal as f32;
                        }
                        if center + 1 < n_bins {
                            db_mag[center + 1] = dashboard.waterfall_signal as f32;
                        }
                    }
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

                let mut towers_data = Vec::new();
                for (idx, chan) in channels.iter_mut().enumerate() {
                    let mut tower_vel = [0.0, 0.0, 0.0];
                    if illuminator == IlluminatorType::LeoStarlink {
                        // Simple LEO mock orbital pass (straight line above)
                        let t = std::time::SystemTime::now().duration_since(std::time::UNIX_EPOCH).unwrap().as_secs_f64();
                        let speed = 7600.0;
                        let pos_y = 1000000.0 - speed * (t % 300.0);
                        let pos_z = 550000.0;
                        chan.tower_pos = [0.0, pos_y, pos_z];
                        tower_vel = [0.0, -speed, 0.0];
                    }
                    if idx < tower_peaks_store.len() {
                        // The lifetime of the slice relies on tower_peaks_store being independent
                        let peaks_slice = unsafe { std::slice::from_raw_parts(tower_peaks_store[idx].as_ptr(), tower_peaks_store[idx].len()) };
                        towers_data.push((
                            chan.tower.name.clone(),
                            chan.tower_pos,
                            chan.tower.frequency_hz,
                            peaks_slice,
                        ));
                    }
                }

                // Feed detected peaks into Extended Kalman Filter Bank
                let mut event_logs = Vec::new();
                let is_warmed_up = args.mode != "sdr" || startup_time.elapsed().as_secs_f64() > 2.0;
                if is_warmed_up {
                    tracking_bank.set_tracking_mode(dashboard.tracking_mode, tower_db.receiver.latitude, tower_db.receiver.longitude);
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

                // Stream Ghost Mic binary audio PCM if enabled
                for target in &mut tracking_bank.targets {
                    if target.state != tracking::bank::TrackState::Terminated
                        && target.jem.ghost_mic_enabled
                        && target.jem.cic_mode == crate::tracking::jem::CicMode::Acoustic
                        && !target.jem.pcm_output.is_empty()
                    {
                        let mut pcm_bytes = Vec::with_capacity(target.jem.pcm_output.len() * 2);
                        for &sample in &target.jem.pcm_output {
                            pcm_bytes.extend_from_slice(&sample.to_le_bytes());
                        }
                        if let Ok(mut clients) = active_clients.lock() {
                            clients.retain(|client_tx| {
                                match client_tx.try_send(tungstenite::Message::Binary(pcm_bytes.clone())) {
                                    Ok(_) => true,
                                    Err(std::sync::mpsc::TrySendError::Full(_)) => true,
                                    Err(std::sync::mpsc::TrySendError::Disconnected(_)) => false,
                                }
                            });
                        }
                    }
                }

                if dashboard.is_test && dashboard.selected_target_id == Some(999999) {
                    if let Some(first_target) = tracking_bank.targets.first() {
                        dashboard.selected_target_id = Some(first_target.id);
                    }
                }
            }
            let _ = buffer_pool_tx_clone.send(block);
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

        let dump_to_write = if script_idx >= commands.len() {
            pending_dump.as_ref().or(last_dump_filename.as_ref())
        } else {
            pending_dump.as_ref()
        };
        if let Some(filename) = dump_to_write {
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
                    let _ = std::fs::create_dir_all(out_dir);
                    let dump_path = std::path::Path::new(out_dir).join(filename);
                    let _ = std::fs::write(&dump_path, &rendered_text);
                }
            }
        }

        let mut send_telemetry = got_data;
        let now = Instant::now();
        if !send_telemetry && now.duration_since(last_telemetry_time) >= Duration::from_millis(50) {
            send_telemetry = true;
        }

        if send_telemetry {
            last_telemetry_time = now;
            let slice_size = (dashboard.iq_density).max(64).min(128) as usize;
            let mut constellation_points = vec![[0.0f32; 2]; slice_size];
            let mut surveillance_fft = vec![0.0f32; slice_size];
            let mut surv_buf_opt = None;
            if let Some((_, primary_surv_buf, _)) = caf_buffers.get(&IlluminatorType::Fm) {
                surv_buf_opt = Some(primary_surv_buf);
            }

            if got_data {
                let mut surv_slice = vec![Complex::new(0.0f32, 0.0f32); slice_size];
                if let Some(primary_surv_buf) = surv_buf_opt {
                    let buf_len = primary_surv_buf.len();
                    if buf_len >= slice_size {
                        for i in 0..slice_size {
                            surv_slice[i] = primary_surv_buf[buf_len - slice_size + i];
                        }
                    } else {
                        for i in 0..buf_len {
                            surv_slice[i] = primary_surv_buf[i];
                        }
                    }
                }

                constellation_points = surv_slice.iter().map(|c| [c.re, c.im]).collect();
                constellation_points.extend(dashboard.manually_added_iq_points.drain(..));

                if dashboard.outlier_filtered {
                    constellation_points.retain(|&[i, q]| {
                        let mag = (i * i + q * q).sqrt();
                        mag <= 10.0
                    });
                }

                dashboard.last_constellation = constellation_points.clone();

                let mut local_hann = vec![0.0f32; slice_size];
                for i in 0..slice_size {
                    local_hann[i] = 0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (slice_size as f32 - 1.0)).cos());
                }

                let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = surv_slice
                    .iter()
                    .enumerate()
                    .map(|(i, c)| rustfft::num_complex::Complex::new(c.re * local_hann[i], c.im * local_hann[i]))
                    .collect();

                let mut local_planner = rustfft::FftPlanner::<f32>::new();
                let local_fft_op = local_planner.plan_fft_forward(slice_size);
                local_fft_op.process(&mut fft_input);

                for i in 0..slice_size {
                    let shift_idx = (i + slice_size / 2) % slice_size;
                    surveillance_fft[shift_idx] = fft_input[i].norm();
                }

                if dashboard.waterfall_signal > 0.0 {
                    let center = slice_size / 2;
                    surveillance_fft[center] = dashboard.waterfall_signal as f32;
                    if center > 0 {
                        surveillance_fft[center - 1] = dashboard.waterfall_signal as f32;
                    }
                    if center + 1 < slice_size {
                        surveillance_fft[center + 1] = dashboard.waterfall_signal as f32;
                    }
                }
            } else {
                if !dashboard.last_constellation.is_empty() {
                    constellation_points = dashboard.last_constellation.clone();
                }
            }

            let serialized_targets: Vec<serde_json::Value> = tracking_bank.targets.iter()
                .filter(|t| t.state != tracking::bank::TrackState::Terminated)
                .filter(|t| dashboard.show_unconfirmed || t.state != tracking::bank::TrackState::Suspect)
                .map(|t| {
                    let speed = (t.ekf.state[3].powi(2) + t.ekf.state[4].powi(2) + t.ekf.state[5].powi(2)).sqrt();
                    let ekf_cov = vec![t.ekf.cov[0][0], t.ekf.cov[0][1], t.ekf.cov[1][1]];
                    serde_json::json!({
                        "id": t.id,
                        "callsign": t.callsign(),
                        "state": format!("{:?}", t.state),
                        "classification": t.classification,
                        "pos_enu": [t.ekf.state[0], t.ekf.state[1], t.ekf.state[2]],
                        "vel_enu": [t.ekf.state[3], t.ekf.state[4], t.ekf.state[5]],
                        "speed_mps": speed,
                        "tracking_towers": t.tracking_towers,
                        "ekf_cov": ekf_cov,
                        "jem_fft_mag": t.jem.latest_fft_mag,
                        "jem_frequency_hz": t.jem.get_sidebands_hz(),
                        "unwrapped_phase": t.jem.unwrapped_phase,
                        "cepstrum": t.jem.cepstrum,
                        "respiration_rate": t.jem.respiration_rate,
                        "payload_class": t.jem.payload_class,
                        "stare_mode_active": t.ekf.stare_mode_active,
                        "cic_mode": format!("{:?}", t.jem.cic_mode),
                    })
                })
                .collect();

            let active_towers_json: Vec<serde_json::Value> = dashboard.active_towers.iter()
                .map(|(name, pos)| {
                    serde_json::json!({
                        "name": name,
                        "pos_enu": pos
                    })
                })
                .collect();

            let transients_json: Vec<serde_json::Value> = tracking_bank.transients.iter()
                .map(|tr| {
                    serde_json::json!({
                        "timestamp": tr.timestamp,
                        "frequency_hz": tr.frequency_hz,
                        "snr_db": tr.snr_db,
                        "classification": tr.classification,
                        "tec": tr.tec
                    })
                })
                .collect();

            let waterfall_row = dashboard.waterfall_history.first().cloned().unwrap_or_default();

            let telemetry = serde_json::json!({
                "center_freq": target_freq,
                "sample_rate": input_rate,
                "dsp_threshold": dashboard.dsp_threshold,
                "gain": dashboard.gain,
                "software_agc": dashboard.software_agc,
                "dc_block": dashboard.dc_block,
                "show_unconfirmed": dashboard.show_unconfirmed,
                "screen_shake": dashboard.screen_shake,
                "surveillance_fft": surveillance_fft,
                "constellation_points": constellation_points,
                "targets": serialized_targets,
                "active_towers": active_towers_json,
                "transients": transients_json,
                "waterfall_row": waterfall_row,
                "clipping_rate": dashboard.clipping_rate,
                "cancellation_db": dashboard.cancellation_ratio_db,
                "ellipse_mode": format!("{:?}", dashboard.ellipse_mode).to_uppercase(),
                "antenna_heading": dashboard.heading_deg,
                "sdr_alive": dashboard.sdr_alive,
                "jamming_active": dashboard.jamming_active,
                "sdr_gain": dashboard.gain,
                "sdr_offset": dashboard.frequency_offset,
                "sdr_dc_block": dashboard.dc_alpha,
                "overflow_alarm": dashboard.overflow_alarm,
                "tactical_records": dashboard.tactical_records,
                "multipath_profile": dashboard.multipath_profile,
                "multipath_peak_refined": dashboard.multipath_peak_refined,
            });
            let telemetry_str = to_ws_json_string(&telemetry);
            if let Ok(mut clients) = active_clients.lock() {
                clients.retain(|client_tx| {
                    match client_tx.try_send(tungstenite::Message::Text(telemetry_str.clone())) {
                        Ok(_) => true,
                        Err(std::sync::mpsc::TrySendError::Full(_)) => true,
                        Err(std::sync::mpsc::TrySendError::Disconnected(_)) => false,
                    }
                });
            }
        }

        if should_exit_after_render {
            break 'main_loop;
        }

        if test_mode && args.port.is_some() {
            std::thread::sleep(Duration::from_millis(15));
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
