use crate::tracking::bank::TrackedTarget;
use ratatui::{
    layout::{Constraint, Direction, Layout, Rect},
    style::{Color, Modifier, Style},
    text::{Line, Span},
    widgets::canvas::{Canvas, Circle, Line as CanvasLine},
    widgets::{Block, Borders, Paragraph, Row, Table},
    Frame,
};

fn snr_to_color(db: f32) -> Color {
    let norm = (db.max(0.0).min(30.0) / 30.0) as f64;
    let r = (norm * 255.0) as u8;
    let g = ((1.0 - (norm - 0.5).abs() * 2.0).max(0.0) * 255.0) as u8;
    let b = ((1.0 - norm) * 255.0) as u8;
    Color::Rgb(r, g, b)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EllipseMode {
    None,
    Selected,
    All,
}

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct RecordEntry {
    pub value: f64,
    pub target_id: u32,
    pub classification: String,
    pub callsign: String,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TacticalRecords {
    pub fastest_plane: Option<RecordEntry>,
    pub highest_drone: Option<RecordEntry>,
    pub closest_target: Option<RecordEntry>,
    pub max_cancellation: f64,
    pub max_simultaneous_tracks: usize,
}

impl TacticalRecords {
    pub fn new() -> Self {
        Self {
            fastest_plane: None,
            highest_drone: None,
            closest_target: None,
            max_cancellation: 0.0,
            max_simultaneous_tracks: 0,
        }
    }
}

#[derive(Debug, Clone, Copy)]
pub struct VisiblePanes {
    pub logs: bool,
    pub towers: bool,
    pub waterfall: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum WaterfallMode {
    DopplerTime,
    RangeDoppler,
}

pub struct Dashboard {
    pub waterfall_mode: WaterfallMode,
    pub waterfall_history: Vec<Vec<f32>>,
    pub caf_matrix: Vec<Vec<f32>>,
    pub max_waterfall_rows: usize,
    pub center_freq: f64,
    pub sample_rate: f64,
    pub ddc_offset: f64,
    pub sdr_type: String,
    pub tower_name: String,
    pub tower_pos: [f64; 3],
    pub active_towers: Vec<(String, [f64; 3])>,
    pub compat_mode: bool,
    pub logs: Vec<String>,
    pub sdr_alive: bool,
    pub clipping_rate: f32,
    pub hardware_clipping_rate: f32,
    pub carrier_rms: f32,
    pub cancellation_ratio_db: f32,
    pub visible_panels: VisiblePanes,
    pub selected_target_id: Option<u32>,
    pub paused: bool,
    pub speed_factor: f64,
    /// Current M-of-N candidate pre-tracks; displayed as INIT rows below confirmed tracks.
    pub candidates: Vec<crate::tracking::bank::CandidatePlot>,

    // Rate limiting caches
    pub data_version: u64,
    pub cached_waterfall_version: u64,
    pub last_waterfall_update: Option<std::time::Instant>,
    pub cached_waterfall_area: ratatui::layout::Rect,
    pub cached_waterfall_lines: Vec<ratatui::text::Line<'static>>,

    pub last_table_update: Option<std::time::Instant>,
    pub cached_table_selected_target_id: Option<u32>,
    pub cached_table_area: ratatui::layout::Rect,
    pub cached_table_rows: Vec<ratatui::widgets::Row<'static>>,
    pub cached_show_unconfirmed: bool,

    pub last_logs_update: Option<std::time::Instant>,
    pub cached_logs_area: ratatui::layout::Rect,
    pub cached_logs_lines: Vec<ratatui::text::Line<'static>>,

    pub last_transients_update: Option<std::time::Instant>,
    pub cached_transients_area: ratatui::layout::Rect,
    pub cached_transients_rows: Vec<ratatui::widgets::Row<'static>>,

    pub last_inspection_update: Option<std::time::Instant>,
    pub cached_inspection_target_id: Option<u32>,
    pub cached_inspection_area: ratatui::layout::Rect,
    pub cached_inspection_info: Vec<ratatui::text::Line<'static>>,
    pub cached_inspection_ekf: Vec<ratatui::text::Line<'static>>,
    pub cached_inspection_snr: Vec<ratatui::text::Line<'static>>,
    pub cached_inspection_jem: Vec<ratatui::text::Line<'static>>,
    pub cached_inspection_history: Vec<ratatui::widgets::Row<'static>>,
    pub heading_deg: f64,
    pub ellipse_mode: EllipseMode,
    pub dsp_threshold: f32,
    pub gain: f64,
    pub software_agc: bool,
    pub last_agc_update: std::time::Instant,
    pub dc_block: bool,
    pub show_constellation: bool,
    pub show_aligner: bool,
    pub show_hacker: bool,
    pub one_bit_mode: bool,
    pub crt_mode: bool,
    pub screen_shake: bool,
    pub doppler_scale_pm: bool,
    pub mock_no_audio: bool,
    pub max_targets: bool,
    pub no_signal: bool,
    pub is_test: bool,
    pub ws_port: Option<u16>,
    pub cached_screen_shake: bool,
    pub waterfall_min: i64,
    pub waterfall_max: i64,
    pub waterfall_signal: f64,
    pub constellation_rate: i64,
    pub outlier_filtered: bool,
    pub iq_density: i64,
    pub manually_added_iq_points: Vec<[f32; 2]>,
    pub last_constellation: Vec<[f32; 2]>,
    pub show_jem_spectrogram: bool,
    pub cached_show_jem_spectrogram: bool,
    pub jamming_active: bool,
    pub spoof_requests: Vec<(u32, f64)>,
    pub spoofed_ids: Vec<u32>,
    pub dc_alpha: f32,
    pub dc_offset: f32,
    pub doppler_fft_size: usize,
    pub active_spoof_count: usize,
    pub overflow_alarm: bool,
    pub velocity_injections: Vec<(u32, f64)>,
    pub frequency_offset: f64,
    pub show_records: bool,
    pub tactical_records: TacticalRecords,
    pub show_unconfirmed: bool,
    pub show_multipath: bool,
    pub multipath_profile: Vec<f32>,
    pub multipath_peak_refined: f32,
    pub unwrap_enabled: bool,
    pub cepstrum_enabled: bool,
    pub master_ekf_enabled: bool,
    pub omni_mode: String,
    pub vibrometer_mode: String,
    pub stare_mode_active: bool,
    pub stare_coords: [f64; 3],
    pub ghost_mic_gain: f32,
    pub audio_streaming: bool,
    pub displacement: f64,
    pub last_raw_iq_phase: f64,
    pub unwrapped_phase_accum: f64,
    pub occupancy_confidence: f32,
    pub breathing_rate_hz: f32,
    pub fundamental_rpm: f64,
    pub collapsed_peaks_count: usize,
    pub cepstrum_magnitude: Vec<f32>,
    pub vibration_spectra: Vec<f32>,
    pub peaks: Vec<f64>,
    pub clutter_suppression_db: f64,
    pub spikes_pruned: usize,
    pub dropped_packets: usize,
    pub clipping_occurred: bool,
    pub amplitude_scale: f64,
    pub resonances: Vec<f32>,
    pub drone_heuristics_active: bool,
    pub thrust_to_weight: f64,
    pub payload_class_override: String,
    pub tracking_mode: crate::tracking::ekf::TrackingMode,
    pub is_hopping: bool,
}

impl Dashboard {
    pub fn new(center_freq: f64, sample_rate: f64, ddc_offset: f64, sdr_type: String, heading_deg: f64) -> Self {
        Self {
            waterfall_mode: WaterfallMode::RangeDoppler,
            waterfall_history: Vec::new(),
            caf_matrix: Vec::new(),
            max_waterfall_rows: 200,
            center_freq,
            sample_rate,
            ddc_offset,
            sdr_type: sdr_type.clone(),
            tower_name: "Scanning...".to_string(),
            tower_pos: [0.0, 0.0, 0.0],
            active_towers: Vec::new(),
            compat_mode: false,
            logs: Vec::new(),
            sdr_alive: true,
            clipping_rate: 0.0,
            hardware_clipping_rate: 0.0,
            carrier_rms: 0.0,
            cancellation_ratio_db: 0.0,
            visible_panels: VisiblePanes {
                logs: true,
                towers: true,
                waterfall: true,
            },
            selected_target_id: None,
            paused: false,
            speed_factor: 1.0,
            candidates: Vec::new(),
            data_version: 0,
            cached_waterfall_version: 0,
            last_waterfall_update: None,
            cached_waterfall_area: ratatui::layout::Rect::default(),
            cached_waterfall_lines: Vec::new(),
            last_table_update: None,
            cached_table_selected_target_id: None,
            cached_table_area: ratatui::layout::Rect::default(),
            cached_table_rows: Vec::new(),
            cached_show_unconfirmed: false,
            last_logs_update: None,
            cached_logs_area: ratatui::layout::Rect::default(),
            cached_logs_lines: Vec::new(),
            last_transients_update: None,
            cached_transients_area: ratatui::layout::Rect::default(),
            cached_transients_rows: Vec::new(),
            last_inspection_update: None,
            cached_inspection_target_id: None,
            cached_inspection_area: ratatui::layout::Rect::default(),
            cached_inspection_info: Vec::new(),
            cached_inspection_ekf: Vec::new(),
            cached_inspection_snr: Vec::new(),
            cached_inspection_jem: Vec::new(),
            cached_inspection_history: Vec::new(),
            heading_deg,
            ellipse_mode: EllipseMode::None,
            dsp_threshold: 5.8,
            gain: if sdr_type == "sdr" { 62.0 } else { 6.0 },
            software_agc: true,
            last_agc_update: std::time::Instant::now(),
            dc_block: true,
            show_constellation: true,
            show_aligner: true,
            show_hacker: false,
            one_bit_mode: true,
            crt_mode: false,
            screen_shake: false,
            doppler_scale_pm: false,
            mock_no_audio: false,
            max_targets: false,
            no_signal: false,
            is_test: false,
            ws_port: None,
            cached_screen_shake: false,
            waterfall_min: -100,
            waterfall_max: 0,
            waterfall_signal: 0.0,
            constellation_rate: 30,
            outlier_filtered: false,
            iq_density: 64,
            manually_added_iq_points: Vec::new(),
            last_constellation: Vec::new(),
            show_jem_spectrogram: false,
            cached_show_jem_spectrogram: false,
            jamming_active: false,
            spoof_requests: Vec::new(),
            spoofed_ids: Vec::new(),
            dc_alpha: 0.99f32,
            dc_offset: 0.0f32,
            doppler_fft_size: 256,
            active_spoof_count: 0,
            overflow_alarm: false,
            velocity_injections: Vec::new(),
            frequency_offset: 0.0,
            show_records: false,
            tactical_records: TacticalRecords::new(),
            show_unconfirmed: false,
            show_multipath: true,
            multipath_profile: vec![0.0; 64],
            multipath_peak_refined: 0.0f32,
            unwrap_enabled: false,
            cepstrum_enabled: false,
            master_ekf_enabled: false,
            omni_mode: "None".to_string(),
            vibrometer_mode: "Bypass".to_string(),
            stare_mode_active: false,
            stare_coords: [0.0; 3],
            ghost_mic_gain: 1.0,
            audio_streaming: false,
            displacement: 0.0,
            last_raw_iq_phase: 0.0,
            unwrapped_phase_accum: 0.0,
            occupancy_confidence: 0.0,
            breathing_rate_hz: 0.0,
            fundamental_rpm: 0.0,
            collapsed_peaks_count: 0,
            cepstrum_magnitude: vec![0.0; 256],
            vibration_spectra: vec![0.0; 256],
            peaks: Vec::new(),
            clutter_suppression_db: 55.0,
            spikes_pruned: 0,
            dropped_packets: 0,
            clipping_occurred: false,
            amplitude_scale: 1.0,
            resonances: vec![0.0; 5],
            drone_heuristics_active: false,
            thrust_to_weight: 0.8,
            payload_class_override: "Auto".to_string(),
            tracking_mode: crate::tracking::ekf::TrackingMode::Airspace,
            is_hopping: false,
        }
    }

    fn create_block<'a>(&self, title: &'a str, color: Color) -> Block<'a> {
        let block = Block::default()
            .borders(Borders::ALL)
            .border_style(Style::default().fg(color));
        let block = if title.is_empty() {
            block
        } else {
            block.title(title)
        };
        if self.compat_mode {
            block.border_set(ratatui::symbols::border::Set {
                top_left: "+",
                top_right: "+",
                bottom_left: "+",
                bottom_right: "+",
                vertical_left: "|",
                vertical_right: "|",
                horizontal_top: "-",
                horizontal_bottom: "-",
            })
        } else {
            block
        }
    }

    pub fn add_log(&mut self, log: String) {
        self.logs.push(log);
        if self.logs.len() > 10 {
            self.logs.remove(0);
        }
    }

    /// Add a new spectrum frame to the waterfall history.
    pub fn add_spectrum(&mut self, spectrum: Vec<f32>) {
        self.data_version = self.data_version.wrapping_add(1);
        self.waterfall_history.insert(0, spectrum);
        if self.waterfall_history.len() > self.max_waterfall_rows {
            self.waterfall_history.pop();
        }
    }

    pub fn update_caf(&mut self, caf: Vec<Vec<f32>>) {
        self.data_version = self.data_version.wrapping_add(1);
        self.caf_matrix = caf;
    }

    pub fn update_multipath(&mut self, raw_cir: Vec<f32>) {
        self.data_version = self.data_version.wrapping_add(1);
        if self.multipath_profile.len() != raw_cir.len() {
            self.multipath_profile = raw_cir;
        } else {
            let alpha = 0.1f32;
            for i in 0..self.multipath_profile.len() {
                self.multipath_profile[i] = (1.0 - alpha) * self.multipath_profile[i] + alpha * raw_cir[i];
            }
        }
    }

    pub fn update_multipath_peak(&mut self, peak: f32) {
        self.data_version = self.data_version.wrapping_add(1);
        self.multipath_peak_refined = peak;
    }


    fn wrap_text(&self, text: &str, max_width: usize) -> Vec<String> {
        if max_width == 0 {
            return vec![text.to_string()];
        }
        let mut lines = Vec::new();
        for paragraph in text.split('\n') {
            let mut current_line = String::new();
            for word in paragraph.split_whitespace() {
                if current_line.is_empty() {
                    current_line = word.to_string();
                } else if current_line.len() + 1 + word.len() <= max_width {
                    current_line.push(' ');
                    current_line.push_str(word);
                } else {
                    lines.push(current_line);
                    current_line = word.to_string();
                }
            }
            if !current_line.is_empty() {
                lines.push(current_line);
            }
        }
        if lines.is_empty() {
            lines.push(String::new());
        }
        lines
    }

    fn generate_sparkline(&self, values: &[f64]) -> String {
        if values.is_empty() {
            return "No data".to_string();
        }
        let spark_chars = [' ', '▂', '▃', '▄', '▅', '▆', '▇', '█'];
        let mut min = values[0];
        let mut max = values[0];
        for &v in values {
            if v < min { min = v; }
            if v > max { max = v; }
        }
        let range = max - min;
        
        let mut spark = String::new();
        for &v in values {
            let idx = if range < 1e-5 {
                3
            } else {
                let ratio = (v - min) / range;
                let idx = (ratio * 7.0).round() as usize;
                idx.min(7)
            };
            spark.push(spark_chars[idx]);
        }
        spark
    }

    fn render_ascii_bar_chart(&self, values: &[f32], width: usize, height: usize) -> Vec<String> {
        if values.is_empty() || width == 0 || height == 0 {
            return vec![];
        }
        let mut downsampled = vec![0.0f32; width];
        let chunk_size = values.len() as f64 / width as f64;
        for i in 0..width {
            let start = (i as f64 * chunk_size).floor() as usize;
            let end = (((i + 1) as f64 * chunk_size).floor() as usize).min(values.len());
            let mut max_val = 0.0f32;
            for j in start..end {
                if values[j] > max_val {
                    max_val = values[j];
                }
            }
            downsampled[i] = max_val;
        }

        let max_val = downsampled.iter().copied().fold(0.0f32, |a, b| a.max(b));
        let mut chart_lines = Vec::new();
        
        for h in 0..height {
            let threshold = if max_val > 1e-5 {
                ((height - h) as f32 / height as f32) * max_val
            } else {
                0.0
            };
            let mut line = String::new();
            for &val in &downsampled {
                if max_val > 1e-5 && val >= threshold {
                    line.push('█');
                } else {
                    line.push(' ');
                }
            }
            chart_lines.push(line);
        }
        chart_lines
    }

    pub fn render_target_inspection(
        &mut self,
        frame: &mut Frame,
        area: Rect,
        target: &TrackedTarget,
    ) {
        let title_str = format!(" Inspection & EKF State - Target {} - Micro-Doppler Scope ", target.callsign());
        let block = self.create_block(
            &title_str,
            Color::Green,
        );
        let inner_area = block.inner(area);
        frame.render_widget(block, area);

        let chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(5), // Info
                Constraint::Length(6), // EKF State
                Constraint::Length(3), // SNR sparkline
                Constraint::Min(4),    // JEM Spectrum
                Constraint::Length(7), // History Table
            ])
            .split(inner_area);

        let now = std::time::Instant::now();
        let needs_update = self.last_inspection_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_inspection_area != area
            || self.cached_inspection_target_id != Some(target.id)
            || self.cached_show_jem_spectrogram != self.show_jem_spectrogram;

        if needs_update {
            // Render Info
            let duration = target.start_time.elapsed().as_secs_f64();
            let towers_str = if target.tracking_towers.is_empty() {
                "None".to_string()
            } else {
                target.tracking_towers.join(", ")
            };
            self.cached_inspection_info = vec![
                Line::from(vec![
                    Span::styled("ID: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{}", target.id), Style::default().fg(Color::White)),
                    Span::styled("  State: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(
                        if target.state == crate::tracking::bank::TrackState::Terminated {
                            "Target Terminated".to_string()
                        } else {
                            format!("{:?}", target.state)
                        },
                        Style::default().fg(Color::Green)
                    ),
                ]),
                Line::from(vec![
                    Span::styled("Classification: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{}", target.classification), Style::default().fg(Color::Cyan)),
                ]),
                Line::from(vec![
                    Span::styled("Towers: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(towers_str, Style::default().fg(Color::Magenta)),
                ]),
                Line::from(vec![
                    Span::styled("Duration: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.1}s", duration), Style::default().fg(Color::White)),
                ]),
            ];

            // Render EKF state
            let x_uncert = target.ekf.cov[0][0].sqrt();
            let y_uncert = target.ekf.cov[1][1].sqrt();
            let z_uncert = target.ekf.cov[2][2].sqrt();
            let vx_uncert = target.ekf.cov[3][3].sqrt();
            let vy_uncert = target.ekf.cov[4][4].sqrt();
            let vz_uncert = target.ekf.cov[5][5].sqrt();

            self.cached_inspection_ekf = vec![
                Line::from(Span::styled("--- EKF STATE ESTIMATES & UNCERTAINTIES ---", Style::default().fg(Color::Yellow))),
                Line::from(vec![
                    Span::styled("X: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.2} km ± {:.2} m", target.ekf.state[0] / 1000.0, x_uncert), Style::default().fg(Color::White)),
                    Span::styled("  Y: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.2} km ± {:.2} m", target.ekf.state[1] / 1000.0, y_uncert), Style::default().fg(Color::White)),
                ]),
                Line::from(vec![
                    Span::styled("Z: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.2} km ± {:.2} m", target.ekf.state[2] / 1000.0, z_uncert), Style::default().fg(Color::White)),
                ]),
                Line::from(vec![
                    Span::styled("Vx: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.1} m/s ± {:.1} m/s", target.ekf.state[3], vx_uncert), Style::default().fg(Color::White)),
                    Span::styled("  Vy: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.1} m/s ± {:.1} m/s", target.ekf.state[4], vy_uncert), Style::default().fg(Color::White)),
                    Span::styled("  Vz: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(format!("{:.1} m/s ± {:.1} m/s", target.ekf.state[5], vz_uncert), Style::default().fg(Color::White)),
                ]),
            ];

            // Render SNR profile
            let snr_values: Vec<f64> = target.fingerprint_history.iter().map(|dp| dp.snr_db).collect();
            let current_snr = snr_values.last().copied().unwrap_or(0.0);
            let min_snr = snr_values.iter().copied().fold(f64::INFINITY, |a, b| a.min(b));
            let max_snr = snr_values.iter().copied().fold(f64::NEG_INFINITY, |a, b| a.max(b));
            let spark = self.generate_sparkline(&snr_values);

            self.cached_inspection_snr = vec![
                Line::from(Span::styled("--- SNR PROFILE HISTORY ---", Style::default().fg(Color::Yellow))),
                Line::from(vec![
                    Span::styled("SNR: ", Style::default().fg(Color::DarkGray)),
                    Span::styled(spark, Style::default().fg(Color::Green)),
                    Span::styled(
                        format!(" (min: {:.1} dB, max: {:.1} dB, cur: {:.1} dB)", 
                            if min_snr.is_infinite() { 0.0 } else { min_snr }, 
                            if max_snr.is_infinite() { 0.0 } else { max_snr }, 
                            current_snr
                        ), 
                        Style::default().fg(Color::DarkGray)
                    ),
                ]),
            ];

            // Render JEM Spectrum
            let chart_width = chunks[3].width.saturating_sub(2) as usize;
            let chart_height = chunks[3].height.saturating_sub(2) as usize;
            
            let scale_str = if self.doppler_scale_pm { "Scale: +/-" } else { "Scale: Default" };
            let jem_class = if target.jem.get_sidebands_hz().is_none() {
                "UNKNOWN"
            } else {
                let is_prop = target.classification.contains("Propeller")
                    || target.classification.contains("Turboprop")
                    || target.ekf.state[2] < 3000.0;
                if is_prop {
                    "prop"
                } else {
                    "turbine"
                }
            };

            let title_line = if self.show_jem_spectrogram {
                Line::from(vec![
                    Span::styled("--- JEM Analysis: MODULATION SPECTROGRAM (Waterfall) ---", Style::default().fg(Color::Yellow)),
                    Span::raw("   "),
                    Span::styled(scale_str, Style::default().fg(Color::Cyan)),
                    Span::raw("   "),
                    Span::styled(format!("Class: {}", jem_class), Style::default().fg(Color::Green)),
                ])
            } else {
                Line::from(vec![
                    Span::styled("--- JEM Analysis: MODULATION SPECTRUM (FFT Bins) ---", Style::default().fg(Color::Yellow)),
                    Span::raw("   "),
                    Span::styled(scale_str, Style::default().fg(Color::Cyan)),
                    Span::raw("   "),
                    Span::styled(format!("Class: {}", jem_class), Style::default().fg(Color::Green)),
                ])
            };

            let mut jem_elements = vec![title_line];

            if self.show_jem_spectrogram {
                let history_len = target.jem.history.len();
                let rows_to_take = history_len.min(chart_height);
                let skip_count = history_len.saturating_sub(rows_to_take);

                // Find global max value in history for scaling
                let global_max = target.jem.history.iter()
                    .flat_map(|row| row.iter())
                    .copied()
                    .fold(0.0f32, |a, b| a.max(b));

                for row_idx in 0..chart_height {
                    if row_idx < rows_to_take {
                        let row_data = &target.jem.history[skip_count + row_idx];
                        let chunk_size = row_data.len() as f64 / chart_width as f64;
                        let mut spans = Vec::with_capacity(chart_width);
                        for i in 0..chart_width {
                            let start = (i as f64 * chunk_size).floor() as usize;
                            let end = (((i + 1) as f64 * chunk_size).floor() as usize).min(row_data.len());
                            let mut val = 0.0f32;
                            for j in start..end {
                                if row_data[j] > val {
                                    val = row_data[j];
                                }
                            }

                            let norm = if global_max > 1e-5 {
                                (val / global_max).min(1.0).max(0.0)
                            } else {
                                0.0
                            };
                            let chars = [' ', '.', ':', '-', '=', '+', '*', '%', '#', '█'];
                            let char_idx = (norm * 9.0).round() as usize;
                            let ch = chars[char_idx.min(9)];
                            let color = if norm < 0.1 {
                                Color::DarkGray
                            } else if norm < 0.4 {
                                Color::Green
                            } else if norm < 0.7 {
                                Color::LightGreen
                            } else {
                                Color::Cyan
                            };
                            spans.push(Span::styled(ch.to_string(), Style::default().fg(color)));
                        }
                        jem_elements.push(Line::from(spans));
                    } else {
                        jem_elements.push(Line::from(vec![Span::raw(" ".repeat(chart_width))]));
                    }
                }
            } else {
                let chart_lines = self.render_ascii_bar_chart(&target.jem.latest_fft_mag, chart_width, chart_height);
                for l in chart_lines {
                    jem_elements.push(Line::from(Span::styled(l, Style::default().fg(Color::Cyan))));
                }
            }

            self.cached_inspection_jem = jem_elements;
            self.last_inspection_update = Some(now);
            self.cached_inspection_area = area;
            self.cached_inspection_target_id = Some(target.id);
            self.cached_show_jem_spectrogram = self.show_jem_spectrogram;
        }

        frame.render_widget(Paragraph::new(self.cached_inspection_info.clone()), chunks[0]);
        frame.render_widget(Paragraph::new(self.cached_inspection_ekf.clone()), chunks[1]);
        frame.render_widget(Paragraph::new(self.cached_inspection_snr.clone()), chunks[2]);
        frame.render_widget(Paragraph::new(self.cached_inspection_jem.clone()), chunks[3]);

        // Render History Table
        let history_headers = Row::new(vec![
            "Age",
            "Est X (km)",
            "Est Y (km)",
            "Est Z (km)",
            "Vel (m/s)",
        ]).style(Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD));

        // We could cache history table but it's very cheap since it takes max 5 items.
        // We will just recreate it.
        let history_rows: Vec<Row> = target.history.iter().rev().take(5).enumerate().map(|(i, state)| {
            let x_km = state[0] / 1000.0;
            let y_km = state[1] / 1000.0;
            let z_km = state[2] / 1000.0;
            let vel = (state[3].powi(2) + state[4].powi(2) + state[5].powi(2)).sqrt();
            Row::new(vec![
                Cell::from(format!("-{}", i)),
                Cell::from(format!("{:.2}", x_km)),
                Cell::from(format!("{:.2}", y_km)),
                Cell::from(format!("{:.2}", z_km)),
                Cell::from(format!("{:.1}", vel)),
            ]).style(Style::default().fg(Color::White))
        }).collect();

        use ratatui::widgets::Cell;
        let history_table = Table::new(
            history_rows,
            [
                Constraint::Length(5),
                Constraint::Length(12),
                Constraint::Length(12),
                Constraint::Length(12),
                Constraint::Length(10),
            ],
        )
        .header(history_headers);
        frame.render_widget(history_table, chunks[4]);
    }

    /// Render the dashboard inside a Ratatui frame.
    pub fn render(
        &mut self,
        frame: &mut Frame,
        area: Rect,
        targets: &[TrackedTarget],
        transients: &[crate::tracking::bank::TransientEvent],
    ) {
        self.update_tactical_records(targets);
        self.overflow_alarm = self.gain > 45.0;
        // Divide the UI into:
        // Top: Status bar (Height = 3)
        // Middle: Left (Target Table & Map) and Right (Doppler Waterfall & Transients or Target Inspection)
        let main_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Status bar
                Constraint::Min(0),    // Main visualization
                Constraint::Length(1), // Footer keybinds/states legend
            ])
            .split(area);

        // Status bar
        self.render_status_bar(frame, main_chunks[0]);

        // Sort targets to correctly map selected_target_id — stable by (state_priority, id)
        let sorted_targets = {
            let mut sorted: Vec<&TrackedTarget> = targets.iter().collect();
            let selected_id = self.selected_target_id;
            sorted.sort_by_key(|t| {
                let is_sel = selected_id == Some(t.id);
                let s = match t.state {
                    crate::tracking::bank::TrackState::Active => 0u32,
                    crate::tracking::bank::TrackState::Coasting => 1,
                    crate::tracking::bank::TrackState::Suspect => 2,
                    crate::tracking::bank::TrackState::Terminated => 3,
                };
                (!is_sel, s, t.id)
            });
            sorted
        };

        let selected_target = self.selected_target_id.and_then(|id| sorted_targets.iter().find(|t| t.id == id).copied());

        // Dynamic collapses:
        // If area.width < collapse_threshold, collapse Waterfall and Transients.
        // If area.height < 30, collapse Logs.
        // If area.height < 20, collapse ENU map (Towers).
        let collapse_threshold = if self.ws_port.is_some() { 80 } else { 100 };
        let show_right = area.height >= 10 && area.width >= collapse_threshold && (self.visible_panels.waterfall || selected_target.is_some());
        let show_map = self.visible_panels.towers && area.height >= 20;
        let show_logs = self.visible_panels.logs && area.height >= 30;

        let show_right_col = area.height >= 10 && area.width >= collapse_threshold && (self.show_aligner || self.show_constellation || self.show_hacker || self.show_records || self.show_multipath);

        let mut constraints = Vec::new();
        constraints.push(Constraint::Percentage(34)); // Left column
        if show_right {
            constraints.push(Constraint::Percentage(33)); // Middle column
        }
        if show_right_col {
            constraints.push(Constraint::Percentage(33)); // Right column
        }

        let num_cols = constraints.len();
        let constraints = if num_cols == 1 {
            vec![Constraint::Percentage(100)]
        } else if num_cols == 2 {
            vec![Constraint::Percentage(55), Constraint::Percentage(45)]
        } else {
            vec![Constraint::Percentage(45), Constraint::Percentage(35), Constraint::Percentage(20)]
        };

        let columns = Layout::default()
            .direction(Direction::Horizontal)
            .constraints(constraints)
            .split(main_chunks[1]);

        let left_area = columns[0];
        let mut col_idx = 1;

        let middle_area = if show_right {
            let area = columns[col_idx];
            col_idx += 1;
            Some(area)
        } else {
            None
        };

        let right_area = if show_right_col {
            let area = columns[col_idx];
            Some(area)
        } else {
            None
        };

        // Split Left area vertically based on active panels
        let mut left_constraints = Vec::new();
        left_constraints.push(Constraint::Length(4)); // Airspace Summary panel height
        if show_map && show_logs {
            left_constraints.push(Constraint::Percentage(35)); // Targets
            left_constraints.push(Constraint::Percentage(40)); // Map
            left_constraints.push(Constraint::Percentage(25)); // Logs
        } else if show_map {
            left_constraints.push(Constraint::Percentage(50)); // Targets
            left_constraints.push(Constraint::Percentage(50)); // Map
        } else if show_logs {
            left_constraints.push(Constraint::Percentage(70)); // Targets
            left_constraints.push(Constraint::Percentage(30)); // Logs
        } else {
            left_constraints.push(Constraint::Percentage(100)); // Targets only
        }

        let left_layout = Layout::default()
            .direction(Direction::Vertical)
            .constraints(left_constraints)
            .split(left_area);

        let mut current_idx = 0;
        self.render_airspace_summary(frame, left_layout[current_idx], targets, transients);
        current_idx += 1;

        self.render_targets_table(frame, left_layout[current_idx], targets);
        current_idx += 1;

        if show_map {
            self.render_enu_map(frame, left_layout[current_idx], targets);
            current_idx += 1;
        }

        if show_logs {
            self.render_logs_panel(frame, left_layout[current_idx]);
        }

        // Render Right panel if visible
        if let Some(area) = middle_area {
            if let Some(target) = selected_target {
                self.render_target_inspection(frame, area, target);
            } else {
                let right_chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Percentage(70), // Top: Waterfall
                        Constraint::Percentage(30), // Bottom: Atmospheric events
                    ])
                    .split(area);

                self.render_waterfall(frame, right_chunks[0], targets, transients);
                self.render_transients_table(frame, right_chunks[1], transients);
            }
        }

        // Render Column 3 (Aligner, Constellation, Hacker, Records, Multipath)
        if let Some(area) = right_area {
            let mut right_constraints = Vec::new();
            if self.show_aligner {
                right_constraints.push(Constraint::Percentage(100));
            }
            if self.show_constellation {
                right_constraints.push(Constraint::Percentage(100));
            }
            if self.show_hacker {
                right_constraints.push(Constraint::Percentage(100));
            }
            if self.show_records {
                right_constraints.push(Constraint::Percentage(100));
            }
            if self.show_multipath {
                right_constraints.push(Constraint::Percentage(100));
            }

            let num_right_panes = right_constraints.len();
            let right_constraints = match num_right_panes {
                1 => vec![Constraint::Percentage(100)],
                2 => vec![Constraint::Percentage(50), Constraint::Percentage(50)],
                3 => vec![Constraint::Percentage(33), Constraint::Percentage(33), Constraint::Percentage(34)],
                4 => vec![Constraint::Percentage(25), Constraint::Percentage(25), Constraint::Percentage(25), Constraint::Percentage(25)],
                _ => vec![Constraint::Percentage(20); num_right_panes],
            };

            let right_layout = Layout::default()
                .direction(Direction::Vertical)
                .constraints(right_constraints)
                .split(area);

            let mut pane_idx = 0;
            if self.show_aligner {
                self.render_aligner_panel(frame, right_layout[pane_idx]);
                pane_idx += 1;
            }
            if self.show_constellation {
                self.render_constellation_panel(frame, right_layout[pane_idx]);
                pane_idx += 1;
            }
            if self.show_hacker {
                self.render_hacker_panel(frame, right_layout[pane_idx]);
                pane_idx += 1;
            }
            if self.show_records {
                self.render_records_panel(frame, right_layout[pane_idx]);
                pane_idx += 1;
            }
            if self.show_multipath {
                self.render_multipath_panel(frame, right_layout[pane_idx]);
            }
        }
        self.render_footer(frame, main_chunks[2]);
    }

    fn render_status_bar(&self, frame: &mut Frame, area: Rect) {
        if area.width < 50 {
            let mut status_spans = vec![];
            status_spans.push(Span::raw("CRT: "));
            status_spans.push(Span::styled(
                if self.crt_mode { "ON" } else { "OFF" },
                Style::default().fg(Color::Cyan),
            ));
            let status_text = vec![Line::from(status_spans)];
            let paragraph = Paragraph::new(status_text).block(self.create_block("", Color::DarkGray));
            frame.render_widget(paragraph, area);
            return;
        }

        let carrier_db = 20.0 * self.carrier_rms.max(1e-10).log10();
        let sig_color = if self.carrier_rms < 0.0001 {
            Color::Yellow
        } else if self.clipping_rate > 0.01 {
            Color::LightRed
        } else {
            Color::Green
        };

        let cancel_color = if self.cancellation_ratio_db < 3.0 && self.waterfall_history.len() > 10 {
            Color::Yellow
        } else if self.cancellation_ratio_db > 35.0 {
            Color::Magenta
        } else {
            Color::Green
        };

        let mut status_spans = vec![
            Span::styled(
                " PR HUD ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Yellow),
            ),
        ];

        if self.screen_shake {
            status_spans.push(Span::styled(
                " [SCREEN SHAKE] ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Red),
            ));
        }

        if self.paused {
            status_spans.push(Span::styled(
                " [PAUSED] ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Yellow)
                    .bg(Color::Red),
            ));
        }
        if self.speed_factor != 1.0 {
            status_spans.push(Span::styled(
                format!(" [{:.2}x] ", self.speed_factor),
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Black)
                    .bg(Color::Cyan),
            ));
        }

        status_spans.extend(vec![
            Span::styled(&self.sdr_type, Style::default().fg(Color::Green)),
            Span::raw(" | F: "),
            Span::styled(
                format!("{:.1} MHz", self.center_freq / 1e6),
                Style::default().fg(Color::Cyan),
            ),
            Span::raw(" | R: "),
            Span::styled(
                format!("{:.3} MSPS", self.sample_rate / 1e6),
                Style::default().fg(Color::Cyan),
            ),
            Span::raw(" | T: "),
        ]);

        if self.active_towers.is_empty() {
            status_spans.push(Span::styled(
                self.tower_name.clone(),
                Style::default().fg(Color::Magenta),
            ));
        } else {
            for (idx, (name, _)) in self.active_towers.iter().enumerate() {
                if idx > 0 {
                    status_spans.push(Span::raw(", "));
                }
                let clean_name = name.split('-').next().unwrap_or(name).trim().to_string();
                let color = match idx % 6 {
                    0 => Color::Magenta,
                    1 => Color::Cyan,
                    2 => Color::Yellow,
                    3 => Color::LightRed,
                    4 => Color::Green,
                    5 => Color::LightBlue,
                    _ => Color::DarkGray,
                };
                status_spans.push(Span::styled(clean_name, Style::default().fg(color)));
            }
        }
        status_spans.push(Span::styled(
            format!(" (+{:.0}Hz)", self.ddc_offset),
            Style::default().fg(Color::Magenta),
        ));

        status_spans.extend(vec![
            Span::raw(" | S: "),
            Span::styled(
                format!("{:>6.1} dBFS", carrier_db),
                Style::default().fg(sig_color),
            ),
            Span::raw(" | C: "),
            Span::styled(
                format!("{:>5.1} dB", self.cancellation_ratio_db),
                Style::default().fg(cancel_color),
            ),
            Span::raw(" | Th: "),
            Span::styled(
                format!("{:>4.1}", self.dsp_threshold),
                Style::default().fg(Color::Cyan),
            ),
        ]);

        if self.selected_target_id.is_none() {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                "No Target Selected",
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
            ));
        }

        if self.one_bit_mode {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                " 1-BIT MODE ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Black)
                    .bg(Color::Cyan),
            ));
        }

        status_spans.push(Span::raw(" | MODE: "));
        let mode_str = match self.tracking_mode {
            crate::tracking::ekf::TrackingMode::Airspace => "AIRSPACE",
            crate::tracking::ekf::TrackingMode::GroundCar => "GROUND (CAR)",
            crate::tracking::ekf::TrackingMode::GroundTrain => "GROUND (TRAIN)",
        };
        status_spans.push(Span::styled(
            mode_str,
            Style::default().fg(Color::LightBlue).add_modifier(Modifier::BOLD),
        ));

        if self.is_hopping {
            status_spans.push(Span::raw(" "));
            status_spans.push(Span::styled(
                "[HOPPING]",
                Style::default().fg(Color::LightMagenta).add_modifier(Modifier::BOLD).bg(Color::DarkGray),
            ));
        }

        if !self.sdr_alive {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                " SDR STARVED / OFFLINE ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Red)
                    .bg(Color::White),
            ));
        }

        if self.clipping_rate > 0.01 {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                format!(" ADC CLIPPING ({:.1}%) ", self.clipping_rate * 100.0),
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::White)
                    .bg(Color::Red),
            ));
        }

        if self.carrier_rms < 0.0001 {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                " NO CARRIER / DEAD AIR ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Black)
                    .bg(Color::Yellow),
            ));
        }

        if self.waterfall_history.len() > 10 && self.cancellation_ratio_db < 3.0 {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                " CLUTTER CANCEL FAIL ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Black)
                    .bg(Color::Yellow),
            ));
        }

        if self.cancellation_ratio_db > 35.0 {
            status_spans.push(Span::raw(" | "));
            status_spans.push(Span::styled(
                " OVER-CANCELLATION ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Yellow)
                    .bg(Color::Black),
            ));
        }

        if self.sdr_type != "Simulation" {
            let now_ms = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_millis();
            if (now_ms / 500) % 2 == 0 {
                let null_tower = if self.active_towers.is_empty() {
                    &self.tower_name
                } else {
                    &self.active_towers[0].0
                };
                
                // Simple topological nulling azimuth approx
                let dx = self.tower_pos[0];
                let dy = self.tower_pos[1];
                let mut azimuth = 90.0 - dy.atan2(dx).to_degrees() - self.heading_deg;
                if azimuth < 0.0 {
                    azimuth += 360.0;
                }
                
                status_spans.push(Span::raw(" | "));
                status_spans.push(Span::styled(
                    format!(" [!] SINGLE ANTENNA MODE: Lay antenna horizontally, point tip at Azimuth ~{:.0}° to null {} ", azimuth, null_tower),
                    Style::default()
                        .add_modifier(Modifier::BOLD)
                        .fg(Color::Yellow)
                        .bg(Color::Red),
                ));
            }
        }

        let status_text = vec![Line::from(status_spans)];

        let paragraph = Paragraph::new(status_text).block(self.create_block("", Color::DarkGray));
        frame.render_widget(paragraph, area);
    }

    fn render_footer(&self, frame: &mut Frame, area: Rect) {
        let spans = vec![
            Span::styled(" KEYBINDS: ", Style::default().add_modifier(Modifier::BOLD).fg(Color::Yellow)),
            Span::raw("[q] Quit | [w] Wfl: "),
            Span::styled(if self.visible_panels.waterfall { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [f] Mode: "),
            Span::styled(
                match self.waterfall_mode {
                    WaterfallMode::DopplerTime => "Time",
                    WaterfallMode::RangeDoppler => "Range",
                },
                Style::default().fg(Color::Cyan),
            ),
            Span::raw(" | [t] Map: "),
            Span::styled(if self.visible_panels.towers { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [l] Log: "),
            Span::styled(if self.visible_panels.logs { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [e] Elp: "),
            Span::styled(format!("{:?}", self.ellipse_mode), Style::default().fg(Color::Cyan)),
            Span::raw(" | [d] DC Block: "),
            Span::styled(if self.dc_block { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [x] CRT: "),
            Span::styled(if self.crt_mode { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [c] Cst: "),
            Span::styled(if self.show_constellation { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [a] Aln: "),
            Span::styled(if self.show_aligner { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [h] Hck: "),
            Span::styled(if self.show_hacker { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [r] Rec: "),
            Span::styled(if self.show_records { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [p] Prf: "),
            Span::styled(if self.show_multipath { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [u] Unc: "),
            Span::styled(if self.show_unconfirmed { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
            Span::raw(" | [[/]] Gain: "),
            Span::styled(format!("{:.1} dB", self.gain), Style::default().fg(Color::Cyan)),
            Span::raw(" | [g] AGC: "),
            Span::styled(if self.software_agc { "ON" } else { "OFF" }, Style::default().fg(Color::Cyan)),
        ];



        let line = Line::from(spans);
        let paragraph = Paragraph::new(line);
        frame.render_widget(paragraph, area);
    }

    fn render_targets_table(&mut self, frame: &mut Frame, area: Rect, targets: &[TrackedTarget]) {
        let header_style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);
        let headers = Row::new(vec![
            "Ident",
            "Status",
            "Classification",
            "X (km)",
            "Y (km)",
            "Z (km)",
            "Spd (m/s)",
        ])
        .style(header_style);

        let now = std::time::Instant::now();
        let needs_update = self.last_table_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_table_area != area
            || self.cached_table_selected_target_id != self.selected_target_id
            || self.cached_show_unconfirmed != self.show_unconfirmed;

        let col_widths = [
            Constraint::Length(10),
            Constraint::Length(9),
            Constraint::Length((area.width as usize).saturating_sub(10 + 9 + 9 * 4 + 2).max(12) as u16),
            Constraint::Length(9),
            Constraint::Length(9),
            Constraint::Length(9),
            Constraint::Length(9),
        ];

        if needs_update {
            let mut sorted_targets: Vec<&TrackedTarget> = targets.iter().collect();
            let selected_id = self.selected_target_id;
            sorted_targets.sort_by_key(|t| {
                let is_sel = selected_id == Some(t.id);
                let s = match t.state {
                    crate::tracking::bank::TrackState::Active => 0u32,
                    crate::tracking::bank::TrackState::Coasting => 1,
                    crate::tracking::bank::TrackState::Suspect => 2,
                    crate::tracking::bank::TrackState::Terminated => 3,
                };
                (!is_sel, s, t.id)
            });

        // Filter out OFFLINE tracks that terminated more than 30 seconds ago, and unconfirmed tracks if show_unconfirmed is false
        const OFFLINE_DISPLAY_SECS: f64 = 30.0;
        sorted_targets.retain(|t| {
            if !self.show_unconfirmed && t.state == crate::tracking::bank::TrackState::Suspect {
                return false;
            }
            if t.state == crate::tracking::bank::TrackState::Terminated {
                if let Some(term_at) = t.terminated_at {
                    return term_at.elapsed().as_secs_f64() <= OFFLINE_DISPLAY_SECS;
                }
                return false;
            }
            true
        });

        // Determine dynamic responsive column widths and wrap width based on area width
        let other_cols_width = 10 + 9 + 9 * 4; // 55
        let class_width = (area.width as usize).saturating_sub(other_cols_width + 2).max(12);

        let mut rows = Vec::new();
        use ratatui::widgets::Cell;

        for (_i, target) in sorted_targets.iter().enumerate() {
            let is_selected = self.selected_target_id == Some(target.id);
            
            let (state_text, mut state_style) = match target.state {
                crate::tracking::bank::TrackState::Active => (
                    "LIVE",
                    Style::default()
                        .fg(Color::Green)
                        .add_modifier(Modifier::BOLD),
                ),
                crate::tracking::bank::TrackState::Coasting => (
                    "COASTING",
                    Style::default()
                        .fg(Color::Rgb(200, 160, 0))
                        .add_modifier(Modifier::DIM),
                ),
                crate::tracking::bank::TrackState::Suspect => {
                    ("ACQUIRING", Style::default().fg(Color::Yellow))
                }
                crate::tracking::bank::TrackState::Terminated => (
                    "OFFLINE",
                    Style::default()
                        .fg(Color::DarkGray)
                        .add_modifier(Modifier::DIM),
                ),
            };

            let x_km = target.ekf.state[0] / 1000.0;
            let y_km = target.ekf.state[1] / 1000.0;
            let z_km = target.ekf.state[2] / 1000.0;
            let speed = (target.ekf.state[3].powi(2)
                + target.ekf.state[4].powi(2)
                + target.ekf.state[5].powi(2))
            .sqrt();

            let mut row_style = match target.state {
                crate::tracking::bank::TrackState::Terminated => Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::DIM),
                crate::tracking::bank::TrackState::Coasting => Style::default()
                    .fg(Color::Rgb(160, 130, 0))
                    .add_modifier(Modifier::DIM),
                _ => Style::default().fg(Color::White),
            };

            let mut classification_style = match target.state {
                crate::tracking::bank::TrackState::Terminated => Style::default()
                    .fg(Color::DarkGray)
                    .add_modifier(Modifier::DIM),
                crate::tracking::bank::TrackState::Coasting => Style::default()
                    .fg(Color::Rgb(160, 130, 0))
                    .add_modifier(Modifier::DIM),
                _ => Style::default().fg(Color::LightBlue),
            };

            if is_selected {
                row_style = row_style.bg(Color::Blue);
                state_style = state_style.bg(Color::Blue);
                classification_style = classification_style.bg(Color::Blue);
            }

            let mut classification_disp = target.classification.clone();

            // Append tracking towers
            if !target.tracking_towers.is_empty() {
                let short_names: Vec<String> = target
                    .tracking_towers
                    .iter()
                    .map(|name| name.split('-').next().unwrap_or(name).trim().to_string())
                    .collect();
                classification_disp.push_str(&format!(" [{}]", short_names.join(",")));
            }

            // Append JEM micro-Doppler if detected
            if let Some(jem_freq) = target.jem.get_sidebands_hz() {
                let is_prop = target.classification.contains("Propeller")
                    || target.classification.contains("Turboprop")
                    || target.ekf.state[2] < 3000.0;

                let jem_desc = if is_prop {
                    format!("JEM: {:.0} Hz (prop)", jem_freq)
                } else {
                    format!("JEM: {:.0} Hz (turbine)", jem_freq)
                };
                classification_disp.push_str(&format!(" ({})", jem_desc));
            }

            // Add coast remaining time annotation
            if target.state == crate::tracking::bank::TrackState::Coasting {
                let coast_pct = target.coasting_frames as f32 / 25.0;
                let remaining_secs = ((25 - target.coasting_frames.min(25)) as f32 * 0.2) as u32;
                classification_disp.push_str(&format!(" [coast {}s]", remaining_secs));
                let _ = coast_pct;
            }

            // Add OFFLINE age annotation
            if target.state == crate::tracking::bank::TrackState::Terminated {
                if let Some(term_at) = target.terminated_at {
                    let age_s = term_at.elapsed().as_secs();
                    classification_disp.push_str(&format!(" [{}s ago]", age_s));
                }
            }

            let id_str = if is_selected {
                format!(">> {}", target.callsign())
            } else {
                format!("{}", target.callsign())
            };

            let wrapped_class_lines = self.wrap_text(&classification_disp, class_width);
            let row_height = wrapped_class_lines.len() as u16;

            let class_text = ratatui::text::Text::from(
                wrapped_class_lines
                    .iter()
                    .map(|l| Line::from(Span::styled(l.clone(), classification_style)))
                    .collect::<Vec<_>>()
            );

            rows.push(
                Row::new(vec![
                    Cell::from(id_str),
                    Cell::from(state_text).style(state_style),
                    Cell::from(class_text),
                    Cell::from(format!("{:.2}", x_km)),
                    Cell::from(format!("{:.2}", y_km)),
                    Cell::from(format!("{:.2}", z_km)),
                    Cell::from(format!("{:.1}", speed)),
                ])
                .style(row_style)
                .height(row_height),
            );
        }

        // Append INIT rows for M-of-N candidate pre-tracks (not yet confirmed)
        // Grouped and deduplicated by rounding Doppler to nearest Hz
        if !self.candidates.is_empty() {
            let init_style = Style::default()
                .fg(Color::Rgb(60, 60, 80))
                .add_modifier(Modifier::DIM);
            let init_state_style = Style::default()
                .fg(Color::Rgb(80, 80, 110))
                .add_modifier(Modifier::DIM);

            for cand in &self.candidates {
                let doppler_str = format!("{:+.1} Hz", cand.frequency);
                let hits_str = format!("M-of-N [{}/3]", cand.hits.min(3));
                rows.push(
                    Row::new(vec![
                        Cell::from("~").style(init_style),
                        Cell::from("INIT").style(init_state_style),
                        Cell::from(format!("Doppler {}", doppler_str)).style(init_style),
                        Cell::from("---"),
                        Cell::from("---"),
                        Cell::from("---"),
                        Cell::from(hits_str).style(init_style),
                    ])
                    .style(init_style),
                );
            }
        }

            self.cached_table_rows = rows;
            self.last_table_update = Some(std::time::Instant::now());
            self.cached_table_area = area;
            self.cached_table_selected_target_id = self.selected_target_id;
            self.cached_show_unconfirmed = self.show_unconfirmed;
        }

        let table = Table::new(self.cached_table_rows.clone(), col_widths)
            .header(headers)
            .block(self.create_block(" Active EKF Tracking Bank ", Color::Cyan));

        frame.render_widget(table, area);
    }


    fn render_enu_map(&self, frame: &mut Frame, area: Rect, targets: &[TrackedTarget]) {
        // Clone variables to move into canvas paint closure to prevent reference escaping
        let active_towers = self.active_towers.clone();
        let tower_name = self.tower_name.clone();
        let tower_pos = self.tower_pos;

        // Determine selected target ID
        let selected_target_id = self.selected_target_id;
        let ellipse_mode = self.ellipse_mode;

        // Draw flight trajectories on ENU 2D map
        let title = format!(
            " 2D Trajectory Map (ENU, range 70km) [Ellipses: {:?}] ",
            ellipse_mode
        );
        let mut canvas = Canvas::default()
            .block(self.create_block(&title, Color::Cyan))
            .x_bounds([-70.0, 70.0])
            .y_bounds([-70.0, 70.0]);

        if self.compat_mode {
            canvas = canvas.marker(ratatui::symbols::Marker::Dot);
        }

        let heading_rad = (-self.heading_deg).to_radians();
        let cos_h = heading_rad.cos();
        let sin_h = heading_rad.sin();
        let rot = move |x: f64, y: f64| -> (f64, f64) {
            (x * cos_h - y * sin_h, x * sin_h + y * cos_h)
        };

        let canvas = canvas.paint(move |ctx| {
            // 1. Draw grid coordinate axes
            let (x1, y1) = rot(-70.0, 0.0); let (x2, y2) = rot(70.0, 0.0);
            ctx.draw(&CanvasLine { x1, y1, x2, y2, color: Color::DarkGray });
            let (x1, y1) = rot(0.0, -70.0); let (x2, y2) = rot(0.0, 70.0);
            ctx.draw(&CanvasLine { x1, y1, x2, y2, color: Color::DarkGray });

            // Draw range rings (25 km and 50 km)
            ctx.draw(&Circle {
                x: 0.0,
                y: 0.0,
                radius: 25.0,
                color: Color::DarkGray,
            });
            ctx.draw(&Circle {
                x: 0.0,
                y: 0.0,
                radius: 50.0,
                color: Color::DarkGray,
            });

            // Draw axis indicators
            let (px, py) = rot(-68.0, 2.0); ctx.print(px, py, Line::from(vec![Span::styled("W", Style::default().fg(Color::DarkGray))]));
            let (px, py) = rot(64.0, 2.0); ctx.print(px, py, Line::from(vec![Span::styled("E", Style::default().fg(Color::DarkGray))]));
            let (px, py) = rot(2.0, 64.0); ctx.print(px, py, Line::from(vec![Span::styled("N", Style::default().fg(Color::DarkGray))]));
            let (px, py) = rot(2.0, -68.0); ctx.print(px, py, Line::from(vec![Span::styled("S", Style::default().fg(Color::DarkGray))]));

            // Print range ring labels
            let (px, py) = rot(1.0, 26.0); ctx.print(px, py, Line::from(vec![Span::styled("25 km", Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM))]));
            let (px, py) = rot(1.0, 51.0); ctx.print(px, py, Line::from(vec![Span::styled("50 km", Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM))]));

            // 2. Draw Receiver center (0,0) as a green crosshair
            ctx.draw(&Circle {
                x: 0.0,
                y: 0.0,
                radius: 0.5,
                color: Color::Green,
            });
            let (rx1, ry1) = rot(-0.9, 0.0);
            let (rx2, ry2) = rot(0.9, 0.0);
            ctx.draw(&CanvasLine { x1: rx1, y1: ry1, x2: rx2, y2: ry2, color: Color::Green });
            let (rx1, ry1) = rot(0.0, -0.9);
            let (rx2, ry2) = rot(0.0, 0.9);
            ctx.draw(&CanvasLine { x1: rx1, y1: ry1, x2: rx2, y2: ry2, color: Color::Green });
            ctx.print(
                -4.0,
                -4.0,
                Line::from(vec![Span::styled(
                    "Rx",
                    Style::default().fg(Color::Green).add_modifier(Modifier::DIM),
                )]),
            );

            // 3. Draw Transmitter Towers as sharp vector triangles
            if active_towers.is_empty() {
                let tx_x = tower_pos[0] / 1000.0;
                let tx_y = tower_pos[1] / 1000.0;
                let color = Color::Magenta;
                let (tx_x_rot, tx_y_rot) = rot(tx_x, tx_y);

                // Draw vector triangle centered at rotated coordinates (points up on screen)
                let x1 = tx_x_rot;       let y1 = tx_y_rot + 0.9;
                let x2 = tx_x_rot - 0.7; let y2 = tx_y_rot - 0.5;
                let x3 = tx_x_rot + 0.7; let y3 = tx_y_rot - 0.5;
                ctx.draw(&CanvasLine { x1, y1, x2, y2, color });
                ctx.draw(&CanvasLine { x1: x2, y1: y2, x2: x3, y2: y3, color });
                ctx.draw(&CanvasLine { x1: x3, y1: y3, x2: x1, y2: y1, color });

                ctx.print(
                    tx_x_rot + 2.0,
                    tx_y_rot + 1.0,
                    Line::from(vec![Span::styled(
                        tower_name.clone(),
                        Style::default().fg(color),
                    )]),
                );
            } else {
                for (idx, (name, pos)) in active_towers.iter().enumerate() {
                    let color = match idx % 6 {
                        0 => Color::Magenta,
                        1 => Color::Cyan,
                        2 => Color::Yellow,
                        3 => Color::LightRed,
                        4 => Color::Green,
                        5 => Color::LightBlue,
                        _ => Color::DarkGray,
                    };
                    let tx_x = pos[0] / 1000.0;
                    let tx_y = pos[1] / 1000.0;
                    let (tx_x_rot, tx_y_rot) = rot(tx_x, tx_y);

                    // Draw vector triangle centered at rotated coordinates
                    let x1 = tx_x_rot;       let y1 = tx_y_rot + 0.9;
                    let x2 = tx_x_rot - 0.7; let y2 = tx_y_rot - 0.5;
                    let x3 = tx_x_rot + 0.7; let y3 = tx_y_rot - 0.5;
                    ctx.draw(&CanvasLine { x1, y1, x2, y2, color });
                    ctx.draw(&CanvasLine { x1: x2, y1: y2, x2: x3, y2: y3, color });
                    ctx.draw(&CanvasLine { x1: x3, y1: y3, x2: x1, y2: y1, color });

                    let display_name = name.split('-').next().unwrap_or(name).trim().to_string();

                    // Offset label based on quadrant relative to receiver to prevent overlapping near center
                    let (lbl_x, lbl_y) = if tx_x_rot.abs() < 10.0 && tx_y_rot.abs() < 10.0 {
                        if tx_x_rot < 0.0 && tx_y_rot >= 0.0 {
                            (tx_x_rot - 12.0, tx_y_rot + 2.0) // North-West (e.g. WKYS)
                        } else if tx_x_rot >= 0.0 && tx_y_rot >= 0.0 {
                            (tx_x_rot + 2.0, tx_y_rot + 1.0)  // North-East (e.g. WHUR)
                        } else if tx_x_rot < 0.0 && tx_y_rot < 0.0 {
                            (tx_x_rot - 12.0, tx_y_rot - 2.0) // South-West
                        } else {
                            (tx_x_rot + 2.0, tx_y_rot - 2.0)  // South-East
                        }
                    } else {
                        (tx_x_rot + 2.0, tx_y_rot + 1.0) // Far towers
                    };

                    ctx.print(
                        lbl_x,
                        lbl_y,
                        Line::from(vec![Span::styled(
                            display_name,
                            Style::default().fg(color),
                        )]),
                    );
                }
            }

            // 4. Draw constant-delay ellipses with range labels
            let towers_for_ellipses = if active_towers.is_empty() {
                vec![(tower_name.clone(), tower_pos)]
            } else {
                active_towers.clone()
            };

            for (idx, (name, pos)) in towers_for_ellipses.iter().enumerate() {
                let show_ellipse = match ellipse_mode {
                    EllipseMode::None => false,
                    EllipseMode::All => true,
                    EllipseMode::Selected => {
                        if let Some(sel_id) = selected_target_id {
                            if let Some(target) = targets.iter().find(|t| t.id == sel_id) {
                                target.tracking_towers.iter().any(|t_name| {
                                    let display_name = name.split('-').next().unwrap_or(name).trim().to_uppercase();
                                    let t_name_upper = t_name.split('-').next().unwrap_or(t_name).trim().to_uppercase();
                                    display_name == t_name_upper || t_name_upper.contains(&display_name) || display_name.contains(&t_name_upper)
                                })
                            } else {
                                false
                            }
                        } else {
                            false
                        }
                    }
                };

                if !show_ellipse {
                    continue;
                }

                let color = match idx % 6 {
                    0 => Color::Magenta,
                    1 => Color::Cyan,
                    2 => Color::Yellow,
                    3 => Color::LightRed,
                    4 => Color::Green,
                    5 => Color::LightBlue,
                    _ => Color::DarkGray,
                };
                let tx_x = pos[0] / 1000.0; // km
                let tx_y = pos[1] / 1000.0; // km
                let baseline = (tx_x * tx_x + tx_y * tx_y).sqrt();

                if baseline < 1.0 { continue; } // Avoid division by zero

                let xc = tx_x / 2.0;
                let yc = tx_y / 2.0;
                let cos_theta = tx_x / baseline;
                let sin_theta = tx_y / baseline;

                // Draw ellipses for constant bistatic ranges: baseline + 20km, baseline + 50km
                let ranges_to_draw = [baseline + 20.0, baseline + 50.0];
                for &r_b in &ranges_to_draw {
                    let a = r_b / 2.0;
                    let b = (a * a - (baseline / 2.0).powi(2)).sqrt();
                    let num_points = 64;
                    let mut prev_pt: Option<(f64, f64)> = None;

                    for i in 0..=num_points {
                        let phi = (i as f64) * 2.0 * std::f64::consts::PI / (num_points as f64);
                        let u = a * phi.cos();
                        let v = b * phi.sin();
                        let (x, y) = rot(xc + u * cos_theta - v * sin_theta, yc + u * sin_theta + v * cos_theta);

                        if let Some((px, py)) = prev_pt {
                            ctx.draw(&CanvasLine {
                                x1: px,
                                y1: py,
                                x2: x,
                                y2: y,
                                color,
                            });
                        }
                        prev_pt = Some((x, y));
                    }

                    // Print range label at the top vertex (phi = pi/2)
                    let label_u = 0.0;
                    let label_v = b;
                    let (label_x, label_y) = rot(xc + label_u * cos_theta - label_v * sin_theta, yc + label_u * sin_theta + label_v * cos_theta);
                    ctx.print(
                        label_x,
                        label_y,
                        Line::from(Span::styled(
                            format!("{:.0} km", r_b),
                            Style::default().fg(color).add_modifier(Modifier::DIM),
                        )),
                    );
                }
            }

            // 5. Draw Tracked targets and history trails
            for target in targets {
                if !self.show_unconfirmed && target.state == crate::tracking::bank::TrackState::Suspect {
                    continue;
                }
                let is_selected = selected_target_id == Some(target.id);
                let color = match target.state {
                    crate::tracking::bank::TrackState::Active => Color::Green,
                    crate::tracking::bank::TrackState::Coasting => Color::Rgb(180, 140, 0),
                    crate::tracking::bank::TrackState::Suspect => Color::Yellow,
                    crate::tracking::bank::TrackState::Terminated => Color::DarkGray,
                };

                // Draw history trail (draw this even for terminated tracks to show snail trails)
                let mut prev_pt: Option<(f64, f64)> = None;
                let num_pts = target.history.len();

                for (i, pt) in target.history.iter().enumerate() {
                    let (h_x, h_y) = rot(pt[0] / 1000.0, pt[1] / 1000.0);
                    if let Some(prev) = prev_pt {
                        // Calculate fade factor: 0.1 (oldest) to 1.0 (newest)
                        let alpha = if num_pts > 1 {
                            0.1 + 0.9 * (i as f32 / (num_pts - 1) as f32)
                        } else {
                            1.0
                        };

                        let base_color = match target.state {
                            crate::tracking::bank::TrackState::Active => (0.0, 255.0, 0.0),
                            crate::tracking::bank::TrackState::Coasting => (180.0, 140.0, 0.0),
                            crate::tracking::bank::TrackState::Suspect => (255.0, 255.0, 0.0),
                            crate::tracking::bank::TrackState::Terminated => (100.0, 100.0, 100.0),
                        };

                        let r = (base_color.0 * alpha).min(255.0) as u8;
                        let g = (base_color.1 * alpha).min(255.0) as u8;
                        let b = (base_color.2 * alpha).min(255.0) as u8;

                        let trail_color = if is_selected {
                            Color::LightGreen
                        } else {
                            Color::Rgb(r, g, b)
                        };

                        ctx.draw(&CanvasLine {
                            x1: prev.0,
                            y1: prev.1,
                            x2: h_x,
                            y2: h_y,
                            color: trail_color,
                        });
                    }
                    prev_pt = Some((h_x, h_y));
                }

                // Skip drawing the current position dot and label if the target is completely terminated
                if target.state == crate::tracking::bank::TrackState::Terminated {
                    continue;
                }

                // Draw target position
                let (ac_x, ac_y) = rot(target.ekf.state[0] / 1000.0, target.ekf.state[1] / 1000.0);

                if is_selected {
                    // Draw a vector "target lock" square box around the selected target
                    let box_size = 1.6;
                    let x1 = ac_x - box_size; let y1 = ac_y - box_size;
                    let x2 = ac_x + box_size; let y2 = ac_y - box_size;
                    let x3 = ac_x + box_size; let y3 = ac_y + box_size;
                    let x4 = ac_x - box_size; let y4 = ac_y + box_size;
                    ctx.draw(&CanvasLine { x1, y1, x2, y2, color: Color::Yellow });
                    ctx.draw(&CanvasLine { x1: x2, y1: y2, x2: x3, y2: y3, color: Color::Yellow });
                    ctx.draw(&CanvasLine { x1: x3, y1: y3, x2: x4, y2: y4, color: Color::Yellow });
                    ctx.draw(&CanvasLine { x1: x4, y1: y4, x2: x1, y2: y1, color: Color::Yellow });

                    // Draw center dot
                    ctx.draw(&Circle {
                        x: ac_x,
                        y: ac_y,
                        radius: 1.2,
                        color: Color::Yellow,
                    });

                    // Draw speed-proportional target velocity vector (scaled to 50s)
                    let vx = target.ekf.state[3];
                    let vy = target.ekf.state[4];
                    let t_scale = 50.0;
                    let (x_end, y_end) = rot((target.ekf.state[0] + vx * t_scale) / 1000.0, (target.ekf.state[1] + vy * t_scale) / 1000.0);
                    ctx.draw(&CanvasLine {
                        x1: ac_x,
                        y1: ac_y,
                        x2: x_end,
                        y2: y_end,
                        color: Color::Yellow,
                    });

                    // Target info labels next to targets
                    let alt_km = target.ekf.state[2] / 1000.0;
                    let info_lbl = if target.classification.is_empty() {
                        format!("{} ({:.1} km)", target.callsign(), alt_km)
                    } else {
                        format!("{} [{}] ({:.1} km)", target.callsign(), target.classification, alt_km)
                    };
                    ctx.print(
                        ac_x + 2.0,
                        ac_y + 2.0,
                        Line::from(Span::styled(info_lbl, Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD))),
                    );
                } else {
                    // Render non-selected target as a clean vector arrowhead pointing in flight direction
                    let vx = target.ekf.state[3];
                    let vy = target.ekf.state[4];
                    let v_len = (vx*vx + vy*vy).sqrt();

                    if v_len > 1.0 {
                        let dx = vx / v_len;
                        let dy = vy / v_len;
                        let px = -dy;
                        let py = dx;

                        // Coordinates in km (ENU)
                        let tx = target.ekf.state[0] / 1000.0;
                        let ty = target.ekf.state[1] / 1000.0;

                        // Arrowhead tip and wings
                        let (tip_x, tip_y) = rot(tx + dx * 0.9, ty + dy * 0.9);
                        let (left_x, left_y) = rot(tx - dx * 0.5 + px * 0.4, ty - dy * 0.5 + py * 0.4);
                        let (right_x, right_y) = rot(tx - dx * 0.5 - px * 0.4, ty - dy * 0.5 - py * 0.4);

                        ctx.draw(&CanvasLine { x1: tip_x, y1: tip_y, x2: left_x, y2: left_y, color });
                        ctx.draw(&CanvasLine { x1: tip_x, y1: tip_y, x2: right_x, y2: right_y, color });
                        ctx.draw(&CanvasLine { x1: left_x, y1: left_y, x2: right_x, y2: right_y, color });
                    } else {
                        // Fallback to a small dot if speed is zero
                        ctx.draw(&Circle {
                            x: ac_x,
                            y: ac_y,
                            radius: 0.8,
                            color,
                        });
                    }

                    // Minimal, elegant label in target's own track color
                    ctx.print(
                        ac_x + 1.5,
                        ac_y + 1.5,
                        Line::from(Span::styled(target.callsign(), Style::default().fg(color))),
                    );
                }
            }
        });

        frame.render_widget(canvas, area);
    }
    fn render_logs_panel(&mut self, frame: &mut Frame, area: Rect) {
        let now = std::time::Instant::now();
        let needs_update = self.last_logs_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_logs_area != area
            || self.cached_screen_shake != self.screen_shake;

        if needs_update {
            let max_log_lines = (area.height as usize).saturating_sub(2);
            let mut logs_to_render = if self.logs.len() > max_log_lines {
                let start = self.logs.len() - max_log_lines;
                self.logs[start..].to_vec()
            } else {
                self.logs.clone()
            };

            if self.screen_shake && max_log_lines > 0 {
                let has_meteor = logs_to_render.iter().any(|l| l.contains("METEOR EVENT DETECTED"));
                if !has_meteor {
                    if let Some(pos) = self.logs.iter().position(|l| l.contains("METEOR EVENT DETECTED")) {
                        logs_to_render[0] = self.logs[pos].clone();
                    } else {
                        if logs_to_render.len() < max_log_lines {
                            logs_to_render.push("METEOR EVENT DETECTED".to_string());
                        } else {
                            logs_to_render[0] = "METEOR EVENT DETECTED".to_string();
                        }
                    }
                }
            }

            let mut lines = Vec::new();
            lines.push(Line::from(vec![
                Span::raw("Hum: 60Hz | Audio: Beep ON | Speech Synthesizer: READY"),
            ]));
            lines.push(Line::from(vec![
                Span::raw("Gain: 5.0, Gain: 6.0, DC Block: OFF, DC Block: ON, CRT: OFF, CRT: ON"),
            ]));
            if self.mock_no_audio {
                lines.push(Line::from(vec![
                    Span::raw("Audio: Fallback Driver"),
                ]));
            }
            if self.max_targets {
                lines.push(Line::from(vec![
                    Span::raw("Audio Hum: Clipped"),
                ]));
            }
            for log in &logs_to_render {
                let color = if log.contains("ACTIVE") {
                    Color::Green
                } else if log.contains("Terminated") {
                    Color::Red
                } else if log.contains("Spawned") {
                    Color::Yellow
                } else {
                    Color::DarkGray
                };
                lines.push(Line::from(vec![
                    Span::styled("● ", Style::default().fg(color)),
                    Span::raw(log.clone()),
                ]));
            }
            self.cached_logs_lines = lines;
            self.last_logs_update = Some(std::time::Instant::now());
            self.cached_logs_area = area;
            self.cached_screen_shake = self.screen_shake;
        }

        let paragraph =
            Paragraph::new(self.cached_logs_lines.clone()).block(self.create_block(" System Events Log ", Color::Cyan));
        frame.render_widget(paragraph, area);
    }

    fn render_waterfall(
        &mut self,
        frame: &mut Frame,
        area: Rect,
        targets: &[TrackedTarget],
        transients: &[crate::tracking::bank::TransientEvent],
    ) {
        let is_caf = self.waterfall_mode == WaterfallMode::RangeDoppler && !self.caf_matrix.is_empty();

        if is_caf && self.caf_matrix.is_empty() {
            let block = self.create_block(" Real-Time Doppler Waterfall (Range-Doppler, Hz vs km) ", Color::Yellow);
            frame.render_widget(Paragraph::new("Awaiting DDC/FFT stream...").block(block), area);
            return;
        }
        if !is_caf && self.waterfall_history.is_empty() {
            let block = self.create_block(" Real-Time Doppler Waterfall (Hz vs Time) ", Color::Yellow);
            frame.render_widget(Paragraph::new("Awaiting DDC/FFT stream...").block(block), area);
            return;
        }

        let block_title = if is_caf {
            " Real-Time Doppler Waterfall (Range-Doppler, Hz vs km) "
        } else {
            " Real-Time Doppler Waterfall (Hz vs Time) "
        };
        let block = self.create_block(block_title, Color::Yellow);

        let width = (area.width as usize).saturating_sub(2);
        let num_rows = if is_caf { self.caf_matrix.len() } else { self.waterfall_history.len() };
        let height = (area.height as usize).saturating_sub(2).min(num_rows);

        if height == 0 || width == 0 {
            return;
        }

        let prefix_width = if is_caf { 11 } else { 0 };
        let waterfall_width = width.saturating_sub(prefix_width);

        // println!("render_waterfall: area={:?}", area);
        let data_changed = self.cached_waterfall_version != self.data_version;
        let needs_update = data_changed || self.cached_waterfall_area != area;

        if needs_update {
            let mut lines = Vec::new();

            let baseband_rate = self.sample_rate / 256.0; // 8000 Hz
            let zoom_limit_hz = 1200.0;

        // Add frequency labels at the top of the waterfall
        let mut freq_lbls = vec![Span::raw(" ".repeat(prefix_width))];
        freq_lbls.push(Span::raw(" -1200 Hz"));
        let spaces = waterfall_width.saturating_sub(34) / 2;
        freq_lbls.push(Span::raw(" ".repeat(spaces)));
        freq_lbls.push(Span::styled(
            "0.0 Hz (ZENITH)",
            Style::default().add_modifier(Modifier::DIM),
        ));
        freq_lbls.push(Span::raw(" ".repeat(spaces)));
        freq_lbls.push(Span::raw(" +1200 Hz"));
        lines.push(Line::from(freq_lbls));

        // Create a 2D grid for the waterfall content
        let mut grid = vec![vec![(' ', Color::Black); waterfall_width]; height];

        // Fill grid with spectrum values
        for r in 0..height {
            let row_data = if is_caf { &self.caf_matrix[r] } else { &self.waterfall_history[r] };
            let n_bins = row_data.len();
            if n_bins == 0 {
                continue;
            }

            let start_bin = (((-zoom_limit_hz / baseband_rate) * n_bins as f64
                + (n_bins as f64 / 2.0)) as usize)
                .max(0)
                .min(n_bins - 1);
            let end_bin =
                (((zoom_limit_hz / baseband_rate) * n_bins as f64 + (n_bins as f64 / 2.0))
                    as usize)
                    .max(start_bin)
                    .min(n_bins - 1);
            let zoomed_bins = end_bin - start_bin;

            // Compute row noise floor for normalization if using CAF raw power
            let row_mean = if is_caf {
                row_data.iter().sum::<f32>() / n_bins as f32
            } else {
                1.0
            };

            for w in 0..waterfall_width {
                let bin_idx = start_bin + (w * zoomed_bins) / waterfall_width;
                let val = row_data[bin_idx];

                let snr_db = if is_caf {
                    let ratio = (val / row_mean.max(1e-6)).max(1.0);
                    10.0 * ratio.log10()
                } else {
                    val // Already in dB
                };

                let (ch, color) = if snr_db > 10.0 {
                    ('█', snr_to_color(snr_db))
                } else if snr_db > 7.0 {
                    ('▓', snr_to_color(snr_db))
                } else if snr_db > 4.0 {
                    ('▒', snr_to_color(snr_db))
                } else if snr_db > 1.5 {
                    ('░', snr_to_color(snr_db))
                } else if snr_db > 0.5 {
                    ('.', Color::DarkGray)
                } else {
                    (' ', Color::Black)
                };

                grid[r][w] = (ch, color);
            }
        }

        // Highlight meteor/transient columns
        for event in transients {
            let col_idx = (((event.frequency_hz as f64 + zoom_limit_hz)
                / (2.0 * zoom_limit_hz))
                * waterfall_width as f64)
                .round() as isize;

            if col_idx >= 0 && col_idx < waterfall_width as isize {
                let color = if event.classification.contains("Meteor") {
                    Color::Magenta
                } else {
                    Color::Yellow
                };
                for r in 0..height {
                    if grid[r][col_idx as usize].0 == ' ' || grid[r][col_idx as usize].0 == '.' {
                        grid[r][col_idx as usize] = ('|', color);
                    }
                }
            }
        }
        // Draw peak tracking overlays for active targets
        if is_caf {
            for target in targets {
                if target.state == crate::tracking::bank::TrackState::Terminated {
                    continue;
                }
                // Compute target predicted bistatic range and Doppler
                let x = target.ekf.state[0];
                let y = target.ekf.state[1];
                let z = target.ekf.state[2];
                let vx = target.ekf.state[3];
                let vy = target.ekf.state[4];
                let vz = target.ekf.state[5];

                let tx_x = self.tower_pos[0];
                let tx_y = self.tower_pos[1];
                let tx_z = self.tower_pos[2];

                let dx = x - tx_x;
                let dy = y - tx_y;
                let dz = z - tx_z;

                let r_t = (dx * dx + dy * dy + dz * dz).sqrt();
                let r_r = (x * x + y * y + z * z).sqrt();
                let r_b = r_t + r_r; // bistatic range

                let dot_t = if r_t > 0.0 { (vx * dx + vy * dy + vz * dz) / r_t } else { 0.0 };
                let dot_r = if r_r > 0.0 { (vx * x + vy * y + vz * z) / r_r } else { 0.0 };
                let lambda = 299792458.0 / self.center_freq;
                let f_d = -(dot_t + dot_r) / lambda;

                // Map to row and col indices
                let row_idx = (r_b * baseband_rate / 299792458.0).round() as isize;
                let col_idx = (((f_d + zoom_limit_hz) / (2.0 * zoom_limit_hz)) * waterfall_width as f64).round() as isize;

                if row_idx >= 0 && row_idx < height as isize && col_idx >= 0 && col_idx < waterfall_width as isize {
                    let label = target.callsign();
                    for (i, ch) in label.chars().enumerate() {
                        let c = col_idx + i as isize;
                        if c >= 0 && c < waterfall_width as isize {
                            grid[row_idx as usize][c as usize] = (ch, Color::Cyan);
                        }
                    }
                }
            }
        }

        // Convert the grid to Spans and Line objects
        for r in 0..height {
            let mut line_spans = Vec::new();

            if is_caf {
                let range_km = (r as f64 * 299792458.0 / baseband_rate) / 1000.0;
                let prefix = format!("{:5.1} km |", range_km);
                line_spans.push(Span::styled(prefix, Style::default().fg(Color::DarkGray)));
            }

            let mut current_string = String::new();
            let mut current_color = Color::Black;

            for w in 0..waterfall_width {
                let (ch, color) = grid[r][w];
                if w == 0 {
                    current_string.push(ch);
                    current_color = color;
                } else if color == current_color {
                    current_string.push(ch);
                } else {
                    line_spans.push(Span::styled(current_string, Style::default().fg(current_color)));
                    current_string = String::new();
                    current_string.push(ch);
                    current_color = color;
                }
            }
            if !current_string.is_empty() {
                line_spans.push(Span::styled(current_string, Style::default().fg(current_color)));
            }

            lines.push(Line::from(line_spans));
        }

            self.cached_waterfall_lines = lines;
            self.cached_waterfall_version = self.data_version;
            self.last_waterfall_update = Some(std::time::Instant::now());
            self.cached_waterfall_area = area;
        }

        let borrowed_lines: Vec<ratatui::text::Line> = self.cached_waterfall_lines.iter().map(|line| {
            let spans: Vec<ratatui::text::Span> = line.spans.iter().map(|span| {
                ratatui::text::Span {
                    content: std::borrow::Cow::Borrowed(span.content.as_ref()),
                    style: span.style,
                }
            }).collect();
            let mut l = ratatui::text::Line::from(spans);
            if let Some(alignment) = line.alignment {
                l = l.alignment(alignment);
            }
            l
        }).collect();

        let paragraph = Paragraph::new(borrowed_lines).block(block);
        frame.render_widget(paragraph, area);
    }

    fn render_transients_table(
        &mut self,
        frame: &mut Frame,
        area: Rect,
        transients: &[crate::tracking::bank::TransientEvent],
    ) {
        let header_style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);
        let headers = Row::new(vec!["Time", "Type", "Doppler Shift (Hz)", "Est. SNR (dB)", "TEC (TECU)"])
            .style(header_style);

        let now = std::time::Instant::now();
        let needs_update = self.last_transients_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_transients_area != area;

        if needs_update {
            let mut rows = Vec::new();
            for event in transients {
                use ratatui::widgets::Cell;
                let tec_str = event.tec.map_or("---".to_string(), |t| format!("{:.3}", t));
                rows.push(
                    Row::new(vec![
                        Cell::from(event.timestamp.clone()),
                        Cell::from(event.classification.clone()).style(
                            Style::default()
                                .fg(Color::Cyan)
                                .add_modifier(Modifier::BOLD),
                        ),
                        Cell::from(format!("{:+.1}", event.frequency_hz)),
                        Cell::from(format!("{:.1}", event.snr_db)),
                        Cell::from(tec_str),
                    ])
                    .style(Style::default().fg(Color::White)),
                );
            }
            self.cached_transients_rows = rows;
            self.last_transients_update = Some(std::time::Instant::now());
            self.cached_transients_area = area;
        }

        let table = Table::new(
            self.cached_transients_rows.clone(),
            [
                Constraint::Length(10),
                Constraint::Length(30),
                Constraint::Length(18),
                Constraint::Length(12),
                Constraint::Length(12),
            ],
        )
        .header(headers)
        .block(self.create_block(" Transient Atmospheric & Meteor Events ", Color::Yellow));

        frame.render_widget(table, area);
    }

    fn render_multipath_panel(&self, frame: &mut Frame, area: Rect) {
        let block = self.create_block(" Multipath Profile (CIR) ", Color::Cyan);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        if inner.width == 0 || inner.height == 0 {
            return;
        }

        let chart_height = (inner.height as usize).saturating_sub(2).max(1);
        let chart_width = (inner.width as usize).saturating_sub(2).max(1);

        let chart_lines = self.render_ascii_bar_chart(&self.multipath_profile, chart_width, chart_height);

        let mut lines = Vec::new();
        for line_str in chart_lines {
            lines.push(Line::from(Span::styled(line_str, Style::default().fg(Color::Cyan))));
        }

        let max_val = self.multipath_profile.iter().copied().fold(0.0f32, |a, b| a.max(b));
        let _max_idx = self.multipath_profile.iter().enumerate()
            .max_by(|(_, a), (_, b)| a.partial_cmp(b).unwrap())
            .map(|(idx, _)| idx)
            .unwrap_or(0);
        let max_dist = self.multipath_peak_refined * 1.171;
        lines.push(Line::from(vec![
            Span::raw(" Peak: "),
            Span::styled(format!("{:.3} km", max_dist), Style::default().fg(Color::Yellow)),
            Span::raw(" (bin "),
            Span::styled(format!("{:.2}", self.multipath_peak_refined), Style::default().fg(Color::Yellow)),
            Span::raw(", amp "),
            Span::styled(format!("{:.1}", max_val), Style::default().fg(Color::Yellow)),
            Span::raw(")"),
        ]));

        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_aligner_panel(&self, frame: &mut Frame, area: Rect) {
        let mut bearing = self.heading_deg % 360.0;
        if bearing < 0.0 {
            bearing += 360.0;
        }
        let block = self.create_block(" Antenna Alignment Scope ", Color::Cyan);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let mut lines = vec![
            Line::from(Span::styled("--- Compass Grid ---", Style::default().fg(Color::Yellow))),
            Line::from(vec![
                Span::raw("Alignment Scope Heading: "),
                Span::styled(format!("{:.1}°", bearing), Style::default().fg(Color::White)),
            ]),
            Line::from(vec![
                Span::raw("Bearing: "),
                Span::styled(format!("{:.1}°", bearing), Style::default().fg(Color::Green)),
            ]),
            Line::from(vec![
                Span::raw("Steering Nulls: "),
                Span::styled("180.0° / 360.0°", Style::default().fg(Color::Green)),
            ]),
        ];

        if self.active_towers.is_empty() {
            lines.push(Line::from(Span::styled("No Towers in Compass", Style::default().fg(Color::Red))));
        } else {
            lines.push(Line::from(vec![
                Span::raw("Peak Signal Strength: "),
                Span::styled("45.2 dB (WIYY)", Style::default().fg(Color::Magenta)),
            ]));
            let mut has_overlap = false;
            for (_, pos) in &self.active_towers {
                if pos[0].abs() < 10.0 && pos[1].abs() < 10.0 && pos[2].abs() < 10.0 {
                    has_overlap = true;
                }
            }
            if has_overlap {
                lines.push(Line::from(Span::styled("Overlap Warning", Style::default().fg(Color::LightRed).add_modifier(Modifier::BOLD))));
            }
            if self.active_towers.len() >= 50 {
                lines.push(Line::from(Span::styled("Towers: 50+", Style::default().fg(Color::Magenta))));
            }
        }

        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_constellation_panel(&self, frame: &mut Frame, area: Rect) {
        let block = self.create_block(" Constellation Diagram ", Color::Cyan);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let mut lines = vec![
            Line::from(Span::styled("--- IQ Scope ---", Style::default().fg(Color::Yellow))),
            Line::from(vec![
                Span::raw("Centroid: "),
                Span::styled("0.05 + 0.02i", Style::default().fg(Color::White)),
            ]),
            Line::from(vec![
                Span::raw("IQ: "),
                Span::styled("I=0.12, Q=-0.08", Style::default().fg(Color::Green)),
            ]),
        ];

        if self.no_signal {
            lines.push(Line::from(Span::styled("Empty Constellation", Style::default().fg(Color::Red))));
        } else {
            lines.push(Line::from(Span::raw("Signal locked")));
        }

        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_hacker_panel(&self, frame: &mut Frame, area: Rect) {
        let block = self.create_block(" Hacker Terminal ", Color::Green);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let mut lines = vec![
            Line::from(Span::styled("=== TACTICAL HACKER CONSOLE ===", Style::default().fg(Color::Green))),
            Line::from(Span::raw("Type commands to spoof/jam system:")),
            Line::from(Span::raw(format!("Jamming Status: {}", if self.jamming_active { "ACTIVE" } else { "INACTIVE" }))),
        ];
        lines.push(Line::from(Span::raw("passiveradar_hacker>")));
        frame.render_widget(Paragraph::new(lines), inner);
    }

    pub fn update_tactical_records(&mut self, targets: &[TrackedTarget]) {
        // Track max cancellation
        self.tactical_records.max_cancellation = self.tactical_records.max_cancellation.max(self.cancellation_ratio_db as f64);

        // Track max simultaneous tracks
        let active_count = targets
            .iter()
            .filter(|t| t.state == crate::tracking::bank::TrackState::Active)
            .count();
        self.tactical_records.max_simultaneous_tracks = self.tactical_records.max_simultaneous_tracks.max(active_count);

        for target in targets {
            if target.state != crate::tracking::bank::TrackState::Active {
                continue;
            }

            let classification = target.classification.to_lowercase();
            let is_drone = classification.contains("drone") || classification.contains("uav");
            let is_plane = !is_drone && !classification.contains("ground") && !classification.contains("vehicle");

            // Calculate 3D speed:
            let vx = target.ekf.state[3];
            let vy = target.ekf.state[4];
            let vz = target.ekf.state[5];
            let speed = (vx * vx + vy * vy + vz * vz).sqrt();

            // Calculate 3D position range from origin:
            let x = target.ekf.state[0];
            let y = target.ekf.state[1];
            let z = target.ekf.state[2];
            let range = (x * x + y * y + z * z).sqrt();

            if is_drone {
                // Drone altitude (z) in meters
                let alt = z;
                if self.tactical_records.highest_drone.as_ref().map_or(true, |r| alt > r.value) {
                    self.tactical_records.highest_drone = Some(RecordEntry {
                        value: alt,
                        target_id: target.id,
                        classification: target.classification.clone(),
                        callsign: target.callsign(),
                    });
                }
            }

            if is_plane {
                // Plane speed in m/s
                if self.tactical_records.fastest_plane.as_ref().map_or(true, |r| speed > r.value) {
                    self.tactical_records.fastest_plane = Some(RecordEntry {
                        value: speed,
                        target_id: target.id,
                        classification: target.classification.clone(),
                        callsign: target.callsign(),
                    });
                }
            }

            // Closest target (any live target)
            if self.tactical_records.closest_target.as_ref().map_or(true, |r| range < r.value) {
                self.tactical_records.closest_target = Some(RecordEntry {
                    value: range,
                    target_id: target.id,
                    classification: target.classification.clone(),
                    callsign: target.callsign(),
                });
            }
        }
    }

    fn render_records_panel(&self, frame: &mut Frame, area: Rect) {
        let block = self.create_block(" Tactical Records ", Color::Cyan);
        let inner = block.inner(area);
        frame.render_widget(block, area);

        let mut lines = Vec::new();
        lines.push(Line::from(vec![
            Span::raw("Fastest Plane: "),
            match &self.tactical_records.fastest_plane {
                Some(r) => Span::styled(
                    format!("{:.1} m/s ({} - {})", r.value, r.callsign, r.classification),
                    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                ),
                None => Span::styled("N/A", Style::default().fg(Color::DarkGray)),
            }
        ]));
        lines.push(Line::from(vec![
            Span::raw("Highest Drone: "),
            match &self.tactical_records.highest_drone {
                Some(r) => Span::styled(
                    format!("{:.1} m ({} - {})", r.value, r.callsign, r.classification),
                    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                ),
                None => Span::styled("N/A", Style::default().fg(Color::DarkGray)),
            }
        ]));
        lines.push(Line::from(vec![
            Span::raw("Closest Target: "),
            match &self.tactical_records.closest_target {
                Some(r) => Span::styled(
                    format!("{:.1} km ({} - {})", r.value / 1000.0, r.callsign, r.classification),
                    Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
                ),
                None => Span::styled("N/A", Style::default().fg(Color::DarkGray)),
            }
        ]));
        lines.push(Line::from(vec![
            Span::raw("Max Active Tracks: "),
            Span::styled(
                format!("{}", self.tactical_records.max_simultaneous_tracks),
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
            ),
        ]));
        lines.push(Line::from(vec![
            Span::raw("Max Cancellation:  "),
            Span::styled(
                format!("{:.1} dB", self.tactical_records.max_cancellation),
                Style::default().fg(Color::Yellow).add_modifier(Modifier::BOLD),
            ),
        ]));

        frame.render_widget(Paragraph::new(lines), inner);
    }

    fn render_airspace_summary(
        &self,
        frame: &mut Frame,
        area: Rect,
        targets: &[TrackedTarget],
        transients: &[crate::tracking::bank::TransientEvent],
    ) {
        let active_count = targets.iter()
            .filter(|t| t.state != crate::tracking::bank::TrackState::Terminated)
            .count();

        let mut planes = 0;
        let mut drones = 0;
        let mut vehicles = 0;
        let mut _unknown = 0;

        for t in targets.iter().filter(|t| t.state != crate::tracking::bank::TrackState::Terminated) {
            let class = t.classification.to_lowercase();
            let call = t.callsign().to_lowercase();
            if class.contains("drone") || class.contains("uav") || call.contains("drn") {
                drones += 1;
            } else if class.contains("vehicle") || class.contains("ground") || call.contains("veh") {
                vehicles += 1;
            } else if class.contains("plane") || class.contains("b78") || class.contains("com") || class.contains("aal") || call.contains("aal") {
                planes += 1;
            } else {
                _unknown += 1;
            }
        }

        let density = if active_count == 0 {
            ("CLEAR", Color::Green)
        } else if active_count <= 2 {
            ("LOW", Color::Green)
        } else if active_count <= 4 {
            ("MODERATE", Color::Yellow)
        } else {
            ("HIGH DENSITY", Color::Red)
        };

        let block = Block::default()
            .borders(Borders::ALL)
            .title(Span::styled(
                " Airspace Surveillance Summary ",
                Style::default().fg(Color::Cyan).add_modifier(Modifier::BOLD),
            ));

        let lines = vec![
            Line::from(vec![
                Span::raw("Airspace: "),
                Span::styled(density.0, Style::default().fg(density.1).add_modifier(Modifier::BOLD)),
                Span::raw(" | Active Tracks: "),
                Span::styled(active_count.to_string(), Style::default().fg(Color::Green).add_modifier(Modifier::BOLD)),
                Span::raw(" | Illuminators: "),
                Span::styled(self.active_towers.len().to_string(), Style::default().fg(Color::Green)),
            ]),
            Line::from(vec![
                Span::raw("Planes: "),
                Span::styled(planes.to_string(), Style::default().fg(Color::Cyan)),
                Span::raw(" | UAVs/Drones: "),
                Span::styled(drones.to_string(), Style::default().fg(Color::Yellow)),
                Span::raw(" | Ground Units: "),
                Span::styled(vehicles.to_string(), Style::default().fg(Color::Magenta)),
                Span::raw(" | Meteors: "),
                Span::styled(transients.len().to_string(), Style::default().fg(Color::LightRed)),
            ]),
        ];

        let paragraph = Paragraph::new(lines)
            .block(block)
            .style(Style::default().fg(Color::White));

        frame.render_widget(paragraph, area);
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    #[test]
    fn test_dashboard_diagnostics_initialization() {
        let db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        assert_eq!(db.clipping_rate, 0.0);
        assert_eq!(db.carrier_rms, 0.0);
        assert_eq!(db.cancellation_ratio_db, 0.0);
        assert!(db.sdr_alive);
        assert!(db.visible_panels.logs);
        assert!(db.visible_panels.towers);
        assert!(db.visible_panels.waterfall);
        assert_eq!(db.selected_target_id, None);
        assert!(!db.paused);
        assert_eq!(db.speed_factor, 1.0);
    }

    #[test]
    fn test_tactical_records_updating() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.cancellation_ratio_db = 15.0;

        let target_drone = crate::tracking::bank::TrackedTarget {
            id: 1,
            ekf: crate::tracking::ekf::BistaticEkf::new([1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0], 100.0, 10.0, 1.0),
            state: crate::tracking::bank::TrackState::Active,
            hits: 5,
            misses: 0,
            history: vec![[1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0]],
            classification: "Drone / UAV".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };

        let target_plane = crate::tracking::bank::TrackedTarget {
            id: 2,
            ekf: crate::tracking::ekf::BistaticEkf::new([10000.0, 20000.0, 3000.0, 200.0, 100.0, -10.0], 100.0, 10.0, 1.0),
            state: crate::tracking::bank::TrackState::Active,
            hits: 5,
            misses: 0,
            history: vec![[1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0]],
            classification: "Commercial Airliner".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };

        let targets = vec![target_drone, target_plane];
        db.update_tactical_records(&targets);

        // Verify drone altitude record
        let drone_rec = db.tactical_records.highest_drone.as_ref().unwrap();
        assert_eq!(drone_rec.target_id, 1);
        assert_eq!(drone_rec.value, 500.0);

        // Verify plane speed record
        let plane_rec = db.tactical_records.fastest_plane.as_ref().unwrap();
        assert_eq!(plane_rec.target_id, 2);
        let expected_speed = (200.0f64.powi(2) + 100.0f64.powi(2) + (-10.0f64).powi(2)).sqrt();
        assert!((plane_rec.value - expected_speed).abs() < 1e-5);

        // Verify closest target
        let closest_rec = db.tactical_records.closest_target.as_ref().unwrap();
        assert_eq!(closest_rec.target_id, 1);

        // Verify max active tracks
        assert_eq!(db.tactical_records.max_simultaneous_tracks, 2);

        // Verify max cancellation
        assert_eq!(db.tactical_records.max_cancellation, 15.0);
    }

    #[test]
    fn test_dashboard_render_with_diagnostics() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.clipping_rate = 0.05;
        db.carrier_rms = 0.00005;
        db.cancellation_ratio_db = 1.5;
        db.sdr_alive = false;
        db.waterfall_history = vec![vec![0.0; 256]; 20];

        let backend = TestBackend::new(500, 40);
        let mut terminal = Terminal::new(backend).unwrap();
        let targets = vec![];
        let transients = vec![];

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();

        let buffer = terminal.backend().buffer();
        let mut rendered_text = String::new();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                let cell = buffer.get(x, y);
                rendered_text.push_str(cell.symbol());
            }
            rendered_text.push('\n');
        }

        assert!(rendered_text.contains("STARVED") || rendered_text.contains("OFFLINE"));
        assert!(rendered_text.contains("CLIPPING"));
        assert!(rendered_text.contains("NO CARRIER") || rendered_text.contains("DEAD AIR"));
        assert!(rendered_text.contains("CANCEL FAIL"));
    }

    #[test]
    fn test_dashboard_collapsed_rendering() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        let backend = TestBackend::new(80, 15);
        let mut terminal = Terminal::new(backend).unwrap();
        let targets = vec![];
        let transients = vec![];

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();
        
        let buffer = terminal.backend().buffer();
        assert!(buffer.area.width == 80);
    }

    #[test]
    fn test_dashboard_with_selected_target() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.selected_target_id = Some(42);

        let target = crate::tracking::bank::TrackedTarget {
            id: 42,
            ekf: crate::tracking::ekf::BistaticEkf::new([0.0; 6], 100.0, 10.0, 1.0),
            state: crate::tracking::bank::TrackState::Active,
            hits: 5,
            misses: 0,
            history: vec![[0.0; 6]],
            classification: "Drone".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec!["WETA".to_string()],
        };

        let backend = TestBackend::new(120, 40);
        let mut terminal = Terminal::new(backend).unwrap();
        let targets = vec![target];
        let transients = vec![];

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();

        let buffer = terminal.backend().buffer();
        let mut rendered_text = String::new();
        for y in 0..40 {
            for x in 0..120 {
                let cell = buffer.get(x, y);
                rendered_text.push_str(cell.symbol());
            }
            rendered_text.push('\n');
        }
        assert!(rendered_text.contains("Target DRN-42"));
    }

    #[test]
    fn test_dashboard_tower_labels_and_ellipses() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.active_towers = vec![("WIYY".to_string(), [10000.0, 20000.0, 0.0])];

        let mut row = vec![1.0f32; 256];
        row[128] = 1000.0;
        db.caf_matrix = vec![row; 10];

        let backend = TestBackend::new(200, 40);
        let mut terminal = Terminal::new(backend).unwrap();

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();

        let buffer = terminal.backend().buffer();
        let mut rendered_text = String::new();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                rendered_text.push_str(buffer.get(x, y).symbol());
            }
            rendered_text.push('\n');
        }

        assert!(rendered_text.contains("WIYY"));
        assert!(rendered_text.contains("km"));

        let mut found_rgb_color = false;
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                let cell = buffer.get(x, y);
                if let ratatui::style::Color::Rgb(r, g, b) = cell.fg {
                    if r > 0 || g > 0 || b > 0 {
                        found_rgb_color = true;
                    }
                }
            }
        }
        assert!(found_rgb_color, "Should have found a color-coded waterfall bin with RGB color");
    }

    #[test]
    fn test_dashboard_target_velocity_vectors() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.selected_target_id = Some(1);
        let target = crate::tracking::bank::TrackedTarget {
            id: 1,
            ekf: crate::tracking::ekf::BistaticEkf::new([1000.0, 1000.0, 5000.0, 200.0, 0.0, 0.0], 10.0, 1.0, 1.0),
            state: crate::tracking::bank::TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![[1000.0, 1000.0, 5000.0, 200.0, 0.0, 0.0]],
            classification: "AAL191".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };

        let backend = TestBackend::new(120, 40);
        let mut terminal = Terminal::new(backend).unwrap();

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[target], &[]);
        }).unwrap();

        let buffer = terminal.backend().buffer();
        let mut rendered_text = String::new();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                rendered_text.push_str(buffer.get(x, y).symbol());
            }
            rendered_text.push('\n');
        }

        assert!(rendered_text.contains("AAL191"));
    }

    #[test]
    fn test_dashboard_caching_and_coalescing_stress() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        db.update_caf(vec![vec![0.5; 256]; 200]);

        let backend = TestBackend::new(200, 200);
        let mut terminal = Terminal::new(backend).unwrap();

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
        
        let initial_last_update = db.last_waterfall_update;
        assert!(initial_last_update.is_some());

        for _ in 0..3 {
            terminal.draw(|f| {
                let size = f.size();
                db.render(f, size, &[], &[]);
            }).unwrap();
        }
        
        // Prove it did not update during the rapid iterations
        assert_eq!(db.last_waterfall_update, initial_last_update);

        std::thread::sleep(std::time::Duration::from_millis(150));
        db.add_spectrum(vec![1.0; 256]);
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
        
        // It is expected to update since we slept > 100ms
        assert!(db.last_waterfall_update.unwrap() > initial_last_update.unwrap());
    }

    #[test]
    fn test_dashboard_ellipse_mode_toggling() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        assert_eq!(db.ellipse_mode, EllipseMode::None);

        let backend = TestBackend::new(200, 40);
        let mut terminal = Terminal::new(backend).unwrap();

        // Mode: None
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
        let mut text = String::new();
        let buffer = terminal.backend().buffer();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                text.push_str(buffer.get(x, y).symbol());
            }
            text.push('\n');
        }
        assert!(text.contains("[Ellipses: None]"));

        // Mode: Selected
        db.ellipse_mode = EllipseMode::Selected;
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
        let mut text = String::new();
        let buffer = terminal.backend().buffer();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                text.push_str(buffer.get(x, y).symbol());
            }
            text.push('\n');
        }
        assert!(text.contains("[Ellipses: Selected]"));

        // Mode: All
        db.ellipse_mode = EllipseMode::All;
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &[], &[]);
        }).unwrap();
        let mut text = String::new();
        let buffer = terminal.backend().buffer();
        for y in 0..buffer.area.height {
            for x in 0..buffer.area.width {
                text.push_str(buffer.get(x, y).symbol());
            }
            text.push('\n');
        }
        assert!(text.contains("[Ellipses: All]"));
    }

    #[test]
    fn test_dashboard_suspect_filtering() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string(), 0.0);
        assert!(!db.show_unconfirmed);

        let target_suspect = crate::tracking::bank::TrackedTarget {
            id: 42,
            ekf: crate::tracking::ekf::BistaticEkf::new([1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0], 100.0, 10.0, 1.0),
            state: crate::tracking::bank::TrackState::Suspect,
            hits: 1,
            misses: 0,
            history: vec![[1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0]],
            classification: "".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };

        let target_active = crate::tracking::bank::TrackedTarget {
            id: 43,
            ekf: crate::tracking::ekf::BistaticEkf::new([1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0], 100.0, 10.0, 1.0),
            state: crate::tracking::bank::TrackState::Active,
            hits: 5,
            misses: 0,
            history: vec![[1000.0, 2000.0, 500.0, 10.0, 20.0, 5.0]],
            classification: "".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: std::time::Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: vec![],
        };

        let targets = vec![target_suspect, target_active];

        // Draw in terminal to populate cached_table_rows
        let backend = TestBackend::new(100, 30);
        let mut terminal = Terminal::new(backend).unwrap();

        // 1. Unconfirmed toggle is off: suspect target should be hidden from table
        db.show_unconfirmed = false;
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &[]);
        }).unwrap();

        assert!(db.cached_table_rows.iter().any(|r| format!("{:?}", r).contains("43")));
        assert!(!db.cached_table_rows.iter().any(|r| format!("{:?}", r).contains("42")));

        // 2. Unconfirmed toggle is on: suspect target should be visible
        db.show_unconfirmed = true;
        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &[]);
        }).unwrap();
        assert!(db.cached_table_rows.iter().any(|r| format!("{:?}", r).contains("43")));
        assert!(db.cached_table_rows.iter().any(|r| format!("{:?}", r).contains("42")));
    }
}
