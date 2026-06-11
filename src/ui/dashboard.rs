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

#[derive(Debug, Clone, Copy)]
pub struct VisiblePanes {
    pub logs: bool,
    pub towers: bool,
    pub waterfall: bool,
}

pub struct Dashboard {
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
}

impl Dashboard {
    pub fn new(center_freq: f64, sample_rate: f64, ddc_offset: f64, sdr_type: String) -> Self {
        Self {
            waterfall_history: Vec::new(),
            caf_matrix: Vec::new(),
            max_waterfall_rows: 200,
            center_freq,
            sample_rate,
            ddc_offset,
            sdr_type,
            tower_name: "Scanning...".to_string(),
            tower_pos: [0.0, 0.0, 0.0],
            active_towers: Vec::new(),
            compat_mode: false,
            logs: Vec::new(),
            sdr_alive: true,
            clipping_rate: 0.0,
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
        let title_str = format!(" Target {} Inspection & EKF State ", target.id);
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
            || self.cached_inspection_target_id != Some(target.id);

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
                    Span::styled(format!("{:?}", target.state), Style::default().fg(Color::Green)),
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
            let chart_lines = self.render_ascii_bar_chart(&target.jem.latest_fft_mag, chart_width, chart_height);
            
            let mut jem_elements = vec![
                Line::from(Span::styled("--- JEM MODULATION SPECTRUM (FFT Bins) ---", Style::default().fg(Color::Yellow))),
            ];
            for l in chart_lines {
                jem_elements.push(Line::from(Span::styled(l, Style::default().fg(Color::Cyan))));
            }
            self.cached_inspection_jem = jem_elements;
            self.last_inspection_update = Some(now);
            self.cached_inspection_area = area;
            self.cached_inspection_target_id = Some(target.id);
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
        // Divide the UI into:
        // Top: Status bar (Height = 3)
        // Middle: Left (Target Table & Map) and Right (Doppler Waterfall & Transients or Target Inspection)
        let main_chunks = Layout::default()
            .direction(Direction::Vertical)
            .constraints([
                Constraint::Length(3), // Status bar
                Constraint::Min(10),   // Main visualization
            ])
            .split(area);

        // Status bar
        self.render_status_bar(frame, main_chunks[0]);

        // Sort targets to correctly map selected_target_id — stable by (state_priority, id)
        let sorted_targets = {
            let mut sorted: Vec<&TrackedTarget> = targets.iter().collect();
            sorted.sort_by_key(|t| {
                let s = match t.state {
                    crate::tracking::bank::TrackState::Active => 0u32,
                    crate::tracking::bank::TrackState::Coasting => 1,
                    crate::tracking::bank::TrackState::Suspect => 2,
                    crate::tracking::bank::TrackState::Terminated => 3,
                };
                (s, t.id)
            });
            sorted
        };

        let selected_target = self.selected_target_id.and_then(|id| sorted_targets.iter().find(|t| t.id == id).copied());

        // Dynamic collapses:
        // If area.width < 100, collapse Waterfall and Transients.
        // If area.height < 30, collapse Logs.
        // If area.height < 20, collapse ENU map (Towers).
        let show_right = area.width >= 100 && (self.visible_panels.waterfall || selected_target.is_some());
        let show_map = self.visible_panels.towers && area.height >= 20;
        let show_logs = self.visible_panels.logs && area.height >= 30;

        // Split middle area
        let visual_chunks = Layout::default()
            .direction(Direction::Horizontal)
            .constraints(if show_right {
                vec![Constraint::Percentage(50), Constraint::Percentage(50)]
            } else {
                vec![Constraint::Percentage(100)]
            })
            .split(main_chunks[1]);

        // Split Left area vertically based on active panels
        let mut left_constraints = Vec::new();
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
            .split(visual_chunks[0]);

        let mut current_idx = 0;
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
        if show_right {
            if let Some(target) = selected_target {
                self.render_target_inspection(frame, visual_chunks[1], target);
            } else {
                let right_chunks = Layout::default()
                    .direction(Direction::Vertical)
                    .constraints([
                        Constraint::Percentage(70), // Top: Waterfall
                        Constraint::Percentage(30), // Bottom: Atmospheric events
                    ])
                    .split(visual_chunks[1]);

                self.render_waterfall(frame, right_chunks[0], targets, transients);
                self.render_transients_table(frame, right_chunks[1], transients);
            }
        }
    }

    fn render_status_bar(&self, frame: &mut Frame, area: Rect) {
        let towers_str = if self.active_towers.is_empty() {
            self.tower_name.clone()
        } else {
            self.active_towers
                .iter()
                .map(|(name, _)| name.split('-').next().unwrap_or(name).trim().to_string())
                .collect::<Vec<String>>()
                .join(", ")
        };

        let carrier_db = 20.0 * self.carrier_rms.max(1e-10).log10();
        let sig_color = if self.carrier_rms < 0.001 {
            Color::Red
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
                " PASSIVE RADAR DSP PIPELINE ",
                Style::default()
                    .add_modifier(Modifier::BOLD)
                    .fg(Color::Yellow),
            ),
        ];

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
            Span::raw(" | Freq: "),
            Span::styled(
                format!("{:.1} MHz", self.center_freq / 1e6),
                Style::default().fg(Color::Cyan),
            ),
            Span::raw(" | Rate: "),
            Span::styled(
                format!("{:.3} MSPS", self.sample_rate / 1e6),
                Style::default().fg(Color::Cyan),
            ),
            Span::raw(" | Active Towers: "),
            Span::styled(
                format!("{} (DDC={:+.1} Hz)", towers_str, self.ddc_offset),
                Style::default().fg(Color::Magenta),
            ),
            Span::raw(" | Signal: "),
            Span::styled(
                format!("{:.1} dBFS", carrier_db),
                Style::default().fg(sig_color),
            ),
            Span::raw(" | Cancel: "),
            Span::styled(
                format!("{:.1} dB", self.cancellation_ratio_db),
                Style::default().fg(cancel_color),
            ),
        ]);

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

        if self.carrier_rms < 0.001 {
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

        let status_text = vec![Line::from(status_spans)];

        let paragraph = Paragraph::new(status_text).block(self.create_block("", Color::DarkGray));
        frame.render_widget(paragraph, area);
    }

    fn render_targets_table(&mut self, frame: &mut Frame, area: Rect, targets: &[TrackedTarget]) {
        let header_style = Style::default()
            .fg(Color::Yellow)
            .add_modifier(Modifier::BOLD);
        let headers = Row::new(vec![
            "ID",
            "Status",
            "Classification",
            "Pos X (km)",
            "Pos Y (km)",
            "Alt Z (km)",
            "Speed (m/s)",
        ])
        .style(header_style);

        let now = std::time::Instant::now();
        let needs_update = self.last_table_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_table_area != area
            || self.cached_table_selected_target_id != self.selected_target_id;

        let col_widths = [
            Constraint::Length(6),
            Constraint::Length(10),
            Constraint::Length((area.width as usize).saturating_sub(4 + 10 + 11 * 4 + 2).max(15) as u16),
            Constraint::Length(11),
            Constraint::Length(11),
            Constraint::Length(11),
            Constraint::Length(11),
        ];

        if needs_update {
            let mut sorted_targets: Vec<&TrackedTarget> = targets.iter().collect();
        // Stable sort: (state_priority, id) — rows never swap positions when state changes
        sorted_targets.sort_by_key(|t| {
            let s = match t.state {
                crate::tracking::bank::TrackState::Active => 0u32,
                crate::tracking::bank::TrackState::Coasting => 1,
                crate::tracking::bank::TrackState::Suspect => 2,
                crate::tracking::bank::TrackState::Terminated => 3,
            };
            (s, t.id)
        });

        // Filter out OFFLINE tracks that terminated more than 30 seconds ago
        const OFFLINE_DISPLAY_SECS: f64 = 30.0;
        sorted_targets.retain(|t| {
            if t.state == crate::tracking::bank::TrackState::Terminated {
                if let Some(term_at) = t.terminated_at {
                    return term_at.elapsed().as_secs_f64() <= OFFLINE_DISPLAY_SECS;
                }
                return false;
            }
            true
        });

        // Determine dynamic responsive column widths and wrap width based on area width
        let other_cols_width = 4 + 10 + 11 * 4; // 58
        let class_width = (area.width as usize).saturating_sub(other_cols_width + 2).max(15);

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
                format!(">> {}", target.id)
            } else {
                format!("{}", target.id)
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

        // Draw flight trajectories on ENU 2D map
        let mut canvas = Canvas::default()
            .block(self.create_block(" 2D Trajectory Map (ENU, range 70km) ", Color::Cyan))
            .x_bounds([-70.0, 70.0])
            .y_bounds([-70.0, 70.0]);

        if self.compat_mode {
            canvas = canvas.marker(ratatui::symbols::Marker::Dot);
        }

        let canvas = canvas.paint(move |ctx| {
            // 1. Draw grid coordinate axes
            ctx.draw(&CanvasLine {
                x1: -70.0,
                y1: 0.0,
                x2: 70.0,
                y2: 0.0,
                color: Color::DarkGray,
            });
            ctx.draw(&CanvasLine {
                x1: 0.0,
                y1: -70.0,
                x2: 0.0,
                y2: 70.0,
                color: Color::DarkGray,
            });

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
            ctx.print(-68.0, 2.0, Line::from(vec![Span::styled("W", Style::default().fg(Color::DarkGray))]));
            ctx.print(64.0, 2.0, Line::from(vec![Span::styled("E", Style::default().fg(Color::DarkGray))]));
            ctx.print(2.0, 64.0, Line::from(vec![Span::styled("N", Style::default().fg(Color::DarkGray))]));
            ctx.print(2.0, -68.0, Line::from(vec![Span::styled("S", Style::default().fg(Color::DarkGray))]));

            // Print range ring labels
            ctx.print(1.0, 26.0, Line::from(vec![Span::styled("25 km", Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM))]));
            ctx.print(1.0, 51.0, Line::from(vec![Span::styled("50 km", Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM))]));

            // 2. Draw Receiver center (0,0)
            ctx.draw(&Circle {
                x: 0.0,
                y: 0.0,
                radius: 0.8,
                color: Color::Green,
            });
            ctx.print(
                -4.0,
                -4.0,
                Line::from(vec![Span::styled(
                    "Rx (You)",
                    Style::default().fg(Color::Green),
                )]),
            );

            // 3. Draw Transmitter Towers
            if active_towers.is_empty() {
                let tx_x = tower_pos[0] / 1000.0;
                let tx_y = tower_pos[1] / 1000.0;
                ctx.draw(&Circle {
                    x: tx_x,
                    y: tx_y,
                    radius: 1.0,
                    color: Color::Magenta,
                });
                ctx.print(
                    tx_x + 2.0,
                    tx_y + 1.0,
                    Line::from(vec![Span::styled(
                        tower_name.clone(),
                        Style::default().fg(Color::Magenta),
                    )]),
                );
            } else {
                for (name, pos) in &active_towers {
                    let tx_x = pos[0] / 1000.0;
                    let tx_y = pos[1] / 1000.0;
                    ctx.draw(&Circle {
                        x: tx_x,
                        y: tx_y,
                        radius: 1.0,
                        color: Color::Magenta,
                    });
                    
                    let display_name = name.split('-').next().unwrap_or(name).trim().to_string();
                    
                    // Offset label based on quadrant relative to receiver to prevent overlapping near center
                    let (lbl_x, lbl_y) = if tx_x.abs() < 10.0 && tx_y.abs() < 10.0 {
                        if tx_x < 0.0 && tx_y >= 0.0 {
                            (tx_x - 12.0, tx_y + 2.0) // North-West (e.g. WKYS)
                        } else if tx_x >= 0.0 && tx_y >= 0.0 {
                            (tx_x + 2.0, tx_y + 1.0)  // North-East (e.g. WHUR)
                        } else if tx_x < 0.0 && tx_y < 0.0 {
                            (tx_x - 12.0, tx_y - 2.0) // South-West
                        } else {
                            (tx_x + 2.0, tx_y - 2.0)  // South-East
                        }
                    } else {
                        (tx_x + 2.0, tx_y + 1.0) // Far towers
                    };
                    ctx.print(
                        lbl_x,
                        lbl_y,
                        Line::from(vec![Span::styled(
                            display_name,
                            Style::default().fg(Color::Magenta),
                        )]),
                    );
                }
            }

            // Draw constant-delay ellipses with range labels
            let towers_for_ellipses = if active_towers.is_empty() {
                vec![(tower_name.clone(), tower_pos)]
            } else {
                active_towers.clone()
            };

            for (_name, pos) in &towers_for_ellipses {
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
                        let x = xc + u * cos_theta - v * sin_theta;
                        let y = yc + u * sin_theta + v * cos_theta;

                        if let Some((px, py)) = prev_pt {
                            ctx.draw(&CanvasLine {
                                x1: px,
                                y1: py,
                                x2: x,
                                y2: y,
                                color: Color::DarkGray,
                            });
                        }
                        prev_pt = Some((x, y));
                    }

                    // Print range label at the top vertex (phi = pi/2)
                    let label_u = 0.0;
                    let label_v = b;
                    let label_x = xc + label_u * cos_theta - label_v * sin_theta;
                    let label_y = yc + label_u * sin_theta + label_v * cos_theta;
                    ctx.print(
                        label_x,
                        label_y,
                        Line::from(Span::styled(
                            format!("{:.0} km", r_b),
                            Style::default().fg(Color::DarkGray).add_modifier(Modifier::DIM),
                        )),
                    );
                }
            }

            // 4. Draw Tracked targets and history trails
            for target in targets {
                let is_selected = selected_target_id == Some(target.id);
                let color = match target.state {
                    crate::tracking::bank::TrackState::Active => Color::Green,
                    crate::tracking::bank::TrackState::Coasting => Color::Rgb(180, 140, 0),
                    crate::tracking::bank::TrackState::Suspect => Color::Yellow,
                    crate::tracking::bank::TrackState::Terminated => Color::DarkGray,
                };

                // Draw history trail (draw this even for terminated tracks to show snail trails)
                let mut prev_pt: Option<(f64, f64)> = None;
                let trail_color = if is_selected {
                    Color::LightGreen
                } else if target.state == crate::tracking::bank::TrackState::Terminated {
                    Color::Rgb(60, 60, 60) // Faded dark gray for dead trails
                } else {
                    Color::DarkGray
                };

                for pt in &target.history {
                    let h_x = pt[0] / 1000.0;
                    let h_y = pt[1] / 1000.0;
                    if let Some(prev) = prev_pt {
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
                let ac_x = target.ekf.state[0] / 1000.0;
                let ac_y = target.ekf.state[1] / 1000.0;
                let dot_color = if is_selected {
                    Color::Yellow
                } else {
                    color
                };
                let dot_radius = if is_selected {
                    2.0
                } else {
                    1.2
                };
                ctx.draw(&Circle {
                    x: ac_x,
                    y: ac_y,
                    radius: dot_radius,
                    color: dot_color,
                });

                if is_selected {
                    ctx.draw(&Circle {
                        x: ac_x,
                        y: ac_y,
                        radius: 4.0,
                        color: Color::Yellow,
                    });
                }

                // Draw speed-proportional target velocity vector (scaled to 50s)
                let vx = target.ekf.state[3];
                let vy = target.ekf.state[4];
                let t_scale = 50.0;
                let x_end = (target.ekf.state[0] + vx * t_scale) / 1000.0;
                let y_end = (target.ekf.state[1] + vy * t_scale) / 1000.0;
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
                    format!("T{} ({:.1} km)", target.id, alt_km)
                } else {
                    format!("T{} [{}] ({:.1} km)", target.id, target.classification, alt_km)
                };
                ctx.print(
                    ac_x + 2.0,
                    ac_y + 2.0,
                    Line::from(Span::styled(info_lbl, Style::default().fg(Color::Cyan))),
                );
            }
        });

        frame.render_widget(canvas, area);
    }
    fn render_logs_panel(&mut self, frame: &mut Frame, area: Rect) {
        let now = std::time::Instant::now();
        let needs_update = self.last_logs_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_logs_area != area;

        if needs_update {
            let mut lines = Vec::new();
            for log in &self.logs {
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
        if self.caf_matrix.is_empty() && self.waterfall_history.is_empty() {
            let block = self.create_block(" Real-Time Doppler Range-Doppler Waterfall (Hz vs km) ", Color::Yellow);
            frame.render_widget(
                Paragraph::new("Awaiting DDC/FFT stream...").block(block),
                area,
            );
            return;
        }

        let is_caf = !self.caf_matrix.is_empty();
        let block_title = if is_caf {
            " Real-Time Doppler Range-Doppler Waterfall (Hz vs km) "
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

        let now = std::time::Instant::now();
        // println!("render_waterfall: area={:?}", area);
        let data_changed = self.cached_waterfall_version != self.data_version;
        let time_elapsed = self.last_waterfall_update.map_or(true, |t| now.duration_since(t).as_millis() >= 100);
        let needs_update = (data_changed && time_elapsed)
            || self.cached_waterfall_area != area;

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

                let (ch, color) = if snr_db > 3.0 {
                    ('█', snr_to_color(snr_db))
                } else if snr_db > 1.0 {
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
                    let label = format!("T{}", target.id);
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
            ratatui::text::Line {
                spans: line.spans.iter().map(|span| {
                    ratatui::text::Span {
                        content: std::borrow::Cow::Borrowed(span.content.as_ref()),
                        style: span.style,
                    }
                }).collect(),
                alignment: line.alignment, style: ratatui::style::Style::default(),
            }
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
        let headers = Row::new(vec!["Time", "Type", "Doppler Shift (Hz)", "Est. SNR (dB)"])
            .style(header_style);

        let now = std::time::Instant::now();
        let needs_update = self.last_transients_update.map_or(true, |t| now.duration_since(t).as_millis() >= 300)
            || self.cached_transients_area != area;

        if needs_update {
            let mut rows = Vec::new();
            for event in transients {
                use ratatui::widgets::Cell;
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
                Constraint::Length(32),
                Constraint::Length(20),
                Constraint::Length(15),
            ],
        )
        .header(headers)
        .block(self.create_block(" Transient Atmospheric & Meteor Events ", Color::Yellow));

        frame.render_widget(table, area);
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use ratatui::backend::TestBackend;
    use ratatui::Terminal;

    #[test]
    fn test_dashboard_diagnostics_initialization() {
        let db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
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
    fn test_dashboard_render_with_diagnostics() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
        db.clipping_rate = 0.05;
        db.carrier_rms = 0.0005;
        db.cancellation_ratio_db = 1.5;
        db.sdr_alive = false;
        db.waterfall_history = vec![vec![0.0; 256]; 20];

        let backend = TestBackend::new(300, 40);
        let mut terminal = Terminal::new(backend).unwrap();
        let targets = vec![];
        let transients = vec![];

        terminal.draw(|f| {
            let size = f.size();
            db.render(f, size, &targets, &transients);
        }).unwrap();

        let buffer = terminal.backend().buffer();
        let mut rendered_text = String::new();
        for y in 0..40 {
            for x in 0..300 {
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
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
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
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
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
        assert!(rendered_text.contains("Target 42"));
    }

    #[test]
    fn test_dashboard_tower_labels_and_ellipses() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
        db.active_towers = vec![("WIYY".to_string(), [10000.0, 20000.0, 0.0])];

        let mut row = vec![1.0f32; 256];
        row[128] = 1000.0;
        db.caf_matrix = vec![row; 10];

        let backend = TestBackend::new(120, 40);
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
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
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

        assert!(rendered_text.contains("T1"));
        assert!(rendered_text.contains("AAL191"));
    }

    #[test]
    fn test_dashboard_caching_and_coalescing_stress() {
        let mut db = Dashboard::new(90.9e6, 2.048e6, 75.0, "sim".to_string());
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
}
