use crate::tracking::ekf::BistaticEkf;
use num_complex::Complex;
use serde::{Deserialize, Serialize};
use std::time::Instant;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FingerprintDatapoint {
    pub time_elapsed_sec: f64,
    pub x_enu: f64,
    pub y_enu: f64,
    pub z_enu: f64,
    pub vx: f64,
    pub vy: f64,
    pub vz: f64,
    pub bistatic_angle_deg: f64,
    pub doppler_hz: f64,
    pub snr_db: f64,
    pub rcs_db: f64,
    pub jem_frequency_hz: Option<f64>,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TargetFingerprint {
    pub target_id: u32,
    pub classification: String,
    pub first_seen: String,
    pub duration_sec: f64,
    pub datapoints: Vec<FingerprintDatapoint>,
}

#[derive(Debug, Clone)]
pub struct TransientEvent {
    pub timestamp: String,
    pub time: Instant,
    pub frequency_hz: f64,
    pub snr_db: f64,
    pub classification: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TrackState {
    Suspect,
    Active,
    /// Signal lost; EKF coasting on prediction-only until MAX_COAST_FRAMES is reached.
    Coasting,
    Terminated,
}

#[derive(Debug, Clone)]
pub struct TrackedTarget {
    pub id: u32,
    pub ekf: BistaticEkf,
    pub state: TrackState,
    pub hits: usize,
    pub misses: usize,
    /// Number of consecutive frames spent in Coasting state.
    pub coasting_frames: u32,
    pub history: Vec<[f64; 6]>, // history of estimated states
    pub classification: String,
    pub terminated_at: Option<Instant>,
    pub start_time: Instant,
    pub fingerprint_history: Vec<FingerprintDatapoint>,
    pub jem: crate::tracking::jem::JemAnalyzer,
    pub tracking_towers: Vec<String>,
}

impl TrackedTarget {
    pub fn callsign(&self) -> String {
        let classification = self.classification.to_lowercase();
        if classification.contains("drone") || classification.contains("uav") {
            format!("DRN-{:02}", self.id)
        } else if classification.contains("ground") || classification.contains("vehicle") {
            format!("VEH-{:02}", self.id)
        } else if !self.classification.is_empty() && !self.classification.contains("Unknown") && !self.classification.contains("Target") {
            let clean_name = self.classification.split_whitespace().next().unwrap_or(&self.classification);
            let clean_name = clean_name.split('(').next().unwrap_or(clean_name).trim().to_uppercase();
            if clean_name.len() >= 3 && clean_name.chars().all(|c| c.is_alphanumeric()) {
                clean_name
            } else if clean_name == "COMMERCIAL" || clean_name == "PROPELLER" || clean_name == "TURBOPROP" || clean_name == "LIGHT" || clean_name == "HELICOPTER" || clean_name == "SUPERSONIC" || clean_name == "HIGH-ALTITUDE" {
                let prefix = match clean_name.as_str() {
                    "COMMERCIAL" => "AAL",
                    "PROPELLER" => "PRP",
                    "TURBOPROP" => "TRB",
                    "LIGHT" => "LGT",
                    "HELICOPTER" => "COP",
                    "SUPERSONIC" => "FTR",
                    _ => "JETA",
                };
                format!("{}-{:02}", prefix, self.id)
            } else {
                format!("FLT-{:02}", self.id)
            }
        } else {
            format!("TRK-{:02}", self.id)
        }
    }
}

#[derive(Debug, Clone)]
pub struct CandidatePlot {
    pub frequency: f64,
    pub hits: usize,
    pub misses: usize,
    pub updated_this_frame: bool,
    pub tower_freq: f64,
}

/// Compute the measurement Jacobian H (1x6) and predicted Doppler shift.
pub fn compute_measurement_jacobian(
    state: &[f64; 6],
    tower_pos: &[f64; 3],
    fc: f64,
) -> ([f64; 6], f64) {
    let x = state[0];
    let y = state[1];
    let z = state[2];
    let vx = state[3];
    let vy = state[4];
    let vz = state[5];

    let dx = x - tower_pos[0];
    let dy = y - tower_pos[1];
    let dz = z - tower_pos[2];

    let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

    let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
    let dot_r = (vx * x + vy * y + vz * z) / r_r;
    let range_rate_pred = dot_t + dot_r;

    let lambda = crate::sdr::C / fc;
    let z_pred = -range_rate_pred / lambda;

    let dr_dvx = dx / r_t + x / r_r;
    let dr_dvy = dy / r_t + y / r_r;
    let dr_dvz = dz / r_t + z / r_r;

    let d_t_x = vx * (r_t.powi(2) - dx * dx) / r_t.powi(3)
        - vy * (dx * dy) / r_t.powi(3)
        - vz * (dx * dz) / r_t.powi(3);
    let d_t_y = -vx * (dx * dy) / r_t.powi(3) + vy * (r_t.powi(2) - dy * dy) / r_t.powi(3)
        - vz * (dy * dz) / r_t.powi(3);
    let d_t_z = -vx * (dx * dz) / r_t.powi(3) - vy * (dy * dz) / r_t.powi(3)
        + vz * (r_t.powi(2) - dz * dz) / r_t.powi(3);

    let d_r_x = vx * (r_r.powi(2) - x * x) / r_r.powi(3)
        - vy * (x * y) / r_r.powi(3)
        - vz * (x * z) / r_r.powi(3);
    let d_r_y = -vx * (x * y) / r_r.powi(3) + vy * (r_r.powi(2) - y * y) / r_r.powi(3)
        - vz * (y * z) / r_r.powi(3);
    let d_r_z = -vx * (x * z) / r_r.powi(3) - vy * (y * z) / r_r.powi(3)
        + vz * (r_r.powi(2) - z * z) / r_r.powi(3);

    let dr_dx = d_t_x + d_r_x;
    let dr_dy = d_t_y + d_r_y;
    let dr_dz = d_t_z + d_r_z;

    let h = [
        -dr_dx / lambda,
        -dr_dy / lambda,
        -dr_dz / lambda,
        -dr_dvx / lambda,
        -dr_dvy / lambda,
        -dr_dvz / lambda,
    ];

    (h, z_pred)
}

/// Compute the squared Mahalanobis distance d_M^2 for a Doppler measurement.
pub fn mahalanobis_distance_doppler(
    state: &[f64; 6],
    cov: &[[f64; 6]; 6],
    r_variance: f64,
    tower_pos: &[f64; 3],
    fc: f64,
    z_meas: f64,
) -> (f64, f64) {
    let (h, z_pred) = compute_measurement_jacobian(state, tower_pos, fc);
    let mut h_p = [0.0; 6];
    for c in 0..6 {
        let mut val = 0.0;
        for r in 0..6 {
            val += h[r] * cov[r][c];
        }
        h_p[c] = val;
    }
    let mut h_p_ht = 0.0;
    for i in 0..6 {
        h_p_ht += h_p[i] * h[i];
    }
    let s = h_p_ht + r_variance;
    let innovation = z_meas - z_pred;
    let d_m_sq = (innovation * innovation) / s;
    (d_m_sq, z_pred)
}

fn get_disk_target_elapsed(fp: &TargetFingerprint, file_path: &std::path::Path) -> f64 {
    if let Ok(first_seen_naive) =
        chrono::NaiveDateTime::parse_from_str(&fp.first_seen, "%Y-%m-%d %H:%M:%S")
    {
        let last_dp_elapsed = fp
            .datapoints
            .last()
            .map(|dp| dp.time_elapsed_sec)
            .unwrap_or(0.0);
        if let Some(dur) =
            chrono::Duration::from_std(std::time::Duration::from_secs_f64(last_dp_elapsed)).ok()
        {
            let term_naive = first_seen_naive + dur;
            let elapsed_dur = chrono::Local::now().naive_local() - term_naive;
            let elapsed_sec = elapsed_dur.num_milliseconds() as f64 / 1000.0;
            if elapsed_sec >= 0.0 {
                return elapsed_sec;
            }
        }
    }
    // Fallback to file modification time
    if let Ok(metadata) = std::fs::metadata(file_path) {
        if let Ok(modified) = metadata.modified() {
            if let Ok(elapsed) = modified.elapsed() {
                return elapsed.as_secs_f64();
            }
        }
    }
    0.0
}

/// Computes the integrated Re-ID match score between a candidate and an offline track.
pub fn compute_reid_score(
    offline_target: &TrackedTarget,
    candidate_state: &[f64; 6],
    candidate_cov: &[[f64; 6]; 6],
    candidate_jem: Option<f64>,
    candidate_rcs_datapoints: &[(f64, f64)],
    dt: f64,
) -> f64 {
    // Process noise variance parameters (matching EKF initialization)
    let q_pos = 1.0;
    let q_vel = 0.1;

    // 1. Extrapolate offline target state and covariance
    let mut f_p = [[0.0; 6]; 6];
    for r in 0..6 {
        for c in 0..6 {
            if r < 3 {
                f_p[r][c] = offline_target.ekf.cov[r][c] + dt * offline_target.ekf.cov[r + 3][c];
            } else {
                f_p[r][c] = offline_target.ekf.cov[r][c];
            }
        }
    }

    let mut ext_cov = [[0.0; 6]; 6];
    for r in 0..6 {
        for c in 0..6 {
            let val = if c < 3 {
                f_p[r][c] + dt * f_p[r][c + 3]
            } else {
                f_p[r][c]
            };
            ext_cov[r][c] = val;
        }
    }

    // Add continuous Q(dt) to ext_cov
    for k in 0..3 {
        ext_cov[k][k] += (dt.powi(3) / 3.0) * q_pos;
        ext_cov[k + 3][k + 3] += dt * q_vel;
        ext_cov[k][k + 3] += (dt.powi(2) / 2.0) * q_vel;
        ext_cov[k + 3][k] += (dt.powi(2) / 2.0) * q_vel;
    }

    let mut ext_state = offline_target.ekf.state;
    ext_state[0] += ext_state[3] * dt;
    ext_state[1] += ext_state[4] * dt;
    ext_state[2] += ext_state[5] * dt;

    // Align candidate state quadrant if needed (resolves 1-tower hemisphere ambiguity)
    let mut candidate_state_mapped = *candidate_state;
    if offline_target.tracking_towers.len() <= 1 && candidate_state_mapped[0] * ext_state[0] < 0.0 {
        candidate_state_mapped[0] = -candidate_state_mapped[0];
        candidate_state_mapped[1] = -candidate_state_mapped[1];
        candidate_state_mapped[3] = -candidate_state_mapped[3];
        candidate_state_mapped[4] = -candidate_state_mapped[4];
    }

    // Compute Kinematic Score
    let mut dx = [0.0; 6];
    for k in 0..6 {
        dx[k] = candidate_state_mapped[k] - ext_state[k];
    }

    let mut p_sum = [[0.0; 6]; 6];
    for r in 0..6 {
        for c in 0..6 {
            p_sum[r][c] = candidate_cov[r][c] + ext_cov[r][c];
        }
    }

    let s_kin = if let Some(z) = cholesky_solve_6(&p_sum, &dx) {
        let mut d2 = 0.0;
        for k in 0..6 {
            d2 += dx[k] * z[k];
        }
        (-0.5 * d2).exp()
    } else {
        0.0
    };

    // 2. Compute JEM Score
    let mut w_jem = 0.20;
    let s_jem = match (offline_target.jem.get_sidebands_hz(), candidate_jem) {
        (Some(f_old), Some(f_cand)) => {
            let diff = f_cand - f_old;
            let sigma = 2.0; // 2 Hz frequency tolerance
            (-0.5 * (diff * diff) / (sigma * sigma)).exp()
        }
        _ => {
            w_jem = 0.0; // Exclude JEM if not available
            0.0
        }
    };

    // 3. Compute RCS Profile Score
    let mut w_rcs = 0.25;
    let s_rcs =
        if candidate_rcs_datapoints.is_empty() || offline_target.fingerprint_history.is_empty() {
            w_rcs = 0.0; // Exclude RCS if no datapoints are available
            0.0
        } else {
            // Bin offline target RCS by bistatic angle in 10-degree increments
            let mut old_bins = vec![Vec::new(); 18];
            for dp in &offline_target.fingerprint_history {
                let bin_idx = ((dp.bistatic_angle_deg.max(0.0) / 10.0).floor() as usize).min(17);
                old_bins[bin_idx].push(dp.rcs_db);
            }

            let mut cand_bins = vec![Vec::new(); 18];
            for &(angle, rcs) in candidate_rcs_datapoints {
                let bin_idx = ((angle.max(0.0) / 10.0).floor() as usize).min(17);
                cand_bins[bin_idx].push(rcs);
            }

            let mut diff_sum = 0.0;
            let mut overlap_count = 0;
            let rcs_uncert = 3.0; // 3.0 dB standard deviation

            for b in 0..18 {
                if !old_bins[b].is_empty() && !cand_bins[b].is_empty() {
                    let mean_old = old_bins[b].iter().sum::<f64>() / (old_bins[b].len() as f64);
                    let mean_cand = cand_bins[b].iter().sum::<f64>() / (cand_bins[b].len() as f64);
                    let diff = mean_cand - mean_old;
                    diff_sum += (diff * diff) / (rcs_uncert * rcs_uncert);
                    overlap_count += 1;
                }
            }

            if overlap_count > 0 {
                (-0.5 * diff_sum / (overlap_count as f64)).exp()
            } else {
                w_rcs = 0.0; // No overlapping bins, exclude RCS score
                0.0
            }
        };

    // Normalize weights
    let w_kin = 1.0 - w_jem - w_rcs;
    w_kin * s_kin + w_jem * s_jem + w_rcs * s_rcs
}

pub struct TrackingBank {
    pub targets: Vec<TrackedTarget>,
    pub candidates: Vec<CandidatePlot>,
    pub transients: Vec<TransientEvent>,
    pub mode: String,
    pub disk_fingerprints: Vec<(TargetFingerprint, std::path::PathBuf)>,

    pos_uncert: f64,
    vel_uncert: f64,
    r_variance: f64,
}

impl TrackingBank {
    pub fn new() -> Self {
        Self {
            targets: Vec::new(),
            candidates: Vec::new(),
            transients: Vec::new(),
            mode: "sim".to_string(),
            disk_fingerprints: Vec::new(),

            pos_uncert: 25_000.0, // 25 km initial position uncertainty
            vel_uncert: 120.0,    // 120 m/s initial velocity uncertainty
            r_variance: 4.0,      // 2 Hz measurement standard deviation squared
        }
    }

    /// Allocate the lowest available target ID not currently in use.
    /// Recycles IDs from pruned targets so display numbers stay compact (1, 2, 3...)
    /// instead of climbing monotonically to high values.
    fn allocate_id(targets: &[TrackedTarget]) -> u32 {
        let mut used: Vec<u32> = targets.iter().map(|t| t.id).collect();
        used.sort_unstable();
        let mut id = 1u32;
        for &u in &used {
            if u == id {
                id += 1;
            } else {
                break;
            }
        }
        id
    }

    pub fn get_fingerprints_dir(&self) -> std::path::PathBuf {
        let base = std::env::var("PASSIVERADAR_FINGERPRINTS_DIR")
            .map(std::path::PathBuf::from)
            .unwrap_or_else(|_| std::path::PathBuf::from("/Volumes/Storage/passiveradar/fingerprints"));
        base.join(&self.mode)
    }

    /// Load all serialized JSON target fingerprints from the mode-specific folder into memory.
    pub fn load_disk_fingerprints(&mut self) {
        self.disk_fingerprints.clear();
        let dir_path = self.get_fingerprints_dir();
        if let Ok(entries) = std::fs::read_dir(&dir_path) {
            for entry in entries {
                if let Ok(entry) = entry {
                    let path = entry.path();
                    if path.is_file() {
                        if let Some(filename) = path.file_name().and_then(|f| f.to_str()) {
                            if filename.starts_with("fingerprint_") && filename.ends_with(".json") {
                                if let Ok(file_content) = std::fs::read_to_string(&path) {
                                    if let Ok(fp) = serde_json::from_str::<TargetFingerprint>(&file_content) {
                                        self.disk_fingerprints.push((fp, path));
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    /// Estimate target type based on EKF altitude (z) and speed (magnitude of vx, vy, vz).
    pub fn classify_target(state: &[f64; 6]) -> String {
        let alt = state[2];
        let vx = state[3];
        let vy = state[4];
        let vz = state[5];
        let speed = (vx * vx + vy * vy + vz * vz).sqrt();

        if alt < 150.0 {
            if speed < 40.0 {
                "Ground Vehicle".to_string()
            } else {
                "Low-Alt UAV / Drone".to_string()
            }
        } else if alt < 2500.0 {
            if speed < 35.0 {
                "Helicopter".to_string()
            } else if speed < 110.0 {
                "Propeller Aircraft".to_string()
            } else {
                "Light Jet / Utility".to_string()
            }
        } else if alt < 13000.0 {
            if speed < 120.0 {
                "Turboprop Airliner".to_string()
            } else if speed < 280.0 {
                "Commercial Airliner".to_string()
            } else {
                "Supersonic Fighter Jet".to_string()
            }
        } else {
            "High-Altitude Jet / Recon".to_string()
        }
    }

    /// Update the tracking bank with a list of detected Doppler peaks from a tower (Compatibility Wrapper).
    pub fn update(
        &mut self,
        tower_pos: &[f64; 3],
        fc: f64,
        dt: f64,
        peaks: &[(f32, f32)],
        log: &mut Vec<String>,
    ) {
        let towers_data = vec![("WETA-FM".to_string(), *tower_pos, fc, peaks)];
        let empty_samples = vec![];
        self.update_multitower(&towers_data, dt, &empty_samples, log);
    }

    /// Update the tracking bank using multiple transmitter tower channels simultaneously.
    pub fn update_multitower(
        &mut self,
        towers_data: &[(String, [f64; 3], f64, &[(f32, f32)])],
        dt: f64,
        baseband_samples: &[Complex<f32>],
        log: &mut Vec<String>,
    ) {
        // Reset updated_this_frame flag for all candidates and clear tracking towers for all targets
        for cand in &mut self.candidates {
            cand.updated_this_frame = false;
        }
        for target in &mut self.targets {
            target.tracking_towers.clear();
        }

        // 1. Predict all active/suspect tracks forward
        for target in &mut self.targets {
            if target.state != TrackState::Terminated {
                target.ekf.predict(dt);
            }
        }

        // 2. Associate peaks to targets tower-by-tower, and run sequential updates using GNN with Mahalanobis distance gating
        let mut tower_associated_peaks = Vec::with_capacity(towers_data.len());
        let mut associated_for_target = vec![false; self.targets.len()];

        for &(ref name, tower_pos, fc, peaks) in towers_data {
            let mut associated_peaks = vec![false; peaks.len()];

            // Construct candidates list: (target_index, peak_index, distance)
            let mut candidates = Vec::new();

            for (t_idx, target) in self.targets.iter().enumerate() {
                if target.state == TrackState::Terminated {
                    continue;
                }

                for (p_idx, &(peak_freq, _snr)) in peaks.iter().enumerate() {
                    let (d_m_sq, _) = mahalanobis_distance_doppler(
                        &target.ekf.state,
                        &target.ekf.cov,
                        self.r_variance,
                        &tower_pos,
                        fc,
                        peak_freq as f64,
                    );

                    if d_m_sq <= 10.828 {
                        candidates.push((t_idx, p_idx, d_m_sq));
                    }
                }
            }

            // Sort candidates by Mahalanobis distance ascending
            candidates.sort_by(|a, b| a.2.partial_cmp(&b.2).unwrap_or(std::cmp::Ordering::Equal));

            // Greedy association
            let mut target_associated_in_tower = vec![false; self.targets.len()];

            for (t_idx, p_idx, _d2) in candidates {
                if target_associated_in_tower[t_idx] || associated_peaks[p_idx] {
                    continue;
                }

                target_associated_in_tower[t_idx] = true;
                associated_peaks[p_idx] = true;
                associated_for_target[t_idx] = true;

                let target = &mut self.targets[t_idx];
                let meas_doppler = peaks[p_idx].0 as f64;
                let snr_db = peaks[p_idx].1 as f64;

                // EKF measurement update for this tower!
                target.ekf.update(&tower_pos, fc, meas_doppler);

                // Target classification updates based on updated state
                target.classification = Self::classify_target(&target.ekf.state);

                // Record fingerprint data for this hit
                let state = target.ekf.state;
                let dx = state[0] - tower_pos[0];
                let dy = state[1] - tower_pos[1];
                let dz = state[2] - tower_pos[2];
                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2])
                    .sqrt()
                    .max(1.0);

                let cos_beta = (dx * state[0] + dy * state[1] + dz * state[2]) / (r_t * r_r);
                let bistatic_angle_rad = cos_beta.clamp(-1.0, 1.0).acos();
                let bistatic_angle_deg = bistatic_angle_rad.to_degrees();

                let rcs_db = snr_db + 20.0 * r_t.log10() + 20.0 * r_r.log10();
                let elapsed_sec = target.start_time.elapsed().as_secs_f64();

                target.fingerprint_history.push(FingerprintDatapoint {
                    time_elapsed_sec: elapsed_sec,
                    x_enu: state[0],
                    y_enu: state[1],
                    z_enu: state[2],
                    vx: state[3],
                    vy: state[4],
                    vz: state[5],
                    bistatic_angle_deg,
                    doppler_hz: meas_doppler,
                    snr_db,
                    rcs_db,
                    jem_frequency_hz: target.jem.get_sidebands_hz(),
                });

                // Add this tower to the list of tracking towers
                target.tracking_towers.push(name.clone());
            }

            tower_associated_peaks.push(associated_peaks);
        }

        // 2.5 Run JEM micro-Doppler analysis for all active/suspect targets using primary tower
        if !baseband_samples.is_empty() && !towers_data.is_empty() {
            let (_, tower_pos, fc, _) = towers_data[0];
            let lambda = crate::sdr::C / fc;

            for target in &mut self.targets {
                if target.state != TrackState::Terminated {
                    let x = target.ekf.state[0];
                    let y = target.ekf.state[1];
                    let z = target.ekf.state[2];
                    let vx = target.ekf.state[3];
                    let vy = target.ekf.state[4];
                    let vz = target.ekf.state[5];

                    let dx = x - tower_pos[0];
                    let dy = y - tower_pos[1];
                    let dz = z - tower_pos[2];
                    let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                    let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

                    let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
                    let dot_r = (vx * x + vy * y + vz * z) / r_r;
                    let pred_doppler = -(dot_t + dot_r) / lambda;

                    target.jem.process_block(pred_doppler, baseband_samples);
                }
            }
        }

        // 2.7 Cohomological Sheaf consistency check & Čech Obstruction pruning
        if towers_data.len() >= 3 {
            let mut to_prune = Vec::new();
            for target in &mut self.targets {
                if target.state != TrackState::Terminated && target.hits >= 15 {
                    let obs = Self::compute_cech_obstruction(&target.ekf.state, towers_data);
                    if obs > 300.0 {
                        log.push(format!(
                            "TrackedTarget Bank: Pruned inconsistent ghost Target {} ({}) due to Čech Obstruction (obs: {:.1} Hz)",
                            target.id, target.classification, obs
                        ));
                        to_prune.push(target.id);
                    }
                }
            }
            for pid in to_prune {
                if let Some(target) = self.targets.iter_mut().find(|t| t.id == pid) {
                    target.state = TrackState::Terminated;
                    target.terminated_at = Some(Instant::now());
                }
            }
        }

        // Update target hit/miss state counters, and manage Coasting/Terminated transitions.
        //
        // State machine:
        //   Suspect ──(≥3 hits)──► Active ──(max_misses missed)──► Coasting ──(dynamic_coast_frames)──► Terminated
        //                               └───────────────────────────────────── hit received ──────────┘
        
        for (t_idx, target) in self.targets.iter_mut().enumerate() {
            if target.state == TrackState::Terminated {
                continue;
            }

            if associated_for_target[t_idx] {
                target.hits += 1;
                target.misses = 0;
                target.coasting_frames = 0;

                // Restore coasting track to Active on hit
                if target.state == TrackState::Coasting {
                    target.state = TrackState::Active;
                    log.push(format!(
                        "TrackedTarget Bank: Recovered Target {} ({}) from coasting",
                        target.id, target.classification
                    ));
                }

                // Promote suspect to active after 3 hits
                if target.state == TrackState::Suspect && target.hits >= 3 {
                    target.state = TrackState::Active;
                    log.push(format!(
                        "TrackedTarget Bank: Confirmed Target {} ({})",
                        target.id, target.classification
                    ));
                }
            } else {
                // Already coasting — count coast frames toward termination
                if target.state == TrackState::Coasting {
                    target.coasting_frames += 1;
                    
                    // Dynamic coast limit: well-established tracks coast longer to survive large dropouts (time-based)
                    let max_coast_time = if target.hits > 50 {
                        40.0 // seconds
                    } else if target.hits > 20 {
                        20.0 // seconds
                    } else {
                        7.0  // seconds
                    };
                    let max_coast_frames = (max_coast_time / dt).round().max(1.0) as u32;

                    if target.coasting_frames >= max_coast_frames {
                        target.state = TrackState::Terminated;
                        target.terminated_at = Some(Instant::now());
                        log.push(format!(
                            "TrackedTarget Bank: Lost Target {} ({}) after coasting",
                            target.id, target.classification
                        ));
                    }
                } else {
                    target.misses += 1;

                    // Stickiness check
                    let alt = target.ekf.state[2];
                    let vx = target.ekf.state[3];
                    let vy = target.ekf.state[4];
                    let vz = target.ekf.state[5];
                    let speed = (vx * vx + vy * vy + vz * vz).sqrt();
                    let is_airliner =
                        (alt >= 2500.0 && alt < 13000.0 && speed >= 120.0 && speed < 280.0)
                            || target.classification.contains("Airliner")
                            || target.classification.contains("AAL")
                            || target.classification.contains("UAL");

                    // Raised thresholds: reduces churn from brief signal dropouts (time-based)
                    let max_miss_time = if is_airliner && target.state == TrackState::Active {
                        3.2 // seconds
                    } else if target.state == TrackState::Suspect {
                        2.3 // seconds
                    } else {
                        1.8 // seconds
                    };
                    let max_misses = (max_miss_time / dt).round().max(1.0) as usize;

                    if target.misses >= max_misses {
                        if target.state == TrackState::Active {
                            // Transition to Coasting instead of immediate termination
                            target.state = TrackState::Coasting;
                            target.coasting_frames = 0;
                            log.push(format!(
                                "TrackedTarget Bank: Target {} ({}) entering coast mode",
                                target.id, target.classification
                            ));
                        } else {
                            // Suspect tracks that time out terminate directly (no coast benefit)
                            target.state = TrackState::Terminated;
                            target.terminated_at = Some(Instant::now());
                        }
                    }
                }
            }

            // Save history (continue coasting into history so trail persists)
            if target.state != TrackState::Terminated {
                target.history.push(target.ekf.state);
                // Keep up to ~1.5 minutes of history for rendering
                if target.history.len() > 500 {
                    target.history.remove(0);
                }
            }
        }

        // 3. For any unassociated peaks, run Track Initiation (M-of-N Filter) using CandidatePlots
        for (t_idx, t_data) in towers_data.iter().enumerate() {
            let _tower_pos = t_data.1;
            let fc = t_data.2;
            let peaks = t_data.3;
            let associated_peaks = &tower_associated_peaks[t_idx];

            for (idx, &associated) in associated_peaks.iter().enumerate() {
                if associated {
                    continue;
                }

                let peak_doppler = peaks[idx].0 as f64;

                // Try to associate with an existing CandidatePlot for THIS tower
                let mut matched = false;
                for cand in &mut self.candidates {
                    if cand.tower_freq == fc && (cand.frequency - peak_doppler).abs() < 3.0 {
                        cand.frequency = 0.7 * cand.frequency + 0.3 * peak_doppler;
                        cand.hits += 1;
                        cand.misses = 0;
                        cand.updated_this_frame = true;
                        matched = true;
                        break;
                    }
                }

                if !matched {
                    self.candidates.push(CandidatePlot {
                        frequency: peak_doppler,
                        hits: 1,
                        misses: 0,
                        updated_this_frame: true,
                        tower_freq: fc,
                    });
                }
            }
        }

        // 4. Update and promote candidates, and prune old candidates
        let mut next_candidates = Vec::new();
        for mut cand in self.candidates.drain(..) {
            if cand.hits >= 3 {
                // Confirm track! We have seen it 3 times consecutively, spawn EKF suspect track
                let peak_doppler = cand.frequency;
                let direction_sign = -peak_doppler.signum(); // positive Doppler means approaching, negative receding

                let init_state = [
                    -20_000.0 * direction_sign, // x offset
                    -10_000.0 * direction_sign, // y offset
                    9500.0,                     // 9.5 km altitude
                    150.0 * direction_sign,     // vx
                    160.0 * direction_sign,     // vy
                    0.0,                        // vz
                ];

                // Check if this new candidate matches any recently terminated offline target (Re-identification)
                #[derive(Clone)]
                enum OfflineTargetSource {
                    InMemory {
                        index: usize,
                    },
                    FromDisk {
                        target: TrackedTarget,
                        file_path: std::path::PathBuf,
                    },
                }

                let mut offline_candidates: Vec<(OfflineTargetSource, TrackedTarget, f64)> =
                    Vec::new();

                // 1. Gather in-memory terminated targets
                for (idx, t) in self.targets.iter().enumerate() {
                    if t.state == TrackState::Terminated {
                        if let Some(t_time) = t.terminated_at {
                            let dt = t_time.elapsed().as_secs_f64();
                            offline_candidates.push((
                                OfflineTargetSource::InMemory { index: idx },
                                t.clone(),
                                dt,
                            ));
                        }
                    }
                }

                // 2. Gather disk target fingerprints from cache
                for (fp, path) in &self.disk_fingerprints {
                    // Check if not already in memory
                    if !self.targets.iter().any(|t| t.id == fp.target_id) {
                        // Reconstruct target
                        if !fp.datapoints.is_empty() {
                            let last_dp = fp.datapoints.last().unwrap();
                            let last_state = [
                                last_dp.x_enu,
                                last_dp.y_enu,
                                last_dp.z_enu,
                                last_dp.vx,
                                last_dp.vy,
                                last_dp.vz,
                            ];
                            let ekf = BistaticEkf::new(
                                last_state,
                                self.pos_uncert,
                                self.vel_uncert,
                                self.r_variance,
                            );
                            let history = fp
                                .datapoints
                                .iter()
                                .map(|dp| {
                                    [
                                        dp.x_enu, dp.y_enu, dp.z_enu,
                                        dp.vx, dp.vy, dp.vz,
                                    ]
                                })
                                .collect::<Vec<_>>();

                            let mut jem = crate::tracking::jem::JemAnalyzer::new();
                            let jem_freq = fp
                                .datapoints
                                .iter()
                                .filter_map(|dp| dp.jem_frequency_hz)
                                .last();
                            jem.set_sidebands_hz(jem_freq);

                            let start_time_instant = Instant::now()
                                - std::time::Duration::from_secs_f64(fp.duration_sec);

                            let target = TrackedTarget {
                                id: fp.target_id,
                                ekf,
                                state: TrackState::Terminated,
                                hits: fp.datapoints.len(),
                                misses: 0,
                                history,
                                classification: fp.classification.clone(),
                                terminated_at: None,
                                start_time: start_time_instant,
                                fingerprint_history: fp.datapoints.clone(),
                                jem,
                                tracking_towers: Vec::new(),
                                coasting_frames: 0,
                            };

                            let dt = get_disk_target_elapsed(fp, path);
                            offline_candidates.push((
                                OfflineTargetSource::FromDisk {
                                    target: target.clone(),
                                    file_path: path.clone(),
                                },
                                target,
                                dt,
                            ));
                        }
                    }
                }

                let mut candidate_cov = [[0.0f64; 6]; 6];
                for i in 0..3 {
                    candidate_cov[i][i] = self.pos_uncert.powi(2);
                    candidate_cov[i + 3][i + 3] = self.vel_uncert.powi(2);
                }

                let mut best_match: Option<(OfflineTargetSource, [f64; 6], f64)> = None;
                let mut best_score = -1.0;

                for (source, offline_target, dt) in offline_candidates {
                    let score = compute_reid_score(
                        &offline_target,
                        &init_state,
                        &candidate_cov,
                        None,
                        &[],
                        dt,
                    );

                    if score > best_score {
                        best_score = score;
                        let mut ext_state = offline_target.ekf.state;
                        ext_state[0] += ext_state[3] * dt;
                        ext_state[1] += ext_state[4] * dt;
                        ext_state[2] += ext_state[5] * dt;

                        best_match = Some((source, ext_state, score));
                    }
                }

                let mut reidentified = false;
                let mut reid_target_id = None;
                if let Some((source, best_ext_state, score)) = best_match {
                    if score >= 0.70 {
                        match source {
                            OfflineTargetSource::InMemory { index } => {
                                let target = &mut self.targets[index];
                                target.ekf = BistaticEkf::new(
                                    best_ext_state,
                                    self.pos_uncert,
                                    self.vel_uncert,
                                    self.r_variance,
                                );
                                target.state = TrackState::Suspect;
                                target.hits = 1;
                                target.misses = 0;
                                target.terminated_at = None;
                                log.push(format!(
                                    "TrackedTarget Bank: Re-identified Target {} ({}) from memory (score: {:.4})",
                                    target.id, target.classification, score
                                ));
                            }
                            OfflineTargetSource::FromDisk {
                                mut target,
                                file_path,
                            } => {
                                target.ekf = BistaticEkf::new(
                                    best_ext_state,
                                    self.pos_uncert,
                                    self.vel_uncert,
                                    self.r_variance,
                                );
                                target.state = TrackState::Suspect;
                                target.hits = 1;
                                target.misses = 0;
                                target.terminated_at = None;
                                log.push(format!(
                                    "TrackedTarget Bank: Re-identified Target {} ({}) from disk (score: {:.4})",
                                    target.id, target.classification, score
                                ));
                                reid_target_id = Some(target.id);
                                self.targets.push(target);
                                let _ = std::fs::remove_file(&file_path);
                            }
                        }
                        reidentified = true;
                    }
                }

                if let Some(pid) = reid_target_id {
                    self.disk_fingerprints.retain(|(fp, _)| fp.target_id != pid);
                }

                if reidentified {
                    // Skip spawning a new track
                } else {
                    // Pre-filtering check: count towers with a matching unassociated peak
                    let mut matching_towers = 0;
                    for (t_idx, t_data) in towers_data.iter().enumerate() {
                        let tower_pos = t_data.1;
                        let fc = t_data.2;
                        let peaks = t_data.3;
                        let associated_peaks = &tower_associated_peaks[t_idx];
                        
                        let (_, pred_doppler) = compute_measurement_jacobian(&init_state, &tower_pos, fc);
                        let mut has_match = false;
                        for (p_idx, &(peak_freq, _)) in peaks.iter().enumerate() {
                            if associated_peaks[p_idx] {
                                continue;
                            }
                            if (peak_freq as f64 - pred_doppler).abs() < 60.0 {
                                has_match = true;
                                break;
                            }
                        }
                        if has_match {
                            matching_towers += 1;
                        }
                    }

                    if matching_towers < 2 && towers_data.len() >= 2 {
                        // Skip running Adelic solver completely since target is not visible on enough towers,
                        // but do not discard the candidate; let it fall back to init_state.
                    }

                    // Try to resolve exact 3D coordinates using Adelic Langevin multilateration (Frontier B)
                    let mut resolved_state = init_state;

                    if towers_data.len() >= 2 && matching_towers >= 2 {
                        let mut opt = crate::math::adelic::AdelicLangevinOptimizer::new();
                        let bounds = [
                            (-100_000.0, 100_000.0),
                            (-100_000.0, 100_000.0),
                            (150.0, 13_000.0),
                            (-350.0, 350.0),
                            (-350.0, 350.0),
                            (-50.0, 50.0),
                        ];

                        let cost_fn = |state: &[f64; 6]| {
                            let mut rss = 0.0;
                            for (t_idx, t_data) in towers_data.iter().enumerate() {
                                let tower_pos = t_data.1;
                                let fc = t_data.2;
                                let peaks = t_data.3;
                                let x = state[0];
                                let y = state[1];
                                let z = state[2];
                                let vx = state[3];
                                let vy = state[4];
                                let vz = state[5];

                                let dx = x - tower_pos[0];
                                let dy = y - tower_pos[1];
                                let dz = z - tower_pos[2];
                                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                                let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

                                let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
                                let dot_r = (vx * x + vy * y + vz * z) / r_r;

                                let lambda = crate::sdr::C / fc;
                                let pred_doppler = -(dot_t + dot_r) / lambda;

                                let associated_peaks = &tower_associated_peaks[t_idx];
                                let mut min_diff = 100.0; // wider gate for initial search
                                for (p_idx, &(peak_freq, _)) in peaks.iter().enumerate() {
                                    if associated_peaks[p_idx] {
                                        continue;
                                    }
                                    let diff = (peak_freq as f64 - pred_doppler).abs();
                                    if diff < min_diff {
                                        min_diff = diff;
                                    }
                                }
                                rss += min_diff * min_diff;
                            }
                            rss
                        };

                        let (best_state, best_rss) =
                            opt.optimize(init_state, &bounds, cost_fn, 100);
                        if best_rss < 200.0 {
                            resolved_state = best_state;
                            log.push(format!("TrackedTarget Bank: Resolved 3D multilateration for new target (RSS: {:.1})", best_rss));
                        } else {
                            log.push(format!("TrackedTarget Bank: Multilateration RSS {:.1} above threshold, falling back to initial track state", best_rss));
                        }
                    }

                    let ekf = BistaticEkf::new(
                        resolved_state,
                        self.pos_uncert,
                        self.vel_uncert,
                        self.r_variance,
                    );
                    let classification = Self::classify_target(&resolved_state);

                    let new_target = TrackedTarget {
                        id: Self::allocate_id(&self.targets),
                        ekf,
                        state: TrackState::Suspect,
                        hits: 1,
                        misses: 0,
                        history: vec![resolved_state],
                        classification,
                        terminated_at: None,
                        start_time: Instant::now(),
                        fingerprint_history: Vec::new(),
                        jem: crate::tracking::jem::JemAnalyzer::new(),
                        tracking_towers: Vec::new(),
                        coasting_frames: 0,
                    };

                    self.targets.push(new_target);
                }
            } else {
                // If it missed this frame, increment misses
                if !cand.updated_this_frame {
                    cand.misses += 1;
                }

                // Keep candidate if misses are below threshold
                if cand.misses < 3 {
                    next_candidates.push(cand);
                }
            }
        }
        self.candidates = next_candidates;

        // Run duplicate track prevention before transient detection
        Self::prevent_duplicate_tracks(&mut self.targets, log);

        // 5. Detect and record high-frequency transients (meteors, lightning, etc.)
        for (t_idx, t_data) in towers_data.iter().enumerate() {
            let fc = t_data.2;
            let peaks = t_data.3;
            let associated_peaks = &tower_associated_peaks[t_idx];
            for (idx, &associated) in associated_peaks.iter().enumerate() {
                if associated {
                    continue;
                }
                let peak_doppler = peaks[idx].0 as f64;
                let peak_snr = peaks[idx].1 as f64;

                // Only report strong transients (SNR >= 12.0 dB) to prevent thermal noise spam
                if peak_doppler.abs() > 300.0 && peak_snr >= 12.0 {
                    let now = Instant::now();
                    let duplicate = self.transients.iter().any(|t| {
                        now.duration_since(t.time) < std::time::Duration::from_secs_f64(2.0)
                            && (t.frequency_hz - peak_doppler).abs() < 150.0
                    });

                    if !duplicate {
                        let timestamp = chrono::Local::now().format("%H:%M:%S").to_string();
                        let lambda = crate::sdr::C / fc;
                        let approx_radial_speed = (peak_doppler.abs() * lambda) / 2.0;
                        let speed_kms = approx_radial_speed / 1000.0;

                        let classification = if peak_doppler.abs() > 1000.0 {
                            format!("Fast Meteor Ping ({:.1} km/s)", speed_kms)
                        } else if peak_doppler.abs() > 500.0 {
                            format!("Ionized Meteor Trail ({:.1} km/s)", speed_kms)
                        } else {
                            format!("Atmospheric Transient ({:.1} km/s)", speed_kms)
                        };

                        self.transients.insert(
                            0,
                            TransientEvent {
                                timestamp,
                                time: now,
                                frequency_hz: peak_doppler,
                                snr_db: peak_snr,
                                classification,
                            },
                        );

                        if self.transients.len() > 10 {
                            self.transients.pop();
                        }

                        log.push(format!(
                            "Atmospheric: Detected {} at {:.1} Hz",
                            self.transients[0].classification, peak_doppler
                        ));
                    }
                }
            }
        }

        // 6. Save fingerprints for targets about to be pruned
        for t in &self.targets {
            if t.state == TrackState::Terminated {
                if let Some(t_time) = t.terminated_at {
                    let is_identified_airliner = t.classification.contains("(")
                        || t.classification.contains("Airliner")
                        || t.classification.contains("AAL")
                        || t.classification.contains("UAL");

                    let timeout = if is_identified_airliner {
                        std::time::Duration::from_secs(90)
                    } else {
                        std::time::Duration::from_secs(10)
                    };

                    if t_time.elapsed() >= timeout {
                        if t.fingerprint_history.len() >= 5 {
                            let duration = t.start_time.elapsed().as_secs_f64();
                            let first_seen_str = chrono::Local::now()
                                .checked_sub_signed(
                                    chrono::Duration::from_std(t.start_time.elapsed())
                                        .unwrap_or_else(|_| chrono::Duration::zero()),
                                )
                                .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                                .unwrap_or_else(|| "Unknown".to_string());

                            let fp = TargetFingerprint {
                                target_id: t.id,
                                classification: t.classification.clone(),
                                first_seen: first_seen_str,
                                duration_sec: duration,
                                datapoints: t.fingerprint_history.clone(),
                            };

                            let dir_path = self.get_fingerprints_dir();
                            let _ = std::fs::create_dir_all(&dir_path);

                            let clean_class = t
                                .classification
                                .chars()
                                .map(|c| if c.is_alphanumeric() { c } else { '_' })
                                .collect::<String>();

                            let file_name = format!("fingerprint_{}_{}.json", clean_class, t.id);
                            let file_path = dir_path.join(file_name);

                            if let Ok(serialized) = serde_json::to_string_pretty(&fp) {
                                if let Ok(mut file) = std::fs::File::create(&file_path) {
                                    use std::io::Write;
                                    let _ = file.write_all(serialized.as_bytes());
                                    log.push(format!(
                                        "TrackedTarget Bank: Saved fingerprint for Target {} to {}",
                                        t.id,
                                        file_path.display()
                                    ));
                                    self.disk_fingerprints.push((fp, file_path));
                                }
                            }
                        }
                    }
                }
            }
        }

        // 7. Remove terminated targets after a timeout to keep them on list with "OFFLINE" indicator
        self.targets.retain(|t| {
            if t.state == TrackState::Terminated {
                if let Some(t_time) = t.terminated_at {
                    let is_identified_airliner = t.classification.contains("(")
                        || t.classification.contains("Airliner")
                        || t.classification.contains("AAL")
                        || t.classification.contains("UAL");

                    let timeout = if is_identified_airliner {
                        std::time::Duration::from_secs(90)
                    } else {
                        std::time::Duration::from_secs(10)
                    };
                    t_time.elapsed() < timeout
                } else {
                    false
                }
            } else {
                true
            }
        });
    }

    /// Scans active and suspect targets and resolves duplicate/redundant tracks.
    pub fn prevent_duplicate_tracks(targets: &mut Vec<TrackedTarget>, log: &mut Vec<String>) {
        let n = targets.len();
        if n < 2 {
            return;
        }

        let mut to_remove = std::collections::HashSet::new();
        let gate_threshold = 12.592; // Chi-squared 6 DOF, 95% confidence

        fn is_generic_classification(c: &str) -> bool {
            matches!(
                c,
                "Ground Vehicle"
                    | "Helicopter"
                    | "Propeller Aircraft"
                    | "Light Jet / Utility"
                    | "Turboprop Airliner"
                    | "Commercial Airliner"
                    | "Supersonic Fighter Jet"
                    | "High-Altitude Jet / Recon"
                    | "Low-Alt UAV / Drone"
                    | "Target"
                    | "Suspect"
                    | "Active"
                    | "Unknown"
                    | ""
            )
        }

        for i in 0..n {
            if (targets[i].state != TrackState::Active
                && targets[i].state != TrackState::Suspect
                && targets[i].state != TrackState::Coasting)
                || to_remove.contains(&targets[i].id)
            {
                continue;
            }
            for j in (i + 1)..n {
                if (targets[j].state != TrackState::Active
                    && targets[j].state != TrackState::Suspect
                    && targets[j].state != TrackState::Coasting)
                    || to_remove.contains(&targets[j].id)
                {
                    continue;
                }

                // Compute difference vector
                let mut dx = [0.0; 6];
                for k in 0..6 {
                    dx[k] = targets[i].ekf.state[k] - targets[j].ekf.state[k];
                }

                // 2D horizontal Euclidean distance gate (10.0 km) to prevent merging mirror/ghost tracks
                let dist_2d_sq = dx[0] * dx[0] + dx[1] * dx[1];
                if dist_2d_sq > 10_000.0 * 10_000.0 {
                    continue;
                }

                // Compute covariance sum
                let mut p_sum = [[0.0; 6]; 6];
                for r in 0..6 {
                    for c in 0..6 {
                        p_sum[r][c] = targets[i].ekf.cov[r][c] + targets[j].ekf.cov[r][c];
                    }
                }

                // Compute Mahalanobis distance squared using Cholesky solver
                if let Some(z) = cholesky_solve_6(&p_sum, &dx) {
                    let mut d2 = 0.0;
                    for k in 0..6 {
                        d2 += dx[k] * z[k];
                    }

                    if d2 <= gate_threshold {
                        // Duplicate detected! Apply subsumption / fusion
                        // Prioritize target with more hits (longer history) or lower ID (older track)
                        let (superior_idx, inferior_idx) = {
                            if targets[i].hits > targets[j].hits {
                                (i, j)
                            } else if targets[i].hits < targets[j].hits {
                                (j, i)
                            } else if targets[i].id <= targets[j].id {
                                (i, j)
                            } else {
                                (j, i)
                            }
                        };

                        log.push(format!(
                            "TrackedTarget Bank: Merging duplicate target {} into superior target {}",
                            targets[inferior_idx].id, targets[superior_idx].id
                        ));

                        // Inherit classification if superior is generic and inferior is specific
                        let is_sup_generic =
                            is_generic_classification(&targets[superior_idx].classification);
                        let is_inf_generic =
                            is_generic_classification(&targets[inferior_idx].classification);
                        if is_sup_generic && !is_inf_generic {
                            targets[superior_idx].classification =
                                targets[inferior_idx].classification.clone();
                        }

                        // Inherit history (both EKF state history and fingerprint history)
                        if targets[superior_idx].history.len() < targets[inferior_idx].history.len()
                        {
                            targets[superior_idx].history = targets[inferior_idx].history.clone();
                        }

                        // Merge fingerprint history
                        let mut merged_fp = targets[superior_idx].fingerprint_history.clone();
                        for dp in &targets[inferior_idx].fingerprint_history {
                            if !merged_fp.iter().any(|existing| {
                                (existing.time_elapsed_sec - dp.time_elapsed_sec).abs() < 1e-5
                            }) {
                                merged_fp.push(dp.clone());
                            }
                        }
                        merged_fp.sort_by(|a, b| {
                            a.time_elapsed_sec
                                .partial_cmp(&b.time_elapsed_sec)
                                .unwrap_or(std::cmp::Ordering::Equal)
                        });
                        targets[superior_idx].fingerprint_history = merged_fp;

                        // Inherit JEM analyzer if superior has no sidebands but inferior does
                        if targets[superior_idx].jem.get_sidebands_hz().is_none()
                            && targets[inferior_idx].jem.get_sidebands_hz().is_some()
                        {
                            targets[superior_idx].jem = targets[inferior_idx].jem.clone();
                        }

                        // Run Covariance Intersection to merge the EKF state estimate
                        if let Some((x_m, p_m)) = covariance_intersection_merge(
                            &targets[superior_idx].ekf.state,
                            &targets[superior_idx].ekf.cov,
                            &targets[inferior_idx].ekf.state,
                            &targets[inferior_idx].ekf.cov,
                        ) {
                            targets[superior_idx].ekf.state = x_m;
                            targets[superior_idx].ekf.cov = p_m;
                        }

                        to_remove.insert(targets[inferior_idx].id);
                    }
                }
            }
        }

        // Prune tracks marked for removal
        targets.retain(|t| !to_remove.contains(&t.id));
    }

    /// Computes the Čech obstruction cycle (absolute sum of cyclic coboundaries)
    /// across active towers to verify target spatial/Doppler consistency.
    pub fn compute_cech_obstruction(
        state: &[f64; 6],
        towers_data: &[(String, [f64; 3], f64, &[(f32, f32)])],
    ) -> f64 {
        let mut total_error = 0.0;
        let x = state[0];
        let y = state[1];
        let z = state[2];
        let vx = state[3];
        let vy = state[4];
        let vz = state[5];

        let r_r = (x * x + y * y + z * z).sqrt().max(1.0);
        let dot_r = (vx * x + vy * y + vz * z) / r_r;

        for (_name, tower_pos, fc, peaks) in towers_data {
            let dx = x - tower_pos[0];
            let dy = y - tower_pos[1];
            let dz = z - tower_pos[2];
            let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);

            let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;

            let lambda = crate::sdr::C / fc;
            let pred_doppler = -(dot_t + dot_r) / lambda;

            let mut min_diff = f64::MAX;
            for (peak_freq, _snr) in *peaks {
                let diff = (*peak_freq as f64 - pred_doppler).abs();
                if diff < min_diff {
                    min_diff = diff;
                }
            }

            // If a tower has no peaks, we can either add 0 or add a penalty. The instruction says: 
            // "then find the minimum absolute difference |peak_freq - pred_doppler| among its peaks, and sum these minimum differences across all towers."
            if min_diff != f64::MAX {
                total_error += min_diff;
            }
        }

        total_error
    }
}

impl Drop for TrackingBank {
    fn drop(&mut self) {
        let dir_path = self.get_fingerprints_dir();
        let _ = std::fs::create_dir_all(&dir_path);

        for t in &self.targets {
            if t.fingerprint_history.len() >= 5 {
                let duration = t.start_time.elapsed().as_secs_f64();
                let first_seen_str = chrono::Local::now()
                    .checked_sub_signed(
                        chrono::Duration::from_std(t.start_time.elapsed())
                            .unwrap_or_else(|_| chrono::Duration::zero()),
                    )
                    .map(|dt| dt.format("%Y-%m-%d %H:%M:%S").to_string())
                    .unwrap_or_else(|| "Unknown".to_string());

                let fp = TargetFingerprint {
                    target_id: t.id,
                    classification: t.classification.clone(),
                    first_seen: first_seen_str,
                    duration_sec: duration,
                    datapoints: t.fingerprint_history.clone(),
                };

                let clean_class = t
                    .classification
                    .chars()
                    .map(|c| if c.is_alphanumeric() { c } else { '_' })
                    .collect::<String>();

                let file_name = format!("fingerprint_{}_{}.json", clean_class, t.id);
                let file_path = dir_path.join(file_name);

                if let Ok(serialized) = serde_json::to_string_pretty(&fp) {
                    if let Ok(mut file) = std::fs::File::create(&file_path) {
                        use std::io::Write;
                        let _ = file.write_all(serialized.as_bytes());
                    }
                }
            }
        }
    }
}

/// Cholesky Decomposition (L * L^T = A) for a symmetric positive definite 6x6 matrix.
/// Returns lower triangular matrix L.
pub fn cholesky_6x6(a: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    let mut l = [[0.0; 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let mut sum = 0.0;
            for k in 0..j {
                sum += l[i][k] * l[j][k];
            }
            if i == j {
                let val = a[i][i] - sum;
                if val <= 1e-12 {
                    return None; // Not positive-definite
                }
                l[i][j] = val.sqrt();
            } else {
                l[i][j] = (a[i][j] - sum) / l[j][j];
            }
        }
    }
    Some(l)
}

/// Solve L * Y = B using forward substitution (L is 6x6 lower triangular).
pub fn forward_substitute_6(l: &[[f64; 6]; 6], b: &[f64; 6]) -> [f64; 6] {
    let mut y = [0.0; 6];
    for i in 0..6 {
        let mut sum = 0.0;
        for k in 0..i {
            sum += l[i][k] * y[k];
        }
        y[i] = (b[i] - sum) / l[i][i];
    }
    y
}

/// Solve L^T * X = Y using backward substitution (L is 6x6 lower triangular).
pub fn backward_substitute_6(l: &[[f64; 6]; 6], y: &[f64; 6]) -> [f64; 6] {
    let mut x = [0.0; 6];
    for i in (0..6).rev() {
        let mut sum = 0.0;
        for k in (i + 1)..6 {
            sum += l[k][i] * x[k]; // Note l[k][i] is (L^T)[i][k]
        }
        x[i] = (y[i] - sum) / l[i][i];
    }
    x
}

/// Solve A * X = B for 6x6 SPD matrix A using Cholesky.
pub fn cholesky_solve_6(a: &[[f64; 6]; 6], b: &[f64; 6]) -> Option<[f64; 6]> {
    let l = cholesky_6x6(a)?;
    let y = forward_substitute_6(&l, b);
    Some(backward_substitute_6(&l, &y))
}

/// Invert a symmetric positive definite 6x6 matrix using Cholesky solvers.
pub fn invert_spd_6x6(a: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    let mut inv = [[0.0; 6]; 6];
    let l = cholesky_6x6(a)?;

    for c in 0..6 {
        let mut e = [0.0; 6];
        e[c] = 1.0;
        let y = forward_substitute_6(&l, &e);
        let x = backward_substitute_6(&l, &y);
        for r in 0..6 {
            inv[r][c] = x[r];
        }
    }
    Some(inv)
}

/// Merges duplicate tracks using Covariance Intersection (CI).
/// Optimizes omega by scanning [0, 1] with a step size of 0.05.
pub fn covariance_intersection_merge(
    x_a: &[f64; 6],
    p_a: &[[f64; 6]; 6],
    x_b: &[f64; 6],
    p_b: &[[f64; 6]; 6],
) -> Option<([f64; 6], [[f64; 6]; 6])> {
    let inv_p_a = invert_spd_6x6(p_a)?;
    let inv_p_b = invert_spd_6x6(p_b)?;

    let mut best_omega = 0.5;
    let mut min_trace = f64::MAX;
    let mut best_inv_p_m = [[0.0; 6]; 6];

    // Scan for optimal omega minimizing the trace of the resulting covariance
    for i in 0..=20 {
        let omega = (i as f64) * 0.05;
        let mut inv_p_m_candidate = [[0.0; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                inv_p_m_candidate[r][c] = omega * inv_p_a[r][c] + (1.0 - omega) * inv_p_b[r][c];
            }
        }
        if let Some(p_m_candidate) = invert_spd_6x6(&inv_p_m_candidate) {
            let trace: f64 = (0..6).map(|idx| p_m_candidate[idx][idx]).sum();
            if trace < min_trace {
                min_trace = trace;
                best_omega = omega;
                best_inv_p_m = inv_p_m_candidate;
            }
        }
    }

    let p_m = invert_spd_6x6(&best_inv_p_m)?;

    // Compute X_m = P_m * (omega * inv_P_A * X_A + (1 - omega) * inv_P_B * X_B)
    let mut term_a = [0.0; 6];
    let mut term_b = [0.0; 6];
    for r in 0..6 {
        for c in 0..6 {
            term_a[r] += inv_p_a[r][c] * x_a[c];
            term_b[r] += inv_p_b[r][c] * x_b[c];
        }
    }

    let mut combined_rhs = [0.0; 6];
    for r in 0..6 {
        combined_rhs[r] = best_omega * term_a[r] + (1.0 - best_omega) * term_b[r];
    }

    let mut x_m = [0.0; 6];
    for r in 0..6 {
        for c in 0..6 {
            x_m[r] += p_m[r][c] * combined_rhs[c];
        }
    }

    Some((x_m, p_m))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tracking::ekf::BistaticEkf;

    #[test]
    fn test_airliner_stickiness() {
        let mut bank = TrackingBank::new();

        // Target 1: A normal UAV/Drone target
        let drone_state = [100.0, 100.0, 100.0, 10.0, 10.0, 0.0];
        let drone_ekf = BistaticEkf::new(drone_state, 1000.0, 10.0, 1.0);
        bank.targets.push(TrackedTarget {
            id: 1,
            ekf: drone_ekf,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![drone_state],
            classification: "Low-Alt UAV / Drone".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        });

        // Target 2: An airliner target (by state)
        let airliner_state1 = [1000.0, 1000.0, 8000.0, 150.0, 150.0, 0.0];
        let airliner_ekf1 = BistaticEkf::new(airliner_state1, 1000.0, 10.0, 1.0);
        bank.targets.push(TrackedTarget {
            id: 2,
            ekf: airliner_ekf1,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![airliner_state1],
            classification: "Commercial Airliner".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        });

        // Target 3: An airliner target (by classification string)
        let airliner_state2 = [5000.0, 5000.0, 100.0, 10.0, 10.0, 0.0];
        let airliner_ekf2 = BistaticEkf::new(airliner_state2, 1000.0, 10.0, 1.0);
        bank.targets.push(TrackedTarget {
            id: 3,
            ekf: airliner_ekf2,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![airliner_state2],
            classification: "AAL191 (B788)".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        });

        let tower_pos = [0.0, 0.0, 0.0];
        let fc = 90.9e6;
        let dt = 0.1;
        let mut log = Vec::new();

        let empty_peaks: &[(f32, f32)] = &[];

        // Run updates with NO peaks for 18 iterations (new time-based threshold for non-airliner Active tracks)
        for _ in 0..18 {
            bank.update(&tower_pos, fc, dt, empty_peaks, &mut log);
        }

        // After 18 iterations:
        // - Drone target (ID 1) should be Coasting (threshold 1.8s / 0.1s = 18 misses)
        // - Airliners (ID 2 and 3) should still be active (threshold 3.2s / 0.1s = 32 misses)
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 1).unwrap().state,
            TrackState::Coasting,
            "Drone target should be coasting after 18 misses"
        );
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 2).unwrap().state,
            TrackState::Active,
            "Airliner target 1 should still be active"
        );
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 3).unwrap().state,
            TrackState::Active,
            "Airliner target 2 should still be active"
        );

        // Run 70 more iterations to push the drone through coasting into Terminated
        // (New dynamic coasting limit for low-hit tracks is 7.0s / 0.1s = 70 frames)
        for _ in 0..70 {
            bank.update(&tower_pos, fc, dt, empty_peaks, &mut log);
        }

        // After 18 + 70 = 88 total iterations: drone should be Terminated
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 1).unwrap().state,
            TrackState::Terminated,
            "Drone target should be terminated after coasting"
        );

        // Mock the drone target's terminated_at to be 11 seconds ago to verify pruning
        if let Some(t) = bank.targets.iter_mut().find(|t| t.id == 1) {
            t.terminated_at = Some(Instant::now() - std::time::Duration::from_secs(11));
        }

        // Run one update to trigger pruning of drone target
        bank.update(&tower_pos, fc, dt, empty_peaks, &mut log);
        assert!(
            bank.targets.iter().all(|t| t.id != 1),
            "Drone target should be pruned after timeout"
        );
        assert!(
            bank.targets.iter().any(|t| t.id == 2),
            "Airliner target 1 should still be on screen"
        );
        assert!(
            bank.targets.iter().any(|t| t.id == 3),
            "Airliner target 2 should still be on screen"
        );

        // Run enough additional iterations to push airliners through Coasting -> Terminated
        // Airliners entered coasting at miss 32, and need 70 coasting frames.
        // Currently at iteration 89 (89 - 32 = 57 coast frames). Need 13 more, run 15.
        for _ in 0..15 {
            bank.update(&tower_pos, fc, dt, empty_peaks, &mut log);
        }

        // Now both airliner targets should be terminated
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 2).unwrap().state,
            TrackState::Terminated,
            "Airliner 1 should be offline"
        );
        assert_eq!(
            bank.targets.iter().find(|t| t.id == 3).unwrap().state,
            TrackState::Terminated,
            "Airliner 2 should be offline"
        );

        // Mock airliner targets' terminated_at to be 91 seconds ago to verify final roll-off
        for t in &mut bank.targets {
            t.terminated_at = Some(Instant::now() - std::time::Duration::from_secs(91));
        }

        // Run update to trigger final roll-off/pruning
        bank.update(&tower_pos, fc, dt, empty_peaks, &mut log);
        assert!(
            bank.targets.is_empty(),
            "All targets should be rolled off and pruned after timeouts"
        );
    }

    #[test]
    fn test_meteor_transient_detection() {
        let mut bank = TrackingBank::new();
        let tower_pos = [0.0, 0.0, 0.0];
        let fc = 90.9e6;
        let dt = 0.1;
        let mut log = Vec::new();

        // 1. Update with a high Doppler peak (1200 Hz) at 15.0 dB SNR
        bank.update(&tower_pos, fc, dt, &[(1200.0, 15.0)], &mut log);

        // Verify transient was registered
        assert_eq!(bank.transients.len(), 1, "Should detect 1 transient event");
        assert_eq!(bank.transients[0].frequency_hz, 1200.0);
        assert!(
            bank.transients[0].classification.contains("Meteor"),
            "Should classify as meteor"
        );

        // 2. Update immediately again with the same peak (should be ignored as duplicate)
        bank.update(&tower_pos, fc, dt, &[(1200.0, 15.0)], &mut log);
        assert_eq!(
            bank.transients.len(),
            1,
            "Should de-duplicate back-to-back similar transients"
        );

        // 3. Update with a peak at a significantly different frequency (e.g., -600 Hz) at 14.0 dB SNR
        bank.update(&tower_pos, fc, dt, &[(-600.0, 14.0)], &mut log);
        assert_eq!(
            bank.transients.len(),
            2,
            "Should detect new transient at different frequency"
        );
        assert_eq!(bank.transients[0].frequency_hz, -600.0);
    }

    #[test]
    fn test_target_re_identification() {
        let mut bank = TrackingBank::new();
        let tower_pos = [0.0, 0.0, 0.0];
        let fc = 90.9e6;
        let dt = 0.1;
        let mut log = Vec::new();

        // 1. Manually add a terminated target (ID 42) to the bank.
        // It was traveling at x= -20km, y= -10km, altitude= 9.5km, speed= 150m/s, 160m/s
        let initial_pos = [-20_000.0, -10_000.0, 9500.0];
        let state = [
            initial_pos[0],
            initial_pos[1],
            initial_pos[2],
            150.0,
            160.0,
            0.0,
        ];
        let ekf = BistaticEkf::new(state, 1000.0, 10.0, 1.0);

        // Terminate it 5 seconds ago
        let terminated_time = Instant::now() - std::time::Duration::from_secs(5);
        bank.targets.push(TrackedTarget {
            id: 42,
            ekf,
            state: TrackState::Terminated,
            hits: 10,
            misses: 18,
            history: vec![state],
            classification: "AAL191 (B788)".to_string(),
            terminated_at: Some(terminated_time),
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
            coasting_frames: 0,
        });

        // 2. Add a CandidatePlot that is close to the expected position
        // Seed candidate: frequency = 113.0 Hz (matching extrapolated Doppler)
        bank.update(&tower_pos, fc, dt, &[(113.0, 15.0)], &mut log);
        bank.update(&tower_pos, fc, dt, &[(113.0, 15.0)], &mut log);
        bank.update(&tower_pos, fc, dt, &[(113.0, 15.0)], &mut log); // 3rd hit triggers promotion

        // 3. Verify that the newly promoted candidate was mapped to the original target ID 42
        // rather than spawning a new target ID 1
        assert!(
            bank.targets.iter().any(|t| t.id == 42),
            "Target 42 should still exist"
        );
        let target_42 = bank.targets.iter().find(|t| t.id == 42).unwrap();
        assert_eq!(
            target_42.state,
            TrackState::Suspect,
            "Target 42 should have been reactivated to Suspect state"
        );
        assert_eq!(
            target_42.terminated_at, None,
            "Target 42's terminated_at should be cleared"
        );
        assert!(
            bank.targets.iter().all(|t| t.id != 1),
            "Should NOT have spawned new target ID 1"
        );
    }

    #[test]
    fn test_fingerprint_collection() {
        let mut bank = TrackingBank::new();
        let tower_pos = [10_000.0, 5000.0, 100.0];
        let fc = 90.9e6;
        let dt = 0.1;
        let mut log = Vec::new();

        // 1. Seed a target
        let initial_pos = [-20_000.0, -10_000.0, 9500.0];
        let state = [
            initial_pos[0],
            initial_pos[1],
            initial_pos[2],
            150.0,
            160.0,
            0.0,
        ];
        let ekf = BistaticEkf::new(state, 1000.0, 10.0, 1.0);

        bank.targets.push(TrackedTarget {
            id: 77,
            ekf,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![state],
            classification: "AAL191 (B788)".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        });

        // 2. Feed updates with a peak matching the target's expected Doppler frequency (around 117.5 Hz)
        // Repeat 5 times to collect 5 fingerprint points
        for _ in 0..5 {
            bank.update(&tower_pos, fc, dt, &[(117.5, 18.5)], &mut log);
        }

        // Verify fingerprint history was populated
        let target = bank.targets.iter().find(|t| t.id == 77).unwrap();
        assert_eq!(target.fingerprint_history.len(), 5);

        let dp = &target.fingerprint_history[0];
        assert_eq!(dp.snr_db, 18.5);
        assert!(
            dp.rcs_db > 0.0,
            "Relative RCS should be calculated and greater than 0"
        );
        assert!(
            dp.bistatic_angle_deg > 0.0,
            "Bistatic angle should be calculated"
        );
        assert!((dp.doppler_hz - 117.5).abs() < 1.0);

        // 3. Mark the target as Terminated and age it beyond the 90 seconds timeout
        // (identified airliners are kept on screen for 90 seconds before roll-off)
        if let Some(t) = bank.targets.iter_mut().find(|t| t.id == 77) {
            t.state = TrackState::Terminated;
            t.terminated_at = Some(Instant::now() - std::time::Duration::from_secs(91));
        }

        // 4. Update once more to trigger pruning and check if the JSON is written to disk
        bank.update(&tower_pos, fc, dt, &[], &mut log);

        // Verify target 77 has been pruned from the bank
        assert!(bank.targets.iter().all(|t| t.id != 77));

        // Verify the file was created in the fingerprints directory
        let dir = std::fs::read_dir(bank.get_fingerprints_dir()).unwrap();
        let mut found = false;
        for entry in dir {
            if let Ok(entry) = entry {
                let path = entry.path();
                if path.is_file() {
                    let filename = path.file_name().unwrap().to_str().unwrap();
                    if filename.starts_with("fingerprint_") && filename.ends_with("_77.json") {
                        found = true;
                        let _ = std::fs::remove_file(path);
                    }
                }
            }
        }
        assert!(
            found,
            "Fingerprint file for target 77 should have been found and deleted"
        );
    }

    #[test]
    fn test_multitower_ekf_update() {
        let mut bank = TrackingBank::new();

        let initial_state = [10_000.0, -5000.0, 8000.0, 150.0, -100.0, 0.0];
        let ekf = BistaticEkf::new(initial_state, 1000.0, 10.0, 1.0);

        bank.targets.push(TrackedTarget {
            id: 99,
            ekf,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![initial_state],
            classification: "Propeller Aircraft".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: Vec::new(),
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        });

        // Let's perform a multi-tower update with WETA and WTOP
        let weta_pos = [-120_000.0, 50_000.0, 350.0];
        let wtop_pos = [20_000.0, 110_000.0, 380.0];
        let dt = 0.1;
        let mut log = Vec::new();

        // Calculate true Doppler shifts for both towers at initial state
        let lambda_weta = crate::sdr::C / 90.9e6;
        let lambda_wtop = crate::sdr::C / 103.5e6;

        let calc_doppler = |pos: &[f64; 3], state: &[f64; 6], lambda: f64| -> f64 {
            let dx = state[0] - pos[0];
            let dy = state[1] - pos[1];
            let dz = state[2] - pos[2];
            let r_t = (dx * dx + dy * dy + dz * dz).sqrt();
            let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2]).sqrt();
            let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
            let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
            -(dot_t + dot_r) / lambda
        };

        let dop_weta = calc_doppler(&weta_pos, &initial_state, lambda_weta);
        let dop_wtop = calc_doppler(&wtop_pos, &initial_state, lambda_wtop);

        let peaks_weta = [(dop_weta as f32, 15.0)];
        let peaks_wtop = [(dop_wtop as f32, 18.0)];
        let towers_data = vec![
            ("WETA-FM".to_string(), weta_pos, 90.9e6, &peaks_weta[..]),
            ("WTOP-FM".to_string(), wtop_pos, 103.5e6, &peaks_wtop[..]),
        ];

        let empty_samples = vec![];
        bank.update_multitower(&towers_data, dt, &empty_samples, &mut log);

        // Check that the target was updated and associated with both towers
        let target = bank.targets.iter().find(|t| t.id == 99).unwrap();
        assert_eq!(target.tracking_towers.len(), 2);
        assert!(target.tracking_towers.contains(&"WETA-FM".to_string()));
        assert!(target.tracking_towers.contains(&"WTOP-FM".to_string()));
        assert_eq!(target.hits, 11);
    }

    #[test]
    fn test_adelic_multilateration_init() {
        let mut bank = TrackingBank::new();

        let weta_pos = [-120_000.0, 50_000.0, 350.0];
        let wtop_pos = [20_000.0, 110_000.0, 380.0];
        let wiyy_pos = [80_000.0, -90_000.0, 420.0];
        let dt = 0.1;
        let mut log = Vec::new();

        // Define a target aircraft position/velocity we want to resolve
        let true_state = [-25_000.0, -15_000.0, 9000.0, 160.0, 170.0, 0.0];

        let lambda_weta = crate::sdr::C / 90.9e6;
        let lambda_wtop = crate::sdr::C / 103.5e6;
        let lambda_wiyy = crate::sdr::C / 97.9e6;

        let calc_doppler = |pos: &[f64; 3], state: &[f64; 6], lambda: f64| -> f64 {
            let dx = state[0] - pos[0];
            let dy = state[1] - pos[1];
            let dz = state[2] - pos[2];
            let r_t = (dx * dx + dy * dy + dz * dz).sqrt();
            let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2]).sqrt();
            let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
            let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
            -(dot_t + dot_r) / lambda
        };

        let dop_weta = calc_doppler(&weta_pos, &true_state, lambda_weta);
        let dop_wtop = calc_doppler(&wtop_pos, &true_state, lambda_wtop);
        let dop_wiyy = calc_doppler(&wiyy_pos, &true_state, lambda_wiyy);

        // We will seed a candidate plot for WETA frequency
        // We run updates 3 times to promote it.
        // On the 3rd update, when it promotes, it will run Adelic solver using the peaks from all towers
        for _ in 1..=3 {
            let peaks_weta = [(dop_weta as f32, 15.0)];
            let peaks_wtop = [(dop_wtop as f32, 18.0)];
            let peaks_wiyy = [(dop_wiyy as f32, 16.5)];
            let towers_data = vec![
                ("WETA-FM".to_string(), weta_pos, 90.9e6, &peaks_weta[..]),
                ("WTOP-FM".to_string(), wtop_pos, 103.5e6, &peaks_wtop[..]),
                ("WIYY-FM".to_string(), wiyy_pos, 97.9e6, &peaks_wiyy[..]),
            ];
            let empty_samples = vec![];
            bank.update_multitower(&towers_data, dt, &empty_samples, &mut log);
        }

        // Verify a new target was spawned and it is active/suspect
        assert!(
            !bank.targets.is_empty(),
            "A target should have been spawned"
        );
        let spawned = &bank.targets[0];

        // Let's verify that the Adelic solver significantly reduced the RSS compared to the default guess
        let direction_sign = -dop_weta.signum();
        let init_state = [
            -20_000.0 * direction_sign,
            -10_000.0 * direction_sign,
            9500.0,
            150.0 * direction_sign,
            160.0 * direction_sign,
            0.0,
        ];

        let get_rss = |state: &[f64; 6]| -> f64 {
            let mut rss = 0.0;
            // WETA
            {
                let dx = state[0] - weta_pos[0];
                let dy = state[1] - weta_pos[1];
                let dz = state[2] - weta_pos[2];
                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2])
                    .sqrt()
                    .max(1.0);
                let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
                let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
                let pred = -(dot_t + dot_r) / lambda_weta;
                let diff = dop_weta - pred;
                rss += diff * diff;
            }
            // WTOP
            {
                let dx = state[0] - wtop_pos[0];
                let dy = state[1] - wtop_pos[1];
                let dz = state[2] - wtop_pos[2];
                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2])
                    .sqrt()
                    .max(1.0);
                let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
                let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
                let pred = -(dot_t + dot_r) / lambda_wtop;
                let diff = dop_wtop - pred;
                rss += diff * diff;
            }
            // WIYY
            {
                let dx = state[0] - wiyy_pos[0];
                let dy = state[1] - wiyy_pos[1];
                let dz = state[2] - wiyy_pos[2];
                let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
                let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2])
                    .sqrt()
                    .max(1.0);
                let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
                let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
                let pred = -(dot_t + dot_r) / lambda_wiyy;
                let diff = dop_wiyy - pred;
                rss += diff * diff;
            }
            rss
        };

        let initial_rss = get_rss(&init_state);
        let resolved_rss = get_rss(&spawned.ekf.state);

        assert!(
            resolved_rss < 350.0,
            "Resolved RSS should be small, got {:.1}",
            resolved_rss
        );
        assert!(
            resolved_rss < initial_rss,
            "Resolved RSS ({:.1}) should be significantly better than initial guess RSS ({:.1})",
            resolved_rss,
            initial_rss
        );
    }

    #[test]
    fn test_mahalanobis_doppler() {
        let state = [10_000.0, -5000.0, 8000.0, 150.0, -100.0, 0.0];
        let mut cov = [[0.0; 6]; 6];
        for i in 0..6 {
            cov[i][i] = 100.0; // simple diagonal covariance
        }
        let r_variance = 4.0;
        let tower_pos = [-120_000.0, 50_000.0, 350.0];
        let fc = 90.9e6;

        // Predict Doppler frequency for state
        let (h, z_pred) = compute_measurement_jacobian(&state, &tower_pos, fc);

        // Let's test a measurement that is exactly equal to prediction
        let (d_m_sq_exact, _) =
            mahalanobis_distance_doppler(&state, &cov, r_variance, &tower_pos, fc, z_pred);
        assert!(
            (d_m_sq_exact).abs() < 1e-9,
            "Mahalanobis distance for exact match should be 0, got {}",
            d_m_sq_exact
        );

        // Let's test a measurement that has some offset
        let offset_meas = z_pred + 5.0;
        let (d_m_sq, _) =
            mahalanobis_distance_doppler(&state, &cov, r_variance, &tower_pos, fc, offset_meas);
        assert!(d_m_sq > 0.0, "Mahalanobis distance should be positive");

        // Manual calculation of H * P * H^T + R:
        let mut h_p = [0.0; 6];
        for c in 0..6 {
            let mut val = 0.0;
            for r in 0..6 {
                val += h[r] * cov[r][c];
            }
            h_p[c] = val;
        }
        let mut h_p_ht = 0.0;
        for i in 0..6 {
            h_p_ht += h_p[i] * h[i];
        }
        let s = h_p_ht + r_variance;
        let expected_d_m_sq = (5.0 * 5.0) / s;
        assert!(
            (d_m_sq - expected_d_m_sq).abs() < 1e-9,
            "Expected {}, got {}",
            expected_d_m_sq,
            d_m_sq
        );
    }

    #[test]
    fn test_duplicate_prevention() {
        let mut bank = TrackingBank::new();

        // 1. Setup target 1 (superior)
        let state1 = [100.0, 100.0, 100.0, 10.0, 10.0, 0.0];
        let mut ekf1 = BistaticEkf::new(state1, 100.0, 10.0, 1.0);
        // Make diagonal covariance entries 10.0
        for i in 0..6 {
            ekf1.cov[i][i] = 10.0;
        }

        let target1 = TrackedTarget {
            id: 1,
            ekf: ekf1,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![state1],
            classification: "Commercial Airliner".to_string(), // generic
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: vec![FingerprintDatapoint {
                time_elapsed_sec: 1.0,
                x_enu: 100.0,
                y_enu: 100.0,
                z_enu: 100.0,
                vx: 10.0,
                vy: 10.0,
                vz: 0.0,
                bistatic_angle_deg: 45.0,
                doppler_hz: 12.0,
                snr_db: 15.0,
                rcs_db: 20.0,
                jem_frequency_hz: None,
            }],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        };

        // 2. Setup target 2 (duplicate / inferior)
        let state2 = [101.0, 99.0, 100.0, 10.5, 9.5, 0.0];
        let mut ekf2 = BistaticEkf::new(state2, 200.0, 20.0, 1.0);
        // Make diagonal covariance entries 20.0 (higher trace -> inferior)
        for i in 0..6 {
            ekf2.cov[i][i] = 20.0;
        }

        let mut jem2 = crate::tracking::jem::JemAnalyzer::new();
        // Generate modulation samples to simulate JEM detection at 40 Hz
        let mut samples = Vec::new();
        for n in 0..3000 {
            let t = (n as f64) / 8000.0;
            let phase = 2.0 * std::f64::consts::PI * 120.0 * t
                + 0.8 * (2.0 * std::f64::consts::PI * 40.0 * t).sin();
            samples.push(Complex::from_polar(1.0, phase as f32));
        }
        jem2.process_block(120.0, &samples);

        let target2 = TrackedTarget {
            id: 2,
            ekf: ekf2,
            state: TrackState::Active,
            hits: 8,
            misses: 0,
            history: vec![[99.0, 99.0, 100.0, 10.0, 10.0, 0.0], state2],
            classification: "AAL191 (B788)".to_string(), // specific
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: vec![
                FingerprintDatapoint {
                    time_elapsed_sec: 0.5,
                    x_enu: 99.0,
                    y_enu: 99.0,
                    z_enu: 100.0,
                    vx: 10.0,
                    vy: 10.0,
                    vz: 0.0,
                    bistatic_angle_deg: 44.0,
                    doppler_hz: 11.5,
                    snr_db: 14.0,
                    rcs_db: 19.0,
                    jem_frequency_hz: None,
                },
                FingerprintDatapoint {
                    time_elapsed_sec: 1.5,
                    x_enu: 101.0,
                    y_enu: 99.0,
                    z_enu: 100.0,
                    vx: 10.5,
                    vy: 9.5,
                    vz: 0.0,
                    bistatic_angle_deg: 46.0,
                    doppler_hz: 12.5,
                    snr_db: 16.0,
                    rcs_db: 21.0,
                    jem_frequency_hz: Some(40.0),
                },
            ],
            jem: jem2,
            tracking_towers: Vec::new(),
        };

        bank.targets.push(target1);
        bank.targets.push(target2);

        // Run duplicate track prevention
        let mut log = Vec::new();
        TrackingBank::prevent_duplicate_tracks(&mut bank.targets, &mut log);

        // Verify Target 2 was pruned and Target 1 was kept
        assert_eq!(bank.targets.len(), 1, "One target should be remaining");
        let merged_target = &bank.targets[0];
        assert_eq!(
            merged_target.id, 1,
            "The superior target (ID 1) should be kept"
        );

        // Verify classification inheritance (should be target 2's specific classification)
        assert_eq!(
            merged_target.classification, "AAL191 (B788)",
            "Should inherit specific classification"
        );

        // Verify history inheritance (should take target 2's history because it's longer: length 2)
        assert_eq!(
            merged_target.history.len(),
            2,
            "Should inherit the longer history"
        );
        assert_eq!(
            merged_target.history[0],
            [99.0, 99.0, 100.0, 10.0, 10.0, 0.0]
        );

        // Verify fingerprint history merge (should have 3 datapoints, sorted chronologically: 0.5, 1.0, 1.5)
        assert_eq!(
            merged_target.fingerprint_history.len(),
            3,
            "Fingerprint histories should be merged"
        );
        assert_eq!(merged_target.fingerprint_history[0].time_elapsed_sec, 0.5);
        assert_eq!(merged_target.fingerprint_history[1].time_elapsed_sec, 1.0);
        assert_eq!(merged_target.fingerprint_history[2].time_elapsed_sec, 1.5);

        // Verify JEM analyzer inheritance (merged target should have JEM frequency detection)
        let sideband = merged_target.jem.get_sidebands_hz();
        assert!(sideband.is_some(), "Should inherit JEM sideband detection");
        assert!(
            (sideband.unwrap() - 40.0).abs() < 5.0,
            "JEM frequency should be close to 40 Hz"
        );

        // Verify EKF state and covariance fusion via Covariance Intersection
        // P_m trace should be minimized.
        let trace_p1: f64 = (0..6).map(|_| 10.0).sum(); // 60.0
        let trace_pm: f64 = (0..6).map(|i| merged_target.ekf.cov[i][i]).sum();
        assert!(
            trace_pm <= trace_p1,
            "Merged trace ({}) should be smaller or equal to superior trace ({})",
            trace_pm,
            trace_p1
        );
    }

    #[test]
    fn test_duplicate_prevention_euclidean_gate() {
        let mut bank = TrackingBank::new();

        let state1 = [20_000.0, 30_000.0, 5000.0, 150.0, 100.0, 0.0];
        let mut ekf1 = BistaticEkf::new(state1, 1000.0, 10.0, 1.0);
        for i in 0..6 {
            ekf1.cov[i][i] = 2_500_000_000.0;
        }

        let target1 = TrackedTarget {
            id: 1,
            ekf: ekf1,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![state1],
            classification: "Target 1".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        };

        let state2 = [-20_000.0, -30_000.0, 5000.0, -150.0, -100.0, 0.0];
        let mut ekf2 = BistaticEkf::new(state2, 1000.0, 10.0, 1.0);
        for i in 0..6 {
            ekf2.cov[i][i] = 2_500_000_000.0;
        }

        let target2 = TrackedTarget {
            id: 2,
            ekf: ekf2,
            state: TrackState::Active,
            hits: 10,
            misses: 0,
            history: vec![state2],
            classification: "Target 2".to_string(),
            terminated_at: None,
            coasting_frames: 0,
            start_time: Instant::now(),
            fingerprint_history: vec![],
            jem: crate::tracking::jem::JemAnalyzer::new(),
            tracking_towers: Vec::new(),
        };

        bank.targets.push(target1);
        bank.targets.push(target2);

        let mut log = Vec::new();
        TrackingBank::prevent_duplicate_tracks(&mut bank.targets, &mut log);

        assert_eq!(bank.targets.len(), 2, "Both targets should remain since they are 50 km apart");
    }

    #[test]
    fn test_disk_re_identification() {
        let mut bank = TrackingBank::new();
        let tower_pos = [0.0, 0.0, 0.0];
        let fc = 90.9e6;
        let dt_step = 0.1;
        let mut log = Vec::new();

        // 1. Write mockup target fingerprint to /Volumes/Storage/passiveradar/fingerprints
        let duration = 2.0;
        let target_id = 88;
        let classification = "AAL191 (B788)".to_string();

        let start_time_local = chrono::Local::now()
            - chrono::Duration::from_std(std::time::Duration::from_secs(12)).unwrap();
        let first_seen_str = start_time_local.format("%Y-%m-%d %H:%M:%S").to_string();

        let last_dp = FingerprintDatapoint {
            time_elapsed_sec: duration,
            x_enu: -20_000.0,
            y_enu: -10_000.0,
            z_enu: 9500.0,
            vx: 150.0,
            vy: 160.0,
            vz: 0.0,
            bistatic_angle_deg: 45.0,
            doppler_hz: 113.0,
            snr_db: 15.0,
            rcs_db: 20.0,
            jem_frequency_hz: Some(40.0),
        };

        let fp = TargetFingerprint {
            target_id,
            classification: classification.clone(),
            first_seen: first_seen_str,
            duration_sec: duration,
            datapoints: vec![last_dp],
        };

        let dir_path = bank.get_fingerprints_dir();
        let _ = std::fs::create_dir_all(&dir_path);
        let file_path = dir_path.join("fingerprint_AAL191__B788__88.json");
        let serialized = serde_json::to_string_pretty(&fp).unwrap();
        std::fs::write(&file_path, serialized).unwrap();

        // Load the disk fingerprints cache since we manually wrote the file
        bank.load_disk_fingerprints();

        // Ensure target 88 is NOT in memory
        assert!(bank.targets.iter().all(|t| t.id != target_id));

        // 2. Programmatically calculate the expected Doppler frequency at the elapsed time
        let dt_elapsed = get_disk_target_elapsed(&fp, &file_path);

        let ext_x = -20_000.0 + 150.0 * dt_elapsed;
        let ext_y = -10_000.0 + 160.0 * dt_elapsed;
        let ext_z = 9500.0;
        let ext_vx = 150.0;
        let ext_vy = 160.0;
        let ext_vz = 0.0;

        let lambda = crate::sdr::C / fc;
        let dx = ext_x - tower_pos[0];
        let dy = ext_y - tower_pos[1];
        let dz = ext_z - tower_pos[2];
        let r_t = (dx * dx + dy * dy + dz * dz).sqrt();
        let r_r = (ext_x * ext_x + ext_y * ext_y + ext_z * ext_z).sqrt();
        let dot_t = (ext_vx * dx + ext_vy * dy + ext_vz * dz) / r_t;
        let dot_r = (ext_vx * ext_x + ext_vy * ext_y + ext_vz * ext_z) / r_r;
        let expected_doppler = -(dot_t + dot_r) / lambda;

        // 3. Feed Doppler plots close to its extrapolated trajectory
        bank.update(
            &tower_pos,
            fc,
            dt_step,
            &[(expected_doppler as f32, 15.0)],
            &mut log,
        );
        bank.update(
            &tower_pos,
            fc,
            dt_step,
            &[(expected_doppler as f32, 15.0)],
            &mut log,
        );
        bank.update(
            &tower_pos,
            fc,
            dt_step,
            &[(expected_doppler as f32, 15.0)],
            &mut log,
        ); // 3rd update promotes candidate

        // 4. Verify that the target is successfully re-identified and loaded back
        assert!(
            bank.targets.iter().any(|t| t.id == target_id),
            "Target 88 should have been re-identified and loaded"
        );
        let target = bank.targets.iter().find(|t| t.id == target_id).unwrap();
        assert_eq!(
            target.state,
            TrackState::Suspect,
            "Target should be in Suspect state"
        );
        assert_eq!(
            target.classification, "AAL191 (B788)",
            "Target classification should match"
        );

        // Clean up the JSON file if it still exists (it should have been deleted by the re-id logic)
        let _ = std::fs::remove_file(file_path);
    }

    #[test]
    fn test_cech_obstruction_calculation() {
        let state_consistent: [f64; 6] = [-20_000.0, -10_000.0, 9500.0, 150.0, 160.0, 0.0];
        
        let tower_positions = vec![
            ("WETA-FM".to_string(), [-120_000.0, 50_000.0, 350.0], 90.9e6),
            ("WKYS-FM".to_string(), [-3900.0, 5009.0, 276.0], 93.9e6),
            ("WHUR-FM".to_string(), [1800.0, 3260.0, 250.0], 96.3e6),
        ];

        // Programmatically calculate the exact consistent Doppler shifts
        let mut peaks = Vec::new();
        for (_, tower_pos, fc) in &tower_positions {
            let x = state_consistent[0];
            let y = state_consistent[1];
            let z = state_consistent[2];
            let vx = state_consistent[3];
            let vy = state_consistent[4];
            let vz = state_consistent[5];

            let dx = x - tower_pos[0];
            let dy = y - tower_pos[1];
            let dz = z - tower_pos[2];
            let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
            let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

            let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
            let dot_r = (vx * x + vy * y + vz * z) / r_r;

            let lambda = crate::sdr::C / fc;
            let pred_doppler = -(dot_t + dot_r) / lambda;
            peaks.push(pred_doppler);
        }

        let peaks_weta = [(peaks[0] as f32, 15.0f32)];
        let peaks_wkys = [(peaks[1] as f32, 15.0f32)];
        let peaks_whur = [(peaks[2] as f32, 15.0f32)];

        let towers_data = vec![
            ("WETA-FM".to_string(), [-120_000.0, 50_000.0, 350.0], 90.9e6, &peaks_weta[..]),
            ("WKYS-FM".to_string(), [-3900.0, 5009.0, 276.0], 93.9e6, &peaks_wkys[..]),
            ("WHUR-FM".to_string(), [1800.0, 3260.0, 250.0], 96.3e6, &peaks_whur[..]),
        ];

        let obs_consistent = TrackingBank::compute_cech_obstruction(&state_consistent, &towers_data);
        assert!(
            obs_consistent < 1e-3,
            "Consistent target should have very low Čech obstruction, got: {}",
            obs_consistent
        );

        // Inconsistent mirror target
        let state_inconsistent: [f64; 6] = [20_000.0, 10_000.0, 9500.0, -150.0, -160.0, 0.0];
        let obs_inconsistent = TrackingBank::compute_cech_obstruction(&state_inconsistent, &towers_data);
        assert!(
            obs_inconsistent > 35.0,
            "Inconsistent mirror target should have high Čech obstruction, got: {}",
            obs_inconsistent
        );
    }

    #[test]
    fn test_tropical_wavelet_notching() {
        let mut magnitudes = vec![1.0; 256];
        // Introduce a sharp stationary spur at bin 50
        magnitudes[50] = 50.0;
        // Introduce a target peak at bin 150
        magnitudes[150] = 40.0;

        let active_dopplers = vec![((150 - 128) as f64 / 256.0) * 8000.0]; // mapped to bin 150
        
        let mut canceller = crate::dsp::tropical::TropicalWaveletCanceller::new(256);
        canceller.notch_stationary_spurs(
            &mut magnitudes,
            &active_dopplers,
            8000.0,
        );


        // Spur at bin 50 should be notched out (restored close to background envelope floor)
        assert!(
            magnitudes[50] < 5.0,
            "Spur at bin 50 should be notched out, got: {}",
            magnitudes[50]
        );

        // Target peak at bin 150 should remain untouched
        assert!(
            magnitudes[150] > 35.0,
            "Target peak at bin 150 should not be notched out, got: {}",
            magnitudes[150]
        );
    }
}

