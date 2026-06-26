use num_complex::Complex;
use std::collections::VecDeque;
use std::sync::Arc;
use rustfft::Fft;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CicMode {
    Seismic,
    Rotary,
    Acoustic,
}

#[derive(Debug, Clone)]
pub struct AcousticBandpassFilter {
    lp_y: f32,
    lp_alpha: f32,
    hp_y: f32,
    hp_x_prev: f32,
    hp_alpha: f32,
}

impl AcousticBandpassFilter {
    pub fn new(fs: f32, fc_low: f32, fc_high: f32) -> Self {
        let hp_alpha = fs / (fs + 2.0 * std::f32::consts::PI * fc_low);
        let lp_alpha = (2.0 * std::f32::consts::PI * fc_high) / (fs + 2.0 * std::f32::consts::PI * fc_high);
        Self {
            lp_y: 0.0,
            lp_alpha,
            hp_y: 0.0,
            hp_x_prev: 0.0,
            hp_alpha,
        }
    }

    pub fn process(&mut self, sample: f32) -> f32 {
        if sample.is_nan() || sample.is_infinite() {
            return self.lp_y;
        }
        let hp_out = self.hp_alpha * (self.hp_y + sample - self.hp_x_prev);
        self.hp_x_prev = sample;
        self.hp_y = hp_out;

        let lp_out = self.lp_y + self.lp_alpha * (hp_out - self.lp_y);
        self.lp_y = lp_out;

        lp_out
    }
}

#[derive(Clone)]
pub struct JemAnalyzer {
    phase: f32,
    buffer: Vec<Complex<f32>>,
    fft_size: usize,
    pub sample_rate: f64,
    sidebands_hz: Option<f64>,
    pub latest_fft_mag: Vec<f32>,
    pub history: VecDeque<Vec<f32>>,
    pub cic_decimator: crate::dsp::cic::CicDecimator,
    pub cic_mode: CicMode,
    pub unwrapped_phase: Vec<f32>,
    pub cepstrum: Vec<f32>,
    pub respiration_rate: Option<f32>,
    pub payload_class: String,
    pub ghost_mic_enabled: bool,
    pub last_vz: f64,
    pub last_acoustic_phase: f32,
    pub acoustic_filter: Option<AcousticBandpassFilter>,
    pub pcm_output: Vec<i16>,
    pub fft_forward: Option<Arc<dyn Fft<f32>>>,
    pub fft_inverse: Option<Arc<dyn Fft<f32>>>,
}

impl std::fmt::Debug for JemAnalyzer {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("JemAnalyzer")
            .field("phase", &self.phase)
            .field("buffer", &self.buffer)
            .field("fft_size", &self.fft_size)
            .field("sample_rate", &self.sample_rate)
            .field("sidebands_hz", &self.sidebands_hz)
            .field("latest_fft_mag", &self.latest_fft_mag)
            .field("history", &self.history)
            .field("cic_decimator", &self.cic_decimator)
            .field("cic_mode", &self.cic_mode)
            .field("unwrapped_phase", &self.unwrapped_phase)
            .field("cepstrum", &self.cepstrum)
            .field("respiration_rate", &self.respiration_rate)
            .field("payload_class", &self.payload_class)
            .field("ghost_mic_enabled", &self.ghost_mic_enabled)
            .field("last_vz", &self.last_vz)
            .field("last_acoustic_phase", &self.last_acoustic_phase)
            .field("acoustic_filter", &self.acoustic_filter)
            .field("pcm_output", &self.pcm_output)
            .field("fft_forward", &self.fft_forward.is_some())
            .field("fft_inverse", &self.fft_inverse.is_some())
            .finish()
    }
}

impl std::panic::UnwindSafe for JemAnalyzer {}
impl std::panic::RefUnwindSafe for JemAnalyzer {}

impl JemAnalyzer {
    pub fn new() -> Self {
        let mode = CicMode::Rotary;
        let r = 8;
        let n = 3;
        let mut planner = rustfft::FftPlanner::new();
        let fft_forward = Some(planner.plan_fft_forward(256));
        let fft_inverse = Some(planner.plan_fft_inverse(256));
        Self {
            phase: 0.0,
            buffer: Vec::with_capacity(2048),
            fft_size: 256,
            sample_rate: 1000.0, // After DDC and 8x decimation (8000 -> 1000 Hz)
            sidebands_hz: None,
            latest_fft_mag: vec![0.0; 256],
            history: VecDeque::with_capacity(60),
            cic_decimator: crate::dsp::cic::CicDecimator::new(r, n),
            cic_mode: mode,
            unwrapped_phase: vec![0.0; 256],
            cepstrum: vec![0.0; 256],
            respiration_rate: None,
            payload_class: "UNLADEN".to_string(),
            ghost_mic_enabled: false,
            last_vz: 0.0,
            last_acoustic_phase: 0.0,
            acoustic_filter: None,
            pcm_output: Vec::new(),
            fft_forward,
            fft_inverse,
        }
    }

    pub fn set_cic_mode(&mut self, mode: CicMode) {
        let r = match mode {
            CicMode::Seismic => 80,
            CicMode::Rotary => 8,
            CicMode::Acoustic => 1,
        };
        let n = 3;
        self.cic_mode = mode;
        self.cic_decimator = crate::dsp::cic::CicDecimator::new(r, n);
        self.sample_rate = 8000.0 / r as f64;
        self.buffer.clear();
    }

    /// Compute instantaneous unwrapped phase and apply linear detrending using least-squares.
    pub fn unwrap_phase(&self, fft_input_samples: &[Complex<f32>]) -> Vec<f32> {
        let fft_size = fft_input_samples.len();
        let mut phases = Vec::with_capacity(fft_size);
        for &sample in fft_input_samples {
            phases.push(sample.im.atan2(sample.re));
        }

        let mut unwrapped = vec![0.0f32; fft_size];
        if fft_size > 0 {
            unwrapped[0] = phases[0];
            for i in 1..fft_size {
                let diff = phases[i] - phases[i - 1];
                let delta = diff.sin().atan2(diff.cos());
                unwrapped[i] = unwrapped[i - 1] + delta;
            }
        }

        // Linear detrending using least-squares regression
        if fft_size > 1 {
            let k_f = fft_size as f64;
            let mean_x = (fft_size - 1) as f64 / 2.0;
            let sum_y: f64 = unwrapped.iter().map(|&val| val as f64).sum();
            let mean_y = sum_y / k_f;

            let mut sum_xy = 0.0;
            for (i, &val) in unwrapped.iter().enumerate() {
                sum_xy += (i as f64) * (val as f64);
            }
            let ss_xy = sum_xy - mean_x * sum_y;
            let ss_xx = k_f * (k_f * k_f - 1.0) / 12.0;

            let m = ss_xy / ss_xx;
            let c = mean_y - m * mean_x;

            for (i, val) in unwrapped.iter_mut().enumerate() {
                let trend = m * (i as f64) + c;
                *val = (*val as f64 - trend) as f32;
            }
        }

        unwrapped
    }

    pub fn compute_cepstrum(&self, fft_input_samples: &[Complex<f32>]) -> Vec<f32> {
        let fft_size = fft_input_samples.len();
        if fft_size == 0 {
            return Vec::new();
        }

        // 1. Forward FFT
        let fft_forward = self.fft_forward.clone()
            .filter(|f| f.len() == fft_size)
            .unwrap_or_else(|| {
                let mut planner = rustfft::FftPlanner::new();
                planner.plan_fft_forward(fft_size)
            });
        let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = fft_input_samples
            .iter()
            .map(|&c| rustfft::num_complex::Complex::new(c.re, c.im))
            .collect();
        let mut scratch_forward = vec![rustfft::num_complex::Complex::new(0.0, 0.0); fft_forward.get_inplace_scratch_len()];
        fft_forward.process_with_scratch(&mut fft_input, &mut scratch_forward);

        // 2. ln(norm() + 1e-6)
        let mut log_mag: Vec<rustfft::num_complex::Complex<f32>> = fft_input
            .iter()
            .map(|c| rustfft::num_complex::Complex::new((c.norm() + 1e-6).ln(), 0.0))
            .collect();

        // 3. IFFT (Inverse FFT)
        let fft_inverse = self.fft_inverse.clone()
            .filter(|f| f.len() == fft_size)
            .unwrap_or_else(|| {
                let mut planner = rustfft::FftPlanner::new();
                planner.plan_fft_inverse(fft_size)
            });
        let mut scratch_inverse = vec![rustfft::num_complex::Complex::new(0.0, 0.0); fft_inverse.get_inplace_scratch_len()];
        fft_inverse.process_with_scratch(&mut log_mag, &mut scratch_inverse);

        // Output complex scaled by 1/fft_size converted to Vec<f32>
        log_mag
            .iter()
            .map(|c| c.norm() / fft_size as f32)
            .collect()
    }

    /// Process a block of baseband samples (sampled at 8 kHz)
    /// `target_doppler`: estimated Doppler frequency from EKF
    /// `samples`: baseband samples associated with the target
    pub fn process_block(&mut self, target_doppler: f64, samples: &[Complex<f32>]) {
        if samples.is_empty() {
            return;
        }

        // 1. Shift target to DC: multiply by e^{-j 2\pi f_d t}
        // Since sample rate is 8000 Hz, phase step is 2 * pi * target_doppler / 8000.0
        let clean_doppler = if target_doppler.is_nan() || target_doppler.is_infinite() {
            0.0
        } else {
            target_doppler
        };
        let phase_step = (2.0 * std::f64::consts::PI * clean_doppler / 8000.0) as f32;
        let mut mixed = Vec::with_capacity(samples.len());

        let (sin_step, cos_step) = phase_step.sin_cos();
        let rotation = Complex::new(cos_step, -sin_step);
        let mut carrier = Complex::from_polar(1.0f32, -self.phase);

        for (i, &sample) in samples.iter().enumerate() {
            mixed.push(sample * carrier);
            carrier = carrier * rotation;

            // Renormalize every 1024 samples to prevent magnitude drift
            if (i & 0x3FF) == 0x3FF {
                let norm = carrier.norm();
                if norm > 0.0 {
                    carrier = carrier / norm;
                }
            }
        }

        // Extract phase directly from exact complex state of carrier to prevent accumulator drift
        let mut next_phase = -carrier.im.atan2(carrier.re);
        if next_phase < 0.0 {
            next_phase += 2.0 * std::f32::consts::PI as f32;
        }
        self.phase = next_phase;

        // Extract acoustic PCM if mode is Acoustic
        if self.cic_mode == CicMode::Acoustic {
            let filter = self.acoustic_filter.get_or_insert_with(|| AcousticBandpassFilter::new(8000.0, 300.0, 3400.0));
            self.pcm_output.clear();
            for &c in &mixed {
                let raw_phase = c.im.atan2(c.re);
                let diff = raw_phase - self.last_acoustic_phase;
                let delta = diff.sin().atan2(diff.cos());
                self.last_acoustic_phase = raw_phase;

                // Numerical derivative: s(n) = delta / dt = delta * 8000.0
                let s_n = delta * 8000.0;

                // Bandpass filter
                let filtered = filter.process(s_n);

                // Convert to 16-bit PCM
                let pcm_val = filtered.clamp(-32768.0, 32767.0) as i16;
                self.pcm_output.push(pcm_val);
            }
        }

        // 2. CIC Decimation Bank
        let mut decimated = Vec::new();
        self.cic_decimator.process_block(&mixed, &mut decimated);
        self.buffer.extend_from_slice(&decimated);

        // 3. If buffer has enough samples (e.g. 256), run FFT, Phase Unwrapping, Cepstral, and sideband detection
        while self.buffer.len() >= self.fft_size {
            let window_samples = &self.buffer[0..self.fft_size];

            // Perform Phase Unwrapping and Cepstral Analysis
            self.unwrapped_phase = self.unwrap_phase(window_samples);
            self.cepstrum = self.compute_cepstrum(window_samples);

            // Extract respiration rate if mode is Seismic
            if self.cic_mode == CicMode::Seismic && !self.unwrapped_phase.is_empty() {
                let fs = self.sample_rate as f32;
                let mut best_freq = 0.1f32;
                let mut max_power = -1.0f32;

                let mut freq = 0.1f32;
                while freq <= 0.5f32 {
                    let mut sum_re = 0.0f32;
                    let mut sum_im = 0.0f32;
                    for (k, &phase_val) in self.unwrapped_phase.iter().enumerate() {
                        let t = (k as f32) / fs;
                        let angle = 2.0 * std::f32::consts::PI * freq * t;
                        sum_re += phase_val * angle.cos();
                        sum_im += phase_val * angle.sin();
                    }
                    let power = sum_re * sum_re + sum_im * sum_im;
                    if power > max_power {
                        max_power = power;
                        best_freq = freq;
                    }
                    freq += 0.002f32;
                }
                self.respiration_rate = Some(best_freq);
            } else if self.cic_mode != CicMode::Seismic {
                self.respiration_rate = None;
            }

            let fft = self.fft_forward.clone().unwrap_or_else(|| {
                let mut planner = rustfft::FftPlanner::new();
                planner.plan_fft_forward(self.fft_size)
            });

            let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = window_samples
                .iter()
                .map(|&c| rustfft::num_complex::Complex::new(c.re, c.im))
                .collect();

            // Run FFT
            let mut scratch =
                vec![rustfft::num_complex::Complex::new(0.0, 0.0); fft.get_inplace_scratch_len()];
            fft.process_with_scratch(&mut fft_input, &mut scratch);

            // Compute magnitude and shift center DC to fft_size/2
            let mut mag = vec![0.0f32; self.fft_size];
            for i in 0..self.fft_size {
                let shift_idx = (i + self.fft_size / 2) % self.fft_size;
                mag[shift_idx] = fft_input[i].norm();
            }
            self.latest_fft_mag = mag.clone();
            self.history.push_back(mag.clone());
            if self.history.len() > 60 {
                self.history.pop_front();
            }

            // Find symmetric sidebands around center DC (index 128)
            let bin_width = self.sample_rate / self.fft_size as f64;
            let (f_min, f_max) = match self.cic_mode {
                CicMode::Seismic => (0.01, 5.0),
                CicMode::Rotary => (10.0, 250.0),
                CicMode::Acoustic => (300.0, 4000.0),
            };
            let mut min_bin = (f_min / bin_width).round() as usize;
            let mut max_bin = (f_max / bin_width).round() as usize;

            if min_bin < 1 {
                min_bin = 1;
            }
            let center = self.fft_size / 2;
            if max_bin >= center - 1 {
                max_bin = center - 2;
            }

            let mut best_peak_idx = None;
            let mut max_val = 0.0;

            if min_bin <= max_bin {
                for offset in min_bin..=max_bin {
                    let left_val = mag[center - offset];
                    let right_val = mag[center + offset];
                    let val = left_val + right_val;

                    // Local maximum check
                    if val > max_val
                        && left_val > mag[center - offset - 1]
                        && left_val > mag[center - offset + 1]
                        && right_val > mag[center + offset - 1]
                        && right_val > mag[center + offset + 1]
                    {
                        max_val = val;
                        best_peak_idx = Some(offset);
                    }
                }
            }

            if let Some(offset) = best_peak_idx {
                let freq = (offset as f64) * bin_width;
                // Threshold peak SNR: must be above noise floor
                let avg_noise: f32 = mag.iter().sum::<f32>() / self.fft_size as f32;
                if mag[center - offset] > avg_noise * 3.0 && mag[center + offset] > avg_noise * 3.0
                {
                    self.sidebands_hz = Some(freq);
                }
            }

            // Drain buffer (50% overlap sliding window)
            self.buffer.drain(0..self.fft_size / 2);
        }
    }

    pub fn get_blade_pass_frequency(&self) -> f64 {
        if self.cic_mode != CicMode::Rotary {
            return 0.0;
        }
        if let Some(sb) = self.sidebands_hz {
            if sb > 0.0 {
                return sb;
            }
        }
        // Spectral search in cepstrum
        if self.cepstrum.len() > 100 {
            let mut max_val = -1.0;
            let mut peak_idx = 0;
            for idx in 4..=100 {
                if self.cepstrum[idx] > max_val {
                    max_val = self.cepstrum[idx];
                    peak_idx = idx;
                }
            }
            if peak_idx > 0 {
                return self.sample_rate / peak_idx as f64;
            }
        }
        0.0
    }

    pub fn update_heuristics(&mut self, vz: f64, dt: f64) {
        let a_z = if dt > 0.0 {
            (vz - self.last_vz) / dt
        } else {
            0.0
        };
        self.last_vz = vz;

        let f_bpf = self.get_blade_pass_frequency();

        let denom = (9.81 + a_z).max(0.1);
        let k = 0.005886;
        let m_empty = 1.0;
        let m_payload = k * f_bpf * f_bpf / denom - m_empty;

        self.payload_class = if m_payload < 0.2 {
            "UNLADEN".to_string()
        } else if m_payload < 1.0 {
            "LIGHT (CAMERA/GIMBAL)".to_string()
        } else if m_payload < 3.0 {
            "HEAVY CARGO".to_string()
        } else {
            "HIGH-RISK PAYLOAD THREAT".to_string()
        };
    }

    pub fn get_sidebands_hz(&self) -> Option<f64> {
        self.sidebands_hz
    }

    pub fn set_sidebands_hz(&mut self, val: Option<f64>) {
        self.sidebands_hz = val;
    }

    pub fn set_fft_size(&mut self, size: usize) {
        if size == 256 || size == 512 || size == 1024 || size == 2048 || size == 4096 || size == 8192 {
            if self.fft_size != size {
                self.fft_size = size;
                self.latest_fft_mag.resize(size, 0.0);
                self.unwrapped_phase.resize(size, 0.0);
                self.cepstrum.resize(size, 0.0);
                
                let mut planner = rustfft::FftPlanner::new();
                self.fft_forward = Some(planner.plan_fft_forward(size));
                self.fft_inverse = Some(planner.plan_fft_inverse(size));
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_jem_sideband_detection() {
        let mut jem = JemAnalyzer::new();

        // Target Doppler of 120 Hz, modulation sidebands at 40 Hz
        let target_doppler = 120.0;
        let fm = 40.0;
        let beta = 0.8f64; // modulation index
        let fs = 8000.0;

        // Generate 3000 samples to ensure the decimation by 8 results in > 256 samples
        let mut samples = Vec::new();
        for n in 0..3000 {
            let t = (n as f64) / fs;

            // Phase modulated signal: s(t) = exp( j * (2*pi*fc*t + beta * sin(2*pi*fm*t)) )
            let phase = 2.0 * std::f64::consts::PI * target_doppler * t
                + beta * (2.0 * std::f64::consts::PI * fm * t).sin();
            let mut sample = Complex::from_polar(1.0, phase as f32);

            // Add a tiny bit of noise
            let noise_re = ((n * 17) % 100) as f32 / 5000.0;
            let noise_im = ((n * 31) % 100) as f32 / 5000.0;
            sample += Complex::new(noise_re, noise_im);

            samples.push(sample);
        }

        jem.process_block(target_doppler, &samples);

        let detected = jem.get_sidebands_hz();
        assert!(detected.is_some(), "Should detect JEM sidebands");
        let freq = detected.unwrap();
        // Since bin size is 1000 / 256 = 3.9 Hz, we check if detected is within 5.0 Hz of 40 Hz
        assert!(
            (freq - fm).abs() < 5.0,
            "Detected frequency {} should be close to 40 Hz",
            freq
        );
    }

    #[test]
    fn test_jem_nan_doppler_resistance() {
        let mut jem = JemAnalyzer::new();
        let samples = vec![Complex::new(0.5, -0.5); 512];

        // Feed NaN target doppler to the analyzer.
        // It must handle this gracefully (e.g. ignore or substitute 0.0)
        // rather than contaminating the whole buffer and latest_fft_mag with NaNs.
        jem.process_block(std::f64::NAN, &samples);

        // Assert that the magnitude output does not contain any NaNs.
        for &val in &jem.latest_fft_mag {
            assert!(!val.is_nan(), "JEM magnitude spectrum contains NaN values!");
        }
    }

    #[test]
    fn test_unwrap_phase_sine_wave() {
        let jem = JemAnalyzer::new();
        let fm = 5.0;
        let fs = 100.0;
        let mut samples = Vec::new();
        let mut expected = Vec::new();
        for n in 0..100 {
            let t = n as f32 / fs;
            let phase = 5.0 * (2.0 * std::f32::consts::PI * fm * t).sin();
            samples.push(Complex::from_polar(1.0, phase));
            expected.push(phase);
        }

        let unwrapped = jem.unwrap_phase(&samples);
        let mut expected_detrended = expected.clone();
        if expected_detrended.len() > 1 {
            let k_f = expected_detrended.len() as f64;
            let mean_x = (expected_detrended.len() - 1) as f64 / 2.0;
            let sum_y: f64 = expected_detrended.iter().map(|&val| val as f64).sum();
            let mean_y = sum_y / k_f;

            let mut sum_xy = 0.0;
            for (i, &val) in expected_detrended.iter().enumerate() {
                sum_xy += (i as f64) * (val as f64);
            }
            let ss_xy = sum_xy - mean_x * sum_y;
            let ss_xx = k_f * (k_f * k_f - 1.0) / 12.0;

            let m = ss_xy / ss_xx;
            let c = mean_y - m * mean_x;

            for (i, val) in expected_detrended.iter_mut().enumerate() {
                let trend = m * (i as f64) + c;
                *val = (*val as f64 - trend) as f32;
            }
        }

        for i in 0..100 {
            let diff = (unwrapped[i] - expected_detrended[i]).abs();
            assert!(diff < 1e-3, "Phase mismatch at {}: expected {}, got {}", i, expected_detrended[i], unwrapped[i]);
        }
    }

    #[test]
    fn test_cepstrum_fundamental_detection() {
        let jem = JemAnalyzer::new();
        let fs = 1000.0f32;
        let f0 = 50.0f32; // Period T0 = fs / f0 = 20 samples
        let n_samples = 256;
        let mut samples = Vec::new();
        for n in 0..n_samples {
            // Harmonic signal: sum of f0, 2*f0, 3*f0
            let phase1 = 2.0 * std::f32::consts::PI * f0 * (n as f32) / fs;
            let phase2 = 2.0 * std::f32::consts::PI * (2.0 * f0) * (n as f32) / fs;
            let phase3 = 2.0 * std::f32::consts::PI * (3.0 * f0) * (n as f32) / fs;
            let sample = Complex::from_polar(1.0, phase1)
                + Complex::from_polar(0.6, phase2)
                + Complex::from_polar(0.3, phase3);
            samples.push(sample);
        }

        let cepstrum = jem.compute_cepstrum(&samples);
        assert_eq!(cepstrum.len(), n_samples);
        // Find the peak in the cepstrum in the range [10, 30]
        let mut max_val = 0.0;
        let mut peak_idx = 0;
        for idx in 10..30 {
            if cepstrum[idx] > max_val {
                max_val = cepstrum[idx];
                peak_idx = idx;
            }
        }

        assert!((peak_idx as i32 - 20).abs() <= 1, "Expected cepstrum peak close to 20, got {}", peak_idx);
    }

    #[test]
    fn test_cic_decimation_bounds() {
        // Test Acoustic mode (R=1, bypass)
        let mut dec_acoustic = crate::dsp::cic::CicDecimator::new(1, 3);
        let input = vec![Complex::new(1.0, 2.0); 1600];
        let mut output = Vec::new();
        dec_acoustic.process_block(&input, &mut output);
        assert_eq!(output.len(), 1600);
        for i in 0..1600 {
            assert_eq!(output[i], input[i]);
        }

        // Test Rotary mode (R=8)
        let mut dec_rotary = crate::dsp::cic::CicDecimator::new(8, 3);
        let mut output_rotary = Vec::new();
        dec_rotary.process_block(&input, &mut output_rotary);
        assert_eq!(output_rotary.len(), 200); // 1600 / 8 = 200
        let last_sample = output_rotary.last().unwrap();
        assert!((last_sample.re - 1.0).abs() < 0.05);
        assert!((last_sample.im - 2.0).abs() < 0.05);

        // Test Seismic mode (R=80)
        let mut dec_seismic = crate::dsp::cic::CicDecimator::new(80, 3);
        let mut output_seismic = Vec::new();
        dec_seismic.process_block(&input, &mut output_seismic);
        assert_eq!(output_seismic.len(), 20); // 1600 / 80 = 20
        let last_sample = output_seismic.last().unwrap();
        assert!((last_sample.re - 1.0).abs() < 0.05);
        assert!((last_sample.im - 2.0).abs() < 0.05);
    }

    #[test]
    fn test_respiration_rate_seismic() {
        let mut jem = JemAnalyzer::new();
        jem.set_cic_mode(CicMode::Seismic);
        jem.set_fft_size(2048);

        let fm = 0.25f64;
        let fs = 8000.0;
        let mut samples = Vec::new();
        for n in 0..180000 {
            let t = (n as f64) / fs;
            let phase = 0.5 * (2.0 * std::f64::consts::PI * fm * t).sin();
            samples.push(Complex::from_polar(1.0f32, phase as f32));
        }

        jem.process_block(0.0, &samples);

        assert!(jem.respiration_rate.is_some());
        let rate = jem.respiration_rate.unwrap();
        assert!((rate - 0.25).abs() < 0.05, "Expected close to 0.25, got {}", rate);
    }

    #[test]
    fn test_ghost_mic_pcm_demodulation() {
        let mut jem = JemAnalyzer::new();
        jem.set_cic_mode(CicMode::Acoustic);

        let fa = 1000.0;
        let fs = 8000.0;
        let mut samples = Vec::new();
        for n in 0..500 {
            let t = (n as f64) / fs;
            let phase = 0.5 * (2.0 * std::f64::consts::PI * fa * t).sin();
            samples.push(Complex::from_polar(1.0f32, phase as f32));
        }

        jem.process_block(0.0, &samples);

        assert_eq!(jem.pcm_output.len(), 500);
        let energy: f64 = jem.pcm_output.iter().map(|&x| (x as f64) * (x as f64)).sum();
        assert!(energy > 0.0);
    }

    #[test]
    fn test_drone_payload_heuristics_mapping() {
        let mut jem = JemAnalyzer::new();
        jem.set_cic_mode(CicMode::Rotary);

        // Test Category: UNLADEN
        jem.set_sidebands_hz(Some(30.0));
        jem.update_heuristics(0.0, 0.1);
        assert_eq!(jem.payload_class, "UNLADEN");

        // Test Category: LIGHT (CAMERA/GIMBAL)
        jem.set_sidebands_hz(Some(50.0));
        jem.update_heuristics(0.0, 0.1);
        assert_eq!(jem.payload_class, "LIGHT (CAMERA/GIMBAL)");

        // Test Category: HEAVY CARGO
        jem.set_sidebands_hz(Some(80.0));
        jem.update_heuristics(0.0, 0.1);
        assert_eq!(jem.payload_class, "HEAVY CARGO");

        // Test Category: HIGH-RISK PAYLOAD THREAT
        jem.set_sidebands_hz(Some(100.0));
        jem.update_heuristics(0.0, 0.1);
        assert_eq!(jem.payload_class, "HIGH-RISK PAYLOAD THREAT");

        // Test acceleration effect
        jem.set_sidebands_hz(Some(80.0));
        jem.last_vz = 0.0;
        jem.update_heuristics(0.2, 0.1); // a_z = 2.0
        assert_eq!(jem.payload_class, "HEAVY CARGO");
    }
}
