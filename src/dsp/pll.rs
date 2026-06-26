use num_complex::Complex;

/// A software digital Phase-Locked Loop (Costas Loop) designed to lock onto
/// a pilot tone (e.g. 19 kHz stereo pilot in FM or 309.44 kHz pilot in ATSC),
/// calculate PPM clock drift, and apply correction phase rotation to discipline the IQ stream.
pub struct VirtualTcxo {
    pilot_frequency_hz: f32,
    sample_rate: f32,
    phase: f32,
    frequency_offset_rad: f32,
    drift_phase: f32, // Track drift phase separately from nominal pilot phase
    // Loop filter coefficients
    alpha: f32,
    beta: f32,
    // Tracked PPM drift
    ppm_drift: f64,
}

impl VirtualTcxo {
    /// Create a new Virtual TCXO.
    pub fn new(pilot_frequency_hz: f32, sample_rate: f32) -> Self {
        // Design loop filter for Costas Loop (damping ratio = 0.707, loop bandwidth = 0.01)
        let damping_ratio = 0.707f32;
        let loop_bandwidth = 0.005f32; // Narrow bandwidth for stable lock
        
        let theta = loop_bandwidth / (damping_ratio + 0.25 / damping_ratio);
        let d = 1.0 + 2.0 * damping_ratio * theta + theta * theta;
        let alpha = (4.0 * damping_ratio * theta) / d;
        let beta = (4.0 * theta * theta) / d;

        Self {
            pilot_frequency_hz,
            sample_rate,
            phase: 0.0,
            frequency_offset_rad: 0.0,
            drift_phase: 0.0,
            alpha,
            beta,
            ppm_drift: 0.0,
        }
    }

    /// Process a block of samples. Lock onto the pilot tone, update loop filter,
    /// compute the PPM drift of the HackRF oscillator, and apply corrective phase rotation.
    pub fn discipline_block(&mut self, input: &[Complex<f32>], output: &mut [Complex<f32>], center_freq_hz: f32) {
        let n = input.len().min(output.len());
        
        // Expected phase step per sample at target pilot frequency
        let pilot_phase_step = 2.0 * std::f32::consts::PI * self.pilot_frequency_hz / self.sample_rate;

        for i in 0..n {
            let sample = input[i];

            // 1. Generate local pilot reference oscillator
            let ref_cos = self.phase.cos();
            let ref_sin = self.phase.sin();

            // 2. Mix input sample with local oscillator (Costas Loop phase detector)
            // Extract the in-phase and quadrature components of the pilot tone
            let i_mixed = sample.re * ref_cos + sample.im * ref_sin;
            let q_mixed = sample.im * ref_cos - sample.re * ref_sin;

            // Phase error detector: e = I * Q
            let phase_error = i_mixed * q_mixed;

            // 3. Loop Filter: Update phase and frequency accumulators
            let drift_update = self.frequency_offset_rad + self.alpha * phase_error;
            self.phase += drift_update;
            self.frequency_offset_rad += self.beta * phase_error;
            
            // Accumulate only drift components for correction
            self.drift_phase += drift_update;

            // Accumulate expected phase step
            self.phase += pilot_phase_step;
            
            // Wrap phase
            self.phase = (self.phase + std::f32::consts::PI) % (2.0 * std::f32::consts::PI) - std::f32::consts::PI;
            self.drift_phase = (self.drift_phase + std::f32::consts::PI) % (2.0 * std::f32::consts::PI) - std::f32::consts::PI;

            // 4. Calculate HackRF's hardware PPM drift
            // frequency_offset_rad is in radians/sample. Convert to Hz:
            let freq_offset_hz = (self.frequency_offset_rad * self.sample_rate) / (2.0 * std::f32::consts::PI);
            // ppm = (measured_offset / expected_frequency) * 1e6
            self.ppm_drift = (freq_offset_hz as f64 / self.pilot_frequency_hz as f64) * 1e6;

            // 5. Apply corrective phase rotation to the wideband input stream
            // Rotate the input sample in the opposite direction of the tracked drift phase
            // Scale correction phase from pilot frequency to RF center frequency
            let correction_phase = -self.drift_phase * (center_freq_hz / self.pilot_frequency_hz);
            output[i] = sample * Complex::from_polar(1.0, correction_phase);
        }
    }

    /// Get the currently tracked HackRF PPM clock drift.
    pub fn get_ppm_drift(&self) -> f64 {
        self.ppm_drift
    }
}
