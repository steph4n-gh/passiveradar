use num_complex::Complex;

#[derive(Debug, Clone)]
pub struct JemAnalyzer {
    phase: f32,
    buffer: Vec<Complex<f32>>,
    fft_size: usize,
    sample_rate: f64,
    sidebands_hz: Option<f64>,
    pub latest_fft_mag: Vec<f32>,
}

impl JemAnalyzer {
    pub fn new() -> Self {
        Self {
            phase: 0.0,
            buffer: Vec::with_capacity(2048),
            fft_size: 256,
            sample_rate: 1000.0, // After DDC and 8x decimation (8000 -> 1000 Hz)
            sidebands_hz: None,
            latest_fft_mag: vec![0.0; 256],
        }
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
        let phase_step = (2.0 * std::f64::consts::PI * target_doppler / 8000.0) as f32;
        let mut mixed = Vec::with_capacity(samples.len());

        for &sample in samples {
            let carrier = Complex::from_polar(1.0, -self.phase);
            mixed.push(sample * carrier);
            self.phase = (self.phase + phase_step) % (2.0 * std::f64::consts::PI as f32);
        }

        // 2. Simple FIR Low-Pass Filter (cutoff 150 Hz) and decimate by 8
        let taps = [
            0.0076, 0.0177, 0.0384, 0.0681, 0.1018, 0.1312, 0.1486, 0.1486, 0.1312, 0.1018, 0.0681,
            0.0384, 0.0177, 0.0076,
        ];

        let mut decimated = Vec::new();
        // Compute filter every 8 samples
        for i in (0..mixed.len()).step_by(8) {
            if i + taps.len() <= mixed.len() {
                let mut acc = Complex::new(0.0, 0.0);
                for (j, &tap) in taps.iter().enumerate() {
                    acc += mixed[i + j] * tap;
                }
                decimated.push(acc);
            }
        }

        self.buffer.extend_from_slice(&decimated);

        // 3. If buffer has enough samples (e.g. 256), run FFT and sideband detection
        if self.buffer.len() >= self.fft_size {
            let mut planner = rustfft::FftPlanner::new();
            let fft = planner.plan_fft_forward(self.fft_size);

            let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = self.buffer
                [0..self.fft_size]
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

            // Find symmetric sidebands around center DC (index 128)
            // We scan range from 10 Hz to 150 Hz: index offset from 128
            // 10 Hz corresponds to: 10 / (1000 / 256) = 2.56 bins
            // 150 Hz corresponds to: 150 / (1000 / 256) = 38.4 bins
            let mut best_peak_idx = None;
            let mut max_val = 0.0;

            let center = self.fft_size / 2; // 128
            let min_bin = 3;
            let max_bin = 38;

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

            if let Some(offset) = best_peak_idx {
                let freq = (offset as f64) * (self.sample_rate / self.fft_size as f64);
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

    pub fn get_sidebands_hz(&self) -> Option<f64> {
        self.sidebands_hz
    }

    pub fn set_sidebands_hz(&mut self, val: Option<f64>) {
        self.sidebands_hz = val;
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
}
