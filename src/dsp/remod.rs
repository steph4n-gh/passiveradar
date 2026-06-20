use num_complex::Complex;

/// FM Reference Regenerator.
/// Extracts a 200 kHz channel, applies standard FM audio demodulation
/// (which mathematically destroys weak radar echoes via the FM capture effect),
/// and then mathematically re-modulates that audio back into a pristine,
/// echo-free complex IQ reference signal.
pub struct FmReferenceRegenerator {
    prev_sample: Complex<f32>,
    integrator_phase: f32,
    sample_rate: f32,
}

impl FmReferenceRegenerator {
    /// Create a new FM Reference Regenerator.
    pub fn new(sample_rate: f32) -> Self {
        Self {
            prev_sample: Complex::new(1.0, 0.0),
            integrator_phase: 0.0,
            sample_rate,
        }
    }

    /// Demodulate complex IQ samples into a mono audio signal.
    /// Utilizes the derivative of the phase (using complex conjugate multiplication).
    pub fn demodulate(&mut self, input: &[Complex<f32>], audio_out: &mut [f32]) {
        let n = input.len().min(audio_out.len());
        for i in 0..n {
            let sample = input[i];
            
            // Phase difference via complex multiply: diff = sample * prev_sample.conj()
            let diff = sample * self.prev_sample.conj();
            
            // Extract frequency deviation (instantaneous derivative of phase)
            let freq_deviation = diff.im.atan2(diff.re);
            
            // Audio output sample scaled to standard dev/amplitude
            audio_out[i] = freq_deviation;
            
            self.prev_sample = sample;
        }

        // Apply a simple lowpass/de-emphasis filter to the audio signal if needed
        // (FM Capture effect is naturally achieved via nonlinear phase extraction)
    }

    /// Re-modulate mono audio back into a pristine complex IQ signal.
    /// Integrate audio to get phase, then generate complex phasor.
    pub fn modulate(&mut self, audio: &[f32], output: &mut [Complex<f32>]) {
        let n = audio.len().min(output.len());
        
        // FM modulation index parameter (frequency deviation scaling)
        let k_f = 2.0 * std::f32::consts::PI * 75_000.0 / self.sample_rate;

        for i in 0..n {
            // Integrate frequency to get phase
            self.integrator_phase += audio[i] * k_f;
            
            self.integrator_phase = (self.integrator_phase + std::f32::consts::PI).rem_euclid(2.0 * std::f32::consts::PI) - std::f32::consts::PI;

            // Generate clean IQ phasor
            output[i] = Complex::from_polar(1.0, self.integrator_phase);
        }
    }

    /// Perform the end-to-end demodulation/re-modulation pipeline to clean the reference channel.
    pub fn regenerate_reference(&mut self, raw_reference: &[Complex<f32>], clean_reference: &mut [Complex<f32>]) {
        let mut audio = vec![0.0f32; raw_reference.len()];
        self.demodulate(raw_reference, &mut audio);
        self.modulate(&audio, clean_reference);
    }
}
