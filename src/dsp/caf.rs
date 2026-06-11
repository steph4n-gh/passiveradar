use num_complex::Complex;
use rustfft::{num_complex::Complex as FftComplex, FftPlanner};
use rayon::prelude::*;
use crate::dsp::fft::{FftBackend, DISABLE_GPU};
use std::sync::atomic::Ordering;

/// Apply a fractional delay using a cubic Lagrange interpolator (Farrow structure).
/// Total delay = int_delay + frac_delay, where frac_delay is in [0, 1).
#[inline(always)]
pub fn interpolate_farrow(samples: &[Complex<f32>], index: usize, frac_delay: f32) -> Complex<f32> {
    if index < 2 || index + 1 >= samples.len() {
        // Return boundary sample or zero if out of bounds
        if index < samples.len() {
            return samples[index];
        } else {
            return Complex::new(0.0, 0.0);
        }
    }

    let d = frac_delay;
    // Lagrange coefficients
    let a_neg1 = -d * (d - 1.0) * (d - 2.0) / 6.0;
    let a_0 = (d + 1.0) * (d - 1.0) * (d - 2.0) / 2.0;
    let a_1 = -(d + 1.0) * d * (d - 2.0) / 2.0;
    let a_2 = (d + 1.0) * d * (d - 1.0) / 6.0;

    // Apply coefficients to samples[index + 1], samples[index], samples[index - 1], samples[index - 2]
    samples[index + 1] * a_neg1
        + samples[index] * a_0
        + samples[index - 1] * a_1
        + samples[index - 2] * a_2
}

/// Generate a delayed version of a sample block using the Farrow interpolator.
pub fn delay_signal_farrow(
    samples: &[Complex<f32>],
    int_delay: usize,
    frac_delay: f32,
    output_len: usize,
) -> Vec<Complex<f32>> {
    let mut output = vec![Complex::new(0.0, 0.0); output_len];
    for n in 0..output_len {
        let ref_idx = n + int_delay;
        if ref_idx < samples.len() {
            output[n] = interpolate_farrow(samples, ref_idx, frac_delay);
        }
    }
    output
}

pub struct CafEngine {
    fft_size: usize,
    backend: FftBackend,
    scratch: Vec<FftComplex<f32>>,
}

impl CafEngine {
    pub fn new(fft_size: usize) -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(fft_size);
        let scratch = vec![FftComplex::new(0.0, 0.0); fft.get_inplace_scratch_len()];

        #[allow(unused_mut)]
        let mut backend = FftBackend::Cpu(fft);

        #[cfg(feature = "gpu-fft")]
        {
            if !DISABLE_GPU.load(Ordering::SeqCst) && wgsl_fft::GpuFft::is_gpu_available() {
                match wgsl_fft::GpuFft::new() {
                    Ok(gpu) => {
                        backend = FftBackend::Gpu(gpu);
                    },
                    Err(_) => {}
                }
            }
        }

        Self {
            fft_size,
            backend,
            scratch,
        }
    }

    /// Computes the 2D Cross-Ambiguity Function (CAF) waterfall.
    /// Returns a 2D matrix of shape [max_delay, fft_size] containing the magnitude-squared correlation.
    pub fn compute(
        &mut self,
        reference: &[Complex<f32>],
        surveillance: &[Complex<f32>],
        max_delay: usize,
    ) -> Vec<Vec<f32>> {
        let mut caf_result = vec![vec![0.0f32; self.fft_size]; max_delay];
        let n_samples = reference.len().min(surveillance.len());

        if n_samples < self.fft_size {
            return caf_result; // Not enough samples
        }

        let fft_size = self.fft_size;

        // Generate the time-domain cross correlation sequences
        let mut inputs = vec![vec![FftComplex::new(0.0, 0.0); fft_size]; max_delay];
        inputs.par_iter_mut().enumerate().for_each(|(delay, row)| {
            for n in 0..fft_size {
                let ref_idx = if n >= delay { n - delay } else { 0 };
                let ref_sample = reference[ref_idx];
                let surv_sample = surveillance[n];
                let prod = Complex::new(
                    surv_sample.re * ref_sample.re + surv_sample.im * ref_sample.im,
                    surv_sample.im * ref_sample.re - surv_sample.re * ref_sample.im,
                );
                row[n] = FftComplex::new(prod.re, prod.im);
            }
        });

        let fft_results = match &self.backend {
            FftBackend::Cpu(fft) => {
                let mut outputs = inputs;
                // Rayon par_iter_mut for CPU fallback
                let scratch_len = fft.get_inplace_scratch_len();
                outputs.par_iter_mut().for_each_init(
                    || vec![FftComplex::new(0.0, 0.0); scratch_len],
                    |scratch, row| {
                        fft.process_with_scratch(row, scratch);
                    }
                );
                outputs
            }
            #[cfg(feature = "gpu-fft")]
            FftBackend::Gpu(gpu) => {
                match gpu.fft(&inputs) {
                    Ok(res) => res,
                    Err(e) => {
                        eprintln!("GPU CAF FFT failed at runtime: {}. Falling back to CPU.", e);
                        let mut planner = FftPlanner::new();
                        let cpu_fft = planner.plan_fft_forward(fft_size);
                        let mut outputs = inputs;
                        let scratch_len = cpu_fft.get_inplace_scratch_len();
                        outputs.par_iter_mut().for_each_init(
                            || vec![FftComplex::new(0.0, 0.0); scratch_len],
                            |scratch, row| {
                                cpu_fft.process_with_scratch(row, scratch);
                            }
                        );
                        outputs
                    }
                }
            }
        };

        // Compute magnitude squared
        caf_result.par_iter_mut().zip(fft_results.par_iter()).for_each(|(row, out_row)| {
            for f in 0..fft_size {
                let shift_idx = (f + fft_size / 2) % fft_size;
                row[shift_idx] = out_row[f].norm_sqr();
            }
        });

        caf_result
    }

    /// Computes a single slice of the CAF at a fractional delay.
    pub fn compute_fractional_slice(
        &mut self,
        reference: &[Complex<f32>],
        surveillance: &[Complex<f32>],
        int_delay: usize,
        frac_delay: f32,
    ) -> Vec<f32> {
        let mut corr_product = vec![FftComplex::new(0.0, 0.0); self.fft_size];
        let n_samples = reference.len().min(surveillance.len());

        if n_samples < self.fft_size {
            return vec![0.0; self.fft_size];
        }

        // Generate the fractionally delayed reference signal
        let delayed_ref = delay_signal_farrow(reference, int_delay, frac_delay, self.fft_size);

        for n in 0..self.fft_size {
            let ref_sample = delayed_ref[n];
            let surv_sample = surveillance[n];
            let prod = Complex::new(
                surv_sample.re * ref_sample.re + surv_sample.im * ref_sample.im,
                surv_sample.im * ref_sample.re - surv_sample.re * ref_sample.im,
            );
            corr_product[n] = FftComplex::new(prod.re, prod.im);
        }

        let mut fft_output = corr_product.clone();
        match &self.backend {
            FftBackend::Cpu(fft) => {
                fft.process_with_scratch(&mut fft_output, &mut self.scratch);
            }
            #[cfg(feature = "gpu-fft")]
            FftBackend::Gpu(gpu) => {
                match gpu.fft(&[corr_product]) {
                    Ok(res) => {
                        fft_output.copy_from_slice(&res[0]);
                    }
                    Err(_) => {
                        let mut planner = FftPlanner::new();
                        let cpu_fft = planner.plan_fft_forward(self.fft_size);
                        cpu_fft.process_with_scratch(&mut fft_output, &mut self.scratch);
                    }
                }
            }
        }

        let mut result = vec![0.0; self.fft_size];
        for f in 0..self.fft_size {
            let shift_idx = (f + self.fft_size / 2) % self.fft_size;
            result[shift_idx] = fft_output[f].norm();
        }

        result
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_farrow_interpolation() {
        // Create a sine wave signal: sin(2*pi*f*t)
        let rate = 8000.0;
        let freq = 100.0;
        let mut samples = Vec::new();
        for n in 0..100 {
            let t = n as f32 / rate;
            let val = (2.0 * std::f32::consts::PI * freq * t).sin();
            samples.push(Complex::new(val, 0.0));
        }

        // Test at index 10 with fractional delay 0.5 (should equal sample at index 9.5)
        let frac = 0.5;
        let val_interp = interpolate_farrow(&samples, 10, frac);

        let t_ideal = 9.5 / rate;
        let val_ideal = (2.0 * std::f32::consts::PI * freq * t_ideal).sin();

        // Error should be extremely small for cubic Lagrange interpolation on a low freq sine wave
        assert!((val_interp.re - val_ideal).abs() < 1e-4);
    }

    #[test]
    fn test_caf_correlation() {
        let fft_size = 128;
        let mut engine = CafEngine::new(fft_size);

        // Generate a reference signal (random noise)
        let mut rng = rand::thread_rng();
        let mut reference = vec![Complex::new(0.0, 0.0); 256];
        for i in 0..256 {
            use rand::Rng;
            reference[i] = Complex::new(rng.gen::<f32>() - 0.5, rng.gen::<f32>() - 0.5);
        }

        // Generate surveillance signal as delayed and Doppler shifted version of reference
        // Delay = 3 samples, Doppler = 10 bins shift (representing frequency shift)
        let delay = 3;
        let doppler_bin_shift = 10;
        let mut surveillance = vec![Complex::new(0.0, 0.0); 256];
        for n in 0..256 {
            if n >= delay {
                let phase = 2.0 * std::f32::consts::PI * (doppler_bin_shift as f32) * (n as f32)
                    / (fft_size as f32);
                let shift = Complex::from_polar(1.0, phase);
                surveillance[n] = reference[n - delay] * shift;
            }
        }

        let max_delay = 8;
        let caf = engine.compute(&reference, &surveillance, max_delay);

        // Find the peak in the 2D CAF matrix
        let mut peak_val = 0.0;
        let mut peak_delay = 0;
        let mut peak_bin = 0;

        for d in 0..max_delay {
            for b in 0..fft_size {
                if caf[d][b] > peak_val {
                    peak_val = caf[d][b];
                    peak_delay = d;
                    peak_bin = b;
                }
            }
        }

        // Verify that the CAF correctly localized the delay and Doppler bin shift!
        assert_eq!(peak_delay, delay, "Localized delay was incorrect");

        // The peak bin should be centered + shift:
        // center = fft_size / 2 = 64
        // shift = 10 -> peak_bin should be 74
        let expected_bin = fft_size / 2 + doppler_bin_shift;
        assert_eq!(
            peak_bin, expected_bin,
            "Localized Doppler bin shift was incorrect"
        );
    }
}
