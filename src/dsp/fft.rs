use num_complex::Complex;
use rustfft::{num_complex::Complex as FftComplex, FftPlanner};
use std::collections::VecDeque;
use std::sync::Arc;
use std::sync::atomic::AtomicBool;

pub static DISABLE_GPU: AtomicBool = AtomicBool::new(false);

pub enum FftBackend {
    Cpu(Arc<dyn rustfft::Fft<f32>>),
    #[cfg(feature = "gpu-fft")]
    Gpu(wgsl_fft::GpuFft),
}

pub struct FftEngine {
    fft_size: usize,
    fft: Arc<dyn rustfft::Fft<f32>>,
    backend: FftBackend,
    window: Vec<f32>,
    buffer: VecDeque<Complex<f32>>,
    scratch: Vec<FftComplex<f32>>,
    // Pre-allocated work buffers to eliminate per-frame heap allocation
    fft_input: Vec<FftComplex<f32>>,
    magnitude: Vec<f32>,
}

impl FftEngine {
    pub fn new(fft_size: usize) -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(fft_size);

        // Generate Hann window
        let mut window = vec![0.0f32; fft_size];
        for i in 0..fft_size {
            let val =
                0.5 * (1.0 - (2.0 * std::f32::consts::PI * i as f32 / (fft_size - 1) as f32).cos());
            window[i] = val;
        }

        let scratch = vec![FftComplex::new(0.0, 0.0); fft.get_inplace_scratch_len()];

        let backend = {
            #[cfg(feature = "gpu-fft")]
            {
                if !DISABLE_GPU.load(std::sync::atomic::Ordering::SeqCst) {
                    match wgsl_fft::GpuFft::new() {
                        Ok(gpu) => FftBackend::Gpu(gpu),
                        Err(_) => FftBackend::Cpu(fft.clone()),
                    }
                } else {
                    FftBackend::Cpu(fft.clone())
                }
            }
            #[cfg(not(feature = "gpu-fft"))]
            {
                FftBackend::Cpu(fft.clone())
            }
        };

        Self {
            fft_size,
            fft,
            backend,
            window,
            buffer: VecDeque::with_capacity(fft_size * 2),
            scratch,
            fft_input: vec![FftComplex::new(0.0, 0.0); fft_size],
            magnitude: vec![0.0f32; fft_size],
        }
    }

    /// Feed new samples into the processing buffer.
    pub fn feed(&mut self, samples: &[Complex<f32>]) {
        self.buffer.extend(samples.iter());
    }

    /// Check if there are enough samples to compute the next frame.
    pub fn has_frame(&self) -> bool {
        self.buffer.len() >= self.fft_size
    }

    /// Pull the next available FFT magnitude frame.
    /// If there are enough samples, applies the window, runs FFT, shifts DC to center,
    /// and advances the buffer by `step_size` (achieving overlap).
    /// Returns `None` if there are not enough samples in the buffer.
    pub fn next_frame(&mut self, step_size: usize) -> Option<Vec<f32>> {
        if self.buffer.len() < self.fft_size {
            return None;
        }

        // 1. Extract the first fft_size samples and apply the window function
        for i in 0..self.fft_size {
            let sample = self.buffer[i];
            let w = self.window[i];
            self.fft_input[i] = FftComplex::new(sample.re * w, sample.im * w);
        }

        // 2. Process forward FFT
        let mut processed_on_gpu = false;

        if !DISABLE_GPU.load(std::sync::atomic::Ordering::SeqCst) {
            #[cfg(feature = "gpu-fft")]
            {
                if let FftBackend::Gpu(ref gpu) = self.backend {
                    let input_vec: Vec<Complex<f32>> = self.fft_input.iter().map(|c| Complex::new(c.re, c.im)).collect();
                    if let Ok(mut results) = gpu.fft(&[input_vec]) {
                        if !results.is_empty() {
                            let output_vec = results.remove(0);
                            for (i, c) in output_vec.into_iter().enumerate() {
                                if i < self.fft_input.len() {
                                    self.fft_input[i] = FftComplex::new(c.re, c.im);
                                }
                            }
                            processed_on_gpu = true;
                        }
                    }
                }
            }
        }

        if !processed_on_gpu {
            self.fft.process_with_scratch(&mut self.fft_input, &mut self.scratch);
        }

        // 3. Compute magnitude and perform fftshift to center DC (0 Hz) at index fft_size / 2
        for i in 0..self.fft_size {
            let shift_idx = (i + self.fft_size / 2) % self.fft_size;
            self.magnitude[shift_idx] = self.fft_input[i].norm();
        }

        // 4. Advance the sliding window
        self.buffer.drain(0..step_size);

        Some(self.magnitude.clone())
    }

    pub fn clear(&mut self) {
        self.buffer.clear();
    }
}
