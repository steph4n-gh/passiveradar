use num_complex::Complex;
use rustfft::{FftPlanner, num_complex::Complex as FftComplex};
use std::sync::Arc;

/// A 100-channel Polyphase Filter Bank (PFB) designed to slice a 20 MSPS wideband input stream
/// into 100 independent 200 kHz channels in O(N log N) time, utilizing a prototype FIR filter
/// and a 100-point FFT.
pub struct PolyphaseChannelizer {
    num_channels: usize,
    decimation_factor: usize,
    taps: Vec<f32>,
    fft: Arc<dyn rustfft::Fft<f32>>,
    subfilters: Vec<Vec<f32>>,
    // Internal state buffers to prevent heap allocation in process loop
    filter_state: Vec<Vec<Complex<f32>>>,
}

impl PolyphaseChannelizer {
    /// Create a new Polyphase Channelizer.
    /// Ingests a wideband sample rate (e.g. 20 MSPS) and divides it into channels.
    pub fn new(num_channels: usize, decimation_factor: usize) -> Self {
        let mut planner = FftPlanner::new();
        let fft = planner.plan_fft_forward(num_channels);

        // Generate a prototype lowpass FIR filter (e.g. windowed sinc)
        // For a 100-channel PFB, standard filter length is L = M * P, where P is the polyphase factor.
        let polyphase_factor = 8; // 8 taps per sub-filter
        let total_taps = num_channels * polyphase_factor;
        let mut taps = vec![0.0f32; total_taps];

        // Sinc filter design: cut-off at 1 / num_channels
        let cutoff = 1.0 / num_channels as f32;
        let center = (total_taps - 1) as f32 / 2.0;
        for i in 0..total_taps {
            let t = i as f32 - center;
            let val = if t == 0.0 {
                2.0 * cutoff
            } else {
                (2.0 * std::f32::consts::PI * cutoff * t).sin() / (std::f32::consts::PI * t)
            };
            // Hamming window
            let w = 0.54 - 0.46 * (2.0 * std::f32::consts::PI * i as f32 / (total_taps - 1) as f32).cos();
            taps[i] = val * w;
        }

        // Decompose prototype taps into polyphase subfilters: h_p[n] = h[n*M + p]
        let mut subfilters = vec![vec![0.0f32; polyphase_factor]; num_channels];
        for p in 0..num_channels {
            for n in 0..polyphase_factor {
                subfilters[p][n] = taps[n * num_channels + p];
            }
        }

        let filter_state = vec![vec![Complex::new(0.0, 0.0); polyphase_factor]; num_channels];

        Self {
            num_channels,
            decimation_factor,
            taps,
            fft,
            subfilters,
            filter_state,
        }
    }

    /// Process a block of 20 MSPS wideband samples.
    /// Slices input samples into 100 channels of 200 kHz and returns them.
    pub fn process_block(&mut self, input: &[Complex<f32>]) -> Vec<Vec<Complex<f32>>> {
        let num_blocks = input.len() / self.decimation_factor;
        let mut output = vec![vec![Complex::new(0.0, 0.0); num_blocks]; self.num_channels];

        if input.is_empty() || num_blocks == 0 {
            return output;
        }

        // Scaffold of polyphase filtering and FFT:
        // For each block of decimation_factor input samples:
        // 1. Shift samples into the polyphase delay line structure.
        // 2. Perform convolution of each subfilter state with corresponding h_p taps.
        // 3. Run a forward FFT on the convolved channel outputs.
        // 4. Extract 100-channel bins.
        for b in 0..num_blocks {
            let offset = b * self.decimation_factor;
            let mut fft_buffer = vec![FftComplex::new(0.0, 0.0); self.num_channels];

            // 1. Update polyphase filter states and convolve
            for p in 0..self.num_channels {
                // Shift delay line
                self.filter_state[p].pop();
                let input_idx = offset + p;
                let sample = if input_idx < input.len() { input[input_idx] } else { Complex::new(0.0, 0.0) };
                self.filter_state[p].insert(0, sample);

                // Convolve filter state with sub-filter coefficients
                let mut sum = Complex::new(0.0, 0.0);
                for n in 0..self.subfilters[p].len() {
                    sum += self.filter_state[p][n] * self.subfilters[p][n];
                }
                fft_buffer[p] = FftComplex::new(sum.re, sum.im);
            }

            // 2. Run FFT to perform the channelizing phase-rotation
            let mut scratch = vec![FftComplex::new(0.0, 0.0); self.fft.get_inplace_scratch_len()];
            self.fft.process_with_scratch(&mut fft_buffer, &mut scratch);

            // 3. Write results to output channel buffers
            for ch in 0..self.num_channels {
                output[ch][b] = Complex::new(fft_buffer[ch].re, fft_buffer[ch].im);
            }
        }

        output
    }

    /// Scan all channels, compute average power, and route the top N strongest
    /// channels (active transmitter towers) to the tracking bank.
    pub fn select_strongest_channels(&self, channelized_data: &[Vec<Complex<f32>>], top_n: usize) -> Vec<usize> {
        let mut power_profile = Vec::with_capacity(self.num_channels);
        for ch in 0..self.num_channels {
            let p_sum: f32 = channelized_data[ch].iter().map(|c| c.norm_sqr()).sum();
            let avg_p = p_sum / channelized_data[ch].len().max(1) as f32;
            power_profile.push((ch, avg_p));
        }

        power_profile.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));
        power_profile.iter().take(top_n).map(|&(ch, _)| ch).collect()
    }
}
