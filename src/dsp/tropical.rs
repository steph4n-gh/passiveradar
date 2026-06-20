/// A non-linear spectral pruner using Tropical Wavelet min-plus algebra
/// and 2-adic Vladimirov fractional derivative checks.
pub struct TropicalWaveletCanceller {
    scratch: Vec<f32>,
    approx: Vec<f32>,
    background: Vec<f32>,
    weights: [f64; 129],
}

impl TropicalWaveletCanceller {
    pub fn new(max_size: usize) -> Self {
        let mut weights = [0.0f64; 129];
        for offset in -64isize..=64isize {
            if offset != 0 {
                let v = offset.unsigned_abs().trailing_zeros() as usize;
                weights[(offset + 64) as usize] = if v < 8 {
                    Self::ADIC_WEIGHTS[v]
                } else {
                    2.0_f64.powf(1.2 * v as f64)
                };
            }
        }
        Self {
            scratch: Vec::with_capacity(max_size),
            approx: Vec::with_capacity(max_size),
            background: Vec::with_capacity(max_size),
            weights,
        }
    }

    /// Computes the dynamic local noise background envelope using a 3-level
    /// min-plus Haar Wavelet decomposition (Tropical algebra addition).
    pub fn compute_background(&mut self, magnitudes: &[f32]) -> &[f32] {
        let n = magnitudes.len();
        if n == 0 {
            self.background.clear();
            return &self.background;
        }
        
        self.approx.clear();
        self.approx.extend_from_slice(magnitudes);
        
        let mut current_len = n;
        let mut levels_done = 0;

        // Perform 3-level decomposition
        for _scale in 0..3 {
            let next_len = current_len / 2;
            if next_len == 0 {
                break;
            }
            // In-place update of approx to save allocation. We read from 0..current_len and write to 0..next_len
            for k in 0..next_len {
                let v1 = if self.approx[2 * k].is_finite() { self.approx[2 * k] } else { 0.0 };
                let v2 = if self.approx[2 * k + 1].is_finite() { self.approx[2 * k + 1] } else { 0.0 };
                self.approx[k] = v1.min(v2); // Min-plus tropical addition
            }
            current_len = next_len;
            levels_done += 1;
        }

        // Reconstruct background by zeroing out detail coefficients
        self.background.clear();
        self.background.extend_from_slice(&self.approx[0..current_len]);
        
        for _scale in 0..levels_done {
            let cur_bg_len = self.background.len();
            // We need to double the size. To avoid allocations, we push elements
            // from back to front, but since we are doubling, we can just push
            for k in 0..cur_bg_len {
                let val = self.background[k];
                self.background.push(val); // this puts it at the end. We need to interleave.
            }
            // Actually, an easier in-place doubling:
            // Resize to 2x, then move elements from back to front
            self.background.resize(cur_bg_len * 2, 0.0);
            for k in (0..cur_bg_len).rev() {
                let val = self.background[k];
                self.background[2 * k] = val;
                self.background[2 * k + 1] = val;
            }
        }

        // Handle case where n was not a multiple of 8
        if self.background.len() < n {
            let last_val = self.background.last().cloned().unwrap_or(0.0);
            self.background.resize(n, last_val);
        } else {
            self.background.truncate(n);
        }

        &self.background
    }

    /// Pre-computed 2-adic weights table for the Vladimirov derivative.
    /// Avoids calling f64::powf() in the inner loop by caching 2^(1.2 * v) for v=0..7.
    /// v = trailing_zeros of |i-j|, max meaningful value is ~7 for neighborhood size 128.
    const ADIC_WEIGHTS: [f64; 8] = [
        1.0,                   // 2^(1.2*0) = 1.0
        2.2973568,             // 2^(1.2*1) = 2^1.2
        5.278031643,           // 2^(1.2*2) = 2^2.4
        12.125732532,          // 2^(1.2*3) = 2^3.6
        27.857618025,          // 2^(1.2*4) = 2^4.8
        64.0,                  // 2^(1.2*5) = 2^6.0
        147.033696,            // 2^(1.2*6) = 2^7.2
        337.794240,            // 2^(1.2*7) = 2^8.4
    ];

    /// Detects sharp stationary spikes (spurs) in the spectrum using a
    /// 2-adic valuation-weighted local difference quotient (Vladimirov fractional derivative).
    ///
    /// Optimizations vs. naive implementation:
    /// 1. Uses a pre-computed lookup table for 2-adic weights instead of f64::powf() per iteration.
    /// 2. Uses nth_element-style partial sort (select_nth_unstable) for O(n) median instead of O(n log n) full sort.
    /// 3. Early-continues on bins below background threshold before entering the inner loop.
    pub fn detect_spurs_vladimirov(&mut self, magnitudes: &[f32], background: &[f32]) -> Vec<usize> {
        let n = magnitudes.len();
        if n == 0 {
            return vec![];
        }

        // O(n) median via partial sort (select_nth_unstable) instead of full O(n log n) sort
        self.scratch.clear();
        for &x in magnitudes {
            if x.is_finite() {
                self.scratch.push(x);
            }
        }
        
        if self.scratch.is_empty() {
            return vec![];
        }
        let mid = self.scratch.len() / 2;
        self.scratch.select_nth_unstable_by(mid, |a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
        let median = self.scratch[mid].max(1e-6);

        let threshold_wavelet = 3.0 * median;
        let threshold_vladimirov = 8.0 * median;

        let mut spikes = Vec::new();

        // Vladimirov 2-adic local difference check using precomputed weights and loop splitting
        let weights = &self.weights;
        for i in 0..n {
            let x_i = if magnitudes[i].is_finite() { magnitudes[i] as f64 } else { 0.0 };

            // Only examine bins exceeding local wavelet background + threshold
            if x_i as f32 <= background[i] + threshold_wavelet {
                continue;
            }

            let mut sum_deriv = 0.0f64;
            // Scan nearby neighborhood for localized difference quotient
            let start = i.saturating_sub(64);
            let end = (i + 64).min(n);

            let offset_base = 64isize - i as isize;

            // Loop 1: left of i (j < i)
            for j in start..i {
                let x_j = if magnitudes[j].is_finite() { magnitudes[j] as f64 } else { 0.0 };
                let weight = unsafe { *weights.get_unchecked((j as isize + offset_base) as usize) };
                sum_deriv += (x_i - x_j) * weight;
            }

            // Loop 2: right of i (j > i)
            for j in (i + 1)..end {
                let x_j = if magnitudes[j].is_finite() { magnitudes[j] as f64 } else { 0.0 };
                let weight = unsafe { *weights.get_unchecked((j as isize + offset_base) as usize) };
                sum_deriv += (x_i - x_j) * weight;
            }

            // If the fractional difference rate is high, it is a sharp spur/spike
            if sum_deriv.is_finite() && sum_deriv as f32 > threshold_vladimirov {
                spikes.push(i);
            }
        }

        spikes
    }

    /// Notches out detected spurs, replacing them with the local background envelope,
    /// while protecting bins near active target Doppler frequencies.
    /// Returns the computed background envelope so the caller can reuse it
    /// without recomputing (saves a full wavelet decomposition per call).
    pub fn notch_stationary_spurs(
        &mut self,
        fft_magnitudes: &mut [f32],
        active_dopplers: &[f64],
        baseband_rate: f64,
    ) -> &[f32] {
        let n = fft_magnitudes.len();
        if n == 0 {
            self.background.clear();
            return &self.background;
        }

        // We compute it, which stores it in self.background, and then we pass a slice to detect_spurs_vladimirov.
        // Wait, self is mutably borrowed by compute_background, so we can't borrow it again. 
        // We have to rely on self.background directly or let compute_background run and then we know it's in self.background.
        self.compute_background(fft_magnitudes);
        
        // Since we need to pass a slice of self.background to detect_spurs, we just let it use self.background directly internally,
        // but wait, detect_spurs_vladimirov takes `background: &[f32]`. We can't pass `&self.background` while mutating `self.scratch`.
        // We'll have to clone or do it differently. Actually we can do:
        let spikes = {
            // we can temporarily swap out the background to borrow it without borrowing self
            let bg = std::mem::take(&mut self.background);
            let s = self.detect_spurs_vladimirov(fft_magnitudes, &bg);
            self.background = bg;
            s
        };

        // Map active target dopplers to bin indices
        let active_bins: Vec<usize> = active_dopplers
            .iter()
            .map(|&dop| {
                let bin = ((dop / baseband_rate) * n as f64 + (n as f64 / 2.0)).round() as isize;
                bin.clamp(0, n as isize - 1) as usize
            })
            .collect();

        // 3 Hz guard window in bin resolution
        let bin_resolution = baseband_rate / n as f64;
        let guard_bins = (3.0 / bin_resolution).ceil() as usize;

        for s in spikes {
            // Check if this spike is near any active target Doppler
            let near_target = active_bins.iter().any(|&target_bin| {
                let diff = (s as isize - target_bin as isize).abs() as usize;
                diff <= guard_bins
            });

            if !near_target {
                // Notch the spur! Replace it with the smooth background level
                fft_magnitudes[s] = self.background[s];
            }
        }

        &self.background
    }
}
