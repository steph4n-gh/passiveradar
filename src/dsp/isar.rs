use num_complex::Complex;

/// Phase 2 Model 1: Wideband OFDM Fractional Delay Cross Ambiguity Function
/// Computes frequency-domain cross-correlation with sub-sample delay tau using Shift Theorem.
pub fn ofdm_fractional_delay_caf(
    surv: &[Complex<f32>],
    reference: &[Complex<f32>],
    tau: f32,
) -> Vec<Complex<f32>> {
    let n = surv.len();
    if n == 0 || reference.len() != n {
        return vec![];
    }

    let mut planner = rustfft::FftPlanner::new();
    let fft = planner.plan_fft_forward(n);
    let ifft = planner.plan_fft_inverse(n);

    let mut surv_fft: Vec<rustfft::num_complex::Complex<f32>> = surv
        .iter()
        .map(|c| rustfft::num_complex::Complex::new(c.re, c.im))
        .collect();
    let mut ref_fft: Vec<rustfft::num_complex::Complex<f32>> = reference
        .iter()
        .map(|c| rustfft::num_complex::Complex::new(c.re, c.im))
        .collect();

    // Run forward FFTs
    let mut scratch = vec![rustfft::num_complex::Complex::new(0.0, 0.0); fft.get_inplace_scratch_len()];
    fft.process_with_scratch(&mut surv_fft, &mut scratch);
    fft.process_with_scratch(&mut ref_fft, &mut scratch);

    // Apply Shift Theorem and element-wise conjugate multiplication:
    // S_surv(f) * S_ref^*(f) * exp(-j * 2 * pi * f * tau)
    let mut prod = vec![rustfft::num_complex::Complex::new(0.0, 0.0); n];
    for k in 0..n {
        // Frequency index f normalized to [-0.5, 0.5]
        let f = if k <= n / 2 {
            (k as f32) / (n as f32)
        } else {
            ((k as f32) - (n as f32)) / (n as f32)
        };

        // Shift phasor: exp(j * 2 * pi * f * tau)
        let phase = 2.0 * std::f32::consts::PI * f * tau;
        let shift = rustfft::num_complex::Complex::new(phase.cos(), phase.sin());

        // Conjugate multiplication and shift
        prod[k] = surv_fft[k] * ref_fft[k].conj() * shift;
    }

    // Run Inverse FFT to obtain time-domain fractional cross-correlation
    let mut scratch_inv = vec![rustfft::num_complex::Complex::new(0.0, 0.0); ifft.get_inplace_scratch_len()];
    ifft.process_with_scratch(&mut prod, &mut scratch_inv);

    // Normalize IFFT outputs
    prod.iter()
        .map(|c| Complex::new(c.re / (n as f32), c.im / (n as f32)))
        .collect()
}

/// Phase 2 Model 2: The CLEAN Algorithm (Deconvolution / Orthogonal Matching Pursuit)
/// Iteratively subtracts 2D point-spread functions (PSF) to expose weak targets.
pub fn clean_ambiguity_map(
    map: &mut [Vec<f32>],
    iterations: usize,
    loop_gain: f32,
) -> Vec<(usize, usize, f32)> {
    let n_rows = map.len();
    if n_rows == 0 {
        return vec![];
    }
    let n_cols = map[0].len();
    if n_cols == 0 {
        return vec![];
    }

    let mut clean_components = Vec::new();
    let sigma_row = 2.0f32;
    let sigma_col = 2.0f32;

    // Pre-allocated column-wise PSF buffer to prevent reallocation inside the loop
    let mut psf_col = vec![0.0f32; n_cols];

    for _ in 0..iterations {
        // 1. Locate absolute peak
        let mut max_val = 0.0f32;
        let mut peak_r = 0;
        let mut peak_c = 0;

        for r in 0..n_rows {
            for c in 0..n_cols {
                let val = map[r][c].abs();
                if val > max_val {
                    max_val = val;
                    peak_r = r;
                    peak_c = c;
                }
            }
        }

        // Check if peak is too small (e.g. down to noise floor)
        if max_val < 1e-4 {
            break;
        }

        let peak_val = map[peak_r][peak_c];
        clean_components.push((peak_r, peak_c, peak_val));

        // 2. Precompute 1D column Gaussian factor using dimensional separability
        for c in 0..n_cols {
            let dc = (c as f32 - peak_c as f32).powi(2);
            psf_col[c] = (-dc / (2.0 * sigma_col.powi(2))).exp();
        }

        // 3. Subtract ideal 2D Gaussian point-spread function
        for r in 0..n_rows {
            let dr = (r as f32 - peak_r as f32).powi(2);
            let psf_row = (-dr / (2.0 * sigma_row.powi(2))).exp();
            let term = loop_gain * peak_val * psf_row;
            for c in 0..n_cols {
                map[r][c] -= term * psf_col[c];
            }
        }
    }

    clean_components
}

/// Phase 2 Model 3: Inverse Radon Filtered Back-Projection (FBP) Tomography
/// Reconstructs 2D spatial tomographic image from range profiles taken at different bistatic angles.
pub fn backproject_tomography(
    profiles: &[Vec<f32>],
    angles_rad: &[f32],
    grid_size: usize,
) -> Vec<Vec<f32>> {
    let n_angles = profiles.len();
    if n_angles == 0 || angles_rad.len() != n_angles {
        return vec![];
    }
    let n_bins = profiles[0].len();
    if n_bins == 0 {
        return vec![];
    }

    // 1. Apply Ram-Lak (ramp) filter to each profile in the frequency domain
    let mut planner = rustfft::FftPlanner::new();
    let fft = planner.plan_fft_forward(n_bins);
    let ifft = planner.plan_fft_inverse(n_bins);

    let mut filtered_profiles = vec![vec![0.0f32; n_bins]; n_angles];

    for i in 0..n_angles {
        let mut fft_input: Vec<rustfft::num_complex::Complex<f32>> = profiles[i]
            .iter()
            .map(|&val| rustfft::num_complex::Complex::new(val, 0.0))
            .collect();

        let mut scratch = vec![rustfft::num_complex::Complex::new(0.0, 0.0); fft.get_inplace_scratch_len()];
        fft.process_with_scratch(&mut fft_input, &mut scratch);

        // Apply Ram-Lak filter H(f) = |f|
        for k in 0..n_bins {
            let f = if k <= n_bins / 2 {
                (k as f32) / (n_bins as f32)
            } else {
                ((n_bins - k) as f32) / (n_bins as f32)
            };
            fft_input[k] = fft_input[k] * f;
        }

        let mut scratch_inv = vec![rustfft::num_complex::Complex::new(0.0, 0.0); ifft.get_inplace_scratch_len()];
        ifft.process_with_scratch(&mut fft_input, &mut scratch_inv);

        for k in 0..n_bins {
            filtered_profiles[i][k] = fft_input[k].re / (n_bins as f32);
        }
    }

    // 2. Filtered Back-Projection
    // Precompute projection angles sin/cos lookup tables to avoid heavy transcendent computation inside the nested loops
    let mut cos_angles = Vec::with_capacity(n_angles);
    let mut sin_angles = Vec::with_capacity(n_angles);
    for &theta in angles_rad {
        let (sin, cos) = theta.sin_cos();
        cos_angles.push(cos);
        sin_angles.push(sin);
    }

    let mut image = vec![vec![0.0f32; grid_size]; grid_size];
    let half_grid = (grid_size as f32) / 2.0;
    let half_bins = (n_bins as f32) / 2.0;

    for x_idx in 0..grid_size {
        // Map grid coordinates to [-1.0, 1.0]
        let x = (x_idx as f32 - half_grid) / half_grid;
        for y_idx in 0..grid_size {
            let y = (y_idx as f32 - half_grid) / half_grid;

            let mut pixel_val = 0.0f32;

            for angle_idx in 0..n_angles {
                let cos_t = cos_angles[angle_idx];
                let sin_t = sin_angles[angle_idx];
                // Project spatial point (x, y) onto the angle direction using precomputed lookups
                let rho = x * cos_t + y * sin_t;

                // Map rho in [-1.0, 1.0] back to bin index in [0, n_bins-1]
                let bin = rho * half_bins + half_bins;
                
                if bin >= 0.0 && bin < (n_bins - 1) as f32 {
                    // Linear interpolation
                    let bin_floor = bin.floor() as usize;
                    let bin_ceil = (bin_floor + 1).min(n_bins - 1);
                    let frac = bin - bin_floor as f32;

                    let val = (1.0 - frac) * filtered_profiles[angle_idx][bin_floor]
                        + frac * filtered_profiles[angle_idx][bin_ceil];
                    pixel_val += val;
                }
            }

            // Store back-projected average
            image[x_idx][y_idx] = pixel_val / (n_angles as f32);
        }
    }

    image
}
