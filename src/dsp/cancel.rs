use num_complex::Complex;

/// A simple first-order IIR DC blocker / high-pass filter.
/// Formula: y(n) = x(n) - x(n-1) + alpha * y(n-1)
pub struct DcBlocker {
    alpha: f32,
    prev_input: Complex<f32>,
    prev_output: Complex<f32>,
}

impl DcBlocker {
    pub fn new(alpha: f32) -> Self {
        Self {
            alpha,
            prev_input: Complex::new(0.0, 0.0),
            prev_output: Complex::new(0.0, 0.0),
        }
    }

    pub fn process(&mut self, sample: Complex<f32>) -> Complex<f32> {
        let output = sample - self.prev_input + self.prev_output * self.alpha;
        self.prev_input = sample;
        self.prev_output = output;
        output
    }

    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        output.clear();
        for &x in input {
            output.push(self.process(x));
        }
    }

    pub fn set_alpha(&mut self, alpha: f32) {
        self.alpha = alpha;
    }
}

/// A Normalized Least Mean Squares (NLMS) adaptive filter for clutter cancellation.
/// Uses a delayed version of the signal as the reference input to subtract correlated clutter
/// while preserving uncorrelated, moving Doppler target echoes.
pub struct NlmsCanceler {
    weights: Vec<Complex<f32>>,
    ref_history: Vec<Complex<f32>>,
    ref_index: usize,
    num_taps: usize,
    mu: f32,
    eps: f32,
    delay: usize,
    signal_history: Vec<Complex<f32>>,
    sig_index: usize,
    power: f32,
}

impl NlmsCanceler {
    pub fn new(num_taps: usize, mu: f32, delay: usize) -> Self {
        Self {
            weights: vec![Complex::new(0.0, 0.0); num_taps],
            // History buffer for reference signal
            ref_history: vec![Complex::new(0.0, 0.0); num_taps],
            ref_index: 0,
            num_taps,
            mu,
            eps: 1e-4,
            delay,
            // History buffer to delay the surveillance signal matching the reference delay
            signal_history: vec![Complex::new(0.0, 0.0); delay + 1],
            sig_index: 0,
            power: 0.0,
        }
    }

    /// Process a single complex sample.
    /// Returns the error signal (filtered output with clutter suppressed).
    pub fn process(&mut self, sample: Complex<f32>) -> Complex<f32> {
        // 1. Get the delayed reference value (from 'delay' steps ago) before writing the current sample
        let target_idx = if self.sig_index + 1 == self.delay + 1 { 0 } else { self.sig_index + 1 };
        let ref_val = self.signal_history[target_idx];

        // 2. Store the current sample in signal history for future delays
        self.signal_history[self.sig_index] = sample;

        self.ref_history[self.ref_index] = ref_val;

        // 3. Exact recomputation of power to prevent numerical drift
        let mut exact_power = 0.0;
        for i in 0..self.num_taps {
            exact_power += self.ref_history[i].norm_sqr();
        }
        self.power = exact_power;

        // 4. Compute filter output (estimated clutter using delayed samples): y(n) = w^H * x_delayed(n)
        let mut y = Complex::new(0.0, 0.0);
        let mut r_idx = self.ref_index;
        for i in 0..self.num_taps {
            y += self.ref_history[r_idx] * self.weights[i].conj();
            if r_idx == 0 {
                r_idx = self.num_taps - 1;
            } else {
                r_idx -= 1;
            }
        }

        // 5. The target to cancel is the current sample (d(n) = x(n))
        let error = sample - y;

        // 6. Update weights: w(n+1) = w(n) + mu * e^*(n) * x_delayed(n) / (power + eps)
        let normalization = self.mu / (self.power + self.eps);
        let error_conj = error.conj();
        let mut r_idx_w = self.ref_index;
        for i in 0..self.num_taps {
            self.weights[i] += self.ref_history[r_idx_w] * error_conj * normalization;
            if r_idx_w == 0 {
                r_idx_w = self.num_taps - 1;
            } else {
                r_idx_w -= 1;
            }
        }

        // 7. Advance indices
        if self.ref_index + 1 == self.num_taps {
            self.ref_index = 0;
        } else {
            self.ref_index += 1;
        }
        
        if self.sig_index + 1 == self.delay + 1 {
            self.sig_index = 0;
        } else {
            self.sig_index += 1;
        }

        error
    }

    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        output.clear();
        for &x in input {
            output.push(self.process(x));
        }
    }
}

fn complex_cholesky_6x6(a: &[[Complex<f32>; 6]; 6]) -> Option<[[Complex<f32>; 6]; 6]> {
    let mut l = [[Complex::new(0.0, 0.0); 6]; 6];
    for i in 0..6 {
        for j in 0..=i {
            let mut sum = a[i][j];
            for k in 0..j {
                sum -= l[i][k] * l[j][k].conj();
            }
            if i == j {
                if sum.re <= 0.0 {
                    return None;
                }
                l[i][j] = Complex::new(sum.re.sqrt(), 0.0);
            } else {
                l[i][j] = sum / l[j][j].re;
            }
        }
    }
    Some(l)
}

fn complex_cholesky_solve_6(a: &[[Complex<f32>; 6]; 6], b: &[Complex<f32>; 6]) -> Option<[Complex<f32>; 6]> {
    let l = complex_cholesky_6x6(a)?;
    // Forward substitution L * y = b
    let mut y = [Complex::new(0.0, 0.0); 6];
    for i in 0..6 {
        let mut sum = b[i];
        for k in 0..i {
            sum -= l[i][k] * y[k];
        }
        y[i] = sum / l[i][i].re;
    }
    // Backward substitution L^H * x = y
    let mut x = [Complex::new(0.0, 0.0); 6];
    for i in (0..6).rev() {
        let mut sum = y[i];
        for k in i+1..6 {
            sum -= l[k][i].conj() * x[k];
        }
        x[i] = sum / l[i][i].re;
    }
    Some(x)
}

/// Extensive Cancellation Algorithm (ECA) filter
/// Uses exact Orthogonal Subspace Projection on blocks of samples to obliterate 
/// the direct path and stationary clutter down to the noise floor.
pub struct EcaCanceler {
    history: [Complex<f32>; 6],
    x_ext: Vec<Complex<f32>>,
}

impl EcaCanceler {
    pub fn new() -> Self {
        Self {
            history: [Complex::new(0.0, 0.0); 6],
            x_ext: Vec::with_capacity(10000),
        }
    }

    pub fn process_block(
        &mut self,
        input: &[Complex<f32>],
        clean_surv_output: &mut Vec<Complex<f32>>,
        surrogate_ref_output: &mut Vec<Complex<f32>>,
    ) {
        let n = input.len();
        clean_surv_output.clear();
        surrogate_ref_output.clear();
        if n == 0 { return; }
        
        let taps = 6;
        let mut r = [[Complex::new(0.0, 0.0); 6]; 6];
        let mut p = [Complex::new(0.0, 0.0); 6];

        // Construct extended signal x_ext = [history, input]
        self.x_ext.resize(n + 6, Complex::new(0.0, 0.0));
        self.x_ext[0..6].copy_from_slice(&self.history);
        self.x_ext[6..n+6].copy_from_slice(input);

        // 1. Compute r[0][d] for d in 0..5 using O(N) operations
        for d in 0..taps {
            let mut sum = Complex::new(0.0, 0.0);
            for i in 0..n {
                sum += self.x_ext[5 + i].conj() * self.x_ext[5 + i - d];
            }
            r[0][d] = sum;
        }

        // 2. Compute the rest of the upper triangle of r using O(1) sliding window updates
        for d in 0..taps {
            for j in 1..(taps - d) {
                let term_in = self.x_ext[5 - j].conj() * self.x_ext[5 - j - d];
                let term_out = self.x_ext[5 - j + n].conj() * self.x_ext[5 - j + n - d];
                r[j][j + d] = r[j - 1][j - 1 + d] + term_in - term_out;
            }
        }

        // 3. Fill in the lower triangle of r using Hermitian symmetry
        for j in 0..taps {
            for k in 0..j {
                r[j][k] = r[k][j].conj();
            }
        }

        // 4. Compute p[j] for j in 0..5 using O(N) operations
        for j in 0..taps {
            let mut sum = Complex::new(0.0, 0.0);
            for i in 0..n {
                sum += self.x_ext[5 + i - j].conj() * input[i];
            }
            p[j] = sum;
        }

        // Diagonal regularization (Ridge / Tikhonov)
        // We use tau to inject a tiny bit of "virtual noise" scaled by the block size.
        // This prevents the linear predictor from achieving "perfect cancellation" of the
        // constant-modulus phase signal, preserving the target echoes!
        let tau = 1e-3 * (r[0][0].re / taps as f32 + 1e-6); 
        for j in 0..taps {
            r[j][j] += Complex::new(tau, 0.0);
        }

        let weights = match complex_cholesky_solve_6(&r, &p) {
            Some(w) => w,
            None => [Complex::new(0.0, 0.0); 6],
        };

        // 5. Generate outputs
        for i in 0..n {
            let mut y_clutter = Complex::new(0.0, 0.0);
            for k in 0..taps {
                y_clutter += self.x_ext[5 + i - k] * weights[k];
            }
            
            surrogate_ref_output.push(y_clutter);
            clean_surv_output.push(input[i] - y_clutter);
        }

        // Update history
        if n >= 6 {
            self.history.copy_from_slice(&input[n - 6..n]);
        } else {
            for i in 0..(6 - n) {
                self.history[i] = self.history[i + n];
            }
            self.history[6 - n..6].copy_from_slice(input);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_eca_projection() {
        let mut canceler = EcaCanceler::new();
        let mut input = vec![Complex::new(0.0, 0.0); 100];

        // Create a strong direct path (sine wave)
        for i in 0..100 {
            let val = Complex::new((i as f32).cos(), (i as f32).sin());
            input[i] = val * 1000.0; // Massive direct path
        }

        let mut clean_surv = Vec::new();
        let mut surr_ref = Vec::new();
        canceler.process_block(&input, &mut clean_surv, &mut surr_ref);

        // Block-based projection means it's cancelled identically across the entire block!
        for i in 10..100 {
            assert!(clean_surv[i].norm() < 0.1, "Failed to cancel direct path, got {}", clean_surv[i]);
            assert!(surr_ref[i].norm() > 900.0, "Failed to capture surrogate reference");
        }
    }

    #[test]
    fn test_eca_batched_canceler_correctness() {
        let mut canceler = EcaBatchedCanceler::new(32, 10);
        let mut input = vec![Complex::new(0.0, 0.0); 256];
        
        for i in 0..256 {
            input[i] = Complex::new((i as f32).cos(), (i as f32).sin()) * 100.0;
        }
        
        for i in 0..256 {
            input[i] += Complex::new((i as f32 * 0.15).cos(), (i as f32 * 0.15).sin()) * 2.0;
        }

        let mut output = Vec::new();
        canceler.process_block(&input, &mut output);

        assert_eq!(output.len(), 256);
        
        let mut sum_target = 0.0;
        for i in 100..256 {
            sum_target += output[i].norm();
        }
        let avg_target = sum_target / 156.0;
        assert!(avg_target < 10.0, "Static clutter not suppressed, avg output norm: {}", avg_target);
        assert!(avg_target > 0.2, "Moving target signal cancelled out, avg output norm: {}", avg_target);
    }
}

pub struct EcaBatchedCanceler {
    num_taps: usize,
    history: Vec<Complex<f32>>,
    pub max_cg_iterations: usize,
    gpu_state: Option<crate::dsp::gpu::GpuEcaState>,
}

impl EcaBatchedCanceler {
    pub fn new(num_taps: usize, max_cg_iterations: usize) -> Self {
        Self {
            num_taps,
            history: vec![Complex::new(0.0, 0.0); num_taps],
            max_cg_iterations,
            gpu_state: None,
        }
    }

    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        let n_samples = input.len();
        output.clear();
        
        if n_samples == 0 {
            return;
        }

        // Try GPU path
        if let Some(pipeline) = crate::dsp::gpu::get_gpu_eca_pipeline() {
            let state = self.gpu_state.get_or_insert_with(|| {
                crate::dsp::gpu::GpuEcaState::new(pipeline, n_samples, self.num_taps)
            });
            let gpu_out = state.process_eca(pipeline, input, &self.history, self.max_cg_iterations);
            *output = gpu_out;

            if n_samples >= self.num_taps {
                for i in 0..self.num_taps {
                    self.history[i] = input[n_samples - self.num_taps + i];
                }
            } else {
                self.history.rotate_left(n_samples);
                for i in 0..n_samples {
                    self.history[self.num_taps - n_samples + i] = input[i];
                }
            }
            return;
        }

        output.resize(n_samples, Complex::new(0.0, 0.0));

        let mut w = vec![Complex::new(0.0, 0.0); self.num_taps];
        let mut r = input.to_vec(); 
        let mut p = vec![Complex::new(0.0, 0.0); self.num_taps];
        self.apply_xh(&r, input, &mut p);
        
        let s_cg = p.clone(); 
        
        let mut norms_sq = 0.0;
        for v in &s_cg { norms_sq += v.norm_sqr(); }

        for _ in 0..self.max_cg_iterations {
            if norms_sq < 1e-10 { break; }
            
            let mut q = vec![Complex::new(0.0, 0.0); n_samples];
            self.apply_x(&p, input, &mut q);
            
            let mut normq_sq = 0.0;
            for v in &q { normq_sq += v.norm_sqr(); }
            
            if normq_sq < 1e-15 { break; }
            
            let alpha = norms_sq / normq_sq;
            
            for i in 0..self.num_taps {
                w[i] = w[i] + p[i] * alpha;
            }
            
            for i in 0..n_samples {
                r[i] = r[i] - q[i] * alpha;
            }
            
            let mut s_new = vec![Complex::new(0.0, 0.0); self.num_taps];
            self.apply_xh(&r, input, &mut s_new);
            
            let mut norms_new_sq = 0.0;
            for v in &s_new { norms_new_sq += v.norm_sqr(); }
            
            let beta = norms_new_sq / norms_sq;
            
            for i in 0..self.num_taps {
                p[i] = s_new[i] + p[i] * beta;
            }
            
            norms_sq = norms_new_sq;
        }
        
        for i in 0..n_samples {
            output[i] = r[i];
        }

        if n_samples >= self.num_taps {
            for i in 0..self.num_taps {
                self.history[i] = input[n_samples - self.num_taps + i];
            }
        } else {
            self.history.rotate_left(n_samples);
            for i in 0..n_samples {
                self.history[self.num_taps - n_samples + i] = input[i];
            }
        }
    }
    
    fn apply_x(&self, p: &[Complex<f32>], input: &[Complex<f32>], q: &mut [Complex<f32>]) {
        let n = input.len();
        let k_taps = p.len();
        
        for i in 0..n {
            let mut sum = Complex::new(0.0, 0.0);
            for k in 0..k_taps {
                let delay = k + 1;
                let val = if i >= delay {
                    input[i - delay]
                } else {
                    self.history[self.num_taps - delay + i]
                };
                sum += val * p[k];
            }
            q[i] = sum;
        }
    }

    fn apply_xh(&self, r: &[Complex<f32>], input: &[Complex<f32>], s: &mut [Complex<f32>]) {
        let n = input.len();
        let k_taps = s.len();
        
        for k in 0..k_taps {
            let mut sum = Complex::new(0.0, 0.0);
            let delay = k + 1;
            for i in 0..n {
                let val = if i >= delay {
                    input[i - delay]
                } else {
                    self.history[self.num_taps - delay + i]
                };
                sum += val.conj() * r[i];
            }
            s[k] = sum;
        }
    }
}
