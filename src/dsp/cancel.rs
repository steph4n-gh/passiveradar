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
