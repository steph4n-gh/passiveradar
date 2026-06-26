use num_complex::Complex;

pub struct CubicHermiteDeclipper {
    max_run_length: usize,
    threshold: f32,
    max_reconstructed_value: f32,
}

impl CubicHermiteDeclipper {
    pub fn new(max_run_length: usize, threshold: f32, max_reconstructed_value: f32) -> Self {
        Self {
            max_run_length,
            threshold,
            max_reconstructed_value,
        }
    }

    /// Process a block of samples in place, declipping both Real and Imaginary components.
    pub fn process_block(&self, block: &mut [Complex<f32>]) {
        let n = block.len();
        if n < 6 {
            return;
        }

        Self::declip_channel(block, true, self.threshold, self.max_run_length, self.max_reconstructed_value);
        Self::declip_channel(block, false, self.threshold, self.max_run_length, self.max_reconstructed_value);
    }

    fn declip_channel(block: &mut [Complex<f32>], is_real: bool, threshold: f32, max_run: usize, max_val: f32) {
        let n = block.len();
        let mut i = 0;

        let get_val = |c: &Complex<f32>| if is_real { c.re } else { c.im };
        let set_val = |c: &mut Complex<f32>, v: f32| {
            if is_real {
                c.re = v;
            } else {
                c.im = v;
            }
        };

        while i < n {
            let val = get_val(&block[i]);
            if val.abs() >= threshold {
                let start = i;
                while i < n && get_val(&block[i]).abs() >= threshold {
                    i += 1;
                }
                let end = i;
                let run_len = end - start;

                // Reconstruct only if the clip run isn't excessively long (which would make estimation wild)
                // and we have enough valid sample history/future around it.
                if run_len <= max_run && start >= 2 && end + 2 <= n {
                    let idx_before = start - 1;
                    let idx_after = end;

                    let y_before = get_val(&block[idx_before]);
                    let y_after = get_val(&block[idx_after]);

                    // Estimate derivatives at boundaries using backward and forward finite differences
                    let m_before = y_before - get_val(&block[start - 2]);
                    let m_after = get_val(&block[end + 1]) - y_after;

                    let h = (idx_after - idx_before) as f32;
                    for k in start..end {
                        let t = (k - idx_before) as f32 / h;
                        let t2 = t * t;
                        let t3 = t2 * t;

                        // Hermite basis functions
                        let h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
                        let h10 = t3 - 2.0 * t2 + t;
                        let h01 = -2.0 * t3 + 3.0 * t2;
                        let h11 = t3 - t2;

                        let mut p = h00 * y_before + h10 * h * m_before + h01 * y_after + h11 * h * m_after;

                        let sign = y_before.signum();
                        // Spline reconstruction must reconstruct a peak larger than the clipping threshold
                        if p.abs() < threshold {
                            p = sign * threshold;
                        }
                        // Clamp to prevent arithmetic instability or extreme overshoot
                        if p.abs() > max_val {
                            p = sign * max_val;
                        }

                        set_val(&mut block[k], p);
                    }
                }
            } else {
                i += 1;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_cubic_hermite_declip() {
        // Create a fake sine wave that gets clipped flat
        let mut block = vec![
            Complex::new(0.5, 0.0),
            Complex::new(0.8, 0.0),
            Complex::new(0.99, 0.0), // Clipped
            Complex::new(0.99, 0.0), // Clipped
            Complex::new(0.8, 0.0),
            Complex::new(0.5, 0.0),
        ];

        let declipper = CubicHermiteDeclipper::new(4, 0.98, 1.5);
        declipper.process_block(&mut block);

        // Clipped values should be reconstructed to be greater than threshold
        assert!(block[2].re > 0.99);
        assert!(block[3].re > 0.99);
        assert!(block[2].re <= 1.5);
    }
}
