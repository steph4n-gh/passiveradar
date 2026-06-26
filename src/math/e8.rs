use num_complex::Complex;

/// E8 Lattice decoder and projection helpers.
#[derive(Debug, Clone)]
pub struct E8Decoder {
    pub scale: f32,
}

impl E8Decoder {
    pub fn new(scale: f32) -> Self {
        Self { scale }
    }

    /// Decode a point in R^8 to the closest point in the E8 lattice.
    /// Uses Conway and Sloane's fast O(8) decoding algorithm.
    pub fn decode(&self, y: &[f32; 8]) -> [f32; 8] {
        // Step 1: Find closest point in D_8 (all integers, sum is even)
        let mut f_y = [0.0f32; 8];
        let mut f_sum = 0i32;
        let mut f_errors = [0.0f32; 8];
        for i in 0..8 {
            let r = y[i].round();
            f_y[i] = r;
            f_sum += r as i32;
            f_errors[i] = (y[i] - r).abs();
        }

        if f_sum % 2 != 0 {
            // Parity is odd. Find index with largest rounding error to flip its rounding.
            let mut max_err_idx = 0;
            let mut max_err = f_errors[0];
            for i in 1..8 {
                if f_errors[i] > max_err {
                    max_err = f_errors[i];
                    max_err_idx = i;
                }
            }
            // Flip rounding direction
            if y[max_err_idx] >= f_y[max_err_idx] {
                f_y[max_err_idx] -= 1.0;
            } else {
                f_y[max_err_idx] += 1.0;
            }
        }

        // Step 2: Find closest point in D_8 + 1/2 (all half-integers, sum is even)
        let mut g_y = [0.0f32; 8];
        let mut g_errors = [0.0f32; 8];
        for i in 0..8 {
            // Round to nearest half-integer: round(y - 0.5) + 0.5
            let r = (y[i] - 0.5).round() + 0.5;
            g_y[i] = r;
            g_errors[i] = (y[i] - r).abs();
        }
        
        let mut g_sum_f = 0.0f32;
        for i in 0..8 {
            g_sum_f += g_y[i];
        }
        let g_sum_i = g_sum_f.round() as i32;

        if g_sum_i % 2 != 0 {
            // Find index with largest rounding error to flip its rounding
            let mut max_err_idx = 0;
            let mut max_err = g_errors[0];
            for i in 1..8 {
                if g_errors[i] > max_err {
                    max_err = g_errors[i];
                    max_err_idx = i;
                }
            }
            // Flip rounding direction by 1.0 (retains half-integer status)
            if y[max_err_idx] >= g_y[max_err_idx] {
                g_y[max_err_idx] -= 1.0;
            } else {
                g_y[max_err_idx] += 1.0;
            }
        }

        // Step 3: Compare distances to f_y and g_y
        let mut dist_f = 0.0f32;
        let mut dist_g = 0.0f32;
        for i in 0..8 {
            dist_f += (y[i] - f_y[i]).powi(2);
            dist_g += (y[i] - g_y[i]).powi(2);
        }

        if dist_f <= dist_g {
            f_y
        } else {
            g_y
        }
    }

    /// Projects 4 complex samples into 8D real space, decodes, and computes the quantization residual error.
    ///
    /// # Note on Scaling and Normalization
    /// The incoming complex samples should ideally be pre-normalized (e.g., using dynamic range compression
    /// or Automatic Gain Control / envelope normalization) prior to scaling by `self.scale` to ensure the 
    /// signals occupy a stable volume in the lattice space and prevent underflow/overflow or poor quantization.
    pub fn project_and_decode(&self, samples: &[Complex<f32>; 4]) -> ([f32; 8], f32) {
        let mut y = [0.0f32; 8];
        for i in 0..4 {
            y[2 * i] = samples[i].re * self.scale;
            y[2 * i + 1] = samples[i].im * self.scale;
        }

        let decoded = self.decode(&y);

        let mut residual = 0.0f32;
        for i in 0..8 {
            residual += (y[i] - decoded[i]).powi(2);
        }
        residual = residual.sqrt();

        (decoded, residual)
    }
}
