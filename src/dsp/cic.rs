use num_complex::Complex;

/// Programmable Cascaded Integrator-Comb (CIC) Decimation filter.
/// Uses wrapping i64 fixed-point arithmetic to prevent overflow and maintain exact state.
#[derive(Debug, Clone)]
pub struct CicDecimator {
    r: usize,
    n: usize,
    integrators_re: Vec<i64>,
    integrators_im: Vec<i64>,
    combs_re: Vec<i64>,
    combs_im: Vec<i64>,
    decimation_counter: usize,
}

impl CicDecimator {
    /// Create a new CicDecimator with decimation factor R and N stages.
    pub fn new(r: usize, n: usize) -> Self {
        Self {
            r,
            n,
            integrators_re: vec![0; n],
            integrators_im: vec![0; n],
            combs_re: vec![0; n],
            combs_im: vec![0; n],
            decimation_counter: 0,
        }
    }

    /// Process a block of input complex float samples and write the decimated/filtered outputs.
    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        if self.r == 1 {
            // Bypass decimation
            output.extend_from_slice(input);
            return;
        }

        let scale = 1.0 / ((self.r as f64).powi(self.n as i32) * 1073741824.0);

        for &sample in input {
            // Scale input float by (1 << 30)
            let mut curr_re = (sample.re as f64 * 1073741824.0).round() as i64;
            let mut curr_im = (sample.im as f64 * 1073741824.0).round() as i64;

            // N integrator stages
            for i in 0..self.n {
                self.integrators_re[i] = self.integrators_re[i].wrapping_add(curr_re);
                self.integrators_im[i] = self.integrators_im[i].wrapping_add(curr_im);
                curr_re = self.integrators_re[i];
                curr_im = self.integrators_im[i];
            }

            // Decimation by R
            self.decimation_counter += 1;
            if self.decimation_counter == self.r {
                self.decimation_counter = 0;

                // N comb stages
                let mut comb_in_re = curr_re;
                let mut comb_in_im = curr_im;

                for i in 0..self.n {
                    let diff_re = comb_in_re.wrapping_sub(self.combs_re[i]);
                    let diff_im = comb_in_im.wrapping_sub(self.combs_im[i]);
                    self.combs_re[i] = comb_in_re;
                    self.combs_im[i] = comb_in_im;
                    comb_in_re = diff_re;
                    comb_in_im = diff_im;
                }

                // Scale back to float
                output.push(Complex::new(
                    (comb_in_re as f64 * scale) as f32,
                    (comb_in_im as f64 * scale) as f32,
                ));
            }
        }
    }
}
