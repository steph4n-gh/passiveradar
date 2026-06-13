use num_complex::Complex;
use rustfft::{num_complex::Complex as FftComplex, FftPlanner, Fft};
use std::sync::Arc;
use rayon::prelude::*;
use std::sync::LazyLock;

#[allow(dead_code)]
static HAS_AVX2_FMA: LazyLock<bool> = LazyLock::new(|| {
    #[cfg(target_arch = "x86_64")]
    {
        std::is_x86_feature_detected!("avx2") && std::is_x86_feature_detected!("fma")
    }
    #[cfg(not(target_arch = "x86_64"))]
    {
        false
    }
});

#[allow(dead_code)]
static HAS_NEON: LazyLock<bool> = LazyLock::new(|| {
    #[cfg(target_arch = "aarch64")]
    {
        std::arch::is_aarch64_feature_detected!("neon")
    }
    #[cfg(not(target_arch = "aarch64"))]
    {
        false
    }
});

#[inline(always)]
pub fn correlate_slices(surv: &[Complex<f32>], reference: &[Complex<f32>]) -> Complex<f32> {
    #[cfg(target_arch = "x86_64")]
    if *HAS_AVX2_FMA {
        unsafe { return correlate_slices_avx2(surv, reference); }
    }

    #[cfg(target_arch = "aarch64")]
    if *HAS_NEON {
        unsafe { return correlate_slices_neon(surv, reference); }
    }

    correlate_slices_scalar(surv, reference)
}

#[cfg(target_arch = "x86_64")]
#[target_feature(enable = "avx2,fma")]
pub unsafe fn correlate_slices_avx2(surv: &[Complex<f32>], reference: &[Complex<f32>]) -> Complex<f32> {
    use std::arch::x86_64::*;
    let len = surv.len();
    let surv_ptr = surv.as_ptr() as *const f32;
    let ref_ptr = reference.as_ptr() as *const f32;
    let float_len = len * 2;
    
    let sign_mask = _mm256_set_ps(-0.0, 0.0, -0.0, 0.0, -0.0, 0.0, -0.0, 0.0);
    
    let mut acc_re = _mm256_setzero_ps();
    let mut acc_im = _mm256_setzero_ps();
    
    let mut i = 0;
    while i + 8 <= float_len {
        let w = _mm256_loadu_ps(surv_ptr.add(i));
        let t = _mm256_loadu_ps(ref_ptr.add(i));
        
        acc_re = _mm256_fmadd_ps(w, t, acc_re);
        
        let w_shuf = _mm256_shuffle_ps(w, w, 0xB1);
        let w_shuf_sign = _mm256_xor_ps(w_shuf, sign_mask);
        acc_im = _mm256_fmadd_ps(w_shuf_sign, t, acc_im);
        
        i += 8;
    }
    
    let mut re_arr = [0.0; 8];
    _mm256_storeu_ps(re_arr.as_mut_ptr(), acc_re);
    let mut re = re_arr[0] + re_arr[1] + re_arr[2] + re_arr[3] + re_arr[4] + re_arr[5] + re_arr[6] + re_arr[7];
    
    let mut im_arr = [0.0; 8];
    _mm256_storeu_ps(im_arr.as_mut_ptr(), acc_im);
    let mut im = im_arr[0] + im_arr[1] + im_arr[2] + im_arr[3] + im_arr[4] + im_arr[5] + im_arr[6] + im_arr[7];
    
    let mut n = i / 2;
    while n < len {
        let s = surv[n];
        let r = reference[n];
        re += s.re * r.re + s.im * r.im;
        im += s.im * r.re - s.re * r.im;
        n += 1;
    }
    
    Complex::new(re, im)
}

#[cfg(target_arch = "aarch64")]
#[target_feature(enable = "neon")]
pub unsafe fn correlate_slices_neon(surv: &[Complex<f32>], reference: &[Complex<f32>]) -> Complex<f32> {
    use std::arch::aarch64::*;
    let len = surv.len();
    let surv_ptr = surv.as_ptr() as *const f32;
    let ref_ptr = reference.as_ptr() as *const f32;
    let float_len = len * 2;
    
    let mask_u32 = [0u32, 0x80000000u32, 0u32, 0x80000000u32];
    let sign_mask = vld1q_u32(mask_u32.as_ptr());
    
    let mut acc_re = vdupq_n_f32(0.0);
    let mut acc_im = vdupq_n_f32(0.0);
    
    let mut i = 0;
    while i + 4 <= float_len {
        let w = vld1q_f32(surv_ptr.add(i));
        let t = vld1q_f32(ref_ptr.add(i));
        
        acc_re = vfmaq_f32(acc_re, w, t);
        
        let w_shuf = vrev64q_f32(w);
        let w_shuf_sign = vreinterpretq_f32_u32(veorq_u32(vreinterpretq_u32_f32(w_shuf), sign_mask));
        acc_im = vfmaq_f32(acc_im, w_shuf_sign, t);
        
        i += 4;
    }
    
    let mut re_arr = [0.0; 4];
    vst1q_f32(re_arr.as_mut_ptr(), acc_re);
    let mut re = re_arr[0] + re_arr[1] + re_arr[2] + re_arr[3];
    
    let mut im_arr = [0.0; 4];
    vst1q_f32(im_arr.as_mut_ptr(), acc_im);
    let mut im = im_arr[0] + im_arr[1] + im_arr[2] + im_arr[3];
    
    let mut n = i / 2;
    while n < len {
        let s = surv[n];
        let r = reference[n];
        re += s.re * r.re + s.im * r.im;
        im += s.im * r.re - s.re * r.im;
        n += 1;
    }
    
    Complex::new(re, im)
}

#[inline(always)]
pub fn correlate_slices_scalar(surv: &[Complex<f32>], reference: &[Complex<f32>]) -> Complex<f32> {
    let mut re = 0.0f32;
    let mut im = 0.0f32;
    for n in 0..surv.len() {
        let s = surv[n];
        let r = reference[n];
        re += s.re * r.re + s.im * r.im;
        im += s.im * r.re - s.re * r.im;
    }
    Complex::new(re, im)
}

pub struct CafEngine {
    fft_512: Arc<dyn Fft<f32>>,
    fft_1024: Arc<dyn Fft<f32>>,
    r_matrix: Vec<Vec<FftComplex<f32>>>,
    scratches: Vec<Vec<FftComplex<f32>>>,
    result: Vec<Vec<f32>>,
}

impl CafEngine {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        let fft_512 = planner.plan_fft_forward(512);
        let fft_1024 = planner.plan_fft_forward(1024);
        
        let initial_max_delay = 128;
        let num_chunks = 512;
        let scratch_len = fft_512.get_inplace_scratch_len();
        
        Self {
            fft_512,
            fft_1024,
            r_matrix: vec![vec![FftComplex::new(0.0, 0.0); num_chunks]; initial_max_delay],
            scratches: vec![vec![FftComplex::new(0.0, 0.0); scratch_len]; initial_max_delay],
            result: vec![vec![0.0f32; num_chunks]; initial_max_delay],
        }
    }

    /// Acquisition Mode (Dense): 
    /// Fast-Time/Slow-Time (Welch) method. Chunk the data into 512-sample blocks, 
    /// cross-correlate for delay, and FFT across 512 chunks for Doppler.
    pub fn compute_acquisition_dense(
        &mut self,
        clean_surv: &[Complex<f32>],
        surrogate_ref: &[Complex<f32>],
        max_delay: usize,
    ) -> Vec<Vec<f32>> {
        let chunk_size = 512;
        let num_chunks = 512;
        
        let available_chunks = ((clean_surv.len().saturating_sub(max_delay)) / chunk_size)
            .min(surrogate_ref.len().saturating_sub(max_delay) / chunk_size)
            .min(num_chunks);

        if available_chunks == 0 {
            return vec![vec![0.0; num_chunks]; max_delay];
        }

        let scratch_len = self.fft_512.get_inplace_scratch_len();

        // Dynamically grow persistent buffers if max_delay exceeds current allocation
        if self.r_matrix.len() < max_delay {
            self.r_matrix.resize(max_delay, vec![FftComplex::new(0.0, 0.0); num_chunks]);
        }
        if self.scratches.len() < max_delay {
            self.scratches.resize(max_delay, vec![FftComplex::new(0.0, 0.0); scratch_len]);
        }
        if self.result.len() < max_delay {
            self.result.resize(max_delay, vec![0.0f32; num_chunks]);
        }

        let r_slice = &mut self.r_matrix[0..max_delay];
        r_slice.par_iter_mut().enumerate().for_each(|(d, row)| {
            for m in 0..available_chunks {
                let offset = m * chunk_size + max_delay;
                let surv_chunk = &clean_surv[offset .. offset + chunk_size];
                let ref_chunk = &surrogate_ref[offset - d .. offset - d + chunk_size];
                let sum = correlate_slices(surv_chunk, ref_chunk);
                row[m] = FftComplex::new(sum.re, sum.im);
            }
            // Zero out remaining chunks
            for m in available_chunks..num_chunks {
                row[m] = FftComplex::new(0.0, 0.0);
            }
        });

        let scratches_slice = &mut self.scratches[0..max_delay];
        let result_slice = &mut self.result[0..max_delay];

        r_slice
            .par_iter_mut()
            .zip(scratches_slice.par_iter_mut())
            .zip(result_slice.par_iter_mut())
            .for_each(|((row, scratch), out_row)| {
                self.fft_512.process_with_scratch(row, scratch);
                
                // Shift FFT and compute magnitude squared
                for k in 0..num_chunks {
                    let shifted_k = (k + num_chunks / 2) % num_chunks;
                    out_row[shifted_k] = row[k].norm_sqr();
                }
            });

        // Copy out the active slice to preserve external expectations
        let mut out = vec![vec![0.0f32; num_chunks]; max_delay];
        for d in 0..max_delay {
            out[d].copy_from_slice(&result_slice[d][0..num_chunks]);
        }
        out
    }

    /// Tracking Mode (Sparse & De-spread): 
    /// Take the predicted delay. Multiply Clean_Surv by the conjugate of Surrogate_Ref delayed.
    /// Pass this element-wise product through a decimator to drop it from 256 kHz down to 8 kHz (32x).
    /// Run a 1024-point FFT to resolve precise Doppler.
    pub fn compute_tracking_sparse(
        &self,
        clean_surv: &[Complex<f32>],
        surrogate_ref: &[Complex<f32>],
        delay: usize,
    ) -> Vec<f32> {
        let decimation_factor = 32;
        let fft_size = 1024;
        let required_samples = delay + (fft_size - 1) * decimation_factor + 65;

        if clean_surv.len() < required_samples || surrogate_ref.len() < required_samples {
            return vec![0.0; fft_size];
        }

        let taps = [
            0.00000000f32, 0.00009801f32, 0.00021781f32, 0.00037666f32, 0.00059265f32, 0.00088412f32, 0.00126903f32, 0.00176431f32, 0.00238522f32, 0.00314473f32, 0.00405295f32, 0.00511665f32, 0.00633879f32, 0.00771826f32, 0.00924963f32, 0.01092304f32, 0.01272432f32, 0.01463506f32, 0.01663294f32, 0.01869218f32, 0.02078398f32, 0.02287723f32, 0.02493914f32, 0.02693603f32, 0.02883414f32, 0.03060047f32, 0.03220354f32, 0.03361424f32, 0.03480654f32, 0.03575815f32, 0.03645112f32, 0.03687226f32, 0.03701355f32, 0.03687226f32, 0.03645112f32, 0.03575815f32, 0.03480654f32, 0.03361424f32, 0.03220354f32, 0.03060047f32, 0.02883414f32, 0.02693603f32, 0.02493914f32, 0.02287723f32, 0.02078398f32, 0.01869218f32, 0.01663294f32, 0.01463506f32, 0.01272432f32, 0.01092304f32, 0.00924963f32, 0.00771826f32, 0.00633879f32, 0.00511665f32, 0.00405295f32, 0.00314473f32, 0.00238522f32, 0.00176431f32, 0.00126903f32, 0.00088412f32, 0.00059265f32, 0.00037666f32, 0.00021781f32, 0.00009801f32, 0.00000000f32
        ];

        let mut decimated = vec![FftComplex::new(0.0, 0.0); fft_size];
        
        for i in 0..fft_size {
            let mut sum = Complex::new(0.0, 0.0);
            for j in 0..65 {
                let idx = delay + i * decimation_factor + j;
                sum += clean_surv[idx] * surrogate_ref[idx - delay].conj() * taps[j];
            }
            decimated[i] = FftComplex::new(sum.re, sum.im);
        }

        let mut scratch = vec![FftComplex::new(0.0, 0.0); self.fft_1024.get_inplace_scratch_len()];
        self.fft_1024.process_with_scratch(&mut decimated, &mut scratch);

        let mut result = vec![0.0; fft_size];
        for k in 0..fft_size {
            let shifted_k = (k + fft_size / 2) % fft_size;
            result[shifted_k] = decimated[k].norm_sqr();
        }

        result
    }
}

/// Channel Impulse Response (CIR): Zero-Doppler cross-correlation between
/// surveillance and reference channels over a range of delay bins.
pub fn compute_cir(
    surv: &[Complex<f32>],
    reference: &[Complex<f32>],
    max_delay: usize,
) -> Vec<f32> {
    let block_size = 512;
    if surv.len() < block_size + max_delay || reference.len() < block_size + max_delay {
        return vec![0.0; max_delay];
    }

    let mut profile = vec![0.0f32; max_delay];
    for d in 0..max_delay {
        let surv_sub = &surv[max_delay..max_delay + block_size];
        let ref_sub = &reference[max_delay - d..max_delay - d + block_size];
        let sum = correlate_slices(surv_sub, ref_sub);
        profile[d] = sum.norm();
    }
    profile
}

pub struct FarrowInterpolator {
    pub history: [Complex<f32>; 4],
}

impl FarrowInterpolator {
    pub fn new() -> Self {
        Self {
            history: [Complex::new(0.0, 0.0); 4],
        }
    }

    pub fn push(&mut self, sample: Complex<f32>) {
        self.history[0] = self.history[1];
        self.history[1] = self.history[2];
        self.history[2] = self.history[3];
        self.history[3] = sample;
    }

    pub fn interpolate(&self, mu: f32) -> Complex<f32> {
        let y_neg1 = self.history[0];
        let y_0 = self.history[1];
        let y_1 = self.history[2];
        let y_2 = self.history[3];

        let v3_re = -1.0 / 6.0 * y_neg1.re + 0.5 * y_0.re - 0.5 * y_1.re + 1.0 / 6.0 * y_2.re;
        let v2_re = 0.5 * y_neg1.re - y_0.re + 0.5 * y_1.re;
        let v1_re = -1.0 / 3.0 * y_neg1.re - 0.5 * y_0.re + y_1.re - 1.0 / 6.0 * y_2.re;
        let v0_re = y_0.re;

        let v3_im = -1.0 / 6.0 * y_neg1.im + 0.5 * y_0.im - 0.5 * y_1.im + 1.0 / 6.0 * y_2.im;
        let v2_im = 0.5 * y_neg1.im - y_0.im + 0.5 * y_1.im;
        let v1_im = -1.0 / 3.0 * y_neg1.im - 0.5 * y_0.im + y_1.im - 1.0 / 6.0 * y_2.im;
        let v0_im = y_0.im;

        let re = ((v3_re * mu + v2_re) * mu + v1_re) * mu + v0_re;
        let im = ((v3_im * mu + v2_im) * mu + v1_im) * mu + v0_im;

        Complex::new(re, im)
    }

    pub fn reset(&mut self) {
        self.history = [Complex::new(0.0, 0.0); 4];
    }
}

pub fn correlate_fractional_delay(
    surv: &[Complex<f32>],
    reference: &[Complex<f32>],
    reference_start: f32,
) -> f32 {
    let block_size = surv.len();
    let mut sum = Complex::new(0.0, 0.0);
    let mut ref_energy = 0.0f32;
    
    for i in 0..block_size {
        let target_idx = reference_start + i as f32;
        let base = target_idx.floor() as i32;
        let mu = target_idx - base as f32;
        
        let idx_neg1 = (base - 1).clamp(0, reference.len() as i32 - 1) as usize;
        let idx_0 = base.clamp(0, reference.len() as i32 - 1) as usize;
        let idx_1 = (base + 1).clamp(0, reference.len() as i32 - 1) as usize;
        let idx_2 = (base + 2).clamp(0, reference.len() as i32 - 1) as usize;
        
        let y_neg1 = reference[idx_neg1];
        let y_0 = reference[idx_0];
        let y_1 = reference[idx_1];
        let y_2 = reference[idx_2];
        
        let v3_re = -1.0 / 6.0 * y_neg1.re + 0.5 * y_0.re - 0.5 * y_1.re + 1.0 / 6.0 * y_2.re;
        let v2_re = 0.5 * y_neg1.re - y_0.re + 0.5 * y_1.re;
        let v1_re = -1.0 / 3.0 * y_neg1.re - 0.5 * y_0.re + y_1.re - 1.0 / 6.0 * y_2.re;
        let v0_re = y_0.re;

        let v3_im = -1.0 / 6.0 * y_neg1.im + 0.5 * y_0.im - 0.5 * y_1.im + 1.0 / 6.0 * y_2.im;
        let v2_im = 0.5 * y_neg1.im - y_0.im + 0.5 * y_1.im;
        let v1_im = -1.0 / 3.0 * y_neg1.im - 0.5 * y_0.im + y_1.im - 1.0 / 6.0 * y_2.im;
        let v0_im = y_0.im;

        let re = ((v3_re * mu + v2_re) * mu + v1_re) * mu + v0_re;
        let im = ((v3_im * mu + v2_im) * mu + v1_im) * mu + v0_im;
        let interp_ref = Complex::new(re, im);
        
        sum += surv[i] * interp_ref.conj();
        ref_energy += interp_ref.norm_sqr();
    }
    
    if ref_energy > 1e-6 {
        sum.norm() / ref_energy.sqrt()
    } else {
        sum.norm()
    }
}

/// Refines an integer delay peak to sub-sample precision using Farrow interpolation.
/// Fits a parabola to the correlation magnitude at three fractional delays: d - 0.5, d, d + 0.5.
/// Returns the refined delay as a float.
pub fn refine_delay_farrow(
    surv: &[Complex<f32>],
    reference: &[Complex<f32>],
    integer_delay: usize,
) -> f32 {
    let block_size = 512;
    let max_cir_delay = 64;
    if surv.len() < block_size + max_cir_delay || reference.len() < block_size + max_cir_delay {
        return integer_delay as f32;
    }
    
    let surv_sub = &surv[max_cir_delay..max_cir_delay + block_size];
    let d_float = integer_delay as f32;
    
    let ref_start_neg = max_cir_delay as f32 - (d_float - 0.5);
    let ref_start_zero = max_cir_delay as f32 - d_float;
    let ref_start_pos = max_cir_delay as f32 - (d_float + 0.5);
    
    let m_neg = correlate_fractional_delay(surv_sub, reference, ref_start_neg);
    let m_zero = correlate_fractional_delay(surv_sub, reference, ref_start_zero);
    let m_pos = correlate_fractional_delay(surv_sub, reference, ref_start_pos);
    
    let a = 2.0 * (m_pos + m_neg - 2.0 * m_zero);
    let b = m_pos - m_neg;
    
    if a >= 0.0 || a.abs() < 1e-6 {
        return integer_delay as f32;
    }
    
    let x_peak = -b / (2.0 * a);
    let x_peak_clamped = x_peak.clamp(-0.5, 0.5);
    
    integer_delay as f32 + x_peak_clamped
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_compute_cir() {
        let mut reference = vec![Complex::new(0.0, 0.0); 1000];
        let mut surv = vec![Complex::new(0.0, 0.0); 1000];
        
        // Generate pseudo-noise reference signal using a recursive LCG
        let mut seed: u64 = 12345;
        for i in 0..1000 {
            seed = (seed.wrapping_mul(1103515245).wrapping_add(12345)) & 0x7fffffff;
            let phase = (seed as f32 / 2147483647.0) * 2.0 * std::f32::consts::PI;
            reference[i] = Complex::new(phase.cos(), phase.sin());
        }
        
        println!("First 5 reference values: {:?}", &reference[0..5]);
        
        // Inject direct path and delayed echo
        // Echo delayed by 5 bins, with 0.5 amplitude
        for i in 0..1000 {
            surv[i] = reference[i]; // Direct path
            if i >= 5 {
                surv[i] += reference[i - 5] * 0.5; // Delayed path
            }
        }
        
        let profile = compute_cir(&surv, &reference, 20);
        println!("CIR Profile: {:?}", profile);
        
        // Compare with scalar correlation directly for d = 0 and d = 1
        let surv_sub = &surv[20..20 + 512];
        let ref_sub_0 = &reference[20..20 + 512];
        let ref_sub_1 = &reference[19..19 + 512];
        let sum_0 = correlate_slices_scalar(surv_sub, ref_sub_0);
        let sum_1 = correlate_slices_scalar(surv_sub, ref_sub_1);
        println!("Scalar sum d=0: {}, d=1: {}", sum_0.norm(), sum_1.norm());
        
        assert_eq!(profile.len(), 20);
        
        // The peak at delay 0 should be the largest (direct path correlation)
        assert!(profile[0] > profile[1], "profile[0] ({}) should be > profile[1] ({})", profile[0], profile[1]);
        
        // There should be a secondary local peak at delay 5 (echo correlation)
        assert!(profile[5] > profile[4], "profile[5] ({}) should be > profile[4] ({})", profile[5], profile[4]);
        assert!(profile[5] > profile[6], "profile[5] ({}) should be > profile[6] ({})", profile[5], profile[6]);
        assert!(profile[5] > 0.1 * profile[0], "profile[5] ({}) should be > 0.1 * profile[0] ({})", profile[5], profile[0]);
    }

    #[test]
    fn test_farrow_interpolator_cubic_lagrange() {
        let mut farrow = FarrowInterpolator::new();
        farrow.push(Complex::new(0.0, 0.0));
        farrow.push(Complex::new(1.0, 0.0));
        farrow.push(Complex::new(2.0, 0.0));
        farrow.push(Complex::new(3.0, 0.0));

        let val_zero = farrow.interpolate(0.0);
        let val_mid = farrow.interpolate(0.5);
        let val_one = farrow.interpolate(1.0);

        assert!((val_zero.re - 1.0).abs() < 1e-5);
        assert!((val_mid.re - 1.5).abs() < 1e-5);
        assert!((val_one.re - 2.0).abs() < 1e-5);
    }

    #[test]
    fn test_refine_delay_farrow() {
        let mut reference = vec![Complex::new(0.0, 0.0); 1000];
        let mut surv = vec![Complex::new(0.0, 0.0); 1000];
        
        let mut raw_ref = vec![Complex::new(0.0, 0.0); 1100];
        let mut seed: u64 = 54321;
        for i in 0..1100 {
            seed = (seed.wrapping_mul(1103515245).wrapping_add(12345)) & 0x7fffffff;
            let phase = (seed as f32 / 2147483647.0) * 2.0 * std::f32::consts::PI;
            raw_ref[i] = Complex::new(phase.cos(), phase.sin());
        }

        // Apply a moving average filter to band-limit the reference signal
        for i in 0..1000 {
            let mut sum = Complex::new(0.0, 0.0);
            for k in 0..8 {
                sum += raw_ref[i + k];
            }
            reference[i] = sum / 8.0;
        }

        for i in 0..1000 {
            let target_idx = i as f32 - 5.25;
            let base = target_idx.floor() as i32;
            let mu = target_idx - base as f32;
            
            let idx_neg1 = (base - 1).clamp(0, 999) as usize;
            let idx_0 = base.clamp(0, 999) as usize;
            let idx_1 = (base + 1).clamp(0, 999) as usize;
            let idx_2 = (base + 2).clamp(0, 999) as usize;
            
            let y_neg1 = reference[idx_neg1];
            let y_0 = reference[idx_0];
            let y_1 = reference[idx_1];
            let y_2 = reference[idx_2];
            
            let v3_re = -1.0 / 6.0 * y_neg1.re + 0.5 * y_0.re - 0.5 * y_1.re + 1.0 / 6.0 * y_2.re;
            let v2_re = 0.5 * y_neg1.re - y_0.re + 0.5 * y_1.re;
            let v1_re = -1.0 / 3.0 * y_neg1.re - 0.5 * y_0.re + y_1.re - 1.0 / 6.0 * y_2.re;
            let v0_re = y_0.re;

            let v3_im = -1.0 / 6.0 * y_neg1.im + 0.5 * y_0.im - 0.5 * y_1.im + 1.0 / 6.0 * y_2.im;
            let v2_im = 0.5 * y_neg1.im - y_0.im + 0.5 * y_1.im;
            let v1_im = -1.0 / 3.0 * y_neg1.im - 0.5 * y_0.im + y_1.im - 1.0 / 6.0 * y_2.im;
            let v0_im = y_0.im;

            let re = ((v3_re * mu + v2_re) * mu + v1_re) * mu + v0_re;
            let im = ((v3_im * mu + v2_im) * mu + v1_im) * mu + v0_im;
            
            surv[i] = Complex::new(re, im);
        }

        let refined = refine_delay_farrow(&surv, &reference, 5);
        println!("test_refine_delay_farrow diagnostics: refined={}", refined);
        assert!((refined - 5.25).abs() < 0.05, "Refined delay {} should be close to 5.25", refined);
    }

    #[test]
    fn test_correlate_slices_equivalence() {
        let surv = vec![
            Complex::new(1.0, 2.0),
            Complex::new(-3.0, 4.0),
            Complex::new(0.5, -1.5),
            Complex::new(-2.2, -3.3),
            Complex::new(4.4, 5.5),
            Complex::new(-1.1, 0.1),
            Complex::new(0.0, 0.0),
            Complex::new(3.0, -2.0),
            Complex::new(1.2, 3.4),
        ];
        let reference = vec![
            Complex::new(0.5, -0.5),
            Complex::new(2.0, 1.0),
            Complex::new(-1.0, 0.0),
            Complex::new(3.0, 2.0),
            Complex::new(-2.0, -1.0),
            Complex::new(0.1, -0.2),
            Complex::new(1.5, 2.5),
            Complex::new(-3.0, 4.0),
            Complex::new(2.1, -1.2),
        ];

        let scalar_res = correlate_slices_scalar(&surv, &reference);
        let simd_res = correlate_slices(&surv, &reference);

        assert!((scalar_res.re - simd_res.re).abs() < 1e-5, "Real parts differ: scalar={}, simd={}", scalar_res.re, simd_res.re);
        assert!((scalar_res.im - simd_res.im).abs() < 1e-5, "Imag parts differ: scalar={}, simd={}", scalar_res.im, simd_res.im);
    }
}
