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
}

impl CafEngine {
    pub fn new() -> Self {
        let mut planner = FftPlanner::new();
        Self {
            fft_512: planner.plan_fft_forward(512),
            fft_1024: planner.plan_fft_forward(1024),
        }
    }

    /// Acquisition Mode (Dense): 
    /// Fast-Time/Slow-Time (Welch) method. Chunk the data into 512-sample blocks, 
    /// cross-correlate for delay, and FFT across 512 chunks for Doppler.
    pub fn compute_acquisition_dense(
        &self,
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

        // 1. Pre-allocate r_matrix sequentially on the main thread to avoid global allocator contention in parallel threads
        let mut r_matrix = vec![vec![FftComplex::new(0.0, 0.0); num_chunks]; max_delay];

        // Cross-correlate in parallel across delays (Fast-Time)
        r_matrix.par_iter_mut().enumerate().for_each(|(d, row)| {
            for m in 0..available_chunks {
                let offset = m * chunk_size + max_delay;
                let surv_chunk = &clean_surv[offset .. offset + chunk_size];
                let ref_chunk = &surrogate_ref[offset - d .. offset - d + chunk_size];
                let sum = correlate_slices(surv_chunk, ref_chunk);
                row[m] = FftComplex::new(sum.re, sum.im);
            }
        });

        // 2. Pre-allocate results and scratch buffers sequentially on the main thread
        let scratch_len = self.fft_512.get_inplace_scratch_len();
        let mut scratches = vec![vec![FftComplex::new(0.0, 0.0); scratch_len]; max_delay];
        let mut result = vec![vec![0.0f32; num_chunks]; max_delay];

        // FFT across chunks in parallel (Slow-Time)
        r_matrix
            .par_iter_mut()
            .zip(scratches.par_iter_mut())
            .zip(result.par_iter_mut())
            .for_each(|((row, scratch), out_row)| {
                self.fft_512.process_with_scratch(row, scratch);
                
                // Shift FFT and compute magnitude squared
                for k in 0..num_chunks {
                    let shifted_k = (k + num_chunks / 2) % num_chunks;
                    out_row[shifted_k] = row[k].norm_sqr();
                }
            });

        result
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

#[cfg(test)]
mod tests {
    use super::*;

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
