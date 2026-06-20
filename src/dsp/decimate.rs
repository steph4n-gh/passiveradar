use num_complex::Complex;
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

/// A standard FIR filter with an efficient circular buffer structure.
pub struct FirFilter {
    taps_simd: Vec<f32>,
    pub num_taps: usize,
}

impl FirFilter {
    /// Create a new FIR filter with specified coefficients (taps).
    pub fn new(taps: Vec<f32>) -> Self {
        let num_taps = taps.len();
        // Reverse taps and duplicate [t0, t0, t1, t1, ...] for SIMD element-wise processing
        let mut taps_simd = Vec::with_capacity(num_taps * 2);
        for &t in taps.iter().rev() {
            taps_simd.push(t);
            taps_simd.push(t);
        }
        Self {
            taps_simd,
            num_taps,
        }
    }

    /// Design a windowed sinc low-pass FIR filter.
    pub fn design_lowpass(cutoff_norm: f32, num_taps: usize) -> Self {
        // Ensure odd number of taps to have an integer group delay
        let num_taps = if num_taps % 2 == 0 {
            num_taps + 1
        } else {
            num_taps
        };
        let mut taps = vec![0.0f32; num_taps];
        let mid = (num_taps / 2) as f32;

        for i in 0..num_taps {
            let n = (i as f32) - mid;
            let sinc_val = if n == 0.0 {
                2.0 * cutoff_norm
            } else {
                ((2.0 * std::f32::consts::PI * cutoff_norm * n).sin()) / (std::f32::consts::PI * n)
            };

            // Hamming window
            let w = 0.54
                - 0.46 * (2.0 * std::f32::consts::PI * (i as f32) / ((num_taps - 1) as f32)).cos();
            taps[i] = sinc_val * w;
        }

        // Normalize taps to ensure unit gain at DC
        let sum: f32 = taps.iter().sum();
        if sum.abs() > 1e-6 {
            for t in &mut taps {
                *t /= sum;
            }
        }

        Self::new(taps)
    }

    /// Compute the filter output for a window of samples.
    #[inline(always)]
    pub fn compute(&self, window: &[Complex<f32>]) -> Complex<f32> {
        assert!(window.len() >= self.num_taps);

        #[cfg(target_arch = "x86_64")]
        if *HAS_AVX2_FMA {
            return unsafe { self.compute_x86_64(window) };
        }

        #[cfg(target_arch = "aarch64")]
        if *HAS_NEON {
            return unsafe { self.compute_aarch64(window) };
        }

        self.compute_scalar(window)
    }

    #[inline(always)]
    pub fn compute_scalar(&self, window: &[Complex<f32>]) -> Complex<f32> {
        assert!(window.len() >= self.num_taps);
        let mut re = 0.0;
        let mut im = 0.0;
        let window_ptr = window.as_ptr() as *const f32;
        let taps_ptr = self.taps_simd.as_ptr();
        let len = self.taps_simd.len();
        
        let mut i = 0;
        while i < len {
            unsafe {
                re += *window_ptr.add(i) * *taps_ptr.add(i);
                im += *window_ptr.add(i + 1) * *taps_ptr.add(i + 1);
            }
            i += 2;
        }
        Complex::new(re, im)
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn compute_x86_64(&self, window: &[Complex<f32>]) -> Complex<f32> {
        assert!(window.len() >= self.num_taps);
        use std::arch::x86_64::*;
        let window_ptr = window.as_ptr() as *const f32;
        let taps_ptr = self.taps_simd.as_ptr();
        let len = self.taps_simd.len();
        
        let mut sum = _mm256_setzero_ps();
        let mut i = 0;
        while i + 8 <= len {
            let w = _mm256_loadu_ps(window_ptr.add(i));
            let t = _mm256_loadu_ps(taps_ptr.add(i));
            sum = _mm256_fmadd_ps(w, t, sum);
            i += 8;
        }
        
        let mut sum_arr = [0.0; 8];
        _mm256_storeu_ps(sum_arr.as_mut_ptr(), sum);
        
        let mut re = sum_arr[0] + sum_arr[2] + sum_arr[4] + sum_arr[6];
        let mut im = sum_arr[1] + sum_arr[3] + sum_arr[5] + sum_arr[7];
        
        while i < len {
            re += *window_ptr.add(i) * *taps_ptr.add(i);
            im += *window_ptr.add(i + 1) * *taps_ptr.add(i + 1);
            i += 2;
        }
        
        Complex::new(re, im)
    }

    #[cfg(target_arch = "aarch64")]
    #[target_feature(enable = "neon")]
    unsafe fn compute_aarch64(&self, window: &[Complex<f32>]) -> Complex<f32> {
        assert!(window.len() >= self.num_taps);
        use std::arch::aarch64::*;
        let window_ptr = window.as_ptr() as *const f32;
        let taps_ptr = self.taps_simd.as_ptr();
        let len = self.taps_simd.len();
        
        let mut sum = vdupq_n_f32(0.0);
        let mut i = 0;
        while i + 4 <= len {
            let w = vld1q_f32(window_ptr.add(i));
            let t = vld1q_f32(taps_ptr.add(i));
            sum = vfmaq_f32(sum, w, t);
            i += 4;
        }
        
        let mut sum_arr = [0.0; 4];
        vst1q_f32(sum_arr.as_mut_ptr(), sum);
        
        let mut re = sum_arr[0] + sum_arr[2];
        let mut im = sum_arr[1] + sum_arr[3];
        
        while i < len {
            re += *window_ptr.add(i) * *taps_ptr.add(i);
            im += *window_ptr.add(i + 1) * *taps_ptr.add(i + 1);
            i += 2;
        }
        
        Complex::new(re, im)
    }
}

// =========================================================================
// Multi-Stage Decimator
// =========================================================================
pub struct DecimatorStage {
    filter: FirFilter,
    decimation_factor: usize,
    counter: usize,
    buffer: Vec<Complex<f32>>,
}

impl DecimatorStage {
    pub fn new(decimation_factor: usize, cutoff_norm: f32, num_taps: usize) -> Self {
        let filter = FirFilter::design_lowpass(cutoff_norm, num_taps);
        let actual_taps = filter.num_taps;
        Self {
            filter,
            decimation_factor,
            counter: 0,
            buffer: vec![Complex::new(0.0, 0.0); actual_taps - 1],
        }
    }

    /// Process input samples and write decimated output samples to output buffer.
    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        self.buffer.extend_from_slice(input);
        
        #[cfg(target_arch = "x86_64")]
        if *HAS_AVX2_FMA {
            unsafe { self.process_block_avx(output) };
            return;
        }

        #[cfg(target_arch = "aarch64")]
        if *HAS_NEON {
            unsafe { self.process_block_neon(output) };
            return;
        }

        self.process_block_scalar(output);
    }

    pub fn process_block_scalar_wrapper(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        self.buffer.extend_from_slice(input);
        self.process_block_scalar(output);
    }

    #[inline(always)]
    pub fn process_block_scalar(&mut self, output: &mut Vec<Complex<f32>>) {
        let mut i = self.counter;
        let num_taps = self.filter.num_taps;
        
        while i + num_taps <= self.buffer.len() {
            output.push(self.filter.compute_scalar(&self.buffer[i .. i + num_taps]));
            i += self.decimation_factor;
        }
        
        let dropped = i.min(self.buffer.len());
        let remaining = self.buffer.len() - dropped;
        unsafe {
            let ptr = self.buffer.as_mut_ptr();
            std::ptr::copy(ptr.add(dropped), ptr, remaining);
        }
        self.buffer.truncate(remaining);
        self.counter = i - dropped;
    }

    #[cfg(target_arch = "x86_64")]
    #[target_feature(enable = "avx2,fma")]
    unsafe fn process_block_avx(&mut self, output: &mut Vec<Complex<f32>>) {
        let mut i = self.counter;
        let num_taps = self.filter.num_taps;
        
        while i + num_taps <= self.buffer.len() {
            output.push(self.filter.compute_x86_64(&self.buffer[i .. i + num_taps]));
            i += self.decimation_factor;
        }
        
        let dropped = i.min(self.buffer.len());
        let remaining = self.buffer.len() - dropped;
        let ptr = self.buffer.as_mut_ptr();
        std::ptr::copy(ptr.add(dropped), ptr, remaining);
        self.buffer.truncate(remaining);
        self.counter = i - dropped;
    }

    #[cfg(target_arch = "aarch64")]
    #[target_feature(enable = "neon")]
    unsafe fn process_block_neon(&mut self, output: &mut Vec<Complex<f32>>) {
        let mut i = self.counter;
        let num_taps = self.filter.num_taps;
        
        while i + num_taps <= self.buffer.len() {
            output.push(self.filter.compute_aarch64(&self.buffer[i .. i + num_taps]));
            i += self.decimation_factor;
        }
        
        let dropped = i.min(self.buffer.len());
        let remaining = self.buffer.len() - dropped;
        let ptr = self.buffer.as_mut_ptr();
        std::ptr::copy(ptr.add(dropped), ptr, remaining);
        self.buffer.truncate(remaining);
        self.counter = i - dropped;
    }
}

/// A pipeline that decimates high-rate IQ data down to low narrowband rates in multiple stages.
pub struct MultiStageDecimator {
    stages: Vec<DecimatorStage>,
    buf1: Vec<Complex<f32>>,
    buf2: Vec<Complex<f32>>,
}

impl MultiStageDecimator {
    /// Create a decimator targeting a decimation factor of 256 (e.g. 2.048 MSPS -> 8 kHz).
    /// Decimation path: 2048 kHz --(/8)--> 256 kHz --(/8)--> 32 kHz --(/4)--> 8 kHz.
    pub fn new_256x() -> Self {
        // Stage 1: factor = 8, cutoff = 0.05, taps = 31
        let s1 = DecimatorStage::new(8, 0.05, 31);
        // Stage 2: factor = 8, cutoff = 0.05, taps = 63
        let s2 = DecimatorStage::new(8, 0.05, 63);
        // Stage 3: factor = 4, cutoff = 0.1, taps = 63
        let s3 = DecimatorStage::new(4, 0.1, 63);

        Self {
            stages: vec![s1, s2, s3],
            buf1: Vec::new(),
            buf2: Vec::new(),
        }
    }

    /// Create a decimator targeting a decimation factor of 8 (e.g. 2.048 MSPS -> 256 kHz).
    pub fn new_8x() -> Self {
        // Stage 1: factor = 8, cutoff = 0.05, taps = 63
        let s1 = DecimatorStage::new(8, 0.05, 63);

        Self {
            stages: vec![s1],
            buf1: Vec::new(),
            buf2: Vec::new(),
        }
    }

    /// Process a block of samples through all decimation stages.
    pub fn process(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        if self.stages.is_empty() {
            output.clear();
            output.extend_from_slice(input);
            return;
        }

        // Temporarily extract buffers to bypass borrow checker
        let mut buf1 = std::mem::take(&mut self.buf1);
        let mut buf2 = std::mem::take(&mut self.buf2);

        buf1.clear();
        buf2.clear();
        output.clear();

        let num_stages = self.stages.len();
        if num_stages == 1 {
            self.stages[0].process_block(input, output);
        } else if num_stages == 2 {
            self.stages[0].process_block(input, &mut buf1);
            self.stages[1].process_block(&buf1, output);
        } else {
            // Assume 3 stages (for 256x)
            self.stages[0].process_block(input, &mut buf1);
            self.stages[1].process_block(&buf1, &mut buf2);
            self.stages[2].process_block(&buf2, output);
        }

        // Put buffers back
        self.buf1 = buf1;
        self.buf2 = buf2;
    }
}

// =========================================================================
// Digital Down Converter (DDC)
// =========================================================================
pub struct DigitalDownConverter {
    phase: f32,
    phase_step: f32,
    decimator: MultiStageDecimator,
    mixed_buf: Vec<Complex<f32>>,
}

impl DigitalDownConverter {
    pub fn new(offset_frequency: f64, sample_rate: f64) -> Self {
        let phase_step = (2.0 * std::f64::consts::PI * offset_frequency / sample_rate) as f32;
        Self {
            phase: 0.0,
            phase_step,
            decimator: MultiStageDecimator::new_256x(),
            mixed_buf: Vec::new(),
        }
    }

    pub fn new_8x(offset_frequency: f64, sample_rate: f64) -> Self {
        let phase_step = (2.0 * std::f64::consts::PI * offset_frequency / sample_rate) as f32;
        Self {
            phase: 0.0,
            phase_step,
            decimator: MultiStageDecimator::new_8x(),
            mixed_buf: Vec::new(),
        }
    }

    pub fn update_offset(&mut self, offset_frequency: f64, sample_rate: f64) {
        self.phase_step = (2.0 * std::f64::consts::PI * offset_frequency / sample_rate) as f32;
    }

    pub fn process_block(&mut self, input: &[Complex<f32>], output: &mut Vec<Complex<f32>>) {
        // Reuse pre-allocated buffer with pre-resized length to avoid push allocation overhead
        self.mixed_buf.resize(input.len(), Complex::new(0.0, 0.0));

        // Rotating phasor oscillator: compute one sin/cos for the step,
        // then multiply forward with complex rotation per sample.
        // This replaces 2 trig calls per sample with 4 float muls + 2 float adds.
        let (sin_step, cos_step) = self.phase_step.sin_cos();
        let rotation = Complex::new(cos_step, -sin_step);
        let mut carrier = Complex::from_polar(1.0, -self.phase);

        for i in 0..input.len() {
            self.mixed_buf[i] = input[i] * carrier;
            carrier = carrier * rotation;

            // Renormalize every 1024 samples to prevent magnitude drift
            // from accumulated floating-point error
            if (i & 0x3FF) == 0x3FF {
                let norm = carrier.norm();
                if norm > 0.0 {
                    carrier = carrier / norm;
                }
            }
        }

        // Extract phase directly from the exact complex state of carrier to prevent accumulator drift
        let mut next_phase = -carrier.im.atan2(carrier.re);
        if next_phase < 0.0 {
            next_phase += 2.0 * std::f32::consts::PI;
        }
        self.phase = next_phase;

        // Decimate down to narrowband rate
        self.decimator.process(&self.mixed_buf, output);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_scalar_baseline() {
        let mut filter = DecimatorStage::new(1, 0.1, 7);
        let input: Vec<Complex<f32>> = (0..20)
            .map(|i| Complex::new(i as f32, (i * 2) as f32))
            .collect();
        
        let mut output = Vec::new();
        filter.process_block(&input, &mut output);

        let expected = vec![
            Complex::new(0.0000000, 0.0000000), Complex::new(0.0134969, 0.0269939),
            Complex::new(0.1054447, 0.2108895), Complex::new(0.4382551, 0.8765101),
            Complex::new(1.1054449, 2.2108898), Complex::new(2.0134971, 4.0269942),
            Complex::new(3.0000005, 6.0000010), Complex::new(4.0000005, 8.0000010),
            Complex::new(5.0000005, 10.0000010), Complex::new(6.0000000, 12.0000000),
            Complex::new(7.0000005, 14.0000010), Complex::new(8.0000010, 16.0000019),
            Complex::new(9.0000010, 18.0000019), Complex::new(10.0000010, 20.0000019),
            Complex::new(11.0000010, 22.0000019), Complex::new(12.0000010, 24.0000019),
            Complex::new(13.0000010, 26.0000019), Complex::new(14.0000019, 28.0000038),
            Complex::new(15.0000010, 30.0000019), Complex::new(16.0000019, 32.0000038),
        ];

        for (i, (out, exp)) in output.iter().zip(expected.iter()).enumerate() {
            assert!(
                (out.re - exp.re).abs() < 1e-5 && (out.im - exp.im).abs() < 1e-5,
                "Mismatch at {}: out={:?}, exp={:?}", i, out, exp
            );
        }
    }
    #[test]
    fn test_multi_stage_decimator_baseline() {
        let mut decimator = MultiStageDecimator::new_256x();
        let input: Vec<Complex<f32>> = (0..2000)
            .map(|i| Complex::new(i as f32, (i * 2) as f32))
            .collect();
        let mut output = Vec::new();
        decimator.process(&input, &mut output);

        let expected = vec![
            Complex::new(0.0000000, 0.0000000), Complex::new(0.0022211, 0.0044423),
            Complex::new(-0.0152263, -0.0304527), Complex::new(-0.7343102, -1.4686204),
            Complex::new(0.4418141, 0.8836282), Complex::new(-1.3286678, -2.6573355),
            Complex::new(-1.6011047, -3.2022095), Complex::new(3.3713717, 6.7427435),
        ];

        for (i, (out, exp)) in output.iter().zip(expected.iter()).enumerate() {
            assert!(
                (out.re - exp.re).abs() < 1e-5 && (out.im - exp.im).abs() < 1e-5,
                "Mismatch at {}: out={:?}, exp={:?}", i, out, exp
            );
        }
    }

    #[test]
    fn test_ddc_update_offset() {
        let mut ddc = DigitalDownConverter::new(75.0, 2.048e6);
        assert!((ddc.phase_step - (2.0 * std::f64::consts::PI * 75.0 / 2.048e6) as f32).abs() < 1e-6);
        ddc.update_offset(150.0, 2.048e6);
        assert!((ddc.phase_step - (2.0 * std::f64::consts::PI * 150.0 / 2.048e6) as f32).abs() < 1e-6);
    }
}

#[cfg(test)]
mod empirical_tests {
    use super::*;

    #[test]
    fn test_simd_vs_scalar_oracle() {
        // Simple LCG random generator to avoid pulling in rand if not available in this scope
        let mut seed: u32 = 123456789;
        let mut rand_f32 = || -> f32 {
            seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
            (seed as f32) / (u32::MAX as f32) * 2.0 - 1.0
        };

        for num_taps in [1, 2, 3, 4, 5, 7, 8, 15, 16, 31, 32, 63, 64] {
            let taps: Vec<f32> = (0..num_taps).map(|_| rand_f32()).collect();
            let filter = FirFilter::new(taps);

            let window: Vec<Complex<f32>> = (0..num_taps)
                .map(|_| Complex::new(rand_f32(), rand_f32()))
                .collect();

            let expected = filter.compute_scalar(&window);
            
            #[cfg(target_arch = "aarch64")]
            if std::arch::is_aarch64_feature_detected!("neon") {
                let actual = unsafe { filter.compute_aarch64(&window) };
                let diff_re = (expected.re - actual.re).abs();
                let diff_im = (expected.im - actual.im).abs();
                assert!(
                    diff_re < 1e-4 && diff_im < 1e-4,
                    "Mismatch AArch64 for num_taps={}:\nexpected={:?}\nactual={:?}",
                    num_taps, expected, actual
                );
            }

            #[cfg(target_arch = "x86_64")]
            if std::is_x86_feature_detected!("avx2") && std::is_x86_feature_detected!("fma") {
                let actual = unsafe { filter.compute_x86_64(&window) };
                let diff_re = (expected.re - actual.re).abs();
                let diff_im = (expected.im - actual.im).abs();
                assert!(
                    diff_re < 1e-4 && diff_im < 1e-4,
                    "Mismatch X86_64 for num_taps={}:\nexpected={:?}\nactual={:?}",
                    num_taps, expected, actual
                );
            }
        }
    }
}

#[cfg(test)]
mod performance_tests {
    use super::*;
    use std::time::Instant;

    #[test]
    fn test_simd_performance() {
        let num_taps = 63;
        let filter = FirFilter::design_lowpass(0.1, num_taps);
        let window: Vec<Complex<f32>> = (0..num_taps)
            .map(|i| Complex::new(i as f32, i as f32))
            .collect();

        let iters = 1_000_000;
        
        let mut sink = Complex::new(0.0, 0.0);

        let start = Instant::now();
        for _ in 0..iters {
            sink += filter.compute_scalar(&window);
        }
        let scalar_time = start.elapsed();

        let mut sink_simd = Complex::new(0.0, 0.0);
        let start_simd = Instant::now();
        #[cfg(target_arch = "aarch64")]
        if std::arch::is_aarch64_feature_detected!("neon") {
            for _ in 0..iters {
                sink_simd += unsafe { filter.compute_aarch64(&window) };
            }
        }
        #[cfg(target_arch = "x86_64")]
        if std::is_x86_feature_detected!("avx2") && std::is_x86_feature_detected!("fma") {
            for _ in 0..iters {
                sink_simd += unsafe { filter.compute_x86_64(&window) };
            }
        }
        let simd_time = start_simd.elapsed();

        println!("Scalar time: {:?}", scalar_time);
        println!("SIMD time: {:?}", simd_time);
        println!("Sink scalar: {:?}, Sink SIMD: {:?}", sink, sink_simd);
    }
}
