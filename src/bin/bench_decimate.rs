use num_complex::Complex;
use passiveradar::dsp::decimate::DecimatorStage;
use std::time::Instant;

fn main() {
    let num_taps = 63;
    let cutoff_norm = 0.1;
    let decimation_factor = 4;
    
    // We'll pre-generate a bunch of random data to avoid measuring RNG
    let input_size = 2048;
    let input: Vec<Complex<f32>> = (0..input_size)
        .map(|i| Complex::new(
            ((i % 100) as f32) * 0.01,
            ((i % 73) as f32) * 0.01
        ))
        .collect();

    // CORRECTNESS TEST
    let mut stage_sc = DecimatorStage::new(decimation_factor, cutoff_norm, num_taps);
    let mut stage_si = DecimatorStage::new(decimation_factor, cutoff_norm, num_taps);
    
    let mut verify_sc = Vec::new();
    let mut verify_si = Vec::new();
    
    // Process several blocks to verify state persists correctly
    for _ in 0..5 {
        stage_sc.process_block_scalar_wrapper(&input, &mut verify_sc);
        stage_si.process_block(&input, &mut verify_si);
    }
    
    assert_eq!(verify_sc.len(), verify_si.len(), "Length mismatch!");
    
    let mut max_diff = 0.0f32;
    for (i, (sc, si)) in verify_sc.iter().zip(verify_si.iter()).enumerate() {
        let diff_re = (sc.re - si.re).abs();
        let diff_im = (sc.im - si.im).abs();
        max_diff = max_diff.max(diff_re).max(diff_im);
        if diff_re > 1e-4 || diff_im > 1e-4 {
            panic!("Mismatch at index {}: scalar={:?}, simd={:?}", i, sc, si);
        }
    }
    println!("Correctness passed. Max diff: {:.2e}", max_diff);

    // PERFORMANCE TEST
    let iters = 100_000;

    let mut out_scalar = Vec::with_capacity(input_size / decimation_factor + 10);
    let mut stage_scalar = DecimatorStage::new(decimation_factor, cutoff_norm, num_taps);
    let start = Instant::now();
    for _ in 0..iters {
        out_scalar.clear();
        stage_scalar.process_block_scalar_wrapper(&input, &mut out_scalar);
    }
    let scalar_time = start.elapsed();

    let mut out_simd = Vec::with_capacity(input_size / decimation_factor + 10);
    let mut stage_simd = DecimatorStage::new(decimation_factor, cutoff_norm, num_taps);
    let start = Instant::now();
    for _ in 0..iters {
        out_simd.clear();
        stage_simd.process_block(&input, &mut out_simd);
    }
    let simd_time = start.elapsed();

    println!("Scalar time: {:?}", scalar_time);
    println!("SIMD time:   {:?}", simd_time);
    
    if simd_time < scalar_time {
        println!("SIMD is {:.2}x FASTER than scalar", scalar_time.as_secs_f64() / simd_time.as_secs_f64());
    } else {
        println!("SIMD is {:.2}x SLOWER than scalar", simd_time.as_secs_f64() / scalar_time.as_secs_f64());
    }
}
