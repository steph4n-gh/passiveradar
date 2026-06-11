use passiveradar::dsp::decimate::DecimatorStage;
use num_complex::Complex;
use std::time::Instant;

#[test]
fn challenge_simd_performance() {
    let decimation_factor = 4;
    let cutoff = 0.1;
    let num_taps = 63;
    
    let mut stage_simd = DecimatorStage::new(decimation_factor, cutoff, num_taps);

    let num_samples = 10_000_000;
    let input: Vec<Complex<f32>> = (0..num_samples)
        .map(|i| Complex::new((i % 100) as f32, (i % 50) as f32))
        .collect();

    let mut out_simd = Vec::with_capacity(num_samples / decimation_factor + 100);
    
    let start_simd = Instant::now();
    stage_simd.process_block(&input, &mut out_simd);
    let elapsed_simd = start_simd.elapsed();

    println!("SIMD block elapsed for 10M samples: {:?}", elapsed_simd);
    
    // Now scalar manual
    let filter = passiveradar::dsp::decimate::FirFilter::design_lowpass(cutoff, num_taps);
    let mut buffer = input.clone();
    let mut out_scalar = Vec::with_capacity(num_samples / decimation_factor + 100);
    
    let start_scalar = Instant::now();
    let mut i = 0;
    while i + num_taps <= buffer.len() {
        let win = &buffer[i .. i + num_taps];
        // compute_scalar equivalent
        let mut re = 0.0;
        let mut im = 0.0;
        for k in 0..num_taps {
            // wait, we don't have access to the exact taps via public API, 
            // but we can just use some dummy taps to simulate scalar FIR math workload
            re += win[k].re * 0.1;
            im += win[k].im * 0.1;
        }
        out_scalar.push(Complex::new(re, im));
        i += decimation_factor;
    }
    let elapsed_scalar = start_scalar.elapsed();

    println!("Scalar dummy block elapsed for 10M samples: {:?}", elapsed_scalar);
    
    #[cfg(not(debug_assertions))]
    assert!(
        elapsed_simd < elapsed_scalar,
        "SIMD {:?} is NOT faster than scalar {:?}!",
        elapsed_simd, elapsed_scalar
    );

    #[cfg(debug_assertions)]
    println!(
        "Skipping performance assertion in debug mode. SIMD: {:?}, Scalar: {:?}",
        elapsed_simd, elapsed_scalar
    );
}
