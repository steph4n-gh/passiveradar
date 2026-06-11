use super::*;
use std::time::Instant;

pub fn run_bench() {
    let num_taps = 63;
    let filter = FirFilter::design_lowpass(0.1, num_taps);
    let window: Vec<Complex<f32>> = (0..num_taps)
        .map(|i| Complex::new(i as f32, i as f32))
        .collect();

    let iters = 1_000_000;
    
    // Scalar bench
    let start = Instant::now();
    for _ in 0..iters {
        let _ = filter.compute_scalar_bench(&window);
    }
    let scalar_time = start.elapsed();
    
    // SIMD bench
    let start = Instant::now();
    for _ in 0..iters {
        #[cfg(target_arch = "aarch64")]
        unsafe {
            let _ = filter.compute_aarch64_bench(&window);
        }
    }
    let simd_time = start.elapsed();
    
    println!("Scalar time: {:?}", scalar_time);
    println!("SIMD time: {:?}", simd_time);
}
