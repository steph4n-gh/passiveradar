use passiveradar::dsp::decimate::FirFilter;
use num_complex::Complex;

fn main() {
    let mut seed: u32 = 123456789;
    let mut rand_f32 = || -> f32 {
        seed = seed.wrapping_mul(1664525).wrapping_add(1013904223);
        (seed as f32) / (u32::MAX as f32) * 2.0 - 1.0
    };

    println!("Starting empirical verification of SIMD FirFilter...");

    for num_taps in 1..=256 {
        let taps: Vec<f32> = (0..num_taps).map(|_| rand_f32()).collect();
        let filter = FirFilter::new(taps);

        // Test with different window sizes >= num_taps
        for window_len in num_taps..=(num_taps + 16) {
            let window: Vec<Complex<f32>> = (0..window_len)
                .map(|_| Complex::new(rand_f32(), rand_f32()))
                .collect();

            let expected = filter.compute_scalar(&window);
            
            // Note: Since we want to check X86_64 and AArch64 specifically, we can just call compute() 
            // since it delegates to them if supported.
            // But to be absolutely sure, we'll test the output of compute() which routes to SIMD
            // vs compute_scalar().
            let actual = filter.compute(&window);
            
            let diff_re = (expected.re - actual.re).abs();
            let diff_im = (expected.im - actual.im).abs();

            if diff_re > 1e-4 || diff_im > 1e-4 {
                println!(
                    "Mismatch! num_taps={}, window_len={}:\nexpected={:?}\nactual={:?}",
                    num_taps, window_len, expected, actual
                );
                std::process::exit(1);
            }
        }
    }
    println!("Correctness verified across 1..=256 taps!");
}
