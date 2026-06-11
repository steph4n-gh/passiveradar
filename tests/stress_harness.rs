use num_complex::Complex;
use passiveradar::dsp::decimate::FirFilter;
use rand::Rng;

#[test]
fn stress_test_simd() {
    let mut rng = rand::thread_rng();

    for num_taps in 1..=64 {
        let mut taps = Vec::new();
        for _ in 0..num_taps {
            taps.push(rng.gen_range(-1.0..1.0));
        }

        let filter = FirFilter::new(taps.clone());

        for window_size in [num_taps, num_taps + 1, num_taps + 3] {
            let mut window = Vec::new();
            for _ in 0..window_size {
                window.push(Complex::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)));
            }

            // Since the compute method expects window to have length equal to num_taps at least
            // We pass a slice of exact size num_taps
            let window_slice = &window[0..num_taps];

            let result_simd = filter.compute(window_slice);

            // Oracle calculation
            let mut expected_re = 0.0;
            let mut expected_im = 0.0;
            for i in 0..num_taps {
                // taps[num_taps - 1 - i] corresponds to reversed taps
                let t = taps[num_taps - 1 - i];
                expected_re += window_slice[i].re * t;
                expected_im += window_slice[i].im * t;
            }

            let diff_re = (result_simd.re - expected_re).abs();
            let diff_im = (result_simd.im - expected_im).abs();
            assert!(
                diff_re < 1e-5 && diff_im < 1e-5,
                "Mismatch at num_taps {}: \nSIMD: {:?}\nOracle: {:?}\nDiff: {}, {}",
                num_taps, result_simd, (expected_re, expected_im), diff_re, diff_im
            );
        }
    }
}
