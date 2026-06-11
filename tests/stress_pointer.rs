use num_complex::Complex;
use passiveradar::dsp::decimate::FirFilter;
use rand::Rng;

#[test]
fn test_fir_filter_stress() {
    let mut rng = rand::thread_rng();

    for _ in 0..10000 {
        let num_taps = rng.gen_range(1..200);
        let window_offset = rng.gen_range(0..50);
        let window_len = num_taps + window_offset;

        let taps: Vec<f32> = (0..num_taps).map(|_| rng.gen_range(-1.0..1.0)).collect();
        let filter = FirFilter::new(taps.clone());

        let window: Vec<Complex<f32>> = (0..window_len)
            .map(|_| Complex::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)))
            .collect();

        let expected = filter.compute_scalar(&window);
        let actual = filter.compute(&window);

        let diff_re = (expected.re - actual.re).abs();
        let diff_im = (expected.im - actual.im).abs();

        // SIMD float math might have slightly different rounding
        assert!(
            diff_re < 1e-3 && diff_im < 1e-3,
            "Mismatch for num_taps={}, window_len={}:\nexpected={:?}\nactual={:?}",
            num_taps, window_len, expected, actual
        );
    }
}

#[test]
fn test_multistage_decimator_stress() {
    let mut rng = rand::thread_rng();
    let mut decimator = passiveradar::dsp::decimate::MultiStageDecimator::new_256x();
    
    // Test with various non-power-of-two input lengths
    for _ in 0..100 {
        let input_len = rng.gen_range(100..5000);
        let input: Vec<Complex<f32>> = (0..input_len)
            .map(|_| Complex::new(rng.gen_range(-1.0..1.0), rng.gen_range(-1.0..1.0)))
            .collect();
        
        let mut output = Vec::new();
        // Just verify it doesn't panic
        decimator.process(&input, &mut output);
    }
}
