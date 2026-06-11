use num_complex::Complex;
use passiveradar::dsp::decimate::FirFilter;

#[test]
fn test_simd_vs_scalar() {
    let taps = vec![1.0, 2.0, 3.0, 4.0, 5.0];
    let filter = FirFilter::new(taps.clone());

    let mut window = Vec::new();
    for i in 0..5 {
        window.push(Complex::new(i as f32, (i * 2) as f32));
    }

    let result_simd = filter.compute(&window);
    
    // Calculate scalar equivalent
    let mut expected_re = 0.0;
    let mut expected_im = 0.0;
    for i in 0..5 {
        expected_re += window[i].re * taps[4 - i];
        expected_im += window[i].im * taps[4 - i];
    }
    let expected = Complex::new(expected_re, expected_im);

    assert_eq!(result_simd.re, expected.re);
    assert_eq!(result_simd.im, expected.im);
}
