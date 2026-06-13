use num_complex::Complex;
use passiveradar::dsp::cic::CicDecimator;
use passiveradar::tracking::ekf::{BistaticEkf, AppletonHartreeDispersion};
use passiveradar::tracking::jem::{JemAnalyzer, CicMode};
use std::panic;

// =========================================================================
// 1. EKF NaN/Infinity Robustness & Singularity Tests
// =========================================================================

#[test]
fn test_ekf_nan_propagation_gap() {
    let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
    let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
    
    // Check NaN measurement update
    let tower_pos = [0.0, 0.0, 0.0];
    let fc = 90.9e6;
    
    // Doppler-only update with NaN
    let res = ekf.update(&tower_pos, fc, std::f64::NAN);
    assert_eq!(res, 0.0);
    
    // Verify NaN has NOT propagated to EKF state
    assert!(!ekf.state[0].is_nan(), "EKF state should NOT be contaminated with NaN");
    // Verify Covariance matrix does not have NaN
    assert!(!ekf.cov[0][0].is_nan(), "EKF covariance should remain valid");
}

#[test]
fn test_ekf_joint_nan_propagation_gap() {
    let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
    let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
    let tower_pos = [10.0, 20.0, 5.0];
    let fc = 90.9e6;

    // Joint update with NaN in measurements
    let r_cov = [[0.5, 0.0], [0.0, 1e-12]];
    let res = ekf.update_joint(&tower_pos, fc, std::f64::NAN, 1e-5, r_cov);
    assert_eq!(res, 0.0);

    assert!(!ekf.state[0].is_nan(), "Joint EKF state should NOT be contaminated with NaN");
}

#[test]
fn test_ekf_predict_negative_dt() {
    let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
    let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
    
    // Predict backwards (negative dt)
    ekf.predict(-1.0);
    
    // Assert that the state updated backwards, but note that Q * dt reduces covariance
    assert_eq!(ekf.state[0], 1000.0 - 50.0);
}

#[test]
fn test_appleton_hartree_nan_handling() {
    let fd_free = AppletonHartreeDispersion::cancel(std::f64::NAN, 90.9e6, 12.0, 15.0);
    assert!(fd_free.is_nan(), "Appleton-Hartree cancellation with NaN frequency should return NaN");
}

// =========================================================================
// 2. JEM / JemAnalyzer Phase-Unwrap, Cepstrum & Acoustic Demodulation
// =========================================================================

#[test]
fn test_acoustic_nan_clamp_behavior() {
    let mut jem = JemAnalyzer::new();
    jem.set_cic_mode(CicMode::Acoustic);

    // Feed samples containing NaN. In Acoustic mode, the raw phase calculations will produce NaN.
    // In Rust, clamp(NaN) returns NaN, and casting NaN to integer saturates/defaults to 0.
    // So the system does not panic but propagates 0 into the PCM output.
    let samples = vec![Complex::new(std::f32::NAN, 0.0); 100];

    let result = panic::catch_unwind(move || {
        let mut local_jem = jem;
        local_jem.process_block(120.0, &samples);
        local_jem
    });

    assert!(result.is_ok(), "JEM process_block should handle NaN by casting NaN to 0 i16 without panicking");
    let returned_jem = result.unwrap();
    assert_eq!(returned_jem.pcm_output.len(), 100);
    for &val in &returned_jem.pcm_output {
        assert_eq!(val, 0, "NaN samples should be demodulated to silence (0 i16)");
    }
}

#[test]
fn test_unwrap_phase_nan_and_inf() {
    let jem = JemAnalyzer::new();
    let samples = vec![
        Complex::new(std::f32::NAN, 0.0),
        Complex::new(1.0, std::f32::INFINITY),
    ];
    let unwrapped = jem.unwrap_phase(&samples);
    assert_eq!(unwrapped.len(), 2);
    assert!(unwrapped[0].is_nan());
    assert!(unwrapped[1].is_nan());
}

#[test]
fn test_unwrap_phase_empty_and_single() {
    let jem = JemAnalyzer::new();
    let unwrapped_empty = jem.unwrap_phase(&[]);
    assert_eq!(unwrapped_empty.len(), 0);

    let unwrapped_single = jem.unwrap_phase(&[Complex::new(1.0, 1.0)]);
    assert_eq!(unwrapped_single.len(), 1);
    assert!((unwrapped_single[0] - 0.785398).abs() < 1e-4);
}

#[test]
fn test_compute_cepstrum_nan() {
    let jem = jem_analyzer_with_size(256);
    let samples = vec![Complex::new(std::f32::NAN, std::f32::NAN); 256];
    let cepstrum = jem.compute_cepstrum(&samples);
    assert_eq!(cepstrum.len(), 256);
    for val in cepstrum {
        assert!(val.is_nan());
    }
}

fn jem_analyzer_with_size(size: usize) -> JemAnalyzer {
    let mut jem = JemAnalyzer::new();
    jem.set_fft_size(size);
    jem
}

// =========================================================================
// 3. CIC / CicDecimator Decimation factor & Bypass boundaries
// =========================================================================

#[test]
fn test_cic_decimator_zero_r_gap() {
    // R = 0 is a critical boundary
    let decimator = CicDecimator::new(0, 3);
    let input = vec![Complex::new(1.0f32, 1.0f32); 10];
    let mut output = Vec::new();

    // The scale factor calculation: let scale = 1.0 / (0^3 * 1073741824.0) -> scale = Infinity
    // Let's check the behavior. We catch the call to prevent test suites hanging due to infinite loop or crash.
    let result = panic::catch_unwind(move || {
        let mut local_dec = decimator;
        local_dec.process_block(&input, &mut output);
    });
    if result.is_err() {
        println!("test_cic_decimator_zero_r_gap panicked as expected in debug mode");
    }
}

#[test]
fn test_cic_decimator_zero_n_gap() {
    // N = 0 (0 filter stages)
    let mut decimator = CicDecimator::new(8, 0);
    let input = vec![Complex::new(1.0f32, 1.0f32); 16];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    // 16 input samples, decimation factor 8 -> 2 output samples
    assert_eq!(output.len(), 2);
}

#[test]
fn test_cic_decimator_large_inputs_wrapping() {
    let mut decimator = CicDecimator::new(8, 3);
    // Extremely large input samples that will exceed wrapping boundaries
    let input = vec![Complex::new(1e10f32, -1e10f32); 16];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    assert_eq!(output.len(), 2);
    // Output should be finite due to wrapping i64 arithmetic and scaling
    assert!(output[0].re.is_finite());
    assert!(output[0].im.is_finite());
}
