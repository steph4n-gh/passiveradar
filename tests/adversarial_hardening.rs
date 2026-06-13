use passiveradar::dsp::cic::CicDecimator;
use passiveradar::tracking::jem::{JemAnalyzer, CicMode, AcousticBandpassFilter};
use passiveradar::tracking::ekf::{BistaticEkf, AppletonHartreeDispersion};
use num_complex::Complex;

// =========================================================================
// 1. CIC Decimator Adversarial Tests
// =========================================================================

#[test]
fn test_cic_zero_r() {
    // Check if decimation factor R=0 is handled or if we bypass/behave stably.
    let mut decimator = CicDecimator::new(0, 3);
    let input = vec![Complex::new(1.0, 2.0); 10];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    // The decimation counter increments and never equals 0, so output should be empty.
    assert!(output.is_empty());
}

#[test]
fn test_cic_zero_n() {
    // N=0 stages: integrators and combs vectors are empty.
    let mut decimator = CicDecimator::new(8, 0);
    let input = vec![Complex::new(1.0, 2.0); 16];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    assert_eq!(output.len(), 2);
    for sample in output {
        assert!((sample.re - 1.0).abs() < 1e-4);
        assert!((sample.im - 2.0).abs() < 1e-4);
    }
}

#[test]
fn test_cic_large_n() {
    // Test large stages N = 100 to check for potential integer/float overflow.
    let mut decimator = CicDecimator::new(2, 100);
    let input = vec![Complex::new(0.5, -0.5); 10];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    assert_eq!(output.len(), 5);
}

#[test]
fn test_cic_extreme_stages_overflow() {
    // N = 1024 stages. (2.0 as f64).powi(1024) overflows f64 to INFINITY.
    // scale = 1.0 / INFINITY = 0.0.
    // Output should be zero, but must not panic.
    let mut decimator = CicDecimator::new(2, 1024);
    let input = vec![Complex::new(0.5, -0.5); 10];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    assert_eq!(output.len(), 5);
    for sample in output {
        assert_eq!(sample.re, 0.0);
        assert_eq!(sample.im, 0.0);
    }
}

#[test]
fn test_cic_nan_infinity_inputs() {
    let mut decimator = CicDecimator::new(2, 3);
    let input = vec![
        Complex::new(std::f32::NAN, 1.0),
        Complex::new(std::f32::INFINITY, std::f32::NEG_INFINITY),
        Complex::new(0.5, 0.5),
        Complex::new(-0.5, -0.5),
    ];
    let mut output = Vec::new();
    decimator.process_block(&input, &mut output);
    // Under standard Rust casting rules (since 1.45), float to int cast saturates.
    assert_eq!(output.len(), 2);
    for sample in &output {
        assert!(!sample.re.is_nan());
        assert!(!sample.im.is_nan());
    }
}

// =========================================================================
// 2. JEM Analyzer Adversarial Tests
// =========================================================================

#[test]
fn test_acoustic_filter_extreme_inputs() {
    // Test sample rates or frequencies that could trigger division by zero or NaN coefficients.
    let mut filter = AcousticBandpassFilter::new(0.0, 0.0, 0.0);
    let out = filter.process(1.0);
    assert!(out.is_nan());

    let fs = -2.0 * std::f32::consts::PI * 100.0;
    let mut filter_div_zero = AcousticBandpassFilter::new(fs, 100.0, 100.0);
    let out = filter_div_zero.process(1.0);
    assert!(out.is_nan() || out.is_infinite());
}

#[test]
fn test_jem_unwrap_phase_empty() {
    let jem = JemAnalyzer::new();
    let unwrapped = jem.unwrap_phase(&[]);
    assert!(unwrapped.is_empty());
}

#[test]
fn test_jem_unwrap_phase_nan_inf() {
    let jem = JemAnalyzer::new();
    let input = vec![
        Complex::new(std::f32::NAN, 1.0),
        Complex::new(1.0, std::f32::INFINITY),
        Complex::new(std::f32::NEG_INFINITY, std::f32::NAN),
    ];
    let unwrapped = jem.unwrap_phase(&input);
    assert_eq!(unwrapped.len(), 3);
}

#[test]
fn test_jem_compute_cepstrum_empty() {
    let jem = JemAnalyzer::new();
    let cep = jem.compute_cepstrum(&[]);
    assert!(cep.is_empty());
}

#[test]
fn test_jem_compute_cepstrum_size_1() {
    let jem = JemAnalyzer::new();
    let input = vec![Complex::new(1.0, 2.0)];
    let cep = jem.compute_cepstrum(&input);
    assert_eq!(cep.len(), 1);
    assert!(!cep[0].is_nan());
}

#[test]
fn test_jem_compute_cepstrum_nan_inf() {
    let jem = JemAnalyzer::new();
    let input = vec![
        Complex::new(std::f32::NAN, 1.0),
        Complex::new(1.0, std::f32::INFINITY),
        Complex::new(std::f32::NEG_INFINITY, 0.0),
        Complex::new(0.0, 0.0),
    ];
    let cep = jem.compute_cepstrum(&input);
    assert_eq!(cep.len(), 4);
}

#[test]
fn test_jem_process_block_extreme_doppler() {
    let mut jem = JemAnalyzer::new();
    let samples = vec![Complex::new(0.5, 0.5); 100];
    
    jem.process_block(std::f64::NAN, &samples);
    jem.process_block(std::f64::INFINITY, &samples);
    jem.process_block(std::f64::NEG_INFINITY, &samples);
    
    for &val in &jem.latest_fft_mag {
        assert!(!val.is_nan());
    }
}


#[test]
fn test_jem_process_block_acoustic_nan_pollution() {
    // Verify that feeding NaN in Acoustic mode does NOT pollute the state of the bandpass filter permanently.
    let mut jem = JemAnalyzer::new();
    jem.set_cic_mode(CicMode::Acoustic);

    let nan_samples = vec![Complex::new(std::f32::NAN, 0.0); 10];
    jem.process_block(0.0, &nan_samples);

    for &val in &jem.pcm_output {
        assert_eq!(val, 0);
    }

    // Use rotating phase so the bandpass filter produces non-zero outputs
    let normal_samples: Vec<Complex<f32>> = (0..20)
        .map(|i| {
            let angle = (i as f32) * 0.5;
            Complex::new(angle.cos(), angle.sin())
        })
        .collect();
    jem.process_block(0.0, &normal_samples);

    let mut has_nonzero = false;
    for &val in &jem.pcm_output {
        if val != 0 {
            has_nonzero = true;
            break;
        }
    }
    assert!(has_nonzero, "Subsequent normal input samples must produce non-zero, valid outputs, showing the filter is not polluted");
}

#[test]
fn test_jem_update_heuristics_extreme_inputs() {
    let mut jem = JemAnalyzer::new();
    
    jem.update_heuristics(10.0, 0.0);
    assert!(!jem.payload_class.is_empty());

    jem.update_heuristics(10.0, -0.5);
    assert!(!jem.payload_class.is_empty());

    jem.update_heuristics(10.0, std::f64::NAN);
    assert!(!jem.payload_class.is_empty());

    jem.update_heuristics(std::f64::NAN, 0.1);
    assert!(!jem.payload_class.is_empty());

    jem.update_heuristics(std::f64::INFINITY, 0.1);
    assert!(!jem.payload_class.is_empty());
}

#[test]
fn test_jem_invalid_fft_size() {
    let mut jem = JemAnalyzer::new();
    let original_size = jem.latest_fft_mag.len();
    
    jem.set_fft_size(999);
    assert_eq!(jem.latest_fft_mag.len(), original_size);

    jem.set_fft_size(0);
    assert_eq!(jem.latest_fft_mag.len(), original_size);
}

#[test]
fn test_jem_get_blade_pass_frequency_empty_cepstrum() {
    let mut jem = JemAnalyzer::new();
    jem.set_cic_mode(CicMode::Rotary);
    jem.cepstrum.clear();
    let bpf = jem.get_blade_pass_frequency();
    assert_eq!(bpf, 0.0);
}

// =========================================================================
// 3. EKF Tracking Adversarial Tests
// =========================================================================

#[test]
fn test_ekf_invalid_init() {
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    let ekf = BistaticEkf::new(init_state, -10.0, 0.0, -1.0);
    assert_eq!(ekf.state, init_state);
}

#[test]
fn test_ekf_predict_extreme_dt() {
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    let mut ekf = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);

    ekf.predict(0.0);
    assert_eq!(ekf.state, init_state);

    ekf.predict(-1.0);
    assert_eq!(ekf.state[0], 100.0 - 10.0);

    let mut ekf_nan = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);
    ekf_nan.predict(std::f64::NAN);
    assert!(ekf_nan.state[0].is_nan());

    let mut ekf_inf = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);
    ekf_inf.predict(std::f64::INFINITY);
    assert!(ekf_inf.state[0].is_infinite());
}

#[test]
fn test_ekf_update_zero_carrier() {
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    let mut ekf = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);
    
    let inn = ekf.update(&[0.0, 0.0, 0.0], 0.0, 15.0);
    assert!(!inn.is_nan());
    assert!(!ekf.state[0].is_nan());
}

#[test]
fn test_ekf_update_nan_inputs() {
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    let mut ekf = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);

    let inn = ekf.update(&[0.0, 0.0, 0.0], std::f64::NAN, 15.0);
    assert_eq!(inn, 0.0);
    assert!(!ekf.state[0].is_nan());
}

#[test]
fn test_ekf_update_joint_nan_det() {
    // Verify that det being NaN is rejected and does NOT propagate NaNs into EKF state and covariance.
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    let r_cov = [[1.0, 0.0], [0.0, 1.0]];

    let mut ekf_nan_cov = BistaticEkf::new(init_state, 10.0, 1.0, 1.0);
    ekf_nan_cov.cov[0][0] = std::f64::NAN;
    let inn = ekf_nan_cov.update_joint(&[0.0, 0.0, 0.0], 90e6, 10.0, 1e-6, r_cov);
    assert_eq!(inn, 0.0);
    
    assert!(!ekf_nan_cov.state[0].is_nan());
    assert!(!ekf_nan_cov.cov[1][1].is_nan());
}

#[test]
fn test_ekf_update_joint_singular_cov() {
    let init_state = [100.0, 200.0, 300.0, 10.0, 10.0, 10.0];
    
    let r_cov_singular = [[0.0, 0.0], [0.0, 0.0]];
    let mut ekf_zero_cov = BistaticEkf::new(init_state, 0.0, 0.0, 0.0);
    let inn_zero = ekf_zero_cov.update_joint(&[0.0, 0.0, 0.0], 90e6, 10.0, 1e-6, r_cov_singular);
    assert_eq!(inn_zero, 0.0);
}

#[test]
fn test_appleton_hartree_exact_match() {
    let val = AppletonHartreeDispersion::cancel(90e6, 90e6, 10.0, 12.0);
    assert_eq!(val, 10.0);

    // For very close frequencies in the MHz range (e.g. difference of 0.5 micro-Hz),
    // the absolute threshold on difference of squares (diff.abs() < 1e-6) is bypassed because
    // diff = f1_sq - f2_sq = (f1 - f2) * (f1 + f2) = 0.5e-6 * 180e6 = 90.0.
    // This causes division by 90.0 and results in massive numerical instability.
    let val_close = AppletonHartreeDispersion::cancel(90e6, 90e6 + 0.5e-6, 10.0, 12.0);
    assert!((val_close - 10.0).abs() < 1e-3);
}

