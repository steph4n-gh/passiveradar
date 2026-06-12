use num_complex::Complex;
use passiveradar::orbit::RealTimeGeoSolver;
use passiveradar::dsp::isar::{ofdm_fractional_delay_caf, clean_ambiguity_map, backproject_tomography};

#[test]
fn test_geodetic_solver_4x4() {
    let receiver: [f64; 3] = [0.0, 0.0, 0.0];
    let transmitters: Vec<[f64; 3]> = vec![
        [12000.0, 0.0, 300.0],
        [0.0, 10000.0, 400.0],
        [-9000.0, -9000.0, 500.0],
        [4000.0, -11000.0, 350.0],
    ];

    let target_pos: [f64; 3] = [2500.0, 3500.0, 1500.0];
    let clock_bias = 350.0; 

    let solver = RealTimeGeoSolver::new(receiver, transmitters.clone());

    // Generate bistatic pseudoranges
    let d_rx = (target_pos[0].powi(2) + target_pos[1].powi(2) + target_pos[2].powi(2)).sqrt();
    let mut measurements = Vec::new();
    for tx in &transmitters {
        let d_tx = ((target_pos[0] - tx[0]).powi(2) + (target_pos[1] - tx[1]).powi(2) + (target_pos[2] - tx[2]).powi(2)).sqrt();
        let d_baseline = (tx[0].powi(2) + tx[1].powi(2) + tx[2].powi(2)).sqrt();
        measurements.push(d_tx + d_rx - d_baseline + clock_bias);
    }

    let guess = [1000.0, 1000.0, 1000.0, 0.0];
    let resolved = solver.update_position(&guess, &measurements).unwrap();

    assert!((resolved[0] - target_pos[0]).abs() < 1e-1);
    assert!((resolved[1] - target_pos[1]).abs() < 1e-1);
    assert!((resolved[2] - target_pos[2]).abs() < 1e-1);
    assert!((resolved[3] - clock_bias).abs() < 1e-1);
}

#[test]
fn test_ofdm_fractional_delay_caf() {
    let n = 256;
    let mut surv = vec![Complex::new(0.0, 0.0); n];
    let mut reference = vec![Complex::new(0.0, 0.0); n];

    // Generate complex sine waves
    for i in 0..n {
        let t = i as f32 / n as f32;
        reference[i] = Complex::new((2.0 * std::f32::consts::PI * 10.0 * t).cos(), (2.0 * std::f32::consts::PI * 10.0 * t).sin());
        
        // Surveillance is delayed by exactly 1.5 samples
        let t_delayed = (i as f32 - 1.5) / n as f32;
        surv[i] = Complex::new((2.0 * std::f32::consts::PI * 10.0 * t_delayed).cos(), (2.0 * std::f32::consts::PI * 10.0 * t_delayed).sin());
    }

    let corr = ofdm_fractional_delay_caf(&surv, &reference, 1.5);
    
    // The cross correlation peak should be at index 0 because the Shift Theorem 
    // fractional phase offset exactly aligns the delay!
    let mut max_idx = 0;
    let mut max_val = 0.0f32;
    for (idx, val) in corr.iter().enumerate() {
        if val.norm() > max_val {
            max_val = val.norm();
            max_idx = idx;
        }
    }
    assert_eq!(max_idx, 0);
    assert!(max_val > 0.9);
}

#[test]
fn test_clean_algorithm_omp() {
    // Generate a simple 10x10 ambiguity map with a large target and a small target
    let mut map = vec![vec![0.0f32; 10]; 10];
    map[3][4] = 100.0; // Large airliner target
    map[6][7] = 45.0;  // Small drone target

    // Add some sidelobes from airliner using Gaussian spread
    for r in 0..10 {
        let dr = (r as f32 - 3.0).powi(2);
        for c in 0..10 {
            let dc = (c as f32 - 4.0).powi(2);
            map[r][c] += 100.0 * (-dr/8.0 - dc/8.0).exp();
        }
    }
    // Set exact peak values again
    map[3][4] = 100.0;
    map[6][7] = 45.0;

    let components = clean_ambiguity_map(&mut map, 2, 0.8);
    
    assert_eq!(components.len(), 2);
    // First component should be airliner at (3, 4)
    assert_eq!(components[0].0, 3);
    assert_eq!(components[0].1, 4);
    assert!(components[0].2 > 90.0);

    // Second component should be drone at (6, 7)
    assert_eq!(components[1].0, 6);
    assert_eq!(components[1].1, 7);
}

#[test]
fn test_backproject_tomography() {
    let n_angles = 4;
    let n_bins = 64;
    let mut profiles = vec![vec![0.0f32; n_bins]; n_angles];

    // Simulate range profiles with a target in the center (index 32)
    for i in 0..n_angles {
        profiles[i][32] = 10.0;
        // Lobe spread
        profiles[i][31] = 5.0;
        profiles[i][33] = 5.0;
    }

    let angles = vec![0.0, std::f32::consts::FRAC_PI_4, std::f32::consts::FRAC_PI_2, 3.0 * std::f32::consts::FRAC_PI_4];
    let image = backproject_tomography(&profiles, &angles, 16);

    assert_eq!(image.len(), 16);
    assert_eq!(image[0].len(), 16);

    // Reconstructed image center (index 8, 8) should have the highest back-projected density!
    let mut max_val = -100.0f32;
    let mut max_x = 0;
    let mut max_y = 0;
    for x in 0..16 {
        for y in 0..16 {
            if image[x][y] > max_val {
                max_val = image[x][y];
                max_x = x;
                max_y = y;
            }
        }
    }
    assert_eq!(max_x, 8);
    assert_eq!(max_y, 8);
}
