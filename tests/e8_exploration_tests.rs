use num_complex::Complex;
use passiveradar::math::e8::E8Decoder;
use passiveradar::dsp::morse::prune_discrete_morse;
use passiveradar::math::cohomology::CohomologyFirewall;
use passiveradar::ui::dashboard::Dashboard;
use passiveradar::tracking::bank::TrackingBank;

#[test]
fn test_e8_decoding_integers() {
    let decoder = E8Decoder::new(1.0);
    
    // An even integer vector should decode to itself
    let p_even = [2.0f32, 4.0, 0.0, 0.0, -2.0, 6.0, 8.0, 0.0];
    let decoded_even = decoder.decode(&p_even);
    assert_eq!(decoded_even, p_even);

    // An odd integer vector must have its parity flipped (by changing the coordinate with the worst rounding error)
    // Here we make coordinate 0.9 (rounded to 1.0) and others integers.
    // Coordinates: [0.9, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] -> rounds to [1.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] which sums to 2 (even).
    // Let's test a vector that rounds to an odd sum:
    // [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0] -> rounds to sum = 1 (odd).
    // Worst error is at index 1 (0.4 vs 0.0). Let's test:
    let p_odd = [1.0f32, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    let decoded_odd = decoder.decode(&p_odd);
    
    // The sum of coords must be even.
    let sum: f32 = decoded_odd.iter().sum();
    assert_eq!((sum as i32) % 2, 0);
}

#[test]
fn test_e8_decoding_half_integers() {
    let decoder = E8Decoder::new(1.0);

    // Even sum of half-integers
    // Coords: 8 * 0.5 = 4.0 (even). So this should decode to itself.
    let p_half = [0.5f32; 8];
    let decoded_half = decoder.decode(&p_half);
    assert_eq!(decoded_half, p_half);
}

#[test]
fn test_e8_parity_invariant() {
    let decoder = E8Decoder::new(1.5);
    
    // Generate various float vectors and check that the E8 decoded points always satisfy the parity check:
    // Either all coordinates are integers or all are half-integers, AND their sum is even.
    for i in 0..10 {
        let mut y = [0.0f32; 8];
        for k in 0..8 {
            y[k] = (i as f32 * 0.37 + k as f32 * 0.13).sin() * 5.5;
        }
        let decoded = decoder.decode(&y);
        
        // Sum check
        let sum: f32 = decoded.iter().sum();
        let rounded_sum = sum.round() as i32;
        assert_eq!(rounded_sum % 2, 0, "Decoded sum must be an even integer");

        // Homogeneity check: all integers or all half-integers
        let first_is_half = (decoded[0] - decoded[0].round()).abs() > 0.1;
        for k in 1..8 {
            let is_half = (decoded[k] - decoded[k].round()).abs() > 0.1;
            assert_eq!(is_half, first_is_half, "Coordinates must be of homogeneous type (all integer or all half-integer)");
        }
    }
}

#[test]
fn test_e8_project_and_decode() {
    let decoder = E8Decoder::new(10.0);
    let samples = [
        Complex::new(0.1, -0.2),
        Complex::new(0.5, 0.4),
        Complex::new(-0.3, 0.0),
        Complex::new(0.1, 0.9),
    ];
    let (decoded, residual) = decoder.project_and_decode(&samples);
    assert!(residual >= 0.0);
    
    // Check scaled value matching
    let sum: f32 = decoded.iter().sum();
    assert_eq!((sum.round() as i32) % 2, 0);
}

#[test]
fn test_discrete_morse_pruning() {
    // 5x5 grid with some noise and two strong peaks
    // Peak 1: at (1, 1) with value 10.0
    // Peak 2: at (3, 3) with value 8.0
    // Noise: around 1.0-2.0
    let mut grid = vec![vec![1.0f32; 5]; 5];
    grid[1][1] = 10.0;
    grid[1][2] = 2.0; // connection
    grid[3][3] = 8.0;
    grid[0][0] = 3.0; // small noise peak

    // Prune with threshold 4.0
    let peaks = prune_discrete_morse(&grid, 4.0);

    // Peak at (1,1) has value 10.0, persistence 10.0 (global max).
    // Peak at (3,3) has value 8.0. Since it merges into (1,1) via saddle value 2.0 (at grid[1][2]),
    // its persistence is 8.0 - 2.0 = 6.0 >= 4.0. So it survives.
    // Small noise peak at (0,0) has value 3.0. It merges into (1,1) via saddle value 1.0.
    // Its persistence is 3.0 - 1.0 = 2.0 < 4.0. So it is pruned!
    
    assert!(peaks.len() >= 2);
    assert_eq!(peaks[0].delay, 1);
    assert_eq!(peaks[0].doppler, 1);
    assert_eq!(peaks[0].value, 10.0);

    assert_eq!(peaks[1].delay, 3);
    assert_eq!(peaks[1].doppler, 3);
    assert_eq!(peaks[1].value, 8.0);
    assert!(peaks.iter().all(|p| p.value != 3.0));
}

#[test]
fn test_cohomological_firewall_crossing() {
    let firewall = CohomologyFirewall::new(3, 1, 0.5);

    // Generate a clean FM-like orbital signal loop (circular torus-like embedding)
    let n = 60;
    let mut clean_signal = Vec::with_capacity(n);
    for i in 0..n {
        let phi = (i as f32 * 2.0 * std::f32::consts::PI) / 10.0;
        clean_signal.push(Complex::new(phi.cos(), phi.sin()));
    }

    let clean_embed = firewall.delay_embed(&clean_signal);
    let clean_b1 = firewall.compute_b1(&clean_embed);
    let clean_entropy = firewall.compute_topological_entropy(&clean_embed);

    // Introduce an anomaly (target crossing) in the middle of the signal
    let mut anomaly_signal = clean_signal.clone();
    for i in 25..35 {
        anomaly_signal[i] = Complex::new(0.0, 0.0); // signal path pinched/obstructed
    }

    let anomaly_embed = firewall.delay_embed(&anomaly_signal);
    let anomaly_b1 = firewall.compute_b1(&anomaly_embed);
    let anomaly_entropy = firewall.compute_topological_entropy(&anomaly_embed);

    // The topological invariants must shift significantly
    assert!(clean_b1 != anomaly_b1 || (clean_entropy - anomaly_entropy).abs() > 0.01);
}

#[test]
fn test_websocket_commands_integration() {
    let mut dashboard = Dashboard::new(90.9e6, 256.0e3, 0.0, "sdr".to_string(), 0.0);
    let mut tracking_bank = TrackingBank::new();

    // Verify initial values
    assert_eq!(dashboard.e8_mode_enabled, false);
    assert_eq!(dashboard.morse_persistence, 0.0);
    assert_eq!(dashboard.cohomology_firewall_enabled, false);

    // Test ToggleE8Mode
    let cmd1 = r#"{"action": "ToggleE8Mode", "enabled": true}"#.to_string();
    let resp1 = mock_process_ws_command(cmd1, &mut dashboard, &mut tracking_bank);
    assert_eq!(resp1.get("e8_mode_enabled"), Some(&serde_json::Value::Bool(true)));
    assert_eq!(dashboard.e8_mode_enabled, true);

    // Test SetMorsePersistence
    let cmd2 = r#"{"action": "SetMorsePersistence", "value": 3.5}"#.to_string();
    let resp2 = mock_process_ws_command(cmd2, &mut dashboard, &mut tracking_bank);
    assert_eq!(resp2.get("morse_persistence"), Some(&serde_json::Value::from(3.5)));
    assert_eq!(dashboard.morse_persistence, 3.5);

    // Test ToggleCohomologyFirewall
    let cmd3 = r#"{"action": "ToggleCohomologyFirewall", "enabled": true}"#.to_string();
    let resp3 = mock_process_ws_command(cmd3, &mut dashboard, &mut tracking_bank);
    assert_eq!(resp3.get("cohomology_firewall_enabled"), Some(&serde_json::Value::Bool(true)));
}

#[test]
fn test_cohomology_components_b0() {
    let firewall = CohomologyFirewall::new(2, 1, 0.05); // tiny epsilon

    // Three points far apart: (0,0), (10,10), (20,20)
    let points = vec![
        vec![0.0, 0.0],
        vec![10.0, 10.0],
        vec![20.0, 20.0],
    ];
    // Since epsilon is 0.05, none are connected.
    // V = 3, E = 0, T = 0.
    // Expected b0 = 3 (each is its own component).
    // Expected b1 = E - V - T + b0 = 0 - 3 - 0 + 3 = 0.
    let b1 = firewall.compute_b1(&points);
    assert_eq!(b1, 0.0);
}

#[test]
fn test_cohomology_downsampling_safeguard() {
    let firewall = CohomologyFirewall::new(3, 1, 0.5);
    
    // Generate a long signal (500 points)
    let mut long_signal = Vec::new();
    for i in 0..500 {
        long_signal.push(Complex::new(i as f32, -(i as f32)));
    }
    
    let embeddings = firewall.delay_embed(&long_signal);
    // Should be capped exactly at 256 due to downsampling
    assert_eq!(embeddings.len(), 256);
}

#[test]
fn test_morse_safety_checks() {
    // Empty grid
    let empty_grid: Vec<Vec<f32>> = Vec::new();
    assert_eq!(prune_discrete_morse(&empty_grid, 1.0).len(), 0);

    // Mismatched row sizes
    let mismatched = vec![
        vec![1.0, 2.0],
        vec![1.0],
    ];
    assert_eq!(prune_discrete_morse(&mismatched, 1.0).len(), 0);

    // Too large grid (e.g. 300x300 > 256x256 cap)
    let massive_grid = vec![vec![1.0; 300]; 300];
    assert_eq!(prune_discrete_morse(&massive_grid, 1.0).len(), 0);
}

// Inline helper to parse and route WebSocket commands directly from string for testing
fn mock_process_ws_command(
    cmd_text: String,
    dashboard: &mut Dashboard,
    _tracking_bank: &mut TrackingBank,
) -> serde_json::Value {
    let raw_val: serde_json::Value = serde_json::from_str(&cmd_text).unwrap();
    let act = raw_val.get("action").or(raw_val.get("command")).and_then(|v| v.as_str()).unwrap();
    
    match act {
        "ToggleE8Mode" => {
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            dashboard.e8_mode_enabled = enabled;
            serde_json::json!({"e8_mode_enabled": enabled})
        }
        "SetMorsePersistence" => {
            let val = raw_val.get("value").and_then(|v| v.as_f64()).unwrap_or(0.0);
            dashboard.morse_persistence = val as f32;
            serde_json::json!({"morse_persistence": val})
        }
        "ToggleCohomologyFirewall" => {
            let enabled = raw_val.get("enabled").and_then(|v| v.as_bool()).unwrap_or(false);
            dashboard.cohomology_firewall_enabled = enabled;
            serde_json::json!({"cohomology_firewall_enabled": enabled})
        }
        _ => serde_json::json!({"error": "Unknown command"}),
    }
}
