
fn calc_doppler(pos: &[f64; 3], state: &[f64; 6], lambda: f64) -> f64 {
    let dx = state[0] - pos[0];
    let dy = state[1] - pos[1];
    let dz = state[2] - pos[2];
    let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
    let r_r = (state[0] * state[0] + state[1] * state[1] + state[2] * state[2]).sqrt().max(1.0);
    let dot_t = (state[3] * dx + state[4] * dy + state[5] * dz) / r_t;
    let dot_r = (state[3] * state[0] + state[4] * state[1] + state[5] * state[2]) / r_r;
    -(dot_t + dot_r) / lambda
}

pub fn monna_map(mut val: u64, p: u64) -> f64 {
    if p < 2 { return 0.0; }
    let mut res = 0.0;
    let mut base = 1.0 / (p as f64);
    while val > 0 {
        let digit = val % p;
        res += (digit as f64) * base;
        val /= p;
        base /= p as f64;
    }
    res
}

pub fn inverse_monna_map(mut x: f64, p: u64, precision: usize) -> u64 {
    if p < 2 { return 0; }
    x = x.clamp(0.0, 1.0 - 1e-15);
    let mut val = 0u64;
    let mut factor = 1u64;
    for _ in 0..precision {
        x *= p as f64;
        let digit = ((x + 1e-10).floor() as u64).min(p - 1);
        val += (digit % p) * factor;
        x -= digit as f64;
        if let Some(next_factor) = factor.checked_mul(p) {
            factor = next_factor;
        } else {
            break;
        }
    }
    val
}

fn main() {
    let weta_pos = [-120_000.0, 50_000.0, 350.0];
    let wtop_pos = [20_000.0, 110_000.0, 380.0];
    let wiyy_pos = [80_000.0, -90_000.0, 420.0];

    let lambda_weta = 3.0e8 / 90.9e6;
    let lambda_wtop = 3.0e8 / 103.5e6;
    let lambda_wiyy = 3.0e8 / 97.9e6;

    let true_state = [-25_000.0, -15_000.0, 9000.0, 160.0, 170.0, 0.0];

    let dop_weta = calc_doppler(&weta_pos, &true_state, lambda_weta);
    let dop_wtop = calc_doppler(&wtop_pos, &true_state, lambda_wtop);
    let dop_wiyy = calc_doppler(&wiyy_pos, &true_state, lambda_wiyy);

    let direction_sign = -dop_weta.signum();
    let init_state = [
        -20_000.0 * direction_sign,
        -10_000.0 * direction_sign,
        9500.0,
        150.0 * direction_sign,
        160.0 * direction_sign,
        0.0,
    ];

    let bounds = [
        (-100_000.0, 100_000.0),
        (-100_000.0, 100_000.0),
        (150.0, 13_000.0),
        (-350.0, 350.0),
        (-350.0, 350.0),
        (-50.0, 50.0),
    ];

    let cost_fn = |state: &[f64; 6]| -> f64 {
        let mut rss = 0.0;
        // WETA
        {
            let pred = calc_doppler(&weta_pos, state, lambda_weta);
            let diff = dop_weta - pred;
            rss += diff * diff;
        }
        // WTOP
        {
            let pred = calc_doppler(&wtop_pos, state, lambda_wtop);
            let diff = dop_wtop - pred;
            rss += diff * diff;
        }
        // WIYY
        {
            let pred = calc_doppler(&wiyy_pos, state, lambda_wiyy);
            let diff = dop_wiyy - pred;
            rss += diff * diff;
        }
        rss
    };

    let mut best_rss = f64::MAX;
    let mut best_state = init_state;

    // Coarse grid restarts (18 points)
    let x_pts = [-60000.0, 0.0, 60000.0];
    let y_pts = [-60000.0, 0.0, 60000.0];
    let z_pts = [3000.0, 9500.0];

    for &x in &x_pts {
        for &y in &y_pts {
            for &z in &z_pts {
                let mut state = [
                    x,
                    y,
                    z,
                    150.0 * direction_sign,
                    160.0 * direction_sign,
                    0.0,
                ];

                let mut rss = cost_fn(&state);
                let mut step_sizes = [15000.0, 15000.0, 1500.0, 50.0, 50.0, 10.0];

                for _iter in 0..40 {
                    let mut improved = false;
                    for i in 0..6 {
                        let step_size = step_sizes[i];
                        
                        let mut state_plus = state;
                        state_plus[i] = (state[i] + step_size).clamp(bounds[i].0, bounds[i].1);
                        let rss_plus = cost_fn(&state_plus);
                        
                        let mut state_minus = state;
                        state_minus[i] = (state[i] - step_size).clamp(bounds[i].0, bounds[i].1);
                        let rss_minus = cost_fn(&state_minus);
                        
                        if rss_plus < rss && rss_plus <= rss_minus {
                            state = state_plus;
                            rss = rss_plus;
                            improved = true;
                        } else if rss_minus < rss {
                            state = state_minus;
                            rss = rss_minus;
                            improved = true;
                        }
                    }
                    if !improved {
                        for i in 0..6 {
                            step_sizes[i] *= 0.5;
                        }
                    }
                    if rss < 1.0 {
                        break;
                    }
                }

                if rss < best_rss {
                    best_rss = rss;
                    best_state = state;
                }
            }
        }
    }

    let mut step_sizes = [2000.0, 2000.0, 200.0, 10.0, 10.0, 2.0];
    for _iter in 0..50 {
        let mut improved = false;
        for i in 0..6 {
            let step_size = step_sizes[i];
            
            let mut state_plus = best_state;
            state_plus[i] = (best_state[i] + step_size).clamp(bounds[i].0, bounds[i].1);
            let rss_plus = cost_fn(&state_plus);
            
            let mut state_minus = best_state;
            state_minus[i] = (best_state[i] - step_size).clamp(bounds[i].0, bounds[i].1);
            let rss_minus = cost_fn(&state_minus);
            
            if rss_plus < best_rss && rss_plus <= rss_minus {
                best_state = state_plus;
                best_rss = rss_plus;
                improved = true;
            } else if rss_minus < best_rss {
                best_state = state_minus;
                best_rss = rss_minus;
                improved = true;
            }
        }
        if !improved {
            for i in 0..6 {
                step_sizes[i] *= 0.5;
            }
        }
        if best_rss < 0.01 {
            break;
        }
    }

    println!("Pattern Search Optimized RSS: {}", best_rss);
    println!("Pattern Search Optimized State: {:?}", best_state);
}
