pub struct RealTimeGeoSolver {
    pub receiver_pos: [f64; 3],
    pub transmitter_positions: Vec<[f64; 3]>,
}

impl RealTimeGeoSolver {
    pub fn new(receiver_pos: [f64; 3], transmitter_positions: Vec<[f64; 3]>) -> Self {
        Self {
            receiver_pos,
            transmitter_positions,
        }
    }

    /// Update target position using Gauss-Newton solver on bistatic range differences.
    /// `initial_guess`: [x, y, z, b]
    /// `measurements`: bistatic range differences for each transmitter (same order as transmitter_positions)
    pub fn update_position(&self, initial_guess: &[f64; 4], measurements: &[f64]) -> Option<[f64; 4]> {
        let n_transmitters = self.transmitter_positions.len();
        if n_transmitters < 4 || measurements.len() < n_transmitters {
            return None;
        }

        let mut state = *initial_guess;
        let max_iters = 15;
        let tolerance = 1e-4;

        for _ in 0..max_iters {
            let mut jt_j = [[0.0f64; 4]; 4];
            let mut jt_r = [0.0f64; 4];

            let x_rx = self.receiver_pos[0];
            let y_rx = self.receiver_pos[1];
            let z_rx = self.receiver_pos[2];

            let d_rx = ((state[0] - x_rx).powi(2) + (state[1] - y_rx).powi(2) + (state[2] - z_rx).powi(2)).sqrt();
            if d_rx < 1e-3 {
                return None;
            }

            for i in 0..n_transmitters {
                let x_tx = self.transmitter_positions[i][0];
                let y_tx = self.transmitter_positions[i][1];
                let z_tx = self.transmitter_positions[i][2];

                let d_tx = ((state[0] - x_tx).powi(2) + (state[1] - y_tx).powi(2) + (state[2] - z_tx).powi(2)).sqrt();
                if d_tx < 1e-3 {
                    continue;
                }

                let d_baseline = ((x_rx - x_tx).powi(2) + (y_rx - y_tx).powi(2) + (z_rx - z_tx).powi(2)).sqrt();

                // h(S) = R_tx + R_rx - baseline + b
                let h_pred = d_tx + d_rx - d_baseline + state[3];
                let residual = measurements[i] - h_pred;

                // Jacobian row: [dx, dy, dz, db]
                let jac_x = (state[0] - x_tx) / d_tx + (state[0] - x_rx) / d_rx;
                let jac_y = (state[1] - y_tx) / d_tx + (state[1] - y_rx) / d_rx;
                let jac_z = (state[2] - z_tx) / d_tx + (state[2] - z_rx) / d_rx;
                let jac_b = 1.0f64;

                let row = [jac_x, jac_y, jac_z, jac_b];

                // Accumulate J^T * J and J^T * r
                for r in 0..4 {
                    for c in 0..4 {
                        jt_j[r][c] += row[r] * row[c];
                    }
                    jt_r[r] += row[r] * residual;
                }
            }

            // Tikhonov regularization (Ridge) to ensure numerical stability
            for r in 0..4 {
                jt_j[r][r] += 1e-5;
            }

            // Solve (J^T * J) * delta = J^T * r using Gaussian Elimination
            if let Some(delta) = solve_4x4(&mut jt_j, &jt_r) {
                state[0] += delta[0];
                state[1] += delta[1];
                state[2] += delta[2];
                state[3] += delta[3];

                let step_sz = (delta[0].powi(2) + delta[1].powi(2) + delta[2].powi(2) + delta[3].powi(2)).sqrt();
                if step_sz < tolerance {
                    break;
                }
            } else {
                return None;
            }
        }

        // Basic sanity check: target shouldn't be infinitely far
        if state[0].is_nan() || state[0].is_infinite() || state[1].is_nan() || state[2].is_nan() {
            return None;
        }

        Some(state)
    }
}

/// Gaussian elimination solver for 4x4 linear systems A * X = B
fn solve_4x4(a: &mut [[f64; 4]; 4], b: &[f64; 4]) -> Option<[f64; 4]> {
    let mut mat = [[0.0f64; 5]; 4];
    for r in 0..4 {
        for c in 0..4 {
            mat[r][c] = a[r][c];
        }
        mat[r][4] = b[r];
    }

    // Forward elimination
    for i in 0..4 {
        // Pivot selection
        let mut max_row = i;
        let mut max_val = mat[i][i].abs();
        for r in (i + 1)..4 {
            if mat[r][i].abs() > max_val {
                max_val = mat[r][i].abs();
                max_row = r;
            }
        }
        if max_val < 1e-12 {
            return None;
        }
        if max_row != i {
            mat.swap(i, max_row);
        }

        // Eliminate column elements
        for r in (i + 1)..4 {
            let factor = mat[r][i] / mat[i][i];
            for c in i..5 {
                mat[r][c] -= factor * mat[i][c];
            }
        }
    }

    // Back substitution
    let mut x = [0.0f64; 4];
    for r in (0..4).rev() {
        let mut sum = 0.0;
        for c in (r + 1)..4 {
            sum += mat[r][c] * x[c];
        }
        x[r] = (mat[r][4] - sum) / mat[r][r];
    }

    Some(x)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_solve_4x4() {
        let mut a = [
            [2.0, 1.0, -1.0, 2.0],
            [4.0, 5.0, -3.0, 6.0],
            [-2.0, 5.0, -2.0, 6.0],
            [4.0, 11.0, -4.0, 8.0],
        ];
        let b = [5.0, 9.0, 4.0, 2.0];
        let x = solve_4x4(&mut a, &b).unwrap();
        
        // Check solution validity:
        // A * x should equal B
        assert!((2.0 * x[0] + 1.0 * x[1] - 1.0 * x[2] + 2.0 * x[3] - 5.0).abs() < 1e-5);
        assert!((4.0 * x[0] + 5.0 * x[1] - 3.0 * x[2] + 6.0 * x[3] - 9.0).abs() < 1e-5);
    }

    #[test]
    fn test_geodetic_solver_convergence() {
        let receiver = [0.0, 0.0, 0.0];
        let transmitters = vec![
            [10000.0, 0.0, 500.0],
            [0.0, 12000.0, 600.0],
            [-8000.0, -8000.0, 700.0],
            [5000.0, -10000.0, 400.0],
        ];

        let target_pos: [f64; 3] = [3000.0, 4000.0, 2500.0];
        let clock_bias = 450.0; // 450 meters bias

        let solver = RealTimeGeoSolver::new(receiver, transmitters.clone());

        // Generate simulated pseudoranges
        let d_rx = (target_pos[0].powi(2) + target_pos[1].powi(2) + target_pos[2].powi(2)).sqrt();
        let mut measurements = Vec::new();
        for tx in &transmitters {
            let d_tx = ((target_pos[0] - tx[0]).powi(2) + (target_pos[1] - tx[1]).powi(2) + (target_pos[2] - tx[2]).powi(2)).sqrt();
            let d_baseline = (tx[0].powi(2) + tx[1].powi(2) + tx[2].powi(2)).sqrt();
            measurements.push(d_tx + d_rx - d_baseline + clock_bias);
        }

        let guess = [2500.0, 3500.0, 2000.0, 100.0];
        let resolved = solver.update_position(&guess, &measurements).unwrap();

        // Check if solver successfully resolved target coordinates and clock bias!
        assert!((resolved[0] - target_pos[0]).abs() < 1e-1);
        assert!((resolved[1] - target_pos[1]).abs() < 1e-1);
        assert!((resolved[2] - target_pos[2]).abs() < 1e-1);
        assert!((resolved[3] - clock_bias).abs() < 1e-1);
    }
}
