// Extended Kalman Filter for Bistatic Passive Radar tracking.
// State vector: X = [x, y, z, vx, vy, vz]^T  (6x1)
// Covariance:   P (6x6)

use crate::sdr::C;

#[derive(Debug, Clone)]
pub struct BistaticEkf {
    pub state: [f64; 6],    // [x, y, z, vx, vy, vz]
    pub cov: [[f64; 6]; 6], // 6x6 covariance matrix
    pub q: [[f64; 6]; 6],   // 6x6 process noise
    pub r: f64,             // measurement noise variance
    pub last_update: std::time::Instant,
    pub stare_mode_active: bool,
    pub stare_coords: [f64; 3],
}

impl BistaticEkf {
    /// Initialize a new EKF with an initial state estimate.
    pub fn new(init_state: [f64; 6], pos_uncert: f64, vel_uncert: f64, r_variance: f64) -> Self {
        let mut cov = [[0.0f64; 6]; 6];
        for i in 0..3 {
            cov[i][i] = pos_uncert.powi(2);
            cov[i + 3][i + 3] = vel_uncert.powi(2);
        }

        // Process noise matrix Q (discrete constant white noise approximation)
        let mut q = [[0.0f64; 6]; 6];
        let q_pos = 1.0; // position process noise variance
        let q_vel = 0.1; // velocity process noise variance
        for i in 0..3 {
            q[i][i] = q_pos;
            q[i + 3][i + 3] = q_vel;
        }

        Self {
            state: init_state,
            cov,
            q,
            r: r_variance,
            last_update: std::time::Instant::now(),
            stare_mode_active: false,
            stare_coords: [0.0; 3],
        }
    }

    /// Set stare mode on or off.
    pub fn set_stare_mode(&mut self, coords: [f64; 3], enabled: bool) {
        self.stare_mode_active = enabled;
        self.stare_coords = coords;
        if enabled {
            self.state = [coords[0], coords[1], coords[2], 0.0, 0.0, 0.0];
            self.cov = [[0.0f64; 6]; 6];
            for i in 0..6 {
                self.cov[i][i] = 1e-9;
            }
        }
    }

    /// Predict the state forward in time by dt seconds using a constant-velocity linear motion model.
    pub fn predict(&mut self, dt: f64) {
        if self.stare_mode_active {
            self.state = [
                self.stare_coords[0],
                self.stare_coords[1],
                self.stare_coords[2],
                0.0,
                0.0,
                0.0,
            ];
            self.cov = [[0.0f64; 6]; 6];
            for i in 0..6 {
                self.cov[i][i] = 1e-9;
            }
            return;
        }

        // 1. Transition state: X = F * X
        self.state[0] += self.state[3] * dt;
        self.state[1] += self.state[4] * dt;
        self.state[2] += self.state[5] * dt;

        // 2. Transition covariance: P = F * P * F^T + Q
        // F = [ I   dt*I ]
        //     [ 0    I   ]
        let mut f_p = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                if r < 3 {
                    f_p[r][c] = self.cov[r][c] + dt * self.cov[r + 3][c];
                } else {
                    f_p[r][c] = self.cov[r][c];
                }
            }
        }

        let mut next_cov = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let val = if c < 3 {
                    f_p[r][c] + dt * f_p[r][c + 3]
                } else {
                    f_p[r][c]
                };
                next_cov[r][c] = val + self.q[r][c] * dt;
            }
        }

        self.cov = next_cov;
    }

    /// Update the state estimate using joint Doppler shift and range delay measurements.
    pub fn update_joint(&mut self, tower_pos: &[f64; 3], fc: f64, z_doppler: f64, z_delay_sec: f64, r_cov: [[f64; 2]; 2]) -> f64 {
        if z_doppler.is_nan() || z_doppler.is_infinite() || z_delay_sec.is_nan() || z_delay_sec.is_infinite() {
            return 0.0;
        }
        if self.stare_mode_active {
            self.state = [
                self.stare_coords[0],
                self.stare_coords[1],
                self.stare_coords[2],
                0.0,
                0.0,
                0.0,
            ];
            self.cov = [[0.0f64; 6]; 6];
            for i in 0..6 {
                self.cov[i][i] = 1e-9;
            }
            return 0.0;
        }

        let x = self.state[0];
        let y = self.state[1];
        let z = self.state[2];
        let vx = self.state[3];
        let vy = self.state[4];
        let vz = self.state[5];

        // 1. Calculate distances from plane to tower (R_t) and receiver (R_r)
        let dx = x - tower_pos[0];
        let dy = y - tower_pos[1];
        let dz = z - tower_pos[2];

        let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let r_r = (x * x + y * y + z * z).sqrt().max(1.0);
        let r_baseline = (tower_pos[0] * tower_pos[0] + tower_pos[1] * tower_pos[1] + tower_pos[2] * tower_pos[2]).sqrt();

        // 2. Predicted range rate and Doppler shift
        let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
        let dot_r = (vx * x + vy * y + vz * z) / r_r;
        let range_rate_pred = dot_t + dot_r;

        let lambda = C / fc;
        let z_doppler_pred = -range_rate_pred / lambda;

        // Predicted range delay (in seconds)
        let z_delay_pred = (r_t + r_r - r_baseline) / C;

        // 3. Compute Jacobian H (2x6 matrix)
        // Row 0: Doppler Jacobian (partial derivatives of z_doppler_pred w.r.t state)
        let dr_dvx = dx / r_t + x / r_r;
        let dr_dvy = dy / r_t + y / r_r;
        let dr_dvz = dz / r_t + z / r_r;

        let d_t_x = vx * (r_t.powi(2) - dx * dx) / r_t.powi(3)
            - vy * (dx * dy) / r_t.powi(3)
            - vz * (dx * dz) / r_t.powi(3);
        let d_t_y = -vx * (dx * dy) / r_t.powi(3) + vy * (r_t.powi(2) - dy * dy) / r_t.powi(3)
            - vz * (dy * dz) / r_t.powi(3);
        let d_t_z = -vx * (dx * dz) / r_t.powi(3) - vy * (dy * dz) / r_t.powi(3)
            + vz * (r_t.powi(2) - dz * dz) / r_t.powi(3);

        let d_r_x = vx * (r_r.powi(2) - x * x) / r_r.powi(3)
            - vy * (x * y) / r_r.powi(3)
            - vz * (x * z) / r_r.powi(3);
        let d_r_y = -vx * (x * y) / r_r.powi(3) + vy * (r_r.powi(2) - y * y) / r_r.powi(3)
            - vz * (y * z) / r_r.powi(3);
        let d_r_z = -vx * (x * z) / r_r.powi(3) - vy * (y * z) / r_r.powi(3)
            + vz * (r_r.powi(2) - z * z) / r_r.powi(3);

        let dr_dx = d_t_x + d_r_x;
        let dr_dy = d_t_y + d_r_y;
        let dr_dz = d_t_z + d_r_z;

        let h0 = [
            -dr_dx / lambda,
            -dr_dy / lambda,
            -dr_dz / lambda,
            -dr_dvx / lambda,
            -dr_dvy / lambda,
            -dr_dvz / lambda,
        ];

        // Row 1: Delay Jacobian (partial derivatives of z_delay_pred w.r.t state)
        let h1 = [
            (dx / r_t + x / r_r) / C,
            (dy / r_t + y / r_r) / C,
            (dz / r_t + z / r_r) / C,
            0.0,
            0.0,
            0.0,
        ];

        let h = [h0, h1];

        // 4. Kalman Gain calculation: K = P * H^T * (H * P * H^T + R)^{-1}
        // Compute H * P (2x6 matrix)
        let mut h_p = [[0.0f64; 6]; 2];
        for i in 0..2 {
            for c in 0..6 {
                let mut val = 0.0;
                for r in 0..6 {
                    val += h[i][r] * self.cov[r][c];
                }
                h_p[i][c] = val;
            }
        }

        // Compute H * P * H^T (2x2 matrix)
        let mut h_p_ht = [[0.0f64; 2]; 2];
        for i in 0..2 {
            for j in 0..2 {
                let mut val = 0.0;
                for k in 0..6 {
                    val += h_p[i][k] * h[j][k];
                }
                h_p_ht[i][j] = val;
            }
        }

        // Innovation covariance S (2x2 matrix)
        let s00 = h_p_ht[0][0] + r_cov[0][0];
        let s01 = h_p_ht[0][1] + r_cov[0][1];
        let s10 = h_p_ht[1][0] + r_cov[1][0];
        let s11 = h_p_ht[1][1] + r_cov[1][1];

        let det = s00 * s11 - s01 * s10;
        if det.is_nan() || det.is_infinite() || det.abs() < 1e-15 {
            return 0.0; // Singular update, abort
        }

        // S_inv (2x2 matrix)
        let s_inv_00 = s11 / det;
        let s_inv_01 = -s01 / det;
        let s_inv_10 = -s10 / det;
        let s_inv_11 = s00 / det;

        // Compute P * H^T (6x2 matrix)
        let mut p_ht = [[0.0f64; 2]; 6];
        for r in 0..6 {
            for j in 0..2 {
                let mut val = 0.0;
                for c in 0..6 {
                    val += self.cov[r][c] * h[j][c];
                }
                p_ht[r][j] = val;
            }
        }

        // Compute Kalman Gain K = P * H^T * S_inv (6x2 matrix)
        let mut k = [[0.0f64; 2]; 6];
        for r in 0..6 {
            k[r][0] = p_ht[r][0] * s_inv_00 + p_ht[r][1] * s_inv_10;
            k[r][1] = p_ht[r][0] * s_inv_01 + p_ht[r][1] * s_inv_11;
        }

        // 5. Update state: X = X + K * y
        let y_doppler = z_doppler - z_doppler_pred;
        let y_delay = z_delay_sec - z_delay_pred;

        for i in 0..6 {
            self.state[i] += k[i][0] * y_doppler + k[i][1] * y_delay;
        }

        // 6. Update covariance using Joseph formulation: P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
        // Compute I - K * H (6x6 matrix)
        let mut i_kh = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let identity = if r == c { 1.0 } else { 0.0 };
                i_kh[r][c] = identity - (k[r][0] * h[0][c] + k[r][1] * h[1][c]);
            }
        }

        // Compute temp = (I - K * H) * P (6x6 matrix)
        let mut temp = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut val = 0.0;
                for i in 0..6 {
                    val += i_kh[r][i] * self.cov[i][c];
                }
                temp[r][c] = val;
            }
        }

        // Compute next_cov = temp * (I - K * H)^T + K * R * K^T (6x6 matrix)
        let mut next_cov = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut val = 0.0;
                for i in 0..6 {
                    val += temp[r][i] * i_kh[c][i];
                }
                let kr_r0 = k[r][0] * r_cov[0][0] + k[r][1] * r_cov[1][0];
                let kr_r1 = k[r][0] * r_cov[0][1] + k[r][1] * r_cov[1][1];
                let kr_kt = kr_r0 * k[c][0] + kr_r1 * k[c][1];

                next_cov[r][c] = val + kr_kt;
            }
        }
        self.cov = next_cov;

        (y_doppler * y_doppler + y_delay * y_delay).sqrt()
    }

    /// Update the state estimate using a bistatic Doppler shift measurement.
    /// `tower_pos`: [x, y, z] of transmitter tower
    /// `fc`: FM carrier frequency of tower
    /// `z_meas`: measured Doppler shift in Hz
    pub fn update(&mut self, tower_pos: &[f64; 3], fc: f64, z_meas: f64) -> f64 {
        if z_meas.is_nan() || z_meas.is_infinite() {
            return 0.0;
        }
        if self.stare_mode_active {
            self.state = [
                self.stare_coords[0],
                self.stare_coords[1],
                self.stare_coords[2],
                0.0,
                0.0,
                0.0,
            ];
            self.cov = [[0.0f64; 6]; 6];
            for i in 0..6 {
                self.cov[i][i] = 1e-9;
            }
            return 0.0;
        }
        let x = self.state[0];
        let y = self.state[1];
        let z = self.state[2];
        let vx = self.state[3];
        let vy = self.state[4];
        let vz = self.state[5];

        // 1. Calculate distances from plane to tower (R_t) and receiver (R_r)
        let dx = x - tower_pos[0];
        let dy = y - tower_pos[1];
        let dz = z - tower_pos[2];

        let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
        let r_r = (x * x + y * y + z * z).sqrt().max(1.0);

        // 2. Predicted range rate and Doppler shift
        let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
        let dot_r = (vx * x + vy * y + vz * z) / r_r;
        let range_rate_pred = dot_t + dot_r;

        let lambda = C / fc;
        let z_pred = -range_rate_pred / lambda;

        // 3. Compute Jacobian H (1x6 matrix of partial derivatives of z_pred w.r.t state)
        // Partial derivatives of range rate w.r.t velocity
        let dr_dvx = dx / r_t + x / r_r;
        let dr_dvy = dy / r_t + y / r_r;
        let dr_dvz = dz / r_t + z / r_r;

        // Partial derivatives of range rate w.r.t position
        let d_t_x = vx * (r_t.powi(2) - dx * dx) / r_t.powi(3)
            - vy * (dx * dy) / r_t.powi(3)
            - vz * (dx * dz) / r_t.powi(3);
        let d_t_y = -vx * (dx * dy) / r_t.powi(3) + vy * (r_t.powi(2) - dy * dy) / r_t.powi(3)
            - vz * (dy * dz) / r_t.powi(3);
        let d_t_z = -vx * (dx * dz) / r_t.powi(3) - vy * (dy * dz) / r_t.powi(3)
            + vz * (r_t.powi(2) - dz * dz) / r_t.powi(3);

        let d_r_x = vx * (r_r.powi(2) - x * x) / r_r.powi(3)
            - vy * (x * y) / r_r.powi(3)
            - vz * (x * z) / r_r.powi(3);
        let d_r_y = -vx * (x * y) / r_r.powi(3) + vy * (r_r.powi(2) - y * y) / r_r.powi(3)
            - vz * (y * z) / r_r.powi(3);
        let d_r_z = -vx * (x * z) / r_r.powi(3) - vy * (y * z) / r_r.powi(3)
            + vz * (r_r.powi(2) - z * z) / r_r.powi(3);

        let dr_dx = d_t_x + d_r_x;
        let dr_dy = d_t_y + d_r_y;
        let dr_dz = d_t_z + d_r_z;

        // Scale by -1/lambda for Doppler Jacobian
        let h = [
            -dr_dx / lambda,
            -dr_dy / lambda,
            -dr_dz / lambda,
            -dr_dvx / lambda,
            -dr_dvy / lambda,
            -dr_dvz / lambda,
        ];

        // 4. Kalman Gain calculation: K = P * H^T * (H * P * H^T + R)^{-1}
        // Compute H * P (1x6 matrix)
        let mut h_p = [0.0f64; 6];
        for c in 0..6 {
            let mut val = 0.0;
            for r in 0..6 {
                val += h[r] * self.cov[r][c];
            }
            h_p[c] = val;
        }

        // Compute H * P * H^T (1x1 scalar)
        let mut h_p_ht = 0.0;
        for i in 0..6 {
            h_p_ht += h_p[i] * h[i];
        }

        // Innovation covariance S
        let s = h_p_ht + self.r;
        if s.is_nan() || s.is_infinite() || s.abs() < 1e-9 {
            return 0.0; // Singular update, abort
        }

        // Compute K = P * H^T / S (6x1 vector)
        let mut k = [0.0f64; 6];
        for r in 0..6 {
            let mut p_ht = 0.0;
            for c in 0..6 {
                p_ht += self.cov[r][c] * h[c];
            }
            k[r] = p_ht / s;
        }

        // 5. Update state: X = X + K * (z_meas - z_pred)
        let innovation = z_meas - z_pred;
        for i in 0..6 {
            self.state[i] += k[i] * innovation;
        }

        // 6. Update covariance using Joseph formulation: P = (I - K * H) * P * (I - K * H)^T + K * R * K^T
        let mut i_kh = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let identity = if r == c { 1.0 } else { 0.0 };
                i_kh[r][c] = identity - k[r] * h[c];
            }
        }

        // temp = (I - K * H) * P
        let mut temp = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut val = 0.0;
                for i in 0..6 {
                    val += i_kh[r][i] * self.cov[i][c];
                }
                temp[r][c] = val;
            }
        }

        // next_cov = temp * (I - K * H)^T + K * R * K^T
        let mut next_cov = [[0.0f64; 6]; 6];
        for r in 0..6 {
            for c in 0..6 {
                let mut val = 0.0;
                for i in 0..6 {
                    val += temp[r][i] * i_kh[c][i];
                }
                next_cov[r][c] = val + k[r] * k[c] * self.r;
            }
        }
        self.cov = next_cov;

        innovation
    }
}

pub struct AppletonHartreeDispersion;

impl AppletonHartreeDispersion {
    pub fn cancel(f1: f64, f2: f64, fd1: f64, fd2: f64) -> f64 {
        if (f1 - f2).abs() < 1e-3 || (f1.abs() > 0.0 && (f1 - f2).abs() / f1.abs() < 1e-9) {
            return fd1;
        }
        let f1_sq = f1 * f1;
        let diff = f1_sq - f2 * f2;
        if diff.abs() < 1e-6 {
            fd1
        } else {
            (f1_sq * fd1 - f1 * f2 * fd2) / diff
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_joseph_covariance_properties() {
        let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
        let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
        
        let tower_pos = [0.0, 0.0, 0.0];
        let fc = 90.9e6;
        let meas = 12.5;

        for _ in 0..50 {
            ekf.predict(0.1);
            ekf.update(&tower_pos, fc, meas);
        }

        // Covariance matrix must remain symmetric
        for r in 0..6 {
            for c in 0..6 {
                assert!((ekf.cov[r][c] - ekf.cov[c][r]).abs() < 1e-9, "Covariance matrix is asymmetric at ({}, {})", r, c);
            }
        }

        // Diagonal entries must remain positive
        for i in 0..6 {
            assert!(ekf.cov[i][i] > 0.0, "Variance at index {} is not positive: {}", i, ekf.cov[i][i]);
        }
    }

    #[test]
    fn test_appleton_hartree_cancellation() {
        let f1 = 89.3e6;
        let f2 = 90.9e6;
        
        let v = 100.0;
        let fd1 = -2.0 * v * f1 / crate::sdr::C;
        let fd2 = -2.0 * v * f2 / crate::sdr::C;
        
        let fd_free = AppletonHartreeDispersion::cancel(f1, f2, fd1, fd2);
        assert!((fd_free - fd1).abs() < 1e-5);
    }

    #[test]
    fn test_ekf_joint_update_correctness() {
        let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
        let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
        let tower_pos = [10.0, 20.0, 5.0];
        let fc = 90.9e6;

        ekf.predict(0.1);

        let r_cov = [[0.5, 0.0], [0.0, 1e-12]];
        let z_doppler = 15.0;
        let z_delay_sec = 1e-5;
        let inn = ekf.update_joint(&tower_pos, fc, z_doppler, z_delay_sec, r_cov);

        assert!(inn >= 0.0);

        for r in 0..6 {
            for c in 0..6 {
                assert!((ekf.cov[r][c] - ekf.cov[c][r]).abs() < 1e-9);
            }
            assert!(ekf.cov[r][r] > 0.0);
        }
    }

    #[test]
    fn test_ekf_stare_mode() {
        let init_state = [1000.0, 2000.0, 500.0, 50.0, -20.0, 5.0];
        let mut ekf = BistaticEkf::new(init_state, 100.0, 10.0, 1.0);
        let stare_coords = [500.0, 600.0, 700.0];
        ekf.set_stare_mode(stare_coords, true);

        assert_eq!(ekf.state[0], 500.0);
        assert_eq!(ekf.state[1], 600.0);
        assert_eq!(ekf.state[2], 700.0);
        assert_eq!(ekf.state[3], 0.0);
        assert_eq!(ekf.state[4], 0.0);
        assert_eq!(ekf.state[5], 0.0);

        ekf.predict(1.0);
        assert_eq!(ekf.state[0], 500.0);
        assert_eq!(ekf.state[3], 0.0);
        assert!(ekf.cov[0][0] <= 1e-8);

        let inn = ekf.update(&[0.0, 0.0, 0.0], 90e6, 10.0);
        assert_eq!(inn, 0.0);
        assert_eq!(ekf.state[0], 500.0);

        let inn_j = ekf.update_joint(&[0.0, 0.0, 0.0], 90e6, 10.0, 1e-6, [[1.0, 0.0], [0.0, 1.0]]);
        assert_eq!(inn_j, 0.0);
        assert_eq!(ekf.state[0], 500.0);
    }
}


