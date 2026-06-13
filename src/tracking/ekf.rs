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
        }
    }

    /// Predict the state forward in time by dt seconds using a constant-velocity linear motion model.
    pub fn predict(&mut self, dt: f64) {
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

    /// Update the state estimate using a bistatic Doppler shift measurement.
    /// `tower_pos`: [x, y, z] of transmitter tower
    /// `fc`: FM carrier frequency of tower
    /// `z_meas`: measured Doppler shift in Hz
    pub fn update(&mut self, tower_pos: &[f64; 3], fc: f64, z_meas: f64) -> f64 {
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
        if s.abs() < 1e-9 {
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
}


