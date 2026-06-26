use serde::{Deserialize, Serialize};

/// A serializable track report sent between fusion nodes.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TrackReport {
    pub node_id: String,
    pub track_id: u32,
    pub state: [f64; 6],           // [x, y, z, vx, vy, vz]
    pub covariance: [[f64; 6]; 6],
    pub timestamp_ms: u64,
    pub classification: String,
}

// ---------------------------------------------------------------------------
// 6×6 matrix helpers (no external deps)
// ---------------------------------------------------------------------------

/// Trace of a 6×6 matrix.
fn trace_6x6(m: &[[f64; 6]; 6]) -> f64 {
    m[0][0] + m[1][1] + m[2][2] + m[3][3] + m[4][4] + m[5][5]
}

/// Element-wise addition of two 6×6 matrices.
fn mat_add_6x6(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut c = [[0.0f64; 6]; 6];
    for r in 0..6 {
        for k in 0..6 {
            c[r][k] = a[r][k] + b[r][k];
        }
    }
    c
}

/// Scalar multiplication of a 6×6 matrix.
fn mat_scale_6x6(m: &[[f64; 6]; 6], s: f64) -> [[f64; 6]; 6] {
    let mut out = [[0.0f64; 6]; 6];
    for r in 0..6 {
        for c in 0..6 {
            out[r][c] = m[r][c] * s;
        }
    }
    out
}

/// Multiply two 6×6 matrices: C = A * B.
#[allow(dead_code)]
fn mat_mul_6x6(a: &[[f64; 6]; 6], b: &[[f64; 6]; 6]) -> [[f64; 6]; 6] {
    let mut c = [[0.0f64; 6]; 6];
    for r in 0..6 {
        for k in 0..6 {
            let a_rk = a[r][k];
            for j in 0..6 {
                c[r][j] += a_rk * b[k][j];
            }
        }
    }
    c
}

/// Multiply a 6×6 matrix by a 6-vector.
fn mat_vec_6(m: &[[f64; 6]; 6], v: &[f64; 6]) -> [f64; 6] {
    let mut out = [0.0f64; 6];
    for r in 0..6 {
        for c in 0..6 {
            out[r] += m[r][c] * v[c];
        }
    }
    out
}

/// In-place 6×6 matrix inversion via Gauss-Jordan elimination.
/// Returns `None` if the matrix is singular.
fn invert_6x6(m: &[[f64; 6]; 6]) -> Option<[[f64; 6]; 6]> {
    // Augment [m | I]
    let mut aug = [[0.0f64; 12]; 6];
    for r in 0..6 {
        for c in 0..6 {
            aug[r][c] = m[r][c];
        }
        aug[r][6 + r] = 1.0;
    }

    for col in 0..6 {
        // Partial pivoting – find row with largest absolute value in column
        let mut max_val = aug[col][col].abs();
        let mut max_row = col;
        for row in (col + 1)..6 {
            let v = aug[row][col].abs();
            if v > max_val {
                max_val = v;
                max_row = row;
            }
        }
        if max_val < 1e-30 {
            return None; // singular
        }
        if max_row != col {
            aug.swap(col, max_row);
        }

        // Scale pivot row
        let pivot = aug[col][col];
        for j in 0..12 {
            aug[col][j] /= pivot;
        }

        // Eliminate column in all other rows
        for row in 0..6 {
            if row == col {
                continue;
            }
            let factor = aug[row][col];
            for j in 0..12 {
                aug[row][j] -= factor * aug[col][j];
            }
        }
    }

    let mut inv = [[0.0f64; 6]; 6];
    for r in 0..6 {
        for c in 0..6 {
            inv[r][c] = aug[r][6 + c];
        }
    }
    Some(inv)
}

// ---------------------------------------------------------------------------
// Covariance Intersection fusion engine
// ---------------------------------------------------------------------------

/// Covariance Intersection fusion engine.
///
/// Fuses two track estimates without requiring cross-covariance information,
/// making it ideal for distributed multi-node passive radar track fusion.
pub struct CovarianceIntersection;

impl CovarianceIntersection {
    /// Fuse two track estimates using Covariance Intersection.
    ///
    /// Solves: ω* = argmin_ω Tr((ω·P_A⁻¹ + (1-ω)·P_B⁻¹)⁻¹)
    /// via golden-section search over ω ∈ [0, 1].
    ///
    /// Returns (fused_state, fused_covariance).
    ///
    /// # Panics
    ///
    /// Panics if either input covariance matrix is singular.
    pub fn fuse(
        state_a: &[f64; 6],
        cov_a: &[[f64; 6]; 6],
        state_b: &[f64; 6],
        cov_b: &[[f64; 6]; 6],
    ) -> ([f64; 6], [[f64; 6]; 6]) {
        let inv_a = invert_6x6(cov_a).expect("cov_a is singular");
        let inv_b = invert_6x6(cov_b).expect("cov_b is singular");

        // Cost function: trace of fused covariance at a given ω
        let cost = |w: f64| -> f64 {
            let combined = mat_add_6x6(&mat_scale_6x6(&inv_a, w), &mat_scale_6x6(&inv_b, 1.0 - w));
            match invert_6x6(&combined) {
                Some(p_fused) => trace_6x6(&p_fused),
                None => f64::MAX,
            }
        };

        // Golden-section search for optimal ω
        let gr = (5.0_f64.sqrt() - 1.0) / 2.0; // golden ratio conjugate
        let mut a = 0.0_f64;
        let mut b = 1.0_f64;
        let mut c = b - gr * (b - a);
        let mut d = a + gr * (b - a);

        for _ in 0..20 {
            if cost(c) < cost(d) {
                b = d;
            } else {
                a = c;
            }
            c = b - gr * (b - a);
            d = a + gr * (b - a);
        }
        let omega = (a + b) / 2.0;

        // Compute fused covariance and state
        let combined_inv =
            mat_add_6x6(&mat_scale_6x6(&inv_a, omega), &mat_scale_6x6(&inv_b, 1.0 - omega));
        let p_fused = invert_6x6(&combined_inv).expect("fused information matrix is singular");

        let info_a = mat_vec_6(&inv_a, state_a);
        let info_b = mat_vec_6(&inv_b, state_b);
        let mut info_combined = [0.0f64; 6];
        for i in 0..6 {
            info_combined[i] = omega * info_a[i] + (1.0 - omega) * info_b[i];
        }
        let x_fused = mat_vec_6(&p_fused, &info_combined);

        (x_fused, p_fused)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn identity_6x6() -> [[f64; 6]; 6] {
        let mut m = [[0.0f64; 6]; 6];
        for i in 0..6 {
            m[i][i] = 1.0;
        }
        m
    }

    fn diagonal_6x6(diag: [f64; 6]) -> [[f64; 6]; 6] {
        let mut m = [[0.0f64; 6]; 6];
        for i in 0..6 {
            m[i][i] = diag[i];
        }
        m
    }

    #[test]
    fn test_identity_inversion() {
        let eye = identity_6x6();
        let inv = invert_6x6(&eye).expect("identity must be invertible");
        for r in 0..6 {
            for c in 0..6 {
                let expected = if r == c { 1.0 } else { 0.0 };
                assert!(
                    (inv[r][c] - expected).abs() < 1e-12,
                    "inv(I)[{r}][{c}] = {} != {expected}",
                    inv[r][c]
                );
            }
        }
    }

    #[test]
    fn test_ci_identical_tracks() {
        let state = [100.0, 200.0, 300.0, 10.0, -5.0, 2.0];
        let cov = diagonal_6x6([4.0, 4.0, 4.0, 1.0, 1.0, 1.0]);

        let (x_fused, p_fused) = CovarianceIntersection::fuse(&state, &cov, &state, &cov);

        // Fused state must equal both inputs (they are identical)
        for i in 0..6 {
            assert!(
                (x_fused[i] - state[i]).abs() < 1e-8,
                "fused state[{i}] = {} != {}",
                x_fused[i],
                state[i]
            );
        }
        // Fused covariance must equal input covariance (ω = 0.5 is optimal)
        for r in 0..6 {
            for c in 0..6 {
                assert!(
                    (p_fused[r][c] - cov[r][c]).abs() < 1e-8,
                    "fused cov[{r}][{c}] = {} != {}",
                    p_fused[r][c],
                    cov[r][c]
                );
            }
        }
    }

    #[test]
    fn test_ci_reduces_uncertainty() {
        let state_a = [100.0, 200.0, 300.0, 10.0, -5.0, 2.0];
        let cov_a = diagonal_6x6([10.0, 10.0, 10.0, 3.0, 3.0, 3.0]);

        let state_b = [102.0, 198.0, 301.0, 9.5, -5.5, 2.1];
        let cov_b = diagonal_6x6([8.0, 8.0, 8.0, 2.0, 2.0, 2.0]);

        let (_, p_fused) = CovarianceIntersection::fuse(&state_a, &cov_a, &state_b, &cov_b);

        let tr_fused = trace_6x6(&p_fused);
        let tr_a = trace_6x6(&cov_a);
        let tr_b = trace_6x6(&cov_b);
        let tr_min = tr_a.min(tr_b);

        // CI with a single scalar ω minimises trace but may have small numerical
        // slack from the golden-section search.
        assert!(
            tr_fused <= tr_min + 0.01,
            "Tr(P_fused) = {tr_fused} > min(Tr(P_A), Tr(P_B)) = {tr_min}"
        );
    }

    #[test]
    fn test_track_report_serialization() {
        let state = [100.0, 200.0, 300.0, 10.0, -5.0, 2.0];
        let cov = diagonal_6x6([4.0, 4.0, 4.0, 1.0, 1.0, 1.0]);
        let report = TrackReport {
            node_id: "node-1".to_string(),
            track_id: 42,
            state,
            covariance: cov,
            timestamp_ms: 1718912345000,
            classification: "Airliner".to_string(),
        };

        let serialized = serde_json::to_string(&report).expect("Failed to serialize TrackReport");
        let deserialized: TrackReport = serde_json::from_str(&serialized).expect("Failed to deserialize TrackReport");

        assert_eq!(deserialized.node_id, "node-1");
        assert_eq!(deserialized.track_id, 42);
        assert_eq!(deserialized.state, state);
        assert_eq!(deserialized.covariance, cov);
        assert_eq!(deserialized.timestamp_ms, 1718912345000);
        assert_eq!(deserialized.classification, "Airliner");
    }

    #[test]
    fn test_inject_fused_track_integration() {
        use crate::tracking::bank::TrackingBank;
        let mut bank = TrackingBank::new();

        let state = [1000.0, 2000.0, 3000.0, 10.0, -5.0, 2.0];
        let cov = diagonal_6x6([4.0, 4.0, 4.0, 1.0, 1.0, 1.0]);

        // Inject first report (should spawn a new target)
        bank.inject_fused_track(state, cov, "node-1".to_string());
        assert_eq!(bank.targets.len(), 1);
        let target = &bank.targets[0];
        assert_eq!(target.ekf.state, state);
        assert_eq!(target.tracking_towers, vec!["node-1".to_string()]);

        // Inject second report correlating within gate (should update target state/covariance)
        let updated_state = [1001.0, 2002.0, 3001.0, 10.5, -4.8, 1.9];
        let updated_cov = diagonal_6x6([2.0, 2.0, 2.0, 0.5, 0.5, 0.5]);
        bank.inject_fused_track(updated_state, updated_cov, "node-2".to_string());
        assert_eq!(bank.targets.len(), 1); // still only 1 target
        let target = &bank.targets[0];
        assert_eq!(target.ekf.state, updated_state);
        assert_eq!(target.ekf.cov, updated_cov);
        assert!(target.tracking_towers.contains(&"node-1".to_string()));
        assert!(target.tracking_towers.contains(&"node-2".to_string()));
    }
}
