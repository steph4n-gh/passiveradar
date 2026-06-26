use rand::Rng;
use rand_distr::{Distribution, Normal};

/// Computes the Monna map of a p-adic integer (represented as u64) to a real value in [0, 1).
/// Formula: sum_{i=0} d_i * p^{-i-1}
pub fn monna_map(mut val: u64, p: u64) -> f64 {
    if p < 2 {
        return 0.0;
    }
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

/// Computes the inverse Monna map of a real value in [0, 1) back to a p-adic integer u64.
pub fn inverse_monna_map(mut x: f64, p: u64, precision: usize) -> u64 {
    if p < 2 {
        return 0;
    }
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

/// Computes the p-adic (ultrametric) distance between two p-adic integers.
/// d_p(a, b) = p^{-v_p(a-b)}
pub fn p_adic_distance(a: u64, b: u64, p: u64) -> f64 {
    if a == b {
        return 0.0;
    }
    let diff = if a > b { a - b } else { b - a };
    let mut temp = diff;
    let mut vp = 0;
    while temp % p == 0 {
        vp += 1;
        temp /= p;
    }
    1.0 / (p as f64).powi(vp)
}

/// Discretization of the Vladimirov fractional derivative over a discrete historical trajectory.
/// Formula: (D^alpha f)(x_i) = C(p, alpha) * sum_{j != i} (f(x_i) - f(x_j)) / |x_i - x_j|_p^{alpha+1} * mu(x_j)
pub fn compute_vladimirov_derivative(x: &[f64]) -> Vec<f64> {
    let n = x.len();
    let mut deriv = vec![0.0; n];
    if n <= 1 {
        return deriv;
    }
    let p = 2; // Prime base
    let alpha = 0.5;

    // Haar measure scale (approximated as 1/n)
    let measure = 1.0 / (n as f64);

    // C(p, alpha) = (p^alpha - 1) / (1 - p^{-alpha-1})
    let p_alpha = (p as f64).powf(alpha);
    let p_neg_alpha_minus_1 = (p as f64).powf(-alpha - 1.0);
    let c_const = (p_alpha - 1.0) / (1.0 - p_neg_alpha_minus_1);

    // Precompute 2^(1.5 * vp) table for vp in 0..64
    let mut adic_pow_table = [0.0f64; 64];
    for vp in 0..64 {
        adic_pow_table[vp] = 2.0f64.powf(1.5 * vp as f64);
    }

    for i in 0..n {
        let mut sum = 0.0;
        for j in 0..n {
            if i != j {
                let diff = (i as isize - j as isize).unsigned_abs();
                let vp = diff.trailing_zeros() as usize;
                let weight = unsafe { *adic_pow_table.get_unchecked(vp) };
                sum += (x[i] - x[j]) * weight;
            }
        }
        deriv[i] = c_const * sum * measure;
    }
    deriv
}

/// Mappings of WGS84 Cartesian ECEF state variables into Adelic Continuous Ring space
pub fn map_to_adelic_ring(coord: &[f64; 6]) -> Vec<f64> {
    let mut result = coord.to_vec();
    let primes = [2, 3, 5, 7];
    // Map position (coord[0..3]) and velocity (coord[3..6])
    for &p in &primes {
        for j in 0..6 {
            // Scale and clamp variable to [0, 1) range
            let scale = if j < 3 { 150_000.0 } else { 500.0 }; // positions up to 150km, velocities up to 500m/s
            let normalized = ((coord[j].abs()) / scale).min(0.99999);
            let val = inverse_monna_map(normalized, p, 16);
            let mapped = monna_map(val, p);
            result.push(mapped);
        }
    }
    result
}

// =========================================================================
// Adelic Ring Vladimirov-Langevin Optimizer
// =========================================================================
pub struct AdelicLangevinOptimizer {
    primes: Vec<u64>,
    pub rng: rand::rngs::StdRng,
}

impl AdelicLangevinOptimizer {
    pub fn new() -> Self {
        use rand::SeedableRng;
        Self {
            primes: vec![2, 3, 5, 7],
            #[cfg(test)]
            rng: rand::rngs::StdRng::seed_from_u64(42),
            #[cfg(not(test))]
            rng: rand::rngs::StdRng::from_entropy(),
        }
    }

    /// Run stochastic p-adic Langevin optimization to fit the aircraft 3D state vector [x, y, z, vx, vy, vz].
    /// `cost_fn` returns the Residual Sum of Squares (RSS).
    /// `initial_state` is the starting guess.
    /// `bounds` are the min/max limits for [x, y, z, vx, vy, vz].
    pub fn optimize<F>(
        &mut self,
        initial_state: [f64; 6],
        bounds: &[(f64, f64); 6],
        mut cost_fn: F,
        steps: usize,
    ) -> ([f64; 6], f64)
    where
        F: FnMut(&[f64; 6]) -> f64,
    {
        let mut current_state = initial_state;
        let mut current_rss = cost_fn(&current_state);

        let mut best_state = current_state;
        let mut best_rss = current_rss;

        let mut lr = 0.3;
        let mut noise_std = 0.02;
        let mut rss_history = Vec::with_capacity(steps);
        let normal_dist = Normal::new(0.0, 1.0).unwrap();

        for step in 0..steps {
            rss_history.push(current_rss);

            // 1. Compute numerical gradient
            let mut grad = [0.0; 6];
            for i in 0..6 {
                let eps = if i < 3 { 100.0 } else { 0.5 }; // perturbation for position (meters) or velocity (m/s)

                let mut state_plus = current_state;
                state_plus[i] += eps;
                let rss_plus = cost_fn(&state_plus);

                let mut state_minus = current_state;
                state_minus[i] -= eps;
                let rss_minus = cost_fn(&state_minus);

                grad[i] = (rss_plus - rss_minus) / (2.0 * eps);
            }

            // 2. Perform Euclidean Langevin step: X_new = X - lr * sign(grad) + noise
            let mut next_state = current_state;
            for i in 0..6 {
                let grad_sign = if grad[i].is_nan() {
                    0.0
                } else {
                    grad[i].signum()
                };
                let scale = if i < 3 { 1500.0 } else { 15.0 }; // step size scale
                let noise = normal_dist.sample(&mut self.rng) * noise_std * scale;

                next_state[i] = current_state[i] - lr * grad_sign * scale + noise;
                // Clamp to bounds
                next_state[i] = next_state[i].clamp(bounds[i].0, bounds[i].1);
            }

            let next_rss = cost_fn(&next_state);
            if next_rss < current_rss {
                current_state = next_state;
                current_rss = next_rss;
            }

            // 3. Adelic Jump: conversion to p-adic tree and branch hopping
            let p = self.primes[step % self.primes.len()];
            let mut jump_state = current_state;

            for i in 0..6 {
                let range = bounds[i].1 - bounds[i].0;
                if range > 0.0 {
                    let val_normalized =
                        ((current_state[i] - bounds[i].0) / range).clamp(0.0, 0.99999);
                    let p_adic_val = inverse_monna_map(val_normalized, p, 16);

                    // Add p-adic perturbation (branch hopping)
                    // Represents Vladimirov fractional diffusion jumps
                    let perturb_scale = 4;
                    let perturbation =
                        (self.rng.gen::<f64>() * (p as f64).powi(perturb_scale)) as u64;
                    let perturbed_p_adic = p_adic_val.wrapping_add(perturbation);

                    let val_jump_normalized = monna_map(perturbed_p_adic, p);
                    let val_jump = bounds[i].0 + val_jump_normalized * range;
                    jump_state[i] = val_jump.clamp(bounds[i].0, bounds[i].1);
                }
            }

            // Evaluate cost at the Adelic Jump point
            let jump_rss = cost_fn(&jump_state);
            if jump_rss < current_rss {
                // Accept the Adelic jump! We successfully tunneled over a local minimum
                current_state = jump_state;
                current_rss = jump_rss;
            }

            // Update global best
            if current_rss < best_rss {
                best_rss = current_rss;
                best_state = current_state;
            }

            // Early stopping check
            if best_rss < 5.0 {
                break;
            }

            // Decay learning rate and noise standard deviation
            lr *= 0.995;
            noise_std *= 0.99;
        }

        // Print Vladimirov derivative of optimization path for logging
        if cfg!(debug_assertions) {
            let _v_derivs = compute_vladimirov_derivative(&rss_history);
        }

        (best_state, best_rss)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_monna_mapping() {
        let p = 3;
        // Map integer to p-adic real
        let val = 12u64; // 12 in base 3 is 110 (1*9 + 1*3 + 0*1)
                         // Monna map of 12 (reversed order digits: 0, 1, 1):
                         // 0 * 3^-1 + 1 * 3^-2 + 1 * 3^-3 = 0 + 1/9 + 1/27 = 4/27 ≈ 0.148148
        let mapped = monna_map(val, p);
        assert!((mapped - 4.0 / 27.0).abs() < 1e-6);

        // Inverse Monna map
        let x = 4.0 / 27.0;
        let val_back = inverse_monna_map(x, p, 3);
        assert_eq!(val_back, val);
    }

    #[test]
    fn test_p_adic_distance() {
        let p = 2;
        // valuation of (10 - 2) = 8 = 2^3 -> vp = 3 -> distance = 2^-3 = 0.125
        assert_eq!(p_adic_distance(10, 2, p), 0.125);
        // valuation of (5 - 4) = 1 = 2^0 -> vp = 0 -> distance = 2^-0 = 1.0
        assert_eq!(p_adic_distance(5, 4, p), 1.0);
        // distance of equal values is 0
        assert_eq!(p_adic_distance(4, 4, p), 0.0);
    }

    #[test]
    fn test_vladimirov_derivative() {
        let trajectory = vec![0.1, 0.2, 0.4, 0.8, 1.6];
        let derivs = compute_vladimirov_derivative(&trajectory);
        assert_eq!(derivs.len(), 5);
        // Check that derivatives are computable numbers
        for d in derivs {
            assert!(!d.is_nan());
        }
    }

    #[test]
    fn test_adelic_optimizer() {
        use rand::SeedableRng;
        let mut opt = AdelicLangevinOptimizer::new();
        opt.rng = rand::rngs::StdRng::seed_from_u64(42);
        // Minimize a simple quadratic sphere: f(x) = sum (x_i - target_i)^2
        let target = [10_000.0, -5000.0, 8000.0, 100.0, -120.0, 10.0];
        let cost_fn = |state: &[f64; 6]| {
            let mut sum = 0.0;
            for i in 0..6 {
                sum += (state[i] - target[i]).powi(2);
            }
            sum
        };

        let initial_state = [0.0, 0.0, 5000.0, 0.0, 0.0, 0.0];
        let bounds = [
            (-50_000.0, 50_000.0),
            (-50_000.0, 50_000.0),
            (0.0, 15_000.0),
            (-300.0, 300.0),
            (-300.0, 300.0),
            (-50.0, 50.0),
        ];

        let (best_state, best_rss) = opt.optimize(initial_state, &bounds, cost_fn, 500);
        // Verify that the optimizer decreased the cost function significantly
        let initial_rss = cost_fn(&initial_state);
        assert!(best_rss < initial_rss);
        // Verify state is close-ish to target
        for i in 0..6 {
            let error = (best_state[i] - target[i]).abs();
            // Positions should be within 6km, velocities within 40m/s for a 500-step search
            let threshold = if i < 3 { 6000.0 } else { 40.0 };
            assert!(
                error < threshold,
                "State index {} error = {} is above threshold {}",
                i,
                error,
                threshold
            );
        }
    }
}
