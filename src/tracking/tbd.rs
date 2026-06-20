/// A Dynamic Programming (Viterbi) Track-Before-Detect (TBD) algorithm.
/// Integrates weak signal energy along kinematically valid flight paths across
/// Range-Doppler waterfall history to pull targets out of the HackRF's quantization noise floor.
#[derive(Clone)]
pub struct ViterbiTbd {
    num_delay_bins: usize,
    num_doppler_bins: usize,
    history_depth: usize,
    // Range-Doppler history: Vec of frames, where each frame is a 1D flattened matrix [delay * doppler]
    rd_history: Vec<Vec<f32>>,
    // Kinematic constraints
    max_acceleration_mps2: f32,
    max_velocity_mps: f32,
}

impl ViterbiTbd {
    /// Create a new Viterbi TBD engine.
    pub fn new(num_delay_bins: usize, num_doppler_bins: usize, history_depth: usize) -> Self {
        Self {
            num_delay_bins,
            num_doppler_bins,
            history_depth,
            rd_history: Vec::new(),
            max_acceleration_mps2: 40.0, // Limit search to valid physical aircraft maneuvering
            max_velocity_mps: 340.0,     // Speed of sound limit
        }
    }

    /// Push a new Range-Doppler matrix frame into the history buffer.
    pub fn push_frame(&mut self, rd_matrix: Vec<f32>) {
        self.rd_history.push(rd_matrix);
        if self.rd_history.len() > self.history_depth {
            self.rd_history.remove(0);
        }
    }

    /// Run the Viterbi dynamic programming back-track.
    /// Returns the most probable target state sequence (delay, Doppler) over time
    /// along with the accumulated detection score.
    pub fn search_trajectory(&self) -> Option<(Vec<(usize, usize)>, f32)> {
        let n_frames = self.rd_history.len();
        if n_frames < 2 {
            return None;
        }

        let total_states = self.num_delay_bins * self.num_doppler_bins;
        
        // 1. Initialize value function (V) with the log probability emission from the first frame
        // V[t][s] holds the maximum accumulated score at time t in state s
        let mut v = vec![vec![-f32::INFINITY; total_states]; n_frames];
        let mut backpointer = vec![vec![0usize; total_states]; n_frames];

        // Fill t = 0
        for s in 0..total_states {
            if s < self.rd_history[0].len() {
                v[0][s] = self.rd_history[0][s].max(1e-6).ln();
            }
        }

        // 2. Forward dynamic programming step
        for t in 1..n_frames {
            let emission = &self.rd_history[t];
            
            for s_curr in 0..total_states {
                let curr_delay = s_curr / self.num_doppler_bins;
                let curr_doppler = s_curr % self.num_doppler_bins;

                let mut max_val = -f32::INFINITY;
                let mut best_prev = 0;

                // Restrict search space of transition states based on kinematic limits:
                // Targets cannot jump across range/Doppler bins faster than physical limits.
                let search_window_delay = 2; // +/- 2 delay bins
                let search_window_doppler = 3; // +/- 3 Doppler bins

                let min_d = curr_delay.saturating_sub(search_window_delay);
                let max_d = (curr_delay + search_window_delay).min(self.num_delay_bins - 1);

                for d_prev in min_d..=max_d {
                    let min_f = curr_doppler.saturating_sub(search_window_doppler);
                    let max_f = (curr_doppler + search_window_doppler).min(self.num_doppler_bins - 1);

                    for f_prev in min_f..=max_f {
                        let s_prev = d_prev * self.num_doppler_bins + f_prev;
                        let val = v[t - 1][s_prev];
                        
                        if val > max_val {
                            max_val = val;
                            best_prev = s_prev;
                        }
                    }
                }

                // Bellman update: V[t][s] = ln(emission[s]) + max_{s'} V[t-1][s']
                let log_emission = if s_curr < emission.len() {
                    emission[s_curr].max(1e-6).ln()
                } else {
                    -10.0
                };
                
                v[t][s_curr] = log_emission + max_val;
                backpointer[t][s_curr] = best_prev;
            }
        }

        // 3. Find the state with the highest value in the final frame
        let mut best_final_state = 0;
        let mut max_final_val = -f32::INFINITY;
        for s in 0..total_states {
            if v[n_frames - 1][s] > max_final_val {
                max_final_val = v[n_frames - 1][s];
                best_final_state = s;
            }
        }

        if max_final_val == -f32::INFINITY {
            return None;
        }

        // 4. Backtrack to recover the optimal path
        let mut path = vec![(0, 0); n_frames];
        let mut curr_state = best_final_state;
        
        for t in (0..n_frames).rev() {
            let delay = curr_state / self.num_doppler_bins;
            let doppler = curr_state % self.num_doppler_bins;
            path[t] = (delay, doppler);
            curr_state = backpointer[t][curr_state];
        }

        // Return optimal path and normalized score
        let accumulated_score = max_final_val / n_frames as f32;
        Some((path, accumulated_score))
    }
}
