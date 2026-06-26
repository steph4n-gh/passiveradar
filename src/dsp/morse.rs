use std::collections::HashSet;

/// A peak representing a critical point (local maximum) of the simplified Morse complex.
#[derive(Debug, Clone, PartialEq)]
pub struct MorsePeak {
    pub delay: usize,
    pub doppler: usize,
    pub value: f32,
    pub persistence: f32,
}

/// Disjoint Set Union (DSU) for tracking peak component merging.
struct Dsu {
    parent: Vec<usize>,
    peak_val: Vec<f32>,
    peak_coords: Vec<(usize, usize)>,
}

impl Dsu {
    fn new(size: usize) -> Self {
        Self {
            parent: (0..size).collect(),
            peak_val: vec![0.0; size],
            peak_coords: vec![(0, 0); size],
        }
    }

    fn find(&mut self, i: usize) -> usize {
        let mut root = i;
        while root != self.parent[root] {
            root = self.parent[root];
        }
        let mut curr = i;
        while curr != root {
            let nxt = self.parent[curr];
            self.parent[curr] = root;
            curr = nxt;
        }
        root
    }

    fn union(&mut self, i: usize, j: usize, saddle_val: f32, dead_peaks: &mut Vec<MorsePeak>) -> bool {
        let root_i = self.find(i);
        let root_j = self.find(j);
        if root_i == root_j {
            return false;
        }

        let val_i = self.peak_val[root_i];
        let val_j = self.peak_val[root_j];

        if val_i >= val_j {
            // Component j merges into component i.
            // Component j's peak is dead.
            let dead_coords = self.peak_coords[root_j];
            dead_peaks.push(MorsePeak {
                delay: dead_coords.0,
                doppler: dead_coords.1,
                value: val_j,
                persistence: val_j - saddle_val,
            });
            self.parent[root_j] = root_i;
        } else {
            // Component i merges into component j.
            // Component i's peak is dead.
            let dead_coords = self.peak_coords[root_i];
            dead_peaks.push(MorsePeak {
                delay: dead_coords.0,
                doppler: dead_coords.1,
                value: val_i,
                persistence: val_i - saddle_val,
            });
            self.parent[root_i] = root_j;
        }
        true
    }
}

/// Prunes a 2D Range-Doppler grid using Discrete Morse Theory / 0D Persistent Homology.
/// Collapses noise peaks with persistence below the threshold.
pub fn prune_discrete_morse(
    rd_grid: &[Vec<f32>],
    persistence_threshold: f32,
) -> Vec<MorsePeak> {
    let h = rd_grid.len();
    if h == 0 {
        return Vec::new();
    }
    let w = rd_grid[0].len();
    if w == 0 || h > 256 || w > 256 {
        return Vec::new();
    }
    // Check for row consistency
    for row in rd_grid {
        if row.len() != w {
            return Vec::new();
        }
    }

    // 1. Collect and sort all cells in descending order of value
    let mut cells = Vec::with_capacity(h * w);
    for r in 0..h {
        for c in 0..w {
            cells.push((r, c, rd_grid[r][c]));
        }
    }
    cells.sort_by(|a, b| b.2.partial_cmp(&a.2).unwrap_or(std::cmp::Ordering::Equal));

    let mut dsu = Dsu::new(h * w);
    let mut active = vec![false; h * w];
    let mut dead_peaks = Vec::new();

    // 2. Process cells in sorted order
    for &(r, c, val) in &cells {
        let idx = r * w + c;
        active[idx] = true;
        dsu.peak_val[idx] = val;
        dsu.peak_coords[idx] = (r, c);

        // Find active 8-neighbors
        let neighbors = [
            (-1, -1), (-1, 0), (-1, 1),
            (0, -1),           (0, 1),
            (1, -1),  (1, 0),  (1, 1),
        ];

        let mut neighbor_roots = HashSet::new();
        for &(dr, dc) in &neighbors {
            let nr = r as isize + dr;
            let nc = c as isize + dc;
            if nr >= 0 && nr < h as isize && nc >= 0 && nc < w as isize {
                let n_idx = (nr as usize) * w + (nc as usize);
                if active[n_idx] {
                    neighbor_roots.insert(dsu.find(n_idx));
                }
            }
        }

        // Merge this cell with all its active neighbors
        for n_root in neighbor_roots {
            dsu.union(idx, n_root, val, &mut dead_peaks);
        }
    }

    // The remaining root after union-find representing the global maximum
    let global_root = dsu.find(cells[0].0 * w + cells[0].1);
    let global_coords = dsu.peak_coords[global_root];
    let global_val = dsu.peak_val[global_root];
    
    // We add the global maximum as a peak with infinite persistence (represented by its value)
    let mut final_peaks = Vec::new();
    final_peaks.push(MorsePeak {
        delay: global_coords.0,
        doppler: global_coords.1,
        value: global_val,
        persistence: global_val,
    });

    // Add other dead peaks that exceed the persistence threshold
    for peak in dead_peaks {
        if peak.persistence >= persistence_threshold {
            final_peaks.push(peak);
        }
    }

    // Sort by value descending
    final_peaks.sort_by(|a, b| b.value.partial_cmp(&a.value).unwrap_or(std::cmp::Ordering::Equal));
    final_peaks
}
