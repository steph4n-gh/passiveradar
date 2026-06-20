# Passive Radar DSP and Mathematical Foundations

This document provides a rigorous mathematical and algorithmic reference for the advanced digital signal processing (DSP), topological analysis, and optimization routines implemented in the Passive Radar system.

---

## Table of Contents
1. [Adelic Langevin Optimization](#1-adelic-langevin-optimization)
2. [Čech Cohomology Target Filtering](#2-čech-cohomology-target-filtering)
3. [Fractional Delay Filtering (Fourier Shift Theorem)](#3-fractional-delay-filtering-fourier-shift-theorem)
4. [Filtered Backprojection Tomography](#4-filtered-backprojection-tomography)
5. [Zero-Allocation DSP Architectural Patterns](#5-zero-allocation-dsp-architectural-patterns)

---

## 1. Adelic Langevin Optimization

To fit the 3D bistatic state vector of an aircraft—consisting of position and velocity in WGS84 Cartesian Earth-Centered, Earth-Fixed (ECEF) coordinates $\mathbf{x} = [x, y, z, v_x, v_y, v_z]^T$—the system employs a hybrid stochastic optimizer in the Adelic Continuous Ring space. This optimizer is defined in [src/math/adelic.rs](../src/math/adelic.rs).

### 1.1 p-adic Coordinates and the Monna Map
A $p$-adic integer $x \in \mathbb{Z}_p$ is represented as a formal power series:
$$
x = \sum_{i=0}^{\infty} d_i p^i, \quad d_i \in \{0, 1, \dots, p-1\}
$$
The **Monna Map** $\Phi_p: \mathbb{Z}_p \to [0, 1)$ projects a $p$-adic integer onto a real-valued interval in $[0, 1)$ via:
$$
\Phi_p(x) = \sum_{i=0}^{\infty} d_i p^{-i-1}
$$
Conversely, the **Inverse Monna Map** $\Phi_p^{-1}: [0, 1) \to \mathbb{Z}_p$ maps a real number $y \in [0, 1)$ back to a $p$-adic integer:
$$
\Phi_p^{-1}(y) = \sum_{i=0}^{N-1} d_i p^i
$$
where $N$ is the desired precision level (set to 16 in the codebase).

In the code, these maps are implemented in:
* `monna_map`: projects a `u64` representing a $p$-adic integer to an `f64` in $[0, 1)$.
* `inverse_monna_map`: computes the $p$-adic coefficients from a real value using successive multiplication and flooring.

### 1.2 p-adic Distance
The $p$-adic valuation $v_p(n)$ for an integer $n$ is the exponent of the highest power of $p$ that divides $n$. The ultrametric $p$-adic distance between two integers $a$ and $b$ is defined as:
$$
d_p(a, b) = p^{-v_p(a-b)}
$$
If $a = b$, then $d_p(a, b) = 0$. This distance satisfies the strong (ultrametric) triangle inequality:
$$
d_p(x, y) \le \max\left(d_p(x, z), d_p(z, y)\right)
$$
This is implemented in `p_adic_distance`.

### 1.3 Vladimirov Fractional Derivative
Diffusion processes on $p$-adic trees are governed by the Vladimirov fractional operator. The discretized Vladimirov derivative $D^\alpha$ of order $\alpha > 0$ over a discrete trajectory $\{x_i\}_{i=0}^{n-1}$ is given by:
$$
(D^\alpha f)(x_i) = C(p, \alpha) \sum_{j \neq i} \frac{f(x_i) - f(x_j)}{d_p(i, j)^{\alpha+1}} \mu(x_j)
$$
where $\mu(x_j)$ is the Haar measure (approximated as $\frac{1}{n}$), and the normalization constant $C(p, \alpha)$ is:
$$
C(p, \alpha) = \frac{1 - p^\alpha}{1 - p^{-\alpha-1}}
$$
In `compute_vladimirov_derivative`, the base is set to $p=2$ and the fractional order is $\alpha = 0.5$.

### 1.4 Hybrid Langevin Step with Adelic Jump Tunneling
The optimization loop combines a standard Euclidean Langevin gradient descent step with discrete $p$-adic jumps to bypass local minima. 

1. **Euclidean Langevin Step**:
   The update rule is:
   $$
   X_{new} = X_{current} - \eta \cdot \text{sign}\left(\nabla V(X_{current})\right) + \sigma \cdot \xi
   $$
   where $\eta$ is the learning rate, $V(X)$ is the Residual Sum of Squares (RSS) cost function, $\sigma$ is the noise scaling factor, and $\xi \sim \mathcal{N}(0, 1)$ is a Gaussian perturbation.

2. **Adelic Jump Tunneling**:
   To prevent entrapment in local minima, a prime $p \in \{2, 3, 5, 7\}$ is selected cyclically. The normalized Euclidean state is converted to a $p$-adic representation using $\Phi_p^{-1}$, perturbed in the p-adic tree, and mapped back:
   $$
   x_i^{jump} = \Phi_p\left(\Phi_p^{-1}(x_i^{norm}) \oplus_p \Delta\right)
   $$
   where $\Delta$ is a random integer offset representing Vladimirov fractional diffusion jumps. If the cost of the jumped state is lower than the current state, the jump is accepted:
   $$
   X_{current} \leftarrow X_{jump} \quad \text{if } V(X_{jump}) < V(X_{new})
   $$
This logic is implemented in the `optimize` method of the `AdelicLangevinOptimizer` struct in [src/math/adelic.rs](../src/math/adelic.rs).

---

## 2. Čech Cohomology Target Filtering

In dense multi-transmitter environments, crossing bistatic range-Doppler ellipses can generate "ghost targets" (spurious intersections of range-Doppler contours). The system resolves these ambiguities by modeling the receiver and transmitters as a Čech complex cover and computing obstruction cycles. This is implemented in [src/tracking/bank.rs](../src/tracking/bank.rs) inside `compute_cech_obstruction`.

### 2.1 Spatial-Doppler Intersection Cover
Let $U_i$ be the spatial-Doppler coverage region associated with the $i$-th transmitter tower. The Čech complex represents intersections of these covers. If a target is real, its spatial coordinate $\mathbf{x} = [x, y, z]^T$ and velocity $\mathbf{v} = [v_x, v_y, v_z]^T$ must consistently project to the observed Doppler peaks of all active towers.

### 2.2 Predicted Bistatic Doppler Shift
For a transmitter tower located at $\mathbf{x}_{tx}$ transmitting at carrier frequency $f_c$, the wavelength is:
$$
\lambda = \frac{c}{f_c}
$$
where $c = 299,792,458 \text{ m/s}$ is the speed of light (defined as `crate::sdr::C`).

The bistatic Doppler shift is the sum of the target's radial velocity relative to the transmitter and the target's radial velocity relative to the receiver:
$$
f_D = -\frac{1}{\lambda} \left( \frac{\mathbf{v} \cdot (\mathbf{x} - \mathbf{x}_{tx})}{\|\mathbf{x} - \mathbf{x}_{tx}\|} + \frac{\mathbf{v} \cdot \mathbf{x}}{\|\mathbf{x}\|} \right)
$$
In the code, this is calculated as:
```rust
let r_r = (x * x + y * y + z * z).sqrt().max(1.0);
let dot_r = (vx * x + vy * y + vz * z) / r_r;
// For each tower:
let dx = x - tower_pos[0];
let dy = y - tower_pos[1];
let dz = z - tower_pos[2];
let r_t = (dx * dx + dy * dy + dz * dz).sqrt().max(1.0);
let dot_t = (vx * dx + vy * dy + vz * dz) / r_t;
let pred_doppler = -(dot_t + dot_r) / lambda;
```

### 2.3 Čech Obstruction Calculation
The Čech obstruction $E$ measures the cohomological inconsistency of a track across all visible towers. It is computed by finding the minimum absolute difference between the predicted Doppler shift and the actual detected peaks $\{f_{peak, j, k}\}$ for each tower $j$, then summing them:
$$
E = \sum_{j=1}^{M} \min_{k} \left| f_D^{(j)} - f_{peak, j, k} \right|
$$
If the cumulative discrepancy exceeds the threshold:
$$
E > 300.0 \text{ Hz}
$$
the target is classified as an inconsistent ghost target and pruned (`TrackState::Terminated`). The threshold of $300.0 \text{ Hz}$ represents the maximum allowable cumulative Doppler discrepancy before a track is considered physically impossible given the transmitter locations.

This pruning logic is in [src/tracking/bank.rs](../src/tracking/bank.rs) at line 650.

---

## 3. Fractional Delay Filtering (Fourier Shift Theorem)

In wideband OFDM systems, target delays often fall between discrete sample intervals. The system implements sub-sample delay interpolation using the Fourier Shift Theorem in [src/dsp/isar.rs](../src/dsp/isar.rs) inside `ofdm_fractional_delay_caf`.

### 3.1 Fourier Shift Theorem
Let $x(t)$ be a continuous signal and $X(f) = \mathcal{F}\{x(t)\}$ be its Fourier transform. A time delay of $\tau$ (which can be a fractional number of samples) corresponds to a phase shift in the frequency domain:
$$
\mathcal{F}\{x(t - \tau)\} = X(f) e^{-j 2\pi f \tau}
$$

### 3.2 Implementation details in ofdm_fractional_delay_caf
The cross-ambiguity function (CAF) between the surveillance signal $s[n]$ and reference signal $r[n]$ at fractional sample delay $\tau$ is computed as:
1. Compute the forward FFT of both signals:
   $$
   S_{surv}(f) = \text{FFT}\{s[n]\}, \quad S_{ref}(f) = \text{FFT}\{r[n]\}
   $$
2. Apply the shift theorem in the frequency domain. For each frequency index $k \in [0, N-1]$, normalize the frequency to $f \in [-0.5, 0.5]$:
   $$
   f = \begin{cases} 
   \frac{k}{N} & \text{if } k \le \frac{N}{2} \\ 
   \frac{k - N}{N} & \text{if } k > \frac{N}{2} 
   \end{cases}
   $$
3. Perform conjugate multiplication and apply the fractional phase shift:
   $$
   Y(f) = S_{surv}(f) \cdot S_{ref}^*(f) \cdot e^{j 2\pi f \tau}
   $$
4. Run the Inverse FFT (IFFT) of $Y(f)$ to return to the time domain, yielding the sub-sample cross-correlation sequence:
   $$
   y[n] = \text{IFFT}\{Y(f)\}
   $$
This is implemented in `ofdm_fractional_delay_caf` using `rustfft` for in-place forward and inverse transforms.

---

## 4. Filtered Backprojection Tomography

Inverse Synthetic Aperture Radar (ISAR) and orbital tracking systems reconstruct 2D spatial images from range profiles acquired at different bistatic angles. The system performs this tomographic reconstruction using the Filtered Backprojection (FBP) algorithm in [src/dsp/isar.rs](../src/dsp/isar.rs) inside `backproject_tomography`.

### 4.1 Radon Transform and Ram-Lak Filtering
The Radon transform represents projection data. According to the Projection-Slice Theorem, the 2D image reconstruction requires filtering the projection slices to counter the low-frequency blurring introduced by backprojection (which acts as a $1/|f|$ filter).

The system applies a frequency-domain **Ram-Lak (ramp) filter** $H(f) = |f|$ to each profile:
$$
P_{\theta_i}^{filtered}(t) = \mathcal{F}^{-1}\left\{ \mathcal{F}\{P_{\theta_i}(t)\} \cdot |f| \right\}
$$
In the code:
```rust
// Apply Ram-Lak filter H(f) = |f|
for k in 0..n_bins {
    let f = if k <= n_bins / 2 {
        (k as f32) / (n_bins as f32)
    } else {
        ((n_bins - k) as f32) / (n_bins as f32)
    };
    fft_input[k] = fft_input[k] * f;
}
```

### 4.2 Backprojection and Linear Interpolation
The filtered profiles are smeared back across the spatial grid. For each grid point $(x, y)$, its projection coordinate $\rho$ at a given rotation angle $\theta$ is:
$$
\rho = x \cos\theta + y \sin\theta
$$
Because $\rho$ is continuous, it must be mapped to discrete profile bin indices using linear interpolation:
$$
\text{bin} = \rho \cdot \frac{N_{bins}}{2} + \frac{N_{bins}}{2}
$$
If $\lfloor\text{bin}\rfloor = b$, the interpolated value is:
$$
V = (1 - \text{frac}) \cdot P^{filtered}[b] + \text{frac} \cdot P^{filtered}[b+1]
$$
where $\text{frac} = \text{bin} - b$. Summing these values across all $N_{\theta}$ angles yields the pixel intensity:
$$
I(x, y) = \frac{1}{N_{\theta}} \sum_{i=1}^{N_{\theta}} P_{\theta_i}^{filtered}(\rho_i)
$$
This is implemented in `backproject_tomography`.

---

## 5. Zero-Allocation DSP Architectural Patterns

To sustain real-time throughput on high-rate IQ data without causing GC or memory allocator pauses, the pipeline avoids dynamic heap allocations in hot loops. The following architectural patterns are used:

### 5.1 Decimation Buffer Swapping (`std::mem::take`)
In [src/dsp/decimate.rs](../src/dsp/decimate.rs), the `MultiStageDecimator::process` function utilizes pre-allocated buffers `buf1` and `buf2` to pass intermediate decimation stage outputs. To comply with Rust's borrow checker rules without copying vectors or allocating new memory, the vectors are temporarily moved out of the struct using `std::mem::take`, processed, and then returned:
```rust
let mut buf1 = std::mem::take(&mut self.buf1);
let mut buf2 = std::mem::take(&mut self.buf2);
// ... process stages ...
self.buf1 = buf1;
self.buf2 = buf2;
```
This pattern provides zero-heap-allocation buffer reuse during real-time decimation.

### 5.2 Pre-allocated Matrices
In [src/dsp/caf.rs](../src/dsp/caf.rs), dense Cross-Ambiguity Function (CAF) calculations require 2D arrays. Instead of allocating memory inside parallel loop threads, which causes thread-lock contention on the global allocator, the buffers are allocated sequentially on the main thread:
```rust
let mut r_matrix = vec![vec![FftComplex::new(0.0, 0.0); num_chunks]; max_delay];
let mut scratches = vec![vec![FftComplex::new(0.0, 0.0); scratch_len]; max_delay];
let mut result = vec![vec![0.0f32; num_chunks]; max_delay];
```
This avoids heap allocations during the parallel processing step.

### 5.3 FFT Engine Buffer Reuse
In [src/dsp/fft.rs](../src/dsp/fft.rs), the `FftEngine` struct caches all work vectors:
```rust
pub struct FftEngine {
    // ...
    scratch: Vec<FftComplex<f32>>,
    fft_input: Vec<FftComplex<f32>>,
    magnitude: Vec<f32>,
}
```
These vectors are allocated once at construction. The `next_frame` method updates `fft_input` in-place, processes it using `process_with_scratch` (avoiding internal FFT allocations), and writes the shifted magnitudes directly into `magnitude`.

### 5.4 Scratch Space Reuse and Temporary Field Take
In [src/dsp/tropical.rs](../src/dsp/tropical.rs), `TropicalWaveletCanceller` maintains pre-allocated `scratch`, `approx`, and `background` vectors. When notch filtering stationary spurs, the background envelope is passed to the spur detection function. To satisfy the borrow checker without copying, the background vector is temporarily swapped out:
```rust
let spikes = {
    let bg = std::mem::take(&mut self.background);
    let s = self.detect_spurs_vladimirov(fft_magnitudes, &bg);
    self.background = bg;
    s
};
```
This preserves the zero-allocation invariant of the spectral notch filter.

---

## Related Documents
*   For hardware setup, FM tuning, and troubleshooting details, see [Practical Operations Guide (setup.md)](setup.md).
