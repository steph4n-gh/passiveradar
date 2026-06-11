# Project: Passive Radar Performance Optimizations

## Architecture
- **Ingestion & Loop (`src/main.rs`)**: Runs the main loop, processes SDR data chunks, feeds DDC/canceler/FFT, and updates the EKF `TrackingBank`.
- **Tracking & Bank (`src/tracking/bank.rs`)**: EKF tracks, duplicate prevention, plot-to-track association.
- **DSP (`src/dsp/`)**: Decimation, cross-correlation, Doppler calculation.
- **UI (`src/ui/dashboard.rs`)**: Terminal user interface rendering map and track lists.

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| 1 | M1: Multithreading the CAF | Parallelize the Cross-Ambiguity Function (CAF) delay-bin processing loop using `rayon`. | None | DONE |
| 2 | M2: SIMD FIR Decimation | Optimize multi-stage FIR decimation filters using SIMD (explicitly handling AVX and Neon). | None | DONE |
| 3 | M3: Toggleable GPU Offloading | Implement GPU-accelerated FFTs acting as default, with a clear fallback/toggle mechanism. | None | DONE |
| 4 | M4: Verification | Ensure `cargo test` and `cargo test --test e2e` pass flawlessly. | M1, M2, M3 | DONE |

## Interface Contracts
- `rayon` integrated into the CAF computation loop without breaking public API.
- FIR decimator optimized using portable SIMD or feature flags without changing public API.
- GPU FFT offloading acts as default but can be toggled via feature flag, CLI argument, or runtime fallback.

## Code Layout
- `src/main.rs` - Main ingestion, event loop.
- `src/dsp/` - DSP pipeline (`caf.rs`, `decimate.rs`, `fft.rs`).
- `src/tracking/` - Tracker, target re-identification.
- `src/ui/` - TUI rendering.
| 5 | M5: Decoupled UI Refresh Rates | Implement decoupled refresh rate throttling for Dashboard UI components. | None | DONE |
