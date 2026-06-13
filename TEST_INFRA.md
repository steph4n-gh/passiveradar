# Passive Radar E2E Test Infrastructure (`TEST_INFRA.md`)

This document outlines the architecture, layout, and execution mechanisms for the End-to-End (E2E) testing framework designed for the Phase 3 (Universal Vibrometer JEM Upgrade) and Phase 4 (Omni-Sensor Operational Modes) features of `passiveradar`.

## 1. Test Architecture

The E2E testing framework is designed as an **opaque-box** testing suite. It verifies the functionality of the new Phase 3 & 4 features without relying on internal Rust structure details. The interfaces validated are:

1. **WebSocket Server**: Tests interact with the vibrometer and omni-sensor modes via JSON-RPC/API messages sent over WebSocket connections. On startup, the application defaults to port `8085` but can accept a custom port via the `--port <port>` CLI option.
2. **Command Line Interface (CLI)**: Tests execute the application binary using varying flags, environment variables, width/height parameters, `--port` overrides, and test scripts.
3. **TUI Interactive Frame Dumps**: Tests provide keypress mock scripts using the `--test-script` flag and assert correctness using visual frame output dumps in the `--test-out` directory.

### Dynamic Port Allocation & Sequential Execution
To avoid local port conflicts, the test suite dynamically selects an available TCP port (using `std::net::TcpListener::bind("127.0.0.1:0")`) and passes it to the spawned process via `--port <port>`. WebSocket clients then connect to `ws://127.0.0.1:<port>`. A global mutex is also used in `tests/phase3_phase4_e2e.rs` to synchronize the lifecycle of spawned processes, ensuring WebSocket tests do not experience timing or resource conflicts. Sequential testing (`--test-threads=1`) remains the standard verification runner command.

### Grouped WebSocket Compound Tests
To optimize test suite execution speed and avoid spawning the application binary for every single WebSocket assertion, individual WebSocket tests are grouped into compound tests. This reduces the total binary spawn and warmup cycles, resulting in a dramatic speedup of the E2E verification.

---

## 2. Test Suite Layout

All E2E test cases for Phase 3 & 4 features are implemented in the Rust integration test file:
* **Path**: `tests/phase3_phase4_e2e.rs`

The test suite covers four distinct testing tiers:

### Tier 1: Feature Coverage (5 cases per feature, 8 features = 40 cases)
Verify the core operational requirements of each feature.
1. **Continuous Phase Unwrapping**:
   - `test_t1_phase_unwrap_normal`: Normal phase unwrapping correctness (monotonic growth, displacement output).
   - `test_t1_phase_unwrap_positive_wrap`: Correct handling of positive phase wraps ($-\pi$ to $\pi$).
   - `test_t1_phase_unwrap_negative_wrap`: Correct handling of negative phase wraps ($\pi$ to $-\pi$).
   - `test_t1_phase_unwrap_waveform`: Instantaneous displacement correctness on sinusoidal phase input.
   - `test_t1_phase_unwrap_ws_sync`: Enabling phase unwrapping via WebSocket returns status and starts streaming displacement.
2. **Cepstral Analysis**:
   - `test_t1_cepstrum_rpm_detection`: Basic RPM/fundamental frequency detection from mock harmonic signal.
   - `test_t1_cepstrum_magnitude`: Verifying IFFT of log FFT magnitude calculation structure.
   - `test_t1_cepstrum_dynamic_tracking`: Real-time tracking as fundamental RPM frequency changes.
   - `test_t1_cepstrum_ws_sync`: Retrieving cepstral output data via WebSocket command.
   - `test_t1_cepstrum_harmonic_collapse`: Verify that harmonic overtones are successfully collapsed to a single peak.
3. **Programmable CIC Decimation Banks**:
   - `test_t1_cic_seismic_mode`: Seismic mode decimation verification (0.01 - 5 Hz bounds).
   - `test_t1_cic_rotary_mode`: Rotary mode decimation verification (10 - 250 Hz bounds).
   - `test_t1_cic_acoustic_mode`: Acoustic mode decimation verification (300 - 4000 Hz bounds).
   - `test_t1_cic_mode_switching`: WS command `SetVibrometerMode` successfully toggles active bank.
   - `test_t1_cic_telemetry`: Verifying active mode is reported in the telemetry JSON payload.
4. **Deep-Integration Master EKF**:
   - `test_t1_ekf_state_init`: Initialize master EKF with correct 3D geodetic position/velocity.
   - `test_t1_ekf_multi_tower`: Simultaneous update using raw IQ phase/Doppler peak errors across all active towers.
   - `test_t1_ekf_predict_propagation`: State propagation/coasting verification when measurements are sparse.
   - `test_t1_ekf_convergence`: Master EKF converges faster than independent single-tower EKFs under noise.
   - `test_t1_ekf_stability`: Stability check (covariance matrix remains positive-definite under high noise).
5. **Wi-Fi Respiration Tracker**:
   - `test_t1_wifi_multipath_cancellation`: `EcaCanceler` successfully cancels static multipath reflections at 2.4 GHz.
   - `test_t1_wifi_spike_pruning`: `TropicalWaveletCanceller` prunes bursty packet spikes.
   - `test_t1_wifi_breathing_extraction`: Correctly extracts 0.3 Hz breathing signature of human occupants.
   - `test_t1_wifi_ws_sync`: Activating Wi-Fi respiration tracking via `SetOmniMode`.
   - `test_t1_wifi_telemetry`: Telemetry reports breathing rate and occupancy confidence correctly.
6. **Ghost Mic**:
   - `test_t1_ghost_mic_displacement`: Window displacement processed using Acoustic decimation filter.
   - `test_t1_ghost_mic_pcm_format`: Unwrapped phase streamed as 16-bit signed PCM audio values.
   - `test_t1_ghost_mic_ws_stream`: WebSocket binary/text endpoint streams PCM audio values continuously.
   - `test_t1_ghost_mic_controls`: Triggering mode via WebSocket `SetOmniMode` with Ghost Mic payload.
   - `test_t1_ghost_mic_scaling`: Verified PCM amplitude changes proportionally to displacement intensity.
7. **Stare Mode**:
   - `test_t1_stare_velocity_lock`: Lock target velocity to exactly 0.0 at specific geodetic coordinates.
   - `test_t1_stare_spectra_output`: Output geodetic vibration spectra for locked target.
   - `test_t1_stare_activation`: Activating stare mode via `SetStareMode` with coordinate arguments.
   - `test_t1_stare_resonance_map`: Verification of structural resonance/magnetostriction mapping.
   - `test_t1_stare_continuous_status`: Confirm stare status remains active 24/7.
8. **Drone Payload Heuristics**:
   - `test_t1_drone_thrust_estimation`: Estimate thrust-to-weight ratio from JEM RPM and EKF vertical velocity.
   - `test_t1_drone_payload_classification`: Classify heavy vs light payload based on vertical speed and RPM sidebands.
   - `test_t1_drone_ws_sync`: Activating drone payload heuristics via WebSocket returns heuristics payload.
   - `test_t1_drone_negative_climb`: Thrust-to-weight correctness when vertical velocity is negative (descending).
   - `test_t1_drone_dynamic_payload`: Verify classification updates dynamically as payload is added or dropped.

### Tier 2: Boundary & Corner Cases (5 cases per feature, 8 features = 40 cases)
Robustness checks for edge inputs, performance boundaries, and error recovery.
* **Continuous Phase Unwrapping**:
  - `test_t2_unwrap_extreme_jump`: Handling of phase jumps > $2\pi$.
  - `test_t2_unwrap_zero_signal`: Zero input signal returns zero displacement without NaN values.
  - `test_t2_unwrap_nan_recovery`: Robust recovery when signal contains NaN or Infinite values.
  - `test_t2_unwrap_rapid_oscillation`: Extreme phase oscillations near wrap boundaries.
  - `test_t2_unwrap_long_run_overflow`: Unwrapped phase accumulator does not overflow float bounds over long runs.
* **Cepstral Analysis**:
  - `test_t2_cepstrum_no_harmonics`: Input signal with pure sine wave (no harmonics) returns zero fundamental.
  - `test_t2_cepstrum_high_frequency`: Fundamental frequency above Nyquist limit handles gracefully.
  - `test_t2_cepstrum_noise_only`: High noise-only floor does not trigger false RPM lock.
  - `test_t2_cepstrum_negative_freq`: Graceful rejection of negative frequency bounds.
  - `test_t2_cepstrum_dense_peaks`: Resolving or failing gracefully under high density of target spectral peaks.
* **Programmable CIC Decimation Banks**:
  - `test_t2_cic_bypass`: Decimation factor R = 1 operates as bypass/passthrough.
  - `test_t2_cic_large_factor`: Extreme decimation factor R boundaries (buffer limits).
  - `test_t2_cic_rapid_toggling`: Rapidly toggling between Seismic, Rotary, and Acoustic modes.
  - `test_t2_cic_amplitude_overflow`: Wrapping i64 integrator overflow boundaries on high amplitude signals.
  - `test_t2_cic_zero_input`: Outputs stabilize at exactly 0.0 with zero input signal.
* **Deep-Integration Master EKF**:
  - `test_t2_ekf_zero_towers`: Correct coasting/prediction behavior with zero active towers.
  - `test_t2_ekf_extreme_outliers`: Rejecting extremely large measurement errors via innovation gating.
  - `test_t2_ekf_origin_singularity`: Target placed at exact coordinate origin (0,0,0) does not cause division-by-zero.
  - `test_t2_ekf_superluminal_speed`: Handling velocities approaching speed of light.
  - `test_t2_ekf_covariance_symmetry`: Covariance matrix forced symmetry and positive-definiteness under high noise.
* **Wi-Fi Respiration Tracker**:
  - `test_t2_wifi_packet_burst`: Rapid burst packet loading does not cause buffer lag.
  - `test_t2_wifi_zero_rate`: Disconnection / zero packet rate suspends extraction gracefully.
  - `test_t2_wifi_multipath_saturation`: EcaCanceler convergence bounds under 99% static clutter.
  - `test_t2_wifi_non_human_frequency`: Rejection of high vibration rates (e.g. 10 Hz) from respiration filter.
  - `test_t2_wifi_rapid_movement`: Transient occupancy detection state transitions.
* **Ghost Mic**:
  - `test_t2_ghost_mic_buffer_overflow`: Client backpressure test (dropping audio packets gracefully when buffer full).
  - `test_t2_ghost_mic_clipping`: Normalization and audio clipping protection on high-intensity displacement.
  - `test_t2_ghost_mic_silence`: Silence/zero vibration outputs clean zero PCM samples without noise floor bleed.
  - `test_t2_ghost_mic_reconnect`: Rapid disconnection and reconnection of audio WebSocket client.
  - `test_t2_ghost_mic_command_bounds`: Command boundaries and invalid parameters.
* **Stare Mode**:
  - `test_t2_stare_out_of_bounds`: Staring at geodetic coordinates extremely far outside coverage area.
  - `test_t2_stare_moving_target`: Lock velocity 0.0 on a highly active target and check tracking stability.
  - `test_t2_stare_tower_dropout`: Suddden tower dropouts do not crash the stare tracker.
  - `test_t2_stare_noise_only`: Locked coordinate with zero signal outputs flat noise spectra.
  - `test_t2_stare_overlapping_targets`: Staring at coordinate where multiple targets overlap.
* **Drone Payload Heuristics**:
  - `test_t2_drone_descending`: Negative vertical speed calculation robustness.
  - `test_t2_drone_supersonic_rpm`: Supersonic blade tip speeds.
  - `test_t2_drone_missing_sidebands`: Graceful degradation to EKF-only state when JEM sidebands are lost.
  - `test_t2_drone_incompatible_state`: Zero speed but maximum RPM (hovering state).
  - `test_t2_drone_high_sideband_noise`: Heavy sideband noise in RPM estimation.

### Tier 3: Cross-Feature Combinations (8 cases)
* `test_t3_stare_seismic_unwrap`: Locked coordinate + Seismic decimation + Continuous Phase Unwrapping (low-frequency structural vibration extraction).
* `test_t3_wifi_master_ekf`: Wi-Fi Respiration Tracker + Master EKF tracking coordinates simultaneously.
* `test_t3_ghost_mic_acoustic_unwrap`: Acoustic decimation + Phase Unwrapping + Ghost Mic streaming (eavesdropping window).
* `test_t3_drone_cepstrum_heuristics`: Drone thrust estimation using Cepstrum RPM.
* `test_t3_stare_rotary_cepstrum`: Locked coordinate + Rotary decimation + Cepstrum analysis (rotary magnetostriction).
* `test_t3_wifi_ghost_mic_coexistence`: Running respiration and acoustic mic streaming concurrently on same target.
* `test_t3_drone_ekf_stare_hover`: Stared coordinate + Master EKF + Drone heuristics.
* `test_t3_unwrap_cepstrum_acoustic`: Audio vibration analysis pipeline combining unwrapping, cepstrum, and acoustic decimation.

### Tier 4: Real-World Application Scenarios (5 scenarios)
* `test_t4_scenario_1_drone_hover_payload`: Drone hovering at stared coordinate, EKF tracks vertical velocity, Cepstrum resolves RPM, payload heuristic classifies heavy package.
* `test_t4_scenario_2_acoustic_window_eavesdropping`: Laser mic simulation against window under heavy ambient traffic noise.
* `test_t4_scenario_3_wifi_breathing_through_wall`: Wi-Fi respiration tracking behind concrete wall using Eca multipath canceler and Wavelet spike pruner.
* `test_t4_scenario_4_transformer_magnetostriction`: Monitoring substation power transformer 120 Hz vibration harmonics via stare mode.
* `test_t4_scenario_5_high_speed_ekf_unwrap`: tracking supersonic target with Master EKF while performing phase unwrapping.

---

## 3. Test Runner & Verification

To run the integration tests:
```bash
cargo test --test phase3_phase4_e2e -- --test-threads=1
```
