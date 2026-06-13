# Original User Request

## 2026-06-12T23:24:41Z

Overhaul the passive radar repository documentation to be comprehensive, accurate, and multi-tiered. The tone must be friendly and honest, reflecting the DIY "doing more with less" ethos of tracking aircraft using ambient FM radio, a HackRF, a vertical whip antenna, and a kitchen baking sheet ground plane.

Working directory: `/Volumes/Storage/passiveradar`
Integrity mode: development

## Requirements

### R1. High-Impact Root README.md
Create a clean, engaging root README.md that serves as the entry point for three distinct audiences:
- **Hobbyists / Beginners**: High-level explanation of the passive radar concept, highlighting the DIY hardware setup (whip antenna + baking sheet + HackRF). Provide a quick-start guide.
- **Developers / Intermediate**: Instructions on compiling, running the Rust CLI, configuring host/ports, and spinning up the web HUD companion.
- **RF / Math Experts**: A conceptual summary of the advanced DSP and topological math used, with links to the detailed guides in the `docs/` folder.
- **Visuals**: Maintain the root screenshot embed.

### R2. Technical & Mathematical Foundations Guide (`docs/math.md`)
Create a comprehensive technical document at docs/math.md detailing the DSP and mathematical algorithms implemented in the codebase:
- **Adelic Langevin Optimization**: Detail the use of p-adic coordinates, the Monna map, and real-space projections for global target optimization/triangulation in math/adelic.rs.
- **Čech Cohomology**: Explain the topological filtering of ghost targets/intersections using Čech complex intersections in math/cohomology.rs.
- **Fractional Delay Filtering**: Explain the Fourier Shift Theorem implementation for sub-sample delay interpolation in dsp/fractional_delay.rs.
- **Filtered Backprojection Tomography**: Document the tomographical reconstruction for ISAR/orbit imaging.
- **Zero-Allocation DSP**: Highlight the architectural patterns used to avoid heap allocations in the hot processing loops.

### R3. Setup, Calibration & Troubleshooting Guide (`docs/setup.md`)
Create a practical operational guide at docs/setup.md:
- **Physical Antenna Setup**: Detailed instructions on positioning the vertical whip antenna and setting up the kitchen baking sheet ground plane.
- **FM Tuning**: Guide the user on selecting reference FM transmitters, determining frequency offset, and optimizing gain settings.
- **Troubleshooting**: A reference table for common failure modes (e.g., websocket connection timeouts, buffer overflows, target ghosting).

## Verification Resources
- The Rust codebase source files and tests in src/ and tests/.

## Acceptance Criteria

### Documentation Structure & Integrity
- [ ] Root README.md contains the multi-tier introduction structure with links to the new guides in `docs/`.
- [ ] docs/math.md is created and contains detailed mathematical equations (LaTeX/MathJax formatting preferred) and direct links to the relevant Rust source files/functions.
- [ ] docs/setup.md is created and includes a step-by-step physical setup and a troubleshooting Q&A table.
- [ ] All markdown links between documents and to local codebase files are fully valid and resolve correctly.

### Technical Accuracy & Verification
- [ ] Every cargo command, CLI argument (e.g., `--port`, `--web-port`), and code signature mentioned in the documentation matches the actual codebase implementation.
- [ ] The math guide accurately describes the p-adic, cohomology, and fractional delay algorithms, verified by checking their inline code implementations.

## Follow-up — 2026-06-13T04:23:29Z

# Teamwork Project Prompt

Implement Phase 3 (Universal Vibrometer JEM Upgrade) and Phase 4 (Omni-Sensor Operational Modes) of the passiveradar system, allowing both high-fidelity simulated test verification and live hardware support.

Working directory: `/Volumes/Storage/passiveradar`
Integrity mode: benchmark

## Requirements

### R1. Universal Vibrometer & JEM Upgrade (Phase 3)
- **Continuous Phase Unwrapping**: Compute instantaneous unwrapped phase from the target's DC-shifted complex IQ signal (using `atan2` and tracking phase jumps larger than $\pi$) to output a physical displacement time-series.
- **Cepstral Analysis**: Implement real-time cepstrum analysis (IFFT of the log FFT magnitude) to collapse harmonic overtones and pinpoint exact rotary fundamental RPMs.
- **CIC Decimation Banks**: Implement programmable Cascaded Integrator-Comb (CIC) decimation banks supporting three target-filtering modes:
  - `Seismic`: 0.01 - 5 Hz (e.g., respiration/structural drift)
  - `Rotary`: 10 - 250 Hz (e.g., turbine/motor vibrations)
  - `Acoustic`: 300 - 4000 Hz (e.g., glass window audio vibrations)
- **Controls**: Support triggering these modes via WebSocket commands (e.g., `SetVibrometerMode`) and update the Web HUD and TUI to select and visualize the results.

### R2. Omni-Sensor Operational Modes (Phase 4)
- **Vector Tracking Master EKF**: Implement a deep-integration EKF that updates a target's 3D geodetic state by taking raw IQ phase/Doppler peak errors across all active towers simultaneously.
- **Wi-Fi Respiration Tracker**: Integrate a mode that uses `EcaCanceler` to obliterate static concrete reflections at 2.4 GHz, uses a `TropicalWaveletCanceller` to prune bursty packet spikes, and applies the `Seismic` vibrometer to extract the 0.3 Hz breathing signature of human occupants.
- **Acoustic Eavesdropping ("Ghost Mic")**: Integrate a mode where the target window displacement is processed using the `Acoustic` vibrometer and the unwrapped phase is streamed as PCM audio values via WebSockets.
- **Stare Mode**: Lock target velocity to 0.0 at a specific geodetic coordinate to map structural resonances or power transformer magnetostriction 24/7.
- **Drone Payload Heuristics**: Estimate drone thrust-to-weight ratio and identify heavy payloads by mapping JEM RPM sidebands against EKF vertical velocity ($vz$).
- **Controls**: Support triggering these modes via WebSocket commands (e.g., `SetOmniMode`, `SetStareMode`) and update the Web HUD and TUI to control and display results.

### R3. Ingestion & Simulation Compatibility
- **Dual-Mode Input**: Implement the new modes (Wi-Fi respiration, Ghost Mic, EKF stare) both as extensions in the high-fidelity simulator (`sdr.rs`) for automated tests, and as live input processors when acquiring raw data via SoapySDR hardware on corresponding RF frequencies.

## Verification Resources
- Existing Rust codebase and test suites in `/Volumes/Storage/passiveradar`.

## Acceptance Criteria

### Universal Vibrometer (Phase 3)
- [ ] Unit tests verify that the continuous phase unwrapping handles boundary wraps ($-\pi$ to $\pi$) correctly.
- [ ] Cepstrum analysis successfully detects fundamental frequencies of harmonically complex mock signals.
- [ ] Programmable CIC decimation banks decimate and filter signals correctly according to `Seismic`, `Rotary`, and `Acoustic` frequency bounds.

### Omni-Sensor Modes (Phase 4)
- [ ] Unit tests verify that the Master EKF converges faster or is more stable than independent single-tower EKFs under high noise.
- [ ] Simulation test shows the Wi-Fi respiration tracker extracts the 0.3 Hz breathing rate from a simulated packet stream with static multipath.
- [ ] "Ghost Mic" WebSocket endpoint streams raw PCM audio values from simulated window vibrations.
- [ ] Stare mode holds the target velocity at 0.0 and outputs geodetic vibration spectra.
- [ ] Drone payload classification function correctly calculates thrust-to-weight ratios under varying vertical speeds and RPM sidebands.
