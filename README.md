# Passive Radar (Forward-Scatter) DSP Pipeline

A high-performance, real-time software-defined radio (SDR) passive radar system written in Rust. This system exploits existing ambient RF illuminators (like FM radio towers) to detect and track aircraft via forward-scatter and bistatic reflections.

## Features

- **Real-Time DSP Pipeline**: Ingests IQ samples from SDR hardware (or high-fidelity simulation) at rates up to 2.4+ MSPS.
- **Hardware-Accelerated Math**:
  - **SIMD Decimation**: Auto-vectorized FIR filtering using AVX2/FMA (x86_64) or Neon (AArch64/Raspberry Pi) for massive downsampling throughput.
  - **GPU-Offloaded FFTs**: Utilizes WebGPU (`wgsl-fft`) to push heavy Cross-Ambiguity Function (CAF) matrix calculations to the GPU.
  - **Multithreaded CAF**: Distributes delay-bin processing across all CPU cores using `rayon`.
- **Bistatic EKF Tracking**: A custom Extended Kalman Filter bank dynamically associates detections and tracks targets in 3D ENU space.
- **Terminal UI Dashboard**: A highly decoupled, frame-limited Ratatui dashboard featuring:
  - 2D Trajectory Map (30 FPS)
  - Color-coded Range-Doppler Waterfall (10 FPS)
  - Target Tracking Bank & JEM (Jet Engine Modulation) Analytics
  - System Diagnostics & CLI Event Logs

## Prerequisites

- **Rust**: Latest stable toolchain (`rustup`).
- **SDR Hardware**: Any SoapySDR-compatible hardware (e.g., RTL-SDR, HackRF) if running in live mode.
- **GPU Drivers**: Vulkan/Metal/DX12 drivers for GPU FFT offloading (optional but recommended).

## Building

```bash
# Build the highly optimized release binary
cargo build --release
```

To compile completely without GPU support (e.g., for minimal edge devices):
```bash
cargo build --release --no-default-features
```

## Usage

Start the system using `cargo run`:

```bash
# Run in simulation mode (default)
cargo run --bin passiveradar --release

# Run with actual SDR hardware on a specific FM tower frequency
cargo run --bin passiveradar --release -- --mode sdr --freq 97.1

# Run with SDR, custom sample rate, and specific gain settings
cargo run --bin passiveradar --release -- --mode sdr --rate 2.4 --lna 32 --vga 30

# Run in CPU-only mode (dynamically disables GPU FFT offloading)
cargo run --bin passiveradar --release -- --disable-gpu
```

### CLI Arguments

- `-m, --mode <MODE>`: Ingestion mode. `sim` (default) or `sdr`.
- `-f, --freq <FREQ>`: Target FM radio frequency in MHz (auto-tunes if omitted).
- `-r, --rate <RATE>`: SDR input sample rate in MSPS (default: 2.048).
- `--lna <GAIN>`: LNA gain in dB (default: 32.0).
- `--vga <GAIN>`: VGA gain in dB (default: 30.0).
- `--disable-gpu`: Forces the DSP engine to fallback to CPU `rustfft` computation.
- `--compat`: Enables compatibility mode for older terminals (ASCII lines).

## Terminal Controls

- **Up/Down Arrows**: Select different targets in the tracking bank to inspect their EKF states and Jet Engine Modulation spectra.
- **ESC**: Deselect target.
- **L**: Toggle System Logs panel.
- **T**: Toggle Active Towers panel.
- **W**: Toggle Waterfall panel.

## Architecture

The system is decoupled into isolated threads to prevent UI rendering from blocking the strict DSP deadlines:
1. **SDR Ingestion**: Continuously pulls raw IQ blocks from hardware.
2. **DSP Pipeline**:
   - Digital Down Conversion (DDC)
   - SIMD FIR Decimation
   - NLMS Clutter Cancellation (Direct path rejection)
   - GPU/Multithreaded Cross-Ambiguity Function (CAF)
3. **Tracking & EKF**: Extracts peaks from the CAF matrix and updates the tracking bank.
4. **TUI Render**: Throttles string allocations and draws cached data to the screen.
