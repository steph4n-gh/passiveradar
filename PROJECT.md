# Project: Passive Radar Premium Tactical HUD

## Architecture
The companion web HUD connects to the Rust DSP backend via a WebSocket connection (port 8085).
- **Frontend (`web/`)**: HTML5/CSS3/JS companion client. Visualizes target telemetry (2D PPI, 3D Elevation, JEM), waterfall, controls DSP threshold and ellipse mode.
- **Backend (`src/main.rs`)**: Runs TCP listener, handles client connections, broadcasts telemetry JSON data, parses incoming JSON client command payloads.
- **SDR Source (`src/sdr.rs`)**: Manages SDR parameters (gain, frequency, etc.) and feeds raw complex float samples.
- **DSP/Telemetry Pipeline**: Extracts complex slices, calculates FFT, builds and formats JSON payloads.

## Milestones
| # | Name | Scope | Dependencies | Status |
|---|------|-------|-------------|--------|
| 1 | M1: SDR Controls & Command Sync | Interactive controls on UI; WebSocket message parsing on Rust backend; update SDR parameters. | None | DONE |
| 2 | M2: RF Waterfall & Constellation | Stream complex slices & FFT magnitudes; render Scrolling Spectrogram & IQ Constellation canvases. | M1 | DONE |
| 3 | M3: Micro-Doppler & Antenna Alignment | Toggle JEM spectrum/Micro-Doppler views; Polar antenna alignment scope widget. | M2 | DONE |
| 4 | M4: Hacker Shell & Aesthetics | Retro hacker console terminal widget; CRT scanline toggles, meteor shake, audio hum/Doppler beeps/speech lock. | M3 | DONE |
| 5 | M5: E2E Integration & Hardening | Integration with E2E tests (Tiers 1-4) and Tier 5 White-Box Adversarial coverage hardening. | M4 | DONE |

## Interface Contracts
### Web Client ↔ Rust Backend (WebSockets)
- **Port**: 8085
- **Telemetry Payload (JSON)**:
  - `system_status`: Connection state string (`ACTIVE` or `OFFLINE`).
  - `clipping_rate`: float.
  - `cancellation_db`: float.
  - `ellipse_mode`: string.
  - `dsp_threshold`: float.
  - `active_towers`: list of `{name, pos_enu: [x,y,z]}`.
  - `targets`: list of target objects.
  - `transients`: list of transient events.
  - `waterfall_row`: list of FFT bin floats.
  - *New*: `sdr_gain`: float, `sdr_offset`: float, `sdr_dc_block`: bool, `overflow_alarm`: bool, `jamming_active`: bool.
  - *New*: `surveillance_fft`: list of floats, `constellation_points`: list of complex float pairs `[re, im]`.
- **Client Commands (JSON)**:
  - `{"command": "set_threshold", "value": f32}`
  - `{"command": "toggle_ellipse_mode"}`
  - *New*: `{"command": "set_sdr_settings", "gain": f32, "offset": f32, "dc_block": bool}`
  - *New*: `{"command": "hacker_cmd", "payload": "jam" | "scan" | "sysinfo" | "logs" | "spoof [id]"}`

## Code Layout
- `web/index.html` - HUD layout structure.
- `web/style.css` - HUD glassmorphism and CRT design styles.
- `web/app.js` - Telemetry handling, Canvas rendering, audio engines, hacker shell.
- `src/main.rs` - WebSockets listener, JSON streaming, telemetry composition, command routing.
- `src/sdr.rs` - SDR hardware and simulator interface.
