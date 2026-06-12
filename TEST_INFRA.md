# Passive Radar E2E Test Infrastructure (`TEST_INFRA.md`)

This document outlines the architecture, layout, and execution mechanisms for the End-to-End (E2E) testing framework designed for the Premium HUD features of `passiveradar`.

## 1. Test Architecture

The E2E testing framework is designed as an **opaque-box** testing suite. It verifies the functionality of the Premium HUD without relying on internal Rust structure details. The interfaces validated are:

1. **WebSocket Server**: Tests interact with the premium features via JSON-RPC/API messages sent over WebSocket connections. On startup, the application defaults to port `8085` but can accept a custom port via the `--port <port>` CLI option.
2. **Command Line Interface (CLI)**: Tests execute the application binary using varying flags, environment variables, width/height parameters, `--port` overrides, and test scripts.
3. **TUI Interactive Frame Dumps**: Tests provide keypress mock scripts using the `--test-script` flag and assert correctness using visual frame output dumps in the `--test-out` directory.

### Dynamic Port Allocation & Sequential Execution
To avoid local port conflicts, the test suite dynamically selects an available TCP port (using `std::net::TcpListener::bind("127.0.0.1:0")`) and passes it to the spawned process via `--port <port>`. WebSocket clients then connect to `ws://127.0.0.1:<port>`. A global mutex is also used in `tests/premium_e2e.rs` to synchronize the lifecycle of spawned processes, ensuring WebSocket tests do not experience timing or resource conflicts. Sequential testing (`--test-threads=1`) remains the standard verification runner command.

### Grouped WebSocket Compound Tests
To optimize test suite execution speed and avoid spawning the application binary for every single WebSocket assertion, individual WebSocket tests are grouped into compound tests (e.g., `test_gain_ws_compound`, `test_dc_block_ws_compound`, etc.). This reduces the total binary spawn and warmup cycles, resulting in a dramatic speedup of the E2E verification.

---

## 2. Test Suite Layout

All E2E test cases for premium features are implemented in the Rust integration test file:
* **Path**: `tests/premium_e2e.rs`

The test suite covers four distinct testing tiers:

### Tier 1: Feature Coverage (5 cases per feature, 8 features = 40 cases)
Verify the core operational requirements of each feature.
1. **Gain Slider**: WS gain set, TUI key increment, TUI key decrement, max limit, WS-TUI sync.
2. **DC Block**: WS enable, WS disable, TUI toggle on, TUI toggle off, WS-TUI sync.
3. **Spectrogram**: WS output data presence, TUI toggle off, power changes, visual rendering symbols, resize width.
4. **IQ Constellation**: WS coordinates streaming, TUI scope rendering, cluster accuracy, toggle off, frame refresh rate.
5. **Micro-Doppler**: WS streaming select target, TUI select target inspection panel, scale toggle, empty on unselected, noise floor.
6. **Antenna Aligner**: TUI scope rendering, bearing calculation/heading argument, target peak detection, toggle off, tower list telemetry.
7. **Hacker Console**: WS jam command, WS spoof target ID command, TUI console display toggle, WS sysinfo command, WS scan command.
8. **CRT/Audio**: TUI CRT toggle indicator, meteor screen shake event trigger, TUI audio hum indicator, TUI Doppler beep indicator, TUI Speech lock status.

### Tier 2: Boundary & Corner Cases (5 cases per feature, 8 features = 40 cases)
Robustness checks for edge inputs, performance boundaries, and error recovery.
* **Gain**: high out-of-bounds, negative out-of-bounds, decimal precision, NaN/garbage inputs, rapid updates.
* **DC Block**: high DC offset inputs, recovery latency, zero input signal, alpha boundaries, rapid toggling.
* **Spectrogram**: zero amplitude spectrum, infinite amplitude scaling, panel height collapse, long-run buffer wrap, zero towers fallback.
* **IQ Constellation**: empty constellation list, extreme outlier coordinates, high density point loading, origin (0,0) plotting, panel width collapse.
* **Micro-Doppler**: supersonic target velocity, rapid switching of selected targets, selected target termination updates, unclassified target defaults, invalid FFT sizes.
* **Antenna Aligner**: heading > 360 degrees wrapping, negative heading wrap, target tower at exact receiver coordinates (0,0,0), zero towers compass grid, many towers layout overlap limits.
* **Hacker Console**: command string buffer overflow, duplicate spoof target ID collision, spoof speed-of-light velocities, rapid API flooding, invalid command error responses.
* **CRT/Audio**: CRT toggle on extremely low resolution, rapid succession meteor shakes, missing audio hardware fallback, 100+ target limit clipping, special character speech sanitization.

### Tier 3: Cross-Feature Combinations (8 cases)
Pairwise test cases checking interactions between key features (as detailed in `analysis.md`):
* Combines Gain (Low/High), DC Block (ON/OFF), Hacker Jamming (Active/Inactive), Target Selection (None/Selected), CRT mode (ON/OFF) in representative combinations.

### Tier 4: Real-World Application Scenarios (5 scenarios)
Complete simulated operations modeling actual operator workflows:
1. **Scenario 1**: Commercial Airliner Tracking under High Jamming.
2. **Scenario 2**: Low-Altitude Drone Detection in Heavy Ground Clutter.
3. **Scenario 3**: Multi-Tower Triangulation during Meteor Scatter Event.
4. **Scenario 4**: Remote Antenna Calibration via WebSocket HUD.
5. **Scenario 5**: Aesthetic Audit and Spoof Threat Mitigation.

---

## 3. Test Runner & Verification

To run the integration tests:
```bash
cargo test --test premium_e2e -- --test-threads=1
```

*Note: The `--test-threads=1` option is crucial for sequentially running WebSocket tests that bind to a single local port.*
