# Premium HUD E2E Test Suite Readiness (`TEST_READY.md`)

This file attests that the Premium HUD End-to-End (E2E) integration test suite is fully designed, implemented, and ready to be run against the implementation.

## 1. Test Runner Command

To run the premium E2E integration test suite, execute the following command from the project root:

```bash
cargo test --test premium_e2e -- --test-threads=1
```

*Note: E2E tests dynamically bind to free TCP ports to avoid port collision. The `--test-threads=1` flag ensures sequential execution, which is required because multiple tests spawn instances of the `passiveradar` binary and interact with them in a timed and synchronized sequence.*

---

### 2. E2E Test Coverage Checklist

The test suite consists of **62 runnable test targets** (incorporating all **93 E2E test cases** via grouped compound assertions) checking 8 premium features across 4 tiers:

### Tier 1: Feature Coverage (Runnable Targets incorporating all core requirements)
- [x] **Gain Slider (SDR gain control)**
  - `test_gain_ws_compound` (Grouped WebSocket assertions: gain adjustment, bounds, sync)
  - `test_tier1_gain_increment_tui` (TUI key increment)
  - `test_tier1_gain_decrement_tui` (TUI key decrement)
  - `test_tier1_gain_max_limit` (Upper bound constraint verification)
- [x] **DC Block (SDR DC offset blocking filter)**
  - `test_dc_block_ws_compound` (Grouped WebSocket assertions: enable, disable, offset injection)
  - `test_tier1_dc_block_toggle_on_tui` (TUI filter activation)
  - `test_tier1_dc_block_toggle_off_tui` (TUI filter deactivation)
- [x] **Spectrogram (RF Signal Waterfall canvas)**
  - `test_spectrogram_ws_compound` (Grouped WebSocket assertions: output presence, power changes)
  - `test_tier1_spectrogram_toggle_off_tui` (TUI panel toggle off)
  - `test_tier1_spectrogram_visual_symbols` (Palette character symbols verification)
  - `test_tier1_spectrogram_resize_width` (Scaling on horizontal resize)
- [x] **IQ Constellation (IQ Constellation diagram)**
  - `test_constellation_ws_compound` (Grouped WebSocket assertions: coordinate streaming, point density)
  - `test_tier1_constellation_tui_rendering` (Constellation grid drawing)
  - `test_tier1_constellation_accuracy` (Centroid/accuracy display)
  - `test_tier1_constellation_toggle_off` (TUI panel toggle off)
- [x] **Micro-Doppler (JEM detail panel)**
  - `test_micro_doppler_ws_compound` (Grouped WebSocket assertions: streaming target selection, noise floor)
  - `test_tier1_micro_doppler_tui_inspect` (TUI target detail drawer display)
  - `test_tier1_micro_doppler_scale_toggle` (Doppler velocity scale change)
  - `test_tier1_micro_doppler_empty_unselected` (Empty/idle panel state)
- [x] **Antenna Aligner (Polar antenna alignment scope)**
  - `test_aligner_ws_compound` (Grouped WebSocket assertions: tower bearing telemetry, antenna calibration)
  - `test_tier1_aligner_tui_rendering` (Compass scope UI display)
  - `test_tier1_aligner_bearing_calc` (Heading-bearing computation correctness)
  - `test_tier1_aligner_peak_detection` (Peak tower highlight)
  - `test_tier1_aligner_toggle_off` (TUI panel toggle off)
- [x] **Hacker Console (Interactive retro monospaced terminal)**
  - `test_hacker_ws_compound` (Grouped WebSocket assertions: jam, spoof, sysinfo, scan, overflow)
  - `test_tier1_hacker_toggle_tui` (TUI console pane toggle)
- [x] **CRT/Audio Aesthetics (CRT scanlines, audio hum, Doppler beeps, speech announcements)**
  - `test_speech_ws_compound` (Grouped WebSocket assertions: voice text injection sanitization)
  - `test_tier1_crt_toggle_indicator` (TUI scanlines state indicator)
  - `test_tier1_crt_meteor_shake` (Screen shake event trigger)
  - `test_tier1_audio_hum_indicator` (TUI audio hum status)
  - `test_tier1_audio_doppler_beep` (TUI Doppler sound toggle status)
  - `test_tier1_audio_speech_lock` (Speech synthesis engine status)

### Tier 2: Boundary & Corner Cases (Runnable Targets incorporating robustness checks)
- [x] **Gain Boundaries**
  - Grouped into `test_gain_ws_compound` (high/negative bounds, decimal precision, garbage inputs)
- [x] **DC Block Boundaries**
  - Grouped into `test_dc_block_ws_compound` (saturated DC offset, recovery, alpha boundaries)
  - `test_tier2_dc_block_rapid_toggling` (Fast toggling state stability)
- [x] **Spectrogram Boundaries**
  - Grouped into `test_spectrogram_ws_compound` (silent spectrum, power settings bounds)
  - `test_tier2_spectrogram_height_collapse` (Graceful panel height collapse)
  - `test_tier2_spectrogram_buffer_wrap` (History circular buffer wrap-around)
  - `test_tier2_spectrogram_zero_towers` (Empty RF environment handling)
- [x] **IQ Constellation Boundaries**
  - Grouped into `test_constellation_ws_compound` (empty constellation, outlier coordinates, density loading)
  - `test_tier2_constellation_empty` (No-signal constellation grid)
  - `test_tier2_constellation_width_collapse` (Graceful panel width collapse)
- [x] **Micro-Doppler Boundaries**
  - Grouped into `test_micro_doppler_ws_compound` (supersonic velocity, invalid FFT sizes)
  - `test_tier2_micro_doppler_rapid_switching` (Rapid selection change stability)
  - `test_tier2_micro_doppler_termination` (Tracking removal on target termination)
  - `test_tier2_micro_doppler_unclassified_default` (Unclassified target JEM defaults)
- [x] **Antenna Aligner Boundaries**
  - Grouped into `test_aligner_ws_compound` (calibration angle settings)
  - `test_tier2_aligner_heading_wrap_positive` (Compass wrapping > 360°)
  - `test_tier2_aligner_heading_wrap_negative` (Compass wrapping < 0°)
  - `test_tier2_aligner_exact_origin` (Tower exactly at receiver location)
  - `test_tier2_aligner_zero_towers` (Compass grid without tracking beacons)
  - `test_tier2_aligner_many_towers_overlap` (Layout overlap limits with 50+ towers)
- [x] **Hacker Console Boundaries**
  - Grouped into `test_hacker_ws_compound` (buffer overflow, duplicate spoof IDs, superluminal velocities)
- [x] **CRT/Audio Boundaries**
  - `test_tier2_crt_low_resolution` (Layout scaling at extremely low resolutions)
  - `test_tier2_crt_rapid_shake` (Consecutive rapid screen shakes stability)
  - `test_tier2_audio_missing_hardware` (Graceful fallback without audio output)
  - `test_tier2_audio_100_targets_clipping` (Clipping protection for high target counts)

### Tier 3: Cross-Feature Combinations (Pairwise Targets)
- [x] Pairwise combinations of Gain, DC Block, Jammer, Target Selection, and CRT mode:
  - `test_tier3_combo_1`
  - `test_tier3_combo_2`
  - `test_tier3_combo_3`
  - `test_tier3_combo_4`
  - `test_tier3_combo_5`
  - `test_tier3_combo_6`
  - `test_tier3_combo_7`
  - `test_tier3_combo_8`

### Tier 4: Real-World Application Scenarios (Comprehensive Workflows)
- [x] Comprehensive workflow simulations:
  - `test_tier4_scenario_1_airliner_jamming` (Airliner tracking under heavy interference)
  - `test_tier4_scenario_2_drone_clutter` (Drone detection close to ground clutter)
  - `test_tier4_scenario_3_multi_tower_meteor` (Multi-tower triangulation during meteor event)
  - `test_tier4_scenario_4_remote_calibration` (WebSocket remote antenna calibration via `test_aligner_ws_compound`)
  - `test_tier4_scenario_5_aesthetic_spoof` (CRT audit and spoof threat mitigation via `test_hacker_ws_compound`)
