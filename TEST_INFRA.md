# E2E Test Infra: Passive Radar Dashboard Overhaul

## Test Philosophy
- **Opaque-Box**: Exercises the compiled binary (`passiveradar`) as a subprocess without internal code dependencies.
- **Requirement-Driven**: Tests are designed directly from the TUI Overhaul user requirements (R1–R4).
- **Methodology**: Category-Partition + Boundary Value Analysis + Pairwise Combinatorial Testing + Real-World Workload Testing.

## Feature Inventory
| # | Feature | Source (requirement) | Tier 1 | Tier 2 | Tier 3 |
|---|---------|---------------------|:------:|:------:|:------:|
| 1 | Responsive Width Collapse (Waterfall/Transients) | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 2 | Responsive Height Collapse (Logs) | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 3 | Responsive Height Collapse (Map/Towers) | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 4 | Target Table Sizing & Wrapping | ORIGINAL_REQUEST §R1 | 5 | 5 | ✓ |
| 5 | Simulation Control: Pause/Resume | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 6 | Simulation Control: Speed & Stepping | ORIGINAL_REQUEST §R2 | 5 | 5 | ✓ |
| 7 | Keyboard Target Selection & Inspection | ORIGINAL_REQUEST §R3 | 5 | 5 | ✓ |
| 8 | Dynamic Panel Visibility Toggles | ORIGINAL_REQUEST §R4 | 5 | 5 | ✓ |

## Test Architecture
- **Test Runner Hook**: Adds `--test-script`, `--test-out`, `--width`, and `--height` CLI flags to `passiveradar`. When run with `--test-script`, it uses `ratatui::backend::TestBackend` instead of interactive `crossterm`.
- **Test Script Format**:
  - `KEY <name>`: Simulates a keystroke (e.g. `space`, `+`, `-`, `up`, `down`, `esc`, `l`, `t`, `w`, `s`, `q`).
  - `TICK <n>`: Advances simulation / main loop by `<n>` ticks.
  - `DUMP <file>`: Writes current terminal frame character buffer to `<test_out>/<file>`.
- **Directory Layout**:
  - `tests/e2e.rs`: Single Rust integration test module compiling to an independent test executable.
  - `test_out/`: Output directory where temporary outputs can be examined.

## Real-World Application Scenarios (Tier 4)
| # | Scenario | Features Exercised | Complexity |
|---|----------|--------------------|------------|
| 1 | Normal Operation | Map, Logs, Waterfall rendering | Low |
| 2 | Operator Inspection | Selection, pause, speed adjustment, inspect panel | Medium |
| 3 | Low-Spec Terminal | Responsive layout collapse, toggles | Medium |
| 4 | Supersonic Intercept | Target table status, map target position | Medium |
| 5 | Operator Recovery | Multikey sequence (pause, speed, toggle, select) | High |

## Coverage Thresholds
- Tier 1: 5 tests per feature = 40 tests
- Tier 2: 5 tests per feature = 40 tests
- Tier 3: 8 pairwise combination tests
- Tier 4: 5 realistic application scenarios
- **Total E2E test cases: 93**
