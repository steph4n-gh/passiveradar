# Original User Request

## Initial Request — 2026-06-11T20:10:17Z

# Teamwork Project Prompt — Draft

Implement decoupled refresh rate throttling for the terminal UI components to eliminate CPU bog-downs caused by excessive string formatting and ANSI code generation.

Working directory: `/Volumes/Storage/passiveradar`
Integrity mode: development

## Requirements

### R1. Decoupled Component Refresh Rates
Modify the `Dashboard` rendering logic so that heavy UI components do not aggressively rebuild their string representations on every single master frame draw. Implement a frame-limiting or timing mechanism to cache or throttle the calculations.

### R2. Target Frequencies
Throttle the components to the following approximate refresh rates:
*   **2D Trajectory Map**: ~30 FPS (needs to be buttery smooth)
*   **Waterfall**: ~10 FPS (saves massive string allocations)
*   **Tracking Bank & Logs**: ~2-5 FPS (human readability limit)

### R3. Additional UI Performance Fixes
If you spot any other low-hanging fruit for massive performance gains in the terminal rendering logic (such as avoiding string allocations or caching rendered objects), implement them as well.

## Acceptance Criteria

### Implementation & Verification
- [ ] The dashboard components update independently at their targeted frequencies while the master terminal loop runs unimpeded.
- [ ] `cargo test` and `cargo test --test e2e` must pass to guarantee that the UI layout, formatting, and headless data dumping systems still function perfectly.
