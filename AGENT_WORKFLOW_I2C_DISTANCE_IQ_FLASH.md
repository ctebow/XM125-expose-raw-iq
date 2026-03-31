# Agent Workflow: I2C Distance + IQ on STM32L431 (FLASH Slimming)

## Context
- Target: `STM32L431CBYx` (128 KB FLASH, 64 KB RAM)
- Problem: Firmware build for an I2C Distance + IQ endpoint exceeded FLASH (initially ~112 bytes, later narrowed as refactors landed).
- Root cause of initial overflow: the IQ app duplicated large portions of the distance detector implementation in a second translation unit.

## Files & key building blocks used in the workflow
- Shared core API header (new): `Inc/i2c_distance_detector_core.h`
- Shared core implementation (new): `Src/applications/i2c/i2c_distance_detector_core.c`
- Custom trimmed core (new): `Src/applications/i2c/i2c_distance_detector_core_custom.c`
- Thin IQ endpoint app layer:
  - `Src/applications/i2c/i2c_iq_custom.c` (refactored to be a thin IQ endpoint on top of the core)
  - IQ stream variant exists in repo for reference:
    - `Src/applications/i2c/i2c_distance_iq_stream.c`
- I2C register protocol:
  - `Inc/distance_reg_protocol.h`
  - `Src/applications/i2c/distance_reg_protocol_access.c`

## Workflow steps (what to do, and why)
### 1) Stop duplication: split detector into “core + thin app”
Goal: ensure the distance detector logic is compiled once into FLASH.

Approach:
- Move detector lifecycle, state, and default command handling into:
  - `i2c_distance_detector_core.c`
- Make app selection work via weak symbols:
  - core provides default detector API behavior
  - IQ endpoint app provides strong overrides for IQ-specific command/data/status behaviors

### 2) Resolve “multiple definition” linker errors (weak exports)
Symptom:
- `multiple definition of i2c_distance_detector_*` when both core and an app compiled strong definitions.

Fix strategy:
- Mark core’s exported detector API functions as `__attribute__((weak))` so the app’s strong versions can override them.
- Ensure each CubeIDE configuration links exactly one “core” implementation object:
  - either `i2c_distance_detector_core.c` or `i2c_distance_detector_core_custom.c`, but not both.

### 3) Verify map evidence instead of guessing
Primary tool:
- CubeIDE link map file: `Debug/xm125.map`

What to inspect:
- `.rodata` contributors to understand string/format-string blowups.
- `_sidata` load address and `.data` section sizing to understand when the binary exceeds FLASH due to init-data placement.

Key realizations during this workflow:
- The custom IQ app’s `.rodata` was significantly larger than the stock detector app.
- After structural refactors and `.rodata` trimming, the failure mode shifted from `.rodata` overflow to `.data` load placement overflow.
- Remaining overflow can still be caused by library/code pulled in by IQ capture/processing paths.

### 4) Trim the remaining optional core features without touching the original core
New file introduced:
- `Src/applications/i2c/i2c_distance_detector_core_custom.c`

High-confidence cuts applied in the custom core:
- Removed auto “measure on wakeup” behavior.
- Removed UART log toggling and verbose logging paths (measurement print became a no-op).
- Removed config logging command support.
- Kept the critical detector lifecycle:
  - create config + sensor
  - apply config
  - calibrate / recalibrate
  - handle `MEASURE_DISTANCE`
  - maintain status and call `i2c_distance_detector_core_on_measurement_done()`

### 5) Handle “debug build warnings” safely
Warnings encountered:
- `is_detector_ready` defined but not used (`-Wunused-function`)
- ELF `LOAD segment with RWX permissions`

Guidance:
- If code paths were removed intentionally (e.g., wakeup auto-measure), unused helpers are expected.
- RWX load segment warnings are typical in bare-metal firmware with linker script memory permission attributes; ensure you’re not actually executing from `.data`.

## “Agent checklist” for future builds
1. Confirm your CubeIDE build links the intended core object and only one core object.
2. If link errors occur:
   - multiple definitions: apply weak-export strategy on the core side
   - FLASH overflow: inspect `Debug/xm125.map` for top `.rodata` and `.data` contributors
3. Cut optional features first (wakeup behavior, config logging, UART toggles).
4. Prefer structural sharing (core vs thin app) over string-only trimming.
5. When you get close (tens to hundreds of bytes):
   - compare `_sidata`, `.rodata`, and library pull-in (e.g., float/math paths) via the map.

## Summary of what was changed/added in this workflow
- Added: `Inc/i2c_distance_detector_core.h`
- Added: `Src/applications/i2c/i2c_distance_detector_core.c`
- Added: `Src/applications/i2c/i2c_distance_detector_core_custom.c` (trimmed optional features)
- Edited: `Src/applications/i2c/i2c_iq_custom.c` to become thin on top of the core
- Adjusted: core exported detector API functions to be `weak` to prevent multiple definition conflicts

