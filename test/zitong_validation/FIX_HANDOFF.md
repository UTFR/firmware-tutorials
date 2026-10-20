# Firmware fix handoff — 2026-10-19

## Outcome

The two wheel RPM failures are fixed. All **36 host tests pass** (exit 0), and `pio run -e teensy41` now succeeds, producing `.pio/build/teensy41/firmware.elf` and `firmware.hex` for the existing **disabled configuration**.

**D1 is mitigated for the disabled build, not resolved for enabled operation.** The external tutorial library implementations are absent from this checkout. Enabling `CONFIGURED` still requires that library, valid pins/cell count/CAN settings, calibration, and verified CAN encoding. No replacement torque algorithm or simulated hardware drivers were added to firmware. The configured host tests use test-only mocks; their success does not establish that a configured Teensy image links or operates on hardware.

No firmware was uploaded. Existing work in the source was preserved; only the changes below were applied. The historical validation report remains in `HANDOFF.md`, and its original machine-readable failures are preserved in `results_before_fix.json`. `results.json` contains the new results and current source hash.

## Exactly where the firmware changed

All firmware changes are in `intro_projects/zitong/intro_project.cpp`. Line numbers refer to the fixed file. `firmware_fix.patch` contains the exact diff against the source present at the start of the fix task, excluding pre-existing user edits.

| Finding | Changed location | Change and resulting behavior |
| --- | --- | --- |
| D2: zero timestamp confused with unseen edge | Lines 62–76, `edge_seen` and `wheelEdge()` | Added a separate per-wheel ISR-owned validity flag. The first edge establishes history regardless of timestamp; subsequent edges compute the unsigned elapsed interval. Timestamp zero now works both at startup and rollover. |
| D2: current timestamp zero rejected by RPM reader | Lines 105–117, `readWheelRPM()` | Removed `!edges[i]` from the invalid-measurement test. A nonzero valid period and a fresh last edge determine validity. No extra shared validity read is needed in the task; periods/timestamps still use the existing interrupt-protected snapshot. |
| D3: speed estimate spans stopped interval | Lines 69–75, `wheelEdge()` | A gap greater than `WHEEL_TIMEOUT_US` clears the period. The first edge after a timeout reports zero; the next fresh edge establishes RPM. A gap exactly equal to the timeout remains valid, matching the reader's existing strict `>` comparison. |
| D1: disabled motor task still links torque calculation | Line 149, `motorTask()` | Added `cfg::CONFIGURED` to the calculation guard. Even an inconsistent RTD state cannot invoke the external torque calculator while configuration is disabled. |
| D1: disabled LCD task still links LCD calls | Lines 265–270, `lcdTask()` | The disabled task waits on its existing 100 ms period and skips the external LCD API. Configured builds retain the complete LCD display path, including the FAULT label. **Behavior change:** the disabled configuration no longer prints to the LCD. |
| D1: unconditional external initialization | Lines 322–331, `setup()` | Moved `lcd_init()` and `bms_init()` inside the existing configuration guard alongside CAN/wheel initialization. Disabled firmware now builds without these external definitions under the existing Teensy release build. |
| D1: explain configuration behavior | Lines 26–30 | Updated the configuration comment to describe the disabled build's exclusion of tutorial-library calls. |

D3 was initially a candidate because the assignment did not specify restart filtering. This implementation deliberately adopts the validation policy: after an excessive gap, require a new pair of edges. That avoids reporting a period which includes time spent stopped. The 500 ms cutoff is unchanged, so low-speed limits inherent in that cutoff are unchanged.

## Test changes

`test/zitong_validation/validation.cpp` adds coverage for an edge landing exactly on rollover zero, a single edge, duplicate timestamps, the inclusive timeout boundary, recovery on a second edge after timeout, and restart across rollover. The original zero-timestamp and timeout regressions remain asserted. The default motor test now forces RTD to prove the new configuration guard independently of state-machine reachability.

The same file adds setup call counters and checks library initialization gating plus creation of all five tasks in both configuration modes. A new disabled-LCD case verifies no LCD API use and the 100 ms wait. The existing `lcd_fault` assertion is retained and runs in the configured fixture, where the LCD is enabled; it was not removed or weakened to conceal a failure.

`test/zitong_validation/run.py:31–43` registers the expanded suite: **19 cases on the actual disabled source and 17 on a temporary configured copy**. The configured copy still changes only configuration constants. Mock headers are unchanged. The runner itself still leaves firmware files unchanged during validation; `source_unchanged: true` describes the test run, not the development edits preceding it.

## Verification evidence

From the repository root:

```sh
python3 test/zitong_validation/run.py
# 36 passed, 0 failed; project files unchanged: True

pio run -e teensy41
# Linking .pio/build/teensy41/firmware.elf
# Building .pio/build/teensy41/firmware.hex
# [SUCCESS]

git diff --check
# exit 0
```

Both formerly failing host tests now pass. The suite runs with AddressSanitizer and UndefinedBehaviorSanitizer; no sanitizer errors were observed. The successful target build uses the repository's existing Teensy 4.1 / Arduino configuration and FreeRTOS library, without PlatformIO configuration edits or fake linked driver definitions.

## Remaining limits

Enabled-operation linkage remains dependent on the missing tutorial library. Hardware behavior, physical precharge, actual protocol encoding, calibration, interrupt timing, RTOS preemption, and deadlines remain unverified. The host tests execute tasks sequentially with controlled clocks; checking delay requests does not prove real-time scheduling. The supplied assignment template in `src/intro_project.cpp` contains API declarations and requirements, but no implementations of the missing functions.
