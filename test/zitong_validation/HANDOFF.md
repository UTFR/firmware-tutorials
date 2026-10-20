# Zitong validation handoff

**Historical report before fixes.** Current changes, exact locations, test results, and the remaining library dependency are documented in [FIX_HANDOFF.md](FIX_HANDOFF.md). The original JSON results are preserved in `results_before_fix.json`; `results.json` now records the fixed source.

Validated on 2026-10-19. Target: `intro_projects/zitong/intro_project.cpp`.
The requested `intro_porjects` path was resolved to the existing `intro_projects` directory.

## Result

- Host suite: **26 passed, 2 failed** across 28 cases; exit status 1.
- Teensy 4.1 build: **FAILED at link time**; exit status 1.
- No project source files were edited. `results.json` records the source SHA-256 and the before/after directory equality check. The source was already modified in Git before validation; that existing change was preserved.
- No firmware was uploaded and no fixes were applied.

## Findings

### D1 — High: firmware does not link with the current dependencies

Reproduce from the repository root:

```sh
pio run -e teensy41
```

The source compiles, but the linker reports undefined references to:

```text
lcd_printf(char const*, ...)
calculate_torque_cmd(float*, float, float*, float)
lcd_init(unsigned char, unsigned char, unsigned char, unsigned char)
bms_init(unsigned char, unsigned char, unsigned char, unsigned char)
collect2: error: ld returned 1 exit status
*** [.pio/build/teensy41/firmware.elf] Error 1
```

Declarations at `intro_project.cpp:8–18` say a tutorial mock library supplies these functions. Their implementations are not resolved by the current build. This blocks generating a new flashable firmware image, even with `CONFIGURED=false`. Link the intended tutorial library or correct its linkage/build inclusion; do not treat the host test stubs as firmware implementations. The linker also emits ARM/Thumb relocation errors associated with those unresolved calls.

The first sandboxed attempt was blocked by access to PlatformIO's external lock/cache directory. A permitted retry reached the real linker failure described above; this is a separate project build failure.

### D2 — Low: timestamp zero is confused with an uninitialized wheel sensor

Failing test: `original / wheel_edge_at_zero`.

Location: `wheelEdge` at lines 64–68 and `readWheelRPM` at lines 107–109.

Reproduction: invoke the front-left wheel ISR at microsecond timestamps 0 and 1000, then read RPM. With 17 pulses per revolution, two valid edges 1000 microseconds apart should report approximately **3529.41 RPM**. Actual output is **0 RPM**. `if (previous)` discards the valid zero timestamp; `!edges[i]` also treats a current edge at zero as missing. Zero is a valid `micros()` value at startup or rollover.

Suggested correction: track edge validity separately from the timestamp, including how many valid edges have been observed. Keep unsigned subtraction for rollover handling. The ordinary rollover case (edges straddling rollover without landing exactly on zero) passes.

### D3 — Medium candidate: a timed-out wheel measurement becomes valid again after one edge

Failing test: `original / wheel_restart_after_timeout`.

Location: `wheelEdge` at lines 64–68 and timeout handling at lines 107–109.

Reproduction: generate edges at 100 and 1100 microseconds, wait 500001 microseconds, generate one new edge, then read RPM. Actual output is approximately **7.06 RPM**, calculated using the edge before the stop. The regression assertion expects **0 RPM until a second fresh edge** establishes a new period after the timeout.

The timeout checks only the age of the latest edge. Updating that edge makes the measurement fresh again while its period spans the entire stopped interval. This may supply a misleading speed during restart.

**Requirement assumption:** the assignment's complete acceptance criteria were not present. This test assumes timeout invalidates the old edge pair. If the intended algorithm explicitly uses the full interval between consecutive edges across a stop, this is an expected behavior and the assertion should be revised. Confirm that policy before calling D3 a confirmed defect. Suggested behavior under the test's policy: reset the period when a new edge arrives after an excessive gap, and wait for another fresh edge.

## Coverage and interpretation

The original-source tests check float serialization, ADC conversion under the current calibration constants, wheel frequency/timeout/rollover, latched faults, LCD fault display, and fail-safe behavior with configuration disabled.

The configured fixture tests check four motor CAN outputs, RTD gating, zero torque in LV and on fault, nonfinite torque sanitization, healthy BMS summaries, inclusive voltage/temperature limits, undervoltage/overvoltage/overtemperature/nonfinite trips, fault persistence, healthy-BMS gating, the five-second precharge relay sequence, RTD entry, and finite/nonfinite steering handling. Delay arguments are checked for motor, steering, BMS, and LCD tasks. Passing these checks verifies the asserted behavior, not compliance with an unavailable full assignment specification.

The fixture is generated in a temporary directory by changing only configuration constants: enable configuration, choose three cells, set synthetic CAN IDs/baud and wheel pins. Those are test values, not validated hardware settings. Thirteen cases execute the original source; fifteen execute the configured copy. Mock functions provide deterministic sensor values, clocks, CAN, GPIO, LCD, and mutex ownership checks. AddressSanitizer and UndefinedBehaviorSanitizer run with each host case; no sanitizer errors were observed.

The production defaults remain intentionally disabled (`CONFIGURED=false`, cell count and baud zero, unset wheel pins/IDs). The CAN encoding is explicitly a placeholder. These are readiness limitations, not additional unexpected test failures.

Not validated: real hardware, electrical precharge completion, actual CAN encoding/transport, sensor calibration, RTOS preemption or races, deadline compliance, stack sizing, setup allocation/task-creation failures, and behavior of blocking peripheral calls. Host task execution is sequential and stopped at mocked delay boundaries. Tests observe requested periods, not actual execution timing.

## Re-run and artifacts

From the repository root:

```sh
python3 test/zitong_validation/run.py
pio run -e teensy41
```

The host runner needs Python 3 and Clang++ or G++ with AddressSanitizer/UndefinedBehaviorSanitizer support. It uses no downloaded Python packages. Each case executes in a fresh process. An exit status of 1 is expected while these regression assertions fail. The runner regenerates `results.json`; compiler outputs and the configured fixture are temporary.

- `run.py`: compilation, configuration fixture, source hashing, case runner, JSON report.
- `validation.cpp`: executable assertions and deterministic API stubs.
- `mocks/`: host-only Arduino and FreeRTOS declarations.
- `results.json`: individual pass/fail evidence and source integrity hash.
- `HANDOFF.md`: this report, including the separate firmware build finding.
