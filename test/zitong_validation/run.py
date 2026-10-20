#!/usr/bin/env python3
"""Host validation; all compiled artifacts and configured copies are temporary.

Exit 1 means a regression assertion failed. The source tree is hashed before
and after the run. This does not simulate FreeRTOS scheduling or real hardware.
"""
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys
import tempfile

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent.parent
PROJECT = ROOT / 'intro_projects' / 'zitong'


def snapshot():
    return {str(p.relative_to(PROJECT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in PROJECT.rglob('*') if p.is_file()}


def main():
    before = snapshot()
    original = PROJECT / 'intro_project.cpp'
    compiler = shutil.which('clang++') or shutil.which('g++')
    if not compiler:
        raise SystemExit('A C++ compiler (clang++ or g++) is required')
    common = ['float_roundtrip', 'adc_endpoints', 'wheel_no_edges',
              'wheel_frequency', 'wheel_timeout', 'wheel_clock_wrap',
              'wheel_edge_at_zero', 'wheel_wrap_to_zero', 'wheel_single_edge',
              'wheel_duplicate_edge', 'wheel_timeout_boundary', 'wheel_restart_after_timeout',
              'wheel_restart_across_wrap', 'fault_latch', 'setup_library_gating']
    cases = {
        'original': common + ['default_motor_inhibited', 'default_bms_fault',
                              'default_button_inhibited', 'default_lcd_inhibited'],
        'configured_fixture': ['motor_rtd', 'motor_lv', 'motor_fault', 'motor_nonfinite',
                               'bms_healthy', 'bms_boundaries', 'bms_bad_low',
                               'bms_bad_high', 'bms_bad_hot', 'bms_bad_nan', 'bms_bad_inf',
                               'precharge_sequence', 'bms_gate', 'steering_valid', 'steering_nan',
                               'lcd_fault', 'setup_library_gating'],
    }
    results = []
    with tempfile.TemporaryDirectory(prefix='zitong-validation-') as directory:
        tmp = Path(directory)
        text = original.read_text()
        replacements = {
            'constexpr bool CONFIGURED = false;': 'constexpr bool CONFIGURED = true;',
            'constexpr uint8_t CELL_COUNT = 0;': 'constexpr uint8_t CELL_COUNT = 3;',
            'constexpr uint32_t CAN_BAUD = 0;': 'constexpr uint32_t CAN_BAUD = 500000;',
            'constexpr uint8_t WHEEL_PINS[4] = {255, 255, 255, 255};':
                'constexpr uint8_t WHEEL_PINS[4] = {30, 31, 32, 33};',
            'constexpr uint8_t STEERING_ID = 0, MOTOR_IDS[4] = {0, 0, 0, 0};':
                'constexpr uint8_t STEERING_ID = 20, MOTOR_IDS[4] = {21, 22, 23, 24};',
        }
        for old, new in replacements.items():
            if text.count(old) != 1:
                raise SystemExit(f'Fixture anchor changed; review configuration: {old}')
            text = text.replace(old, new)
        fixture = tmp / 'configured.cpp'
        fixture.write_text(text)
        for mode, tests in cases.items():
            source = original if mode == 'original' else fixture
            binary = tmp / mode
            subprocess.run([compiler, '-std=c++17', '-Wall', '-Wextra',
                            '-fsanitize=address,undefined', '-fno-omit-frame-pointer',
                            '-I', str(HERE / 'mocks'),
                            f'-DVALIDATION_SOURCE="{source}"',
                            str(HERE / 'validation.cpp'), '-o', str(binary)], check=True)
            for name in tests:
                run = subprocess.run([str(binary), name], text=True, capture_output=True, timeout=10)
                output = run.stdout.strip()
                if run.stderr:
                    output += '\n' + run.stderr.strip()
                print(f'[{mode}] {output}', flush=True)
                results.append({'mode': mode, 'test': name, 'passed': run.returncode == 0, 'output': output})
    unchanged = before == snapshot()
    report = {'source_unchanged': unchanged, 'source_sha256': before,
              'passed': sum(r['passed'] for r in results),
              'failed': sum(not r['passed'] for r in results), 'results': results}
    (HERE / 'results.json').write_text(json.dumps(report, indent=2) + '\n')
    print(f"{report['passed']} passed, {report['failed']} failed; project files unchanged: {unchanged}")
    return int(not unchanged or report['failed'] != 0)


if __name__ == '__main__':
    sys.exit(main())
