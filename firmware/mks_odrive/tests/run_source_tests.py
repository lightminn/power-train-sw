#!/usr/bin/env python3
"""Execute real vendor/patched CAN functions against deterministic HAL/RTOS boundaries.
Host source execution only: not an STM32 timing, interrupt, or motor test.
"""
import argparse
from pathlib import Path
import re
import subprocess
import tempfile


def function(text, signature):
    start = text.index(signature)
    opening = text.index('{', start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def stripped(path):
    return re.sub(r'^\s*#(?:include|pragma once)[^\n]*', '', path.read_text(), flags=re.M)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('source', type=Path, help='Extracted ODrive-fw-v0.5.1 root or Firmware directory')
    args = ap.parse_args()
    root = args.source if (args.source / 'communication').is_dir() else args.source / 'Firmware'
    local = Path(__file__).parent
    interface = root / 'communication/interface_can.hpp'
    patched = 'reliability_patch_' in interface.read_text()
    axis_header = (root / 'MotorControl/axis.hpp').read_text()
    axis_methods = function(axis_header, 'void clear_errors()') + '\n' + function(axis_header, 'bool inline check_for_errors()')
    if 'void latch_can_fault()' in axis_header:
        axis_methods += '\n' + function(axis_header, 'void latch_can_fault()')
    prelude = (local / 'hal_boundary.hpp').read_text().replace('/* ACTUAL_AXIS_METHODS */', axis_methods)
    hal = function((root / 'Board/v3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c').read_text(), 'HAL_StatusTypeDef HAL_CAN_AddTxMessage(')
    source = '\n'.join([
        '#define PATCHED ' + str(int(patched)), prelude,
        stripped(root / 'communication/can_helpers.hpp'),
        stripped(interface).replace('private:', 'public:'),
        stripped(root / 'communication/can_simple.hpp'), hal,
        function((root / 'MotorControl/axis.cpp').read_text(), 'void Axis::watchdog_feed()'),
        function((root / 'MotorControl/axis.cpp').read_text(), 'bool Axis::watchdog_check()'),
        function((root / 'MotorControl/low_level.cpp').read_text(), 'void safety_critical_arm_motor_pwm('),
        function((root / 'MotorControl/low_level.cpp').read_text(), 'bool safety_critical_disarm_motor_pwm('),
        stripped(root / 'communication/interface_can.cpp'),
        stripped(root / 'communication/can_simple.cpp'),
        (local / 'scenarios.cpp').read_text(),
    ])
    cases = ['tx_race', 'mailbox_full', 'hal_add_failure', 'server_error', 'server_retry_failure',
             'reinit_exclusion', 'startup', 'recovery_latch', 'clear_before_idle', 'telemetry_watchdog',
             'unknown_watchdog', 'rtr_motion', 'short_motion', 'nonfinite_motion', 'valid_control',
             'rtr_state_clear', 'short_state', 'watchdog_expiry', 'idle_watchdog', 'idle_retains_expiry',
             'automatic_rearm', 'clear_during_recovery', 'irq_error_retained', 'baud_request_coalescing', 'persistent_recovery_failure', 'stop_error_recorded']
    with tempfile.TemporaryDirectory(prefix='mks-source-tests-') as temp:
        cpp, exe = Path(temp) / 'source.cpp', Path(temp) / 'source-tests'
        cpp.write_text(source)
        subprocess.run(['g++', '-std=c++17', '-O0', '-Wall', '-Wextra', '-Wno-unused-parameter', '-Wno-unused-but-set-variable', '-Wno-unused-function', str(cpp), '-o', str(exe)], check=True)
        failed = 0
        for case in cases:
            result = subprocess.run([str(exe), case], capture_output=True, text=True, timeout=3)
            print(('PASS' if result.returncode == 0 else 'FAIL') + ': ' + case + ' ' + result.stdout.strip() + result.stderr.strip())
            failed += result.returncode != 0
        print(f'{len(cases)-failed} passed, {failed} failed; PATCHED={patched}; source execution only')
        return bool(failed)

if __name__ == '__main__':
    raise SystemExit(main())
