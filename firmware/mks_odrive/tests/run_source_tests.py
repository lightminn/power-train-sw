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
    if 'void latch_can_fault(' in axis_header:
        axis_methods += '\n' + function(axis_header, 'void latch_can_fault(')
    prelude = (local / 'hal_boundary.hpp').read_text().replace('/* ACTUAL_AXIS_METHODS */', axis_methods)
    fields = '\n'.join(re.findall(r'^\s*(?:bool|uint8_t) can_recovery_\w+_ = [^;]+;', axis_header, re.M))
    prelude = prelude.replace('/* ACTUAL_CAN_AXIS_FIELDS */', fields or
                            'bool can_recovery_latched_=false, can_recovery_in_progress_=false;')
    hal = function((root / 'Board/v3/Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c').read_text(), 'HAL_StatusTypeDef HAL_CAN_AddTxMessage(')
    can_source = stripped(root / 'communication/interface_can.cpp')
    # Observe the actual encoder's packet at the TX boundary, including gated
    # attempts during reset. The production write body still executes unchanged.
    signature = 'uint32_t ODriveCAN::write(can_Message_t &txmsg) {'
    assert can_source.count(signature) == 1
    can_source = can_source.replace(signature, signature + '\n    observe_tx_attempt(txmsg);')
    source = '\n'.join([
        '#define PATCHED ' + str(int(patched)),
        '#define RECOVERY_METADATA ' + str(int('can_recovery_auto_resume_eligible_' in axis_header)), prelude,
        stripped(root / 'communication/can_helpers.hpp'),
        stripped(interface).replace('private:', 'public:'),
        stripped(root / 'communication/can_simple.hpp'), hal,
        function((root / 'MotorControl/axis.cpp').read_text(), 'void Axis::watchdog_feed()'),
        function((root / 'MotorControl/axis.cpp').read_text(), 'bool Axis::watchdog_check()'),
        function((root / 'MotorControl/low_level.cpp').read_text(), 'void safety_critical_arm_motor_pwm('),
        function((root / 'MotorControl/low_level.cpp').read_text(), 'bool safety_critical_disarm_motor_pwm('),
        can_source,
        stripped(root / 'communication/can_simple.cpp'),
        (local / 'scenarios.cpp').read_text(),
    ])
    cases = ['tx_race', 'mailbox_full', 'hal_add_failure', 'server_error', 'server_retry_failure',
             'reinit_exclusion', 'startup', 'recovery_latch', 'clear_before_idle', 'telemetry_watchdog',
             'unknown_watchdog', 'rtr_motion', 'short_motion', 'nonfinite_motion', 'valid_control',
             'rtr_state_clear', 'short_state', 'watchdog_expiry', 'idle_watchdog', 'idle_retains_expiry',
             'automatic_rearm', 'clear_during_recovery', 'irq_error_retained', 'baud_request_coalescing', 'persistent_recovery_failure', 'stop_error_recorded']
    cases += ['resume_heartbeat', 'resume_healthy', 'resume_retries', 'resume_manual',
              'resume_manual_then_hal', 'resume_hal_then_manual', 'resume_manual_during_hal', 'resume_manual_at_dispatch',
              'resume_manual_revokes', 'resume_reinit_revokes', 'resume_estop_before',
              'resume_estop_after', 'resume_clear', 'resume_early_clear', 'resume_direct_latch',
              'resume_pending_idle', 'resume_axis_error', 'resume_motor_error',
              'resume_encoder_error', 'resume_controller_error', 'resume_sensorless_error',
              'resume_generation_wrap', 'resume_reboot_defaults', 'resume_invalid_baud',
              'resume_patch_identity']
    cases += ['guard_resume_success', 'guard_resume_generation', 'guard_resume_new_axis_error', 'guard_resume_estop', 'guard_resume_manual_reinit', 'guard_resume_pending_hal', 'guard_resume_hal_error', 'guard_resume_reinitializing', 'guard_resume_progress', 'guard_resume_nonidle', 'guard_resume_pending_arm', 'guard_resume_motor_error', 'guard_resume_encoder_error', 'guard_resume_controller_error', 'guard_resume_sensorless_error', 'guard_resume_not_latched', 'guard_resume_ineligible', 'guard_resume_external_clear', 'guard_resume_wrong_magic', 'guard_resume_reserved', 'guard_resume_wrong_state', 'guard_resume_invalid_frame', 'guard_resume_pending_idle', 'guard_resume_atomic', 'guard_resume_replay_after_fault', 'guard_resume_late_fault', 'guard_manual_zero', 'guard_manual_empty', 'guard_clear_nonzero', 'guard_clear_old_cas']
    cases += ['guard_resume_reserved_eligible', 'guard_resume_new_generation', 'guard_resume_no_can', 'guard_manual_state4', 'guard_manual_state8']
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
