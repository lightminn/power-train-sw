#!/usr/bin/env python3
"""Installed ROS + real GTK/TCP/UDP + a supervised virtual pad; no hardware."""
import argparse
import json
import os
from pathlib import Path
import runpy
import shlex
import signal
import socket
import subprocess
import sys
import tempfile
import time


def virtual_controller(path):
    import pygame
    from motor_control.laptop import dualsense_output
    dualsense_output._default_backend_factory = lambda: None
    state = lambda: json.loads(Path(path).read_text())

    class Joystick:
        def init(self): pass
        def quit(self): pass
        def get_init(self): return True
        def get_guid(self): return 'mock-xbox'
        def get_name(self): return 'Xbox Controller'
        def get_numbuttons(self): return 15
        def get_button(self, index): return int(index in state().get('buttons', []))
        def get_axis(self, index): return -1.0 if index in (2, 5) else 0.0
        def get_hat(self, index): return (0, 0)

    pygame.init = pygame.joystick.init = lambda: None
    pygame.event.pump = lambda: None
    pygame.event.get = lambda: []
    pygame.joystick.get_count = lambda: int(state().get('connected', True))
    pygame.joystick.Joystick = lambda _: Joystick()
    runpy.run_module('motor_control.laptop.controller_process', run_name='__main__')


def free_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(('127.0.0.1', 0))
        return sock.getsockname()[1]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', default=None)
    args = parser.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '77' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        parser.error('requires ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1; run in the isolated test container')
    base = Path(args.output_dir or tempfile.mkdtemp(prefix='arm-console-mock-')).resolve()
    base.mkdir(parents=True, exist_ok=True)
    (base / 'result.json').unlink(missing_ok=True)
    (base / 'failure.json').unlink(missing_ok=True)
    runtime_dir = base / 'runtime'
    runtime_dir.mkdir(exist_ok=True, mode=0o700)
    os.environ['XDG_RUNTIME_DIR'] = str(runtime_dir)
    os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = '1'
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    os.environ['SDL_AUDIODRIVER'] = 'dummy'
    (base / 'ops_console.token').write_text('arm-console-mock-token')
    (base / 'ops_console.token').chmod(0o600)
    pad_file = base / 'pad.json'

    def pad(*buttons, connected=True):
        temporary = pad_file.with_suffix('.tmp')
        temporary.write_text(json.dumps(dict(buttons=list(buttons), connected=connected)))
        temporary.replace(pad_file)

    pad()
    wrapper = base / 'controller-fixture'
    wrapper.write_text('#!/bin/sh\nexec ' + shlex.quote(sys.executable) + ' ' +
        shlex.quote(str(Path(__file__).resolve())) + ' --controller-fixture ' +
        shlex.quote(str(pad_file)) + ' "$@"\n')
    wrapper.chmod(0o700)
    processes, logs = [], []
    window = node = None
    checks = []

    def start(name, package, executable, parameters):
        log = (base / (name + '.log')).open('w')
        logs.append(log)
        command = ['ros2', 'run', package, executable, '--ros-args']
        for key, value in parameters.items():
            command += ['-p', f'{key}:={value}']
        processes.append(subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT,
                                          start_new_session=True))

    try:
        import gi
        gi.require_version('Gtk', '3.0')
        gi.require_version('Gst', '1.0')
        from gi.repository import Gtk, GLib, Gdk, Gst
        import rclpy
        from std_msgs.msg import String
        from operator_console.app import OperatorConsole
        from operator_console.operation_runtime import ControllerSupervisor
        assert Gtk.init_check()[0], 'needs a working Xvfb/X11 display'
        Gst.init(None)
        rclpy.init()
        node = rclpy.create_node('arm_console_mock_qualification')
        observed = {}

        def status(message):
            observed.clear()
            observed.update(json.loads(message.data))

        node.create_subscription(String, '/tool/status', status, 10)
        ops_port, arm_port = free_port(), free_port()
        start('bridge', 'dynamixel_control', 'moveit_dynamixel_bridge', dict(
            mock_mode='true', tool_type='dual_motor_gripper', control_scope='END_EFFECTOR_ONLY',
            toolchanger_enabled='false', auto_tool_detection='false'))
        start('broker', 'powertrain_ros', 'ops_broker', dict(
            host='127.0.0.1', port=ops_port, token_dir=str(base)))
        start('mirror', 'powertrain_ros', 'arm_console_bridge', dict(
            console_host='127.0.0.1', telemetry_port=arm_port, publish_hz=10.0))
        window = OperatorConsole('127.0.0.1', free_port(), free_port(), free_port(), 50,
            free_port(), free_port(), arm_telemetry_port=arm_port,
            environment_telemetry_port=free_port(), ops_host='127.0.0.1', ops_port=ops_port,
            ops_token_file=str(base / 'ops_console.token'), input_source='MOCK')
        window._arm_pad_reader = ControllerSupervisor(str(wrapper))
        window.show_all()
        # Use the actual tab key rather than relying on a translated title.
        from operator_console.arm_ui import ARM_MANUAL_TAB_NAME
        window._stack.set_visible_child_name(ARM_MANUAL_TAB_NAME)
        window.present()
        window.get_window().focus(Gdk.CURRENT_TIME)

        def wait(predicate, label, timeout=12):
            end = time.monotonic() + timeout
            context = GLib.MainContext.default()
            while time.monotonic() < end:
                for _ in range(30):
                    if not context.pending(): break
                    context.iteration(False)
                rclpy.spin_once(node, timeout_sec=.01)
                if any(p.poll() is not None for p in processes):
                    raise AssertionError('installed node exited; see logs in ' + str(base))
                if predicate():
                    checks.append(label)
                    return
            debug = dict(observed=observed, ui=repr(window._arm_manual_tab._state),
                ack=window._ops_panel._latest_ack_text,
                submitted=[dict(action=r.action, params=dict(r.params))
                           for r in window._arm_ops_adapter.session.submitted[-8:]])
            (base / 'failure.json').write_text(json.dumps(debug, ensure_ascii=False, indent=2))
            raise AssertionError(label + '; see ' + str(base / 'failure.json'))

        def settle(seconds=.8):
            end = time.monotonic() + seconds
            wait(lambda: time.monotonic() >= end, 'settled', timeout=seconds + 2)

        def positions():
            return {a['id']: a['position'] for a in observed.get('actuators', [])}

        def torque(value):
            rows = observed.get('actuators', [])
            return bool(rows) and all(a.get('torque_state') == value for a in rows)

        def buttons(widget, text):
            result = []
            if isinstance(widget, Gtk.Button) and widget.get_label() == text:
                result.append(widget)
            if isinstance(widget, Gtk.Container):
                for child in widget.get_children(): result.extend(buttons(child, text))
            return result

        tab = window._arm_manual_tab
        wait(lambda: window._ops_panel.link_ready() and tab._state.detected_tool.kind == 'dual_gripper',
             'installed bridge → UDP → GTK detects dual')
        tab._manual_button.clicked()
        wait(lambda: tab._state.authority.granted_mode == 'MANUAL', 'GTK manual grant through authenticated TCP')
        wait(lambda: tab._tool_enable_button.get_sensitive(), 'tool enable available')
        tab._tool_enable_button.clicked()
        wait(lambda: torque('ON') and 'dual_sync' in tab._state.capabilities, 'explicit activation and dual capability')
        before = positions()
        buttons(tab.tool_panels.active_panel.box, '열기')[0].clicked()
        wait(lambda: positions().get(3) == -675, 'left endpoint reaches only ID3')
        assert positions()[4] == before[4]
        buttons(tab.tool_panels.active_panel.box, '열기')[1].clicked()
        wait(lambda: positions().get(4) == 2559, 'right endpoint reaches ID4')
        sync_close = buttons(tab.tool_panels.active_panel.box, '닫기')[-1]
        wait(lambda: sync_close.get_sensitive(), 'synchronized GUI button enabled')
        sync_close.clicked()
        wait(lambda: positions() == {3: -2211, 4: 1017}, 'synchronized endpoint reaches both motors')
        window._arm_pad_toggle.set_active(True)
        wait(lambda: not window._arm_pad_control._needs_release, 'real supervised virtual Xbox is ready')
        before = positions()
        pad(2)
        wait(lambda: positions().get(3, -2211) > before[3], 'Xbox X produces authenticated leased dual motion')
        pad()
        wait(lambda: observed.get('fsm_state') == 'READY', 'button release holds torque')
        assert torque('ON')
        stopped = positions()
        settle()
        assert positions() == stopped
        pad(1)
        wait(lambda: observed.get('fsm_state') == 'CLOSING', 'Xbox B closes dual')
        pad(1, connected=False)
        wait(lambda: observed.get('fsm_state') == 'READY', 'pad disconnect expires/holds receiver lease')
        assert torque('ON')
        stopped = positions()
        pad(2)
        settle()
        assert positions() == stopped, 'held button resumed after reconnect'
        pad()
        settle(.4)
        window._arm_pad_toggle.set_active(False)
        wait(lambda: not window._arm_ops_adapter.session.registry.pending_stops,
             'GUI hold requests drained before independent lease test')
        settle(.3)
        identity = tab._state.detected_tool
        window._ops_panel.submit_external('robot_arm_tool_jog', dict(
            target='both', direction=1, sequence=window._arm_ops_adapter.session.registry._next_sequence(),
            expires_at_s=time.monotonic() + .6, ttl_s=.6,
            tool_id=identity.tool_id, tool_generation=identity.generation),
            (window._ops_panel.latest_state() or {})['revision'])
        wait(lambda: observed.get('fsm_state') == 'OPENING', 'one-shot jog without GUI refresh')
        wait(lambda: observed.get('fsm_state') == 'READY', 'receiver deadline stops without any GUI stop request')
        assert torque('ON')
        window._arm_pad_toggle.set_active(True)
        wait(lambda: not window._arm_pad_control._needs_release, 'pad re-enabled after neutral')
        pad(3)
        wait(lambda: torque('OFF'), 'Xbox Y disables tool')
        pad()
        settle(.3)
        assert window._arm_ops_adapter.session.request_tool_change('single_gripper')
        wait(lambda: tab._state.detected_tool.kind == 'single_gripper', 'runtime swap rebuilds single GUI')
        wait(lambda: tab._tool_enable_button.get_sensitive(), 'replacement requires explicit activation')
        tab._tool_enable_button.clicked()
        wait(lambda: torque('ON'), 'single tool activated')
        before = positions()[5]
        pad(1)
        wait(lambda: positions().get(5, before) > before, 'Xbox B controls only replacement ID5')
        pad(0)
        wait(lambda: observed.get('fsm_state') == 'READY', 'Xbox A soft stop')
        assert torque('ON')
        pad()
        settle(.3)
        assert window._arm_ops_adapter.session.request_tool_change('cleaner')
        wait(lambda: tab._state.detected_tool.kind == 'cleaner', 'runtime swap rebuilds cleaner GUI')
        wait(lambda: tab._tool_enable_button.get_sensitive(), 'cleaner explicit activation available')
        tab._tool_enable_button.clicked()
        wait(lambda: torque('ON'), 'cleaner activation uses current ID2 context')
        pad(2)
        wait(lambda: observed.get('fsm_state') == 'CLEANING', 'Xbox X routes to cleaner LEFT')
        pad()
        wait(lambda: observed.get('fsm_state') == 'READY', 'cleaner release soft stops')
        assert torque('ON')
        pad(3)
        wait(lambda: torque('OFF'), 'cleaner Y disables current tool only')
        pad()
        tab._release_button.clicked()
        wait(lambda: tab._state.authority.granted_mode != 'MANUAL', 'GUI releases manual authority')
        size = window.get_size()
        pixbuf = Gdk.pixbuf_get_from_window(window.get_window(), 0, 0, size.width, size.height)
        if pixbuf is not None: pixbuf.savev(str(base / 'console.png'), 'png', [], [])
        (base / 'result.json').write_text(json.dumps(dict(passed=True, checks=checks,
            checked_at_unix_s=time.time(), final_tool=observed, simulated_pad=True,
            physical_hardware=False), ensure_ascii=False, indent=2))
        print(f'PASS: {len(checks)} installed ROS/GTK/TCP/UDP/pad checks; evidence {base}', flush=True)
    finally:
        if window is not None: window.destroy()
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()
        for process in reversed(processes):
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGINT)
                try: process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(process.pid, signal.SIGKILL)
                    process.wait(timeout=3)
        for log in logs: log.close()


if __name__ == '__main__':
    if len(sys.argv) > 2 and sys.argv[1] == '--controller-fixture':
        assert sys.argv[3:] == ['-m', 'motor_control.laptop.controller_process']
        virtual_controller(sys.argv[2])
    else:
        main()
