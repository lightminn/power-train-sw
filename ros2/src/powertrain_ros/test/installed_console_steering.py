"""Installed broker <-> real GTK selector acceptance with a synthetic chassis.

Run server in a disposable ROS domain77 container; publish only its TCP port
to host loopback. Run gui under Xvfb. Neither side accesses a motor or Jetson.
--omit-push-fields reinjects the original broker defect in the disposable install.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

TOKEN = 'fixture-only-console-steering-token'


def serve(args):
    assert os.environ.get('ROS_DOMAIN_ID') == '77'
    assert not Path('/sys/class/net/can0').exists(), 'physical CAN is forbidden'
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from std_srvs.srv import SetBool
    import powertrain_ros.ops_broker_node as broker

    module = Path(broker.__file__).resolve()
    assert module.is_relative_to(args.install.resolve()), module
    original = module.read_text()
    proof = {'installed_module': str(module),
             'original_sha256': hashlib.sha256(original.encode()).hexdigest(),
             'omitted_push_fields': args.omit_push_fields, 'calls': []}
    args.evidence.mkdir(parents=True, exist_ok=True)
    (args.evidence / 'ops_console.token').write_text(TOKEN + '\n')
    if args.omit_push_fields:
        changed = original
        for field in ('steering_mode', 'steering_available', 'drive_transport'):
            line = f'                    "{field}": state.{field},\n'
            assert changed.count(line) == 1, field
            changed = changed.replace(line, '')
        module.write_text(changed)
    rclpy.init()
    node = Node('synthetic_steering_chassis')
    mode = ['ackermann']
    safety = node.create_publisher(String, '/chassis/safety_state', 10)
    authority = node.create_publisher(String, '/command_authority/state', 10)
    gateway = node.create_publisher(String, '/teleop/gateway_state', 10)

    def publish():
        now = time.monotonic()
        authority.publish(String(data='IDLE|fixture'))
        gateway.publish(String(data=json.dumps(dict(
            state='DRIVE', input_fresh=True, neutral=True, stamp_s=now))))
        safety.publish(String(data=json.dumps(dict(
            mode='IDLE', estop_latched=False, active_estop_sources=[],
            steering_mode=mode[0], steering_available=True, drive_transport='can',
            hardware_stop_proof=dict(source='chassis_can', valid=True, stopped=True,
                node_ids=list(range(11, 17)), max_feedback_age_ms=5), stamp_s=now))))

    def switch(request, response):
        mode[0] = 'skid' if request.data else 'ackermann'
        proof['calls'].append(dict(data=request.data, applied_mode=mode[0]))
        response.success, response.message = True, mode[0]
        publish()
        return response

    node.create_service(SetBool, '/chassis_node/steer_mode_skid', switch)
    node.create_timer(.05, publish)
    log = (args.evidence / 'broker.log').open('w')
    child = None
    try:
        child = subprocess.Popen(['ros2', 'run', 'powertrain_ros', 'ops_broker',
            '--ros-args', '-p', 'host:=0.0.0.0', '-p', f'port:={args.port}',
            '-p', f'token_dir:={args.evidence}'], stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True)
        (args.evidence / 'ready').write_text('ready\n')
        deadline = time.monotonic() + 60
        while time.monotonic() < deadline and not (args.evidence / 'stop').exists():
            assert child.poll() is None, 'installed broker exited'
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGTERM)
            child.wait(timeout=5)
        log.close()
        module.write_text(original)
        node.destroy_node()
        rclpy.shutdown()
        proof['traceback'] = 'Traceback (most recent call last)' in (
            args.evidence / 'broker.log').read_text()
        (args.evidence / 'server-result.json').write_text(json.dumps(proof, indent=2))


def gui(args):
    os.environ['GDK_BACKEND'] = 'x11'
    os.environ.pop('WAYLAND_DISPLAY', None)
    from operator_console.app import OperatorConsole, Gtk, Gdk, GLib, Gst

    args.evidence.mkdir(parents=True, exist_ok=True)
    token = args.evidence / 'gui-fixture.token'
    token.write_text(TOKEN)
    Gst.init(None)
    window = OperatorConsole('127.0.0.1', 59302, 59300, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0,
        ops_port=args.port, ops_token_file=str(token))
    result = {'transitions': []}

    def wait(predicate, seconds=8):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            while GLib.MainContext.default().pending():
                GLib.MainContext.default().iteration(False)
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError('GTK steering state/confirmation did not converge')

    try:
        window.show_all()
        window._ops_settings_button.clicked()
        panel = window._ops_panel
        button = panel._action_buttons['steer_mode_skid']
        wait(lambda: button.get_sensitive() and button.get_label() == '조향 방식 [애커만]')
        for target, korean in (('skid', '스키드'), ('ackermann', '애커만')):
            button.clicked()
            assert panel._confirm_strip.get_visible()
            assert korean in panel._confirm_copy.get_text()
            panel._confirm_button.clicked()
            wait(lambda: button.get_label() == f'조향 방식 [{korean}]'
                 and not panel._pending_requests)
            state = panel._client.latest_state()
            assert state['steering_mode'] == target and state['chassis_mode'] == 'IDLE'
            assert not state['estop_latched']
            result['transitions'].append(dict(mode=target, revision=state['revision'],
                                               label=button.get_label(), ack=panel._latest_ack_text))
            settings = window._ops_settings_window
            pixbuf = Gdk.pixbuf_get_from_window(settings.get_window(), 0, 0,
                settings.get_allocated_width(), settings.get_allocated_height())
            pixbuf.savev(str(args.evidence / (target + '.png')), 'png', [], [])
        result['result'] = 'PASS'
    except BaseException as error:
        result.update(result='FAIL', error=repr(error))
        raise
    finally:
        window.destroy()
        (args.evidence / 'gui-result.json').write_text(json.dumps(result, indent=2))


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('role', choices=('server', 'gui'))
    parser.add_argument('--evidence', type=Path, required=True)
    parser.add_argument('--port', type=int, default=9001)
    parser.add_argument('--install', type=Path)
    parser.add_argument('--omit-push-fields', action='store_true')
    args = parser.parse_args()
    if args.role == 'server':
        serve(args)
    else:
        # Match the real console entrypoint's teardown of the installed SRT
        # library: close GTK/clients first, then skip its unsafe global destructor.
        status = 0
        try:
            gui(args)
        except BaseException:
            import traceback
            traceback.print_exc()
            status = 1
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(status)
