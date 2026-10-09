"""Execute the shipped adapter callbacks without importing unavailable host rclpy."""
import ast
import json
import math
from pathlib import Path
from types import SimpleNamespace

from chassis.authority import CommandAuthority, MANUAL_SOURCE, TELEOP
from powertrain_ros.remote_input_gateway import GatewayConfig, RemoteInputGateway
from powertrain_ros.remote_input import RemoteInputDecoder
from motor_control.laptop.remote_operation_client import ClientInput, encode_frame


SOURCE = Path(__file__).parents[1] / 'powertrain_ros'


def method(filename, class_name, name):
    tree = ast.parse((SOURCE / filename).read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == class_name)
    item = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    module = ast.Module(body=[item], type_ignores=[])
    namespace = {'json': json, 'math': math, 'String': SimpleNamespace,
                 'ManualDriveCommand': SimpleNamespace, 'Twist': SimpleNamespace,
                 'time': __import__('time'), 'threading': __import__('threading'), 'MAX_EVENTS_PER_TICK':256}
    exec(compile(ast.fix_missing_locations(module), str(SOURCE / filename), 'exec'), namespace)
    return namespace[name]


def test_gateway_cached_tick_keeps_tcp_receipt_in_drive_message():
    gateway = RemoteInputGateway(GatewayConfig(input_timeout_s=.3))
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    gateway.begin_connection(connection_session_id="server-1")
    def submit(sample, seq, stamp):
        result = decoder.feed(encode_frame(sample, session_id='e628ef42-8120-4d81-9555-d7e7244b5dc0',
            sequence=seq, client_monotonic_ns=1), receive_monotonic_s=stamp)
        gateway.submit(result[0].frame)
    submit(ClientInput(), 0, 10.)
    gateway.tick(10.)
    submit(ClientInput(deadman=True, right_trigger=.4), 1, 10.01)
    messages = []
    node = SimpleNamespace(_manual_command_format='steering', pub_drive=SimpleNamespace(publish=messages.append))
    publish = method('teleop_command_node.py', 'TeleopCommandNode', '_publish_drive')
    for stamp in (10.02, 10.15, 10.29):
        publish(node, gateway.tick(stamp))
    assert [m.source_received_s for m in messages] == [10.01] * 3


def test_gateway_state_cannot_promote_old_command_receipt_on_new_neutral():
    authority = CommandAuthority()
    authority.set_mode(TELEOP)
    authority.submit(MANUAL_SOURCE, 0, 0, 10., source_received_s=10., connection_session_id="server-1")
    authority.select(10.)
    authority.submit(MANUAL_SOURCE, .4, 0, 10.1, source_received_s=10.01, connection_session_id="server-1")
    input_states, commands = [], []
    node = SimpleNamespace(_authority=authority, _now_s=lambda:10.25,
        _command_received_s=lambda info:10.25,
        cm=SimpleNamespace(set_manual_input_state=lambda active, **kw: input_states.append((active, kw)),
                           set=lambda *a, **kw:commands.append((a,kw))),
        pub_authority_state=SimpleNamespace(publish=lambda msg:None))
    state_callback = method('chassis_node.py','ChassisNode','_on_gateway_state')
    state_callback(node, SimpleNamespace(data=json.dumps({'state':'DRIVE','input_fresh':True,
        'input_received_s':10.24,'stamp_s':10.25,'connection_session_id':'server-1'})), {})
    tick = method('chassis_node.py','ChassisNode','_tick_authority')
    tick(node,10.25)
    assert input_states[-1] == (True, {'received_s':10.01,'connection_session_id':'server-1'})
    assert commands[-1][1]['source_received_s'] == 10.01
    state_callback(node, SimpleNamespace(data=json.dumps({'state':'DISCONNECTED','input_fresh':False,'stamp_s':10.25})), {})
    assert input_states[-1][0] is False


def test_legacy_twist_and_repeated_selection_cannot_rejuvenate_recovery_input():
    authority = CommandAuthority()
    authority.set_mode(TELEOP)
    authority.submit(MANUAL_SOURCE, 0, 0, 10.)
    authority.select(10.)
    authority.submit(MANUAL_SOURCE, .4, 0, 10.1)
    input_states, commands = [], []
    node = SimpleNamespace(_authority=authority, _gateway_input_ok=True, _gateway_received_s=10.1,
        cm=SimpleNamespace(set_manual_input_state=lambda active,**kw:input_states.append((active,kw)),
                           set=lambda *a,**kw:commands.append(kw)),
        pub_authority_state=SimpleNamespace(publish=lambda msg:None))
    tick = method('chassis_node.py','ChassisNode','_tick_authority')
    tick(node,10.15)
    tick(node,10.25)
    assert all(active is False for active,_ in input_states)
    assert [command['received_s'] for command in commands] == [10.1,10.1]


def test_coalesced_disconnect_connect_neutral_retains_server_epoch_not_client_session():
    from collections import deque
    import threading
    gateway = RemoteInputGateway(GatewayConfig(input_timeout_s=.3))
    gateway.begin_connection(connection_session_id='server-old')
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    # Reusing the client-selected session ID does not preserve server ownership.
    frame = decoder.feed(encode_frame(ClientInput(), session_id='e628ef42-8120-4d81-9555-d7e7244b5dc0',
        sequence=0, client_monotonic_ns=1), receive_monotonic_s=10.2)[0].frame
    gateway.submit(frame)
    old_output = gateway.tick(10.2)
    harness = SimpleNamespace(_events_lock=threading.Lock(), _gateway=gateway,
        _lifecycle_events=deque([('disconnect','server-old'),('connect','server-new')]),
        _motion_frame=frame, _violation_events=deque())
    drain = method('teleop_command_node.py','TeleopCommandNode','_drain_events')
    drain(harness,now_s=10.2)
    new_output = gateway.tick(10.2)
    assert new_output.state == 'DRIVE'
    assert old_output.drive.connection_session_id == 'server-old'
    assert new_output.drive.connection_session_id == 'server-new'
    messages = []
    publish = method('teleop_command_node.py','TeleopCommandNode','_publish_drive')
    publisher = SimpleNamespace(_manual_command_format='steering',pub_drive=SimpleNamespace(publish=messages.append))
    publish(publisher,old_output)
    publish(publisher,new_output)
    assert [m.connection_session_id for m in messages] == ['server-old','server-new']
    # Even if KEEP_LAST discards the intermediate DISCONNECTED state, the
    # received new connection epoch immediately cancels the old pending resume.
    seen = []
    receiver = SimpleNamespace(_gateway_connection_session_id='server-old',
        _command_received_s=lambda info:10.2,
        cm=SimpleNamespace(set_manual_input_state=lambda active,**kw:seen.append(active)))
    callback = method('chassis_node.py','ChassisNode','_on_gateway_state')
    callback(receiver,SimpleNamespace(data=json.dumps({'state':'DRIVE','input_fresh':True,
        'connection_session_id':'server-new'})),{})
    assert seen == [False]
