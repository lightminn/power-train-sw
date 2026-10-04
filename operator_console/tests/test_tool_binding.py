import json
import time
import socket
from types import SimpleNamespace

import gi
import pytest

gi.require_version('Gtk', '3.0')
from gi.repository import Gtk
from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.tool_binding import refresh_tool
from powertrain_observability.tool_snapshot import normalize_tool
from powertrain_ros.arm_console_mirror import build_arm_telemetry_payload


def tool(kind='spur_1motor_gripper', ids=None):
    ids = ids or [5]
    return dict(tool_type=kind, tool_profile={'actuator_ids': ids},
                actuators_discovered=True, actuators=[dict(id=i, online=True,
                position=-675, effort=12, torque_state='OFF') for i in ids])


def snapshot(value, age=0):
    return parse_arm_telemetry(build_arm_telemetry_payload(
        sequence=1, stamp_s=0, motors=None, joints=None, source_age_s={},
        tool_runtime={'tool': normalize_tool(value), 'source_age_s': age}))


def test_projection_filters_old_tool_and_preserves_signed_ticks():
    value = tool()
    value['actuators'].append(dict(id=3, position=99))
    result = snapshot(value).tool_runtime['tool']
    assert [x['id'] for x in result['actuators']] == [5]
    assert result['actuators'][0]['position'] == -675


@pytest.mark.parametrize('age', [-1, float('nan'), float('inf'), True])
def test_invalid_age_is_rejected(age):
    raw = dict(schema_version=1, sequence=1, tool_runtime={'tool': None, 'source_age_s': age})
    with pytest.raises(ValueError):
        parse_arm_telemetry(json.dumps(raw).encode())


def test_real_gtk_selection_stale_and_candidate():
    if not Gtk.init_check()[0]:
        pytest.skip('display unavailable')
    selector = Gtk.ComboBoxText()
    for name in ('미확인', '그리퍼 1', '청소 모듈'):
        selector.append_text(name)
    selector.set_active(0)
    w = SimpleNamespace(_mission_tool_selector=selector,
        _end_effector_popup_values={k: Gtk.Label() for k in
            ('selection', 'attachment', 'operation', 'load', 'telemetry')})
    for name in ('_mission_tool_reading', '_mission_tool_purpose',
                 '_end_effector_popup_purpose', '_mission_arm_mode', '_mission_arm_load'):
        setattr(w, name, Gtk.Label())
    w._set_end_effector_summary_state = lambda text, css: setattr(w, 'state', text)
    status_selector = Gtk.ComboBoxText()
    status_selector.append_text('미확인')
    status_selector.set_active(0)
    w._robot_status = SimpleNamespace(
        _end_effector_selector=status_selector,
        select_end_effector=lambda name: status_selector.set_active(
            [row[0] for row in status_selector.get_model()].index(name)),
    )
    assert refresh_tool(w, snapshot(tool()))
    assert selector.get_active_text() == '단일 그리퍼'
    assert w.state == '감지됨'
    assert '기계 체결 미확인' in w._end_effector_popup_values['attachment'].get_text()
    refresh_tool(w, snapshot(tool('dual_motor_gripper', [3, 4])))
    assert selector.get_active_text() == '듀얼 그리퍼'
    assert 'ID 5:' not in w._end_effector_popup_values['load'].get_text()
    selector.set_active(2)  # Explicit candidate must survive further observations.
    refresh_tool(w, snapshot(tool()))
    assert selector.get_active_text() == '청소 모듈'
    refresh_tool(w, snapshot(tool(), age=2))
    assert w.state == '수신 지연'
    assert w._end_effector_popup_values['load'].get_text() == '정보 없음'
    refresh_tool(w, None)
    assert w.state == '수신 대기'


def test_udp_to_existing_console(tmp_path):
    from operator_console.app import OperatorConsole, Gst, GLib
    if not Gtk.init_check()[0]:
        pytest.skip('display unavailable')
    Gst.init(None)
    window = OperatorConsole('127.0.0.1', 51002, 51000, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0,
        ops_token_file=str(tmp_path / 'absent-token'))
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    def pump_until(predicate):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            while GLib.MainContext.default().pending():
                GLib.MainContext.default().iteration(False)
            if predicate():
                return
            time.sleep(.01)
        assert predicate()
    try:
        window.show_all()
        port = window._arm_receiver._socket.getsockname()[1]
        payload = build_arm_telemetry_payload(sequence=1, stamp_s=0,
            motors=None, joints=None, source_age_s={},
            tool_runtime={'tool': normalize_tool(tool()), 'source_age_s': 0})
        sender.sendto(payload, ('127.0.0.1', port))
        pump_until(lambda: window._arm_receiver.latest() is not None)
        window._refresh_health()
        assert window._stack.get_child_by_name('arm_manual') is window._arm_manual_tab.widget
        assert window._stack.get_child_by_name('arm_calibration') is window._arm_calibration_tab.widget
        assert window._arm_manual_tab._state.detected_tool.kind == 'single_gripper'
        assert window._arm_manual_tab.gate.widgets
        assert not any(button.get_sensitive() for button in window._arm_manual_tab.gate.widgets)
        window._refresh_end_effector_summary()
        assert window._mission_tool_selector.get_active_text() == '단일 그리퍼'
        assert 'ID 5' in window._mission_tool_reading.get_text()
        window._show_end_effector_popup()
        assert '현재 감지 도구' in window._end_effector_popup_purpose.get_text()
        time.sleep(1.05)
        window._refresh_end_effector_summary()
        assert window._end_effector_popup_values['load'].get_text() == '정보 없음'
        # A resumed bridge datagram must replace stale tool details; it must not
        # retain the previous ID5 projection after a tool change/reconnection.
        resumed = build_arm_telemetry_payload(sequence=2, stamp_s=0,
            motors=None, joints=None, source_age_s={},
            tool_runtime={'tool': normalize_tool(tool('dual_motor_gripper', [3, 4])),
                          'source_age_s': 0})
        sender.sendto(resumed, ('127.0.0.1', port))
        pump_until(lambda: window._arm_receiver.latest().sequence == 2)
        window._refresh_health()
        window._refresh_end_effector_summary()
        assert window._mission_tool_selector.get_active_text() == '듀얼 그리퍼'
        assert window._arm_manual_tab._state.detected_tool.kind == 'dual_gripper'
        assert 'ID 3, 4' in window._mission_tool_reading.get_text()
        assert 'ID 5:' not in window._end_effector_popup_values['load'].get_text()
    finally:
        sender.close()
        window.destroy()
