"""Bind tool observations to existing labels; never emit a robot command."""
import time
from powertrain_observability.tool_snapshot import TOOL_LABELS
from operator_console.arm_binding import refresh_arm_summary


def refresh_tool(window, snapshot):
    refresh_arm_summary(window, snapshot)
    runtime = getattr(snapshot, 'tool_runtime', None)
    if runtime is None:
        if not getattr(window, '_runtime_tool_seen', False):
            return False
        runtime = {'tool': None, 'source_age_s': None}
    window._runtime_tool_seen = True
    tool = runtime['tool']
    age = runtime['source_age_s']
    fresh = (tool is not None and age is not None and
             age + max(0, time.monotonic() - snapshot.received_monotonic_s) <= 1.0)
    selector = window._mission_tool_selector
    selected = selector.get_active_text() or '미확인'
    previous = getattr(window, '_runtime_auto_label', None)
    label = TOOL_LABELS[tool['tool_type']] if fresh else None
    # Follow observations until the operator explicitly chooses another candidate.
    if label and (selected == '미확인' or selected == previous):
        names = [row[0] for row in selector.get_model()]
        if label not in names:
            selector.append_text(label)
            names.append(label)
        window._syncing_end_effector_selectors = True
        try:
            selector.set_active(names.index(label))
            # The existing system-status selector shares the same observed
            # value, but its original inventory labels are deliberately not
            # treated as aliases for an unconfirmed gripper mechanism.
            status_selector = window._robot_status._end_effector_selector
            status_names = [row[0] for row in status_selector.get_model()]
            if label not in status_names:
                status_selector.append_text(label)
            window._robot_status.select_end_effector(label)
        finally:
            window._syncing_end_effector_selectors = False
        selected = label
        window._runtime_auto_label = label
    if selected == '환경 센서 모듈':
        return False
    state, css = '수신 대기', 'status-muted'
    attachment, reading, load = '정보 없음', '도구 상태 미수신', '정보 없음'
    if fresh:
        ids = tool['actuator_ids']
        rows = {row['id']: row for row in tool['actuators']}
        detached = tool['tool_detached'] is True or tool['physical_tool_detached'] is True
        online = tool['actuators_discovered'] is True and all(
            rows.get(i, {}).get('online') is True for i in ids)
        state = '분리됨' if detached else '감지됨' if online else '확인 필요'
        css = 'status-live' if online and not detached else 'status-warn'
        if tool['mock_mode']:
            state = '시험 데이터 · ' + state
        attachment = f'{label} · {state} · 기계 체결 미확인'
        reading = '모터 ID ' + ', '.join(map(str, ids))
        details = []
        for i in ids:
            row = rows.get(i, {})
            details.append(f"ID {i}: 위치 {row.get('position')} tick · 피드백 {row.get('effort')} raw · "
                           f"토크 {row.get('torque_state', 'UNKNOWN')} · 오류 {row.get('hardware_error')}")
        load = '\n'.join(details).replace('None', '미수신')
        if selected != label:
            reading += f' · 감지: {label} / 선택: {selected}'
    elif age is not None:
        state, reading = '수신 지연', '도구 원천 데이터 지연'
    window._set_end_effector_summary_state(state, css)
    window._mission_tool_reading.set_text(reading)
    window._mission_tool_purpose.set_text('현재 도구 관측 · 변경 요청은 별도 제어 연결 필요')
    window._end_effector_popup_purpose.set_text('현재 감지 도구의 상태')
    values = window._end_effector_popup_values
    values['selection'].set_text(f'{selected} · {state}')
    values['attachment'].set_text(attachment)
    values['operation'].set_text('연동 예정 · 조종 모드는 다음 단계에서 연결')
    values['load'].set_text(load)
    values['telemetry'].set_text(reading + ' · ROS /tool/status')
    return True
