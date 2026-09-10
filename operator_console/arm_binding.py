"""Read-only arm runtime labels for existing operator-console widgets."""
import time


def _fresh(snapshot, key):
    runtime = getattr(snapshot, 'arm_runtime', None)
    if not runtime:
        return None
    age = runtime['source_age_s'].get(key)
    if age is None or age + max(0, time.monotonic() - snapshot.received_monotonic_s) > 1.0:
        return None
    return runtime.get(key)


def control_mode_text(snapshot):
    return _fresh(snapshot, 'control_mode') or '수신 대기'


def fsm_text(snapshot):
    return _fresh(snapshot, 'fsm_state') or '수신 대기'


def contract_text(snapshot):
    return _fresh(snapshot, 'arm_status') or '수신 대기'


def refresh_arm_summary(window, snapshot):
    if snapshot is None:
        window._mission_arm_mode.set_text('수신 대기')
        window._mission_arm_load.set_text('정보 없음')
        return
    window._mission_arm_mode.set_text(control_mode_text(snapshot))
    joints_age = snapshot.joints_age_s
    fresh = (joints_age is not None and
             joints_age + max(0, time.monotonic() - snapshot.received_monotonic_s) <= 1.0)
    if not fresh or not snapshot.joint_names:
        window._mission_arm_load.set_text('정보 없음')
    elif snapshot.joint_effort_raw:
        window._mission_arm_load.set_text(
            f'최고 {max(abs(value) for value in snapshot.joint_effort_raw):.0f} raw · '
            f'관절 {len(snapshot.joint_names)}개')
    else:
        window._mission_arm_load.set_text(
            f'관절 {len(snapshot.joint_names)}개 · 피드백 raw 미수신')
