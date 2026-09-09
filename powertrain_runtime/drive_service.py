"""Nonblocking manual-start orchestration over the existing gated ops client.

This module never owns a motor. ROS and ChassisManager remain the authorities
for state transitions; a successful ACK and a fresh observed result are both
required. Cancellation destroys the client so pending mutations cannot replay.
"""
from copy import deepcopy
import math
import threading
import time


class DriveRejected(Exception):
    pass


class DriveUnknown(DriveRejected):
    pass


class _Cancelled(Exception):
    pass


def _freshness_blocker(state, received_age, fields=("authority", "gateway", "safety", "wheels")):
    if not isinstance(state, dict) or not 0 <= received_age <= .5:
        return "운용 상태 수신 대기"
    ages = state.get("field_age_s", {})
    for name in fields:
        age = ages.get(name)
        if isinstance(age, bool) or not isinstance(age, (float, int)) or not math.isfinite(age) or not 0 <= age + received_age <= .5:
            return "최신 %s 상태 대기" % name
    return None


def _stop_postcondition(state):
    return (isinstance(state, dict) and state.get("chassis_mode") in ("IDLE", "ESTOP")
            and state.get("authority_mode") in ("IDLE", "MOTION_HOLD")
            and state.get("wheels_stopped") is True)


def start_blocker(state, received_age=0.0, *, allow_hold=True):
    """HOLD acknowledgement can precede fresh input; arming never can."""
    freshness = _freshness_blocker(state, received_age)
    if freshness:
        return freshness
    if state.get("estop_latched") is not False or state.get("chassis_mode") == "ESTOP" or state.get("active_estop_sources"):
        return "비상정지 원인을 확인하고 별도로 초기화하세요"
    recovering = allow_hold and state.get("gateway_state") == "MOTION_HOLD"
    if (state.get("gateway_input_fresh") is not True and not recovering) or state.get("gateway_neutral") is not True:
        return "패드 스틱·버튼을 모두 놓으세요"
    if state.get("wheels_stopped") is not True:
        return "바퀴 정지 확인 대기"
    if state.get("authority_mode") not in ("IDLE", "TELEOP", "MOTION_HOLD"):
        return "수동 운전으로 전환할 수 없는 권한 상태"
    if state.get("gateway_state") not in ("DRIVE", "MOTION_HOLD"):
        return "주행 게이트 준비 대기"
    if state.get("chassis_mode") not in ("IDLE", "ARMED"):
        return "차체 준비 대기"
    return None


class ManualDriveService:
    def __init__(self, client_factory, *, timeout_s=5.0, clock=time.monotonic):
        self._factory = client_factory
        self._timeout = timeout_s
        self._clock = clock
        self._lock = threading.RLock()
        self._closed = threading.Event()
        self._cancel = threading.Event()
        self._wake = threading.Event()
        self._pending = None
        self._client = None
        self._state = None
        self._received = None
        self._result = dict(status="IDLE", action="none", detail="연결 대기")
        self._arm_attempted = False
        self._thread = threading.Thread(target=self._run, name="manual-drive-service", daemon=True)
        self._thread.start()

    def snapshot(self):
        with self._lock:
            result = dict(self._result)
            age = float("inf") if self._received is None else self._clock() - self._received
            result["ops_state"] = deepcopy(self._state) if age <= .5 else None
            if result["ops_state"] is not None:
                result["ops_state"]["field_age_s"] = {
                    key: value + age if type(value) in (int, float) else value
                    for key, value in result["ops_state"].get("field_age_s", {}).items()
                }
            result["start_blocker"] = start_blocker(self._state, age)
            return result

    def start(self):
        with self._lock:
            if self._closed.is_set():
                return dict(status="REJECTED", action="start", detail="운용 서비스 종료됨")
            if self._result["status"] == "PENDING":
                return dict(status="REJECTED", action="start", detail="이전 조작 처리 중")
            self._cancel.clear()
            self._pending = "start"
            self._result = dict(status="PENDING", action="start", detail="운전 시작 확인 중")
            result = dict(self._result)
            self._wake.set()
            return result

    def stop(self):
        with self._lock:
            if self._closed.is_set():
                return dict(status="OUTCOME_UNKNOWN", action="stop", detail="운용 서비스 종료됨")
            if self._result["status"] == "PENDING" and self._result["action"] == "stop":
                return dict(self._result)
            self._cancel.set()
            self._pending = "stop"
            self._result = dict(status="PENDING", action="stop", detail="주행 해제 확인 중")
            result = dict(self._result)
            self._wake.set()
            return result

    def _drop_client(self):
        if self._client is not None:
            self._client.close()
        self._client = None
        with self._lock:
            self._state = None
            self._received = None

    def _pump(self):
        if self._client is None:
            self._client = self._factory()
        replies = self._client.pump()
        with self._lock:
            for reply in replies:
                if reply.get("push") == "ops_state":
                    self._state = deepcopy(reply)
                    self._received = self._clock()
        return replies

    def _check_cancel(self):
        if self._closed.is_set() or self._cancel.is_set():
            raise _Cancelled()

    def _state_now(self, *, allow_hold=False):
        with self._lock:
            age = float("inf") if self._received is None else self._clock() - self._received
            reason = start_blocker(self._state, age, allow_hold=allow_hold)
            if reason:
                raise DriveRejected(reason)
            return deepcopy(self._state)

    def _command(self, action, *, gate=False):
        self._check_cancel()
        self._pump()
        if gate:
            self._state_now(allow_hold=action == "clear_transient_hold")
        # Never queue an offline mutation that might replay after reconnect.
        if self._client.sock is None:
            raise DriveUnknown("조작 채널 연결 없음")
        request_id = self._client.submit(action)
        deadline = self._clock() + self._timeout
        while self._clock() < deadline:
            self._check_cancel()
            replies = self._pump()
            if self._client.sock is None:
                self._drop_client()
                raise DriveUnknown("조작 채널 연결 끊김")
            for reply in replies:
                if reply.get("request_id") != request_id:
                    continue
                status = reply.get("status")
                if status == "FINAL_SUCCESS":
                    return
                if status == "FINAL_REJECTED":
                    raise DriveRejected(reply.get("detail") or action)
                if status == "OUTCOME_UNKNOWN":
                    self._drop_client()
                    raise DriveUnknown(reply.get("detail") or action)
            self._closed.wait(.02)
        self._drop_client()
        raise DriveUnknown("%s 응답 확인 시간 초과" % action)

    def _wait_state(self, predicate, *, start_gate=True, recovering_hold=False):
        deadline = self._clock() + self._timeout
        while self._clock() < deadline:
            self._check_cancel()
            self._pump()
            with self._lock:
                age = float("inf") if self._received is None else self._clock() - self._received
                state = deepcopy(self._state)
            if start_gate:
                if _freshness_blocker(state, age):
                    self._closed.wait(.02)
                    continue
                # Clearing resets the gateway to DISCONNECTED until the next
                # neutral frame. Wait there without sending any arm/manual.
                if recovering_hold and state.get("gateway_state") in ("DISCONNECTED", "MOTION_HOLD", "DRIVE") and state.get("gateway_input_fresh") is not True:
                    if state.get("estop_latched") is not False or state.get("active_estop_sources") or state.get("wheels_stopped") is not True:
                        raise DriveRejected("HOLD 해제 중 안전 조건 변경")
                    self._closed.wait(.02)
                    continue
                reason = start_blocker(state, age, allow_hold=False)
                if reason:
                    raise DriveRejected(reason)
            relevant = ("authority", "safety", "wheels")
            ages = {} if state is None else state.get("field_age_s", {})
            fresh = all(type(ages.get(k)) in (int, float) and math.isfinite(ages[k]) and 0 <= ages[k] + age <= .5 for k in relevant)
            if fresh and age <= .5 and state is not None and predicate(state):
                return
            self._closed.wait(.02)
        raise DriveUnknown("명령 후 실제 상태 확인 시간 초과")

    def _start_drive(self):
        current = self._state_now(allow_hold=True)
        if current["chassis_mode"] == "ARMED":
            raise DriveRejected("이미 주행 활성 상태입니다")
        if "MOTION_HOLD" in (current["authority_mode"], current["gateway_state"]):
            self._command("clear_transient_hold", gate=True)
            self._wait_state(lambda s: s["gateway_state"] == "DRIVE" and s["authority_mode"] != "MOTION_HOLD", recovering_hold=True)
        if self._state_now()["authority_mode"] != "TELEOP":
            self._command("authority_manual", gate=True)
            self._wait_state(lambda s: s["authority_mode"] == "TELEOP")
        self._arm_attempted = True
        self._command("arm", gate=True)
        self._wait_state(lambda s: s["authority_mode"] == "TELEOP" and s["chassis_mode"] == "ARMED")

    def _stop_drive(self):
        failures = []
        for action in ("disarm", "authority_idle"):
            deadline = self._clock() + self._timeout
            while True:
                try:
                    self._command(action)
                    break
                except DriveUnknown as exc:
                    failures.append(str(exc))
                    break
                except DriveRejected as exc:
                    if action == "authority_idle":
                        with self._lock:
                            age = float("inf") if self._received is None else self._clock() - self._received
                            state = deepcopy(self._state)
                        # A latched HOLD rejects mode changes and already
                        # blocks input. Keep it latched; only a confirmed
                        # physical stop can satisfy this stopping action.
                        if (not _freshness_blocker(state, age, ("authority", "safety", "wheels"))
                                and state.get("authority_mode") == "MOTION_HOLD"
                                and _stop_postcondition(state)):
                            break
                    # Stop can race a previously accepted asynchronous mutation.
                    # Retry only these stopping actions, never start/arm.
                    if self._clock() >= deadline:
                        failures.append(str(exc))
                        break
                    self._closed.wait(.05)
        if failures:
            raise DriveUnknown("; ".join(failures))
        self._wait_state(_stop_postcondition, start_gate=False)

    def _run(self):
        try:
            while not self._closed.is_set():
                try:
                    self._pump()
                    with self._lock:
                        action, self._pending = self._pending, None
                        if action == "stop":
                            self._cancel.clear()
                    if action:
                        self._arm_attempted = False
                        try:
                            (self._start_drive if action == "start" else self._stop_drive)()
                            status, detail = "SUCCEEDED", ("운전 준비 완료" if action == "start" else "주행 해제 확인됨")
                        except _Cancelled:
                            self._drop_client()
                            continue
                        except DriveUnknown as exc:
                            status, detail = "OUTCOME_UNKNOWN", str(exc)
                        except DriveRejected as exc:
                            status, detail = "REJECTED", str(exc)
                        if action == "start" and status != "SUCCEEDED" and self._arm_attempted:
                            try:
                                self._stop_drive()
                                detail += " · 주행 해제 확인됨"
                            except _Cancelled:
                                self._drop_client()
                                continue
                            except DriveRejected:
                                status = "OUTCOME_UNKNOWN"
                                detail += " · 주행 해제 확인 불가"
                        with self._lock:
                            if self._pending is None:
                                self._result = dict(status=status, action=action, detail=detail)
                except Exception as exc:
                    self._drop_client()
                    with self._lock:
                        if self._result["status"] == "PENDING" and self._pending is None:
                            self._result.update(status="OUTCOME_UNKNOWN", detail="운용 채널 오류: " + type(exc).__name__)
                    self._closed.wait(.1)
                self._wake.wait(.02)
                self._wake.clear()
        finally:
            self._drop_client()

    def close(self):
        self._closed.set()
        self._cancel.set()
        self._wake.set()
        self._thread.join(timeout=2)
