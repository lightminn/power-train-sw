"""ops broker 순수 코어 — 인증·인가·멱등·단일 mutation 직렬화 (§3.1).

소켓·ROS 무관: 시계와 상태 공급자를 주입받고, 실행은 ExecutionOrder 로
노드에 위임한다. 모든 결정(수락·거부·완료)은 응답 bytes 로 표현된다.
"""
from dataclasses import dataclass, field
import collections
import uuid

from powertrain_ros import ops_contract as oc


DEFAULT_COMPONENT_MASK = {
    "drive": True,
    "steer": True,
    "us100": True,
    "robot_arm": True,
}


@dataclass(frozen=True)
class OpsState:
    revision: int
    authority_mode: str
    gateway_state: str
    gateway_input_fresh: bool
    gateway_neutral: bool
    estop_latched: bool
    active_estop_sources: tuple
    wheels_stopped: bool
    field_age_s: dict
    component_mask: dict = field(
        default_factory=lambda: dict(DEFAULT_COMPONENT_MASK)
    )


@dataclass(frozen=True)
class ExecutionOrder:
    pending_key: tuple
    action: str
    kind: str
    targets: tuple
    params: dict


@dataclass(frozen=True)
class Decision:
    response: bytes = None
    execute: ExecutionOrder = None


class OpsBrokerCore:
    def __init__(self, token_roles, *, clock, state_provider,
                 cache_size=64, rate_limit_per_s=10):
        self._token_roles = dict(token_roles)
        self._clock = clock
        self._state = state_provider
        self._cache_size = int(cache_size)
        self._rate_limit = int(rate_limit_per_s)
        self.boot_id = uuid.uuid4().hex
        self._sequences = {}                 # client_key -> last sequence
        self._cache = collections.OrderedDict()   # (client,rid) -> bytes|None
        self._pending_order = None           # ExecutionOrder | None
        self._rate_window = {}               # client_key -> (window_s, count)
        self._emergency = {}                 # (client,action) -> begin_s

    # -- 응답 헬퍼 ------------------------------------------------------
    def _revision(self):
        return int(self._state().revision)

    def _reject(self, request_id, detail):
        return oc.encode_response(
            request_id=request_id, status=oc.STATUS_FINAL_REJECTED,
            state_revision=self._revision(), detail=detail,
        )

    # -- 핸드셰이크 -----------------------------------------------------
    def handshake(self, client_key, line):
        try:
            import json

            payload = json.loads(line)
            token = payload.get("token")
            is_hello = bool(payload.get("hello"))
        except (TypeError, ValueError):
            return None, self._reject("hello", "invalid handshake")
        if not is_hello:
            return None, self._reject("hello", "handshake required first")
        role = self._token_roles.get(token)
        if role is None:
            return None, self._reject("hello", "unauthorized token")
        self._sequences.setdefault(client_key, -1)
        return role, oc.encode_response(
            request_id="hello", status=oc.STATUS_FINAL_SUCCESS,
            state_revision=self._revision(),
            detail="role=%s broker_boot_id=%s" % (role, self.boot_id),
        )

    # -- 본 처리 --------------------------------------------------------
    def _rate_ok(self, client_key):
        now_s = float(self._clock())
        window, count = self._rate_window.get(client_key, (now_s, 0))
        if now_s - window >= 1.0:
            window, count = now_s, 0
        count += 1
        self._rate_window[client_key] = (window, count)
        return count <= self._rate_limit

    def _cache_put(self, key, response):
        self._cache[key] = response
        while len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)

    def handle_line(self, client_key, role, line):
        try:
            request = oc.decode_request(line)
        except ValueError as exc:
            return Decision(response=self._reject("invalid", str(exc)))
        request_id = request["request_id"]
        key = (client_key, request_id)

        if not self._rate_ok(client_key):
            return Decision(
                response=self._reject(request_id, "rate limit exceeded")
            )
        if self._token_roles.get(request["token"]) != role:
            return Decision(
                response=self._reject(request_id, "token/role mismatch")
            )

        last = self._sequences.get(client_key, -1)
        if key in self._cache:                    # 멱등 재전송
            cached = self._cache[key]
            if cached is None:                    # 아직 PENDING
                return Decision(response=oc.encode_response(
                    request_id=request_id, status=oc.STATUS_PENDING,
                    state_revision=self._revision(), detail="in flight",
                ))
            return Decision(response=cached)
        if request["sequence"] <= last:
            return Decision(
                response=self._reject(request_id, "sequence regression")
            )
        self._sequences[client_key] = request["sequence"]

        spec = oc.ACTIONS[request["action"]]
        emergency = self._authorize(role, request, spec)
        if isinstance(emergency, bytes):
            return Decision(response=emergency)

        if request["action"] == "status_query":
            return Decision(response=self._status_query(request))

        if "expected_state_revision" in request and (
            request["expected_state_revision"] != self._revision()
        ):
            return Decision(response=self._reject(
                request_id, "state revision mismatch"
            ))

        gate = self._precondition_gate(role, request, spec,
                                       emergency=emergency)
        if gate is not None:
            return Decision(response=gate)

        if self._pending_order is not None:
            return Decision(
                response=self._reject(request_id, "busy: mutation in flight")
            )
        order = ExecutionOrder(
            pending_key=key, action=request["action"], kind=spec.kind,
            targets=tuple(spec.target), params=dict(request["params"]),
        )
        self._pending_order = order
        self._cache_put(key, None)
        return Decision(
            response=oc.encode_response(
                request_id=request_id, status=oc.STATUS_PENDING,
                state_revision=self._revision(), detail="accepted",
            ),
            execute=order,
        )

    def _authorize(self, role, request, spec):
        """일반 인가. 반환: False(일반)/True(비상 경로)/bytes(거부)."""
        if role in spec.roles:
            return False
        if role in spec.emergency_roles:
            return True
        return self._reject(request["request_id"], "role not authorized")

    _STOPPING_STATES = ("STOPPING_FOR_ARM", "STOPPING_FOR_DRIVE")

    def _precondition_gate(self, role, request, spec, *, emergency):
        request_id = request["request_id"]
        state = self._state()
        action = request["action"]

        if emergency:
            phase = request.get("phase")
            if phase is None:
                return self._reject(
                    request_id, "emergency action requires phase"
                )
            hold_s = oc.EMERGENCY_HOLD_S[action]
            key = ("emergency", role, action)
            if phase == "begin":
                self._emergency[key] = float(self._clock())
                return oc.encode_response(
                    request_id=request_id, status=oc.STATUS_FINAL_SUCCESS,
                    state_revision=self._revision(),
                    detail="begin recorded; hold %.1fs" % hold_s,
                )
            begin_s = self._emergency.get(key)
            if begin_s is None:
                return self._reject(request_id, "execute without begin")
            if float(self._clock()) - begin_s < hold_s:
                return self._reject(request_id, "hold not satisfied")
            del self._emergency[key]
            if action == "arm" and not (
                state.gateway_neutral and state.gateway_input_fresh
                and state.wheels_stopped
            ):
                return self._reject(
                    request_id,
                    "arm requires released neutral input and stopped wheels",
                )
            return None
        if request.get("phase") is not None:
            return self._reject(request_id, "phase is emergency-only")

        if action.startswith("authority_"):
            if state.field_age_s.get("authority", 9.9) > oc.OPS_STATE_STALE_S \
                    or state.field_age_s.get("gateway", 9.9) \
                    > oc.OPS_STATE_STALE_S:
                return self._reject(request_id, "stale state; retry")
            if state.authority_mode == "MOTION_HOLD" and action in (
                "authority_manual", "authority_auto",
            ):
                return self._reject(
                    request_id, "MOTION_HOLD: clear_transient_hold first"
                )
            if state.gateway_state in ("ARM",) + self._STOPPING_STATES:
                return self._reject(
                    request_id, "gateway busy: %s" % state.gateway_state
                )
            if state.estop_latched and action in (
                "authority_manual", "authority_auto",
            ):
                return self._reject(request_id, "E-stop latched")
            if action == "authority_manual" and not (
                state.gateway_state == "DRIVE" and state.gateway_input_fresh
                and state.authority_mode in ("IDLE", "AUTONOMY")
            ):
                return self._reject(
                    request_id, "manual preconditions not met"
                )
            if action == "authority_auto" and state.authority_mode not in (
                "IDLE", "TELEOP",
            ):
                return self._reject(request_id, "auto preconditions not met")
        return None

    def _status_query(self, request):
        target = request["params"].get("request_id")
        for (client, rid), cached in reversed(self._cache.items()):
            if rid == target:
                if cached is None:
                    return oc.encode_response(
                        request_id=request["request_id"],
                        status=oc.STATUS_PENDING,
                        state_revision=self._revision(), detail="in flight",
                    )
                import json

                body = json.loads(cached)
                return oc.encode_response(
                    request_id=request["request_id"], status=body["status"],
                    state_revision=self._revision(), detail=body["detail"],
                )
        return oc.encode_response(
            request_id=request["request_id"],
            status=oc.STATUS_OUTCOME_UNKNOWN,
            state_revision=self._revision(), detail="no record",
        )

    def complete(self, pending_key, success, detail):
        status = (
            oc.STATUS_FINAL_SUCCESS if success else oc.STATUS_FINAL_REJECTED
        )
        response = oc.encode_response(
            request_id=pending_key[1], status=status,
            state_revision=self._revision(), detail=detail,
        )
        self._cache_put(pending_key, response)
        if self._pending_order is not None \
                and self._pending_order.pending_key == pending_key:
            self._pending_order = None
        return response
