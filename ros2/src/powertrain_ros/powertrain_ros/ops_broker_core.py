"""ops broker 순수 코어 — 인증·인가·멱등·단일 mutation 직렬화 (§3.1).

소켓·ROS 무관: 시계와 상태 공급자를 주입받고, 실행은 ExecutionOrder 로
노드에 위임한다. 모든 결정(수락·거부·완료)은 응답 bytes 로 표현된다.
"""
from dataclasses import dataclass, field
import collections
import hashlib
import json
import math
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
    chassis_mode: str = "UNKNOWN"
    estop_source: str = ""
    estop_detail: str = ""


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
        # transport key -> (stable authenticated identity, role, clock offset)
        self._sessions = {}
        self._sequences = {}                 # client identity -> last sequence
        # (client,rid) -> (request fingerprint, pending_key, bytes|None)
        self._cache = collections.OrderedDict()
        self._order_generation = 0
        # pending_key -> ((client,rid), request fingerprint)
        self._pending_requests = {}
        self._latest_requests = {}           # (client,rid) -> pending_key
        self._pending_order = None           # ExecutionOrder | None
        self._rate_window = {}               # client identity -> window/count
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
    @staticmethod
    def _token_identity(token):
        return hashlib.sha256(token.encode("utf-8")).hexdigest()

    @classmethod
    def _client_identity(cls, token, client_id):
        return cls._token_identity(token), client_id

    def handshake(self, client_key, line):
        try:
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
        client_id = payload.get("client_id")
        if not isinstance(client_id, str) or not client_id.strip():
            return None, self._reject(
                "hello", "client_id must be a non-empty string"
            )
        client_id = client_id.strip()
        offset_s = None
        if "stamp_s" in payload:
            try:
                client_stamp_s = float(payload["stamp_s"])
            except (TypeError, ValueError):
                return None, self._reject(
                    "hello", "invalid handshake stamp_s"
                )
            if not math.isfinite(client_stamp_s):
                return None, self._reject(
                    "hello", "invalid handshake stamp_s"
                )
            offset_s = float(self._clock()) - client_stamp_s
        identity = self._client_identity(token, client_id)
        self._sessions[client_key] = (identity, role, offset_s)
        self._sequences.setdefault(identity, -1)
        return role, oc.encode_response(
            request_id="hello", status=oc.STATUS_FINAL_SUCCESS,
            state_revision=self._revision(),
            detail="role=%s broker_boot_id=%s clock_sync=%s" % (
                role,
                self.boot_id,
                "ready" if offset_s is not None else "required",
            ),
        )

    def disconnect(self, client_key):
        self._sessions.pop(client_key, None)

    def _session(self, client_key, role, request):
        session = self._sessions.get(client_key)
        if session is None:
            return None, self._reject(
                request["request_id"], "handshake required"
            )
        identity, authenticated_role, offset_s = session
        if authenticated_role != role \
                or self._token_identity(request["token"]) != identity[0]:
            return None, self._reject(
                request["request_id"], "token/role mismatch"
            )
        return session, None

    def _request_time_rejection(self, request, offset_s):
        request_id = request["request_id"]
        if offset_s is None:
            return self._reject(request_id, "clock sync required")
        now_s = float(self._clock())
        server_stamp_s = float(request["stamp_s"]) + offset_s
        age_s = now_s - server_stamp_s
        if age_s > oc.REQUEST_DEADLINE_S:
            return self._reject(request_id, "request deadline exceeded")
        if age_s < -oc.REQUEST_FUTURE_SKEW_S:
            return self._reject(request_id, "request stamp too far in future")
        return None

    # -- 본 처리 --------------------------------------------------------
    def _rate_ok(self, client_key):
        now_s = float(self._clock())
        window, count = self._rate_window.get(client_key, (now_s, 0))
        if now_s - window >= 1.0:
            window, count = now_s, 0
        count += 1
        self._rate_window[client_key] = (window, count)
        return count <= self._rate_limit

    @staticmethod
    def _request_fingerprint(role, request):
        params = json.dumps(
            request["params"], separators=(",", ":"), sort_keys=True
        )
        return (
            role,
            request["action"],
            params,
            request.get("expected_state_revision"),
            request.get("phase"),
        )

    def _cache_put(self, key, fingerprint, pending_key, response):
        self._cache[key] = (fingerprint, pending_key, response)
        while len(self._cache) > self._cache_size:
            self._cache.popitem(last=False)

    def _accept_order(self, key, role, request, spec):
        self._order_generation += 1
        pending_key = key + (self._order_generation,)
        fingerprint = self._request_fingerprint(role, request)
        order = ExecutionOrder(
            pending_key=pending_key, action=request["action"], kind=spec.kind,
            targets=tuple(spec.target), params=dict(request["params"]),
        )
        self._pending_requests[pending_key] = (key, fingerprint)
        self._latest_requests[key] = pending_key
        self._pending_order = order
        self._cache_put(key, fingerprint, pending_key, None)
        return Decision(
            response=oc.encode_response(
                request_id=request["request_id"], status=oc.STATUS_PENDING,
                state_revision=self._revision(), detail="accepted",
            ),
            execute=order,
        )

    def handle_line(self, client_key, role, line):
        try:
            request = oc.decode_request(line)
        except ValueError as exc:
            return Decision(response=self._reject("invalid", str(exc)))
        request_id = request["request_id"]
        session, rejection = self._session(client_key, role, request)
        if rejection is not None:
            return Decision(response=rejection)
        identity, role, offset_s = session
        rejection = self._request_time_rejection(request, offset_s)
        if rejection is not None:
            return Decision(response=rejection)
        key = (identity, request_id)

        if request["action"] == "estop":
            if self._token_roles.get(request["token"]) != role:
                return Decision(
                    response=self._reject(request_id, "token/role mismatch")
                )
            spec = oc.ACTIONS[request["action"]]
            authorized = self._authorize(role, request, spec)
            if isinstance(authorized, bytes):
                return Decision(response=authorized)
            return self._accept_order(key, role, request, spec)

        if not self._rate_ok(identity):
            return Decision(
                response=self._reject(request_id, "rate limit exceeded")
            )
        if self._token_roles.get(request["token"]) != role:
            return Decision(
                response=self._reject(request_id, "token/role mismatch")
            )

        last = self._sequences.get(identity, -1)
        fingerprint = self._request_fingerprint(role, request)
        if key in self._cache:                    # 멱등 재전송
            cached_fingerprint, _, cached = self._cache[key]
            if cached_fingerprint != fingerprint:
                return Decision(response=self._reject(
                    request_id, "request_id reused with different request"
                ))
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
        self._sequences[identity] = request["sequence"]

        spec = oc.ACTIONS[request["action"]]
        emergency = self._authorize(role, request, spec)
        if isinstance(emergency, bytes):
            return Decision(response=emergency)

        if request["action"] == "status_query":
            return Decision(response=self._status_query(identity, request))

        if "expected_state_revision" in request and (
            request["expected_state_revision"] != self._revision()
        ):
            return Decision(response=self._reject(
                request_id, "state revision mismatch"
            ))

        gate = self._precondition_gate(identity, role, request, spec,
                                       emergency=emergency)
        if gate is not None:
            return Decision(response=gate)

        if self._pending_order is not None:
            return Decision(
                response=self._reject(request_id, "busy: mutation in flight")
            )
        return self._accept_order(key, role, request, spec)

    def _authorize(self, role, request, spec):
        """일반 인가. 반환: False(일반)/True(비상 경로)/bytes(거부)."""
        if role in spec.roles:
            return False
        if role in spec.emergency_roles:
            return True
        return self._reject(request["request_id"], "role not authorized")

    _STOPPING_STATES = ("STOPPING_FOR_ARM", "STOPPING_FOR_DRIVE")

    def _precondition_gate(
        self, identity, role, request, spec, *, emergency
    ):
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
            key = ("emergency", identity, action)
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

    def _status_query(self, identity, request):
        target = request["params"].get("request_id")
        cached_entry = self._cache.get((identity, target))
        if cached_entry is not None:
            cached = cached_entry[2]
            if cached is None:
                return oc.encode_response(
                    request_id=request["request_id"],
                    status=oc.STATUS_PENDING,
                    state_revision=self._revision(), detail="in flight",
                )
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

    def complete(self, pending_key, success, detail, *, status=None):
        if status is None:
            status = (
                oc.STATUS_FINAL_SUCCESS
                if success else oc.STATUS_FINAL_REJECTED
            )
        elif status not in (
            oc.STATUS_FINAL_SUCCESS,
            oc.STATUS_FINAL_REJECTED,
            oc.STATUS_OUTCOME_UNKNOWN,
        ):
            raise ValueError("invalid final status: %s" % status)
        pending = self._pending_requests.pop(pending_key, None)
        if pending is None:
            cache_key = pending_key[:2]
            entry = self._cache.get(cache_key)
            fingerprint = entry[0] if entry is not None else None
        else:
            cache_key, fingerprint = pending
        entry = self._cache.get(cache_key)
        latest_pending_key = self._latest_requests.get(cache_key)
        if latest_pending_key is not None \
                and latest_pending_key != pending_key:
            if entry is not None and entry[1] == latest_pending_key \
                    and entry[2] is not None:
                response = entry[2]
            else:
                latest_is_pending = latest_pending_key \
                    in self._pending_requests
                response = oc.encode_response(
                    request_id=cache_key[1],
                    status=(
                        oc.STATUS_PENDING if latest_is_pending
                        else oc.STATUS_OUTCOME_UNKNOWN
                    ),
                    state_revision=self._revision(),
                    detail="superseded by newer request",
                )
            if not any(
                item[0] == cache_key
                for item in self._pending_requests.values()
            ):
                self._latest_requests.pop(cache_key, None)
            return response
        response = oc.encode_response(
            request_id=cache_key[1], status=status,
            state_revision=self._revision(), detail=detail,
        )
        if entry is None or entry[1] == pending_key:
            self._cache_put(cache_key, fingerprint, pending_key, response)
        if not any(
            item[0] == cache_key for item in self._pending_requests.values()
        ):
            self._latest_requests.pop(cache_key, None)
        if self._pending_order is not None \
                and self._pending_order.pending_key == pending_key:
            self._pending_order = None
        return response
