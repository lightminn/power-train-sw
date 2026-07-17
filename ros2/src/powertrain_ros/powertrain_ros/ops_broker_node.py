"""ops broker 노드 — TCP :9001 소유, 복구·운용 명령의 단일 게이트 (§3.1).

패턴은 teleop_command_node의 TCP 스레드를 따르되, 실행은 rclpy call_async
+ future 타임아웃으로 한다. ops-state push 타이머는 서비스 콜과 분리된
callback group이라 서비스 지연이 push를 막지 않는다.
"""
from dataclasses import dataclass, field
import json
import math
import os
import socket
import threading
import time
import uuid

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

from powertrain_msgs.msg import WheelStates
from powertrain_observability.client import EventClient
from powertrain_ros import ops_contract as oc
from powertrain_ros.ops_broker_core import OpsBrokerCore, OpsState


CLIENT_IDLE_TIMEOUT_S = 10.0
WHEEL_STOP_TURNS = 0.1
_SEMANTIC_FIELDS = (
    "authority_mode",
    "gateway_state",
    "gateway_input_fresh",
    "gateway_neutral",
    "estop_latched",
    "active_estop_sources",
    "wheels_stopped",
)


def load_token_roles(token_dir):
    """Load one token line per role; missing files fail closed."""
    roles = {}
    for filename, role in (
        ("ops_console.token", oc.ROLE_CONSOLE),
        ("ops_controller.token", oc.ROLE_CONTROLLER),
    ):
        path = os.path.join(token_dir, filename)
        try:
            with open(path, "r", encoding="utf-8") as handle:
                token = handle.readline().strip()
        except OSError:
            continue
        if token:
            roles[token] = role
    return roles


@dataclass(eq=False)
class _Connection:
    sock: socket.socket
    client_key: str
    send_lock: threading.Lock = field(default_factory=threading.Lock)


@dataclass
class _PendingService:
    order: object
    connection: _Connection
    index: int = 0
    results: list = field(default_factory=list)
    future: object = None
    started_s: float = 0.0
    timeout_journaled: bool = False


class OpsBrokerNode(Node):
    def __init__(self, parameter_overrides=None, port_override=None,
                 token_dir_override=None):
        super().__init__(
            "ops_broker", parameter_overrides=parameter_overrides or []
        )
        self.declare_parameter("port", oc.DEFAULT_PORT)
        self.declare_parameter("token_dir", "/etc/powertrain")
        self._port = int(
            port_override
            if port_override is not None
            else self.get_parameter("port").value
        )
        token_dir = str(
            token_dir_override
            if token_dir_override is not None
            else self.get_parameter("token_dir").value
        )
        roles = load_token_roles(token_dir)
        if not roles:
            self.get_logger().error(
                "no ops tokens under %s — all clients will be rejected"
                % token_dir
            )

        self._state_lock = threading.Lock()
        self._fields = {name: None for name in _SEMANTIC_FIELDS}
        self._stamps = {
            "authority": None,
            "gateway": None,
            "safety": None,
            "wheels": None,
        }
        self._revision = 0
        self._last_semantic = None
        self._core_lock = threading.Lock()
        self._core = OpsBrokerCore(
            roles,
            clock=time.monotonic,
            state_provider=self._ops_state,
        )

        self._connections = []
        self._connections_lock = threading.Lock()
        self._client_threads = []
        self._client_threads_lock = threading.Lock()
        self._service_group = MutuallyExclusiveCallbackGroup()
        self._push_group = MutuallyExclusiveCallbackGroup()
        self._service_clients = {}
        self._service_clients_lock = threading.Lock()
        self._pending = {}
        self._pending_lock = threading.Lock()
        self._event_client = EventClient()

        self._section_pub = self.create_publisher(
            String, "/section_events", 10
        )
        self.create_subscription(
            String, "/command_authority/state", self._on_authority, 10
        )
        self.create_subscription(
            String, "/chassis/safety_state", self._on_safety, 10
        )
        self.create_subscription(
            String, "/teleop/gateway_state", self._on_gateway, 10
        )
        self.create_subscription(
            WheelStates, "/wheel_states", self._on_wheels, 10
        )
        self._service_poll_timer = self.create_timer(
            0.05, self._poll_pending, callback_group=self._service_group
        )
        self._push_timer = self.create_timer(
            0.2, self._push_ops_state, callback_group=self._push_group
        )

        self._stop_event = threading.Event()
        self._closed = False
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self._port))
            server.listen(4)
            server.settimeout(0.2)
        except BaseException:
            server.close()
            raise
        self._server_socket = server
        self._port = int(server.getsockname()[1])
        self._server_thread = threading.Thread(
            target=self._serve, name="ops-broker-tcp", daemon=True
        )
        self._server_thread.start()
        self.get_logger().info("ops broker TCP :%d" % self._port)

    # -- ops-state inputs -------------------------------------------------
    def _on_authority(self, message):
        mode = str(message.data).split("|", 1)[0].strip()
        if not mode:
            return
        with self._state_lock:
            self._fields["authority_mode"] = mode
            self._stamps["authority"] = time.monotonic()

    def _on_safety(self, message):
        try:
            decoded = json.loads(message.data)
            stamp_s = float(decoded["stamp_s"])
            if not math.isfinite(stamp_s):
                raise ValueError("stamp_s must be finite")
            estop_latched = bool(decoded["estop_latched"])
            sources = decoded["active_estop_sources"]
            if not isinstance(sources, list):
                raise ValueError("active_estop_sources must be a list")
            active_sources = tuple(sorted(str(item) for item in sources))
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid /chassis/safety_state ignored: %s" % exc
            )
            return
        with self._state_lock:
            self._fields["estop_latched"] = estop_latched
            self._fields["active_estop_sources"] = active_sources
            self._stamps["safety"] = stamp_s

    def _on_gateway(self, message):
        try:
            decoded = json.loads(message.data)
            stamp_s = float(decoded["stamp_s"])
            if not math.isfinite(stamp_s):
                raise ValueError("stamp_s must be finite")
            state = str(decoded["state"]).strip()
            if not state:
                raise ValueError("state must be non-empty")
            input_fresh = bool(decoded["input_fresh"])
            neutral = bool(decoded["neutral"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid /teleop/gateway_state ignored: %s" % exc
            )
            return
        with self._state_lock:
            self._fields["gateway_state"] = state
            self._fields["gateway_input_fresh"] = input_fresh
            self._fields["gateway_neutral"] = neutral
            self._stamps["gateway"] = stamp_s

    def _on_wheels(self, message):
        try:
            stopped = all(
                abs(float(wheel.drive_turns_per_s)) < WHEEL_STOP_TURNS
                for wheel in message.wheels
            )
        except (AttributeError, TypeError, ValueError):
            return
        with self._state_lock:
            self._fields["wheels_stopped"] = bool(stopped)
            self._stamps["wheels"] = time.monotonic()

    def _ops_state(self):
        now_s = time.monotonic()
        with self._state_lock:
            values = {
                "authority_mode": self._fields["authority_mode"] or "UNKNOWN",
                "gateway_state": self._fields["gateway_state"] or "UNKNOWN",
                "gateway_input_fresh": bool(
                    self._fields["gateway_input_fresh"]
                ),
                "gateway_neutral": bool(self._fields["gateway_neutral"]),
                "estop_latched": bool(self._fields["estop_latched"]),
                "active_estop_sources": tuple(
                    self._fields["active_estop_sources"] or ()
                ),
                "wheels_stopped": bool(self._fields["wheels_stopped"]),
            }
            semantic = tuple(values[name] for name in _SEMANTIC_FIELDS)
            if semantic != self._last_semantic:
                self._revision += 1
                self._last_semantic = semantic
            ages = {
                name: (
                    9.9
                    if stamp_s is None
                    else max(0.0, now_s - float(stamp_s))
                )
                for name, stamp_s in self._stamps.items()
            }
            return OpsState(
                revision=self._revision,
                field_age_s=ages,
                **values,
            )

    # -- TCP owner --------------------------------------------------------
    def _serve(self):
        server = self._server_socket
        try:
            while not self._stop_event.is_set():
                try:
                    sock, _address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                thread = threading.Thread(
                    target=self._serve_client,
                    args=(sock,),
                    name="ops-broker-client",
                    daemon=True,
                )
                with self._client_threads_lock:
                    self._client_threads.append(thread)
                thread.start()
        except BaseException as exc:
            if not self._stop_event.is_set():
                self.get_logger().error("ops broker TCP failed: %r" % exc)
        finally:
            try:
                server.close()
            except OSError:
                pass

    def _serve_client(self, sock):
        connection = _Connection(sock=sock, client_key=uuid.uuid4().hex)
        registered = False
        role = None
        buffer = b""
        last_data_s = time.monotonic()
        try:
            sock.settimeout(0.2)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            while not self._stop_event.is_set():
                try:
                    data = sock.recv(4096)
                except socket.timeout:
                    data = None
                except OSError:
                    break
                if data == b"":
                    break
                now_s = time.monotonic()
                if data:
                    last_data_s = now_s
                    buffer += data
                elif now_s - last_data_s > CLIENT_IDLE_TIMEOUT_S:
                    break

                if b"\n" not in buffer and len(buffer) > oc.MAX_RECORD_BYTES:
                    break
                while b"\n" in buffer:
                    raw, _, buffer = buffer.partition(b"\n")
                    line = raw.decode("utf-8", errors="replace")
                    if role is None:
                        with self._core_lock:
                            role, response = self._core.handshake(
                                connection.client_key, line
                            )
                        if not self._send(connection, response):
                            return
                        if role is None:
                            return
                        with self._connections_lock:
                            self._connections.append(connection)
                        registered = True
                        continue

                    with self._core_lock:
                        decision = self._core.handle_line(
                            connection.client_key, role, line
                        )
                    if decision.response is not None:
                        self._send(connection, decision.response)
                        self._journal_immediate(line, role, decision.response)
                    if decision.execute is not None:
                        self._execute(decision.execute, connection, role)
        finally:
            if registered:
                self._remove_connection(connection)
            try:
                sock.close()
            except OSError:
                pass
            current = threading.current_thread()
            with self._client_threads_lock:
                self._client_threads = [
                    thread for thread in self._client_threads
                    if thread is not current
                ]

    def _send(self, connection, payload):
        try:
            with connection.send_lock:
                connection.sock.sendall(payload)
            return True
        except OSError:
            self._remove_connection(connection)
            return False

    def _remove_connection(self, connection):
        with self._connections_lock:
            self._connections = [
                item for item in self._connections if item is not connection
            ]

    # -- rclpy execution proxy -------------------------------------------
    def _client_for(self, target):
        with self._service_clients_lock:
            client = self._service_clients.get(target)
            if client is None:
                client = self.create_client(
                    Trigger, target, callback_group=self._service_group
                )
                self._service_clients[target] = client
            return client

    def _execute(self, order, connection, role):
        if order.kind in ("service", "composite"):
            pending = _PendingService(order=order, connection=connection)
            self._start_service_call(pending, role)
            return
        if order.kind == "publish":
            message = String()
            message.data = json.dumps(
                {
                    "type": order.action.upper(),
                    "stamp_s": time.monotonic(),
                    "payload": dict(order.params),
                },
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            self._section_pub.publish(message)
            self._complete_order(
                order,
                connection,
                role,
                True,
                "published %s" % order.action,
            )
            return
        self._complete_order(
            order, connection, role, False, "unsupported kind: %s" % order.kind
        )

    def _start_service_call(self, pending, role):
        target = pending.order.targets[pending.index]
        try:
            pending.future = self._client_for(target).call_async(
                Trigger.Request()
            )
        except Exception as exc:
            self._record_service_result(
                pending,
                False,
                "error(%s:%s)" % (self._target_name(target), exc),
            )
            self._advance_or_complete(pending, role)
            return
        pending.started_s = time.monotonic()
        pending.timeout_journaled = False
        with self._pending_lock:
            self._pending[pending.order.pending_key] = (pending, role)

    def _poll_pending(self):
        if self._closed:
            return
        with self._pending_lock:
            entries = list(self._pending.items())
        now_s = time.monotonic()
        for pending_key, (pending, role) in entries:
            future = pending.future
            if not future.done():
                if (
                    not pending.timeout_journaled
                    and now_s - pending.started_s
                    >= oc.SERVICE_CALL_TIMEOUT_S
                ):
                    with self._pending_lock:
                        current = self._pending.get(pending_key)
                        if current is None or current[0] is not pending:
                            continue
                        pending.timeout_journaled = True
                    target = pending.order.targets[pending.index]
                    self._journal(
                        severity="WARN",
                        payload={
                            "request_id": pending_key[1],
                            "action": pending.order.action,
                            "role": role,
                            "status": oc.STATUS_PENDING,
                            "target": target,
                            "detail": (
                                "service timeout; awaiting late completion"
                            ),
                        },
                    )
                continue

            with self._pending_lock:
                current = self._pending.get(pending_key)
                if current is None or current[0] is not pending:
                    continue
                del self._pending[pending_key]
            target = pending.order.targets[pending.index]
            try:
                response = future.result()
                success = bool(response.success)
                message = str(response.message).strip()
                if success:
                    result_detail = "ok"
                else:
                    suffix = self._target_name(target)
                    if message:
                        suffix += ":%s" % message
                    result_detail = "failed(%s)" % suffix
            except Exception as exc:
                success = False
                result_detail = "error(%s:%s)" % (
                    self._target_name(target), exc
                )
                message = str(exc)
            self._record_service_result(
                pending, success, result_detail, message=message
            )
            self._advance_or_complete(pending, role)

    @staticmethod
    def _target_name(target):
        return str(target).rstrip("/").rsplit("/", 1)[-1]

    @staticmethod
    def _target_label(target):
        if str(target).startswith("/teleop_command/"):
            return "teleop"
        if str(target).startswith("/chassis_node/"):
            return "chassis"
        return OpsBrokerNode._target_name(target)

    def _record_service_result(self, pending, success, detail, message=""):
        target = pending.order.targets[pending.index]
        pending.results.append(
            {
                "target": target,
                "label": self._target_label(target),
                "success": bool(success),
                "detail": str(detail),
                "message": str(message),
            }
        )

    def _advance_or_complete(self, pending, role):
        if pending.index + 1 < len(pending.order.targets):
            pending.index += 1
            self._start_service_call(pending, role)
            return
        success = all(result["success"] for result in pending.results)
        if pending.order.kind == "composite":
            detail = " ".join(
                "%s=%s" % (result["label"], result["detail"])
                for result in pending.results
            )
        else:
            result = pending.results[0]
            detail = result["message"] or result["detail"]
        self._complete_order(
            pending.order, pending.connection, role, success, detail
        )

    def _complete_order(self, order, connection, role, success, detail):
        with self._core_lock:
            response = self._core.complete(
                order.pending_key, bool(success), str(detail)
            )
        self._send(connection, response)
        decoded = json.loads(response)
        self._journal(
            severity="INFO" if success else "WARN",
            payload={
                "request_id": order.pending_key[1],
                "action": order.action,
                "role": role,
                "status": decoded["status"],
                "state_revision": decoded["state_revision"],
                "detail": decoded["detail"],
            },
        )

    # -- push and journal -------------------------------------------------
    def _push_ops_state(self):
        if self._closed:
            return
        state = self._ops_state()
        payload = (
            json.dumps(
                {
                    "push": "ops_state",
                    "revision": state.revision,
                    "authority_mode": state.authority_mode,
                    "gateway_state": state.gateway_state,
                    "gateway_input_fresh": state.gateway_input_fresh,
                    "gateway_neutral": state.gateway_neutral,
                    "estop_latched": state.estop_latched,
                    "active_estop_sources": list(
                        state.active_estop_sources
                    ),
                    "wheels_stopped": state.wheels_stopped,
                    "field_age_s": dict(state.field_age_s),
                    "stamp_s": time.monotonic(),
                },
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        with self._connections_lock:
            connections = list(self._connections)
        for connection in connections:
            self._send(connection, payload)

    def _journal_immediate(self, line, role, response):
        try:
            decoded = json.loads(response)
        except (TypeError, ValueError):
            return
        if decoded.get("status") == oc.STATUS_PENDING:
            return
        try:
            request = json.loads(line)
            action = str(request.get("action", "invalid"))
        except (TypeError, ValueError):
            action = "invalid"
        status = str(decoded.get("status", ""))
        self._journal(
            severity=(
                "INFO" if status == oc.STATUS_FINAL_SUCCESS else "WARN"
            ),
            payload={
                "request_id": str(decoded.get("request_id", "invalid")),
                "action": action,
                "role": role,
                "status": status,
                "state_revision": int(decoded.get("state_revision", 0)),
                "detail": str(decoded.get("detail", "")),
            },
        )

    def _journal(self, *, severity, payload):
        safe_payload = {
            key: value
            for key, value in dict(payload).items()
            if "token" not in str(key).lower()
        }
        event = {
            "schema_version": 1,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "source": "ops_broker_node",
            "event_type": "OPS_COMMAND",
            "severity": str(severity),
            "payload": safe_payload,
        }
        try:
            self._event_client.emit(event)
        except Exception:
            pass

    # -- lifecycle --------------------------------------------------------
    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        server = self._server_socket
        self._server_socket = None
        if server is not None:
            try:
                server.close()
            except OSError:
                pass
        with self._connections_lock:
            connections = list(self._connections)
            self._connections = []
        for connection in connections:
            try:
                connection.sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                connection.sock.close()
            except OSError:
                pass
        if self._server_thread.is_alive():
            self._server_thread.join(timeout=1.0)
        current = threading.current_thread()
        with self._client_threads_lock:
            threads = list(self._client_threads)
        for thread in threads:
            if thread is not current and thread.is_alive():
                thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = OpsBrokerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
