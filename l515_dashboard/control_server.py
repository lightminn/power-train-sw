"""Bounded independent-client Unix control server."""

from dataclasses import dataclass
import os
import queue
import socket
import struct
import threading

from .endpoint import abstract_address
from .protocol import ProtocolError, decode_request, encode_message, response


@dataclass(frozen=True)
class DeferredResponse:
    payload: dict
    after_send: object = None


class UnixControlServer:
    def __init__(self, path, handler, *, max_message_bytes=65536,
                 on_disconnect=None, on_action_error=None, max_clients=8,
                 idle_timeout_s=5.0, peer_authorizer=None):
        self.path = str(path)
        self._handler = handler
        self._max = int(max_message_bytes)
        self._on_disconnect = on_disconnect or (lambda: None)
        self._on_action_error = on_action_error or (lambda _exc: None)
        self.last_action_error = None
        self._max_clients = int(max_clients)
        self._idle_timeout = float(idle_timeout_s)
        self._owner_guard = None
        self._socket_factory = socket.socket
        self._stop_event = threading.Event()
        self._socket = None
        self._thread = None
        self._clients = {}
        self._lock = threading.Lock()
        self._peer_authorizer = peer_authorizer or self._same_uid
        self._action_capacity = int(max_clients)
        self._actions = queue.Queue(maxsize=self._action_capacity)
        self._action_thread = None

    def require_owner(self, guard):
        self._owner_guard = guard
        return self

    @staticmethod
    def _same_uid(client):
        raw = client.getsockopt(socket.SOL_SOCKET, socket.SO_PEERCRED,
                                struct.calcsize("3i"))
        _pid, uid, _gid = struct.unpack("3i", raw)
        return uid == os.geteuid()

    def start(self):
        if self._socket is not None:
            return
        if self._action_thread is not None:
            self._actions = queue.Queue(maxsize=self._action_capacity)
        if self._owner_guard is not None and not self._owner_guard.acquired:
            raise RuntimeError("socket owner guard is not acquired")
        sock = self._socket_factory(socket.AF_UNIX)
        try:
            sock.bind(abstract_address(self.path))
            sock.listen(self._max_clients)
            sock.settimeout(0.1)
        except Exception:
            sock.close()
            raise
        self._socket = sock
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._accept, daemon=True,
                                        name="l515-control-server")
        self._action_thread = threading.Thread(target=self._run_actions,
                                               name="l515-control-actions")
        try:
            self._action_thread.start()
            self._thread.start()
        except Exception:
            self._stop_event.set()
            sock.close()
            self._socket = None
            if self._action_thread.is_alive():
                self._actions.put(None)
                self._action_thread.join(1)
            raise

    def _run_actions(self):
        while True:
            item = self._actions.get()
            if item is None:
                return
            gate, cancelled, action = item
            while not gate.wait(.05):
                if self._stop_event.is_set():
                    break
            if not self._stop_event.is_set() and not cancelled.is_set():
                try:
                    action()
                except Exception as exc:
                    self.last_action_error = str(exc)
                    self._on_action_error(exc)

    def _accept(self):
        while not self._stop_event.is_set():
            try:
                client, _ = self._socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
            try:
                authorized = self._peer_authorizer(client)
            except Exception:
                authorized = False
            if not authorized:
                client.close()
                continue
            with self._lock:
                if len(self._clients) >= self._max_clients:
                    client.close()
                    continue
                client.settimeout(self._idle_timeout)
                thread = threading.Thread(target=self._serve, args=(client,),
                                          daemon=True, name="l515-control-client")
                self._clients[client] = thread
                thread.start()

    def _serve(self, client):
        buffer = bytearray()
        try:
            while not self._stop_event.is_set():
                try:
                    chunk = client.recv(4096)
                except socket.timeout:
                    return
                if not chunk:
                    return
                buffer.extend(chunk)
                if len(buffer) > self._max and b"\n" not in buffer:
                    client.sendall(encode_message(
                        response(None, error="message exceeds size limit"), self._max))
                    return
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n")
                    buffer = bytearray(remainder)
                    action = None
                    gate = None
                    cancelled = None
                    try:
                        request = decode_request(raw, self._max)
                        if self._stop_event.is_set():
                            return
                        result = self._handler(request)
                        if self._stop_event.is_set():
                            return
                        if isinstance(result, DeferredResponse):
                            payload, action = result.payload, result.after_send
                            if action is not None:
                                gate = threading.Event()
                                cancelled = threading.Event()
                                try:
                                    self._actions.put_nowait((gate, cancelled, action))
                                except queue.Full:
                                    action = None
                                    output = response(request["request_id"],
                                                      error="action queue is full")
                                    client.sendall(encode_message(output, self._max))
                                    continue
                        else:
                            payload = result
                        output = response(request["request_id"], payload)
                    except ProtocolError as exc:
                        output = response(exc.request_id, error=str(exc))
                    except Exception as exc:
                        output = response(None, error=f"command failed: {exc}")
                    try:
                        client.sendall(encode_message(output, self._max))
                    except Exception:
                        if gate is not None:
                            cancelled.set()
                            gate.set()
                        raise
                    if gate is not None:
                        gate.set()
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        finally:
            with self._lock:
                self._clients.pop(client, None)
            client.close()
            self._on_disconnect()

    def stop(self):
        sock = self._socket
        if sock is None:
            return
        self._socket = None
        self._stop_event.set()
        sock.close()
        with self._lock:
            clients = list(self._clients.items())
        for client, _ in clients:
            try:
                client.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            client.close()
        current = threading.current_thread()
        if self._thread is not current:
            self._thread.join()
        for _, thread in clients:
            if thread is not current:
                thread.join()
        while True:
            try:
                item = self._actions.get_nowait()
                if item is not None:
                    item[0].set()
            except queue.Empty:
                break
        self._actions.put(None)
        if self._action_thread is not current:
            self._action_thread.join()
