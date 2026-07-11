"""Bounded independent-client Unix control server."""

from dataclasses import dataclass
import os
import socket
import threading

from .protocol import ProtocolError, decode_request, encode_message, response


@dataclass(frozen=True)
class DeferredResponse:
    payload: dict
    after_send: object = None


class UnixControlServer:
    def __init__(self, path, handler, *, max_message_bytes=65536,
                 on_disconnect=None, max_clients=8, idle_timeout_s=5.0):
        self.path = str(path)
        self._handler = handler
        self._max = int(max_message_bytes)
        self._on_disconnect = on_disconnect or (lambda: None)
        self._max_clients = int(max_clients)
        self._idle_timeout = float(idle_timeout_s)
        self._owner_guard = None
        self._stop_event = threading.Event()
        self._socket = None
        self._thread = None
        self._clients = {}
        self._lock = threading.Lock()
        self._socket_identity = None

    def require_owner(self, guard):
        self._owner_guard = guard
        return self

    def start(self):
        if self._socket is not None:
            return
        if self._owner_guard is not None and not self._owner_guard.acquired:
            raise RuntimeError("socket owner guard is not acquired")
        parent = os.path.dirname(self.path) or "."
        if not os.path.exists(parent):
            os.makedirs(parent, mode=0o750)
        # Never unlink here: stale removal belongs exclusively to ResourceGuard.
        sock = socket.socket(socket.AF_UNIX)
        try:
            sock.bind(self.path)
            os.chmod(self.path, 0o660)
            stat = os.stat(self.path)
            self._socket_identity = (stat.st_dev, stat.st_ino)
            sock.listen(self._max_clients)
            sock.settimeout(0.1)
        except Exception:
            sock.close()
            raise
        self._socket = sock
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._accept, daemon=True,
                                        name="l515-control-server")
        self._thread.start()

    def _accept(self):
        while not self._stop_event.is_set():
            try:
                client, _ = self._socket.accept()
            except socket.timeout:
                continue
            except OSError:
                return
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
                    try:
                        request = decode_request(raw, self._max)
                        result = self._handler(request)
                        if isinstance(result, DeferredResponse):
                            payload, action = result.payload, result.after_send
                        else:
                            payload = result
                        output = response(request["request_id"], payload)
                    except ProtocolError as exc:
                        output = response(exc.request_id, error=str(exc))
                    except Exception as exc:
                        output = response(None, error=f"command failed: {exc}")
                    client.sendall(encode_message(output, self._max))
                    if action is not None:
                        threading.Thread(target=action, daemon=True,
                                         name="l515-control-action").start()
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
            self._thread.join(1)
        for _, thread in clients:
            if thread is not current:
                thread.join(1)
        try:
            stat = os.stat(self.path)
            if (stat.st_dev, stat.st_ino) == self._socket_identity:
                os.unlink(self.path)
        except FileNotFoundError:
            pass
        self._socket_identity = None
