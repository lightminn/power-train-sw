"""Independent-client Unix socket control server."""

import os
import socket
import threading

from .protocol import ProtocolError, decode_request, encode_message, response


class UnixControlServer:
    def __init__(self, path, handler, *, max_message_bytes=65536, on_disconnect=None):
        self.path = str(path); self._handler = handler; self._max = int(max_message_bytes)
        self._on_disconnect = on_disconnect or (lambda: None)
        self._stop = threading.Event(); self._socket = None; self._thread = None
        self._clients = set(); self._lock = threading.Lock()

    def start(self):
        if self._socket is not None: return
        os.makedirs(os.path.dirname(self.path) or ".", exist_ok=True)
        try: os.unlink(self.path)
        except FileNotFoundError: pass
        sock = socket.socket(socket.AF_UNIX); sock.bind(self.path); sock.listen(16); sock.settimeout(.1)
        self._socket = sock; self._stop.clear()
        self._thread = threading.Thread(target=self._accept, daemon=True, name="l515-control-server"); self._thread.start()

    def _accept(self):
        while not self._stop.is_set():
            try: client, _ = self._socket.accept()
            except socket.timeout: continue
            except OSError: return
            client.settimeout(.2)
            with self._lock: self._clients.add(client)
            threading.Thread(target=self._serve, args=(client,), daemon=True).start()

    def _serve(self, client):
        buffer = bytearray()
        try:
            while not self._stop.is_set():
                try: chunk = client.recv(4096)
                except socket.timeout: continue
                if not chunk: return
                buffer.extend(chunk)
                if len(buffer) > self._max and b"\n" not in buffer:
                    client.sendall(encode_message(response(None, error="message exceeds size limit"), self._max)); return
                while b"\n" in buffer:
                    raw, _, remainder = buffer.partition(b"\n"); buffer = bytearray(remainder)
                    try:
                        request = decode_request(raw, self._max)
                        payload = self._handler(request)
                        output = response(request["request_id"], payload)
                    except ProtocolError as exc: output = response(None, error=str(exc))
                    except Exception as exc: output = response(None, error=f"command failed: {exc}")
                    client.sendall(encode_message(output, self._max))
        except (BrokenPipeError, ConnectionError, OSError):
            pass
        finally:
            with self._lock: self._clients.discard(client)
            try: client.close()
            finally: self._on_disconnect()

    def stop(self):
        if self._socket is None: return
        self._stop.set(); sock, self._socket = self._socket, None; sock.close()
        with self._lock: clients=list(self._clients)
        for client in clients:
            try: client.shutdown(socket.SHUT_RDWR)
            except OSError: pass
            client.close()
        if self._thread and self._thread is not threading.current_thread(): self._thread.join(1)
        try: os.unlink(self.path)
        except FileNotFoundError: pass

