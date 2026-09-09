"""Bounded mutual-HMAC session owner and authenticated TCP proxies.

Authentication provides pair identity and replay resistance, not encryption.
The inner input watchdog and chassis safety policies remain authoritative.
"""
import errno
import hashlib
import hmac
import json
import math
import os
from pathlib import Path
import secrets
import select
import socket
import threading
import time

MAX_RECORD = 8192


class SessionError(OSError):
    """A transport, authentication, or ownership failure (safe to display)."""


def _proof(key, *parts):
    payload = json.dumps(parts, separators=(',', ':'), ensure_ascii=True).encode()
    return hmac.new(key, payload, hashlib.sha256).hexdigest()


def _token(path):
    key = Path(path).expanduser().read_bytes().strip()
    if not key or len(key) > 4096:
        raise ValueError('invalid token file')
    return key


def _send(sock, message):
    raw = json.dumps(message, separators=(',', ':'), allow_nan=False).encode() + b'\n'
    if len(raw) > MAX_RECORD:
        raise SessionError('record too large')
    sock.sendall(raw)


def _read(sock):
    # Bound the entire record, including a slow peer sending one byte at a time.
    original_timeout = sock.gettimeout()
    deadline = time.monotonic() + (original_timeout or 1)
    data = bytearray()
    try:
        while len(data) < MAX_RECORD:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise SessionError('record timed out')
            sock.settimeout(remaining)
            piece = sock.recv(1)
            if not piece:
                raise SessionError('connection closed')
            if piece == b'\n':
                try:
                    value = json.loads(data)
                except (ValueError, UnicodeError) as exc:
                    raise SessionError('invalid record') from exc
                if not isinstance(value, dict):
                    raise SessionError('record must be an object')
                return value
            data.extend(piece)
        raise SessionError('record too large')
    finally:
        try:
            sock.settimeout(original_timeout)
        except OSError:
            pass


def _close(sock):
    try:
        sock.shutdown(socket.SHUT_RDWR)
    except OSError:
        pass
    sock.close()


class SessionServer:
    """One lease, two proxy channels, and a fixed maximum of live workers."""

    def __init__(self, config, on_start, on_stop, state_provider):
        self.config = dict(config)
        self.on_start, self.on_stop, self.state_provider = on_start, on_stop, state_provider
        self.robot_id = config['robot_id']
        self._key = _token(config['token_file'])
        self.boot_id = secrets.token_hex(16)
        self._timeout = float(config.get('lease_timeout_s', 2))
        if not math.isfinite(self._timeout) or not .2 <= self._timeout <= 30:
            raise ValueError('invalid lease timeout')
        self._lock = threading.RLock()
        self._done = threading.Event()
        self._listener_error = None
        self._slots = threading.BoundedSemaphore(8)
        self._workers = set()
        self._sockets = set()
        self._listeners = []
        self._threads = []
        self._lease = None
        self._cleanup_pending = False
        self._stop_result = {'status': 'IDLE'}
        self._started = False
        for kind in ('session', 'input', 'ops'):
            setattr(self, f'{kind}_port', config.get(f'{kind}_port', {'session': 9002, 'input': 9000, 'ops': 9001}[kind]))

    def start(self):
        if self._started:
            return self
        self._started = True
        try:
            for kind in ('session', 'input', 'ops'):
                listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self._listeners.append(listener)
                listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                listener.bind((self.config.get('host', '0.0.0.0'), getattr(self, f'{kind}_port')))
                listener.listen(8)
                listener.settimeout(.1)
                setattr(self, f'{kind}_port', listener.getsockname()[1])
            # Reserve every public port before touching another owner's record.
            self._remove_destination()
            for listener, kind in zip(self._listeners, ('session', 'input', 'ops')):
                thread = threading.Thread(target=self._accept, args=(listener, kind), daemon=True)
                thread.start()
                self._threads.append(thread)
            thread = threading.Thread(target=self._watch, daemon=True)
            thread.start()
            self._threads.append(thread)
        except BaseException:
            self.close()
            raise
        return self

    def _accept(self, listener, kind):
        while not self._done.is_set():
            try:
                conn, peer = listener.accept()
            except socket.timeout:
                continue
            except OSError as exc:
                if self._done.is_set():
                    return
                if exc.errno in (errno.EBADF, errno.ENOTSOCK, errno.EINVAL):
                    # A broken listener needs process-level recreation. The
                    # service supervisor checks this and runs normal cleanup.
                    self._listener_error = exc
                    self._done.set()
                    return
                # Linux may surface an aborted pending connection (or temporary
                # resource exhaustion) from accept. Keep the listening socket,
                # with bounded backoff that is interruptible during shutdown.
                self._done.wait(.05)
                continue
            if not self._slots.acquire(blocking=False):
                _close(conn)
                continue
            with self._lock:
                if self._done.is_set():
                    _close(conn)
                    self._slots.release()
                    return
                self._sockets.add(conn)
                worker = None
                try:
                    worker = threading.Thread(target=self._handle, args=(conn, peer, kind), daemon=True)
                    self._workers.add(worker)
                    worker.start()
                except Exception as exc:
                    self._workers.discard(worker)
                    self._sockets.discard(conn)
                    _close(conn)
                    self._slots.release()
                    self._listener_error = exc
                    self._done.set()
                    return

    def check_health(self):
        """Raise when the process supervisor must recreate broken listeners."""
        if self._listener_error is not None:
            raise SessionError('session listener failed; restart required') from self._listener_error

    def _handle(self, conn, peer, kind):
        try:
            conn.settimeout(1)
            if kind == 'session':
                self._control(conn, peer)
            else:
                self._proxy(conn, peer, kind)
        except (OSError, ValueError, TypeError, KeyError):
            try:
                _send(conn, {'status': 'REJECTED', 'reason': 'session rejected'})
            except (OSError, ValueError):
                pass
        finally:
            with self._lock:
                if self._lease and self._lease['socket'] is conn:
                    self._revoke()
                self._sockets.discard(conn)
                self._workers.discard(threading.current_thread())
            _close(conn)
            self._slots.release()

    def _call(self, callback):
        try:
            value = callback()
            if not isinstance(value, dict):
                raise ValueError('callback result')
            encoded = json.dumps(value, allow_nan=False)
            if len(encoded) > MAX_RECORD // 2:
                raise ValueError('callback result size')
            return value
        except Exception:
            return {'status': 'REJECTED', 'reason': 'service callback failed'}

    def _snapshot(self):
        state = self._call(self.state_provider)
        lease = self._lease
        return {**state, 'connected': lease is not None, 'robot_id': self.robot_id,
                'boot_id': self.boot_id, 'lease_id': lease['id'] if lease else None,
                'ticket': lease['ticket'] if lease else None,
                'input_port': self.input_port, 'ops_port': self.ops_port}

    def _control(self, conn, peer):
        hello = _read(conn)
        nonce = hello.get('nonce')
        if hello.get('robot_id') != self.robot_id or not isinstance(nonce, str) or len(nonce) != 64:
            raise SessionError('wrong robot')
        challenge = secrets.token_hex(32)
        _send(conn, dict(robot_id=self.robot_id, boot_id=self.boot_id, nonce=challenge,
                         proof=_proof(self._key, 'server', self.robot_id, self.boot_id, nonce, challenge)))
        response = _read(conn)
        expected = _proof(self._key, 'client', self.robot_id, self.boot_id, nonce, challenge)
        if not isinstance(response.get('proof'), str) or not hmac.compare_digest(response['proof'], expected):
            raise SessionError('authentication failed')
        with self._lock:
            self._expire()
            if self._cleanup_pending:
                state = self._call(self.state_provider)
                self._cleanup_pending = state.get('status') == 'PENDING'
            if self._done.is_set() or self._lease or self._cleanup_pending:
                raise SessionError('operator already connected or stop pending')
            self._lease = dict(id=secrets.token_hex(16), ticket=secrets.token_hex(32),
                               socket=conn, host=peer[0], expires=time.monotonic() + self._timeout,
                               channels={}, start_at=None)
            self._write_destination()
            _send(conn, self._snapshot())
        conn.settimeout(self._timeout + .2)
        while not self._done.is_set():
            request = _read(conn)
            with self._lock:
                self._expire()
                if not self._lease or self._lease['socket'] is not conn:
                    return
                op = request.get('op')
                result = {}
                if op == 'heartbeat':
                    self._lease['expires'] = time.monotonic() + self._timeout
                    self._write_destination()
                elif op == 'start_begin':
                    self._lease['start_at'] = time.monotonic()
                    result = {'status': 'PENDING'}
                elif op == 'start':
                    started = self._lease['start_at']
                    self._lease['start_at'] = None
                    result = (self._call(self.on_start) if started is not None and time.monotonic() - started >= 1.5
                              else {'status': 'REJECTED', 'reason': 'start requires a 1.5 second hold'})
                elif op == 'start_cancel':
                    self._lease['start_at'] = None
                    result = {'status': 'SUCCEEDED'}
                elif op == 'stop':
                    self._lease['start_at'] = None
                    result = self._call(self.on_stop)
                elif op == 'release':
                    result = self._revoke(close_control=False)
                    _send(conn, {**result, 'connected': False})
                    return
                else:
                    result = {'status': 'REJECTED', 'reason': 'unknown operation'}
                _send(conn, {**self._snapshot(), **result})

    def _proxy(self, conn, peer, kind):
        request = _read(conn)
        target = None
        lease = None
        try:
            with self._lock:
                self._expire()
                lease = self._lease
                ticket = request.get('ticket')
                if (not lease or request.get('lease_id') != lease['id'] or request.get('kind') != kind
                        or peer[0] != lease['host'] or not isinstance(ticket, str)
                        or not hmac.compare_digest(ticket, lease['ticket']) or kind in lease['channels']):
                    raise SessionError('channel rejected')
                if kind == 'input':
                    snapshot = self._call(self.state_provider)
                    if not self._confirmed_stationary(snapshot, chassis_mode='IDLE', estop_latched=False):
                        # A stopped, latched ESTOP still rejects input, but pad
                        # retries must leave the ops mutation gate free for
                        # recovery. Uncertain or unsafe states still stop.
                        if not self._confirmed_stationary(snapshot, chassis_mode='ESTOP', estop_latched=True):
                            self._call(self.on_stop)
                        raise SessionError('waiting for confirmed idle chassis')
                target = socket.create_connection(('127.0.0.1', self.config[f'{kind}_target_port']), timeout=.3)
                lease['channels'][kind] = (conn, target)
                self._sockets.add(target)
                _send(conn, {'status': 'SUCCEEDED'})
            conn.settimeout(.3)
            target.settimeout(.3)
            while not self._done.is_set():
                readers, _, _ = select.select([conn, target], [], [], .1)
                for source in readers:
                    data = source.recv(16384)
                    if not data:
                        return
                    (target if source is conn else conn).sendall(data)
        finally:
            with self._lock:
                if lease and lease['channels'].get(kind) == (conn, target):
                    # A pad/channel flap must not inherit an armed authority,
                    # even if it reconnects faster than the raw input watchdog.
                    # Only the authenticated, registered channel can revoke.
                    if kind == 'input' and self._lease is lease:
                        self._revoke()
                    lease['channels'].pop(kind)
                if target is not None:
                    self._sockets.discard(target)
                    _close(target)

    @staticmethod
    def _confirmed_stationary(snapshot, *, chassis_mode, estop_latched):
        if snapshot.get('status') == 'PENDING':
            return False
        state = snapshot.get('ops_state')
        if not isinstance(state, dict):
            return False
        if (state.get('chassis_mode') != chassis_mode or state.get('authority_mode') not in ('IDLE', 'MOTION_HOLD')
                or state.get('estop_latched') is not estop_latched or state.get('wheels_stopped') is not True):
            return False
        ages = state.get('field_age_s', {})
        if not isinstance(ages, dict):
            return False
        # ManualDriveService.snapshot already adds its local receive age.
        return all(type(ages.get(key)) in (float, int) and math.isfinite(ages[key]) and 0 <= ages[key] <= .5
                   for key in ('authority', 'safety', 'wheels'))

    def _write_destination(self):
        destination = self.config.get('destination_file')
        if destination is None:
            return
        path = Path(destination)
        lease = self._lease
        raw = json.dumps(dict(host=lease['host'], lease_id=lease['id'], expires_at=lease['expires']))
        temporary = path.with_name(path.name + '.' + self.boot_id + '.tmp')
        try:
            fd = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
            with os.fdopen(fd, 'w') as stream:
                stream.write(raw)
            os.replace(temporary, path)
        finally:
            temporary.unlink(missing_ok=True)

    def _remove_destination(self):
        if self.config.get('destination_file') is not None:
            Path(self.config['destination_file']).unlink(missing_ok=True)

    def _revoke(self, close_control=True):
        lease = self._lease
        if lease is None:
            return self._stop_result
        # Keep ownership until the stop request and channel cleanup both finish.
        for pair in list(lease['channels'].values()):
            for sock in pair:
                _close(sock)
        try:
            self._remove_destination()
        except OSError:
            pass  # Its short monotonic expiry still suppresses stale delivery.
        self._stop_result = self._call(self.on_stop)
        self._cleanup_pending = self._stop_result.get('status') == 'PENDING'
        self._lease = None
        if close_control:
            _close(lease['socket'])
        return self._stop_result

    def _expire(self):
        if self._lease and time.monotonic() >= self._lease['expires']:
            self._revoke()

    def _watch(self):
        while not self._done.wait(.025):
            with self._lock:
                self._expire()

    def close(self):
        self._done.set()
        with self._lock:
            result = self._revoke()
            for sock in list(self._sockets) + self._listeners:
                _close(sock)
            workers = list(self._workers)
        for thread in self._threads + workers:
            if thread is not threading.current_thread():
                thread.join(timeout=1.2)
        return result
