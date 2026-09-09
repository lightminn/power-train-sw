"""Paired IPv4 discovery and client-side session/channel handshakes."""
import hmac
import secrets
import socket
import threading

from .session import SessionError, _close, _proof, _read, _send, _token


def open_session_channel(host, port, lease_id, ticket, kind):
    if kind not in ('input', 'ops'):
        raise SessionError('invalid channel kind')
    sock = socket.create_connection((host, port), timeout=1)
    try:
        _send(sock, dict(lease_id=lease_id, ticket=ticket, kind=kind))
        if _read(sock).get('status') != 'SUCCEEDED':
            raise SessionError('channel rejected')
        sock.settimeout(None)
        return sock
    except BaseException:
        _close(sock)
        raise


class SessionClient:
    def __init__(self, config):
        self.config = dict(config)
        self._key = _token(config['token_file'])
        self._socket = None
        self._lock = threading.RLock()
        self.host = None
        self.snapshot = {'connected': False}

    def connect(self):
        with self._lock:
            if self._socket is not None:
                return self.heartbeat()
            addresses = []
            for host in self.config['hosts']:
                try:
                    candidates = socket.getaddrinfo(host, self.config.get('session_port', 9002),
                                                    socket.AF_INET, socket.SOCK_STREAM)
                except OSError:
                    continue
                for candidate in candidates:
                    if candidate[4] not in addresses:
                        addresses.append(candidate[4])
            for address in addresses:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(self.config.get('timeout_s', 1))
                try:
                    sock.connect(address)
                    robot_id = self.config['robot_id']
                    nonce = secrets.token_hex(32)
                    _send(sock, dict(robot_id=robot_id, nonce=nonce))
                    reply = _read(sock)
                    challenge, boot_id = reply.get('nonce'), reply.get('boot_id')
                    if (reply.get('robot_id') != robot_id or not isinstance(challenge, str)
                            or len(challenge) != 64 or not isinstance(boot_id, str)
                            or not isinstance(reply.get('proof'), str)
                            or not hmac.compare_digest(reply['proof'], _proof(self._key, 'server', robot_id,
                                                                             boot_id, nonce, challenge))):
                        raise SessionError('robot authentication failed')
                    _send(sock, {'proof': _proof(self._key, 'client', robot_id, boot_id, nonce, challenge)})
                    snapshot = _read(sock)
                    if not snapshot.get('connected') or snapshot.get('robot_id') != robot_id:
                        raise SessionError('lease rejected')
                    self._socket, self.host, self.snapshot = sock, address[0], snapshot
                    return dict(snapshot)
                except (OSError, ValueError, TypeError):
                    _close(sock)
            self.snapshot = {'connected': False}
            raise SessionError('paired robot unavailable or lease rejected')

    def request(self, op):
        with self._lock:
            if self._socket is None:
                raise SessionError('not connected')
            try:
                _send(self._socket, {'op': op})
                snapshot = _read(self._socket)
                self.snapshot = snapshot
                return dict(snapshot)
            except (OSError, ValueError, TypeError) as exc:
                _close(self._socket)
                self._socket = None
                self.snapshot = {'connected': False}
                raise SessionError('session lost') from exc

    def heartbeat(self):
        return self.request('heartbeat')

    def open_channel(self, kind):
        with self._lock:
            if self._socket is None or not self.snapshot.get('connected'):
                raise SessionError('not connected')
            if kind not in ('input', 'ops'):
                raise SessionError('invalid channel kind')
            return open_session_channel(self.host, self.snapshot[f'{kind}_port'],
                                        self.snapshot['lease_id'], self.snapshot['ticket'], kind)

    def close(self):
        with self._lock:
            result = {'status': 'IDLE', 'connected': False}
            if self._socket is not None:
                try:
                    result = self.request('release')
                except OSError:
                    result = {'status': 'OUTCOME_UNKNOWN', 'connected': False}
                finally:
                    if self._socket is not None:
                        _close(self._socket)
                    self._socket = None
            self.snapshot = {'connected': False}
            return result
