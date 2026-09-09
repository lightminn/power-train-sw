"""Recover real authenticated sessions after an injected accept boundary error."""
import errno
import threading

import pytest

from powertrain_runtime.client import SessionClient
from powertrain_runtime.session import SessionError, SessionServer


@pytest.mark.parametrize('error', [errno.ECONNABORTED, errno.EINTR, errno.EPROTO, errno.EMFILE])
def test_accept_error_does_not_disable_authenticated_reconnection(tmp_path, monkeypatch, error):
    token = tmp_path / 'pair.token'
    token.write_text('test-only-pair-key')
    config = dict(robot_id='listener-recovery', token_file=str(token), host='127.0.0.1',
                  session_port=0, input_port=0, ops_port=0)
    original = SessionServer._accept

    class FailOnce:
        def __init__(self, listener):
            self.listener, self.failed = listener, False

        def accept(self):
            if not self.failed:
                self.failed = True
                raise OSError(error, 'injected accept failure')
            return self.listener.accept()

    def wrapped(server, listener, kind):
        return original(server, FailOnce(listener), kind)

    monkeypatch.setattr(SessionServer, '_accept', wrapped)
    server = SessionServer(config, lambda: {}, lambda: {'status': 'SUCCEEDED'}, lambda: {})
    client = None
    try:
        server.start()
        client = SessionClient(dict(config, hosts=['127.0.0.1'],
                                    session_port=server.session_port, timeout_s=.8))
        first = client.connect()
        assert first['connected'] is True
        assert client.heartbeat()['lease_id'] == first['lease_id']
        client.close()
        assert client.connect()['lease_id'] != first['lease_id']
    finally:
        if client is not None:
            client.close()
        server.close()


def test_unrecoverable_listener_failure_is_reported_to_service_supervisor(tmp_path):
    token = tmp_path / 'pair.token'
    token.write_text('test-only-pair-key')
    server = SessionServer(dict(robot_id='listener-recovery', token_file=str(token)),
                           lambda: {}, lambda: {}, lambda: {})

    class ClosedListener:
        def accept(self):
            raise OSError(errno.EBADF, 'invalid listener')

    thread = threading.Thread(target=server._accept, args=(ClosedListener(), 'session'))
    thread.start()
    thread.join(.5)
    assert not thread.is_alive()
    with pytest.raises(SessionError, match='listener'):
        server.check_health()


def test_worker_start_failure_is_supervised_and_releases_connection(tmp_path, monkeypatch):
    import socket
    token = tmp_path / 'pair.token'
    token.write_text('test-only-pair-key')
    server = SessionServer(dict(robot_id='worker-recovery', token_file=str(token)),
                           lambda: {}, lambda: {}, lambda: {})
    conn, peer = socket.socketpair()

    class Listener:
        def accept(self):
            return conn, ('127.0.0.1', 1)

    def cannot_start(_):
        raise RuntimeError("can't start new thread")

    monkeypatch.setattr(threading.Thread, 'start', cannot_start)
    try:
        server._accept(Listener(), 'session')
        with pytest.raises(SessionError, match='listener'):
            server.check_health()
        assert not server._workers and not server._sockets
        assert conn.fileno() == -1
        assert all(server._slots.acquire(blocking=False) for _ in range(8))
        assert not server._slots.acquire(blocking=False)
        server.close()  # Must not join an unstarted worker.
    finally:
        conn.close()
        peer.close()
