"""Exercise the broker's accept loop on a real TCP listener."""
import errno
import socket
import threading
from types import SimpleNamespace

import pytest

from test_ops_state_sources import _extract_broker_method, _broker_harness, _OPS_STATE, _PUSH_OPS_STATE


@pytest.mark.parametrize('error', [errno.ECONNABORTED, errno.EMFILE])
def test_transient_accept_failure_still_dispatches_next_connection(error):
    listener = socket.socket()
    listener.bind(('127.0.0.1', 0))
    listener.listen(2)
    listener.settimeout(.05)
    peer = socket.create_connection(listener.getsockname(), timeout=.5)
    accepted = threading.Event()

    class FailOnce:
        failed = False

        def accept(self):
            if not self.failed:
                self.failed = True
                raise OSError(error, 'injected failure')
            return listener.accept()

        def close(self):
            listener.close()

    def handle(sock):
        accepted.set()
        sock.close()

    node = SimpleNamespace(_server_socket=FailOnce(), _stop_event=threading.Event(),
        _serve_client=handle, _client_threads_lock=threading.Lock(), _client_threads=[],
        get_logger=lambda: SimpleNamespace(error=lambda *_: None))
    run = _extract_broker_method('_serve')
    thread = threading.Thread(target=run, args=(node,))
    thread.start()
    try:
        assert accepted.wait(.5), 'transient accept must not permanently terminate the listener'
    finally:
        node._stop_event.set()
        thread.join(1)
        peer.close()
        listener.close()


def test_listener_failure_surfaces_on_executor_for_process_restart():
    node = _broker_harness()
    node._closed = False
    node._tcp_error = OSError(errno.EBADF, 'listener lost')
    node._ops_state = lambda: _OPS_STATE(node)
    node._connections_lock, node._connections = threading.Lock(), []
    with pytest.raises(RuntimeError, match='listener'):
        _PUSH_OPS_STATE(node)
