"""Real TCP regression tests for ownership and authenticated forwarding."""
import importlib
import json
import socket
import socketserver
import threading
import time

import pytest


class Echo(socketserver.BaseRequestHandler):
    def handle(self):
        while data := self.request.recv(8192):
            self.request.sendall(data)


def safe_admission_state(**changes):
    state = dict(chassis_mode='IDLE', authority_mode='IDLE', estop_latched=False,
                 wheels_stopped=True, field_age_s=dict(authority=0, safety=0, wheels=0))
    state.update(changes)
    return dict(status='IDLE', ops_state=state, drive={'status': 'IDLE'})


@pytest.fixture
def runtime(tmp_path):
    try:
        api = importlib.import_module('powertrain_runtime.session')
        client_api = importlib.import_module('powertrain_runtime.client')
    except ModuleNotFoundError:
        pytest.fail('authenticated session runtime is not implemented')
    token = tmp_path / 'token'
    token.write_text('a-private-paired-key-with-enough-entropy')
    echo = socketserver.ThreadingTCPServer(('127.0.0.1', 0), Echo)
    echo.daemon_threads = True
    thread = threading.Thread(target=echo.serve_forever, daemon=True)
    thread.start()
    stopped = threading.Event()
    config = dict(robot_id='paired-robot', token_file=str(token), host='127.0.0.1',
                  session_port=0, input_port=0, ops_port=0,
                  input_target_port=echo.server_address[1], ops_target_port=echo.server_address[1],
                  destination_file=str(tmp_path / 'destination.json'), lease_timeout_s=.4)
    server = api.SessionServer(config, lambda: {'status': 'SUCCEEDED'},
                               lambda: (stopped.set() or {'status': 'SUCCEEDED'}),
                               safe_admission_state)
    server.start()
    client_config = dict(robot_id='paired-robot', token_file=str(token),
                         hosts=['127.0.0.1'], session_port=server.session_port, timeout_s=.5)
    clients = []
    def client(**changes):
        c = client_api.SessionClient(dict(client_config, **changes))
        clients.append(c)
        return c
    yield server, client, config, stopped, client_api
    for c in clients:
        c.close()
    server.close()
    echo.shutdown()
    echo.server_close()


def test_wrong_key_and_robot_are_rejected(runtime, tmp_path):
    _, client, _, _, _ = runtime
    bad = tmp_path / 'bad-token'
    bad.write_text('wrong-paired-key')
    with pytest.raises(OSError):
        client(token_file=str(bad)).connect()
    with pytest.raises(OSError):
        client(robot_id='other-robot').connect()
    assert client().connect()['connected'] is True


def test_failed_duplicate_start_keeps_original_destination(runtime):
    server, client, config, _, _ = runtime
    from powertrain_runtime.session import SessionServer
    from pathlib import Path
    owner = client()
    owner.connect()
    path = Path(config['destination_file'])
    before = path.read_text()
    duplicate = SessionServer(dict(config, session_port=server.session_port,
                                   input_port=server.input_port, ops_port=server.ops_port),
                              lambda: {}, lambda: {}, lambda: {})
    with pytest.raises(OSError):
        duplicate.start()
    assert path.read_text() == before
    assert owner.heartbeat()['connected']


def test_single_owner_real_forwarding_and_single_channel(runtime):
    _, client, _, _, _ = runtime
    owner = client()
    snapshot = owner.connect()
    assert snapshot['robot_id'] == 'paired-robot'
    with pytest.raises(OSError):
        client().connect()
    with owner.open_channel('input') as channel:
        assert channel.getblocking()
        channel.settimeout(1)
        channel.sendall(b'raw-input\n')
        assert channel.recv(100) == b'raw-input\n'
        with pytest.raises(OSError):
            owner.open_channel('input')
        with owner.open_channel('ops') as ops:
            ops.settimeout(1)
            ops.sendall(b'gated-ops\n')
            assert ops.recv(100) == b'gated-ops\n'


def test_expiry_stops_and_closes_channel_revokes_ticket(runtime):
    server, client, config, stopped, api = runtime
    owner = client()
    snapshot = owner.connect()
    channel = owner.open_channel('input')
    channel.settimeout(2)
    assert stopped.wait(2), 'lease loss must request stop'
    assert channel.recv(1) == b''
    channel.close()
    assert not __import__('pathlib').Path(config['destination_file']).exists()
    with pytest.raises(OSError):
        api.open_session_channel('127.0.0.1', server.input_port,
                                 snapshot['lease_id'], snapshot['ticket'], 'input')
    successor = client().connect()
    assert successor['lease_id'] != snapshot['lease_id']
    assert successor['ticket'] != snapshot['ticket']


def test_input_disconnect_revokes_live_lease_even_with_control_connected(runtime):
    server, client, _, stopped, api = runtime
    owner = client()
    before = owner.connect()
    channel = owner.open_channel('input')
    channel.close()
    assert stopped.wait(.25), 'input loss must stop without waiting for lease expiry'
    with pytest.raises(OSError):
        owner.heartbeat()
    with pytest.raises(OSError):
        api.open_session_channel('127.0.0.1', server.input_port,
                                 before['lease_id'], before['ticket'], 'input')
    assert client().connect()['lease_id'] != before['lease_id']


@pytest.mark.parametrize('changes', [dict(chassis_mode='ARMED'), dict(authority_mode='TELEOP'),
    dict(wheels_stopped=False), dict(field_age_s=dict(authority=.6, safety=0, wheels=0))])
def test_new_input_cannot_inherit_unconfirmed_prior_drive(runtime, changes):
    server, client, _, stopped, _ = runtime
    server.state_provider = lambda: safe_admission_state(**changes)
    owner = client()
    owner.connect()
    with pytest.raises(OSError):
        owner.open_channel('input')
    assert stopped.is_set()
    with owner.open_channel('ops') as diagnostics:
        diagnostics.settimeout(1)
        diagnostics.sendall(b'can still diagnose\n')
        assert diagnostics.recv(100) == b'can still diagnose\n'
    server.state_provider = safe_admission_state
    with owner.open_channel('input'):
        pass


@pytest.mark.parametrize('authority', ['IDLE', 'MOTION_HOLD'])
def test_stationary_estop_input_retries_leave_recovery_ops_available(runtime, authority):
    server, client, _, stopped, _ = runtime
    server.state_provider = lambda: safe_admission_state(
        chassis_mode='ESTOP', estop_latched=True, authority_mode=authority)
    owner = client()
    before = owner.connect()
    with owner.open_channel('ops') as diagnostics:
        diagnostics.settimeout(1)
        for _ in range(3):
            with pytest.raises(OSError):
                owner.open_channel('input')
            assert not stopped.is_set(), 'confirmed ESTOP retries must not starve recovery with stop mutations'
            assert owner.heartbeat()['lease_id'] == before['lease_id']
            diagnostics.sendall(b'recovery remains available\n')
            assert diagnostics.recv(100) == b'recovery remains available\n'
    assert owner.request('stop')['status'] == 'SUCCEEDED'
    assert stopped.is_set(), 'explicit stop must still execute in confirmed ESTOP'


@pytest.mark.parametrize('changes,pending', [
    (dict(chassis_mode='ARMED'), False),
    (dict(wheels_stopped=False), False),
    (dict(estop_latched=False), False),
    (dict(authority_mode='TELEOP'), False),
    (dict(field_age_s=dict(authority=.6, safety=0, wheels=0)), False),
    (dict(field_age_s=dict(authority=0, safety=float('nan'), wheels=0)), False),
    (dict(field_age_s=dict(authority=0, safety=0, wheels=True)), False),
    (dict(field_age_s={}), False),
    ({}, True),
])
def test_estop_retry_exception_does_not_survive_unconfirmed_state(runtime, changes, pending):
    server, client, _, stopped, _ = runtime
    state = dict(chassis_mode='ESTOP', estop_latched=True)
    server.state_provider = lambda: safe_admission_state(**state)
    owner = client()
    owner.connect()
    with pytest.raises(OSError):
        owner.open_channel('input')
    assert not stopped.is_set()
    state.update(changes)
    snapshot = safe_admission_state(**state)
    if pending:
        snapshot['status'] = 'PENDING'
    server.state_provider = lambda: snapshot
    with pytest.raises(OSError):
        owner.open_channel('input')
    assert stopped.is_set(), 'unsafe or uncertain state must request stop on the same lease'


def test_heartbeat_refreshes_lease_and_snapshot(runtime):
    _, client, config, stopped, _ = runtime
    owner = client()
    first = owner.connect()
    for _ in range(4):
        time.sleep(.15)
        response = owner.heartbeat()
        assert response['lease_id'] == first['lease_id']
        assert owner.snapshot['drive']['status'] == 'IDLE'
    assert not stopped.is_set()
    record = json.loads(__import__('pathlib').Path(config['destination_file']).read_text())
    assert record['host'] == '127.0.0.1'
    assert record['expires_at'] > time.monotonic()
    assert 'ticket' not in record and 'token' not in record


def test_disconnect_requests_stop_and_allows_reconnect(runtime):
    _, client, _, stopped, _ = runtime
    owner = client()
    owner.connect()
    owner.close()
    assert stopped.wait(1)
    assert client().connect()['connected']
    assert owner.snapshot['connected'] is False


def test_host_candidates_and_reconnect_new_identity(runtime):
    _, client, _, _, _ = runtime
    owner = client(hosts=['127.0.0.2', 'localhost'])
    first = owner.connect()
    assert owner.host == '127.0.0.1'
    owner.close()
    second = owner.connect()
    assert first['lease_id'] != second['lease_id']


def test_hold_start_cancel_and_errors(runtime):
    server, client, _, _, _ = runtime
    owner = client()
    owner.connect()
    assert owner.request('start')['status'] == 'REJECTED'
    assert owner.request('start_begin')['status'] == 'PENDING'
    assert owner.request('start')['status'] == 'REJECTED'
    owner.request('start_cancel')
    assert owner.request('start')['status'] == 'REJECTED'
    server.on_stop = lambda: (_ for _ in ()).throw(RuntimeError('secret must not leak'))
    result = owner.request('stop')
    assert result['status'] == 'REJECTED'
    assert 'secret' not in json.dumps(result)


def test_shutdown_closes_proxy_and_reports_stop_failure(runtime):
    server, client, _, _, _ = runtime
    owner = client()
    owner.connect()
    channel = owner.open_channel('ops')
    channel.settimeout(1)
    server.on_stop = lambda: {'status': 'REJECTED', 'reason': 'not stopped'}
    result = server.close()
    assert result['status'] == 'REJECTED'
    assert channel.recv(1) == b''
    channel.close()


def test_destination_fail_closed_and_legacy(tmp_path, monkeypatch):
    try:
        from powertrain_runtime.destination import resolve_destination
    except ImportError:
        pytest.fail('dynamic session destination is not implemented')
    monkeypatch.delenv('POWERTRAIN_OPERATOR_SESSION_FILE', raising=False)
    assert resolve_destination('legacy', 5006) == ('legacy', 5006)
    path = tmp_path / 'session.json'
    monkeypatch.setenv('POWERTRAIN_OPERATOR_SESSION_FILE', str(path))
    assert resolve_destination('legacy', 5006) is None
    for record in ({}, {'host': '127.0.0.1', 'expires_at': 10},
                   {'host': '127.0.0.1', 'lease_id': 'lease', 'expires_at': float('nan')},
                   {'host': 'bad host', 'lease_id': 'lease', 'expires_at': 20}):
        path.write_text(json.dumps(record))
        assert resolve_destination('legacy', 5006, now=10) is None
    path.write_text(json.dumps(dict(host='127.0.0.2', lease_id='lease', expires_at=20)))
    assert resolve_destination('legacy', 5006, now=10) == ('127.0.0.2', 5006)
    assert resolve_destination('legacy', 5006, now=20) is None


def test_config_defaults_expands_paths_and_rejects_invalid(tmp_path):
    try:
        from powertrain_runtime.config import load_config
    except ImportError:
        pytest.fail('runtime config loader is not implemented')
    path = tmp_path / 'operator.json'
    path.write_text(json.dumps(dict(robot_id='paired', hosts=['localhost'], token_file='~/token')))
    config = load_config(path)
    assert config['session_port'] == 9002
    assert config['token_file'].startswith('/')
    path.write_text(json.dumps(dict(robot_id='paired', hosts=[], token_file='~/token')))
    with pytest.raises(ValueError):
        load_config(path)


def test_async_stop_blocks_next_lease_until_terminal(runtime):
    server, client, _, _, _ = runtime
    state = {'status': 'PENDING', 'action': 'stop'}
    server.on_stop = lambda: dict(state)
    server.state_provider = lambda: dict(state)
    owner = client()
    owner.connect()
    assert owner.close()['status'] == 'PENDING'
    with pytest.raises(OSError):
        client().connect()
    state.update(status='REJECTED', detail='needs recovery')
    assert client().connect()['connected'] is True


def test_hold_can_complete_once_with_heartbeats(runtime):
    _, client, _, _, _ = runtime
    owner = client()
    owner.connect()
    owner.request('start_begin')
    for _ in range(11):
        time.sleep(.15)
        owner.heartbeat()
    assert owner.request('start')['status'] == 'SUCCEEDED'
    assert owner.request('start')['status'] == 'REJECTED'


def test_nonheartbeat_traffic_does_not_extend_lease(runtime):
    _, client, _, stopped, _ = runtime
    owner = client()
    owner.connect()
    began = time.monotonic()
    while time.monotonic() - began < .7:
        try:
            owner.request('start_cancel')
        except OSError:
            break
        time.sleep(.04)
    assert stopped.is_set()
    assert time.monotonic() - began < .65


def test_stalled_handshake_has_total_deadline(runtime):
    server, _, _, _, _ = runtime
    sock = socket.create_connection(('127.0.0.1', server.session_port), timeout=2)
    started = time.monotonic()
    failed = False
    for _ in range(16):
        try:
            sock.sendall(b' ')
        except OSError:
            failed = True
            break
        time.sleep(.15)
    if not failed:
        try:
            sock.settimeout(.1)
            data = sock.recv(1024)
            failed = not data or b'REJECTED' in data
        except socket.timeout:
            pass
    sock.close()
    assert failed, 'slow peers must not retain one bounded worker indefinitely'
    assert time.monotonic() - started < 2.4


def test_unexpected_control_eof_requests_stop(runtime):
    _, client, _, stopped, _ = runtime
    owner = client()
    owner.connect()
    # Actual lost transport, without the graceful release request.
    owner._socket.shutdown(socket.SHUT_RDWR)
    owner._socket.close()
    assert stopped.wait(1)
    assert client().connect()['connected']


def test_changed_peer_ip_updates_real_udp_destination(runtime):
    server, client, config, _, _ = runtime
    from powertrain_runtime.destination import resolve_destination
    from powertrain_runtime.session import _proof, _read, _send, _token
    import secrets
    destination = config['destination_file']
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as first_receiver, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as second_receiver, \
            socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        first_receiver.bind(('127.0.0.1', 0))
        port = first_receiver.getsockname()[1]
        second_receiver.bind(('127.0.0.2', port))
        first_receiver.settimeout(.5)
        second_receiver.settimeout(.5)
        owner = client()
        owner.connect()
        sender.sendto(b'first', resolve_destination('legacy', port, path=destination))
        assert first_receiver.recv(32) == b'first'
        owner.close()
        assert resolve_destination('legacy', port, path=destination) is None
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as moved:
            moved.settimeout(.5)
            moved.bind(('127.0.0.2', 0))
            moved.connect(('127.0.0.1', server.session_port))
            nonce = secrets.token_hex(32)
            _send(moved, dict(robot_id='paired-robot', nonce=nonce))
            challenge = _read(moved)
            proof = _proof(_token(config['token_file']), 'client', 'paired-robot',
                           challenge['boot_id'], nonce, challenge['nonce'])
            _send(moved, dict(proof=proof))
            assert _read(moved)['connected']
            sender.sendto(b'moved', resolve_destination('legacy', port, path=destination))
            assert second_receiver.recv(32) == b'moved'
            _send(moved, dict(op='release'))
            assert _read(moved)['connected'] is False


def test_unauthenticated_peer_cannot_forward_to_inner_service(runtime):
    server, _, _, _, _ = runtime
    for port in (server.input_port, server.ops_port):
        with socket.create_connection(('127.0.0.1', port), timeout=.5) as sock:
            sock.sendall(b'{"v":1,"omega":0}\n')
            data = sock.recv(1024)
            assert b'REJECTED' in data
            assert b'"v":1' not in data
