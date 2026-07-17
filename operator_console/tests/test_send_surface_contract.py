import ast
from pathlib import Path


CONSOLE_DIR = Path(__file__).resolve().parents[1]
FORBIDDEN_SEND_FILES = (
    "app.py",
    "telemetry.py",
    "udp_source.py",
    "metadata.py",
    "pipelines.py",
)


def _source(name):
    return (CONSOLE_DIR / name).read_text(encoding="utf-8")


def test_only_ops_client_may_expose_outbound_socket_send_calls():
    # SRT is described as a GStreamer pipeline string in pipelines.py; it is
    # not a Python outbound socket API. GTK signal ``widget.connect`` calls in
    # app.py are likewise not socket connections: their first argument is a
    # GTK/GStreamer signal name. Any other connect/send call is forbidden.
    for name in FORBIDDEN_SEND_FILES:
        source = _source(name)
        violations = []
        for node in ast.walk(ast.parse(source)):
            if not isinstance(node, ast.Call) or not isinstance(node.func, ast.Attribute):
                continue
            method = node.func.attr
            if method == "connect" and name == "app.py" and node.args:
                signal = node.args[0]
                if isinstance(signal, ast.Constant) and isinstance(signal.value, str):
                    continue
            if method in {"connect", "create_connection", "sendall", "sendto"}:
                violations.append((node.lineno, method))
        assert violations == [], (name, violations)


def test_banner_declares_rx_only_observation_and_token_gated_ops():
    source = _source("app.py")

    assert "READ-ONLY CONSOLE" not in source
    assert "OBSERVE: RX-ONLY  |  OPS: TOKEN-GATED  |  " in source


def test_app_consumes_confirm_flow_and_console_ops_client():
    source = _source("app.py")

    assert "ConfirmFlow" in source
    assert "ConsoleOpsClient" in source
    assert "ConfirmFlow(" in source
    assert "ConsoleOpsClient(" in source
