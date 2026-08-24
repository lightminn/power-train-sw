import io
import importlib.util
import json
import os
from pathlib import Path
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]
VIEWER_PATH = ROOT / "scripts/pdist80b_view.py"
VALID_RESPONSE = bytes.fromhex(
    "AC BA 01 EE 0C E7 01 85 FF 51 00 00 00 2D 00 00 00 B5"
)
PID_238_REQUEST = bytes.fromhex("BA AC 01 04 01 EE A6")
_VIEWER = None


def _viewer():
    global _VIEWER
    assert VIEWER_PATH.exists(), "scripts/pdist80b_view.py has not been implemented"
    if _VIEWER is None:
        spec = importlib.util.spec_from_file_location("pdist80b_view_under_test", VIEWER_PATH)
        module = importlib.util.module_from_spec(spec)
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        _VIEWER = module
    return _VIEWER


class FakeSerial:
    def __init__(self, response=VALID_RESPONSE, read_error=None):
        self.response = response
        self.read_error = read_error
        self.writes = []
        self.closed = False
        self.input_resets = 0
        self.flushes = 0

    def reset_input_buffer(self):
        self.input_resets += 1

    def write(self, packet):
        self.writes.append(packet)
        return len(packet)

    def flush(self):
        self.flushes += 1

    def read(self, size):
        assert size == 18
        if self.read_error is not None:
            raise self.read_error
        return self.response

    def close(self):
        self.closed = True


class FakeDatagramSocket:
    def __init__(self, datagrams):
        self.datagrams = list(datagrams)
        self.options = []
        self.bound_to = None
        self.timeout_s = None
        self.closed = False

    def setsockopt(self, level, option, value):
        self.options.append((level, option, value))

    def bind(self, address):
        self.bound_to = address

    def getsockname(self):
        return ("0.0.0.0", 50444)

    def settimeout(self, timeout_s):
        self.timeout_s = timeout_s

    def recvfrom(self, size):
        assert size == 65535
        return self.datagrams.pop(0), ("127.0.0.1", 5004)

    def close(self):
        self.closed = True


class TtyStringIO(io.StringIO):
    def isatty(self):
        return True


def _normal_sample(viewer, *, local_time="12:34:56"):
    return viewer.sample_from_values(
        local_time=local_time,
        voltage_v=48.7,
        current_a=-12.3,
        charge_current_a=4.5,
        soc_percent=81,
        battery_flags=0,
        protection_flags=0,
        rs485_state="LIVE",
        consecutive_failures=0,
        detail="",
        source="serial",
    )


def test_panel_formats_exactly_seven_fixed_position_lines_with_hex_only_normal_flags():
    viewer = _viewer()
    sample = viewer.sample_from_values(
        local_time="05:07:25",
        voltage_v=44.1,
        current_a=0.0,
        charge_current_a=1.8,
        soc_percent=56,
        battery_flags=0x02,
        protection_flags=0x20,
        rs485_state="LIVE",
        consecutive_failures=0,
        detail="",
        source="serial",
    )

    lines = viewer.format_panel(sample, columns=80)

    assert lines == [
        "  PDIST80B  05:07:25                 RS485 LIVE",
        "    전압    44.1 V      SOC   56 %",
        "    방전     0.0 A        0 W",
        "    충전     1.8 A       79 W",
        "    상태    정상 · 충전 중",
        "    플래그  batt 0x02  prot 0x20",
        "",
    ]


def test_panel_decodes_only_fault_bits_on_the_offending_flag_byte():
    viewer = _viewer()
    sample = viewer.sample_from_values(
        local_time="12:34:58",
        voltage_v=51.2,
        current_a=3.0,
        charge_current_a=0.0,
        soc_percent=90,
        battery_flags=0x04,
        protection_flags=0x00,
        rs485_state="LIVE",
        consecutive_failures=0,
        detail="",
        source="serial",
    )

    lines = viewer.format_panel(sample, columns=80)

    assert lines[4] == "    상태    ⚠ 과전압 보호"
    assert lines[5] == "    플래그  batt 0x04(과전압 보호)  prot 0x00"


def test_panel_error_uses_na_measurements_and_failure_detail_line():
    viewer = _viewer()
    sample = viewer.error_sample(
        "could not open port /dev/ttyUSB0",
        consecutive_failures=12,
        local_time="12:34:58",
    )

    lines = viewer.format_panel(sample, columns=80)

    assert lines[0].endswith("RS485 ERROR · 실패 12")
    assert lines[1] == "    전압     N/A V      SOC  N/A %"
    assert lines[2] == "    방전     N/A A      N/A W"
    assert lines[3] == "    충전     N/A A      N/A W"
    assert lines[4] == "    상태    상태 미수신"
    assert lines[5] == "    플래그  batt N/A  prot N/A"
    assert lines[6] == "    실패 12 · could not open port /dev/ttyUSB0"


def test_panel_clips_every_line_by_display_width_without_embedded_newlines():
    viewer = _viewer()
    sample = viewer.error_sample(
        "연결 실패: 장치를 열 수 없습니다 /dev/ttyUSB0",
        consecutive_failures=123,
        local_time="12:34:58",
    )

    lines = viewer.format_panel(sample, columns=40)

    assert len(lines) == 7
    assert all(viewer._display_width(line) <= 40 for line in lines)
    assert all("\n" not in line and "\r" not in line for line in lines)


def test_non_tty_writer_keeps_the_existing_single_plain_line():
    viewer = _viewer()
    stream = io.StringIO()
    writer = viewer.OutputWriter(stream, json_mode=False)
    sample = _normal_sample(viewer)

    writer.write(sample)
    writer.close()

    assert stream.getvalue() == viewer.format_human(sample) + "\n"


def test_live_tty_writer_redraws_seven_lines_and_restores_cursor(monkeypatch):
    viewer = _viewer()
    widths = iter((80, 40))
    monkeypatch.setattr(
        viewer.shutil,
        "get_terminal_size",
        lambda: os.terminal_size((next(widths), 24)),
    )
    stream = TtyStringIO()
    writer = viewer.OutputWriter(stream, json_mode=False)

    writer.write(_normal_sample(viewer, local_time="12:34:56"))
    first_frame = stream.getvalue()
    writer.write(_normal_sample(viewer, local_time="12:34:57"))
    writer.close()
    output = stream.getvalue()

    assert first_frame.startswith("\x1b[?25l")
    assert "\x1b[7A" not in first_frame
    assert output[len(first_frame):].startswith("\x1b[7A")
    assert output.count("\x1b[K") == 7
    assert output.endswith("\x1b[?25h")


def test_once_tty_writer_prints_panel_without_cursor_control(monkeypatch):
    viewer = _viewer()
    monkeypatch.setattr(
        viewer.shutil,
        "get_terminal_size",
        lambda: os.terminal_size((80, 24)),
    )
    stream = TtyStringIO()
    writer = viewer.OutputWriter(stream, json_mode=False, live_panel=False)

    writer.write(_normal_sample(viewer))
    writer.close()

    assert "\x1b" not in stream.getvalue()
    assert stream.getvalue().count("\n") == 7


def test_keyboard_interrupt_path_restores_tty_cursor(monkeypatch):
    viewer = _viewer()
    monkeypatch.setattr(
        viewer.shutil,
        "get_terminal_size",
        lambda: os.terminal_size((80, 24)),
    )

    class InterruptingPoller:
        def __init__(self):
            self.calls = 0
            self.closed = False

        def poll(self):
            self.calls += 1
            if self.calls > 1:
                raise KeyboardInterrupt
            return _normal_sample(viewer)

        def close(self):
            self.closed = True

    stream = TtyStringIO()
    writer = viewer.OutputWriter(stream, json_mode=False)
    poller = InterruptingPoller()

    with pytest.raises(KeyboardInterrupt):
        viewer.run_serial(
            poller,
            writer,
            hz=2.0,
            once=False,
            timeout_s=None,
            monotonic=lambda: 0.0,
            sleep=lambda _delay: None,
        )

    assert poller.closed is True
    assert stream.getvalue().endswith("\x1b[?25h")


def test_formats_normal_sample_with_all_operator_fields():
    viewer = _viewer()

    line = viewer.format_human(_normal_sample(viewer))

    assert line == (
        "12:34:56 | 전압 48.7 V | 방전 -12.3 A | 충전 4.5 A | "
        "방전전력 -599.0 W | 충전전력 219.2 W | SOC 81% | batt 0x00 | "
        "prot 0x00 | 정상 | "
        "RS485 LIVE | 실패 0"
    )


def test_formats_live_charger_status_bits_as_decoded_normal_state():
    viewer = _viewer()
    sample = viewer.sample_from_values(
        local_time="12:34:57",
        voltage_v=43.8,
        current_a=0.0,
        charge_current_a=2.0,
        soc_percent=54,
        battery_flags=0x02,
        protection_flags=0x20,
        rs485_state="LIVE",
        consecutive_failures=0,
        detail="",
        source="serial",
    )

    line = viewer.format_human(sample)

    assert (
        "batt 0x02(자동 충전기 결합) | prot 0x20(외부 제어) | 정상"
        in line
    )


def test_formats_all_set_flag_meanings_and_warns_only_for_fault_masks():
    viewer = _viewer()
    sample = viewer.sample_from_values(
        local_time="12:34:58",
        voltage_v=51.2,
        current_a=3.0,
        charge_current_a=0.0,
        soc_percent=90,
        battery_flags=0xA5,
        protection_flags=0x5A,
        rs485_state="LIVE",
        consecutive_failures=0,
        detail="",
        source="serial",
    )

    line = viewer.format_human(sample)

    assert (
        "batt 0xA5(수동 충전기 결합, 과전압 보호, 충전 저온 보호, 방전 저온 보호)"
        in line
    )
    assert (
        "prot 0x5A(방전 과전류 보호, 단락 보호, 예약, 충전 플래그)"
        in line
    )
    assert "| ⚠ 보호 경고 |" in line


def test_formats_failed_sample_as_readable_error_row():
    viewer = _viewer()
    sample = viewer.error_sample(
        "PID 238 response must be 18 bytes",
        consecutive_failures=2,
        local_time="12:34:58",
    )

    line = viewer.format_human(sample)

    assert sample["power_w"] is None
    assert sample["charge_power_w"] is None
    assert line == (
        "12:34:58 | 전압 N/A | 방전 N/A | 충전 N/A | 방전전력 N/A | "
        "충전전력 N/A | SOC N/A | batt N/A | prot N/A | 상태 미수신 | "
        "RS485 ERROR | 실패 2 | detail: PID 238 response must be 18 bytes"
    )


def test_serial_poll_writes_only_the_pid_238_monitor_request():
    viewer = _viewer()
    fake_port = FakeSerial()
    poller = viewer.SerialPoller(
        "/dev/fake-pdist",
        1,
        serial_factory=lambda *_args, **_kwargs: fake_port,
    )

    sample = poller.poll(local_time="12:34:56")
    poller.close()

    assert sample["rs485_state"] == "LIVE"
    assert fake_port.writes == [PID_238_REQUEST]
    assert fake_port.input_resets == 1
    assert fake_port.flushes == 1
    assert sample["charge_power_w"] == pytest.approx(219.15)


def test_serial_poll_closes_and_reopens_after_failure():
    viewer = _viewer()
    first_port = FakeSerial(read_error=OSError("adapter offline"))
    second_port = FakeSerial()
    available_ports = iter((first_port, second_port))
    open_calls = []

    def serial_factory(*args, **kwargs):
        open_calls.append((args, kwargs))
        return next(available_ports)

    poller = viewer.SerialPoller("/dev/fake-pdist", 1, serial_factory=serial_factory)

    failed = poller.poll(local_time="12:34:56")
    recovered = poller.poll(local_time="12:34:57")
    poller.close()

    assert failed["rs485_state"] == "ERROR"
    assert failed["rs485_consecutive_failures"] == 1
    assert failed["rs485_detail"] == "adapter offline"
    assert first_port.closed is True
    assert len(open_calls) == 2
    assert recovered["rs485_state"] == "LIVE"
    assert recovered["rs485_consecutive_failures"] == 0
    assert second_port.writes == [PID_238_REQUEST]


def test_udp_ignores_malformed_datagram_then_renders_sender_payload():
    viewer = _viewer()
    payload = {
        "schema_version": 1,
        "pdist_soc_percent": 81,
        "pdist_battery_flags": 0,
        "pdist_protection_flags": 0,
        "pdist_charge_current_a": 4.5,
        "voltage_v": 48.7,
        "current_a": -12.3,
        "power_w": -599.01,
        "rs485_state": "LIVE",
        "rs485_consecutive_failures": 0,
        "rs485_detail": "",
    }
    fake_socket = FakeDatagramSocket(
        (b"{not-json", json.dumps(payload).encode("utf-8"))
    )
    receiver = viewer.UdpReceiver(0, socket_factory=lambda *_args: fake_socket)
    try:
        sample = receiver.receive(local_time="12:34:56")
    finally:
        receiver.close()

    assert fake_socket.options == [(viewer.socket.SOL_SOCKET, viewer.socket.SO_REUSEADDR, 1)]
    assert fake_socket.bound_to == ("", 0)
    assert sample["udp_ignored_datagrams"] == 1
    assert sample["charge_power_w"] == pytest.approx(219.15)
    assert viewer.format_human(sample) == (
        "12:34:56 | 전압 48.7 V | 방전 -12.3 A | 충전 4.5 A | "
        "방전전력 -599.0 W | 충전전력 219.2 W | SOC 81% | batt 0x00 | "
        "prot 0x00 | 정상 | RS485 LIVE | 실패 0 | UDP 무시 1"
    )


def test_udp_ignores_non_v1_payloads_while_counting_them():
    viewer = _viewer()
    valid = {
        "schema_version": 1,
        "pdist_soc_percent": None,
        "pdist_battery_flags": None,
        "pdist_protection_flags": None,
        "pdist_charge_current_a": None,
        "voltage_v": None,
        "current_a": None,
        "power_w": None,
        "rs485_state": "ERROR",
        "rs485_consecutive_failures": 3,
        "rs485_detail": "timeout",
    }
    fake_socket = FakeDatagramSocket(
        (
            json.dumps({"schema_version": 2}).encode(),
            json.dumps(valid).encode(),
        )
    )
    receiver = viewer.UdpReceiver(0, socket_factory=lambda *_args: fake_socket)
    try:
        sample = receiver.receive(local_time="12:34:56")
    finally:
        receiver.close()

    assert sample["udp_ignored_datagrams"] == 1
    assert sample["rs485_detail"] == "timeout"


def test_json_output_is_compact_flushed_jsonl_without_carriage_returns():
    viewer = _viewer()
    stream = io.StringIO()
    writer = viewer.OutputWriter(stream, json_mode=True)
    sample = _normal_sample(viewer)

    writer.write(sample)

    output = stream.getvalue()
    assert output.endswith("\n")
    assert "\r" not in output
    assert ": " not in output
    decoded = json.loads(output)
    assert decoded["schema_version"] == 1
    assert decoded["power_w"] == pytest.approx(-599.01)
    assert decoded["charge_power_w"] == pytest.approx(219.15)
    keys = list(decoded)
    assert keys[keys.index("power_w") + 1] == "charge_power_w"
    assert decoded["pdist_soc_percent"] == 81
    assert decoded["pdist_battery_flags"] == 0
    assert decoded["pdist_protection_flags"] == 0
    assert decoded["pdist_charge_current_a"] == 4.5
    assert decoded["verdict"] == "정상"
    assert "soc_percent" not in decoded


def test_once_returns_failure_when_serial_sample_cannot_be_read():
    viewer = _viewer()
    fake_port = FakeSerial(read_error=OSError("no response"))
    poller = viewer.SerialPoller(
        "/dev/fake-pdist",
        1,
        serial_factory=lambda *_args, **_kwargs: fake_port,
    )
    stream = io.StringIO()
    writer = viewer.OutputWriter(stream, json_mode=False)

    returncode = viewer.run_serial(
        poller,
        writer,
        hz=2.0,
        once=True,
        timeout_s=None,
    )

    assert returncode == 1
    assert "RS485 ERROR" in stream.getvalue()
