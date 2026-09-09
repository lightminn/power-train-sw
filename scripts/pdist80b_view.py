#!/usr/bin/env python3
"""View PDIST80B read-only battery measurements from serial or UDP."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import socket
import sys
import time
import unicodedata


sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2/src/powertrain_ros"))

from powertrain_ros.pdist80b import (  # noqa: E402
    BATTERY_FAULT_MASK,
    PROTECTION_FAULT_MASK,
    battery_flag_labels,
    bms_monitor_request,
    parse_bms_monitor_response,
    protection_flag_labels,
)


SERIAL_BAUDRATE = 57600
SERIAL_TIMEOUT_S = 0.25
UDP_RECEIVE_TIMEOUT_S = 0.25
_CALCULATE_POWER = object()
_UDP_FIELDS = {
    "pdist_soc_percent",
    "pdist_battery_flags",
    "pdist_protection_flags",
    "pdist_charge_current_a",
    "voltage_v",
    "current_a",
    "power_w",
    "rs485_state",
    "rs485_consecutive_failures",
    "rs485_detail",
}


def _local_time() -> str:
    return time.strftime("%H:%M:%S")


def health_verdict(battery_flags, protection_flags) -> str:
    """Return a fault-mask-based health verdict without GTK."""
    battery_faults = (battery_flags or 0) & BATTERY_FAULT_MASK
    protection_faults = (protection_flags or 0) & PROTECTION_FAULT_MASK
    if battery_faults or protection_faults:
        return "⚠ 보호 경고"
    if battery_flags is None or protection_flags is None:
        return "상태 미수신"
    return "정상"


def sample_from_values(
    *,
    local_time: str | None,
    voltage_v,
    current_a,
    charge_current_a,
    soc_percent,
    battery_flags,
    protection_flags,
    rs485_state: str,
    consecutive_failures: int,
    detail: str,
    source: str,
    power_w=_CALCULATE_POWER,
    udp_ignored_datagrams: int | None = None,
) -> dict[str, object]:
    """Build the common serial/UDP display record."""
    if power_w is _CALCULATE_POWER:
        power_w = (
            None
            if voltage_v is None or current_a is None
            else voltage_v * current_a
        )
    charge_power_w = (
        None
        if voltage_v is None or charge_current_a is None
        else voltage_v * charge_current_a
    )
    sample = {
        "local_time": local_time or _local_time(),
        "source": source,
        "voltage_v": voltage_v,
        "current_a": current_a,
        "charge_current_a": charge_current_a,
        "power_w": power_w,
        "charge_power_w": charge_power_w,
        "soc_percent": soc_percent,
        "battery_flags": battery_flags,
        "protection_flags": protection_flags,
        "verdict": health_verdict(battery_flags, protection_flags),
        "rs485_state": rs485_state,
        "rs485_consecutive_failures": consecutive_failures,
        "rs485_detail": detail,
    }
    if udp_ignored_datagrams is not None:
        sample["udp_ignored_datagrams"] = udp_ignored_datagrams
    return sample


def error_sample(
    detail: str,
    *,
    consecutive_failures: int,
    local_time: str | None = None,
) -> dict[str, object]:
    """Build a visible serial ERROR sample with unknown measurements."""
    return sample_from_values(
        local_time=local_time,
        voltage_v=None,
        current_a=None,
        charge_current_a=None,
        soc_percent=None,
        battery_flags=None,
        protection_flags=None,
        rs485_state="ERROR",
        consecutive_failures=consecutive_failures,
        detail=detail,
        source="serial",
    )


def _format_measurement(value, unit: str) -> str:
    return "N/A" if value is None else f"{value:.1f} {unit}"


def _format_percent(value) -> str:
    return "N/A" if value is None else f"{value}%"


def _format_flags(value, labeler) -> str:
    if value is None:
        return "N/A"
    labels = labeler(value)
    meaning = "" if not labels else f"({', '.join(labels)})"
    return f"0x{value:02X}{meaning}"


def format_human(sample: dict[str, object]) -> str:
    """Format one complete operator-facing status row."""
    parts = [
        str(sample["local_time"]),
        f"전압 {_format_measurement(sample['voltage_v'], 'V')}",
        f"방전 {_format_measurement(sample['current_a'], 'A')}",
        f"충전 {_format_measurement(sample['charge_current_a'], 'A')}",
        f"방전전력 {_format_measurement(sample['power_w'], 'W')}",
        f"충전전력 {_format_measurement(sample['charge_power_w'], 'W')}",
        f"SOC {_format_percent(sample['soc_percent'])}",
        f"batt {_format_flags(sample['battery_flags'], battery_flag_labels)}",
        f"prot {_format_flags(sample['protection_flags'], protection_flag_labels)}",
        str(sample["verdict"]),
        f"RS485 {sample['rs485_state']}",
        f"실패 {sample['rs485_consecutive_failures']}",
    ]
    detail = sample.get("rs485_detail")
    if detail:
        parts.append(f"detail: {detail}")
    if "udp_ignored_datagrams" in sample:
        parts.append(f"UDP 무시 {sample['udp_ignored_datagrams']}")
    return " | ".join(parts)


def _display_width(text: str) -> int:
    width = 0
    for character in text:
        if unicodedata.combining(character):
            continue
        width += 2 if unicodedata.east_asian_width(character) in ("F", "W") else 1
    return width


def _clip_display(text: str, width: int) -> str:
    """Clip text without exceeding an east-Asian-aware display width."""
    if width <= 0:
        return ""
    clipped: list[str] = []
    used_width = 0
    for character in text:
        character_width = _display_width(character)
        if used_width + character_width > width:
            break
        clipped.append(character)
        used_width += character_width
    return "".join(clipped)


def _align_display(text: str, width: int, *, right: bool = False) -> str:
    """Fit text to a display-width field and pad it on the requested side."""
    clipped = _clip_display(text, width)
    padding = " " * max(0, width - _display_width(clipped))
    return padding + clipped if right else clipped + padding


def _panel_decimal(value) -> str:
    return "N/A" if value is None else f"{value:.1f}"


def _panel_integer(value) -> str:
    return "N/A" if value is None else f"{value:.0f}"


def _panel_fault_reasons(sample: dict[str, object]) -> tuple[str, ...]:
    battery_flags = sample["battery_flags"]
    protection_flags = sample["protection_flags"]
    if battery_flags is None or protection_flags is None:
        return ()
    labels = (
        battery_flag_labels(battery_flags & BATTERY_FAULT_MASK)
        + protection_flag_labels(protection_flags & PROTECTION_FAULT_MASK)
    )
    reasons: list[str] = []
    for label in labels:
        if label not in reasons:
            reasons.append(label)
    return tuple(reasons)


def _panel_verdict(sample: dict[str, object]) -> str:
    reasons = _panel_fault_reasons(sample)
    if reasons:
        return f"⚠ {', '.join(reasons)}"
    if sample["battery_flags"] is None or sample["protection_flags"] is None:
        return str(sample["verdict"])
    charge_current_a = sample["charge_current_a"]
    if charge_current_a is not None and charge_current_a > 0.0:
        return "정상 · 충전 중"
    return str(sample["verdict"])


def _panel_flags(value, fault_mask: int, labeler) -> str:
    if value is None:
        return "N/A"
    fault_flags = value & fault_mask
    labels = labeler(fault_flags)
    meaning = "" if not labels else f"({', '.join(labels)})"
    return f"0x{value:02X}{meaning}"


def format_panel(sample: dict[str, object], columns: int) -> list[str]:
    """Return the fixed seven-line TTY panel clipped to terminal width."""
    label_width = 8
    failures = sample["rs485_consecutive_failures"]
    rs485 = f"RS485 {sample['rs485_state']}"
    if failures:
        rs485 += f" · 실패 {failures}"

    header_left = f"PDIST80B  {sample['local_time']}"
    voltage = _panel_decimal(sample["voltage_v"])
    discharge_current = _panel_decimal(sample["current_a"])
    charge_current = _panel_decimal(sample["charge_current_a"])
    discharge_power = _panel_integer(sample["power_w"])
    charge_power = _panel_integer(sample["charge_power_w"])
    soc = "N/A" if sample["soc_percent"] is None else str(sample["soc_percent"])
    battery_flags = _panel_flags(
        sample["battery_flags"],
        BATTERY_FAULT_MASK,
        battery_flag_labels,
    )
    protection_flags = _panel_flags(
        sample["protection_flags"],
        PROTECTION_FAULT_MASK,
        protection_flag_labels,
    )

    detail = ""
    if sample["rs485_state"] != "LIVE":
        detail = f"    실패 {failures}"
        if sample["rs485_detail"]:
            detail += f" · {sample['rs485_detail']}"

    lines = [
        f"  {_align_display(header_left, 35)}{rs485}",
        (
            f"    {_align_display('전압', label_width)}"
            f"{_align_display(voltage, 4, right=True)} V      SOC"
            f"{_align_display(soc, 5, right=True)} %"
        ),
        (
            f"    {_align_display('방전', label_width)}"
            f"{_align_display(discharge_current, 4, right=True)} A"
            f"{_align_display(discharge_power, 9, right=True)} W"
        ),
        (
            f"    {_align_display('충전', label_width)}"
            f"{_align_display(charge_current, 4, right=True)} A"
            f"{_align_display(charge_power, 9, right=True)} W"
        ),
        f"    {_align_display('상태', label_width)}{_panel_verdict(sample)}",
        (
            f"    {_align_display('플래그', label_width)}"
            f"batt {battery_flags}  prot {protection_flags}"
        ),
        detail,
    ]
    return [_clip_display(line, columns) for line in lines]


def _json_sample(sample: dict[str, object]) -> dict[str, object]:
    record = {
        "schema_version": 1,
        "local_time": sample["local_time"],
        "source": sample["source"],
        "voltage_v": sample["voltage_v"],
        "current_a": sample["current_a"],
        "power_w": sample["power_w"],
        "charge_power_w": sample["charge_power_w"],
        "pdist_soc_percent": sample["soc_percent"],
        "pdist_battery_flags": sample["battery_flags"],
        "pdist_protection_flags": sample["protection_flags"],
        "pdist_charge_current_a": sample["charge_current_a"],
        "rs485_state": sample["rs485_state"],
        "rs485_consecutive_failures": sample["rs485_consecutive_failures"],
        "rs485_detail": sample["rs485_detail"],
        "verdict": sample["verdict"],
    }
    if "udp_ignored_datagrams" in sample:
        record["udp_ignored_datagrams"] = sample["udp_ignored_datagrams"]
    return record


class OutputWriter:
    """Render JSONL, terminal-live, or plain line-oriented output."""

    def __init__(self, stream, *, json_mode: bool, live_panel: bool = True):
        self._stream = stream
        self._json_mode = json_mode
        self._tty = not json_mode and bool(stream.isatty())
        self._live_panel = live_panel
        self._panel_drawn = False
        self._cursor_hidden = False

    def write(self, sample: dict[str, object]) -> None:
        if self._json_mode:
            line = json.dumps(
                _json_sample(sample),
                ensure_ascii=False,
                separators=(",", ":"),
            )
            self._stream.write(line + "\n")
        else:
            line = format_human(sample)
            if self._tty:
                columns = shutil.get_terminal_size().columns
                lines = format_panel(sample, columns=columns)
                if self._live_panel:
                    if not self._cursor_hidden:
                        self._cursor_hidden = True
                        self._stream.write("\x1b[?25l")
                    if self._panel_drawn:
                        self._stream.write("\x1b[7A")
                        for panel_line in lines:
                            self._stream.write(f"\r{panel_line}\x1b[K\n")
                    else:
                        for panel_line in lines:
                            self._stream.write(panel_line + "\n")
                        self._panel_drawn = True
                else:
                    for panel_line in lines:
                        self._stream.write(panel_line + "\n")
            else:
                self._stream.write(line + "\n")
        self._stream.flush()

    def close(self) -> None:
        if self._cursor_hidden:
            self._stream.write("\x1b[?25h")
            self._stream.flush()
            self._cursor_hidden = False


class SerialPoller:
    """Own one serial port and reopen it after a failed read."""

    def __init__(self, port_path: str, device_id: int, *, serial_factory=None):
        self._port_path = port_path
        self._device_id = device_id
        self._serial_factory = serial_factory
        self._port = None
        self._consecutive_failures = 0

    def _open(self):
        if self._serial_factory is None:
            try:
                import serial
            except ImportError as exc:
                raise OSError(
                    "pyserial is required for --source serial; use the conda base Python"
                ) from exc
            self._serial_factory = serial.Serial
        return self._serial_factory(
            self._port_path,
            SERIAL_BAUDRATE,
            bytesize=8,
            parity="N",
            stopbits=1,
            timeout=SERIAL_TIMEOUT_S,
            write_timeout=SERIAL_TIMEOUT_S,
        )

    def _close_port(self) -> None:
        port, self._port = self._port, None
        if port is None:
            return
        try:
            port.close()
        except (OSError, ValueError):
            pass

    def poll(self, *, local_time: str | None = None) -> dict[str, object]:
        try:
            if self._port is None:
                self._port = self._open()
            self._port.reset_input_buffer()
            self._port.write(bms_monitor_request(self._device_id))
            self._port.flush()
            status = parse_bms_monitor_response(
                self._port.read(18),
                self._device_id,
            )
        except (OSError, ValueError) as exc:
            self._consecutive_failures += 1
            self._close_port()
            return error_sample(
                str(exc),
                consecutive_failures=self._consecutive_failures,
                local_time=local_time,
            )

        self._consecutive_failures = 0
        return sample_from_values(
            local_time=local_time,
            voltage_v=status.voltage_v,
            current_a=status.discharge_current_a,
            charge_current_a=status.charge_current_a,
            soc_percent=status.soc_percent,
            battery_flags=status.battery_flags,
            protection_flags=status.protection_flags,
            rs485_state="LIVE",
            consecutive_failures=0,
            detail="",
            source="serial",
        )

    def close(self) -> None:
        self._close_port()


def _optional_number(value) -> bool:
    return value is None or (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _optional_int(value) -> bool:
    return value is None or (isinstance(value, int) and not isinstance(value, bool))


def _decode_udp_payload(data: bytes) -> dict[str, object] | None:
    try:
        payload = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        return None
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        return None
    if not _UDP_FIELDS.issubset(payload):
        return None
    if not all(
        _optional_number(payload[key])
        for key in ("voltage_v", "current_a", "power_w", "pdist_charge_current_a")
    ):
        return None
    if not all(
        _optional_int(payload[key])
        for key in ("pdist_soc_percent", "pdist_battery_flags", "pdist_protection_flags")
    ):
        return None
    failures = payload["rs485_consecutive_failures"]
    if not isinstance(failures, int) or isinstance(failures, bool) or failures < 0:
        return None
    if not isinstance(payload["rs485_state"], str):
        return None
    if not isinstance(payload["rs485_detail"], str):
        return None
    return payload


class UdpReceiver:
    """Receive schema-version-1 PDIST80B sender datagrams."""

    def __init__(self, listen_port: int, *, socket_factory=socket.socket):
        self._socket = socket_factory(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("", listen_port))
        self.port = self._socket.getsockname()[1]
        self._ignored_datagrams = 0

    def receive(
        self,
        *,
        local_time: str | None = None,
        timeout_s: float | None = None,
    ) -> dict[str, object]:
        self._socket.settimeout(timeout_s)
        while True:
            data, _address = self._socket.recvfrom(65535)
            payload = _decode_udp_payload(data)
            if payload is None:
                self._ignored_datagrams += 1
                continue
            return sample_from_values(
                local_time=local_time,
                voltage_v=payload["voltage_v"],
                current_a=payload["current_a"],
                charge_current_a=payload["pdist_charge_current_a"],
                soc_percent=payload["pdist_soc_percent"],
                battery_flags=payload["pdist_battery_flags"],
                protection_flags=payload["pdist_protection_flags"],
                rs485_state=payload["rs485_state"],
                consecutive_failures=payload["rs485_consecutive_failures"],
                detail=payload["rs485_detail"],
                source="udp",
                power_w=payload["power_w"],
                udp_ignored_datagrams=self._ignored_datagrams,
            )

    def close(self) -> None:
        self._socket.close()


def _remaining_s(deadline: float | None, monotonic) -> float | None:
    return None if deadline is None else max(0.0, deadline - monotonic())


def _sample_succeeded(sample: dict[str, object]) -> bool:
    return sample["rs485_state"] == "LIVE"


def run_serial(
    poller: SerialPoller,
    writer: OutputWriter,
    *,
    hz: float,
    once: bool,
    timeout_s: float | None,
    monotonic=time.monotonic,
    sleep=time.sleep,
) -> int:
    """Run the serial polling loop and return the requested process status."""
    deadline = None if timeout_s is None else monotonic() + timeout_s
    sampled = False
    try:
        while not sampled or deadline is None or monotonic() < deadline:
            tick_started = monotonic()
            sample = poller.poll()
            sampled = True
            writer.write(sample)
            if once:
                return 0 if _sample_succeeded(sample) else 1
            delay = max(0.0, 1.0 / hz - (monotonic() - tick_started))
            remaining = _remaining_s(deadline, monotonic)
            if remaining is not None:
                delay = min(delay, remaining)
            if delay > 0.0:
                sleep(delay)
        return 0
    finally:
        poller.close()
        writer.close()


def run_udp(
    receiver: UdpReceiver,
    writer: OutputWriter,
    *,
    once: bool,
    timeout_s: float | None,
    monotonic=time.monotonic,
) -> int:
    """Run the UDP receive loop and return the requested process status."""
    deadline = None if timeout_s is None else monotonic() + timeout_s
    try:
        while True:
            remaining = _remaining_s(deadline, monotonic)
            if remaining is not None and remaining <= 0.0:
                return 1 if once else 0
            receive_timeout = UDP_RECEIVE_TIMEOUT_S
            if remaining is not None:
                receive_timeout = min(receive_timeout, remaining)
            try:
                sample = receiver.receive(timeout_s=receive_timeout)
            except socket.timeout:
                continue
            writer.write(sample)
            if once:
                return 0 if _sample_succeeded(sample) else 1
    finally:
        receiver.close()
        writer.close()


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", choices=("serial", "udp"), default="serial")
    parser.add_argument("--port", default="/dev/ttyUSB0")
    parser.add_argument("--device-id", type=int, default=1)
    parser.add_argument("--hz", type=float, default=2.0)
    parser.add_argument("--listen-port", type=int, default=5004)
    parser.add_argument("--json", action="store_true", dest="json_output")
    parser.add_argument("--once", action="store_true")
    parser.add_argument("--timeout-s", type=float)
    args = parser.parse_args(argv)
    if not 0.2 <= args.hz <= 5.0:
        parser.error("hz must be within 0.2..5.0")
    if not 0 <= args.device_id <= 253:
        parser.error("device-id must be within 0..253")
    if not 1 <= args.listen_port <= 65535:
        parser.error("listen-port must be within 1..65535")
    if args.timeout_s is not None and args.timeout_s <= 0.0:
        parser.error("timeout-s must be greater than 0")
    return args


def main(argv=None) -> int:
    args = parse_args(argv)
    writer = OutputWriter(
        sys.stdout,
        json_mode=args.json_output,
        live_panel=not args.once,
    )
    try:
        if args.source == "serial":
            poller = SerialPoller(args.port, args.device_id)
            return run_serial(
                poller,
                writer,
                hz=args.hz,
                once=args.once,
                timeout_s=args.timeout_s,
            )
        try:
            receiver = UdpReceiver(args.listen_port)
        except OSError as exc:
            print(f"PDIST80B UDP bind failed: {exc}", file=sys.stderr, flush=True)
            return 1
        return run_udp(
            receiver,
            writer,
            once=args.once,
            timeout_s=args.timeout_s,
        )
    except KeyboardInterrupt:
        writer.close()
        return 0


if __name__ == "__main__":
    raise SystemExit(main())
