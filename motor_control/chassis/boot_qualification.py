from dataclasses import dataclass
from typing import Iterable, Mapping


@dataclass(frozen=True)
class AxisReport:
    node_id: int
    pre_calibrated: bool
    is_calibrated: bool
    encoder_ready: bool
    axis_error: int
    board_serial: str
    config_crc: str


@dataclass(frozen=True)
class QualificationResult:
    qualified: bool
    disqualified_axes: tuple
    voltage_ok: bool


def qualify_axes(
    reports: Iterable[AxisReport],
    *,
    expected_fingerprints: Mapping[int, tuple[str, str]],
    voltage_v: float,
    min_voltage_v: float = 42.0,
) -> QualificationResult:
    disqualified_axes = []

    for report in reports:
        reasons = []
        if not report.pre_calibrated:
            reasons.append("pre_calibrated=false")
        if not report.is_calibrated:
            reasons.append("is_calibrated=false")
        if not report.encoder_ready:
            reasons.append("encoder_ready=false")
        if report.axis_error != 0:
            reasons.append(f"axis_error={report.axis_error}")

        if report.node_id in expected_fingerprints:
            actual_fingerprint = (report.board_serial, report.config_crc)
            if actual_fingerprint != expected_fingerprints[report.node_id]:
                reasons.append("fingerprint_mismatch")

        if reasons:
            disqualified_axes.append((report.node_id, ";".join(reasons)))

    voltage_ok = voltage_v >= min_voltage_v
    return QualificationResult(
        qualified=voltage_ok and not disqualified_axes,
        disqualified_axes=tuple(disqualified_axes),
        voltage_ok=voltage_ok,
    )
