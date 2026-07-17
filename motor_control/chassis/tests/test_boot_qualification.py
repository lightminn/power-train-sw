from chassis.boot_qualification import (
    AxisReport,
    QualificationResult,
    qualify_axes,
)


def _report(
    node_id: int,
    *,
    pre_calibrated: bool = True,
    is_calibrated: bool = True,
    encoder_ready: bool = True,
    axis_error: int = 0,
    board_serial: str | None = None,
    config_crc: str | None = None,
) -> AxisReport:
    return AxisReport(
        node_id=node_id,
        pre_calibrated=pre_calibrated,
        is_calibrated=is_calibrated,
        encoder_ready=encoder_ready,
        axis_error=axis_error,
        board_serial=board_serial or f"board-{node_id}",
        config_crc=config_crc or f"crc-{node_id}",
    )


def test_all_axes_qualified() -> None:
    reports = (_report(11), _report(12))
    fingerprints = {
        11: ("board-11", "crc-11"),
        12: ("board-12", "crc-12"),
    }

    result = qualify_axes(
        reports,
        expected_fingerprints=fingerprints,
        voltage_v=48.0,
    )

    assert result == QualificationResult(
        qualified=True,
        disqualified_axes=(),
        voltage_ok=True,
    )


def test_one_uncalibrated_axis_is_disqualified() -> None:
    fingerprints = {
        11: ("board-11", "crc-11"),
        12: ("board-12", "crc-12"),
    }

    pre_calibrated_result = qualify_axes(
        (_report(11), _report(12, pre_calibrated=False)),
        expected_fingerprints=fingerprints,
        voltage_v=48.0,
    )
    is_calibrated_result = qualify_axes(
        (_report(11), _report(12, is_calibrated=False)),
        expected_fingerprints=fingerprints,
        voltage_v=48.0,
    )

    assert pre_calibrated_result.qualified is False
    assert pre_calibrated_result.disqualified_axes == (
        (12, "pre_calibrated=false"),
    )
    assert is_calibrated_result.qualified is False
    assert is_calibrated_result.disqualified_axes == (
        (12, "is_calibrated=false"),
    )


def test_fingerprint_mismatch_disqualifies_axis() -> None:
    result = qualify_axes(
        (_report(11),),
        expected_fingerprints={11: ("replacement-board", "crc-11")},
        voltage_v=48.0,
    )

    assert result.qualified is False
    assert result.disqualified_axes == ((11, "fingerprint_mismatch"),)


def test_axis_error_disqualifies_axis() -> None:
    result = qualify_axes(
        (_report(11, axis_error=16),),
        expected_fingerprints={11: ("board-11", "crc-11")},
        voltage_v=48.0,
    )

    assert result.qualified is False
    assert result.disqualified_axes == ((11, "axis_error=16"),)


def test_voltage_below_minimum_disqualifies_whole_result() -> None:
    result = qualify_axes(
        (_report(11),),
        expected_fingerprints={11: ("board-11", "crc-11")},
        voltage_v=41.9,
    )

    assert result == QualificationResult(
        qualified=False,
        disqualified_axes=(),
        voltage_ok=False,
    )


def test_unregistered_fingerprint_is_skipped_without_disqualification() -> None:
    result = qualify_axes(
        (_report(11),),
        expected_fingerprints={},
        voltage_v=48.0,
    )

    assert result == QualificationResult(
        qualified=True,
        disqualified_axes=(),
        voltage_ok=True,
    )


def test_encoder_not_ready_disqualifies_axis() -> None:
    result = qualify_axes(
        (_report(11, encoder_ready=False),),
        expected_fingerprints={11: ("board-11", "crc-11")},
        voltage_v=48.0,
    )

    assert result.qualified is False
    assert result.disqualified_axes == ((11, "encoder_ready=false"),)


def test_voltage_at_minimum_is_qualified() -> None:
    result = qualify_axes(
        (_report(11),),
        expected_fingerprints={11: ("board-11", "crc-11")},
        voltage_v=42.0,
    )

    assert result == QualificationResult(
        qualified=True,
        disqualified_axes=(),
        voltage_ok=True,
    )
