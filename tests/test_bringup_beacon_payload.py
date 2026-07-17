import json

from scripts.pdist80b_telemetry_sender import (
    build_bringup_status,
    encode_payload,
    payload_for,
)


def _raise_probe_error():
    raise OSError("probe unavailable")


def test_build_bringup_status_maps_unit_and_compose_status():
    status = build_bringup_status(
        lambda: {
            "powertrain-bringup-preflight.service": "active",
            "powertrain-pdist80b-telemetry.service": "active",
        },
        lambda: {"powertrain_control": "healthy"},
        lambda: ["bring-up ready"],
    )

    assert status["unit_status"] == {
        "powertrain-bringup-preflight.service": "active",
        "powertrain-pdist80b-telemetry.service": "active",
    }
    assert status["compose_status"] == {"powertrain_control": "healthy"}


def test_build_bringup_status_redacts_journal_secrets():
    token = "tok-console-do-not-leak"
    password = "robot-password-do-not-leak"
    database_password = "database-password-do-not-leak"
    sudo_password = "sudo-password-do-not-leak"

    status = build_bringup_status(
        lambda: {},
        lambda: {},
        lambda: [
            f"loaded ops_console.token={token}",
            f'login rejected password: "{password}"',
            f"DB_PASSWORD={database_password}",
            f'SUDO_PASS: "{sudo_password}"',
        ],
    )

    encoded = json.dumps(status)
    assert token not in encoded
    assert password not in encoded
    assert database_password not in encoded
    assert sudo_password not in encoded
    assert encoded.count("[REDACTED]") == 4


def test_build_bringup_status_defends_each_probe_failure_with_none():
    status = build_bringup_status(
        _raise_probe_error,
        _raise_probe_error,
        _raise_probe_error,
    )

    assert status == {
        "unit_status": None,
        "compose_status": None,
        "journal_tail": None,
    }


def test_payload_for_preserves_existing_fields_when_bringup_status_is_added():
    bringup_status = build_bringup_status(
        lambda: {"powertrain-bringup-preflight.service": "active"},
        lambda: {"powertrain_control": "running"},
        lambda: ["오류🚫" * 2000, "ready"],
    )

    payload = payload_for(
        None,
        sequence=7,
        rs485_state="ERROR",
        consecutive_failures=2,
        detail="timeout",
        bringup_status=bringup_status,
    )

    encoded = encode_payload(payload)
    decoded = json.loads(encoded)

    assert len(encoded) <= 8192
    assert decoded["schema_version"] == 1
    assert decoded["sequence"] == 7
    assert decoded["rs485_state"] == "ERROR"
    assert decoded["rs485_detail"] == "timeout"
    assert decoded["unit_status"] == {
        "powertrain-bringup-preflight.service": "active"
    }
    assert decoded["compose_status"] == {"powertrain_control": "running"}
    assert decoded["journal_tail"] == ["ready"]
