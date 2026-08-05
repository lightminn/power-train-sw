"""조향/트랜스포트 계약.

ROS-free 순수 모듈 — 호스트 pytest 로 검증한다. ops_contract.py 와 같은 이유로
어떤 ROS 심볼도 사용하지 않는다.
"""

DRIVE_TRANSPORTS = ("can", "usb")


def validate_transport_mode(drive_transport, steering_mode):
    """기동 파라미터 조합을 검증한다. 잘못되면 ``ValueError``.

    USB 스택에는 조향 액추에이터가 없다(조향 AK45-36 은 CAN 전용). 따라서
    USB × 애커만은 존재할 수 없는 조합이다.
    """
    if drive_transport not in DRIVE_TRANSPORTS:
        raise ValueError(
            "unknown drive_transport %r (expected one of %s)"
            % (drive_transport, ", ".join(DRIVE_TRANSPORTS)))
    if steering_mode not in ("ackermann", "skid"):
        raise ValueError("unknown steering_mode %r" % (steering_mode,))
    if drive_transport == "usb" and steering_mode == "ackermann":
        raise ValueError(
            "drive_transport=usb 는 ackermann 을 지원하지 않는다 — "
            "USB 스택에는 조향 액추에이터가 없다(AK 는 CAN 전용). "
            "steering_mode=skid 로 기동하라.")


def steering_state_fields(manager, drive_transport):
    """`/chassis/safety_state` 에 실을 조향/트랜스포트 필드."""
    return {
        "steering_mode": str(getattr(manager, "steering_mode", "ackermann")),
        "steering_available": bool(getattr(manager, "steering_available", False)),
        "drive_transport": str(drive_transport),
    }
