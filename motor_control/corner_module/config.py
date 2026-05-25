"""코너 모듈 설정값·한계 및 공용 유틸."""
from dataclasses import dataclass


def clamp(value: float, lo: float, hi: float) -> float:
    """value 를 [lo, hi] 로 제한."""
    return max(lo, min(hi, value))


@dataclass
class CornerConfig:
    steer_min_deg: float = -45.0    # 조향 출력축 최소각
    steer_max_deg: float = 45.0     # 조향 출력축 최대각
    drive_vel_limit: float = 5.0    # 구동 최대속도 (turns/s)
    watchdog_ms: float = 300.0      # 텔레옵 입력 타임아웃 (ms)
    loop_hz: float = 50.0           # 제어 루프 주기
    steer_gate: bool = False        # 협조 로직 on/off (기본 OFF)
    gate_deg: float = 10.0          # 협조 감속 시작 조향오차
    stale_ms: float = 200.0         # AK status 미수신 stale 임계
    steer_current_limit_a: float = 5.0  # 조향 모터 과전류 트립 한계 (A)
