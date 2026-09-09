"""트랜스포트 무관 액추에이터 인터페이스.

CornerModule 은 이 인터페이스 뒤의 구체 구현(AK40/ODrive USB/CAN/Fake)을
교체해도 동작이 변하지 않는다.
"""
from abc import ABC, abstractmethod
import math
import time


class FeedbackClock:
    """Translate kernel realtime RX stamps without rejuvenating queued frames.

    The smaller of the initial and current realtime-to-monotonic translations
    is conservative across wall-clock corrections. Zero timestamps are allowed
    only for explicitly injected non-CAN test doubles; real sockets fail closed.
    """

    def __init__(self, clock):
        self._now = clock
        self._offset_ms = clock() * 1000.0 - time.time() * 1000.0

    def received_ms(self, timestamp, *, allow_unstamped=False):
        try:
            stamp = float(timestamp)
        except (TypeError, ValueError):
            return None
        now_ms = self._now() * 1000.0
        if stamp == 0 and allow_unstamped:
            return now_ms
        wall_s = time.time()
        if not math.isfinite(stamp) or stamp <= 0 or stamp > wall_s + .1:
            return None
        return min(now_ms, stamp * 1000.0 + self._offset_ms,
                   now_ms - max(0.0, wall_s - stamp) * 1000.0)


class Actuator(ABC):
    @abstractmethod
    def connect(self) -> None:
        """버스/USB 연결."""

    @abstractmethod
    def arm(self) -> None:
        """폐루프 진입. 현재 상태로 타깃을 동기해 점프를 방지한다."""

    @abstractmethod
    def disarm(self) -> None:
        """폐루프 해제."""

    @abstractmethod
    def tick(self) -> None:
        """매 제어 루프 호출. 명령 재전송·상태 폴링 등 통신 서비스."""

    def poll_feedback(self) -> None:
        """Optional bounded feedback queries; never send motion/state commands."""

    @abstractmethod
    def state(self) -> dict:
        """정규화 텔레메트리 딕셔너리."""

    @abstractmethod
    def estop(self) -> None:
        """즉시 정지."""

    @abstractmethod
    def close(self) -> None:
        """연결 해제·정리."""


class SteerActuator(Actuator):
    @abstractmethod
    def set_angle(self, deg: float) -> None:
        """출력축 목표각(도)."""


class DriveActuator(Actuator):
    @abstractmethod
    def set_velocity(self, turns_per_s: float) -> None:
        """목표 속도(turns/s)."""
