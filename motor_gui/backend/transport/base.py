from __future__ import annotations

from abc import ABC, abstractmethod


class TransportError(Exception):
    """전송 계층 연결/IO 실패."""


class Transport(ABC):
    """모든 전송(USB/CAN/Fake)의 공통 계약.

    sample()/apply()/capabilities() 는 **JSON-직렬화 가능한 dict 만** 주고받는다
    (웹 레이어와의 seam — 향후 프로세스 격리 시 그대로 IPC 경계가 됨).
    """

    name: str = "base"

    @abstractmethod
    def connect(self) -> None:
        """장치 연결. 실패 시 TransportError."""

    @abstractmethod
    def sample(self) -> dict:
        """텔레메트리 1프레임. 항상 't_mono' 포함, 키는 '<device>.<signal>'."""

    @abstractmethod
    def apply(self, cmd: dict) -> dict:
        """정규화된 command envelope 적용. ack dict 반환."""

    @abstractmethod
    def capabilities(self) -> dict:
        """이 트랙이 노출하는 devices/signals/commands/limits/notes."""

    @abstractmethod
    def close(self) -> None:
        """안전 정지 + 자원 해제."""
