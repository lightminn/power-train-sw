from __future__ import annotations

import time
from abc import ABC, abstractmethod

from .base import Transport, TransportError


class CanDevice(ABC):
    """공유 CAN 버스 위 한 모터 유닛. CanTransport 가 집계한다."""

    name: str = "base"

    @abstractmethod
    def attach(self, bus) -> None:
        """공유 버스 주입 (connect 시 1회)."""

    @abstractmethod
    def capabilities_fragment(self) -> dict:
        """이 디바이스의 signals/commands/control_modes/inputs/tunables/limits/signal_meta 조각."""

    def request(self, bus) -> None:
        """폴링형 디바이스의 RTR 송신. 기본 no-op."""

    def on_rx(self, msg) -> None:
        """내 프레임이면 캐시 상태 갱신. 기본 no-op."""

    def tick(self, bus) -> None:
        """워치독 재전송 등 주기 동작. 기본 no-op."""

    @abstractmethod
    def sample(self) -> dict:
        """캐시 상태 → 텔레메트리 조각."""

    @abstractmethod
    def apply(self, bus, op: str, args: dict) -> dict:
        """이 디바이스 대상 명령 처리. ack dict."""

    def close(self, bus) -> None:
        """안전 정지. 기본 no-op."""


class CanTransport(Transport):
    """can0 버스 1개 + CanDevice 리스트 집계 (Transport 계약 구현)."""

    name = "can"

    def __init__(self, devices, channel: str = "can0",
                 track: str = "can", bus=None) -> None:
        self._devices = devices
        self._channel = channel
        self._track = track
        self._bus = bus              # 주입 시 테스트용 (socketcan open 생략)
        self._owns_bus = bus is None

    def connect(self) -> None:
        if self._bus is None:
            import can
            try:
                self._bus = can.interface.Bus(channel=self._channel,
                                              interface="socketcan")
            except OSError as e:
                raise TransportError(
                    f"{self._channel} open 실패 — 'bash scripts/can_setup.sh' 먼저 ({e})")
        for d in self._devices:
            d.attach(self._bus)

    def sample(self) -> dict:
        for d in self._devices:
            d.request(self._bus)
        deadline = time.monotonic() + 0.008
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.002)
            if msg is None:
                break
            for d in self._devices:
                d.on_rx(msg)
        for d in self._devices:
            d.tick(self._bus)
        s = {"t_mono": time.monotonic()}
        for d in self._devices:
            s.update(d.sample())
        return s

    def apply(self, cmd: dict) -> dict:
        target = cmd.get("target")
        for d in self._devices:
            if d.name == target:
                return d.apply(self._bus, cmd["op"], cmd.get("args", {}))
        return {"ok": False, "target": target,
                "op": cmd.get("op"), "detail": "unknown target"}

    def capabilities(self) -> dict:
        caps = {"track": self._track, "devices": [], "signals": [],
                "commands": {}, "control_modes": {}, "inputs": {},
                "tunables": {}, "limits": {}, "signal_meta": {},
                "notes": ["CAN 트랙 — NVM 저장 불가 (USB 전용)"]}
        for d in self._devices:
            f = d.capabilities_fragment()
            caps["devices"] += f.get("devices", [])
            caps["signals"] += f.get("signals", [])
            for key in ("commands", "control_modes", "inputs", "tunables", "limits"):
                caps[key].update(f.get(key, {}))
            caps["signal_meta"].update(f.get("signal_meta", {}))
        return caps

    def close(self) -> None:
        for d in self._devices:
            try:
                d.close(self._bus)
            except Exception:
                pass
        if self._owns_bus and self._bus is not None:
            try:
                self._bus.shutdown()
            except Exception:
                pass
