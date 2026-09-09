"""CAN TX-stall recovery with exclusive maintenance and owner coordination.

Standalone mode resets only while no motor owner exists. Active owners call
step() in their control executor with owner_session and before_reset=cm.estop.
The stop latch remains set after recovery; automatic re-arm is never performed.
A persistent reset generation and under-lock probes reject old observations.
"""
import fcntl
import socket
import struct
import threading
import time

from chassis.runtime_lock import (
    CanMaintenanceSession, CanOwnershipError, reset_generation,
)

# ioctl 상수 (linux/sockios.h)
_SIOCGIFFLAGS = 0x8913
_SIOCSIFFLAGS = 0x8914
_SIOCSIFTXQLEN = 0x8943
_IFF_UP = 0x1

_CAN_RTR_FLAG = 0x40000000
_PROBE_ARB = (21 << 5) | 0x09          # 미사용 노드 21 RTR — 아무도 처리 안 함


class CanWatchdog:
    """CAN stall observer; active owners call step() in their control executor."""

    def __init__(self, channel: str = "can0", period_s: float = 1.0,
                 txqueuelen: int = 1000, *, owner_session=None,
                 before_reset=None, lock_path=None):
        self._channel = channel
        self._period = period_s
        self._txqueuelen = txqueuelen
        self._sock = None
        self.resets = 0                # 복구 횟수 (텔레메트리/디버깅용)
        self._fails = 0
        self._last_tx = None
        self._last_reset_error = None
        self._owner_session = owner_session
        self._before_reset = before_reset
        self._lock_path = lock_path or (owner_session.path if owner_session else
                                       f"/run/powertrain/{channel}.lock")
        self._last_generation = None
        self._reset_pending = False

    # ------------------------------------------------------------------
    def start(self) -> threading.Thread:
        t = threading.Thread(target=self._run, daemon=True,
                             name="can-watchdog-%s" % self._channel)
        t.start()
        return t

    # ------------------------------------------------------------------
    def _open_probe_socket(self):
        s = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        # 빈 필터 = 아무것도 수신 안 함 (송신 전용 프로브)
        s.setsockopt(socket.SOL_CAN_RAW, socket.CAN_RAW_FILTER, b"")
        s.bind((self._channel,))
        s.setblocking(False)
        return s

    def _probe_ok(self) -> bool:
        """프로브 프레임 1개 송신 — 큐에 실리기만 하면 True."""
        frame = struct.pack("=IB3x8s", _PROBE_ARB | _CAN_RTR_FLAG, 0, bytes(8))
        try:
            self._sock.send(frame)
            return True
        except OSError:
            return False

    def _tx_packets(self):
        try:
            with open("/sys/class/net/%s/statistics/tx_packets" % self._channel) as f:
                return int(f.read())
        except (OSError, ValueError):
            return None

    def _interface_is_up(self):
        """인터페이스가 존재하고 IFF_UP 상태인지 조회한다.

        부팅 직후처럼 아직 CAN bit timing이 설정되지 않은 상태와 조회 실패는
        모두 ``None/False``로 취급해 복구 대상에서 제외한다.
        """
        name = self._channel.encode()
        s = None
        try:
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            ifr = fcntl.ioctl(s, _SIOCGIFFLAGS, struct.pack("16sh", name, 0))
            return bool(struct.unpack("16sh", ifr)[1] & _IFF_UP)
        except OSError:
            return None
        finally:
            if s is not None:
                s.close()

    def _reset_interface(self):
        """ioctl down → up → txqueuelen (ip 바이너리 불필요)."""
        name = self._channel.encode()
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            ifr = fcntl.ioctl(s, _SIOCGIFFLAGS, struct.pack("16sh", name, 0))
            flags = struct.unpack("16sh", ifr)[1]
            fcntl.ioctl(s, _SIOCSIFFLAGS, struct.pack("16sh", name, flags & ~_IFF_UP))
            time.sleep(0.05)
            fcntl.ioctl(s, _SIOCSIFFLAGS, struct.pack("16sh", name, flags | _IFF_UP))
            fcntl.ioctl(s, _SIOCSIFTXQLEN, struct.pack("16si", name, self._txqueuelen))
        finally:
            s.close()

    def _reopen_probe_socket(self):
        replacement = self._open_probe_socket()
        old = self._sock
        self._sock = replacement
        if old is not None:
            old.close()

    def step(self):
        """Run one observation in the owner's executor; lazily open the probe."""
        if self._reset_pending:
            return self._retry_interrupted_reset()
        if self._sock is None:
            if not self._interface_is_up():
                self._fails = 0
                self._last_tx = None
                return "down"
            try:
                self._sock = self._open_probe_socket()
            except OSError as exc:
                self._last_reset_error = exc
                return "probe_unavailable"
        return self._step()

    def close(self):
        """Close the private probe socket; never change the link or owner lock."""
        old, self._sock = self._sock, None
        if old is not None:
            old.close()

    def _step(self):
        if self._reset_pending:
            return self._retry_interrupted_reset()
        try:
            is_up = self._interface_is_up()
        except OSError:
            is_up = None
        if not is_up:
            self._fails = 0
            self._last_tx = None
            return "down"

        try:
            generation = reset_generation(self._lock_path)
        except (OSError, ValueError) as exc:
            self._last_reset_error = exc
            self._fails = 0
            return "reset_failed"
        if self._last_generation is not None and generation != self._last_generation:
            self._fails = 0
            self._last_tx = None
        self._last_generation = generation
        tx = self._tx_packets()
        if self._probe_ok():
            self._fails = 0
            self._last_tx = tx
            return "ok"
        if tx is None:
            self._fails = 0
            self._last_tx = None
            return "unknown"

        stalled = self._last_tx is None or tx == self._last_tx
        self._fails = self._fails + 1 if stalled else 0
        self._last_tx = tx
        if self._fails < 2:
            return "failed"
        self._fails = 0

        try:
            with CanMaintenanceSession(self._channel, path=self._lock_path,
                                       owner_session=self._owner_session) as maintenance:
                # The competing watchdog may have reset while we were observing.
                if maintenance.generation != generation:
                    self._last_generation = maintenance.generation
                    self._last_tx = None
                    return "superseded"
                # Actual old observations are invalidated under the lock, even
                # if a different process recovered without changing generation.
                if (not self._interface_is_up() or self._tx_packets() != tx
                        or self._probe_ok()):
                    self._last_tx = None
                    return "recovered"
                return self._perform_reset(maintenance)
        except CanOwnershipError as exc:
            self._last_reset_error = exc
            return "owner_busy"
        except Exception as exc:
            self._last_reset_error = exc
            return "reset_failed"

    def _perform_reset(self, maintenance):
        if self._owner_session is not None:
            if self._before_reset is None:
                raise CanOwnershipError("owner recovery requires a stop latch callback")
            if self._before_reset() is False:
                raise RuntimeError("CAN recovery stop latch was refused")
        self._last_generation = maintenance.mark_reset()
        self._reset_pending = True
        self._reset_interface()
        self._reset_pending = False
        self.resets += 1
        self._last_reset_error = None
        self._last_tx = None
        try:
            self._reopen_probe_socket()
        except OSError:
            pass
        return "reset"

    def _retry_interrupted_reset(self):
        # Only our own incomplete down/up can be retried while DOWN. An initially
        # down link is never raised, and any newer maintenance cancels this intent.
        try:
            with CanMaintenanceSession(self._channel, path=self._lock_path,
                                       owner_session=self._owner_session) as maintenance:
                if maintenance.generation != self._last_generation:
                    self._reset_pending = False
                    self._last_tx = None
                    return "superseded"
                return self._perform_reset(maintenance)
        except CanOwnershipError as exc:
            self._last_reset_error = exc
            return "owner_busy"
        except Exception as exc:
            self._last_reset_error = exc
            return "reset_failed"

    def _run(self):
        print("[can_watchdog] %s monitoring; external reset requires no owner"
              % self._channel, flush=True)
        previous_event = None
        try:
            while True:
                event = self.step()
                if event == "reset":
                    print("[can_watchdog] %s reset (%d)" %
                          (self._channel, self.resets), flush=True)
                elif event in ("reset_failed", "owner_busy", "probe_unavailable"):
                    if event != previous_event:
                        print("[can_watchdog] %s: %s" %
                              (event, self._last_reset_error), flush=True)
                previous_event = event
                time.sleep(self._period)
        finally:
            self.close()


def main(argv=None):
    """컨테이너 상주 서비스 진입점 — 포그라운드 실행."""
    import argparse

    p = argparse.ArgumentParser(description="mttcan TX 웻지 자가복구 워치독")
    p.add_argument("--channel", default="can0")
    p.add_argument("--period", type=float, default=1.0, help="감시 주기 s (기본 1)")
    args = p.parse_args(argv)
    CanWatchdog(args.channel, period_s=args.period)._run()


if __name__ == "__main__":
    main()
