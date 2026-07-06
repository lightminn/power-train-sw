"""mttcan TX 웻지 자가복구 워치독 — 인프로세스(데몬 스레드) 판.

배경: 구동모터 PWM 노이즈로 CAN TX 에러 폭풍→bus-off 가 반복되면 Jetson mttcan
드라이버가 TX 큐를 영구 정지(웻지)한다 — berr 0·ERROR-ACTIVE 로 멀쩡해 보이는데
qdisc 백로그에 프레임이 갇히고 모든 send 가 ENOBUFS ("잘 되다가 아예 안 됨").
down/up 만이 복구한다 (2026-07-07 재현·검증, scripts/can_watchdog.sh 의 파이썬판).

사용 두 가지 (정본 = ① 컨테이너 상주 서비스):

① 컨테이너 상주 (docker-compose.jetson.yml 의 `canwatchdog` 서비스 — 컨테이너 켜지면
   자동 가동, restart: unless-stopped):

    python3 -u -m corner_module.can_watchdog          # 포그라운드

② 인프로세스 (텔레옵 진입점에 이미 내장 — ①과 중복 가동해도 무해; 리셋 조건이
   "2연속 정지"라 상대가 먼저 살리면 그냥 조용함):

    from corner_module.can_watchdog import CanWatchdog
    CanWatchdog("can0").start()          # 데몬 스레드 — 프로그램 종료 시 함께 종료

감지(오탐 없음): 1초 주기로 프로브 프레임(빈 노드 21 RTR — 전 노드가 ACK만 함)을
자체 raw 소켓으로 송신. **송신 실패 + tx_packets 카운터 정지**가 2연속이면 웻지 판정
(일시 폭주는 tx_packets 가 계속 증가해 구분됨).

복구: 순수 파이썬 ioctl 로 can0 down→up + txqueuelen 복원 (~0.2s). privileged
컨테이너에서 동작(`ip` 바이너리 불필요). 기존 SocketCAN 소켓들은 down/up 후에도
그대로 살아 있음(ifindex 유지) — 제어 루프는 프레임 몇 개 유실 후 재개된다.
리셋 순간 조향 status 공백으로 코너 stale→FAULT 가 뜰 수 있음(□ 재무장).
"""
import fcntl
import socket
import struct
import threading
import time

# ioctl 상수 (linux/sockios.h)
_SIOCGIFFLAGS = 0x8913
_SIOCSIFFLAGS = 0x8914
_SIOCSIFTXQLEN = 0x8943
_IFF_UP = 0x1

_CAN_RTR_FLAG = 0x40000000
_PROBE_ARB = (21 << 5) | 0x09          # 미사용 노드 21 RTR — 아무도 처리 안 함


class CanWatchdog:
    """can0 TX 웻지 감시·자동복구. start() 후엔 손댈 것 없음."""

    def __init__(self, channel: str = "can0", period_s: float = 1.0,
                 txqueuelen: int = 1000):
        self._channel = channel
        self._period = period_s
        self._txqueuelen = txqueuelen
        self._sock = None
        self.resets = 0                # 복구 횟수 (텔레메트리/디버깅용)

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

    # ------------------------------------------------------------------
    def _run(self):
        # can0 이 아직 없거나(부팅 직후, can_setup 전) 사라져도 포기하지 않고 재시도
        # — 상주 서비스로 쓰일 때의 생존성.
        while self._sock is None:
            try:
                self._sock = self._open_probe_socket()
            except OSError as e:
                print("[can_watchdog] %s 프로브 소켓 실패(%s) — 5s 후 재시도"
                      % (self._channel, e), flush=True)
                time.sleep(5.0)
        print("[can_watchdog] %s 감시 시작 (주기 %.0fs)"
              % (self._channel, self._period), flush=True)
        fails = 0
        last_tx = None
        while True:
            time.sleep(self._period)
            tx = self._tx_packets()
            if self._probe_ok():
                fails = 0
            else:
                stalled = (tx is None) or (tx == last_tx)
                fails = fails + 1 if stalled else 0
                if fails >= 2:                       # 2연속(≈2s) 정지 → 웻지
                    self.resets += 1
                    print("[can_watchdog] TX 웻지 감지 → %s 리셋 (%d회째)"
                          % (self._channel, self.resets), flush=True)
                    try:
                        self._reset_interface()
                    except OSError as e:
                        print("[can_watchdog] 리셋 실패: %s (권한? privileged 필요)"
                              % e, flush=True)
                    # 인터페이스 재생성(ifindex 변경) 대비 — 프로브 소켓 재오픈
                    try:
                        self._sock.close()
                        self._sock = self._open_probe_socket()
                    except OSError:
                        pass                          # 다음 주기 프로브가 다시 걸러냄
                    fails = 0
            last_tx = tx


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
