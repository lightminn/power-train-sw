#!/usr/bin/env python3
"""새 `gripper_close_tick` 이 **실제로 도달 가능한 값인지** 실기로 확인한다 (2026-08-19).

## 왜 필요한가

`gripper_presets.py` 의 `gripper_close_tick` 이 2026-08-19 에 -1895 → **-1925** 로
30 tick(3.0°) 깊어졌다. 손(토크 OFF)으로 잰 하드스톱이 -1933 이라 **여유가 8 tick 뿐**이라,
Goal PWM 이 이 목표까지 실제로 밀어낼 수 있는지가 확인되지 않았다.

도달 못 하면 이 저장소가 이미 두 번 겪은 실패가 그대로 재현된다:

    서보가 목표에 영영 못 닿음 → 끝까지 밀며 전류 유지 → **빈손 effort 가 높게 유지**
    → 파지 임계를 넘겨 **빈손을 파지 성공으로 오판** → 빈 그리퍼로 미션 진행

그래서 재는 것은 두 가지다:

  1. **도달점** — 닫기 명령 후 서보가 실제로 멈추는 tick. 목표와의 차이가 곧 마진이다.
  2. **정지 후 빈손 load** — `grasp_effort_thresh`(140) / `drop_effort_thresh`(120) 와
     비교한다. 도달했으면 바닥으로 떨어지고, 못 했으면 뜬 채 유지된다.

## 쓰는 법 (컨테이너 안, 실물 그리퍼 필요 · **조 사이를 비워둘 것**)

    docker exec -it ros2_humble bash
    cd /root/ros2_ws

    # ⚠️ 버스를 독점한다 — 브릿지/픽 스택을 먼저 내린다
    pkill -f '[p]ick.launch.py'; sleep 3
    pkill -9 -f '[m]oveit_dynamixel_bridge|[a]rm_fsm'; sleep 2

    python3 src/dynamixel_control/scripts/verify_gripper_close.py

  · `--pwm`        Goal PWM (기본: preset 의 gripper_goal_pwm)
  · `--close-tick` 확인할 닫힘 목표 (기본: preset 의 gripper_close_tick)
  · `--hold`       닫기 명령 후 관찰 시간 [s]. 기본은 preset 의 트립 시간 표에서
                   **안전한 상한을 자동 계산**한다(미측정 PWM 이면 보수적 하한의 60%).
  · `--keep-open`  끝나고 열림 상태로 두고 토크를 끄지 않는다(연속 시험용)

## 이 스크립트가 하는 "초기화"

  REBOOT(Hardware Error 래치 해제) → Operating Mode 4(Extended Position)
  → Profile Acceleration 25 / Velocity 80 → Goal PWM → Torque ON

브릿지가 기동 때 하는 것과 같은 순서다. 이전 시도에서 Overload 가 래치돼 있으면
REBOOT 없이는 토크가 안 걸려 **"명령했는데 아무 일도 안 일어남"** 으로 보인다.

⚠️ **끝단을 오래 밀지 말 것.** PWM 600 은 트립 시간이 미측정이라 `trip_seconds_for` 가
   885 의 3.5초를 보수적 하한으로 돌려준다. 이 스크립트는 관찰이 끝나면 즉시 열림으로
   물러나고 토크를 끈다.
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

sys.path.insert(0, "/root/ros2_ws/src/dynamixel_control")
from dynamixel_control import bus_lock                     # noqa: E402
from dynamixel_control.gripper_presets import (            # noqa: E402
    DEFAULT_GRIPPER, GRIPPER_PRESETS, trip_seconds_for)

DEVICE = "/dev/ttyUSB0"
BAUD_RATE = 1_000_000
PROTOCOL_VERSION = 2.0

ADDR_OPERATING_MODE = 11
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_GOAL_PWM = 100
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_POSITION = 132

MODE_EXTENDED_POSITION = 4
PROFILE_ACCELERATION = 25       # 브릿지와 같은 값 — 다르면 도달 속도가 달라져 비교가 안 된다
PROFILE_VELOCITY = 80
HWERR_OVERLOAD = 0x20
POLL_HZ = 10.0

#: 이 tick 미만으로 움직이면 "멈췄다" 로 본다. 서보 분해능 노이즈(±1~2)보다 넉넉히 위.
STALL_EPS_TICK = 3
#: 연속 몇 샘플이 정지여야 확정할지 (10Hz → 0.5초)
STALL_SAMPLES = 5


def signed(value, bits):
    sign_bit = 1 << (bits - 1)
    return value - (1 << bits) if value & sign_bit else value


class Bus:
    def __init__(self):
        self.port = PortHandler(DEVICE)
        self.packet = PacketHandler(PROTOCOL_VERSION)

    def open(self):
        if not self.port.openPort():
            raise SystemExit(f"포트를 열 수 없습니다: {DEVICE}")
        if not self.port.setBaudRate(BAUD_RATE):
            self.port.closePort()
            raise SystemExit(f"보레이트 설정 실패: {BAUD_RATE}")

    def close(self):
        self.port.closePort()

    def _check(self, dxl_id, result, label):
        if result != COMM_SUCCESS:
            raise SystemExit(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)} — "
                "브릿지/position_node 가 같은 포트를 쓰고 있지 않은지 확인하세요.")

    def read(self, dxl_id, address, size, label):
        reader = {1: self.packet.read1ByteTxRx,
                  2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, result, _error = reader(self.port, dxl_id, address)
        self._check(dxl_id, result, label)
        return value

    def write(self, dxl_id, address, size, value, label):
        writer = {1: self.packet.write1ByteTxRx,
                  2: self.packet.write2ByteTxRx,
                  4: self.packet.write4ByteTxRx}[size]
        result, _error = writer(self.port, dxl_id, address, value)
        self._check(dxl_id, result, label)

    def reboot(self, dxl_id):
        self.packet.reboot(self.port, dxl_id)


def sample(bus, dxl_id):
    return {
        "hwerr": bus.read(dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1, "hwerr"),
        "load": signed(bus.read(dxl_id, ADDR_PRESENT_LOAD, 2, "load"), 16),
        "pos": signed(bus.read(dxl_id, ADDR_PRESENT_POSITION, 4, "pos"), 32),
    }


def initialize(bus, dxl_id, pwm):
    """REBOOT → 모드/프로파일/PWM → 토크 ON. 브릿지 기동과 같은 순서."""
    print("── 초기화 ──")
    before = sample(bus, dxl_id)
    print(f"  현재: pos={before['pos']} load={before['load']} "
          f"hwerr=0x{before['hwerr']:02X}")
    if before["hwerr"]:
        print(f"  ⚠️ Hardware Error 0x{before['hwerr']:02X} 래치됨 → REBOOT")
    bus.reboot(dxl_id)                       # 래치가 없어도 상태를 아는 지점에서 시작한다
    time.sleep(1.5)
    bus.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 0, "torque off")
    bus.write(dxl_id, ADDR_OPERATING_MODE, 1, MODE_EXTENDED_POSITION, "operating mode")
    bus.write(dxl_id, ADDR_PROFILE_ACCELERATION, 4, PROFILE_ACCELERATION, "profile acc")
    bus.write(dxl_id, ADDR_PROFILE_VELOCITY, 4, PROFILE_VELOCITY, "profile vel")
    bus.write(dxl_id, ADDR_GOAL_PWM, 2, pwm, "goal pwm")
    bus.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 1, "torque on")
    after = sample(bus, dxl_id)
    print(f"  REBOOT 후: pos={after['pos']} hwerr=0x{after['hwerr']:02X} "
          f"/ mode=Extended(4) profile acc={PROFILE_ACCELERATION} vel={PROFILE_VELOCITY} "
          f"/ Goal PWM={pwm}")
    if after["hwerr"]:
        raise SystemExit(
            f"  ✗ REBOOT 후에도 Hardware Error 0x{after['hwerr']:02X} 가 남아 있습니다 — "
            "전원/배선을 확인하세요.")
    return after["pos"]


def drive_to(bus, dxl_id, target, hold_s, label):
    """`target` 으로 명령하고 멈출 때까지(또는 hold_s 까지) 추적한다."""
    print(f"\n── {label}: goal {target} ──")
    bus.write(dxl_id, ADDR_GOAL_POSITION, 4, target & 0xFFFFFFFF, label)
    start = time.time()
    last, still, stalled_at = None, 0, None
    s = sample(bus, dxl_id)
    while True:
        time.sleep(1.0 / POLL_HZ)
        s = sample(bus, dxl_id)
        elapsed = time.time() - start
        if last is not None and abs(s["pos"] - last) < STALL_EPS_TICK:
            still += 1
            if still >= STALL_SAMPLES and stalled_at is None:
                stalled_at = elapsed
        else:
            still = 0
        last = s["pos"]
        if s["hwerr"] & HWERR_OVERLOAD:
            print(f"  ⚠️ {elapsed:.1f}s 에 Overload(0x20) 트립 — 중단")
            break
        if stalled_at is not None and elapsed >= stalled_at + 1.0:
            break                            # 정지 확정 후 1초 더 보고 종료
        if elapsed >= hold_s:
            print(f"  ⏱ {hold_s:.1f}s 관찰 상한 도달 — 중단(끝단을 계속 밀지 않는다)")
            break
    s = sample(bus, dxl_id)
    print(f"  도달 {s['pos']}  (목표 대비 {s['pos'] - target:+d} tick)  "
          f"load={s['load']}  hwerr=0x{s['hwerr']:02X}  "
          f"{'정지 확정 @%.1fs' % stalled_at if stalled_at is not None else '정지 미확정'}")
    return s


def main():
    preset = GRIPPER_PRESETS[DEFAULT_GRIPPER]
    ap = argparse.ArgumentParser(
        description=__doc__.split("\n")[0],
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--id", type=int, default=preset["gripper_ids"][0])
    ap.add_argument("--pwm", type=int, default=preset["gripper_goal_pwm"])
    ap.add_argument("--open-tick", type=int, default=preset["gripper_open_tick"])
    ap.add_argument("--close-tick", type=int, default=preset["gripper_close_tick"])
    ap.add_argument("--hold", type=float, default=None,
                    help="닫기 관찰 상한 [s]. 기본은 트립 시간 표에서 자동 계산")
    ap.add_argument("--keep-open", action="store_true",
                    help="끝나고 토크를 켠 채 열림으로 둔다")
    args = ap.parse_args()

    trip_s, measured = trip_seconds_for(args.pwm)
    if args.hold is not None:
        hold = args.hold
    elif trip_s is None:
        hold = 8.0                            # 무트립이 확인된 PWM
    else:
        hold = max(2.0, trip_s * 0.6)         # 트립 시간의 60% 안에서만 관찰
    print(f"Goal PWM {args.pwm}: 트립까지 {trip_s}s "
          f"({'실측' if measured else '미측정 — 보수적 하한'}) → 관찰 상한 {hold:.1f}s\n")

    grasp_th = preset["grasp_effort_thresh"]
    drop_th = preset["drop_effort_thresh"]

    # 브릿지가 떠 있으면 여기서 막는다 — 같은 버스를 둘이 쓰면 패킷이 섞여
    # **축 하나만 조용히 빠지는** 형태로 망가진다(bus_lock 모듈 주석 참고).
    try:
        lock_fd = bus_lock.acquire(DEVICE)
    except bus_lock.BusInUseError as exc:
        raise SystemExit(f"✗ {exc}") from exc
    bus = Bus()
    bus.open()
    try:
        initialize(bus, args.id, args.pwm)
        drive_to(bus, args.id, args.open_tick, hold, "열기")
        closed = drive_to(bus, args.id, args.close_tick, hold, "닫기(빈손)")

        gap = closed["pos"] - args.close_tick
        load = abs(closed["load"])
        print("\n══ 판정 ══")
        print(f"  목표 close_tick        : {args.close_tick}")
        print(f"  실제 도달              : {closed['pos']}  ({gap:+d} tick)")
        print(f"  정지 후 빈손 load      : {load}")
        print(f"  임계값                 : drop={drop_th:.0f}  grasp={grasp_th:.0f}")
        if abs(gap) <= STALL_EPS_TICK:
            print(f"\n  ✅ 도달했다 — close_tick {args.close_tick} 을 그대로 쓸 수 있다.")
        else:
            print(f"\n  ⚠️ {abs(gap)} tick 못 미쳤다 — 서보가 끝까지 밀며 멈춘 것이다.")
            print(f"     gripper_presets.py 의 gripper_close_tick 을 "
                  f"**{closed['pos']}** 로 내릴 것(도달 가능한 값).")
        if load >= grasp_th:
            print(f"  ❌ 빈손 load {load} ≥ grasp 임계 {grasp_th:.0f} — "
                  "**빈손을 파지 성공으로 오판한다.** close_tick 을 반드시 물릴 것.")
        elif load >= drop_th:
            print(f"  ⚠️ 빈손 load {load} ≥ drop 임계 {drop_th:.0f} — "
                  "낙하 판정이 발화하지 않는다. 임계 재조정 또는 close_tick 완화 필요.")
        else:
            print(f"  ✅ 빈손 load {load} < drop 임계 {drop_th:.0f} — 임계 배치 유효.")
    finally:
        try:
            bus.write(args.id, ADDR_GOAL_POSITION, 4,
                      args.open_tick & 0xFFFFFFFF, "복귀(열기)")
            time.sleep(2.0)
            if not args.keep_open:
                bus.write(args.id, ADDR_TORQUE_ENABLE, 1, 0, "torque off")
                print("\n정리: 열림으로 복귀 + 토크 OFF")
            else:
                print("\n정리: 열림으로 복귀 (토크 유지)")
        except SystemExit:
            pass
        bus.close()
        import os
        os.close(lock_fd)


if __name__ == "__main__":
    main()
