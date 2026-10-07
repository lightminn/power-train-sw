#!/usr/bin/env python3
"""그리퍼 Goal PWM 별 **Overload 트립까지 걸리는 시간**을 실측한다.

## 왜 필요한가

그리퍼(XL430-W250)는 전류 센싱이 없어 **파지력이 Goal PWM(주소 100)으로만** 정해진다.
그런데 XL430 의 Overload 보호는 부하를 **시간에 누적**해 판정해서, PWM 을 올리면
"무한정 버팀" 이 "유한 시간 뒤 트립" 으로 바뀐다. 트립하면 Hardware Error 0x20 이
래치되고 **토크가 끊겨 화물을 떨어뜨리며, REBOOT 전까지 응답하지 않는다.**

그래서 "파지력을 더 올려도 되는가" 는 **유지 시간을 재봐야만** 답할 수 있는 질문이다.
지금까지 있던 근거는 2026-08-09 즉석 실측 3점뿐이었다:

    PWM 280 → 유지 load 317 → 40초+ 무트립
    PWM 400 → 유지 load 452 → 17초 트립
    PWM 885 → 유지 load 773 → 3.5초 트립

3점 사이가 매우 비선형이라(400→885 는 힘 2.2배에 시간 4.9배) 내삽으로 추정하면
틀린다. 이 스크립트가 원하는 PWM 에서 직접 잰다.

## 쓰는 법 (컨테이너 안, 실물 그리퍼 + 대상 물체 필요)

    docker exec -it ros2_humble bash
    cd /root/ros2_ws
    python3 src/dynamixel_control/scripts/measure_gripper_pwm_limit.py \
        --pwm 400,500,600 --hold 30 --required-hold 20

  · `--pwm`           재볼 Goal PWM 목록 (쉼표 구분)
  · `--hold`          한 시도에서 최대 몇 초까지 버티는지 볼지 (이 시간 넘기면 '생존')
  · `--required-hold` 실제 운용에서 필요한 유지 시간 — 이걸 넘긴 PWM 만 추천한다

⚠️ **버스를 독점한다.** 브릿지(`moveit_dynamixel_bridge`)나 `position_node` 가 떠 있으면
   같은 `/dev/ttyUSB0` 을 두드려 조용한 통신 실패가 난다. 먼저 내릴 것:
       pkill -f '[m]oveit_dynamixel_bridge'; pkill -f '[a]rm_fsm'

⚠️ **매 시도마다 물체를 실제로 물린다.** 대상 물체(95mm 큐브)를 조 사이에 두고 시작할 것.
   트립하면 스크립트가 REBOOT 으로 래치를 풀고 다음 PWM 으로 넘어간다.
"""

import argparse
import sys
import time

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

sys.path.insert(0, "/root/ros2_ws/src/dynamixel_control")
try:
    from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, GRIPPER_PRESETS
except ImportError:      # 워크스페이스 밖에서 돌릴 때
    GRIPPER_PRESETS, DEFAULT_GRIPPER = None, None

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

HWERR_OVERLOAD = 0x20
POLL_HZ = 10.0


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

    def _check(self, dxl_id, result, error, label):
        if result != COMM_SUCCESS:
            raise SystemExit(
                f"ID {dxl_id} {label}: {self.packet.getTxRxResult(result)} — "
                "브릿지/position_node 가 같은 포트를 쓰고 있지 않은지 확인하세요.")
        if error:
            # Hardware Error 는 여기서 예외로 올리지 않는다(측정 대상이다).
            pass

    def read(self, dxl_id, address, size, label):
        reader = {1: self.packet.read1ByteTxRx,
                  2: self.packet.read2ByteTxRx,
                  4: self.packet.read4ByteTxRx}[size]
        value, result, error = reader(self.port, dxl_id, address)
        self._check(dxl_id, result, error, label)
        return value

    def write(self, dxl_id, address, size, value, label):
        writer = {1: self.packet.write1ByteTxRx,
                  2: self.packet.write2ByteTxRx,
                  4: self.packet.write4ByteTxRx}[size]
        result, error = writer(self.port, dxl_id, address, value)
        self._check(dxl_id, result, error, label)

    def reboot(self, dxl_id):
        self.packet.reboot(self.port, dxl_id)


def sample(bus, dxl_id):
    return {
        "hwerr": bus.read(dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1, "hwerr"),
        "load": signed(bus.read(dxl_id, ADDR_PRESENT_LOAD, 2, "load"), 16),
        "pos": signed(bus.read(dxl_id, ADDR_PRESENT_POSITION, 4, "pos"), 32),
    }


def run_trial(bus, dxl_id, pwm, open_tick, close_tick, hold_s, settle_s):
    """PWM 하나에 대해 '닫아서 물고 유지' 를 돌리고 트립까지의 시간을 잰다."""
    # 래치가 남아 있으면 먼저 푼다 — 안 그러면 시작하자마자 트립으로 오독한다.
    if sample(bus, dxl_id)["hwerr"]:
        print("    이전 Hardware Error 래치 → REBOOT 으로 해제")
        bus.reboot(dxl_id)
        time.sleep(1.0)

    bus.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 0, "torque off")
    bus.write(dxl_id, ADDR_GOAL_PWM, 2, pwm, "goal pwm")
    bus.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 1, "torque on")

    # 먼저 열어 물체를 놓아준다(직전 시도의 파지를 물고 시작하면 누적이 섞인다).
    bus.write(dxl_id, ADDR_GOAL_POSITION, 4, open_tick & 0xFFFFFFFF, "open")
    time.sleep(settle_s)

    # 닫아서 문다. 물체가 막으므로 목표에 영영 도달하지 못하고 계속 민다 = 파지 유지.
    bus.write(dxl_id, ADDR_GOAL_POSITION, 4, close_tick & 0xFFFFFFFF, "close")

    start = time.time()
    loads, trip_at, stall_pos = [], None, None
    while True:
        time.sleep(1.0 / POLL_HZ)
        s = sample(bus, dxl_id)
        elapsed = time.time() - start
        if elapsed > 1.0:                      # 닫히는 동안은 빼고 유지 구간만 집계
            loads.append(abs(s["load"]))
            stall_pos = s["pos"]
        if s["hwerr"] & HWERR_OVERLOAD:
            trip_at = elapsed
            break
        if s["hwerr"]:
            print(f"    ⚠️ Overload 가 아닌 Hardware Error 0x{s['hwerr']:02X} — 중단")
            trip_at = elapsed
            break
        if elapsed >= hold_s:
            break

    bus.write(dxl_id, ADDR_TORQUE_ENABLE, 1, 0, "torque off")
    if trip_at is not None:
        bus.reboot(dxl_id)
        time.sleep(1.0)
    hold_load = round(sum(loads) / len(loads)) if loads else None
    return {"pwm": pwm, "trip_at": trip_at, "hold_load": hold_load,
            "stall_pos": stall_pos, "survived": trip_at is None}


def main():
    preset = (GRIPPER_PRESETS or {}).get(DEFAULT_GRIPPER or "", {})
    ap = argparse.ArgumentParser(
        description="그리퍼 Goal PWM 별 Overload 트립 시간 실측",
        formatter_class=argparse.RawDescriptionHelpFormatter, epilog=__doc__)
    ap.add_argument("--id", type=int,
                    default=(preset.get("gripper_ids") or [3])[0],
                    help="그리퍼 서보 ID (기본: gripper_presets 의 첫 id)")
    ap.add_argument("--pwm", default="400,500,600",
                    help="재볼 Goal PWM 목록 (쉼표 구분, 1~885)")
    ap.add_argument("--hold", type=float, default=30.0,
                    help="한 시도에서 최대 유지 시간 [s] (넘기면 '생존')")
    ap.add_argument("--required-hold", type=float, default=20.0,
                    help="실제 운용에 필요한 유지 시간 [s] — 추천 기준")
    ap.add_argument("--open-tick", type=int,
                    default=preset.get("gripper_open_tick"))
    ap.add_argument("--close-tick", type=int,
                    default=preset.get("gripper_close_tick"))
    ap.add_argument("--settle", type=float, default=2.5,
                    help="열림 완료를 기다리는 시간 [s]")
    args = ap.parse_args()

    if args.open_tick is None or args.close_tick is None:
        raise SystemExit(
            "open/close tick 을 못 읽었습니다 — --open-tick/--close-tick 을 직접 주세요.")
    pwms = [int(v) for v in args.pwm.split(",") if v.strip()]
    for p in pwms:
        if not 1 <= p <= 885:
            raise SystemExit(f"Goal PWM 은 1~885 입니다 (받은 값 {p})")

    print(f"그리퍼 ID {args.id} / open_tick={args.open_tick} close_tick={args.close_tick}")
    print(f"측정 PWM: {pwms}  최대 유지 {args.hold:.0f}s  (필요 유지 {args.required_hold:.0f}s)")
    print("⚠️ 대상 물체를 조 사이에 두세요. 매 시도마다 열었다 다시 뭅니다.\n")

    bus = Bus()
    bus.open()
    results = []
    try:
        mode = bus.read(args.id, ADDR_OPERATING_MODE, 1, "operating mode")
        if mode not in (3, 4):
            raise SystemExit(
                f"ID {args.id} 가 위치제어 모드가 아닙니다(mode={mode}). "
                "브릿지를 한 번 띄웠다 내려 모드를 정리하세요.")
        # 프로파일은 브릿지와 같은 값으로 — 접근 속도가 다르면 트립 시간도 달라진다.
        bus.write(args.id, ADDR_PROFILE_ACCELERATION, 4, 25, "profile accel")
        bus.write(args.id, ADDR_PROFILE_VELOCITY, 4, 80, "profile velocity")

        for pwm in pwms:
            print(f"[PWM {pwm}] 측정 중...")
            r = run_trial(bus, args.id, pwm, args.open_tick, args.close_tick,
                          args.hold, args.settle)
            results.append(r)
            if r["survived"]:
                print(f"    ✅ {args.hold:.0f}초 생존 (유지 load {r['hold_load']})")
            else:
                print(f"    ❌ {r['trip_at']:.1f}초 만에 트립 "
                      f"(유지 load {r['hold_load']}) → REBOOT 완료")
            time.sleep(1.0)
    finally:
        try:
            bus.write(args.id, ADDR_TORQUE_ENABLE, 1, 0, "torque off")
        except SystemExit:
            pass
        bus.close()

    print("\n" + "=" * 62)
    print(f"{'Goal PWM':>9} | {'유지 load':>9} | {'트립까지':>10} | 판정")
    print("-" * 62)
    usable = []
    for r in results:
        when = f"{args.hold:.0f}s+ 무트립" if r["survived"] else f"{r['trip_at']:.1f}s"
        ok = r["survived"] or r["trip_at"] >= args.required_hold
        if ok:
            usable.append(r["pwm"])
        print(f"{r['pwm']:>9} | {str(r['hold_load']):>9} | {when:>10} | "
              f"{'사용 가능' if ok else '❌ 유지시간 부족'}")
    print("=" * 62)
    if usable:
        best = max(usable)
        print(f"\n👉 필요 유지 {args.required_hold:.0f}초를 만족하는 최대값: "
              f"**gripper_goal_pwm = {best}**")
        print("   gripper_presets.py 의 gripper_goal_pwm 에 넣고, 근거(이 표)를 "
              "주석에 남기세요.")
    else:
        print(f"\n👉 필요 유지 {args.required_hold:.0f}초를 만족하는 PWM 이 없습니다.")
        print("   PWM 으로는 해결이 안 된다는 뜻입니다 — 손가락 마찰(고무/실리콘 패드)을")
        print("   올리세요. 미끄럼 힘은 μ×법선력인데 PWM 은 법선력만 건드립니다.")


if __name__ == "__main__":
    main()
