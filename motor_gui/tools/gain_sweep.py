#!/usr/bin/env python3
"""모터 게인 스텝-응답 스윕 — 최적 제어 계수 탐색 도구.

motor_gui 의 HardwareWorker + Transport 를 그대로 써서, 여러 게인 조합으로
위치 스텝 명령을 주고 (1) 정착 오차 (2) 한계진동(p2p) (3) 트립 여부 를 측정한다.
방법론·해석은 docs/motor-gui-tuning-guide.md 참고.

⚠️ 사용 전:
  - 모터가 **자유 회전** 가능한지 (부하·간섭 없음) 확인.
  - GUI 서버가 같은 ODrive 를 점유 중이면 **먼저 종료** (ODrive USB 는 한 프로세스만
    점유 가능):  pkill -f motor_gui.backend.server
  - Jetson 컨테이너 안에서 실행:
      docker compose -f docker/docker-compose.jetson.yml exec -T powertrain \
        bash -lc "cd /workspace && python3 motor_gui/tools/gain_sweep.py --track usb"

새 모터로 바꾸면 아래 COMBOS 와 STEP_TURNS / BW 를 그 모터에 맞게 편집.
"""
from __future__ import annotations

import argparse
import statistics
import time

from motor_gui.backend.worker import HardwareWorker


# (pos_gain, vel_gain, vel_integrator_gain) 조합 — 새 모터면 여기를 편집.
# 권장 순서: 먼저 vel_gain 을 단독으로 올려 '트립 한계'를 찾고(아래 주석 참고),
# 안전한 vel_gain 고정 후 vel_integrator_gain(0 부터) 과 pos_gain 을 스윕.
COMBOS = [
    (8.0, 0.015, 0.0),
    (8.0, 0.015, 0.10),
    (8.0, 0.015, 0.25),
    (12.0, 0.015, 0.0),
    (20.0, 0.015, 0.0),
]


def _make_transport(track: str):
    if track == "usb":
        from motor_gui.backend.transport.usb_odrive import UsbOdriveBackend
        return UsbOdriveBackend(timeout=15)
    if track == "can":
        from motor_gui.backend.transport.can_bus import CanBackend
        return CanBackend()
    if track == "fake":
        from motor_gui.backend.transport.fake import FakeTransport
        return FakeTransport()
    raise SystemExit(f"unknown track: {track}")


def run(track: str, step: float, bw: float, settle: float) -> None:
    w = HardwareWorker(_make_transport(track), rate_hz=100)
    w.start()
    time.sleep(0.3)
    w.submit({"target": "odrive", "op": "set_mode",
              "args": {"control_mode": "position"}})
    w.submit({"target": "odrive", "op": "set_gain",
              "args": {"input_filter_bandwidth": bw}})

    print(f"# track={track} step={step}turn bw={bw} settle={settle}s")
    print("pos_g  vel_g  vel_i | settle_err  p2p(last3s)  trip   verdict")
    try:
        for pg, vg, vig in COMBOS:
            # ⚠️ 콤보마다 반드시 clear_errors + 폐루프 재진입 — 안 그러면 한 콤보가
            #    트립한 뒤 축이 디스암된 채로 남아 이후 콤보가 전부 '안 움직임'으로 나온다.
            w.submit({"target": "odrive", "op": "clear_errors", "args": {}})
            w.submit({"target": "odrive", "op": "set_gain", "args": {
                "pos_gain": pg, "vel_gain": vg, "vel_integrator_gain": vig}})
            w.submit({"target": "odrive", "op": "set_state",
                      "args": {"state": "closed_loop"}})
            time.sleep(0.4)
            st = w.latest()
            if st.get("odrive.state") != 8 or st.get("odrive.axis_err", 0) != 0:
                print(f"{pg:5.1f} {vg:6.3f} {vig:5.2f} | TRIP at arm "
                      f"(state={st.get('odrive.state')} err=0x{st.get('odrive.axis_err', 0):x})")
                continue

            w.submit({"target": "odrive", "op": "set_origin", "args": {}})
            time.sleep(0.3)
            w.submit({"target": "odrive", "op": "set_input", "args": {"pos": step}})

            samples, errs = [], []
            t_end = time.time() + settle
            while time.time() < t_end:
                s = w.latest()
                samples.append(s["odrive.pos"])
                errs.append(s.get("odrive.axis_err", 0))
                time.sleep(0.02)

            tail = samples[-int(3 / 0.02):]            # 마지막 3초
            err = statistics.fmean(samples[-50:]) - step
            p2p = max(tail) - min(tail)
            tripped = any(e != 0 for e in errs)
            verdict = ("TRIP" if tripped else
                       "OK" if (abs(err) < 0.03 and p2p < 0.02) else
                       "진동" if p2p >= 0.02 else f"잔차{abs(err) * 360:.0f}deg")
            print(f"{pg:5.1f} {vg:6.3f} {vig:5.2f} | {err:+.4f}    "
                  f"{p2p:.4f}      0x{max(errs):x}    {verdict}")
            w.submit({"target": "odrive", "op": "set_input", "args": {"pos": 0.0}})
            time.sleep(1.0)
    finally:
        w.estop()
        w.stop()
    print("done")


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--track", choices=["usb", "can", "fake"], default="usb")
    p.add_argument("--step", type=float, default=2.0, help="스텝 목표 (turns)")
    p.add_argument("--bw", type=float, default=50.0, help="input_filter_bandwidth (Hz)")
    p.add_argument("--settle", type=float, default=7.0, help="콤보당 관찰 시간 (s)")
    args = p.parse_args()
    run(args.track, args.step, args.bw, args.settle)


if __name__ == "__main__":
    main()
