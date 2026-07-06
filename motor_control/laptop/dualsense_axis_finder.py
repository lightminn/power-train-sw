"""DualSense(또는 아무 게임패드) 축·버튼 인덱스 찾기 — 가이드형.

pygame 축/버튼 번호는 컨트롤러·드라이버·연결(USB/BT)·SDL 버전마다 달라서, teleop 클라의
상수(LX_AXIS/RT_AXIS/...)가 안 맞으면 조작이 이상해진다. 이 스크립트는 **시키는 대로
하나씩 조작하면 어떤 축/버튼이 움직였는지 자동으로 짚어** 정확한 매핑을 찾아준다.
끝에 `laptop_client_chassis.py` 에 그대로 붙여넣을 상수 블록을 출력한다.

각 단계는 **메인스레드 타임드 캡처**(기본 5초). "지금!" 뜨면 그 컨트롤만 크게 조작하면
되고, 화면에 현재 최다변화 축이 실시간으로 뜬다. (SDL 이벤트 펌프는 메인스레드 전용이라
스레드 캡처는 값이 안 잡힘 → 타임드 방식이 안전)

실행 (노트북, DualSense 연결). ⚠️ pygame 있는 파이썬으로:
  python laptop/dualsense_axis_finder.py                 # conda base(pygame 설치됨) 권장
  /home/light/anaconda3/bin/python laptop/dualsense_axis_finder.py
  python laptop/dualsense_axis_finder.py --monitor       # 원시 실시간 뷰(모든 축/버튼)
  python laptop/dualsense_axis_finder.py --seconds 8      # 캡처창 늘리기
"""
import argparse
import time

import pygame

# (프롬프트, 종류, 클라 상수명)
STEPS = [
    ("좌스틱을 오른쪽 끝까지 밀고 유지",   "axis",   "LX_AXIS"),
    ("RT(R2) 를 끝까지 당기고 유지",       "axis",   "RT_AXIS"),
    ("LT(L2) 를 끝까지 당기고 유지",       "axis",   "LT_AXIS"),
    ("□(Square) 버튼 누르고 유지",         "button", "SQ_BTN"),
    ("○(Circle) 버튼 누르고 유지",         "button", "CI_BTN"),
]


def open_joystick():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("❌ 게임패드가 안 잡힘. DualSense 를 USB/BT 로 연결하고 다시 실행하세요.")
        return None
    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("🎮 컨트롤러: %s" % joy.get_name())
    print("   축 %d개 · 버튼 %d개 · hat %d개\n"
          % (joy.get_numaxes(), joy.get_numbuttons(), joy.get_numhats()))
    return joy


def capture(joy, seconds):
    """메인스레드에서 seconds 동안 폴링 — 축별 최대편차/눌린 버튼 추적."""
    naxes, nbtn = joy.get_numaxes(), joy.get_numbuttons()
    pygame.event.pump()
    base = [joy.get_axis(i) for i in range(naxes)]
    maxdev = [0.0] * naxes
    extreme = list(base)
    pressed = set()
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        pygame.event.pump()
        for i in range(naxes):
            v = joy.get_axis(i)
            d = abs(v - base[i])
            if d > maxdev[i]:
                maxdev[i] = d
                extreme[i] = v
        for b in range(nbtn):
            if joy.get_button(b):
                pressed.add(b)
        lead = max(range(naxes), key=lambda i: maxdev[i])
        remain = seconds - (time.monotonic() - t0)
        btxt = ("버튼 " + ",".join(map(str, sorted(pressed)))) if pressed else "버튼 -"
        print("\r      ⏱ %.0fs  최다변화 axis %d(Δ%.2f) | %s      "
              % (remain, lead, maxdev[lead], btxt), end="", flush=True)
        time.sleep(0.02)
    print()
    return maxdev, extreme, base, pressed


def countdown(msg, secs=2.0):
    for k in range(int(secs), 0, -1):
        print("\r   %s — %d초 뒤 시작… (손 떼고 대기)   " % (msg, k), end="", flush=True)
        time.sleep(1.0)
    print("\r   %s — ▶▶ 지금! 조작하세요                 " % msg)


def guided(joy, seconds):
    print("=== 가이드형 축/버튼 찾기 — 한 번에 하나씩만 조작 ===")
    print("각 단계: '지금!' 뜨면 시킨 것만 크게 조작. 다른 스틱/버튼은 건드리지 말 것.\n")
    result = {}
    detail = {}
    for prompt, kind, const in STEPS:
        print("▶ [%s] %s" % (const, prompt))
        countdown("준비", 2.0)
        maxdev, extreme, base, pressed = capture(joy, seconds)
        if kind == "axis":
            idx = max(range(len(maxdev)), key=lambda i: maxdev[i])
            if maxdev[idx] < 0.3:
                print("   ⚠️ 뚜렷한 축 변화 없음(Δ=%.2f) — 이 단계 재시도 권장\n" % maxdev[idx])
                result[const] = None
                continue
            result[const] = idx
            detail[const] = (base[idx], extreme[idx])
            print("   ✅ %s = axis %d   (%.2f→%.2f, Δ%.2f)\n"
                  % (const, idx, base[idx], extreme[idx], maxdev[idx]))
        else:
            if not pressed:
                print("   ⚠️ 눌린 버튼 없음 — 이 단계 재시도 권장\n")
                result[const] = None
                continue
            idx = sorted(pressed)[0]
            extra = ("  (동시감지 %s — 하나만 눌렀는지 확인)" % sorted(pressed)
                     if len(pressed) > 1 else "")
            result[const] = idx
            print("   ✅ %s = button %d%s\n" % (const, idx, extra))

    print("=" * 54)
    print("결과 매핑:")
    for _, _, const in STEPS:
        v = result.get(const)
        print("   %-9s = %s" % (const, v if v is not None else "??? (재시도)"))

    print("\n극성 확인:")
    if "LX_AXIS" in detail:
        b, e = detail["LX_AXIS"]
        pol = "오른쪽=+ (정상)" if e > 0 else "오른쪽=− (서버 ω 부호 반대 — 알려줘)"
        print("   · 좌스틱 X: %.2f→%.2f  %s" % (b, e, pol))
    for c in ("RT_AXIS", "LT_AXIS"):
        if c in detail:
            b, e = detail[c]
            ok = "정상(뗌−1→당김+1, trig 공식 OK)" if b < -0.6 else \
                 "⚠️ 뗌값이 -1 아님 — 클라 trig()=(raw+1)/2 수정 필요"
            print("   · %s: 뗌 %.2f → 당김 %.2f  %s" % (c, b, e, ok))

    ok = all(result.get(c) is not None for _, _, c in STEPS)
    if ok:
        print("\n▼ laptop_client_chassis.py 상수 블록에 붙여넣기:")
        print("-" * 54)
        print("LX_AXIS = %d      # 좌스틱 X" % result["LX_AXIS"])
        print("RT_AXIS = %d      # R2 (전진)" % result["RT_AXIS"])
        print("LT_AXIS = %d      # L2 (후진)" % result["LT_AXIS"])
        print("SQ_BTN  = %d      # □ arm/disarm" % result["SQ_BTN"])
        print("CI_BTN  = %d      # ○ estop" % result["CI_BTN"])
        print("-" * 54)
        print("\n(이 5줄을 알려주면 유선/무선 클라 상수를 맞춰 커밋할게)")
    else:
        print("\n⚠️ ??? 항목은 그 단계만 다시(재실행) 하거나 --monitor 로 직접 확인.")


def monitor(joy):
    print("=== 원시 실시간 뷰 — 조작 시 값 변하는 축/버튼 인덱스 확인 (Ctrl-C 종료) ===\n")
    naxes, nbtn, nhat = joy.get_numaxes(), joy.get_numbuttons(), joy.get_numhats()
    try:
        while True:
            pygame.event.pump()
            ax = "  ".join("[%d]%+.2f" % (i, joy.get_axis(i)) for i in range(naxes))
            bt = " ".join("%d" % b for b in range(nbtn) if joy.get_button(b)) or "-"
            ht = " ".join(str(joy.get_hat(h)) for h in range(nhat)) or "-"
            print("\r축 %s | 눌린버튼 %s | hat %s        " % (ax, bt, ht),
                  end="", flush=True)
            time.sleep(0.05)
    except KeyboardInterrupt:
        print()


def main():
    ap = argparse.ArgumentParser(description="DualSense 축/버튼 인덱스 찾기")
    ap.add_argument("--monitor", action="store_true",
                    help="가이드 대신 모든 축/버튼 원시값 실시간 표시")
    ap.add_argument("--seconds", type=float, default=5.0,
                    help="각 단계 캡처창 길이(초, 기본 5)")
    args = ap.parse_args()

    joy = open_joystick()
    if joy is None:
        return
    if args.monitor:
        monitor(joy)
    else:
        guided(joy, args.seconds)
    pygame.quit()


if __name__ == "__main__":
    main()
