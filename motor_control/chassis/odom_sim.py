"""오도메트리 시각 검증 시뮬레이터 — "내 코드가 진짜 맞나"를 눈으로 본다.

    실행:  motor_control/ 에서
        python3 -m chassis.odom_sim                          # 기본(사각형 코스) → PNG
        python3 -m chassis.odom_sim --course circle --show   # 창 띄우기
        python3 -m chassis.odom_sim --slip front_left:1.5    # 앞왼쪽 50% 헛돌림
        python3 -m chassis.odom_sim --noise 0.02             # HALL 노이즈 주입
        python3 -m chassis.odom_sim --no-reject              # 슬립 배제 끄고 비교

────────────────────────────────────────────────────────────────────────
왜 이게 검증이 되는가
────────────────────────────────────────────────────────────────────────
실차는 아직 지상 주행을 못 했으므로 "실제로 어디로 갔는지"(정답)를 알 수 없다.
그런데 **가상 로봇은 정답을 우리가 안다** — 우리가 그렇게 움직이라고 시켰으니까.

    ① 코스 명령 (v, ω) 를 준다                      → 이것이 **정답 경로(ground truth)**
    ② kinematics.solve() 로 바퀴 명령을 만든다       [실제 레포 코드]
    ③ 바퀴가 그대로 굴렀다고 보고 '센서 실측값'을 만든다 (+ 슬립/노이즈 주입 가능)
    ④ odometry.solve_twist() 로 차체 운동을 되푼다    [실제 레포 코드]
    ⑤ 적분해 **추정 경로**를 그리고 ①과 겹쳐 본다

두 선이 포개지면 코어가 맞는 것이고, 벌어지면 그 순간·그 크기가 그대로 보인다.

⚠️ **이 시뮬이 증명하는 것과 못 하는 것**
  · 증명함 : kinematics ↔ odometry 의 정합성, 슬립 배제의 효과, 적분 오차의 크기.
  · 못 함  : **절대 정확도.** 가상 로봇은 `ChassisGeometry` 를 그대로 믿는데, 그 치수
             (CAD-derived commissioning 후보)와 바퀴반경(공칭 0.10 m — 50 kg 하중에서 눌리면 실효
             반경은 더 작다)은 아직 플레이스홀더다. 실제 지면·타이어·서스펜션도 없다.
             절대 정확도는 조립 후 **지상 캘리브레이션**으로만 확정된다.

가정: 가상 로봇은 명령받은 트위스트 (v, ω_applied) 를 정확히 달성한다고 본다.
피벗처럼 조향이 ±45°에 클램프되는 구간에서는 바퀴들이 서로 모순된 값을 보고하므로
(→ `residual` 이 뜬다) 추정이 정답에서 살짝 벌어진다. 그것도 화면에 나타난다.
"""
import argparse
import math

import matplotlib
import matplotlib.pyplot as plt

from chassis.kinematics import default_geometry, solve
from chassis.odometry import (
    OdometryConfig, OdometryIntegrator, WheelObservation, solve_twist,
)

DT = 0.02                                   # 50 Hz — 실기 제어루프와 동일

# 코스 = [(지속시간 s, v m/s, ω rad/s), ...]
COURSES = {
    "straight": [(4.0, 0.5, 0.0)],
    "circle":   [(2 * math.pi / 0.4, 0.4, 0.4)],                  # 반경 1 m 한 바퀴
    "pivot":    [(2.0, 0.0, math.pi / 2)],                        # 제자리 180°
    "square":   sum(([(3.0, 0.4, 0.0), (2.0, 0.4, math.pi / 4)]   # 직진 → 90° 코너
                     for _ in range(4)), []),
    "figure8":  [(2 * math.pi / 0.5, 0.4, 0.5), (2 * math.pi / 0.5, 0.4, -0.5)],
    "slalom":   sum(([(1.2, 0.4, 0.5), (1.2, 0.4, -0.5)] for _ in range(4)), []),
}


def _rng(seed):
    import random
    return random.Random(seed)


def simulate(course, geom_design, geom_true, cfg, slip=None, noise=0.0, seed=0):
    """코스를 돌며 정답 경로와 추정 경로를 함께 만든다.

    **기하를 둘로 나눈다** — 설계값과 실제 제작치수는 반드시 다르기 때문이다.
      geom_design : 우리가 **믿는** 기하. 제어(역기구학)와 추정(순기구학)이 이걸 쓴다.
      geom_true   : 로봇이 **실제로 가진** 기하. 가상 세계의 진실.
    둘이 같으면 이상적인 경우, 다르면 그 차이가 그대로 오차로 나타난다.

    센서 값은 ODrive 가 실제로 주는 **rev/s** 로 주고받는다 — 바퀴 반경 오차가
    어디서 개입하는지를 정확히 모사하기 위해서다:
      · 제어기는 v[m/s] → rev/s 변환에 **설계 반경**을 쓴다.
      · 실제 바퀴는 그 rev/s 로 굴러 **실제 반경**만큼의 지면 속도를 낸다. (← 진실)
      · 오도메트리는 같은 rev/s 를 다시 **설계 반경**으로 되돌린다.
    → 반경 오차는 **오도메트리에 보이지 않는다**(명령값을 그대로 되돌려 받으므로).
      로봇은 실제로 다른 속도로 갔는데 오도메트리는 모른다. 이것이 조립 후 줄자로
      직진 실측을 해야만 하는 이유다 — 스케일 오차는 스스로 못 잡는다.
    """
    slip = slip or {}
    rnd = _rng(seed)
    truth = OdometryIntegrator()             # 정답 = 실제 기하가 만들어내는 실제 운동
    odom = OdometryIntegrator()              # 추정 = 우리가 믿는 기하로 되푼 운동
    log = {k: [] for k in
           ("t", "tx", "ty", "tth", "ox", "oy", "oth", "err", "res", "rej", "vx", "omega")}
    t = 0.0
    circ_design = 2.0 * math.pi * geom_design.wheel_radius_m
    circ_true = 2.0 * math.pi * geom_true.wheel_radius_m

    for dur, v, omega in course:
        for _ in range(int(round(dur / DT))):
            # ── ② 역기구학: 차체 명령 → 바퀴 명령. 제어기는 **설계 기하**만 안다.
            cmd = solve(geom_design, v, omega)

            # ── ③ 실제 바퀴 거동 vs 센서가 보는 값 — **둘을 구분해야 한다**
            #   · 슬립 = **물리**. 바퀴가 헛돌면 지면 속도는 명령대로인데 **센서가 보는
            #     회전수만 부풀려진다** → 정답에는 안 들어가고 추정만 오염시킨다.
            #   · 노이즈 = **센서 오차**. 마찬가지로 추정에만 들어간다.
            #   (둘을 정답에도 넣으면 서로 상쇄돼 오차가 0 으로 나온다 — 실제로 그랬다.)
            obs_true, obs_est = [], []
            for name, wc in cmd.wheels.items():
                rev = wc.drive_turns_per_s                       # 실제로 구른 회전수
                obs_true.append(WheelObservation(name, rev * circ_true, wc.steer_deg))

                rev_meas = rev * slip.get(name, 1.0)             # 헛돌면 더 많이 센다
                if noise:
                    rev_meas += rnd.gauss(0.0, noise / circ_design)
                obs_est.append(WheelObservation(name, rev_meas * circ_design, wc.steer_deg))

            # ── ①' 정답: 실제 기하 위에서 그 바퀴 거동이 만들어내는 차체 운동
            truth.update(solve_twist(geom_true, obs_true, cfg), DT)

            # ── ④ 추정: 같은 센서값을 **설계 기하**로 되푼다 (실제 레포 코드)
            est = solve_twist(geom_design, obs_est, cfg)

            # ── ⑤ 적분
            odom.update(est, DT)

            tx, ty, tth = truth.pose()
            ox, oy, oth = odom.pose()
            t += DT
            for k, val in zip(log, (t, tx, ty, tth, ox, oy, oth,
                                    math.hypot(ox - tx, oy - ty),
                                    est.residual_mps, len(est.rejected),
                                    est.vx, est.omega)):
                log[k].append(val)
    return log


def perturb(geom, radius_pct=0.0, track_pct=0.0, wheelbase_pct=0.0):
    """설계 기하를 비틀어 '실제 제작된 로봇'을 만든다 (백분율 오차)."""
    from chassis.kinematics import ChassisGeometry, Wheel
    return ChassisGeometry(
        wheels=[Wheel(w.name,
                      w.x * (1 + wheelbase_pct / 100.0),
                      w.y * (1 + track_pct / 100.0),
                      w.steerable) for w in geom.wheels],
        wheel_radius_m=geom.wheel_radius_m * (1 + radius_pct / 100.0),
        steer_limit_deg=geom.steer_limit_deg,
        drive_limit_mps=geom.drive_limit_mps,
    )


# ── 그리기 ───────────────────────────────────────────────────────────────


def _draw_robot(ax, geom, pose, color, alpha=1.0):
    """차체 사각형 + 바퀴를 pose 위치에 그린다."""
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)

    def to_world(px, py):
        return (x + px * c - py * s, y + px * s + py * c)

    xs = [w.x for w in geom.wheels]
    ys = [w.y for w in geom.wheels]
    hx, hy = max(xs) + 0.08, max(ys) + 0.05
    corners = [(hx, hy), (hx, -hy), (-hx, -hy), (-hx, hy), (hx, hy)]
    pts = [to_world(*p) for p in corners]
    ax.plot([p[0] for p in pts], [p[1] for p in pts], color=color, lw=1.2, alpha=alpha)

    # 진행 방향 화살표 (앞을 표시)
    nose = to_world(hx + 0.06, 0.0)
    ax.plot([x, nose[0]], [y, nose[1]], color=color, lw=1.2, alpha=alpha)
    for w in geom.wheels:
        wx, wy = to_world(w.x, w.y)
        ax.plot(wx, wy, "o", ms=4, color=color, alpha=alpha,
                mfc=color if w.steerable else "none")


def plot(log, geom, args, out):
    fig = plt.figure(figsize=(13, 6))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.5, 1], hspace=0.35, wspace=0.25)
    ax = fig.add_subplot(gs[:, 0])
    ax_err = fig.add_subplot(gs[0, 1])
    ax_res = fig.add_subplot(gs[1, 1])

    # ── 경로 ──
    ax.plot(log["tx"], log["ty"], "-", color="#2ca02c", lw=2.5,
            label="정답 경로 (명령한 대로 움직인 가상 로봇)")
    ax.plot(log["ox"], log["oy"], "--", color="#d62728", lw=2.0,
            label="오도메트리 추정 경로 (바퀴 실측값에서 되푼 것)")
    _draw_robot(ax, geom, (log["tx"][0], log["ty"][0], log["tth"][0]), "#2ca02c", 0.5)
    _draw_robot(ax, geom, (log["tx"][-1], log["ty"][-1], log["tth"][-1]), "#2ca02c")
    _draw_robot(ax, geom, (log["ox"][-1], log["oy"][-1], log["oth"][-1]), "#d62728")

    final = log["err"][-1]
    ax.set_title(f"코스 '{args.course}'  ·  최종 위치오차 {final*100:.1f} cm", fontsize=12)
    ax.set_xlabel("x [m]  (앞)")
    ax.set_ylabel("y [m]  (왼쪽)")
    ax.axis("equal")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=9)

    # ── 위치 오차 ──
    ax_err.plot(log["t"], [e * 100 for e in log["err"]], color="#d62728")
    ax_err.set_title("정답 대비 위치 오차", fontsize=10)
    ax_err.set_ylabel("cm")
    ax_err.grid(alpha=0.3)

    # ── 잔차 / 배제 ──
    ax_res.plot(log["t"], log["res"], color="#1f77b4", label="잔차 RMS [m/s]")
    ax_res.set_ylabel("m/s", color="#1f77b4")
    ax_res.set_xlabel("시간 [s]")
    ax_res.grid(alpha=0.3)
    ax_res.set_title("바퀴들이 서로 얼마나 안 맞는가 (= 슬립·스크럽 지표)", fontsize=10)
    ax2 = ax_res.twinx()
    ax2.plot(log["t"], log["rej"], color="#ff7f0e", lw=1.0, label="배제된 바퀴 수")
    ax2.set_ylabel("배제 바퀴 수", color="#ff7f0e")
    ax2.set_ylim(-0.2, 3)

    sub = []
    if args.slip:
        sub.append(f"슬립 {args.slip}")
    if args.noise:
        sub.append(f"노이즈 σ={args.noise} m/s")
    for label, val in (("반경", args.err_radius), ("윤거", args.err_track),
                       ("축거", args.err_wheelbase)):
        if val:
            sub.append(f"{label}오차 {val:+g}%")
    sub.append("배제 OFF" if args.no_reject else "배제 ON")
    fig.suptitle("4WS 오도메트리 검증  —  " + " · ".join(sub), fontsize=13)

    if args.show:
        plt.show()
    else:
        fig.savefig(out, dpi=130, bbox_inches="tight")
        print(f"저장: {out}")
    return final


def _korean_font():
    """한글 폰트가 있으면 쓰고, 없으면 조용히 기본 폰트로 (깨진 네모 방지)."""
    from matplotlib import font_manager
    # Noto Sans CJK 는 지역 변종(JP/SC/TC)도 한글 글리프를 포함하는 pan-CJK 폰트다
    for name in ("NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "Noto Sans CJK JP",
                 "Malgun Gothic", "AppleGothic", "UnDotum"):
        if any(f.name == name for f in font_manager.fontManager.ttflist):
            plt.rcParams["font.family"] = name
            plt.rcParams["axes.unicode_minus"] = False
            return True
    return False


def main():
    p = argparse.ArgumentParser(description="4WS 오도메트리 시각 검증")
    p.add_argument("--course", default="square", choices=sorted(COURSES))
    p.add_argument("--slip", default="", help="예: front_left:1.5  (쉼표로 여러 개)")
    p.add_argument("--noise", type=float, default=0.0, help="바퀴 속도 가우시안 노이즈 σ [m/s]")
    p.add_argument("--no-reject", action="store_true", help="슬립 아웃라이어 배제를 끈다")
    p.add_argument("--err-radius", type=float, default=0.0,
                   help="실제 바퀴반경이 설계보다 N%% 다름 (하중에 눌리면 음수)")
    p.add_argument("--err-track", type=float, default=0.0, help="실제 윤거가 N%% 다름")
    p.add_argument("--err-wheelbase", type=float, default=0.0, help="실제 축거가 N%% 다름")
    p.add_argument("--show", action="store_true", help="저장 대신 창 띄우기")
    p.add_argument("--out", default="odom_sim.png")
    p.add_argument("--seed", type=int, default=0)
    args = p.parse_args()

    if not args.show:
        matplotlib.use("Agg")
    if not _korean_font():
        print("⚠️ 한글 폰트 없음 — 라벨이 깨질 수 있다 (sudo apt install fonts-nanum)")

    slip = {}
    for tok in filter(None, args.slip.split(",")):
        name, _, factor = tok.partition(":")
        slip[name.strip()] = float(factor)

    geom = default_geometry()                       # 우리가 믿는 설계 기하
    geom_true = perturb(geom, args.err_radius, args.err_track, args.err_wheelbase)
    cfg = OdometryConfig(max_reject=0 if args.no_reject else OdometryConfig().max_reject)

    log = simulate(COURSES[args.course], geom, geom_true, cfg, slip, args.noise, args.seed)
    final = plot(log, geom, args, args.out)

    dist = sum(abs(v) * DT for v in log["vx"])
    pct = f" ({final/dist*100:.2f}% of dist)" if dist > 1e-6 else ""   # 피벗은 거리 0
    print(f"코스={args.course}  최종 위치오차={final*100:.2f} cm  "
          f"주행거리≈{dist:.2f} m{pct}  "
          f"평균잔차={sum(log['res'])/len(log['res']):.4f} m/s")


if __name__ == "__main__":
    main()
