#!/usr/bin/env python3
"""팔 관절의 안전 가동범위 — 단일 출처 (2026-08-07 신설).

## 왜 이 모듈이 필요한가

`moveit_dynamixel_bridge` 의 `rad_to_tick` 은 그동안 **서보 tick 범위로만** clamp 했다
(0~4095, 다회전축은 ±256회전). 그건 "서보가 표현할 수 있는 값" 이지 "관절이 부딪히지
않는 범위" 가 아니다 — IK 가 엉뚱한 각도를 내면 그대로 서보로 나가 구조물을 때린다.

URDF 를 쓰면 될 것 같지만 못 쓴다: `arm_joint_2/3` 의 URDF 리밋은 `0~π` 자동생성
placeholder 이고(CAD 미반영), `arm_joint_1/5` 는 `continuous` 라 리밋 자체가 없다.
실제로 믿을 수 있는 건 CAD 실측이 반영된 `arm_joint_4` 뿐이다.

그래서 **실측 가동범위**를 이 모듈에 모아 단일 출처로 둔다.

## 도메인 주의 (제일 헷갈리는 지점)

여기 값은 전부 **관절 각도(rad)** 다 — `moveit_dynamixel_bridge.JOINT_CONFIG` 의
`center`/`gear_ratio` 로 환산된 도메인이고, MoveIt/`arm_fsm` 이 쓰는 도메인과 같다.

`teleop_core_node.py` 의 `DEFAULT_MIN_RADS`/`DEFAULT_MAX_RADS` 는 **서보축 각도**라
숫자가 전혀 다르다. **두 파일 값을 서로 복사하지 말 것.**

⚠️ `center` 나 `gear_ratio` 가 바뀌면(=팔 재조립, 서보 뿔 재장착, 기어비 재측정)
   여기 값은 **전부 무효**다. 영점 재측정 후 이 범위도 다시 재야 한다.

## confidence 필드

- `measured`   — 이 캘리브 도메인에서 양쪽 하드스톱까지 직접 재서 넣은 값. 신뢰 가능.
- `derived`    — 2026-08-02 서보 도메인 실측을 2026-08-07 캘리브로 **환산**한 값.
                 원 측정이 다른 조립 상태였을 수 있어 `measured` 보다 약하다.
- `provisional`— 실측이 없거나 모순이라 **보수적으로 좁게 박아둔** 값. 실기에서
                 "왜 조금밖에 안 움직이지?" 싶으면 이 표시부터 확인할 것.
- `user-asserted` — 사용자가 "여기까지 가도 된다" 고 확인해 **실측 스톱보다 넓힌** 값.
                 위 셋과 위험의 방향이 반대다: 좁아서 못 쓰는 게 아니라 **넓어서
                 하드스톱을 때릴 수 있다.** 기동할 때마다 경고를 띄우고, 스윕
                 재측정(`scripts/measure_joint_limits.py`)으로 확정되면 `measured`
                 로 바꿀 것.

새로 재려면: `scripts/measure_joint_limits.py` (run_calib.sh limits)
"""

#: 실측이 없는 축에 쓰는 보수적 기본 폭 [rad] (≈±8.6°).
#: 안 움직이는 것보다 조금 움직이는 게 낫지만, 하드스톱 위치를 모르는 상태에서
#: 넓게 열어두면 그대로 들이받는다 — 좁게 두고 실측으로 넓히는 방향이 맞다.
PROVISIONAL_HALF_RANGE = 0.15

# ⚠️ **하한 0.0 의 의미** (arm_joint_2/3/5)
#
# 2026-08-07 실측은 사용자가 **의도적으로 하드스톱 안쪽에서 멈춘** 보수적 범위다
# (첫 구동이라 여유를 크게 뒀다 — 하드스톱 실측이 아니다). 그 결과 세 축에서 측정
# 하한이 home(0.0) 위로 올라와 home 이 범위 밖이 됐다.
#
# 그대로 두면 **파워트레인 계약이 깨진다**: `arm_fsm` 의 stow 자세가 all-zero
# (`stow_joint_positions=[0,0,0,0]`)인데, 리밋이 0 을 배제하면 stow 명령이 clamp 되고
# → `_near_stow_posture()` 의 허용오차(`stow_pos_tol_rad`=0.1rad)를 못 맞추고
# → `STOWED_LOCKED` 를 영영 발행 못 하고
# → 파워트레인이 주행 허가를 못 받아 **차가 출발하지 못한다.**
#
# home 은 영점 캘리브 때 팔이 실제로 그 자세에 서 있었으므로 **도달 가능이 확인된
# 지점**이다. 그래서 상한(사용자의 보수적 값)은 그대로 두고 하한만 0.0 까지 넓혔다.
# 넓힌 방향은 "이미 가 봤던 곳"이라 새로운 위험을 만들지 않는다.
JOINT_LIMITS = {
    # 🆕 2026-08-19 신설. ID 11(XM540-W270) 서보가 실재하는 것이 확인돼 새로 등록된
    # 축이다. 실측이 **하나도 없어** PROVISIONAL_HALF_RANGE 로 좁게 잠가둔다 —
    # 등록을 안 하면 clamp() 가 이 축을 그냥 통과시켜(모듈 규약) 미지의 기어비 위에서
    # 무제한으로 명령이 나간다. 좁혀두면 최악이 "조금밖에 안 돎"이다.
    "arm_joint_1": {
        "lower": -0.2504,
        "upper": +0.2504,
        "confidence": "measured",
        "source": "2026-08-19 하드스톱 실측 (span 32.7°, 마진 2.0°). 감속기 없는 "
                  "직결축(XM540-W270 ID 11). 도달 범위 자체가 ±16.35°로 좁다 — "
                  "리밋이 아니라 기구가 그렇다.",
    },
    "arm_joint_2": {
        "lower": 0.0,
        "upper": +1.7768,
        "confidence": "user-asserted",
        "source": "상한: **2026-08-19 사용자 지시로 +1.4276(81.79°) → +1.7768(101.79°) 확장.** "
                  "실기에서 낮고 먼 목표(z≈0.06, y≈-0.29)가 top-down 파지로 안 닿아 "
                  "DESCEND 에서 IK 가 계속 실패했는데, URDF FK 스캔 결과 해가 j2 를 "
                  "**상한에 붙여 쓰고 있어** 상한이 곧 병목이었다(하한 확장은 같은 "
                  "목표에 효과 0 — 검증함). 확장 효과: 목표까지 2.9cm 부족 → 1.0cm 도달, "
                  "y=-0.29 에서 닿는 최저 z 가 +0.078 → -0.031m. "
                  "⚠️ **이 값은 하드스톱 실측이 아니라 사용자 확인 값이다** — 같은 회차의 "
                  "스윕 실측 스톱은 +1.4625(83.79°)라 그보다 18° 넘어선다. 사용자가 팔이 "
                  "실제로 여기까지 부딪힘 없이 간다고 확인했고, 그 스윕은 하한 쪽에서 "
                  "이미 한 번 진짜 스톱에 못 닿은 전력이 있어(무부하 정지 자세가 측정 "
                  "최소보다 3.7° 아래였다) 상한도 일찍 멈췄을 개연성이 있다. "
                  "**여전히 스윕 재측정으로 확정할 것**(scripts/measure_joint_limits.py). "
                  "하한: 2026-08-19 실측 +0.0577 → 0.0 으로 넓힘(위 '하한 0.0 의 의미'). "
                  "하한도 측정이 의심스럽다(같은 이유) — 다만 이 축에서 하한은 도달 "
                  "범위에 기여하지 않으므로 우선순위가 낮다.",
    },
    "arm_joint_3": {
        "lower": -0.0737,
        "upper": +2.8557,
        "confidence": "measured",
        "source": "2026-08-19 하드스톱 실측 (도달 [-6.22°, +165.62°], 마진 2.0°). "
                  "옛 상한 +2.0341 대비 크게 넓어졌다 — 옛 값은 하드스톱 실측이 "
                  "아니라 사용자가 의도적으로 좁게 멈춘 값이었다.",
    },
    # 이 축만 측정 범위가 home 을 이미 포함해서 손대지 않았다.
    "arm_joint_4": {
        "lower": -1.1650,
        "upper": +1.4910,
        "confidence": "measured",
        "source": "2026-08-19 하드스톱 실측 (도달 [-34.54°, +121.64°], 마진 2.0°) 후 "
                  "같은 날 2차 영점 정정(-34.21°)만큼 평행이동. 하드스톱 위치 자체는 "
                  "재측정하지 않았다 — tick 은 그대로고 라벨만 옮긴 것이다.",
    },
    "arm_joint_5": {
        "lower": -1.5543,
        "upper": +1.4883,
        "confidence": "measured",
        "source": "2026-08-19 하드스톱 실측 (도달 [-91.05°, +87.28°], 마진 2.0°). "
                  "옛 범위 [0.0, +3.1588] 에서 0 을 중심으로 이동했다 — 같은 회차에 "
                  "이 축 영점(center 985 → 675)이 바뀐 결과다.",
    },
}


#: top-down(수직 아래) 파지 자세로 도달 가능한 **tip_link(link_043) 작업공간 경계상자**
#: [m, base_link 기준]. 위 JOINT_LIMITS + URDF FK 전수 스캔(131만 자세, 2026-08-19)의
#: **산출물**이지 독립적으로 잰 값이 아니다 — 리밋이나 URDF가 바뀌면 여기도 무효다.
#:
#: ⚠️ **경계상자는 필요조건이지 충분조건이 아니다.** 상자 안이라고 반드시 닿는 건
#: 아니고(작업공간은 상자보다 훨씬 좁은 곡면 덩어리다), 상자 **밖이면 확실히 못 닿는다.**
#: 그래서 "밖 = 확정 불가" 판정에만 쓰고, 안쪽은 IK 에 최종 판정을 맡긴다.
#:
#: 왜 필요한가 (2026-08-19 실기): pick 을 눌렀는데 팔이 안 움직였다. 원인은 목표
#: `(0.045, -0.412, 0.136)` 이 y 로 10cm 밖이라 IK 가 실패한 것이었는데, 실패 이유가
#: launch 로그 파일에만 남아 콘솔에서는 그냥 "안 움직인다" 로만 보였다. 콘솔이 pick
#: **전에** 이걸로 미리 걸러준다(`mission_console_node.target_report`).
#:
#: 팔은 base_link **-Y 방향으로만** 뻗는다 — `arm_joint_1` 이 ±14.3° 뿐이라 방위
#: 회전으로 거리를 벌 수 없다. y 범위가 전부 음수인 게 그 결과다.
#: ⚠️ **JOINT_LIMITS 를 고치면 이 값도 반드시 다시 산출할 것.** 2026-08-19 `arm_joint_2`
#: 상한을 81.79° → 101.79° 로 넓혔을 때 y 도달이 -0.323 → -0.427 로 10cm 늘었다 —
#: 안 고쳤으면 콘솔이 **실제로는 닿는 목표를 "도달 불가" 로 오판**했을 것이다.
REACH_TOPDOWN = {
    "x": (-0.081, +0.130),
    "y": (-0.427, -0.023),
    "z": (-0.060, +0.216),
}


def outside_topdown_reach(xyz):
    """(x,y,z)[m, base_link] 가 top-down 도달 경계상자 밖이면 축별 이탈량을 돌려준다.

    반환: `[(축이름, 값, 하한, 상한, 이탈량_m), ...]` — 상자 안이면 빈 리스트.
    `REACH_TOPDOWN` 주석의 "필요조건일 뿐" 경고를 그대로 물려받는다: 빈 리스트가
    "도달 가능" 을 보장하지는 않는다.
    """
    out = []
    for axis, value in zip(("x", "y", "z"), xyz):
        lo, hi = REACH_TOPDOWN[axis]
        if value < lo:
            out.append((axis, value, lo, hi, lo - value))
        elif value > hi:
            out.append((axis, value, lo, hi, value - hi))
    return out


def get_limits(joint_name):
    """(lower, upper) 반환. 등록되지 않은 관절이면 None."""
    entry = JOINT_LIMITS.get(joint_name)
    if entry is None:
        return None
    return entry["lower"], entry["upper"]


def clamp(joint_name, rad):
    """관절각을 안전 범위로 제한. (clamped_rad, was_clamped) 반환.

    등록되지 않은 관절은 통과시킨다 — 이 모듈이 모르는 축까지 막아버리면
    새 축을 배선할 때 원인 모를 정지가 난다(호출부가 경고를 찍는다).
    """
    limits = get_limits(joint_name)
    if limits is None:
        return rad, False
    lower, upper = limits
    clamped = max(lower, min(upper, rad))
    return clamped, clamped != rad


def provisional_joints():
    """실측이 안 된(보수적으로 좁혀둔) 관절 목록 — 기동 시 경고용."""
    return [n for n, e in JOINT_LIMITS.items() if e["confidence"] == "provisional"]


def user_asserted_joints():
    """실측 하드스톱보다 **넓혀둔** 관절 목록 — 기동 시 경고용.

    `provisional` 과 위험의 방향이 반대라 별도로 센다: 그쪽은 좁아서 축을 덜 쓰는
    것이고(최악이 "조금밖에 안 돎"), 이쪽은 넓어서 **하드스톱을 때릴 수 있다.**
    같은 경고문으로 뭉치면 심각도가 뒤바뀌어 전달된다.
    """
    return [n for n, e in JOINT_LIMITS.items() if e["confidence"] == "user-asserted"]
