#!/usr/bin/env python3
"""캘리브레이션 측정식 — 단일 출처 (ROS 비의존).

## 왜 이 모듈이 필요한가

영점·기어비·그리퍼 끝단 공식이 그동안 `scripts/measure_*.py` **안에** 있었다. 각
스크립트가 자기 화면에 쓰려고 직접 계산했고, 그때는 소비자가 하나뿐이라 문제가 없었다.

이제 관제 GUI 의 캘리브 마법사가 **같은 값을 다시 계산**한다. 공식을 복사하면
언젠가 갈라지고, 갈라진 사실은 "GUI 로 잰 영점과 스크립트로 잰 영점이 다르다" 는
형태로 실기에서 발견된다 — 그때는 어느 쪽이 맞는지 알 수 없다. 그래서 식을 여기
한 곳에 두고 스크립트와 GUI 가 **똑같이 import** 한다.

ROS 를 전혀 모르므로 하드웨어·ROS 없이 pytest 로 고정된다(`joint_limits` 와 같은 사상).

## 도메인 주의 (제일 헷갈리는 지점)

여기 함수들은 **서보 tick ↔ 관절 rad** 사이를 오간다. `gear_ratio` 는 "관절 1 rad 당
서보축이 도는 rad" 이고, `center`/`direction` 은 `moveit_dynamixel_bridge.JOINT_CONFIG`
의 값이다 — 전부 **관절각 도메인** 기준이다.

`teleop_core_node.py` 의 `DEFAULT_CENTERS`/`DEFAULT_*_RADS` 는 **서보축 도메인**이라
숫자가 전혀 다르다. **두 쪽 값을 서로 복사하지 말 것.**

## 측정 순서 (어기면 결과가 통째로 틀린다)

**기어비 → 영점 → 가동범위.** 기어비가 틀리면 영점 역산이 같이 틀어지고, 둘 중
하나라도 바뀌면 기존 가동범위는 전부 무효다.
"""

import math

#: 단일회전(0~4095) 축의 raw tick 한계.
DXL_MINIMUM_POSITION_VALUE = 0
DXL_MAXIMUM_POSITION_VALUE = 4095
DXL_CENTER_POSITION = 2048

#: X 시리즈 Extended Position Control Mode 의 raw tick 한계(약 ±256회전).
DXL_EXTENDED_MIN_TICK = -1_048_575
DXL_EXTENDED_MAX_TICK = 1_048_575

#: 물리 인코더 상수. 한 바퀴 4096 tick.
DXL_TICKS_PER_REV = 4096.0
TICKS_PER_RAD = DXL_TICKS_PER_REV / (2.0 * math.pi)

#: 기어비 측정에서 "실제로 돌리지 않았다"로 보는 서보축 회전량 하한 [rad].
MIN_SERVO_DELTA_RAD = 0.05

#: 그리퍼 끝단 측정에서 "실제로 여닫지 않았다"로 보는 tick 차이 하한.
MIN_GRIPPER_SPAN_TICK = 50

#: 스트로크가 서보 한 바퀴에 가까우면 단일회전 모드에서 wrap 경계가 사용 범위
#: 한가운데 걸린다 — 그 경고 임계.
GRIPPER_STROKE_WARN_DEG = 300.0


def ticks_per_joint_rad(gear_ratio):
    """관절 1 rad 당 서보 tick 수."""
    return TICKS_PER_RAD * gear_ratio


def tick_bounds(extended):
    """축의 raw tick 한계 `(lo, hi)` — 다회전 여부에 따라 다르다."""
    if extended:
        return DXL_EXTENDED_MIN_TICK, DXL_EXTENDED_MAX_TICK
    return DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE


def rad_to_tick(center, direction, gear_ratio, rad):
    """관절각 → raw tick (clamp 없음).

    `moveit_dynamixel_bridge.rad_to_tick` 의 순수 부분과 같은 식이다. 그쪽은
    여기에 joint_limits clamp 와 tick 한계 clamp 를 덧붙인다.
    """
    return center + direction * rad * ticks_per_joint_rad(gear_ratio)


def tick_to_rad(center, direction, gear_ratio, tick):
    """서보 raw tick → 관절각. `rad_to_tick` 의 역변환."""
    return (tick - center) / (direction * ticks_per_joint_rad(gear_ratio))


def center_from_measurement(center_old, direction, gear_ratio,
                            rad_measured, rad_ref=0.0):
    """기준 자세에서 새 영점(center tick)을 역산한다.

    브릿지가 발행하는 관절각은 **지금의 center 로 환산된 값**이라, 먼저 raw tick 으로
    되돌린 뒤 "그 tick 이 기준각 `rad_ref` 를 가리키도록" center 를 다시 푼다:

        tick_now   = center_old + direction * rad_measured * TICKS_PER_RAD * gear_ratio
        center_new = tick_now   - direction * rad_ref      * TICKS_PER_RAD * gear_ratio

    `rad_ref` 는 기준 자세에서 그 관절이 가리켜야 하는 각(기본 0 = URDF home).
    구조상 0 으로 세울 수 없는 축은 실제 기준각을 넣는다.

    ⚠️ **기어비가 먼저 확정돼 있어야 한다.** 기어비가 틀리면 영점도 같이 틀어진다.
    """
    tpr = ticks_per_joint_rad(gear_ratio)
    tick_now = center_old + direction * rad_measured * tpr
    return tick_now - direction * rad_ref * tpr


def center_shift_deg(shift_tick, gear_ratio):
    """영점 이동량(tick) → 관절 각도 변화량(deg)."""
    return math.degrees(shift_tick / ticks_per_joint_rad(gear_ratio))


def center_out_of_range(center, extended):
    """새 center 가 tick 한계를 벗어나면 사유, 아니면 None.

    벗어났다는 건 보통 기준 자세를 잘못 세웠거나 기어비가 틀렸다는 뜻이다.
    """
    lo, hi = tick_bounds(extended)
    if lo <= center <= hi:
        return None
    return (f'새 center {round(center)} 가 이 축의 tick 범위({lo}~{hi})를 벗어납니다 — '
            '기준 자세나 기어비를 다시 확인하세요.')


def gear_ratio_from_span(servo_delta_rad, joint_deg):
    """서보축 회전량과 실제 관절 회전량으로 기어비를 구한다.

    `(ratio, inverted)` 를 돌려준다. `ratio` 는 **절대값**이고, `inverted` 가 True 면
    서보와 관절이 반대로 돈다(= `JOINT_CONFIG` 의 `direction` 부호를 뒤집어야 할 수
    있다는 신호). 값이 너무 작으면 측정으로 인정하지 않고 `ValueError` 를 낸다 —
    "안 움직였는데 잰 것으로 처리" 가 가장 나쁜 실패 모드다.
    """
    if abs(servo_delta_rad) < MIN_SERVO_DELTA_RAD:
        raise ValueError(
            f'서보가 거의 안 움직였습니다 ({servo_delta_rad:+.4f} rad) — '
            '관절을 실제로 돌렸는지 확인하세요.')
    if abs(joint_deg) < 1e-6:
        raise ValueError('관절 각도가 0 이면 기어비를 계산할 수 없습니다.')
    ratio = servo_delta_rad / math.radians(joint_deg)
    return abs(ratio), ratio < 0


def gripper_endpoints(closed_tick, opened_tick, margin=0):
    """그리퍼 개폐 끝단 tick → 적용할 `(close, open)` + 경고.

    `{'close', 'open', 'stroke_tick', 'stroke_deg', 'warnings'}` 를 돌려준다.

    ⚠️ 마진은 항상 **안쪽**으로 넣는다. 열림/닫힘 tick 의 대소 관계는 조립에 따라
    뒤집히므로(2026-08-07 재실측에서 실제로 뒤집혔다) 부호를 span 에서 가져온다.
    """
    span = opened_tick - closed_tick
    if abs(span) < MIN_GRIPPER_SPAN_TICK:
        raise ValueError(
            f'개폐 tick 차이가 {abs(span):.0f} 밖에 안 됩니다 — 실제로 여닫으셨나요?')

    direction = 1 if span > 0 else -1
    close_final = int(round(closed_tick + direction * margin))
    open_final = int(round(opened_tick - direction * margin))
    stroke_tick = abs(open_final - close_final)
    stroke_deg = stroke_tick / DXL_TICKS_PER_REV * 360.0

    warnings = []
    if stroke_deg > GRIPPER_STROKE_WARN_DEG:
        warnings.append(
            f'스트로크가 서보 한 바퀴에 가깝습니다({stroke_deg:.1f}°). 단일회전(0~4095) '
            '모드면 wrap 경계가 사용 범위 한가운데 걸려 양 끝이 막힙니다 — Extended '
            'Position 모드인지 확인하세요.')
    if close_final < DXL_MINIMUM_POSITION_VALUE or open_final < DXL_MINIMUM_POSITION_VALUE:
        warnings.append(
            '끝단 tick 이 음수입니다 — 다회전 영역이라 extended 설정이 필요합니다'
            '(단일회전으로 clamp 하면 완전 닫힘이 tick 0 에서 잘립니다).')
    return {'close': close_final, 'open': open_final,
            'stroke_tick': stroke_tick, 'stroke_deg': stroke_deg,
            'warnings': warnings}


def format_joint_config_entry(name, joint_id, center, direction, gear_ratio, extended):
    """`JOINT_CONFIG` 에 그대로 붙여넣을 수 있는 두 줄 문자열.

    스크립트와 GUI 가 같은 형식을 내야 해서 서식도 여기 둔다 — 한쪽만 고쳐 두면
    복사한 사람이 어느 쪽이 최신인지 알 수 없다.
    """
    return (f'    "{name}": {{"id": {joint_id}, "center": {int(round(center))}, '
            f'"direction": {direction},\n'
            f'                    "gear_ratio": {gear_ratio}, "extended": {extended}}},')
