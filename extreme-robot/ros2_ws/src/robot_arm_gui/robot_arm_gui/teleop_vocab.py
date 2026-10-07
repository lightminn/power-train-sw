#!/usr/bin/env python3
"""`/arm/teleop_cmd` 어휘 화이트리스트 — GUI 가 보낼 수 있는 명령의 단일 정의.

## 왜 화이트리스트인가

`teleop_core_node.on_cmd` 는 모르는 명령을 받으면 경고만 찍고 넘긴다. 즉 브라우저가
오타를 보내도 **아무 일도 안 일어나고 아무도 모른다**. 제어 경로에서는 그게 가장
나쁜 실패 모드라, GUI 쪽에서 먼저 걸러 400 으로 되돌려준다.

또한 이 파일이 없으면 어휘가 프론트엔드 JS 에 흩어져 박히고, `teleop_core` 가
명령을 추가/변경했을 때 어디를 고쳐야 하는지 알 수 없게 된다.

⚠️ **여기 목록은 `teleop_core_node.on_cmd` 의 분기와 짝이다.** 그쪽이 바뀌면 여기도
바꾼다(계약 어휘를 `dynamixel_control.contract` 에서 그대로 읽어오는 것과 같은 사상 —
다만 이쪽은 상수가 아니라 파서 분기라 import 로 공유할 수가 없어 목록을 둔다).
"""

import re

#: 인자가 없는 명령.
NO_ARG = (
    'stop',              # 즉시 정지 + 전 관절 토크 차단 (팔이 중력으로 처질 수 있다)
    'resume',            # stop 으로 끊은 토크 복귀 (현재 자세 홀드)
    'home',              # 저장된 자세 'home' 으로 이동 (all-zero 낙하가 아니다)
    'poses',             # 저장된 자세 목록 재발행
    'freedrive',         # 팔 관절 토크 OFF — 손으로 자세 잡기
    'freedrive_cancel',
    'calib_start',       # 가동범위 측정 시작
    'calib_mark',        # 현재 지점을 리밋으로 기록
    'calib_cancel',
)

#: 이름 인자가 **필수**인 명령.
#: 'delete' 는 2026-08-19 추가 — teleop_core._cmd_delete_pose 와 짝이다.
NAME_ARG = ('save', 'goto', 'delete', 'reboot')

#: on|off|<정수 임계값> 을 받는 명령.
THRESHOLD_ARG = ('spike', 'trip')

#: on|off 만 받는 명령.
TOGGLE_ARG = ('limit',)

#: 자세/관절 이름에 허용하는 문자 — 공백과 쉼표를 막아야 한다.
#: (쉼표는 `/arm/teleop_poses` 의 구분자이고, 공백은 action/arg 분리자다.)
_NAME_RE = re.compile(r'^[A-Za-z0-9_-]{1,32}$')


def all_commands():
    """UI 가 버튼을 그릴 때 쓰는 전체 목록."""
    return {
        'no_arg': list(NO_ARG),
        'name_arg': list(NAME_ARG),
        'threshold_arg': list(THRESHOLD_ARG),
        'toggle_arg': list(TOGGLE_ARG),
    }


def validate(raw):
    """`(정규화된 명령, None)` 또는 `(None, 사유)`.

    정규화는 `teleop_core.on_cmd` 와 같은 규칙이다 — action 만 소문자로 바꾸고
    인자의 대소문자는 보존한다(그쪽이 자세 이름 조회에서 따로 흡수한다).
    """
    if not isinstance(raw, str):
        return None, '명령이 문자열이 아닙니다'
    text = raw.strip()
    if not text:
        return None, '빈 명령'
    if len(text) > 96:
        return None, '명령이 너무 깁니다'

    action, _, arg = text.partition(' ')
    action = action.lower()
    arg = arg.strip()

    if action in NO_ARG:
        if arg:
            return None, f'{action} 은 인자를 받지 않습니다'
        return action, None

    if action in NAME_ARG:
        if not arg:
            return None, f'{action} 에는 이름이 필요합니다 (예: {action} home)'
        if not _NAME_RE.match(arg):
            return None, f'이름에 쓸 수 없는 문자: {arg!r} (영숫자·_·- 만, 32자 이하)'
        return f'{action} {arg}', None

    if action in THRESHOLD_ARG:
        if arg in ('on', 'off'):
            return f'{action} {arg}', None
        if arg.isdigit():
            return f'{action} {arg}', None
        return None, f'{action} 은 on|off|<정수> 만 받습니다 (받은 값: {arg!r})'

    if action in TOGGLE_ARG:
        if arg in ('on', 'off'):
            return f'{action} {arg}', None
        return None, f'{action} 은 on|off 만 받습니다 (받은 값: {arg!r})'

    return None, f'알 수 없는 명령: {action!r}'
