#!/usr/bin/env python3
"""`/dynamixel/hardware_error` 문자열 파서 (ROS 비의존).

`dynamixel_position_node._publish_hardware_errors()` 가 만드는 문자열을 되돌린다.
원본 조립 코드(dynamixel_control/dynamixel_position_node.py)::

    parts = [f"{joint_name}(ID{dxl_id}):{self._describe_hw_error(...)}" ...]
    self.error_pub.publish(String(data=",".join(parts)))
    # _describe_hw_error 는 비트 라벨들을 "|" 로 잇는다

## ⚠️ 쉼표 함정 — 단순 split(",") 은 틀린다

`HW_ERROR_BITS[7] = "전류급변(SW,비상정지)"` 라벨 **안에 쉼표가 있다.** 그래서

    arm_joint_2(ID14):과부하,gripper_left_pinion_joint(ID3):전류급변(SW,비상정지)|과열

를 `split(",")` 하면 모터 2개가 **3조각**으로 쪼개진다. 항목 경계는 "그 뒤에
`(ID숫자):` 가 오는 쉼표" 뿐이므로 lookahead 로만 자른다.

기존 `keyboard_teleop_node._on_hw_error` 는 원문을 그대로 출력만 해서 이 함정에
걸리지 않았다 — 파싱하는 소비자는 이 모듈이 처음이다.

## 이 모듈이 ROS 비의존인 이유

`dynamixel_position_node` 의 `HW_ERROR_BITS` 를 import 하면 그 모듈 최상단의
`dynamixel_sdk` 까지 통째로 끌려온다. GUI 는 라벨 문자열을 그대로 보여주면
되므로 비트 테이블 자체가 필요 없다. `contract.py` 를 ROS 비의존으로 유지하는
것과 같은 사상이고, 덕분에 하드웨어·ROS 없이 pytest 로 검증된다.
"""

import re


#: 항목 구분자. "뒤에 `<이름>(ID숫자):` 가 오는 쉼표" 에서만 자른다.
#: `[^,]*` 로 다음 쉼표 전까지만 훑기 때문에 라벨 안의 쉼표는 경계로 보지 않는다.
_SPLIT = re.compile(r',(?=[^,]*\(ID\d+\):)')

#: 한 항목: `<관절이름>(ID<숫자>):<라벨|라벨|...>`
_ENTRY = re.compile(r'^(?P<joint>.+?)\(ID(?P<dxl_id>\d+)\):(?P<labels>.*)$')

#: 비트 라벨 구분자 (`_describe_hw_error` 의 "|".join).
_LABEL_SEP = '|'

#: 소프트웨어 합성 비트 라벨 — 실제 레지스터 비트가 아니라 노드가 만든 것이다.
#: (bit 6 = 절대 과전류 트립, bit 7 = 전류 급변 → 전 관절 비상정지)
SOFT_TRIP_LABEL = '전류초과(SW)'
SOFT_SPIKE_LABEL = '전류급변(SW,비상정지)'


def parse_hardware_error(raw):
    """`/dynamixel/hardware_error` 문자열 → 모터별 dict 리스트.

    반환 항목::

        {'dxl_id': 14, 'joint': 'arm_joint_2',
         'labels': ['과부하'],
         'soft_trip': False,      # bit 6 (절대 과전류)
         'soft_spike': False}     # bit 7 (급변 → 전 관절 비상정지)

    에러가 없으면 빈 문자열이 오므로 `[]` 를 돌려준다. 노드가 이 토픽을
    **매 read 주기(30Hz) 무조건** 발행하므로, 호출자는 이전 결과와 diff 해서
    상승/하강 엣지만 이벤트로 남겨야 한다(안 그러면 초당 30줄이 쌓인다).

    형식이 깨진 항목은 조용히 버리지 않고 `joint=None, dxl_id=None` 으로 실어
    보낸다 — 파서가 틀렸을 때 화면에서 보이는 편이 낫다.
    """
    if not raw or not raw.strip():
        return []

    entries = []
    for chunk in _SPLIT.split(raw.strip()):
        chunk = chunk.strip()
        if not chunk:
            continue
        m = _ENTRY.match(chunk)
        if not m:
            entries.append({
                'dxl_id': None, 'joint': None, 'raw': chunk,
                'labels': [chunk], 'soft_trip': False, 'soft_spike': False,
            })
            continue
        labels = [s for s in m.group('labels').split(_LABEL_SEP) if s]
        entries.append({
            'dxl_id': int(m.group('dxl_id')),
            'joint': m.group('joint'),
            'raw': chunk,
            'labels': labels,
            'soft_trip': SOFT_TRIP_LABEL in labels,
            'soft_spike': SOFT_SPIKE_LABEL in labels,
        })
    return entries


def summarize(entries):
    """이벤트 로그 한 줄용 요약. 예) `ID14 과부하 · ID3 전류급변(SW,비상정지)|과열`."""
    if not entries:
        return ''
    parts = []
    for e in entries:
        label = ' | '.join(e['labels']) if e['labels'] else '?'
        parts.append(f"ID{e['dxl_id']} {label}" if e['dxl_id'] is not None else label)
    return ' · '.join(parts)


def latched_ids(entries):
    """latch 된 모터 ID 집합. 엣지 판정(diff)에 쓴다."""
    return frozenset(e['dxl_id'] for e in entries if e['dxl_id'] is not None)
