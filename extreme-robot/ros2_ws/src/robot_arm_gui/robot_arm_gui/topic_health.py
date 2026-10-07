#!/usr/bin/env python3
"""토픽 신선도 규약과 `/joint_states` 도메인 판별 (ROS 비의존).

## 왜 도메인 판별이 필요한가

`/joint_states` 는 발행자가 **셋**이고 값의 의미가 서로 다르다. 같은 숫자가
발행자에 따라 "서보축 raw 근사"이기도 하고 "실제 관절 rad"이기도 하다.
어느 쪽인지 모르면 각도를 숫자로 그리면 안 되므로, 화면에 항상 같이 찍는다.

launch 파일이 하나만 뜨도록 배타 처리하지만, 수동 기동으로 둘이 동시에 뜨면
값이 뒤섞인다 — 그 경우를 `conflict` 로 잡아 빨간 배너를 띄운다.
"""

#: 토픽별 "이 나이를 넘으면 낡았다" 기준 [s].
#: `/arm_status` 0.5 는 계약값(`contract.HEARTBEAT_TIMEOUT_S`)이라 임의로 바꾸면 안 된다 —
#: 파워트레인이 같은 기준으로 차를 세운다.
STALE_AFTER = {
    '/arm_status': 0.5,
    '/chassis_mode': 1.0,
    '/dynamixel/state': 0.5,
    '/dynamixel/hardware_error': 0.5,
    '/dynamixel/controller_fault': 0.5,
    '/joint_states': 0.5,
    '/detected_objects': 2.0,
    '/joy': 0.5,
    '/arm/teleop_jog': 0.5,
}

#: 위 표에 없는 토픽의 기본 기준. 이벤트성 토픽은 오래 조용해도 정상이라 넉넉히.
DEFAULT_STALE_AFTER = 5.0

#: 발행자 노드명 → `/joint_states` 값의 도메인.
JOINT_STATE_DOMAINS = {
    'dynamixel_position_node': {
        'domain': 'raw',
        'label': 'raw · 기어비 미적용',
        'note': ('position 은 (tick-2048)·2π/4096 서보축 근사이고 '
                 'velocity/effort 는 raw 레지스터값이다. 실제 관절각이 아니다.'),
        'severity': 'warning',
    },
    'moveit_dynamixel_bridge': {
        'domain': 'joint',
        'label': '관절 rad',
        'note': 'center/gear_ratio 가 반영된 실제 관절각·각속도. effort 는 raw load.',
        'severity': 'good',
    },
    'teleop_core': {
        'domain': 'sim',
        'label': 'SIM — 실측 아님',
        'note': '목표값을 그대로 되먹인 open-loop 값이다. 서보 엔코더가 아니다.',
        'severity': 'warning',
    },
}

UNKNOWN_DOMAIN = {
    'domain': 'unknown',
    'label': '발행자 미상',
    'note': '알 수 없는 노드가 /joint_states 를 발행 중이다. 값의 단위를 확인할 것.',
    'severity': 'warning',
}

NO_PUBLISHER = {
    'domain': 'none',
    'label': '발행자 없음',
    'note': 'position_node / moveit_dynamixel_bridge / teleop_core 중 아무것도 안 떠 있다.',
    'severity': 'serious',
}


def stale_limit(topic):
    return STALE_AFTER.get(topic, DEFAULT_STALE_AFTER)


def is_stale(topic, age):
    """age 가 None(한 번도 못 받음)이면 낡은 것으로 본다."""
    if age is None:
        return True
    return age > stale_limit(topic)


def classify_joint_publishers(node_names):
    """`/joint_states` 발행자 노드명 목록 → 도메인 판정 dict."""
    known = [n for n in node_names if n in JOINT_STATE_DOMAINS]
    if not node_names:
        return dict(NO_PUBLISHER, publishers=[])
    if len(node_names) > 1:
        return {
            'domain': 'conflict',
            'label': f'충돌 — 발행자 {len(node_names)}개',
            'note': ('두 드라이버가 같은 버스/토픽을 동시에 잡고 있다. '
                     '값이 뒤섞이며 서보 제어도 경합한다 — 하나만 남길 것.'),
            'severity': 'critical',
            'publishers': list(node_names),
        }
    if known:
        return dict(JOINT_STATE_DOMAINS[known[0]], publishers=list(node_names))
    return dict(UNKNOWN_DOMAIN, publishers=list(node_names))
