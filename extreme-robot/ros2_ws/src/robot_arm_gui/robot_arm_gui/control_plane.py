#!/usr/bin/env python3
"""제어 평면 — GUI 가 실제로 ROS 에 쓰는 유일한 곳.

## 왜 별도 객체인가 (런타임 분기가 아니라 구조로 막는다)

**퍼블리셔가 이 클래스의 생성자에서만 만들어진다.** `control:=false` 면 이 객체를
아예 안 만들고, 그러면 `/arm/teleop_jog`·`/arm/teleop_cmd` 퍼블리셔가 **존재 자체를
하지 않는다.** 노드 코드 안에 "제어 모드면 건너뛰기" 같은 분기를 두지 않는 이유는
이 저장소가 이미 정한 원칙이다 — 안전 게이트에 스킵 분기가 있으면 실기에서 켜진 채
도는 사고가 난다(벤치 텔레옵 격리를 launch 파일로만 하는 것과 같은 사상).

## 발행하는 것과 발행하지 않는 것

발행: `/arm/teleop_jog`(JointJog) · `/arm/teleop_cmd`(String) — 계약상 owner 가 없다.
발행 안 함: `/arm_status`·`/chassis_mode`·`/arrival_status`(계약 owner 존재) 와
`/dynamixel/goal_position`(계약이 금지하는 direct goal publisher). GUI 는
`teleop_core` 를 거쳐서만 서보에 닿는다 — 기존 벤치 텔레옵과 정확히 같은 위치다.

## 스레드

HTTP 스레드는 `CommandBus` 에 의도만 적는다. 여기 `tick()` 은 **실행기 스레드**의
타이머에서만 불리며, rclpy 엔티티(퍼블리셔·서비스 클라이언트)는 전부 그 안에서만
만진다.
"""

import time

from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically

from control_msgs.msg import JointJog
from std_msgs.msg import String

from . import teleop_vocab
from .command_bus import CommandBus

#: teleop_core 기본값과 같은 안전 상한. 파라미터 조회에 성공하면 그 값으로 대체된다.
DEFAULT_MAX_VEL_RAD_S = 1.0

#: 키보드 프론트엔드의 조그 속도 기본값(`keyboard_teleop_node` 의 같은 이름 파라미터).
DEFAULT_JOG_VELOCITY_RAD_S = 0.7
DEFAULT_JOG_VELOCITY_DELTA = 0.2


def gamepad_defaults():
    """게임패드 기본 매핑 — `joystick_teleop_node` 의 실측값을 **import 해서** 쓴다.

    값을 프론트엔드에 복사해 두면 그쪽이 패드를 다시 재서 상수를 고쳤을 때 조용히
    갈라진다(`contract.py` 를 import 하는 것과 같은 이유). 그 모듈은 import 만으로는
    아무 부작용이 없다 — 상수와 클래스 정의뿐이다.

    ⚠️ **브라우저 Gamepad API 의 인덱스가 저쪽 joydev 인덱스와 같다는 보장은 없다.**
    스틱(0/1=왼쪽 XY, 2/3=오른쪽 XY)은 표준 매핑이 저쪽 실측 배치와 우연히 일치하지만
    버튼은 드라이버/브라우저마다 다르다. 그래서 화면이 축·버튼 실시간 값을 그대로
    띄우고 데드맨을 눌러서 다시 바인딩할 수 있게 한다 — 여기 값은 **출발점**이다.
    """
    try:
        from dynamixel_control.joystick_teleop_node import (
            DEFAULT_AXIS_IDS, DEFAULT_AXIS_INVERTED, DEFAULT_AXIS_SCALES,
            DEFAULT_BUTTON_MINUS_IDS, DEFAULT_BUTTON_PLUS_IDS, DEFAULT_JOINT_NAMES,
        )
    except ImportError as exc:                                # pragma: no cover
        return {'joints': [], 'reason': f'joystick_teleop 기본값을 못 읽었다: {exc}'}

    joints = [
        {
            'name': name,
            'axis': DEFAULT_AXIS_IDS[i],
            'scale': DEFAULT_AXIS_SCALES[i],
            'inverted': DEFAULT_AXIS_INVERTED[i],
            'button_plus': DEFAULT_BUTTON_PLUS_IDS[i],
            'button_minus': DEFAULT_BUTTON_MINUS_IDS[i],
        }
        for i, name in enumerate(DEFAULT_JOINT_NAMES)
    ]
    return {
        'joints': joints,
        # 이 둘만은 그쪽 declare_parameter 기본값이라 import 할 상수가 없다
        # (joystick_teleop_node 의 `deadzone` · `deadman_button`).
        'deadzone': 0.15,
        'deadman_button': 9,
    }


class ControlPlane:
    """노드가 제어 모드일 때만 생성하는 쓰기 경로."""

    def __init__(self, node, *, joint_names, publish_hz=20.0,
                 token_ttl_s=5.0, intent_timeout_s=0.3):
        self.node = node
        self.bus = CommandBus(token_ttl_s=token_ttl_s, intent_timeout_s=intent_timeout_s)
        self.joint_names = list(joint_names)
        self.max_vel_rad_s = DEFAULT_MAX_VEL_RAD_S
        self.jog_step_rad = 0.05
        self.publish_hz = float(publish_hz)
        self.intent_timeout_s = float(intent_timeout_s)

        # ⚠️ 이 두 줄이 읽기 전용/제어 모드를 가르는 실제 경계다.
        self.pub_jog = node.create_publisher(JointJog, '/arm/teleop_jog', 10)
        self.pub_cmd = node.create_publisher(String, '/arm/teleop_cmd', 10)

        self._set_param_clients = {}
        self._task_handlers = {}
        self._info_sources = {}
        self._model_source = None
        self._last_publish_state = 'idle'

        period = 1.0 / max(float(publish_hz), 1.0)
        self._timer = node.create_timer(period, self.tick)

    # ------------------------------------------------------------ 타이머
    def tick(self):
        """실행기 스레드 — 버스에 쌓인 의도를 실제 ROS 동작으로 옮긴다."""
        now = time.monotonic()
        state, velocities = self.bus.take_jog(now)
        if state == 'active':
            self._publish_jog(velocities)
        elif state == 'stop':
            # 정확히 한 번 0 을 쏘고 멈춘다. 계속 쏘면 teleop_core 의
            # deadman(0.5초 무입력 → 적분 정지)이 영원히 발동하지 못한다.
            self._publish_jog({})
            # 발행은 같지만 기록은 다르다 — 놓은 것과 끊긴 것을 섞으면 진짜
            # 두절 신호가 정상 조작에 묻힌다.
            if self.bus.last_stop_reason() == 'released':
                self.node.store.add_event('teleop', '조그 해제 — 전 관절 0 발행',
                                          'info', now)
            else:
                self.node.get_logger().warn(
                    '텔레옵 워치독: 조그 의도가 끊겨 전 관절 0 을 발행했다')
                self.node.store.add_event('teleop', '워치독 정지 — 조그 의도 끊김',
                                          'warning', now)
        self._last_publish_state = state

        for cmd in self.bus.drain_cmds():
            self.pub_cmd.publish(String(data=cmd))
            self.node.get_logger().info(f'teleop_cmd 발행: {cmd}')
            self.node.store.add_event('teleop', f'명령 발행: {cmd}', 'info', now)

        for task in self.bus.drain_tasks():
            self._run_task(task, now)

        # 화면의 '조종 중/관전 중'과 워치독 잔여 시간은 1Hz 로는 이미 늦다 —
        # 이 타이머 주기(기본 20Hz)로 store 에 밀어 넣어 hot 스냅샷에 실리게 한다.
        self.node.store.set_control(self.bus.snapshot(time.monotonic()))

    def _publish_jog(self, velocities):
        """⚠️ **전 관절을 매번 실어야 한다.**

        `teleop_core.on_jog` 는 메시지에 없는 관절의 속도를 0 으로 만들지 않는다.
        움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다** —
        실제로 있었던 버그라 그쪽 모듈 docstring 에도 못박혀 있다.
        """
        limit = self.max_vel_rad_s
        msg = JointJog()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.joint_names = list(self.joint_names)
        msg.velocities = [
            max(-limit, min(limit, float(velocities.get(name, 0.0))))
            for name in self.joint_names
        ]
        self.pub_jog.publish(msg)

    # ------------------------------------------------------------ 세션 훅
    def jog_publisher_conflict(self):
        """다른 노드가 `/arm/teleop_jog` 를 발행 중이면 그 사유, 아니면 None.

        `keyboard_teleop`/`joystick_teleop` 가 같이 떠 있으면 두 속도원이 서로를
        덮어써서 **어느 쪽도 명령한 대로 움직이지 않는다.** 발행자 목록은 노드의
        2Hz 조정 타이머가 이미 채워 두고 있어(`_reconcile_publishers`) 여기서는
        읽기만 한다.

        ⚠️ `fake_publisher` 도 이 토픽을 발행하므로 fake 검증에서도 걸린다 —
        여기에 "가짜면 예외" 분기를 넣지 않는다(안전 게이트의 스킵 분기 금지).
        뚫어야 하면 강제 획득으로 뚫고, 그 사실이 이벤트에 남는다.
        """
        mine = self.node.get_name()
        others = [n for n in self.node.store.teleop_jog_publishers()
                  if n.lstrip('/') != mine]
        if not others:
            return None
        return ('다른 텔레옵 프론트엔드가 /arm/teleop_jog 를 발행 중입니다 '
                f'({", ".join(sorted(others))}) — 두 속도원이 겹치면 어느 쪽도 '
                '명령대로 움직이지 않습니다. 그 노드를 내리거나 강제로 획득하세요.')

    def on_claim(self, label, force, conflict=None):
        note = f'조종권 획득: {label}' + (' (강제)' if force else '')
        if force and conflict:
            note += f' — 발행자 충돌을 무시함: {conflict}'
        self.node.get_logger().info(note)
        self.node.store.add_event('control', note,
                                  'warning' if force else 'info', time.monotonic())

    def on_release(self):
        self.node.get_logger().info('조종권 반납')
        self.node.store.add_event('control', '조종권 반납', 'info', time.monotonic())

    # ------------------------------------------------------------ 작업
    def register_task(self, kind, validator, runner):
        """단계별 기능이 자기 작업 종류를 여기에 등록한다.

        `validator(payload) -> (정규화된 payload, None)` 또는 `(None, 사유)`
        `runner(payload) -> (state, detail)`
            state: `'done'` | `'error'` | `'async'`.
            `'async'` 는 "결과가 나중에 콜백으로 온다"는 뜻이며, 그때는 runner 가
            `bus.finish_task` 를 직접 부를 책임을 진다(`payload['_task_id']` 에
            작업 번호가 들어 있다).
        """
        self._task_handlers[kind] = (validator, runner)

    def register_model_source(self, fn):
        self._model_source = fn

    def register_info(self, key, fn):
        """단계별 기능이 `describe()` 에 실을 정보를 등록한다(화면이 UI 를 그릴 근거)."""
        self._info_sources[key] = fn

    def validate_task(self, kind, payload):
        handler = self._task_handlers.get(kind)
        if handler is None:
            known = sorted(self._task_handlers) or ['(없음)']
            return None, f'알 수 없는 작업: {kind!r} (가능: {", ".join(known)})'
        return handler[0](payload)

    def _run_task(self, task, now):
        handler = self._task_handlers.get(task['kind'])
        if handler is None:
            self.bus.finish_task(task['id'], 'error', '핸들러가 사라졌습니다', now)
            return
        self.bus.finish_task(task['id'], 'running', '', now)
        payload = task['payload']
        payload['_task_id'] = task['id']
        try:
            state, detail = handler[1](payload)
        except Exception as exc:                              # noqa: BLE001
            state, detail = 'error', f'{type(exc).__name__}: {exc}'
            self.node.get_logger().error(f'작업 실패 ({task["kind"]}): {detail}')
        if state != 'async':
            self.bus.finish_task(task['id'], state, detail, time.monotonic())
        self.node.store.add_event(
            'task', f'{task["kind"]}: {detail or state}',
            'critical' if state == 'error' else 'info', time.monotonic())

    # ------------------------------------------------------------ 파라미터 쓰기
    def set_remote_params(self, node_name, values, done=None):
        """다른 노드의 파라미터를 **원자적으로** 설정한다.

        ⚠️ `set_parameters`(비원자) 를 쓰면 안 된다 — 그쪽은 파라미터를 **한 개씩**
        대상 노드의 콜백에 넘긴다. 서로 의존하는 값들(`model_name` + `model_path` 같은)이
        따로 도착하면 각각이 불완전한 상태로 검증돼 거절되거나, 더 나쁘게는 절반만
        적용된다. 실제로 모델 교체가 "경로는 바뀌었는데 이름은 안 바뀐" 상태로
        끝나는 것을 이 방식으로 확인했다.

        ⚠️ `spin_until_future_complete` 도 쓰지 않는다 — 타이머 콜백 안에서 결과를
        기다리며 spin 하면 재진입으로 데드락이 난다(`arm_fsm` 의 FK 클라이언트가
        같은 이유로 별도 노드를 쓴다). 결과는 done 콜백으로 받는다.

        `values`: `{이름: 파이썬 값}`. bool/int/float/str/list 를 지원한다.
        """
        client = self._set_param_clients.get(node_name)
        if client is None:
            client = self.node.create_client(
                SetParametersAtomically, f'/{node_name}/set_parameters_atomically')
            self._set_param_clients[node_name] = client
        if not client.service_is_ready():
            return False, f'{node_name} 의 파라미터 서비스가 준비되지 않았습니다'

        params = []
        for name, value in values.items():
            converted = _to_parameter_value(value)
            if converted is None:
                return False, f'{name}: 지원하지 않는 파라미터 타입 {type(value).__name__}'
            params.append(Parameter(name=name, value=converted))

        future = client.call_async(
            SetParametersAtomically.Request(parameters=params))
        if done is not None:
            future.add_done_callback(done)
        return True, None

    # ------------------------------------------------------------ HTTP 쪽 인터페이스
    def describe(self):
        """화면이 어떤 제어가 가능한지 판단할 근거.

        어휘·기본값을 프론트엔드에 복사해 두면 실제 게이트와 언젠가 어긋난다 —
        `teleop_vocab` 과 `joystick_teleop` 의 값을 그대로 실어 보내고 화면은
        읽기만 한다(계약 상수를 `contract.py` 에서 실어 보내는 것과 같은 사상).
        """
        return {
            'joint_names': list(self.joint_names),
            'max_vel_rad_s': self.max_vel_rad_s,
            'jog_step_rad': self.jog_step_rad,
            'task_kinds': sorted(self._task_handlers),
            'commands': teleop_vocab.all_commands(),
            'publish_hz': self.publish_hz,
            'intent_timeout_s': self.intent_timeout_s,
            'keyboard': {
                'jog_velocity_rad_s': DEFAULT_JOG_VELOCITY_RAD_S,
                'jog_velocity_delta': DEFAULT_JOG_VELOCITY_DELTA,
            },
            'gamepad': gamepad_defaults(),
            'jog_conflict': self.jog_publisher_conflict(),
            **{key: fn() for key, fn in self._info_sources.items()},
        }

    def list_models(self):
        if self._model_source is None:
            return {'models': [], 'reason': '모델 제어가 등록되지 않았습니다'}
        return self._model_source()

    def cloud_frame(self, since_seq):
        return None


def _to_parameter_value(value):
    """파이썬 값 → `rcl_interfaces/ParameterValue` (지원 안 하면 None)."""
    if isinstance(value, bool):
        return ParameterValue(type=ParameterType.PARAMETER_BOOL, bool_value=value)
    if isinstance(value, int):
        return ParameterValue(type=ParameterType.PARAMETER_INTEGER,
                              integer_value=int(value))
    if isinstance(value, float):
        return ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                              double_value=float(value))
    if isinstance(value, str):
        return ParameterValue(type=ParameterType.PARAMETER_STRING, string_value=value)
    if isinstance(value, (list, tuple)):
        items = list(value)
        if all(isinstance(v, str) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_STRING_ARRAY,
                                  string_array_value=items)
        if all(isinstance(v, bool) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_BOOL_ARRAY,
                                  bool_array_value=items)
        if all(isinstance(v, int) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_INTEGER_ARRAY,
                                  integer_array_value=items)
        if all(isinstance(v, (int, float)) for v in items):
            return ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                  double_array_value=[float(v) for v in items])
    return None
