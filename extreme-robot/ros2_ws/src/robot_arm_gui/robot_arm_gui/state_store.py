#!/usr/bin/env python3
"""관제 GUI 의 상태 저장소 (ROS 비의존).

ROS 콜백과 HTTP 스레드가 만나는 **유일한 지점**이다. 설계 원칙 두 가지:

1. **콜백은 O(1) 쓰기만 한다.** 최신값 dict 과 고정 길이 링버퍼에만 쓰고,
   가공(다운샘플·집계·직렬화)은 전부 읽는 쪽(`snapshot`)에서 한다.
2. **큐를 클라이언트별로 만들지 않는다.** SSE 핸들러가 자기 속도로 `snapshot()`
   을 떠 가는 "스냅샷 폴" 모델이라, 느린 클라이언트는 자기 스레드에서만 막히고
   프레임을 건너뛴다. 텔레메트리에서는 그게 옳은 동작이고, 백프레셔가
   구조적으로 발생하지 않는다.

시간은 전부 **단조시계(monotonic) 초** 를 인자로 받는다 — 테스트에서 시간을
마음대로 밀 수 있어야 하기 때문이다. 화면에 찍을 벽시계는 `wall_fn` 으로 주입한다.
"""

import threading
from collections import deque

from .calib_status_parse import parse as parse_calib_status
from .calib_status_parse import summary as summarize_calib_status
from .topic_health import classify_joint_publishers


#: XL430 Present Current 1 LSB [mA] (데이터시트).
CURRENT_LSB_MA = 2.69

#: 스파크라인 버킷 길이 [s] 와 개수 → 0.5 × 120 = 60초 창.
DEFAULT_BUCKET_S = 0.5
DEFAULT_SPARK_BUCKETS = 120

#: `dynamixel_position_node.CURRENT_SPIKE_WINDOW` 와 **같은 값이어야 한다.**
#: 여기서 노드의 급변 판정을 재현하므로 창 길이가 다르면 여유 표시가 틀어진다.
DEFAULT_SPIKE_WINDOW = 90

#: 노드 기본 임계값(`DEFAULT_CURRENT_TRIP_THRESHOLD` / `DEFAULT_CURRENT_SPIKE_DELTA`).
#: 어디까지나 **기동 기본값 추정치**다 — 출처를 'startup' 으로 표기해 구분한다.
DEFAULT_TRIP_THRESHOLD = 500
DEFAULT_SPIKE_THRESHOLD = 350


def _clip_ratio(value, limit):
    """0 나눗셈 없이 비율을 낸다. limit 이 없거나 0이면 None."""
    if not limit or limit <= 0 or value is None:
        return None
    return value / float(limit)


class StateStore:
    """스레드 안전 최신값 저장소 + 링버퍼 + 트립 블랙박스."""

    def __init__(self, *, wall_fn=None, spike_window=DEFAULT_SPIKE_WINDOW,
                 state_rate_hz=30.0, spark_buckets=DEFAULT_SPARK_BUCKETS,
                 bucket_s=DEFAULT_BUCKET_S, trace_seconds=3.0,
                 max_traces=5, max_events=200):
        if wall_fn is None:
            import time
            wall_fn = time.time
        self._wall_fn = wall_fn
        self._lock = threading.RLock()

        self._spike_window = int(spike_window)
        self._spark_buckets = int(spark_buckets)
        self._bucket_s = float(bucket_s)

        # 트립 블랙박스용 전 모터 raw 링버퍼. 3초 × 30Hz = 90 프레임.
        self._trace_len = max(1, int(state_rate_hz * trace_seconds))
        self._raw_ring = deque(maxlen=self._trace_len)
        self._traces = deque(maxlen=int(max_traces))
        self._trace_seq = 0

        self._events = deque(maxlen=int(max_events))
        self._event_seq = 0

        self._motors = {}          # dxl_id -> dict
        self._motor_order = []     # 첫 등장 순서 유지 (id 정렬은 의미가 없다)
        self._topics = {}          # topic name -> {'count', 'last'}

        self._hw_entries = []
        self._hw_ids = frozenset()

        self._thresholds = {
            'trip': {'value': DEFAULT_TRIP_THRESHOLD, 'enabled': True,
                     'source': 'startup', 'at': None},
            'spike': {'value': DEFAULT_SPIKE_THRESHOLD, 'enabled': True,
                      'source': 'startup', 'at': None},
        }

        self._arm = {'status': None, 'mission_id': None, 'at': None, 'stamp_age': None}
        self._chassis = {'mode': None, 'at': None}
        self._arrival = {'status': None, 'mission_id': None, 'at': None}
        self._controller_fault = {'value': None, 'at': None}

        self._joints = {}          # name -> {'position','velocity','effort','at'}
        self._joint_publishers = []
        self._goals = {}           # dxl_id -> {'tick', 'at'}
        self._tick_limits = {'map': {}, 'received': False, 'empty': None, 'at': None}

        self._detections = {'objects': [], 'at': None, 'hz': None}
        self._detect_times = deque(maxlen=30)
        self._pick_target = None
        #: perception_node 의 모델 로드 결과(/perception/model_status).
        self._model_status = None

        #: 캘리브 마법사의 마지막 계산 결과(제어 모드에서만 채워진다).
        self._calib_result = None

        self._teleop = {
            'jog_at': None, 'jog_joints': [], 'jog_velocities': [],
            'jog_publishers': [], 'last_cmd': None, 'last_cmd_at': None,
            'poses': [], 'calib': None, 'calib_info': None,
        }
        self._joy = {'at': None, 'buttons': [], 'axes': []}
        self._joy_params = {'deadman_button': None, 'turbo_button': None,
                            'estop_button': None, 'resolved': False}

        self._video = {'source': 'off', 'clients': 0, 'fps': None, 'at': None}
        #: 제어 세션(조종권·워치독). 읽기 전용 모드에서는 영원히 None 이고,
        #: 화면은 그걸로 '제어 미탑재'를 판정한다.
        self._control = None
        self._system = {}
        # 계약 어휘(LOCK_MODES 등)와 표시 임계. 노드가 기동 시 한 번 심어두면
        # 프론트엔드가 "작업 허가/잠금" 같은 파생 판정을 자기 쪽에서 할 수 있다.
        self._contract = {}

    # ------------------------------------------------------------ 공통 유틸
    def _note(self, topic, now):
        """토픽 수신 기록. 신선도 판정과 '노드가 살아있나' 표시에 쓴다."""
        slot = self._topics.get(topic)
        if slot is None:
            slot = {'count': 0, 'last': None}
            self._topics[topic] = slot
        slot['count'] += 1
        slot['last'] = now

    def _add_event(self, kind, text, severity, now, trace=None):
        self._event_seq += 1
        self._events.append({
            'seq': self._event_seq,
            'wall': self._wall_fn(),
            'mono': now,
            'kind': kind,
            'text': text,
            'severity': severity,
            'trace': trace,
        })

    def add_event(self, kind, text, severity, now, trace=None):
        with self._lock:
            self._add_event(kind, text, severity, now, trace)

    # ------------------------------------------------------------ 모터
    def _motor(self, dxl_id):
        m = self._motors.get(dxl_id)
        if m is None:
            m = {
                'id': dxl_id, 'joint': None,
                'tick': None, 'velocity': None, 'current': None,
                'temp': 0, 'temp_known': False, 'temp_change_at': None,
                'spike_window': deque(maxlen=self._spike_window),
                'spike_baseline': None, 'spike_delta': None,
                'spark': deque(maxlen=self._spark_buckets),
                'bucket': None, 'bucket_idx': None,
                'at': None,
            }
            self._motors[dxl_id] = m
            self._motor_order.append(dxl_id)
        return m

    def _push_spark(self, m, value, now):
        """0.5초 버킷에 min/max/last 로 누적. O(1) — 읽을 때 재계산하지 않는다."""
        idx = int(now / self._bucket_s)
        if m['bucket_idx'] is None:
            m['bucket_idx'] = idx
            m['bucket'] = [value, value, value]
            return
        if idx == m['bucket_idx']:
            b = m['bucket']
            if b is None:
                m['bucket'] = [value, value, value]
            else:
                b[0] = min(b[0], value)
                b[1] = max(b[1], value)
                b[2] = value
            return
        if m['bucket'] is not None:
            m['spark'].append(m['bucket'])
        # 수신이 끊겼던 구간은 None 으로 채워 "값 없음"과 "0"을 구분한다.
        gap = idx - m['bucket_idx'] - 1
        for _ in range(max(0, min(gap, self._spark_buckets))):
            m['spark'].append(None)
        m['bucket_idx'] = idx
        m['bucket'] = [value, value, value]

    def update_motors(self, samples, now):
        """`/dynamixel/state` 한 건 반영.

        samples: `[(dxl_id, tick, velocity_raw, current_raw, temp_c), ...]`
        — `/dynamixel/state` 는 모터당 5개 int32 가 연속 반복되는 배열이고,
        **인덱스가 아니라 각 그룹 첫 값의 ID 로 매핑해야 한다**(모터가 ping 에
        실패하면 그 자리가 통째로 빠진다).
        """
        with self._lock:
            self._note('/dynamixel/state', now)
            spike_cfg = self._thresholds['spike']
            frame = []
            for dxl_id, tick, velocity, current, temp in samples:
                m = self._motor(int(dxl_id))
                m['tick'] = int(tick)
                m['velocity'] = int(velocity)
                m['current'] = int(current)
                m['at'] = now

                temp = int(temp)
                # ⚠️ 온도 0 은 "0°C" 가 아니라 **아직 폴링이 안 닿았다**는 뜻이다.
                #    노드가 라운드로빈 2Hz 로 한 모터씩만 읽고, 첫 도달 전에는
                #    dict 기본값 0 을 그대로 실어 보낸다.
                if temp != m['temp']:
                    m['temp_change_at'] = now
                m['temp'] = temp
                if temp:
                    m['temp_known'] = True

                a = abs(m['current'])
                self._push_spark(m, a, now)

                # 노드(`read_state`)의 급변 판정을 **순서까지** 그대로 재현한다:
                # baseline 은 append **전**의 창 최솟값이다. 순서를 바꾸면
                # 항상 자기 자신이 최솟값 후보가 되어 여유가 과대평가된다.
                if spike_cfg['enabled']:
                    win = m['spike_window']
                    baseline = min(win) if win else None
                    win.append(a)
                    m['spike_baseline'] = baseline
                    m['spike_delta'] = (a - baseline) if baseline is not None else None
                else:
                    m['spike_window'].clear()
                    m['spike_baseline'] = None
                    m['spike_delta'] = None

                frame.append([m['id'], m['tick'], m['velocity'], m['current'], temp])

            self._raw_ring.append({'mono': now, 'wall': self._wall_fn(), 'motors': frame})

    def set_motor_names(self, mapping):
        """dxl_id → 관절 이름. 노드 파라미터에서 받아 한 번만 채운다."""
        with self._lock:
            for dxl_id, name in mapping.items():
                self._motor(int(dxl_id))['joint'] = name

    # ------------------------------------------------------------ HW 에러
    def set_hardware_error(self, entries, now):
        """파싱된 hw 에러 목록 반영 + **엣지만** 이벤트화.

        노드가 이 토픽을 30Hz 로 무조건 발행하므로(에러가 없으면 빈 문자열),
        매 메시지를 이벤트로 남기면 초당 30줄이 쌓인다. 이전 latch 집합과
        비교해 새로 생긴 것/풀린 것만 기록한다.

        상승 엣지에서는 직전 `trace_seconds` 의 전 모터 raw 트레이스를
        **동결**한다 — 트립 당시 수치는 노드에서 로그로만 남고 사라지기 때문이다.
        """
        from .hw_error_parse import latched_ids, summarize

        with self._lock:
            self._note('/dynamixel/hardware_error', now)
            new_ids = latched_ids(entries)
            appeared = new_ids - self._hw_ids
            cleared = self._hw_ids - new_ids
            self._hw_entries = entries
            self._hw_ids = new_ids

            trace_id = None
            if appeared:
                trace_id = self._freeze_trace(
                    f'hw_error {summarize([e for e in entries if e["dxl_id"] in appeared])}',
                    now)
                for e in entries:
                    if e['dxl_id'] in appeared:
                        label = ' | '.join(e['labels']) or '?'
                        sev = 'critical' if (e['soft_spike'] or e['soft_trip']) else 'serious'
                        self._add_event(
                            'hw_error',
                            f"{e['joint'] or '?'}(ID{e['dxl_id']}) {label} — reboot 전까지 latch",
                            sev, now, trace=trace_id)
            for dxl_id in sorted(cleared):
                self._add_event('hw_error_clear', f'ID{dxl_id} 에러 해제', 'good', now)
            return trace_id

    def _freeze_trace(self, reason, now):
        self._trace_seq += 1
        trace_id = self._trace_seq
        self._traces.append({
            'id': trace_id,
            'reason': reason,
            'wall': self._wall_fn(),
            'mono': now,
            'thresholds': {k: dict(v) for k, v in self._thresholds.items()},
            'samples': list(self._raw_ring),
        })
        return trace_id

    def get_trace(self, trace_id):
        with self._lock:
            for t in self._traces:
                if t['id'] == int(trace_id):
                    return t
            return None

    def list_traces(self):
        with self._lock:
            return [{'id': t['id'], 'reason': t['reason'], 'wall': t['wall'],
                     'samples': len(t['samples'])} for t in self._traces]

    # ------------------------------------------------------------ 임계값
    def set_threshold(self, kind, enabled, value, now, source='runtime'):
        """`/dynamixel/current_{trip,spike}_config` 반영.

        ⚠️ 이 토픽들은 latched 가 아니라 depth 10 volatile 이라, GUI 가 나중에
        붙으면 **이전 변경을 놓친다.** 그래서 화면에는 값과 함께 출처를
        ('기동값' vs '런타임 변경 hh:mm:ss') 반드시 같이 찍는다.
        """
        with self._lock:
            slot = self._thresholds[kind]
            slot['enabled'] = bool(enabled)
            if value is not None and value > 0:
                slot['value'] = int(value)
            slot['source'] = source
            slot['at'] = self._wall_fn() if source == 'runtime' else None
            if source == 'runtime':
                self._note(f'/dynamixel/current_{kind}_config', now)
                state = 'on' if slot['enabled'] else 'off'
                self._add_event('threshold',
                                f"{kind} 임계값 {slot['value']} ({state})", 'info', now)

    # ------------------------------------------------------------ 계약/FSM
    def joint_positions(self, max_age_s=None, now=None):
        """`{관절: rad}` — 캘리브 마법사가 "지금 값"을 캡처할 때 쓴다.

        `max_age_s` 를 주면 그보다 낡은 관절은 뺀다. 캘리브에서 낡은 값을 캡처하면
        **손으로 이미 움직인 뒤의 자세를 그 전 값으로 기록**하게 된다.
        """
        with self._lock:
            out = {}
            for name, slot in self._joints.items():
                if slot['position'] is None:
                    continue
                if max_age_s is not None and (
                        slot['at'] is None or now is None
                        or now - slot['at'] > max_age_s):
                    continue
                out[name] = slot['position']
            return out

    def set_arm_status(self, status, mission_id, now, stamp_age=None):
        with self._lock:
            self._note('/arm_status', now)
            prev = self._arm['status']
            self._arm.update({'status': status, 'mission_id': mission_id,
                              'at': now, 'stamp_age': stamp_age})
            if prev != status:
                self._add_event('arm_status', f'{prev or "—"} → {status}', 'info', now)

    def set_chassis_mode(self, mode, now):
        with self._lock:
            self._note('/chassis_mode', now)
            prev = self._chassis['mode']
            self._chassis.update({'mode': mode, 'at': now})
            if prev != mode:
                self._add_event('chassis_mode', f'{prev or "—"} → {mode}', 'info', now)

    def set_arrival(self, status, mission_id, now):
        with self._lock:
            self._note('/arrival_status', now)
            self._arrival.update({'status': status, 'mission_id': mission_id, 'at': now})
            self._add_event('arrival', f'{status} (mission {mission_id})', 'info', now)

    def set_controller_fault(self, value, now):
        with self._lock:
            self._note('/dynamixel/controller_fault', now)
            prev = self._controller_fault['value']
            self._controller_fault.update({'value': bool(value), 'at': now})
            if prev is not None and prev != bool(value):
                self._add_event('controller_fault',
                                'fault=True (어느 관절인지는 이 토픽에 없음)'
                                if value else 'fault 해제',
                                'serious' if value else 'good', now)

    # ------------------------------------------------------------ 관절
    def set_joint_states(self, names, positions, velocities, efforts, now):
        with self._lock:
            self._note('/joint_states', now)
            for i, name in enumerate(names):
                slot = self._joints.setdefault(
                    name, {'position': None, 'velocity': None, 'effort': None, 'at': None})
                if i < len(positions):
                    slot['position'] = float(positions[i])
                if i < len(velocities):
                    slot['velocity'] = float(velocities[i])
                if i < len(efforts):
                    slot['effort'] = float(efforts[i])
                slot['at'] = now

    def set_joint_publishers(self, publishers):
        """`/joint_states` 발행자 노드명 목록.

        발행자에 따라 **값의 도메인이 다르다** — position_node 는 기어비 미적용
        서보축 raw 근사, bridge 는 실제 관절 rad, teleop_core 는 sim 되먹임.
        도메인을 모르면 각도를 숫자로 그리면 안 되므로 화면에 항상 같이 찍는다.
        """
        with self._lock:
            self._joint_publishers = list(publishers)

    def set_goal(self, dxl_id, tick, now):
        with self._lock:
            self._note('/dynamixel/goal_position', now)
            self._goals[int(dxl_id)] = {'tick': int(tick), 'at': now}

    def set_tick_limits(self, data, now):
        """`[id, min, max, ...]`. **빈 배열은 '리밋 OFF'** 라는 뜻이다."""
        with self._lock:
            self._note('/dynamixel/tick_limits', now)
            mapping = {}
            for i in range(0, len(data) - 2, 3):
                mapping[int(data[i])] = (int(data[i + 1]), int(data[i + 2]))
            empty = len(data) == 0
            prev_empty = self._tick_limits['empty']
            self._tick_limits.update({'map': mapping, 'received': True,
                                      'empty': empty, 'at': now})
            if prev_empty != empty:
                self._add_event('tick_limits',
                                '소프트 리밋 OFF' if empty else f'리밋 {len(mapping)}축 적용',
                                'serious' if empty else 'good', now)

    # ------------------------------------------------------------ 인식
    def set_detections(self, objects, now):
        with self._lock:
            self._note('/detected_objects', now)
            self._detect_times.append(now)
            self._detections = {'objects': objects, 'at': now}

    def set_pick_target(self, obj, now):
        with self._lock:
            self._note('/pick_target', now)
            self._pick_target = dict(obj, at=now)
            self._add_event('pick_target',
                            f"{obj.get('class_name')} conf={obj.get('confidence'):.2f}",
                            'info', now)

    # ------------------------------------------------------------ 텔레옵
    def set_teleop_jog(self, joint_names, velocities, now):
        with self._lock:
            self._note('/arm/teleop_jog', now)
            self._teleop['jog_at'] = now
            self._teleop['jog_joints'] = list(joint_names)
            self._teleop['jog_velocities'] = [float(v) for v in velocities]

    def set_teleop_publishers(self, publishers):
        with self._lock:
            self._teleop['jog_publishers'] = list(publishers)

    def teleop_jog_publishers(self):
        """`/arm/teleop_jog` 를 발행 중인 노드 이름 — 제어 획득 전 충돌 검사용."""
        with self._lock:
            return list(self._teleop['jog_publishers'])

    def set_teleop_cmd(self, cmd, now):
        with self._lock:
            self._note('/arm/teleop_cmd', now)
            self._teleop['last_cmd'] = cmd
            self._teleop['last_cmd_at'] = now
            self._add_event('teleop_cmd', cmd, 'info', now)

    def set_poses(self, names, now):
        with self._lock:
            self._note('/arm/teleop_poses', now)
            self._teleop['poses'] = list(names)

    def set_calib_status(self, text, now):
        """원문과 파싱 결과를 함께 담는다.

        원문을 버리지 않는 이유: 형식이 바뀌어 파서가 `unknown` 을 내도 운영자가
        무엇이 왔는지 볼 수 있어야 한다(→ `calib_status_parse`).
        """
        with self._lock:
            self._note('/arm/calib_status', now)
            prev = self._teleop['calib']
            self._teleop['calib'] = text
            self._teleop['calib_info'] = parse_calib_status(text)
            if prev != text:
                self._add_event('calib', summarize_calib_status(
                    self._teleop['calib_info']), 'info', now)

    def set_joy(self, buttons, axes, now):
        with self._lock:
            self._note('/joy', now)
            self._joy = {'at': now, 'buttons': [int(b) for b in buttons],
                         'axes': [round(float(a), 3) for a in axes]}

    def set_joy_params(self, deadman, turbo, estop, resolved):
        """`joystick_teleop` 의 버튼 인덱스 파라미터(읽기 전용 조회 결과).

        데드맨은 `/joy`.buttons 의 순수 함수라 **인덱스만 알면 관측 가능**하다.
        노드가 안 떠 있으면 resolved=False 로 두고 화면에 '기본값 가정'을 붙인다.
        """
        with self._lock:
            self._joy_params = {'deadman_button': deadman, 'turbo_button': turbo,
                                'estop_button': estop, 'resolved': bool(resolved)}

    # ------------------------------------------------------------ 기타
    def set_video(self, source, clients, fps, now):
        with self._lock:
            self._video = {'source': source, 'clients': clients, 'fps': fps, 'at': now}

    def set_calib_result(self, info):
        """캘리브 마법사의 계산 결과(복사 블록 포함).

        작업 결과의 `detail` 문자열에는 한 줄 요약만 담고, 붙여넣을 블록과 축별 수치는
        여기 담아 SSE 로 내보낸다 — 결과가 나온 뒤 새로고침해도 사라지지 않아야 한다.
        """
        with self._lock:
            self._calib_result = None if info is None else dict(info)

    def set_model_status(self, info, now):
        """모델 교체 결과. 상태가 바뀔 때만 이벤트를 남긴다(로그 도배 방지)."""
        with self._lock:
            self._note('/perception/model_status', now)
            prev = self._model_status
            self._model_status = dict(info, at=now)
            key = (info.get('state'), info.get('name'), info.get('path'))
            prev_key = (None if prev is None else
                        (prev.get('state'), prev.get('name'), prev.get('path')))
            if key == prev_key:
                return
            state = info.get('state')
            if state == 'loaded':
                seconds = info.get('seconds')
                took = '' if seconds is None else f' ({seconds}초)'
                self._add_event('model', f'모델 로드: {info.get("name")}{took}',
                                'info', now)
            elif state == 'error':
                self._add_event('model',
                                f'모델 로드 실패: {info.get("name")} — '
                                f'{info.get("detail")}', 'critical', now)

    def set_control(self, info):
        """제어 평면의 세션 스냅샷. 제어 모드에서만 채워진다."""
        with self._lock:
            self._control = None if info is None else dict(info)

    def set_system(self, info):
        with self._lock:
            self._system = dict(info)

    def set_contract(self, info):
        """계약 상수·표시 임계. 기동 시 한 번만 심는다(2Hz 갱신에 덮이면 안 된다)."""
        with self._lock:
            self._contract = dict(info)

    # ------------------------------------------------------------ 읽기
    def _age(self, at, now):
        return None if at is None else max(0.0, now - at)

    def motors_snapshot(self, now):
        trip = self._thresholds['trip']
        spike = self._thresholds['spike']
        out = []
        for dxl_id in self._motor_order:
            m = self._motors[dxl_id]
            cur = m['current']
            a = None if cur is None else abs(cur)
            spark = [b for b in m['spark']]
            if m['bucket'] is not None:
                spark = spark + [m['bucket']]
            out.append({
                'id': m['id'],
                'joint': m['joint'],
                'tick': m['tick'],
                'velocity': m['velocity'],
                'current': cur,
                'current_ma': None if cur is None else round(cur * CURRENT_LSB_MA, 1),
                'trip_ratio': _clip_ratio(a, trip['value'] if trip['enabled'] else None),
                'trip_headroom': None if a is None or not trip['enabled']
                else trip['value'] - a,
                'spike_baseline': m['spike_baseline'],
                'spike_delta': m['spike_delta'],
                'spike_ratio': _clip_ratio(m['spike_delta'],
                                           spike['value'] if spike['enabled'] else None),
                # temp 0 = 미수신. 화면에서 절대 "0°C 정상"으로 그리지 말 것.
                'temp': m['temp'] if m['temp'] else None,
                'temp_known': m['temp_known'],
                'temp_change_age': self._age(m['temp_change_at'], now),
                'age': self._age(m['at'], now),
                'spark': spark,
                'goal_tick': self._goals.get(dxl_id, {}).get('tick'),
                'goal_error': (None if m['tick'] is None or dxl_id not in self._goals
                               else self._goals[dxl_id]['tick'] - m['tick']),
                'limits': list(self._tick_limits['map'].get(dxl_id, ())) or None,
                'hw_error': next((e for e in self._hw_entries if e['dxl_id'] == dxl_id), None),
            })
        return out

    def _detect_hz(self, now):
        ts = [t for t in self._detect_times if now - t <= 5.0]
        if len(ts) < 2:
            return None
        span = ts[-1] - ts[0]
        return None if span <= 0 else round((len(ts) - 1) / span, 1)

    def snapshot(self, now, *, include_spark=True, include_events=True):
        """HTTP 스레드가 자기 속도로 떠 가는 전체 스냅샷 (JSON 직렬화 가능)."""
        with self._lock:
            motors = self.motors_snapshot(now)
            if not include_spark:
                for m in motors:
                    m.pop('spark', None)

            deadman = self._joy_params['deadman_button']
            buttons = self._joy['buttons']
            deadman_held = None
            if deadman is not None and deadman >= 0 and self._joy['at'] is not None:
                deadman_held = bool(deadman < len(buttons) and buttons[deadman])

            snap = {
                'now': now,
                'wall': self._wall_fn(),
                'motors': motors,
                'hw_errors': self._hw_entries,
                'thresholds': {k: dict(v) for k, v in self._thresholds.items()},
                'arm': dict(self._arm, age=self._age(self._arm['at'], now)),
                'chassis': dict(self._chassis, age=self._age(self._chassis['at'], now)),
                'arrival': dict(self._arrival, age=self._age(self._arrival['at'], now)),
                'controller_fault': dict(
                    self._controller_fault,
                    age=self._age(self._controller_fault['at'], now)),
                'joints': {
                    name: dict(v, age=self._age(v['at'], now))
                    for name, v in self._joints.items()
                },
                'joint_publishers': list(self._joint_publishers),
                # 값의 단위가 발행자에 따라 다르다 — 화면이 각도를 그리기 전에 이걸 봐야 한다.
                'joint_domain': classify_joint_publishers(self._joint_publishers),
                'tick_limits': {
                    'received': self._tick_limits['received'],
                    'empty': self._tick_limits['empty'],
                    'count': len(self._tick_limits['map']),
                    'age': self._age(self._tick_limits['at'], now),
                },
                'detections': {
                    'objects': self._detections['objects'],
                    'age': self._age(self._detections['at'], now),
                    'hz': self._detect_hz(now),
                },
                'pick_target': (None if self._pick_target is None else dict(
                    self._pick_target, age=self._age(self._pick_target.get('at'), now))),
                'model': (None if self._model_status is None else dict(
                    self._model_status,
                    age=self._age(self._model_status.get('at'), now))),
                'calib_result': (None if self._calib_result is None
                                 else dict(self._calib_result)),
                'teleop': {
                    'jog_age': self._age(self._teleop['jog_at'], now),
                    'jog_joints': self._teleop['jog_joints'],
                    'jog_velocities': self._teleop['jog_velocities'],
                    'jog_publishers': self._teleop['jog_publishers'],
                    'last_cmd': self._teleop['last_cmd'],
                    'last_cmd_age': self._age(self._teleop['last_cmd_at'], now),
                    'poses': self._teleop['poses'],
                    'calib': self._teleop['calib'],
                    'calib_info': self._teleop['calib_info'],
                },
                'joy': {
                    'age': self._age(self._joy['at'], now),
                    'buttons': buttons,
                    'axes': self._joy['axes'],
                    'deadman_button': deadman,
                    'deadman_held': deadman_held,
                    'turbo_button': self._joy_params['turbo_button'],
                    'estop_button': self._joy_params['estop_button'],
                    'params_resolved': self._joy_params['resolved'],
                },
                'video': dict(self._video),
                'control': None if self._control is None else dict(self._control),
                'system': dict(self._system),
                'contract': dict(self._contract),
                'topics': {
                    name: {'count': slot['count'], 'age': self._age(slot['last'], now)}
                    for name, slot in self._topics.items()
                },
                'traces': [{'id': t['id'], 'reason': t['reason'], 'wall': t['wall'],
                            'samples': len(t['samples'])} for t in self._traces],
            }
            if include_events:
                snap['events'] = list(self._events)
            return snap

    def hot_snapshot(self, now):
        """5Hz 로 보내는 축약본 — 모터 수치와 상태 스트립만.

        ⚠️ **모터 표를 그리는 데 필요한 것은 여기 전부 들어 있어야 한다.**
        `thresholds` 를 빼뒀더니 화면이 5Hz 갱신마다 예외로 죽어 표가 멈췄다
        (전체 스냅샷은 1Hz 라 "가끔 되살아나는" 형태로 나타나 원인 찾기가 나쁘다).
        """
        with self._lock:
            snap = {
                'now': now,
                'motors': self.motors_snapshot(now),
                'thresholds': {k: dict(v) for k, v in self._thresholds.items()},
                'arm': dict(self._arm, age=self._age(self._arm['at'], now)),
                'chassis': dict(self._chassis, age=self._age(self._chassis['at'], now)),
                'controller_fault': dict(
                    self._controller_fault,
                    age=self._age(self._controller_fault['at'], now)),
                'hw_errors': self._hw_entries,
                # 워치독 잔여 시간은 1Hz 로 보면 이미 늦다 — 조종 중 표시는 hot 에 싣는다.
                'control': None if self._control is None else dict(self._control),
                'topics': {
                    name: {'count': slot['count'], 'age': self._age(slot['last'], now)}
                    for name, slot in self._topics.items()
                },
            }
            return snap

    def events_since(self, seq):
        with self._lock:
            return [e for e in self._events if e['seq'] > seq]
