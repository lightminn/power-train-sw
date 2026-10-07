#!/usr/bin/env python3
"""관절 캘리브 마법사 — `ControlPlane` 에 계산·적용 작업을 얹는다.

## 무엇을 대체하나

지금 영점·기어비·그리퍼 끝단은 `scripts/measure_*.py` 세 개를 각각 터미널에서 돌려야
하고, 결과는 **소스를 고쳐 재빌드해야** 반영된다. 여기서는 같은 측정을 브라우저에서
하고(`dynamixel_control.calib_math` 로 **같은 식**을 쓴다), 브릿지 파라미터로 그 자리에서
적용해 볼 수 있다.

가동범위(리밋)만은 여기 없다 — 그건 `teleop_core` 가 `calib_start`/`calib_mark` 로 이미
전체 절차를 갖고 있고 결과를 JSON 으로 자동 저장한다. GUI 는 그 명령을 보내고
`/arm/calib_status` 진행률을 그리기만 한다(→ `calib_status_parse`).

## 브릿지에 묻고 나서 계산한다

브릿지가 CLI 로 기어비·영점을 덮어쓴 채 떠 있으면 `JOINT_CONFIG` 기본값과 다르다.
그걸 모르고 역산하면 결과가 통째로 틀리므로 **매번 `get_parameters` 로 물어본다**
(`measure_zero_offset.py` 가 같은 이유로 같은 일을 한다). 결과를 기다리며 spin 하지
않고 done 콜백으로 받는다 — 타이머 콜백 안에서 spin 하면 재진입 데드락이다.

## 자동으로 소스를 고치지 않는다

결과는 **복사용 블록**까지만 만든다. `JOINT_CONFIG` 주석의 `confidence`/`source` 에
담긴 근거(언제 무엇으로 쟀는지)가 기계 기록으로 덮이면 사라지기 때문이다. 즉시
적용은 파라미터로만 하고, 영속화는 사람이 판단해서 한다.
"""

import time

from rcl_interfaces.srv import GetParameters

from dynamixel_control import calib_math

#: 캡처값의 신선도 한계. 이보다 낡은 `/joint_states` 는 캡처로 인정하지 않는다 —
#: 손으로 이미 움직인 뒤에 그 전 값을 기록하면 조용히 틀린 캘리브가 나온다.
MAX_SAMPLE_AGE_S = 1.0

#: 그리퍼 rad↔tick 역산에 필요한 브릿지 파라미터.
GRIPPER_PARAMS = ('gripper_open_tick', 'gripper_close_tick',
                  'gripper_open_rad', 'gripper_close_rad', 'gripper_joints')


class CalibControl:
    """영점·기어비·그리퍼 마법사 + 브릿지 파라미터 적용."""

    def __init__(self, node, plane, *, bridge_node_name, joint_config):
        self.node = node
        self.plane = plane
        self.bridge_node_name = bridge_node_name
        #: `{관절: {id, center, direction, gear_ratio, extended}}` — 브릿지의 JOINT_CONFIG.
        self.joint_config = joint_config
        self._get_client = None

        plane.register_task('calib_zero', self._validate_zero, self._run_zero)
        plane.register_task('calib_gear', self._validate_gear, self._run_gear)
        plane.register_task('calib_gripper', self._validate_gripper, self._run_gripper)
        plane.register_task('calib_apply', self._validate_apply, self._run_apply)

    # ------------------------------------------------------------ 공통
    def describe(self):
        """화면이 마법사를 그릴 근거."""
        return {
            'bridge_node': self.bridge_node_name,
            'joints': {
                name: {'id': cfg['id'], 'center': cfg['center'],
                       'direction': cfg['direction'],
                       'gear_ratio': cfg['gear_ratio'], 'extended': cfg['extended']}
                for name, cfg in self.joint_config.items()
            },
        }

    def _fetch(self, names, done):
        """브릿지 파라미터 조회 → `(True, None)` 또는 `(False, 사유)`.

        ⚠️ `spin_until_future_complete` 금지 — 이 함수는 타이머(실행기) 스레드에서
        불리므로 결과를 기다리며 spin 하면 재진입 데드락이 난다.
        """
        if self._get_client is None:
            self._get_client = self.node.create_client(
                GetParameters, f'/{self.bridge_node_name}/get_parameters')
        if not self._get_client.service_is_ready():
            return False, (f'{self.bridge_node_name} 의 파라미터 서비스가 준비되지 '
                           '않았습니다 — 브릿지를 read_only:=true 로 띄우세요')
        future = self._get_client.call_async(GetParameters.Request(names=list(names)))
        future.add_done_callback(done)
        return True, None

    def _capture(self, joints, allow_partial=False):
        """지금 `/joint_states` 에서 캡처 → `(값, 사유)`.

        `allow_partial` 이면 값이 안 오는 축은 **건너뛰고** 나머지로 계속한다.
        서보 하나가 죽었다고 다른 축의 영점까지 못 재면 안 된다 —
        `measure_zero_offset.py` 도 없는 축은 경고만 하고 넘어간다. 다만 하나도 없으면
        측정 자체가 성립하지 않으므로 그때는 거절한다.
        """
        now = time.monotonic()
        positions = self.node.store.joint_positions(max_age_s=MAX_SAMPLE_AGE_S, now=now)
        missing = [n for n in joints if n not in positions]
        captured = {n: positions[n] for n in joints if n in positions}
        if missing and not (allow_partial and captured):
            return None, (f'{", ".join(missing)} 의 최신 /joint_states 가 없습니다 '
                          f'({MAX_SAMPLE_AGE_S}초 이내 값 필요) — 브릿지가 떠 있는지, '
                          '그 서보가 버스에 응답하는지 확인하세요')
        return captured, None

    def _finish(self, payload, state, detail):
        task_id = payload.get('_task_id')
        if task_id is not None:
            self.plane.bus.finish_task(task_id, state, detail, time.monotonic())

    @staticmethod
    def _parse_overrides(value, cast):
        """`["arm_joint_2:9.034", …]` → `{이름: 값}` (브릿지 파라미터 형식)."""
        out = {}
        for entry in getattr(value, 'string_array_value', []):
            name, _, raw = str(entry).partition(':')
            try:
                out[name] = cast(raw)
            except ValueError:
                continue
        return out

    # ------------------------------------------------------------ 영점
    def _validate_zero(self, payload):
        reference = payload.get('reference') or {}
        if not isinstance(reference, dict):
            return None, 'reference 는 {관절: rad} 객체여야 합니다'
        out = {}
        for name, value in reference.items():
            if name not in self.joint_config:
                return None, f'모르는 관절: {name!r}'
            try:
                out[name] = float(value)
            except (TypeError, ValueError):
                return None, f'{name} 기준각이 숫자가 아닙니다: {value!r}'
        return {'reference': out}, None

    def _run_zero(self, payload):
        joints = list(self.joint_config)
        sample, reason = self._capture(joints, allow_partial=True)
        if sample is None:
            return 'error', reason
        payload['_skipped'] = [n for n in joints if n not in sample]

        def done(future):
            try:
                values = future.result().values
            except Exception as exc:                          # noqa: BLE001
                self._finish(payload, 'error', f'파라미터 조회 실패: {exc}')
                return
            ratios = self._parse_overrides(values[0], float)
            centers = self._parse_overrides(
                values[1], lambda v: int(round(float(v))))
            self._compute_zero(payload, sample, ratios, centers)

        ok, reason = self._fetch(['gear_ratios', 'centers'], done)
        if not ok:
            return 'error', reason
        return 'async', '기준 자세 캡처 완료 — 브릿지 파라미터 조회 중'

    def _compute_zero(self, payload, sample, ratios, centers):
        reference = payload['reference']
        rows, warnings, block, applies = [], [], [], []
        for name, cfg in self.joint_config.items():
            if name not in sample:
                continue
            ratio = ratios.get(name, cfg['gear_ratio'])
            center_old = centers.get(name, cfg['center'])
            center_new = calib_math.center_from_measurement(
                center_old, cfg['direction'], ratio,
                rad_measured=sample[name], rad_ref=reference.get(name, 0.0))
            shift = center_new - center_old
            reason = calib_math.center_out_of_range(center_new, cfg['extended'])
            if reason is not None:
                warnings.append(f'{name}: {reason}')
            rows.append({
                'joint': name, 'center_old': center_old,
                'center_new': int(round(center_new)), 'shift_tick': round(shift, 1),
                'shift_deg': round(calib_math.center_shift_deg(shift, ratio), 2),
                'measured_rad': round(sample[name], 4),
                'reference_rad': reference.get(name, 0.0),
                'gear_ratio': ratio, 'out_of_range': reason is not None,
            })
            block.append(calib_math.format_joint_config_entry(
                name, cfg['id'], center_new, cfg['direction'], ratio, cfg['extended']))
            applies.append(f'{name}:{int(round(center_new))}')

        skipped = payload.get('_skipped') or []
        if skipped:
            # 건너뛴 축은 **영점이 갱신되지 않는다.** 그 사실이 화면에서 사라지면
            # 나머지만 맞춰 놓고 다 됐다고 오해한다.
            warnings.append(
                f'{", ".join(skipped)} 는 /joint_states 가 없어 건너뛰었습니다 — '
                '그 축의 영점은 그대로입니다(서보 응답을 먼저 확인하세요).')

        self.node.store.set_calib_result({
            'kind': 'zero', 'rows': rows, 'warnings': warnings,
            'skipped': skipped,
            'block': '\n'.join(block), 'apply_target': 'centers',
            'apply_values': applies, 'at': time.time(),
        })
        self._finish(payload, 'done',
                     f'{len(rows)}개 축 영점 계산 완료'
                     + (f', {len(skipped)}개 축 건너뜀' if skipped else '')
                     + (f' — 경고 {len(warnings)}건' if warnings else ''))

    # ------------------------------------------------------------ 기어비
    def _validate_gear(self, payload):
        name = payload.get('joint')
        if name not in self.joint_config:
            return None, f'모르는 관절: {name!r}'
        try:
            start = float(payload['start_rad'])
            end = float(payload['end_rad'])
            joint_deg = float(payload['joint_deg'])
        except (KeyError, TypeError, ValueError):
            return None, 'start_rad·end_rad·joint_deg 가 모두 숫자여야 합니다'
        return {'joint': name, 'start_rad': start, 'end_rad': end,
                'joint_deg': joint_deg}, None

    def _run_gear(self, payload):
        def done(future):
            try:
                values = future.result().values
            except Exception as exc:                          # noqa: BLE001
                self._finish(payload, 'error', f'파라미터 조회 실패: {exc}')
                return
            ratios = self._parse_overrides(values[0], float)
            self._compute_gear(payload, ratios)

        ok, reason = self._fetch(['gear_ratios'], done)
        if not ok:
            return 'error', reason
        return 'async', '브릿지 기어비 조회 중'

    def _compute_gear(self, payload, ratios):
        name = payload['joint']
        cfg = self.joint_config[name]
        current = ratios.get(name, cfg['gear_ratio'])

        # ⚠️ 브릿지가 발행하는 `/joint_states` 는 **관절각 도메인**이다 — 이미 지금의
        #    기어비로 나눠진 값이라, 그 차이를 그대로 서보축 회전량으로 쓰면 현재
        #    기어비가 1.0 이 아닌 축에서 결과가 그 배수만큼 틀린다(이미 9.034 로
        #    설정된 축을 재면 1.0 이 나온다 — 그럴듯해서 더 위험하다).
        published_delta = payload['end_rad'] - payload['start_rad']
        servo_delta = published_delta * current
        try:
            ratio, inverted = calib_math.gear_ratio_from_span(
                servo_delta, payload['joint_deg'])
        except ValueError as exc:
            self._finish(payload, 'error', str(exc))
            return

        warnings = []
        if inverted:
            warnings.append(
                '부호가 음수입니다 — 서보와 관절이 반대로 돕니다. JOINT_CONFIG 의 '
                'direction 부호를 뒤집어야 할 수 있습니다(기어비는 절대값을 씁니다).')
        self.node.store.set_calib_result({
            'kind': 'gear', 'joint': name,
            'rows': [{
                'joint': name, 'gear_ratio_old': current,
                'gear_ratio_new': round(ratio, 3),
                'published_delta_rad': round(published_delta, 4),
                'servo_delta_rad': round(servo_delta, 4),
                'joint_deg': payload['joint_deg'], 'inverted': inverted,
            }],
            'warnings': warnings,
            'block': calib_math.format_joint_config_entry(
                name, cfg['id'], cfg['center'], cfg['direction'],
                round(ratio, 3), cfg['extended']),
            'apply_target': 'gear_ratios',
            'apply_values': [f'{name}:{ratio:.3f}'], 'at': time.time(),
        })
        self._finish(payload, 'done', f'{name} 기어비 {ratio:.3f} : 1')

    # ------------------------------------------------------------ 그리퍼 끝단
    def _validate_gripper(self, payload):
        try:
            closed = float(payload['closed_rad'])
            opened = float(payload['opened_rad'])
        except (KeyError, TypeError, ValueError):
            return None, 'closed_rad·opened_rad 가 숫자여야 합니다'
        try:
            margin = int(payload.get('margin') or 0)
        except (TypeError, ValueError):
            return None, 'margin 은 정수여야 합니다'
        if margin < 0:
            return None, 'margin 은 0 이상이어야 합니다'
        return {'closed_rad': closed, 'opened_rad': opened, 'margin': margin}, None

    def _run_gripper(self, payload):
        def done(future):
            try:
                values = future.result().values
            except Exception as exc:                          # noqa: BLE001
                self._finish(payload, 'error', f'파라미터 조회 실패: {exc}')
                return
            self._compute_gripper(payload, values)

        ok, reason = self._fetch(GRIPPER_PARAMS, done)
        if not ok:
            return 'error', reason
        return 'async', '브릿지 그리퍼 캘리브 조회 중'

    def _compute_gripper(self, payload, values):
        open_tick = int(values[0].integer_value)
        close_tick = int(values[1].integer_value)
        open_rad = float(values[2].double_value)
        close_rad = float(values[3].double_value)
        joints = list(values[4].string_array_value)
        if open_rad == close_rad:
            self._finish(payload, 'error',
                         '브릿지의 gripper_open_rad 와 close_rad 가 같습니다')
            return

        # 브릿지 `gripper_tick_to_pos` 의 역 — 발행된 rad 에서 raw tick 을 복원한다
        # (`measure_gripper_endpoints.py` 와 같은 방식. 포트를 열지 않는다).
        def to_tick(rad):
            frac = (rad - close_rad) / (open_rad - close_rad)
            return close_tick + frac * (open_tick - close_tick)

        try:
            result = calib_math.gripper_endpoints(
                to_tick(payload['closed_rad']), to_tick(payload['opened_rad']),
                payload['margin'])
        except ValueError as exc:
            self._finish(payload, 'error', str(exc))
            return

        self.node.store.set_calib_result({
            'kind': 'gripper',
            'joint': joints[0] if joints else 'gripper',
            'rows': [{
                'close_tick_old': close_tick, 'open_tick_old': open_tick,
                'close_tick_new': result['close'], 'open_tick_new': result['open'],
                'stroke_tick': result['stroke_tick'],
                'stroke_deg': round(result['stroke_deg'], 1),
                'margin': payload['margin'],
            }],
            'warnings': result['warnings'],
            'block': (f'        "gripper_open_tick": {result["open"]},\n'
                      f'        "gripper_close_tick": {result["close"]},'),
            'apply_target': 'gripper',
            'apply_values': [f'gripper_open_tick:{result["open"]}',
                             f'gripper_close_tick:{result["close"]}'],
            'at': time.time(),
        })
        self._finish(payload, 'done',
                     f'닫힘 {result["close"]} / 열림 {result["open"]} tick '
                     f'(stroke {result["stroke_deg"]:.1f}°)')

    # ------------------------------------------------------------ 즉시 적용
    def _validate_apply(self, payload):
        target = payload.get('target')
        if target not in ('centers', 'gear_ratios', 'gripper'):
            return None, f'적용 대상이 잘못되었습니다: {target!r}'
        values = payload.get('values')
        if not isinstance(values, list) or not values:
            return None, 'values 가 비어 있습니다'
        if not all(isinstance(v, str) for v in values):
            return None, 'values 는 "이름:값" 문자열 배열이어야 합니다'
        return {'target': target, 'values': list(values)}, None

    def _run_apply(self, payload):
        target, values = payload['target'], payload['values']
        if target == 'gripper':
            # 그리퍼만 형식이 다르다 — 브릿지가 tick 을 정수 파라미터 두 개로 받는다.
            params = {}
            for entry in values:
                name, _, raw = entry.partition(':')
                if name not in ('gripper_open_tick', 'gripper_close_tick'):
                    return 'error', f'적용할 수 없는 파라미터: {name!r}'
                try:
                    params[name] = int(raw)
                except ValueError:
                    return 'error', f'정수가 아닙니다: {entry!r}'
        else:
            params = {target: values}

        def done(future):
            try:
                response = future.result()
            except Exception as exc:                          # noqa: BLE001
                self._finish(payload, 'error', f'서비스 호출 실패: {exc}')
                return
            result = getattr(response, 'result', None)
            if result is not None and not result.successful:
                self._finish(payload, 'error', result.reason or '거절됨')
                return
            self._finish(payload, 'done',
                         f'{target} 적용됨 — read_only 로 /joint_states 를 다시 '
                         '확인하세요 (영속화는 소스 반영이 필요합니다)')

        ok, reason = self.plane.set_remote_params(
            self.bridge_node_name, params, done=done)
        if not ok:
            return 'error', reason
        return 'async', f'{target} 적용 요청 전송'
