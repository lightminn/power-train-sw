#!/usr/bin/env python3
"""인식 모델 제어 — 핫스왑 + 재시작. `ControlPlane` 에 작업 두 개를 얹는다.

## 두 경로의 차이

`model_swap`    파라미터만 바꾼다. `perception_node` 가 프레임 사이에서 `.pt` 를 다시
                열고 결과를 `/perception/model_status` 로 알린다. 프로세스가 살아 있어
                카메라 재초기화가 없다 — **실습에서 "빠르게 바뀐다"를 보여줄 경로**다.
`model_restart` 프로세스를 죽였다 다시 띄운다. `width`/`height`/`fps`/`camera_mode` 처럼
                생성자에서만 읽는 값이나, `backend:=trt` 로 엔진을 다시 굽는 경우에만
                필요하다. `manage_perception:=true` 로 GUI 가 프로세스를 소유할 때만
                가능하다.

## 결과 보고가 두 단계인 이유

`SetParameters` 호출의 성공은 "노드가 요청을 받아들였다"는 뜻이지 "모델이 로드됐다"가
아니다(로드는 추론 스레드가 나중에 한다). 그래서 작업 결과는 서비스 응답으로 한 번,
`/perception/model_status` 로 다시 한 번 갱신된다 — 화면은 후자를 신뢰해야 한다.
"""

import time


class PerceptionControl:
    """모델 카탈로그 + 교체 작업."""

    def __init__(self, node, plane, *, perception_node_name, models_dir,
                 workspace_root, supervisor=None):
        self.node = node
        self.plane = plane
        self.perception_node_name = perception_node_name
        self.models_dir = models_dir
        self.workspace_root = workspace_root
        self.supervisor = supervisor

        plane.register_model_source(self.list_models)
        plane.register_task('model_swap', self._validate_swap, self._run_swap)
        plane.register_task('model_restart', self._validate_restart, self._run_restart)

    # ------------------------------------------------------------ 카탈로그
    def catalog(self):
        """호출할 때마다 다시 스캔한다 — 실습 중 떨군 파일이 즉시 떠야 한다."""
        from robot_arm_perception.model_presets import MODEL_PRESETS
        from .model_catalog import build_catalog
        return build_catalog(MODEL_PRESETS, self.models_dir, self.workspace_root)

    def list_models(self):
        return {
            'models': self.catalog(),
            'models_dir': self.models_dir,
            'can_restart': self.supervisor is not None,
            'restart_reason': (None if self.supervisor is not None else
                               'manage_perception:=false — GUI 가 이 프로세스를 '
                               '소유하지 않아 재시작할 수 없습니다(핫스왑은 가능)'),
            'supervisor': (None if self.supervisor is None
                           else self.supervisor.status()),
        }

    # ------------------------------------------------------------ 핫스왑
    def _resolve(self, payload):
        """payload → `(카탈로그 항목 기반 설정, 사유)`."""
        from .model_catalog import find
        key = payload.get('key')
        if not isinstance(key, str) or not key:
            return None, 'key 가 필요합니다'
        entry = find(self.catalog(), key)
        if entry is None:
            return None, f'목록에 없는 모델: {key}'
        if not entry['exists']:
            return None, f'파일이 없습니다: {entry["path"]}'

        task = payload.get('task') or entry['task']
        if task not in ('segment', 'detect'):
            return None, f'task 는 segment|detect 만 됩니다: {task!r}'
        classes = payload.get('classes')
        pick_classes = payload.get('pick_classes')
        return {
            # 프리셋이면 키가 곧 이름, 스캔 파일이면 파일명을 표시용 이름으로 쓴다.
            'model_name': key if entry['source'] == 'preset' else entry['label'],
            'model_path': entry['path'],
            'task': task,
            'classes': entry['classes'] if classes is None else str(classes),
            'pick_classes': (entry['pick_classes'] if pick_classes is None
                             else str(pick_classes)),
        }, None

    def _validate_swap(self, payload):
        params, reason = self._resolve(payload)
        if params is None:
            return None, reason
        return {'params': params, 'key': payload.get('key')}, None

    def _run_swap(self, payload):
        params = payload['params']
        started = time.monotonic()

        def done(future):
            try:
                response = future.result()
            except Exception as exc:                          # noqa: BLE001
                self._finish(payload, 'error', f'서비스 호출 실패: {exc}')
                return
            # 원자적 설정이라 결과는 하나다(`result`, 복수형 `results` 가 아니다).
            result = getattr(response, 'result', None)
            if result is not None and not result.successful:
                self._finish(payload, 'error', result.reason or '거절됨')
                return
            self._finish(payload, 'done',
                         f'요청 수락 ({time.monotonic() - started:.2f}s) — '
                         '실제 로드 결과는 model_status 참고')

        ok, reason = self.plane.set_remote_params(
            self.perception_node_name, params, done=done)
        if not ok:
            return 'error', reason
        # `_task_id` 는 `_run_task` 가 이미 넣어 뒀다 — done 콜백이 그걸로 결과를 적는다.
        return 'async', '교체 요청 전송'

    def _finish(self, payload, state, detail):
        task_id = payload.get('_task_id')
        if task_id is not None:
            self.plane.bus.finish_task(task_id, state, detail, time.monotonic())

    # ------------------------------------------------------------ 재시작
    def _validate_restart(self, payload):
        if self.supervisor is None:
            return None, ('manage_perception:=false — GUI 가 perception_node 를 '
                          '소유하지 않아 재시작할 수 없습니다')
        params, reason = self._resolve(payload)
        if params is None:
            return None, reason
        extra = {}
        for name in ('backend', 'camera_mode', 'width', 'height', 'fps'):
            if name in payload:
                extra[name] = payload[name]
        return {'params': params, 'extra': extra}, None

    def _run_restart(self, payload):
        others = [n for n in self.node.get_node_names()
                  if n == self.perception_node_name]
        if others and not self.supervisor.alive():
            # RealSense 는 프로세스 하나만 장치를 열 수 있다 — 남이 띄운 노드가
            # 있는데 또 띄우면 반드시 실패하고, 그 실패가 "카메라 고장"으로 오인된다.
            return 'error', (f'{self.perception_node_name} 가 이미 떠 있습니다 '
                             '(GUI 가 띄운 것이 아님) — 그쪽을 먼저 내리세요')

        params = dict(payload['params'])
        params.update(payload['extra'])
        ok, reason = self.supervisor.restart(params)
        if not ok:
            return 'error', reason
        return 'done', f'재시작 요청 완료 (pid={self.supervisor.status()["pid"]})'
