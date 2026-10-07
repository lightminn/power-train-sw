#!/usr/bin/env python3
"""제어 의도의 스레드 경계 — HTTP 스레드가 쓰고, 실행기 스레드가 읽어서 실행한다.

## 왜 이 파일이 따로 있나

`telemetry_node` 는 **rclpy 엔티티 조작을 HTTP 스레드에서 하지 않는다**는 규칙을
이미 갖고 있다(그 파일의 2Hz 조정 타이머 주석). `video_hub` 가 그 규칙의 선례다 —
HTTP 스레드는 refcount 만 올리고, 실제 구독 생성/파괴는 노드 타이머가 한다.

제어도 똑같이 간다. HTTP 핸들러는 여기에 **의도만** 적고 즉시 응답하며, 발행과
파라미터 설정은 전부 실행기 스레드의 타이머가 가져가서 한다. 그래서 이 모듈은
ROS 를 전혀 모르고(=하드웨어 없이 pytest 로 검증 가능), `state_store` 와 같은
사상이다.

## 단일 조종자

탭 두 개가 동시에 조그를 밀면 서로의 속도를 덮어써서 **어느 쪽도 자기가 명령한
대로 움직이지 않는다.** 그래서 토큰 하나만 살아 있게 하고(TTL), 나머지는 관전만
한다. 토큰은 하트비트로 갱신되므로, 조종하던 탭이 죽으면 TTL 뒤에 자동으로 풀린다.

## 워치독

조그 의도가 `intent_timeout_s` 넘게 갱신되지 않으면 `take_jog` 가 **정확히 한 번**
`'stop'` 을 돌려준다. 브라우저가 죽거나 네트워크가 끊겨도 팔이 마지막 속도로 계속
도는 일이 없어야 하기 때문이다(`joystick_teleop_node` 의 `_zeroed` 래치와 같은 모양 —
0 을 계속 쏘지 않고 한 번만 쏘고 멈춘다).

## 정지에는 두 가지 사유가 있고, 둘을 구분해야 한다

키를 떼는 정상 조작도 "의도가 더 이상 안 온다"로 보면 워치독과 구별되지 않는다.
그러면 **키를 뗄 때마다 경고가 감사 로그에 쌓여 진짜 두절 신호가 묻힌다.** 그래서
브라우저가 `release_jog` 로 "내가 놓았다"를 명시하고, 정지의 사유를 `last_stop_reason`
으로 남긴다 — `'released'`(정상 조작) 와 `'watchdog'`(의도가 끊김). 발행되는 것은
어느 쪽이든 **전 관절 0 한 번**으로 똑같다. 다른 건 기록뿐이다.
"""

import secrets
import threading
from collections import deque

#: 조종권 토큰 수명. 브라우저는 이보다 짧은 주기로 renew 한다.
DEFAULT_TOKEN_TTL_S = 5.0

#: 조그 의도의 신선도 한계. 넘으면 워치독이 0 을 한 번 발행한다.
DEFAULT_INTENT_TIMEOUT_S = 0.3

#: 명령/작업 큐 상한 — 클라이언트가 폭주해도 메모리가 늘지 않게.
DEFAULT_MAX_QUEUE = 64

#: 완료된 작업 결과 보관 개수(브라우저가 폴링해서 가져간다).
DEFAULT_MAX_RESULTS = 32


class CommandBus:
    """락으로 보호되는 제어 의도 저장소. ROS 비의존."""

    def __init__(self, *, token_ttl_s=DEFAULT_TOKEN_TTL_S,
                 intent_timeout_s=DEFAULT_INTENT_TIMEOUT_S,
                 max_queue=DEFAULT_MAX_QUEUE, max_results=DEFAULT_MAX_RESULTS):
        self.token_ttl_s = float(token_ttl_s)
        self.intent_timeout_s = float(intent_timeout_s)

        self._lock = threading.RLock()

        # 조종권
        self._token = None
        self._label = ''
        self._claimed_at = 0.0
        self._seen_at = 0.0

        # 조그 의도 (마지막 값만 — 큐를 쌓으면 낡은 속도가 뒤늦게 나간다)
        self._jog = {}
        self._jog_at = 0.0
        self._jog_seq = 0
        self._zeroed = True        # 부팅 직후엔 이미 정지 상태다 — 0 을 쏠 필요 없다
        self._released = False     # 다음 정지가 '놓았다'인지 '끊겼다'인지
        self._stop_reason = None   # 마지막으로 실제 발행된 정지의 사유

        # 이산 명령 / 비동기 작업
        self._cmds = deque(maxlen=max_queue)
        self._tasks = deque(maxlen=max_queue)
        self._task_seq = 0
        self._results = deque(maxlen=max_results)

    # ------------------------------------------------------------ 조종권
    def holder(self, now):
        """현재 조종자 정보(없으면 None). 만료된 토큰은 여기서 정리된다."""
        with self._lock:
            self._expire_token(now)
            if self._token is None:
                return None
            return {
                'label': self._label,
                'held_s': round(now - self._claimed_at, 1),
                'expires_in_s': round(self.token_ttl_s - (now - self._seen_at), 2),
            }

    def claim(self, label, now, *, force=False):
        """조종권 획득 → `(token, None)` 또는 `(None, 사유)`."""
        with self._lock:
            self._expire_token(now)
            if self._token is not None and not force:
                return None, f'다른 클라이언트가 조종 중입니다 ({self._label})'
            self._token = secrets.token_urlsafe(16)
            self._label = str(label)[:64]
            self._claimed_at = now
            self._seen_at = now
            # 새 조종자에게 이전 조종자의 속도를 물려주지 않는다.
            self._jog = {}
            self._jog_at = 0.0
            return self._token, None

    def renew(self, token, now):
        with self._lock:
            self._expire_token(now)
            if token != self._token:
                return False
            self._seen_at = now
            return True

    def release(self, token, now):
        """조종권 반납 — 조그 의도도 함께 지워 다음 tick 이 정지를 내게 한다.

        반납은 **정상 조작**이므로 그 정지는 워치독으로 기록하지 않는다.
        """
        with self._lock:
            if token != self._token:
                return False
            self._released = not self._zeroed
            self._drop_token()
            return True

    def _expire_token(self, now):
        if self._token is not None and now - self._seen_at > self.token_ttl_s:
            self._drop_token()

    def _drop_token(self):
        """락을 쥔 채로만 호출할 것."""
        self._token = None
        self._label = ''
        self._jog = {}
        self._jog_at = 0.0

    # ------------------------------------------------------------ 조그
    def set_jog(self, token, velocities, now):
        """`{관절이름: rad/s}` 의도를 갱신. 토큰이 안 맞으면 False."""
        with self._lock:
            self._expire_token(now)
            if token != self._token:
                return False
            self._seen_at = now
            self._jog = {str(k): float(v) for k, v in velocities.items()}
            self._jog_at = now
            self._jog_seq += 1
            self._released = False
            return True

    def release_jog(self, token, now):
        """"데드맨을 놓았다" — 다음 tick 이 0 을 한 번 쏘고 멈춘다.

        의도 전송을 그냥 끊어도 워치독이 같은 일을 하지만, 그러면 정상 조작과
        통신 두절이 로그에서 구별되지 않는다(모듈 docstring 참고).
        """
        with self._lock:
            self._expire_token(now)
            if token != self._token:
                return False
            self._seen_at = now
            self._released = not self._zeroed
            self._jog = {}
            self._jog_at = 0.0
            return True

    def take_jog(self, now):
        """타이머가 부른다 → `('active', {…})` | `('stop', {})` | `('idle', None)`.

        `'stop'` 은 만료 직후 **한 번만** 나온다. 계속 0 을 쏘면 `teleop_core` 의
        deadman(0.5초 무입력이면 적분 정지)이 영원히 발동하지 못해, 오히려 정지
        상태를 흐리게 만든다.

        `'stop'` 을 돌려준 직후 `last_stop_reason()` 이 그 사유를 말해준다.
        """
        with self._lock:
            self._expire_token(now)
            fresh = (self._token is not None
                     and self._jog_at > 0.0
                     and now - self._jog_at <= self.intent_timeout_s)
            if fresh:
                self._zeroed = False
                return 'active', dict(self._jog)
            if not self._zeroed:
                self._zeroed = True
                self._jog = {}
                self._stop_reason = 'released' if self._released else 'watchdog'
                self._released = False
                return 'stop', {}
            return 'idle', None

    def last_stop_reason(self):
        """마지막으로 발행된 정지의 사유 — `'released'` | `'watchdog'` | None."""
        with self._lock:
            return self._stop_reason

    def jog_age(self, now):
        with self._lock:
            return None if self._jog_at <= 0.0 else now - self._jog_at

    # ------------------------------------------------------------ 이산 명령
    def push_cmd(self, token, cmd, now):
        with self._lock:
            self._expire_token(now)
            if token != self._token:
                return False
            self._seen_at = now
            self._cmds.append(str(cmd))
            return True

    def drain_cmds(self):
        with self._lock:
            out = list(self._cmds)
            self._cmds.clear()
            return out

    # ------------------------------------------------------------ 비동기 작업
    def push_task(self, token, kind, payload, now):
        """모델 교체·파라미터 설정 등 → `(task_id, None)` 또는 `(None, 사유)`."""
        with self._lock:
            self._expire_token(now)
            if token != self._token:
                return None, '조종권이 없습니다 (만료되었거나 다른 클라이언트 보유)'
            self._seen_at = now
            self._task_seq += 1
            task_id = self._task_seq
            self._tasks.append({'id': task_id, 'kind': str(kind),
                                'payload': dict(payload or {})})
            self._results.append({'id': task_id, 'kind': str(kind),
                                  'state': 'pending', 'detail': '', 'at': now})
            return task_id, None

    def drain_tasks(self):
        with self._lock:
            out = list(self._tasks)
            self._tasks.clear()
            return out

    def finish_task(self, task_id, state, detail='', now=0.0):
        """실행기 스레드가 결과를 적는다 (`done` | `error` | `running`)."""
        with self._lock:
            for entry in self._results:
                if entry['id'] == task_id:
                    entry['state'] = state
                    entry['detail'] = str(detail)
                    entry['at'] = now
                    return True
            return False

    def task_results(self, since=0):
        with self._lock:
            return [dict(e) for e in self._results if e['id'] > since]

    # ------------------------------------------------------------ 스냅샷
    def snapshot(self, now):
        """SSE 로 나가는 제어 상태 — 화면이 '조종 중/관전 중'을 그린다."""
        with self._lock:
            self._expire_token(now)
            age = None if self._jog_at <= 0.0 else round(now - self._jog_at, 2)
            return {
                'holder': None if self._token is None else {
                    'label': self._label,
                    'held_s': round(now - self._claimed_at, 1),
                    'expires_in_s': round(self.token_ttl_s - (now - self._seen_at), 2),
                },
                'jog_intent_age': age,
                'jog_seq': self._jog_seq,
                'stopped': self._zeroed,
                'stop_reason': self._stop_reason,
                'token_ttl_s': self.token_ttl_s,
                'intent_timeout_s': self.intent_timeout_s,
                'pending_tasks': len(self._tasks),
                'results': [dict(e) for e in self._results][-8:],
            }
