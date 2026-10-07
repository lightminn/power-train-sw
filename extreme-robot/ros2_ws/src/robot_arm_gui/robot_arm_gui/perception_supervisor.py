#!/usr/bin/env python3
"""`perception_node` 프로세스 감독 — 핫스왑이 안 통할 때의 폴백.

## 왜 재시작 경로가 따로 필요한가

핫스왑은 `.pt` 를 바꾸는 데는 충분하지만, **생성자에서만 읽는 값**은 못 바꾼다:
`width`/`height`/`fps`/`camera_mode`, 그리고 RealSense 파이프라인 자체. `backend` 를
`trt` 로 바꾸는 것도 엔진을 굽는 동안 추론 스레드가 멈춰 있어 재시작이 더 낫다.
모델이 이상한 상태로 물렸을 때 "그냥 다시 띄우기"가 가장 확실한 복구이기도 하다.

## ⚠️ 기본은 꺼져 있다 (`manage_perception:=false`)

켜면 GUI 가 `perception_node` 를 **자기 자식 프로세스로** 띄운다. 이미 다른 곳에서
띄워 둔 노드가 있으면 spawn 을 거부한다 — RealSense 는 프로세스 하나만 장치를 열 수
있어서, 중복 실행은 반드시 실패하고 그 실패가 "카메라가 고장났나"로 오인된다.

꺼져 있으면 재시작 버튼은 비활성이고 사유가 화면에 뜬다. 핫스왑은 그대로 된다.
"""

import os
import shlex
import signal
import subprocess
import threading
import time
from collections import deque

#: 종료 시 SIGTERM 뒤 이만큼 기다리고 SIGKILL.
TERM_GRACE_S = 5.0

#: 화면에 보여줄 최근 출력 줄 수.
LOG_TAIL = 40


class PerceptionSupervisor:
    """`ros2 run robot_arm_perception perception_node` 자식 프로세스 하나를 관리한다."""

    def __init__(self, *, workspace_root, logger=None):
        self.workspace_root = workspace_root
        self._logger = logger
        self._lock = threading.RLock()
        self._proc = None
        self._args = {}
        self._log = deque(maxlen=LOG_TAIL)
        self._reader = None
        self._started_at = None

    # ------------------------------------------------------------ 상태
    def alive(self):
        with self._lock:
            return self._proc is not None and self._proc.poll() is None

    def status(self):
        with self._lock:
            proc = self._proc
            return {
                'managed': True,
                'running': proc is not None and proc.poll() is None,
                'pid': None if proc is None else proc.pid,
                'returncode': None if proc is None else proc.returncode,
                'uptime_s': (None if self._started_at is None
                             else round(time.time() - self._started_at, 1)),
                'args': dict(self._args),
                'log': list(self._log),
            }

    # ------------------------------------------------------------ 기동/종료
    def start(self, params):
        """`{파라미터명: 값}` 으로 새로 띄운다. `(ok, 사유)`."""
        with self._lock:
            if self.alive():
                return False, '이미 GUI 가 관리하는 perception_node 가 돌고 있습니다'

            argv = ['ros2', 'run', 'robot_arm_perception', 'perception_node',
                    '--ros-args']
            for name, value in params.items():
                argv += ['-p', f'{name}:={_ros_arg(value)}']

            # ⚠️ cwd 가 워크스페이스 루트여야 한다 — 프리셋의 model_path 가 CWD
            # 상대라서다. GUI 는 절대경로를 넘기지만, 노드가 자기 기본값으로
            # 폴백하는 경우까지 안전하게 만들어 둔다.
            try:
                proc = subprocess.Popen(
                    argv, cwd=self.workspace_root,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                    text=True, bufsize=1, start_new_session=True)
            except OSError as exc:
                return False, f'실행 실패: {exc}'

            self._proc = proc
            self._args = dict(params)
            self._started_at = time.time()
            self._log.clear()
            self._log.append(f'$ {" ".join(shlex.quote(a) for a in argv)}')
            self._reader = threading.Thread(
                target=self._drain, args=(proc,), daemon=True,
                name='perception-log')
            self._reader.start()
            self._note(f'perception_node 기동 (pid={proc.pid})')
            return True, None

    def stop(self):
        """`(ok, 사유)`. 이미 죽어 있으면 성공으로 본다."""
        with self._lock:
            proc = self._proc
            if proc is None or proc.poll() is not None:
                return True, None
            pgid = None
            try:
                pgid = os.getpgid(proc.pid)
            except OSError:
                pass

        # ⚠️ `ros2 run` 은 래퍼라 부모만 죽이면 파이썬 노드가 남는다 — 이 저장소가
        # 반복해서 밟은 함정이라(유령 프로세스가 /dev 를 계속 물고 있다) 프로세스
        # 그룹 전체에 보낸다(start_new_session=True 로 그룹을 따로 뒀다).
        try:
            if pgid is not None:
                os.killpg(pgid, signal.SIGTERM)
            else:
                proc.terminate()
            proc.wait(timeout=TERM_GRACE_S)
        except subprocess.TimeoutExpired:
            try:
                if pgid is not None:
                    os.killpg(pgid, signal.SIGKILL)
                else:
                    proc.kill()
                proc.wait(timeout=TERM_GRACE_S)
            except (OSError, subprocess.TimeoutExpired) as exc:
                return False, f'종료 실패: {exc}'
        except OSError as exc:
            return False, f'종료 신호 실패: {exc}'

        self._note('perception_node 종료')
        return True, None

    def restart(self, params):
        ok, reason = self.stop()
        if not ok:
            return False, reason
        # 포트/장치가 완전히 풀릴 때까지 짧게 기다린다 — RealSense 는 즉시
        # 다시 열면 이따금 'device busy' 로 실패한다.
        time.sleep(1.0)
        return self.start(params)

    # ------------------------------------------------------------ 내부
    def _drain(self, proc):
        """자식의 출력을 링버퍼에 담는다 — 실패 사유가 화면에 그대로 뜨게."""
        try:
            for line in proc.stdout:
                with self._lock:
                    self._log.append(line.rstrip())
        except (ValueError, OSError):
            pass
        with self._lock:
            self._log.append(f'[프로세스 종료: returncode={proc.returncode}]')

    def _note(self, text):
        if self._logger is not None:
            self._logger.info(text)


def _ros_arg(value):
    """파이썬 값 → `ros2 run --ros-args -p name:=value` 의 값 표기."""
    if isinstance(value, bool):
        return 'true' if value else 'false'
    return str(value)
