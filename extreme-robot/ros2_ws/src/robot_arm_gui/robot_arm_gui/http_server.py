#!/usr/bin/env python3
"""stdlib HTTP 서버 — 정적 페이지 + SSE 텔레메트리 + MJPEG 영상.

## 왜 stdlib 인가

컨테이너에 웹 스택이 하나도 없다(flask/fastapi/rosbridge/foxglove/
web_video_server 전부 미설치). 이 저장소는 시스템 의존성을 Dockerfile 로만
추가하도록 강제하고, Dockerfile 을 바꾸면 팀 전원이 arm64 이미지를 다시 빌드해야
한다. `http.server` + SSE + MJPEG 는 **새 의존성 0개**로 같은 일을 한다
(JPEG 인코딩만 이미 있는 cv2 를 쓴다).

## 백프레셔를 만들지 않는 구조

클라이언트별 큐를 두지 않는다. ROS 콜백은 `StateStore` 에 O(1) 로 쓰기만 하고,
각 SSE 스레드가 **자기 속도로** 스냅샷을 떠 간다. 느린 클라이언트는 자기
스레드에서만 `wfile.write` 에 막히고 프레임을 건너뛴다 — 텔레메트리에서는 그게
옳은 동작이며, 실행기는 전혀 영향받지 않는다.

## 상시 연결은 2개까지

브라우저는 오리진당 동시 연결이 6개다. SSE 1 + MJPEG 1 로 고정하고 나머지는
단발 fetch 로 처리한다. 탭을 3개 이상 열면 고갈되므로 `/api/health` 가 접속 수를
노출해 운영자가 인지하게 한다.

## 두 가지 모드 — 기본은 여전히 읽기 전용

`control` 객체가 **없으면**(기본) 이 서버는 예전 그대로다. GET 만 처리하고
POST 는 403, 노드는 퍼블리셔를 하나도 만들지 않는다.

`control:=true` 로 띄우면 `control` 객체가 주입되고 `/api/control/*` 계열이 살아난다.
그래도 **계약이 owner 를 정해 둔 토픽**(`/arm_status`·`/chassis_mode`·
`/arrival_status`)과 `/dynamixel/goal_position` 은 어느 모드에서도 발행하지 않는다 —
제어 모드가 미는 것은 owner 가 없는 `/arm/teleop_jog`·`/arm/teleop_cmd` 와
다른 노드의 파라미터뿐이다.

## POST 는 CSRF 를 막아야 한다

`bind:=0.0.0.0` 이 곧 현장 네트워크 노출인 구조라(노드가 이미 경고한다), 브라우저의
다른 탭이 이 서버로 요청을 밀어 넣을 수 있으면 안 된다. 두 겹으로 막는다:

1. `Origin` 헤더가 있으면 `Host` 와 일치해야 한다.
2. 커스텀 헤더 `X-Monitor-Control` 을 요구한다. 크로스오리진에서 커스텀 헤더를
   붙이려면 preflight(OPTIONS)가 통과해야 하는데 **이 서버는 OPTIONS 를 구현하지
   않는다** → 남의 페이지에서는 요청 자체가 성립하지 않는다.
"""

import json
import math
import os
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from .video_hub import SOURCES

#: SSE 주기. hot=모터 수치·상태 스트립, full=전체(관절/검출/텔레옵/이벤트).
HOT_INTERVAL_S = 0.2
FULL_INTERVAL_S = 1.0

#: 동시 SSE 클라이언트 상한 — 탭이 쌓여도 스레드가 무한히 늘지 않게.
MAX_SSE_CLIENTS = 8
MAX_VIDEO_CLIENTS = 4

_MJPEG_BOUNDARY = 'frameboundary'

#: POST 본문 상한. 제어 페이로드는 전부 작다(가장 큰 게 캘리브 실측점 목록).
MAX_BODY_BYTES = 64 * 1024

#: CSRF 방어용 커스텀 헤더 — 크로스오리진에서는 preflight 없이 붙일 수 없다.
CONTROL_HEADER = 'X-Monitor-Control'

_STATIC_TYPES = {
    '.html': 'text/html; charset=utf-8',
    '.js': 'application/javascript; charset=utf-8',
    '.css': 'text/css; charset=utf-8',
    '.svg': 'image/svg+xml',
    '.png': 'image/png',
    '.ico': 'image/x-icon',
}


#: 조그 한 건에 실을 수 있는 관절 수 / 속도 크기 상한(방어적 상한이며, 실제 제한은
#: 노드가 `teleop_core` 의 `max_vel_rad_s` 와 같은 값으로 다시 clamp 한다).
_MAX_JOG_JOINTS = 16
_MAX_JOG_ABS = 10.0


def _first_int(values, default):
    try:
        return int(values[0])
    except (TypeError, ValueError, IndexError):
        return default


def _parse_velocities(raw):
    """`{관절이름: rad/s}` 검증 → `(dict, None)` 또는 `(None, 사유)`."""
    if not isinstance(raw, dict):
        return None, 'velocities 는 {관절이름: rad/s} 객체여야 합니다'
    if len(raw) > _MAX_JOG_JOINTS:
        return None, f'관절이 너무 많습니다 ({len(raw)} > {_MAX_JOG_JOINTS})'
    out = {}
    for name, value in raw.items():
        if not isinstance(name, str) or not name or len(name) > 64:
            return None, f'관절 이름이 잘못되었습니다: {name!r}'
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            return None, f'{name} 속도가 숫자가 아닙니다: {value!r}'
        value = float(value)
        if not math.isfinite(value):
            return None, f'{name} 속도가 유한하지 않습니다'
        if abs(value) > _MAX_JOG_ABS:
            return None, f'{name} 속도가 상한을 넘습니다 (|{value}| > {_MAX_JOG_ABS})'
        out[name] = value
    return out, None


class MonitorHTTPServer(ThreadingHTTPServer):
    """daemon_threads 필수 — 안 그러면 스트리밍 스레드가 종료를 막는다."""

    daemon_threads = True
    allow_reuse_address = True

    def __init__(self, addr, handler, *, store, video_hub, web_root, logger=None,
                 control=None):
        super().__init__(addr, handler)
        self.store = store
        self.video_hub = video_hub
        self.web_root = os.path.realpath(web_root)
        self.logger = logger
        #: 제어 평면. None 이면 읽기 전용 모드(POST 전부 403).
        #: 노드가 주입하며, HTTP 스레드는 이 객체를 통해서만 ROS 에 닿는다.
        self.control = control
        self.sse_clients = 0
        self.video_clients = 0
        self.started_at = time.time()
        self.counter_lock = threading.Lock()


class MonitorHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'
    server_version = 'robot_arm_gui'

    # ------------------------------------------------------------ 로깅
    def log_message(self, fmt, *args):
        """기본 구현은 요청마다 stderr 에 찍는다 — SSE/MJPEG 때문에 시끄럽다."""
        logger = getattr(self.server, 'logger', None)
        if logger is not None:
            logger.debug(f'{self.address_string()} {fmt % args}')

    # ------------------------------------------------------------ 응답 헬퍼
    def _send_json(self, obj, status=200):
        body = json.dumps(obj, ensure_ascii=False, default=str).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)

    def _send_text(self, text, status=200, content_type='text/plain; charset=utf-8',
                   filename=None):
        body = text.encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(body)))
        if filename:
            self.send_header('Content-Disposition', f'attachment; filename="{filename}"')
        self.end_headers()
        self.wfile.write(body)

    def _send_binary(self, payload, content_type='application/octet-stream',
                     extra_headers=()):
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(payload)))
        self.send_header('Cache-Control', 'no-store')
        for key, value in extra_headers:
            self.send_header(key, str(value))
        self.end_headers()
        self.wfile.write(payload)

    def _begin_stream(self, content_type):
        """스트리밍 응답 시작 — Content-Length 없이 연결 종료로 끝낸다."""
        self.send_response(200)
        self.send_header('Content-Type', content_type)
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Connection', 'close')
        self.end_headers()
        self.close_connection = True

    # ------------------------------------------------------------ 요청 파싱
    def _read_json(self):
        """POST 본문 → dict. 실패하면 `(None, 사유)`."""
        try:
            length = int(self.headers.get('Content-Length') or 0)
        except ValueError:
            return None, 'Content-Length 가 숫자가 아닙니다'
        if length <= 0:
            return {}, None
        if length > MAX_BODY_BYTES:
            return None, f'본문이 너무 큽니다 ({length} > {MAX_BODY_BYTES})'
        raw = self.rfile.read(length)
        try:
            body = json.loads(raw.decode('utf-8'))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return None, f'JSON 파싱 실패: {exc}'
        if not isinstance(body, dict):
            return None, 'JSON 최상위는 객체여야 합니다'
        return body, None

    def _csrf_reason(self):
        """CSRF 검사 — 통과하면 None, 막히면 사유 문자열."""
        if self.headers.get(CONTROL_HEADER) is None:
            return f'{CONTROL_HEADER} 헤더가 없습니다'
        origin = self.headers.get('Origin')
        if origin:
            host = self.headers.get('Host') or ''
            if urlparse(origin).netloc != host:
                return f'Origin({origin}) 이 Host({host}) 와 다릅니다'
        return None

    # ------------------------------------------------------------ 라우팅
    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path
        try:
            if path == '/api/state':
                return self._api_state()
            if path == '/api/health':
                return self._api_health()
            if path == '/api/traces':
                return self._send_json(self.server.store.list_traces())
            if path.startswith('/api/trace/'):
                return self._api_trace(path.rsplit('/', 1)[-1])
            if path == '/api/stream':
                return self._sse()
            if path.startswith('/video/'):
                return self._mjpeg(path.rsplit('/', 1)[-1])
            if path.startswith('/api/'):
                return self._api_get(path, parse_qs(parsed.query))
            return self._static(path)
        except (BrokenPipeError, ConnectionResetError):
            # 브라우저가 탭을 닫으면 정상적으로 발생한다 — 에러가 아니다.
            self.close_connection = True

    def do_POST(self):
        path = urlparse(self.path).path
        try:
            control = self.server.control
            if control is None:
                return self._send_json(
                    {'error': '읽기 전용 모드입니다 — control:=true 로 다시 띄우세요'},
                    status=403)
            reason = self._csrf_reason()
            if reason is not None:
                return self._send_json({'error': f'요청이 거부되었습니다: {reason}'},
                                       status=403)
            body, reason = self._read_json()
            if body is None:
                return self._send_json({'error': reason}, status=400)
            return self._api_post(path, body, control)
        except (BrokenPipeError, ConnectionResetError):
            self.close_connection = True

    def do_PUT(self):
        self._send_json({'error': 'PUT 은 쓰지 않습니다'}, status=405)

    do_DELETE = do_PUT

    # ------------------------------------------------------------ API
    def _api_state(self):
        self._send_json(self.server.store.snapshot(time.monotonic()))

    def _api_health(self):
        srv = self.server
        self._send_json({
            'uptime_s': round(time.time() - srv.started_at, 1),
            'threads': threading.active_count(),
            'sse_clients': srv.sse_clients,
            'video_clients': srv.video_clients,
            'video': srv.video_hub.stats(),
            'limits': {'sse': MAX_SSE_CLIENTS, 'video': MAX_VIDEO_CLIENTS},
        })

    def _api_trace(self, raw_id):
        """트립 블랙박스 — 상승 엣지 직전 링버퍼를 JSONL 로 내려준다."""
        trace_id = raw_id.split('.')[0]
        if not trace_id.isdigit():
            return self._send_json({'error': 'bad trace id'}, status=400)
        trace = self.server.store.get_trace(int(trace_id))
        if trace is None:
            return self._send_json({'error': 'not found'}, status=404)
        lines = [json.dumps({'meta': {k: v for k, v in trace.items() if k != 'samples'}},
                            ensure_ascii=False)]
        lines += [json.dumps(s, ensure_ascii=False) for s in trace['samples']]
        self._send_text('\n'.join(lines) + '\n',
                        content_type='application/x-ndjson; charset=utf-8',
                        filename=f'trace_{trace_id}.jsonl')

    # ------------------------------------------------------------ 제어 API (GET)
    def _api_get(self, path, query):
        control = self.server.control
        if path == '/api/control':
            if control is None:
                return self._send_json({'enabled': False,
                                        'reason': '읽기 전용 모드 (control:=false)'})
            payload = {'enabled': True}
            payload.update(control.describe())
            payload['session'] = control.bus.snapshot(time.monotonic())
            return self._send_json(payload)

        if control is None:
            return self._send_json({'error': '읽기 전용 모드입니다'}, status=403)

        if path == '/api/models':
            return self._send_json(control.list_models())
        if path == '/api/tasks':
            since = _first_int(query.get('since'), 0)
            return self._send_json({'results': control.bus.task_results(since)})
        if path == '/api/cloud':
            return self._api_cloud(control, _first_int(query.get('since'), -1))
        return self._send_json({'error': 'not found', 'path': path}, status=404)

    def _api_cloud(self, control, since):
        """점구름 바이너리 — Float32 xyz 나열. 브라우저가 몇 Hz 로 폴링한다.

        SSE 는 텍스트라 수만 점을 실을 수 없고, stdlib 로 WebSocket 을 짜는 비용은
        정당화되지 않는다. 프레임이 안 바뀌었으면 204 로 즉시 끝낸다.
        """
        frame = control.cloud_frame(since)
        if frame is None:
            self.send_response(204)
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            return
        seq, payload, meta = frame
        headers = [('X-Cloud-Seq', seq), ('X-Cloud-Points', len(payload) // 12)]
        for key, value in meta.items():
            headers.append((f'X-Cloud-{key}', value))
        self._send_binary(payload, extra_headers=headers)

    # ------------------------------------------------------------ 제어 API (POST)
    def _api_post(self, path, body, control):
        now = time.monotonic()
        bus = control.bus

        if path == '/api/control/claim':
            label = str(body.get('label') or 'browser')[:64]
            force = bool(body.get('force'))
            # 다른 텔레옵 프론트엔드가 이미 /arm/teleop_jog 를 밀고 있으면 두 속도원이
            # 겹쳐 **어느 쪽도 명령대로 움직이지 않는다.** 강제로만 뚫을 수 있다.
            conflict = control.jog_publisher_conflict()
            if conflict is not None and not force:
                return self._send_json({'error': conflict, 'conflict': True}, status=409)
            token, reason = bus.claim(label, now, force=force)
            if token is None:
                return self._send_json({'error': reason}, status=409)
            control.on_claim(label, force, conflict)
            return self._send_json({'token': token, 'session': bus.snapshot(now)})

        if path == '/api/control/renew':
            ok = bus.renew(body.get('token'), now)
            if not ok:
                return self._send_json({'error': '토큰이 만료되었거나 무효합니다'},
                                       status=409)
            return self._send_json({'session': bus.snapshot(now)})

        if path == '/api/control/release':
            bus.release(body.get('token'), now)
            control.on_release()
            return self._send_json({'session': bus.snapshot(now)})

        if path == '/api/teleop/jog':
            velocities, reason = _parse_velocities(body.get('velocities'))
            if velocities is None:
                return self._send_json({'error': reason}, status=400)
            if not bus.set_jog(body.get('token'), velocities, now):
                return self._send_json({'error': '조종권이 없습니다'}, status=409)
            return self._send_json({'ok': True, 'seq': bus.snapshot(now)['jog_seq']})

        if path == '/api/teleop/release_jog':
            # "데드맨을 놓았다" — 전송을 그냥 끊어도 워치독이 세우지만, 그러면
            # 정상 조작이 통신 두절과 같은 경고로 기록된다.
            if not bus.release_jog(body.get('token'), now):
                return self._send_json({'error': '조종권이 없습니다'}, status=409)
            return self._send_json({'ok': True})

        if path == '/api/teleop/cmd':
            from .teleop_vocab import validate as validate_cmd
            cmd, reason = validate_cmd(body.get('cmd'))
            if cmd is None:
                return self._send_json({'error': reason}, status=400)
            if not bus.push_cmd(body.get('token'), cmd, now):
                return self._send_json({'error': '조종권이 없습니다'}, status=409)
            return self._send_json({'ok': True, 'cmd': cmd})

        if path == '/api/task':
            kind = str(body.get('kind') or '')
            payload, reason = control.validate_task(kind, body.get('payload') or {})
            if payload is None:
                return self._send_json({'error': reason}, status=400)
            task_id, reason = bus.push_task(body.get('token'), kind, payload, now)
            if task_id is None:
                return self._send_json({'error': reason}, status=409)
            return self._send_json({'task_id': task_id})

        return self._send_json({'error': 'not found', 'path': path}, status=404)

    # ------------------------------------------------------------ SSE
    def _sse(self):
        srv = self.server
        with srv.counter_lock:
            if srv.sse_clients >= MAX_SSE_CLIENTS:
                return self._send_json({'error': 'too many SSE clients'}, status=503)
            srv.sse_clients += 1
        try:
            self._begin_stream('text/event-stream; charset=utf-8')
            # 브라우저 EventSource 자동 재접속 간격.
            self.wfile.write(b'retry: 3000\n\n')
            self.wfile.flush()

            store = srv.store
            next_hot = 0.0
            next_full = 0.0
            while True:
                now = time.monotonic()
                if now >= next_full:
                    self._sse_send('full', store.snapshot(now))
                    next_full = now + FULL_INTERVAL_S
                    next_hot = now + HOT_INTERVAL_S
                elif now >= next_hot:
                    self._sse_send('hot', store.hot_snapshot(now))
                    next_hot = now + HOT_INTERVAL_S
                time.sleep(0.02)
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            with srv.counter_lock:
                srv.sse_clients -= 1
            self.close_connection = True

    def _sse_send(self, event, payload):
        body = json.dumps(payload, ensure_ascii=False, default=str)
        self.wfile.write(f'event: {event}\ndata: {body}\n\n'.encode('utf-8'))
        self.wfile.flush()

    # ------------------------------------------------------------ MJPEG
    def _mjpeg(self, source):
        if source not in SOURCES:
            return self._send_json(
                {'error': f'unknown source (가능: {sorted(SOURCES)})'}, status=404)

        srv = self.server
        with srv.counter_lock:
            if srv.video_clients >= MAX_VIDEO_CLIENTS:
                return self._send_json({'error': 'too many video clients'}, status=503)
            srv.video_clients += 1

        hub = srv.video_hub
        hub.acquire(source)
        try:
            self._begin_stream(
                f'multipart/x-mixed-replace; boundary={_MJPEG_BOUNDARY}')
            seq = 0
            idle = 0
            while True:
                seq, jpeg = hub.wait_frame(source, seq, timeout=1.0)
                if jpeg is None:
                    # 구독이 살아나기까지(노드 2Hz 조정 타이머) 잠깐 걸린다.
                    # 너무 오래 비면 클라이언트가 끊긴 것으로 보고 정리한다.
                    idle += 1
                    if idle > 30:
                        break
                    continue
                idle = 0
                head = (f'--{_MJPEG_BOUNDARY}\r\n'
                        f'Content-Type: image/jpeg\r\n'
                        f'Content-Length: {len(jpeg)}\r\n\r\n').encode('ascii')
                self.wfile.write(head)
                self.wfile.write(jpeg)
                self.wfile.write(b'\r\n')
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass
        finally:
            hub.release(source)
            with srv.counter_lock:
                srv.video_clients -= 1
            self.close_connection = True

    # ------------------------------------------------------------ 정적 파일
    def _static(self, path):
        rel = 'index.html' if path in ('/', '') else path.lstrip('/')
        target = os.path.realpath(os.path.join(self.server.web_root, rel))
        # 경로 탈출 방어 — web_root 밖은 절대 서빙하지 않는다.
        if not target.startswith(self.server.web_root + os.sep):
            return self._send_json({'error': 'forbidden'}, status=403)
        if not os.path.isfile(target):
            return self._send_json({'error': 'not found', 'path': rel}, status=404)

        ctype = _STATIC_TYPES.get(os.path.splitext(target)[1], 'application/octet-stream')
        with open(target, 'rb') as fh:
            body = fh.read()
        self.send_response(200)
        self.send_header('Content-Type', ctype)
        self.send_header('Content-Length', str(len(body)))
        self.send_header('Cache-Control', 'no-store')
        self.end_headers()
        self.wfile.write(body)


def serve_forever_in_thread(*, store, video_hub, web_root, bind='127.0.0.1',
                            port=8088, logger=None, control=None):
    """서버를 daemon 스레드에서 띄우고 (server, thread) 를 돌려준다.

    메인 스레드는 `rclpy.spin()` 이 가져간다 — 블록하는 쪽(HTTP)을 스레드로
    빼는 게 더 단순하고, `spin_once(0.0)` 바쁜 대기의 CPU 낭비도 없다.
    (`vision_test_node` 가 반대로 한 건 cv2.imshow 가 메인 스레드를 요구해서다.)

    `control` 이 None 이면 읽기 전용 모드다(POST 전부 403).
    """
    server = MonitorHTTPServer((bind, port), MonitorHandler, store=store,
                               video_hub=video_hub, web_root=web_root, logger=logger,
                               control=control)
    thread = threading.Thread(target=server.serve_forever, name='monitor-http',
                              daemon=True)
    thread.start()
    return server, thread
