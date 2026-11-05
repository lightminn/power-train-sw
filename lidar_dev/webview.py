# webview.py — 화면 없는 실행(Jetson 등, opencv-headless)용 브라우저 보기(live_overlay 7판 --headless, 10-22 밤)
#   표준 라이브러리 http.server만: / = 화면(MJPEG) + 키 버튼 + 화면 클릭(카메라 점 재기·지도 칸), /stream = multipart MJPEG, /key?k=문자, /click?x=&y=(원 프레임 좌표)
#   같은 네트워크(테일넷 포함)에서 http://<호스트>:<포트>/ 로 연다. 인증 없음 — 신뢰하는 네트워크에서만(기본 0.0.0.0이 아니라 --serve-host로 정함)
import collections, html, threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2

KEYS = [("w", "화면 녹화"), ("k", "장착 캘리브레이션"), ("p", "바닥 사전값"), ("m", "지도 새로"), ("v", "지도 보기"), ("[", "비스듬히 낮게"), ("]", "비스듬히 높게"), ("f", "따라가기"), ("e", "지도 지우기"),
        ("o", "광민 윤곽"), ("b", "덩어리 상자"), ("r", "기록"), ("s", "화면 저장"), ("g", "중력 확인"), ("q", "끝")]


class WebView:
    def __init__(self, host="127.0.0.1", port=8090, quality=80, max_width=1280):
        self.quality, self.max_width = quality, max_width
        self.jpg, self.size, self.seq = None, (1, 1), 0
        self.cond = threading.Condition()
        self.keys, self.clicks = collections.deque(maxlen=32), collections.deque(maxlen=32)
        view = self

        class H(BaseHTTPRequestHandler):
            def log_message(self, *a):
                pass

            def do_GET(self):
                u = urlparse(self.path)
                q = parse_qs(u.query)
                if u.path == "/":
                    btn = "".join(f'<button onclick="fetch(\'/key?k={k}\')">{k} · {html.escape(t)}</button> ' for k, t in KEYS)
                    body = ("<!doctype html><meta charset=utf-8><title>L515 overlay</title><body style='background:#111;color:#ddd;font-family:sans-serif;margin:8px'>"
                            f"<div>{btn}</div><img id=v src='/stream' style='max-width:100%;margin-top:6px' "
                            "onclick=\"var r=this.getBoundingClientRect();fetch('/click?x='+((event.clientX-r.left)/r.width)+'&y='+((event.clientY-r.top)/r.height))\">"
                            "<div style='font-size:12px;color:#888'>화면 클릭 = 카메라 점 거리·높이 / 지도 칸 정보 · 같은 키는 키보드로도(창이 있을 때)</div></body>")
                    b = body.encode("utf-8")
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html; charset=utf-8")
                    self.send_header("Content-Length", str(len(b)))
                    self.end_headers()
                    self.wfile.write(b)
                elif u.path == "/stream":
                    self.send_response(200)
                    self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
                    self.end_headers()
                    last = -1
                    try:
                        while True:
                            with view.cond:
                                view.cond.wait_for(lambda: view.seq != last, timeout=2.0)
                                jpg, last = view.jpg, view.seq
                            if jpg is None:
                                continue
                            self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + str(len(jpg)).encode() + b"\r\n\r\n" + jpg + b"\r\n")
                    except (BrokenPipeError, ConnectionResetError):
                        return
                elif u.path == "/key" and q.get("k"):
                    view.keys.append(ord(q["k"][0][0]))
                    self.send_response(204)
                    self.end_headers()
                elif u.path == "/click" and q.get("x") and q.get("y"):
                    w, h = view.size
                    view.clicks.append((int(float(q["x"][0]) * w), int(float(q["y"][0]) * h)))
                    self.send_response(204)
                    self.end_headers()
                else:
                    self.send_response(404)
                    self.end_headers()

        self.server = ThreadingHTTPServer((host, port), H)
        self.server.daemon_threads = True
        threading.Thread(target=self.server.serve_forever, name="webview", daemon=True).start()
        self.url = f"http://{host}:{port}/"

    def publish(self, frame):
        h, w = frame.shape[:2]
        self.size = (w, h)
        img = frame if w <= self.max_width else cv2.resize(frame, (self.max_width, int(h * self.max_width / w)))
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, self.quality])
        if ok:
            with self.cond:
                self.jpg, self.seq = buf.tobytes(), self.seq + 1
                self.cond.notify_all()

    def poll_key(self):
        return self.keys.popleft() if self.keys else 255

    def poll_click(self):
        return self.clicks.popleft() if self.clicks else None

    def close(self):
        self.server.shutdown()
