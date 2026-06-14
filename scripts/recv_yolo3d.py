#!/usr/bin/env python3
"""노트북 측 수신기 ② — 좌표 오버레이 뷰어 (SRT 영상 + UDP 좌표 JSON).

용도: 검출 결과(박스·3D 좌표)를 영상 위에 보며 확인 — 정밀 접근·좌표 점검.
파이썬/cv2 로 합성하므로 ①번 recv_stream.sh(네이티브)보다 표시 지연이
~수십 ms 더 크다. 단순 원격주행처럼 저지연이 우선이면 ①번을 쓴다.

송신측은 깨끗한 color 만 보내고 검출 좌표는 별도 UDP 채널로 오므로, 여기서
OpenCV 로 박스/라벨을 그린다 — 오버레이가 압축 손실을 안 타고, 좌표를
데이터로도 쓸 수 있다. 영상이 막혀도 좌표 패킷(작음)은 살아 있어 연막
구간 등에서 주 정보원이 된다.

sync 는 시간 기반 느슨한 동기: 디코드된 최신 프레임 위에 최신 좌표 패킷을
그린다. H.264 스트림에 frame id 를 싣지 않으므로 프레임 단위 정밀 매칭은
하지 않는다 — 검출 15~30fps 기준 어긋남은 1~2프레임으로 조종 용도에 충분.
좌표 패킷이 0.7s 이상 늙으면 STALE 경고를 띄운다.

사용 (노트북, conda base 또는 cv2 있는 환경):
    python3 scripts/recv_yolo3d.py                       # jetson-orin.local 접속
    python3 scripts/recv_yolo3d.py --host 192.168.50.x   # IP 직접 지정
종료: 영상 창에서 q 또는 ESC. 영상이 끊기면 자동 재접속한다.
"""
import argparse
import json
import os
import select
import signal
import socket
import subprocess
import sys
import threading
import time

import cv2
import numpy as np

GREEN = (0, 255, 0)
RED = (0, 0, 255)
ORANGE = (0, 165, 255)
GRAY = (160, 160, 160)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="jetson-orin.local",
                   help="Jetson 주소 (SRT listener)")
    p.add_argument("--port", type=int, default=5000, help="영상 SRT 포트")
    p.add_argument("--coord-port", type=int, default=5001,
                   help="좌표 JSON UDP 수신 포트")
    p.add_argument("--width", type=int, default=848,
                   help="송신 영상 해상도 (송신측과 일치해야 함). 848x480=16:9, "
                        "4:3 이면 640")
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--scale", type=float, default=1.8,
                   help="표시 창 확대 배율 (영상은 640x480, 창만 키움). "
                        "창 모서리를 끌어 자유 조절 가능")
    p.add_argument("--latency", type=int, default=60,
                   help="SRT 재전송 지연 예산 (ms). 송신측 --srt-latency 와 "
                        "큰 값으로 협상되므로 같이 맞춰야 함")
    p.add_argument("--stall-timeout", type=float, default=4.0,
                   help="이 시간(s) 동안 영상 바이트가 안 오면 gst 를 죽이고 "
                        "재접속 — srtsrc caller 는 접속 실패/링크 사망 시 EOF "
                        "없이 무한 대기하므로 워치독 필수")
    p.add_argument("--headless", action="store_true",
                   help="창 없이 수신 통계만 출력 (자동 테스트용)")
    p.add_argument("--clock", action="store_true",
                   help="화면 우상단에 노트북 현재시각 표시 — 영상 속 송신 tx-stamp "
                        "와 비교해 종단 지연 측정용 (송신측 --tx-stamp 와 함께)")
    p.add_argument("--max-frames", type=int, default=0,
                   help="0=무한, >0=N프레임 후 종료 (테스트용)")
    return p.parse_args()


def resolve_ipv4(host: str) -> str:
    """host 를 IPv4 로 강제 해석. 실패하면 원본 그대로.

    `jetson-orin.local` 같은 mDNS 이름은 IPv6 link-local(fe80::…)로 먼저
    풀리는데, gst SRT URI 는 scope id(%wlan0)를 못 실어 접속이 무한 실패한다
    (HIL 에서 확인). getaddrinfo 로 A 레코드만 뽑아 이 함정을 피한다.
    """
    try:
        infos = socket.getaddrinfo(host, None, socket.AF_INET)
        return infos[0][4][0]
    except (socket.gaierror, IndexError):
        return host


def build_recv_command(host: str, port: int, w: int, h: int,
                       latency: int) -> list:
    """SRT → H.264 디코드 → raw BGR 를 stdout 으로 출력하는 gst argv.

    videoscale + caps 강제: 송신 해상도가 달라도 프레임 크기를 고정해
    stdout 바이트 파싱이 깨지지 않게 한다 (박스 좌표는 패킷의 w/h 로 스케일).
    avdec max-threads=1: 멀티스레드 디코딩의 프레임 개수 고정지연 회피.
    """
    host = resolve_ipv4(host)
    return [
        "gst-launch-1.0", "-q",
        "srtsrc", f"uri=srt://{host}:{port}?mode=caller&latency={latency}",
        "!", "tsdemux", "!", "h264parse",
        "!", "avdec_h264", "max-threads=1",
        "!", "videoconvert", "!", "videoscale",
        "!", f"video/x-raw,format=BGR,width={w},height={h}",
        "!", "fdsink", "fd=1",
    ]


class CoordReceiver(threading.Thread):
    """UDP 좌표 패킷 수신 데몬 — 최신 패킷과 도착 시각만 유지."""

    def __init__(self, port: int):
        super().__init__(daemon=True)
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(("0.0.0.0", port))
        self._sock.settimeout(0.5)
        self._lock = threading.Lock()
        self._pkt: dict | None = None
        self._rx_t = 0.0
        self.start()

    def run(self) -> None:
        while True:
            try:
                data, _ = self._sock.recvfrom(65535)
                pkt = json.loads(data)
            except socket.timeout:
                continue
            except (json.JSONDecodeError, OSError):
                continue
            with self._lock:
                self._pkt, self._rx_t = pkt, time.time()

    def latest(self) -> tuple:
        """(패킷 or None, 도착 후 경과초). 경과는 수신측 시계 기준 — 기기 간
        시계 오프셋을 타지 않는다."""
        with self._lock:
            if self._pkt is None:
                return None, float("inf")
            return self._pkt, time.time() - self._rx_t


def det_lines(d: dict) -> list:
    """송신측 --draw 와 동일한 3줄 라벨 포맷 (조작감 일관성)."""
    if d.get("xyz") is not None:
        X, Y, Z = d["xyz"]
        return [f"{d['cls']} d={d['d']:.2f}m",
                f"X{X:+.2f} Y{Y:+.2f} Z{Z:+.2f}m",
                f"az{d['az']:+.1f} el{d['el']:+.1f}"]
    return [f"{d['cls']} no-depth"]


def draw_overlay(frame: np.ndarray, pkt: dict | None, age: float) -> None:
    h, w = frame.shape[:2]
    if pkt is None:
        cv2.putText(frame, "coords: waiting...", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, GRAY, 2)
        return
    stale = age > 0.7
    sx, sy = w / max(pkt.get("w", w), 1), h / max(pkt.get("h", h), 1)
    for d in pkt.get("dets", []):
        x1, y1, x2, y2 = (int(v * s) for v, s in
                          zip(d["box"], (sx, sy, sx, sy)))
        color = ORANGE if stale else GREEN
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.circle(frame, ((x1 + x2) // 2, (y1 + y2) // 2), 3, RED, -1)
        for li, txt in enumerate(det_lines(d)):
            ty = y1 - 8 - 18 * (len(det_lines(d)) - 1 - li)
            if ty < 14:  # 박스가 화면 상단이면 박스 안쪽에
                ty = y1 + 18 * (li + 1)
            cv2.putText(frame, txt, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, (0, 0, 0), 3)
            cv2.putText(frame, txt, (x1, ty), cv2.FONT_HERSHEY_SIMPLEX,
                        0.48, color, 1)
    if stale:
        cv2.putText(frame, f"coords STALE {age:.1f}s", (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, ORANGE, 2)


def read_frame(fd: int, nbytes: int, stall_timeout: float) -> bytes | None:
    """fd 에서 정확히 한 프레임 분량 읽기. EOF 또는 stall_timeout 초과 시 None.

    select 기반 — srtsrc 는 접속 실패·링크 사망 시 EOF 를 안 주고 멈추므로
    블로킹 read 만으로는 재접속 루프가 영영 안 돈다.
    """
    chunks = []
    remain = nbytes
    while remain > 0:
        ready, _, _ = select.select([fd], [], [], stall_timeout)
        if not ready:
            return None  # stall — 호출측이 gst 재기동
        chunk = os.read(fd, remain)
        if not chunk:
            return None  # EOF
        chunks.append(chunk)
        remain -= len(chunk)
    return b"".join(chunks)


def main() -> None:
    a = parse_args()
    # SIGTERM 에도 finally(gst 자식 종료)가 돌도록 — 안 하면 미접속 상태로
    # 죽을 때 gst caller 가 고아로 남아 다음 송신 listener 를 오염시킨다.
    signal.signal(signal.SIGTERM, lambda *_: sys.exit(143))
    coords = CoordReceiver(a.coord_port)
    frame_bytes = a.width * a.height * 3
    win = "yolo_depth_3d (SRT)"
    total = 0
    quit_req = False

    if not a.headless:
        # WINDOW_NORMAL: 창을 자유 리사이즈 가능 + OpenGL 로 내용 스케일.
        # 640x480 1:1 로 띄우면 고해상도 모니터에서 손톱만 하게 보이므로
        # 기본 배율만큼 키워 연다 (영상 원본은 그대로, 창만 확대).
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, int(a.width * a.scale), int(a.height * a.scale))

    while not quit_req:  # 영상 연결이 끊기면 재접속 (좌표 수신은 계속 유지)
        cmd = build_recv_command(a.host, a.port, a.width, a.height, a.latency)
        print("[gst-launch]", " ".join(cmd), file=sys.stderr)
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
        fd = proc.stdout.fileno()
        t_conn = time.time()
        n = 0
        try:
            while True:
                buf = read_frame(fd, frame_bytes, a.stall_timeout)
                if buf is None:
                    break  # 스트림 종료/끊김/stall → 재접속
                frame = np.frombuffer(buf, np.uint8).reshape(
                    a.height, a.width, 3).copy()
                pkt, age = coords.latest()
                n += 1
                total += 1
                if a.headless:
                    if n % 30 == 0:
                        fps = n / max(time.time() - t_conn, 1e-6)
                        ndet = len(pkt["dets"]) if pkt else -1
                        # flush: 리다이렉트 시 블록버퍼링으로 로그가 안 보이는 것 방지
                        print(f"[{total:5d}] fps={fps:4.1f} "
                              f"coord_age={age:5.2f}s dets={ndet}", flush=True)
                else:
                    draw_overlay(frame, pkt, age)
                    if a.clock:
                        # 노트북 현재시각(송신측 tx-stamp 와 같은 mod100 포맷)을
                        # 화면 우상단에 굵게 — 영상의 tx 값과 비교해 종단 지연을
                        # 직접 읽는다. latency = laptop − tx (시계 오프셋 ~0).
                        ct = f"laptop {time.time() % 100:06.2f}"
                        cv2.putText(frame, ct, (frame.shape[1] - 360, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (0, 0, 0), 6)
                        cv2.putText(frame, ct, (frame.shape[1] - 360, 44),
                                    cv2.FONT_HERSHEY_SIMPLEX, 1.1, (255, 0, 255), 2)
                    cv2.imshow(win, frame)
                    if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                        quit_req = True
                        break
                if a.max_frames and total >= a.max_frames:
                    quit_req = True
                    break
        except KeyboardInterrupt:
            quit_req = True
        finally:
            proc.terminate()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                proc.kill()
        if not quit_req:
            why = "영상 끊김" if n else "송신자 대기"
            print(f"[recv] {why} (frames={n}) — 1초 후 재접속...",
                  file=sys.stderr)
            time.sleep(1.0)

    if not a.headless:
        cv2.destroyAllWindows()
    print(f"[summary] total_frames={total}")


if __name__ == "__main__":
    main()
