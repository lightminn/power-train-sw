#!/usr/bin/env python3
"""영상 소스 허브 — 동적 구독 + **단일** JPEG 인코더.

## 왜 동적 구독인가 — 그리고 두 소스의 비용이 다르다

`perception_node` 는 `/perception/debug_image` 를 **구독자가 있을 때만** 만든다
(`if self.pub_debug.get_subscription_count() > 0:` → `color_img.copy()` +
`_draw_debug` + `cv2_to_imgmsg`). 즉 GUI 가 붙어 있기만 해도 Jetson 에 오버레이
그리기 비용이 추론 주기마다 계속 붙는다. 그래서 브라우저 영상 탭이 꺼져 있으면
**구독 자체를 파괴**해 발행을 멈추게 한다.

반면 `/perception/raw_image` 는 **게이트가 없다** — 캡처 스레드가 프레임을 받는
즉시 무조건 발행한다(추론과 분리된 이유가 "raw sender 는 YOLO/debug 를 기다리지
않는다" 이다). 따라서 raw 를 구독해도 인식 노드의 일은 **하나도 늘지 않고**,
DDS 전송분만 생긴다. 그래도 동적 구독 대상에 함께 두는 이유는 대역폭(JPEG 인코딩
+ 네트워크)은 여전히 우리 쪽 비용이기 때문이다.

정리하면 — **debug = 오버레이가 필요할 때만, raw = 평소에 켜둬도 되는 쪽**이다.

구독 생성/파괴는 HTTP 스레드가 직접 하지 않는다(rclpy 엔티티 조작은 실행기
스레드에서 하는 게 안전하다). HTTP 스레드는 `acquire`/`release` 로 refcount 만
건드리고, 노드의 2Hz 조정 타이머가 `desired()` 와 실제 구독을 맞춘다.

## 왜 인코더가 하나인가

클라이언트마다 인코딩하면 N배 비용이다. 프레임이 들어올 때 **한 번만** JPEG 로
만들어 공유 슬롯에 넣고, HTTP 스레드들은 그 bytes 를 쓰기만 한다.
`fps` 상한을 넘는 프레임은 인코딩 없이 버린다.
"""

import threading


#: 소스 이름 → 토픽. 'off' 는 아무것도 구독하지 않는다는 뜻.
SOURCES = {
    'debug': '/perception/debug_image',   # bbox·마스크·거리 오버레이 (인식 노드에 비용 발생)
    'raw': '/perception/raw_image',       # 원본 (오버레이 비용 없음)
    # 손목 캠(`wrist_camera` 노드). 안 띄워져 있으면 그냥 프레임이 안 올 뿐이라
    # 나머지 소스에 영향이 없다 — 구독은 refcount 로 동적 생성/파괴된다.
    'wrist': '/wrist/raw_image',          # 원본 (손목 노드도 게이트 없이 발행)
    'wrist_debug': '/wrist/debug_image',  # 오버레이 (구독자가 있을 때만 그린다)
}


class VideoHub:
    """소스별 refcount + 최신 JPEG 슬롯."""

    def __init__(self, fps=10.0, quality=70):
        self.fps = float(fps)
        self.quality = int(quality)
        self._cond = threading.Condition()
        self._refs = {name: 0 for name in SOURCES}
        self._slots = {
            name: {'seq': 0, 'jpeg': None, 'at': None, 'shape': None, 'dropped': 0}
            for name in SOURCES
        }
        self._last_encode = {name: 0.0 for name in SOURCES}

    # ------------------------------------------------------------ refcount
    def acquire(self, source):
        """HTTP 스레드용. 실제 구독은 노드 타이머가 만든다."""
        if source not in SOURCES:
            return False
        with self._cond:
            self._refs[source] += 1
        return True

    def release(self, source):
        with self._cond:
            if source in self._refs and self._refs[source] > 0:
                self._refs[source] -= 1

    def desired(self):
        """지금 구독돼 있어야 하는 소스 집합."""
        with self._cond:
            return {name for name, n in self._refs.items() if n > 0}

    def clients(self):
        with self._cond:
            return dict(self._refs)

    # ------------------------------------------------------------ 프레임
    def offer(self, source, bgr, now):
        """ROS 콜백에서 호출. fps 상한을 넘으면 **인코딩 없이** 버린다."""
        import cv2

        with self._cond:
            if self._refs.get(source, 0) <= 0:
                return False
            interval = 1.0 / self.fps if self.fps > 0 else 0.0
            if now - self._last_encode[source] < interval:
                self._slots[source]['dropped'] += 1
                return False
            self._last_encode[source] = now

        ok, buf = cv2.imencode('.jpg', bgr, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
        if not ok:
            return False
        data = buf.tobytes()

        with self._cond:
            slot = self._slots[source]
            slot['seq'] += 1
            slot['jpeg'] = data
            slot['at'] = now
            slot['shape'] = [int(bgr.shape[1]), int(bgr.shape[0])]
            self._cond.notify_all()
        return True

    def wait_frame(self, source, last_seq, timeout=1.0):
        """새 프레임을 기다린다. 없으면 (last_seq, None).

        조건변수라 폴링 지연이 없고, 타임아웃이 있어야 클라이언트가 끊겼을 때
        스레드가 영원히 잠들지 않는다.
        """
        with self._cond:
            slot = self._slots.get(source)
            if slot is None:
                return last_seq, None
            if slot['seq'] == last_seq:
                self._cond.wait(timeout)
            if slot['seq'] == last_seq:
                return last_seq, None
            return slot['seq'], slot['jpeg']

    def stats(self):
        with self._cond:
            return {
                name: {'clients': self._refs[name], 'seq': s['seq'],
                       'shape': s['shape'], 'dropped': s['dropped'],
                       'bytes': len(s['jpeg']) if s['jpeg'] else 0}
                for name, s in self._slots.items()
            }
