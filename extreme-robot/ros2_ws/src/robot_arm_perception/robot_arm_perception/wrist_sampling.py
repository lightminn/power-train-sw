"""`/wrist/metrics` 표본 수집기 — 실측 스크립트 2종이 공유한다 (2026-08-13 추가).

`measure_wrist_proximity.py`(거리 곡선)와 `measure_wrist_grasp_band.py`(파지 밴드)가
같은 토픽을 같은 방식으로 읽는다. 수집을 각자 구현하면 **한쪽만 필터를 고쳐 두 실측이
서로 다른 표본을 쓰게 되는데**, 그 둘로 뽑은 임계값은 같은 축 위에 있지 않다.

⚠️ 이 모듈은 ROS 에 의존한다(`wrist_metrics`/`wrist_color_mask` 와 다르다). 계산은
전부 `wrist_metrics` 의 순수 함수가 하고, 여기는 **모으는 일만** 한다.
"""
import json
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TOPIC_METRICS = '/wrist/metrics'


class MetricsCollector(Node):
    """`/wrist/metrics`(JSON) 를 받아 최신값과 표본 묶음을 넘겨준다."""

    def __init__(self, topic=TOPIC_METRICS):
        super().__init__('wrist_metrics_collector')
        self._lock = threading.Lock()
        self._latest = None
        self._seq = 0
        self.create_subscription(String, topic, self._on_metrics, 10)
        self.topic = topic

    def _on_metrics(self, msg):
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        with self._lock:
            self._seq += 1
            self._latest = (self._seq, payload)

    def latest(self):
        with self._lock:
            return None if self._latest is None else dict(self._latest[1])

    def collect(self, count, timeout_s=20.0, on_progress=None):
        """**새로** 들어오는 표본 `count` 개를 모은다.

        ⚠️ 이미 받아 둔 값을 재사용하지 않는다(seq 로 새 프레임만 센다) — 자를 대고
        상자를 옮긴 직후에 옛 프레임이 섞이면 그 점 하나가 거리 곡선을 통째로 휜다.

        반환: `(표본 리스트, 타임아웃 여부)`.
        """
        samples = []
        with self._lock:
            last_seq = self._seq
        deadline = time.time() + float(timeout_s)
        while len(samples) < int(count):
            if not rclpy.ok():
                break
            if time.time() > deadline:
                return samples, True
            with self._lock:
                current = self._latest
                seq = self._seq
            if current is not None and seq != last_seq:
                last_seq = seq
                samples.append(dict(current[1]))
                if on_progress is not None:
                    on_progress(len(samples), int(count))
            else:
                time.sleep(0.01)
        return samples, False


def spin_in_background(node):
    """`input()` 으로 사람을 기다리는 동안에도 구독이 살아 있어야 한다.

    실측 스크립트는 전부 대화형이라 메인 스레드가 `input()` 에 막힌다 — spin 을 그
    스레드에 두면 그동안 토픽이 한 건도 안 들어온다.
    """
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    return thread


def shutdown(node, thread=None, timeout_s=2.0):
    """spin 스레드를 먼저 세우고 나서 노드를 파괴한다.

    ⚠️ 순서가 반대면(스핀 도중 `destroy_node()`) rclpy 가 `terminate called without an
    active exception` + core dump 로 죽는다. 결과를 다 출력한 뒤에 나는 크래시라 값은
    멀쩡한데, 그걸 본 사람은 **실측이 실패한 줄 안다.**
    """
    if rclpy.ok():
        rclpy.shutdown()                    # spin() 이 여기서 반환된다
    if thread is not None:
        thread.join(timeout=float(timeout_s))
    node.destroy_node()
