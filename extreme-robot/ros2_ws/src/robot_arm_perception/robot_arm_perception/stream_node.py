#!/usr/bin/env python3
"""perception 이미지 토픽 → SRT 스트리밍 노드.

image_topic(기본 /perception/raw_image, YOLO/오버레이와 무관한 raw 프레임)을 구독해
H.264/SRT 로 송신한다. PC 에서 power-train-sw/scripts/recv_stream.sh <port> <JetsonIP>
로 수신.

2026-07-13 파워트레인 "계약 v2"(Notion `자율주행 SW 개발계획` §7.1
Task6/§15) 반영 — 포트 배정: L515 :5000, **D435i raw :5002**(본 노드 기본값),
D435i YOLO metadata UDP :5003(별도, `metadata_sender_node`). 기존 기본 포트 5000은
L515와 충돌해 5002로 변경.

로컬 디버그용으로 YOLO 오버레이(/perception/debug_image)를 스트리밍하고 싶으면
`image_topic:=/perception/debug_image`와 겹치지 않는 `port`를 지정해서 별도 실행.

실행:
    ros2 run robot_arm_perception stream_node --ros-args -p host_ip:=<젯슨IP>
    (host_ip 는 SRT listener 주소 — 기본 0.0.0.0, 포트 5002)
"""
import subprocess
import threading

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge


def _build_gst_cmd(port: int, width: int, height: int, fps: int,
                   latency_ms: int = 60, bitrate_kbps: int = 4500,
                   encoder_threads: int = 4) -> list:
    return [
        "gst-launch-1.0",
        "fdsrc", "fd=0", "do-timestamp=true",
        "!", "rawvideoparse",
             "format=bgr",
             f"width={width}", f"height={height}",
             f"framerate={fps}/1",
        "!", "videoconvert", "!", "video/x-raw,format=I420",
        "!", "x264enc", "tune=zerolatency", "speed-preset=ultrafast",
             f"threads={encoder_threads}", "sliced-threads=true",
             f"bitrate={bitrate_kbps}", "key-int-max=30",
        "!", "h264parse", "config-interval=-1",
        "!", "mpegtsmux", "alignment=7",
        "!", "srtsink",
             f"uri=srt://:{port}?mode=listener&latency={latency_ms}",
             "wait-for-connection=false", "sync=false", "async=false",
    ]


class StreamNode(Node):
    def __init__(self):
        super().__init__('stream_node')

        self.declare_parameter('port', 5002)
        self.declare_parameter('fps', 30)
        self.declare_parameter('bitrate_kbps', 4500)
        self.declare_parameter('latency_ms', 60)
        self.declare_parameter('image_topic', '/perception/raw_image')
        self.declare_parameter('encoder_threads', 4)  # Orin Nano 6코어 기준 — 나머지는 perception_node 추론용

        self._port = self.get_parameter('port').value
        self._fps = self.get_parameter('fps').value
        self._bitrate = self.get_parameter('bitrate_kbps').value
        self._latency = self.get_parameter('latency_ms').value
        self._image_topic = self.get_parameter('image_topic').value
        self._encoder_threads = self.get_parameter('encoder_threads').value

        self._bridge = CvBridge()
        self._proc = None
        self._lock = threading.Lock()
        self._width = None
        self._height = None

        self.create_subscription(ImageMsg, self._image_topic, self._cb, 1)
        self.get_logger().info(
            f'stream_node 시작 — {self._image_topic} → SRT listener :{self._port} '
            f'(PC에서: scripts/recv_stream.sh {self._port} <JetsonIP>)')

    def _ensure_proc(self, width: int, height: int):
        if (self._proc is not None
                and self._proc.poll() is None
                and self._width == width
                and self._height == height):
            return
        self._kill_proc()
        cmd = _build_gst_cmd(self._port, width, height,
                             self._fps, self._latency, self._bitrate,
                             self._encoder_threads)
        self.get_logger().info('gst-launch: ' + ' '.join(cmd))
        self._proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
        self._width, self._height = width, height

    def _kill_proc(self):
        if self._proc is not None:
            try:
                self._proc.stdin.close()
                self._proc.terminate()
                self._proc.wait(timeout=2)
            except Exception:
                try:
                    self._proc.kill()
                except Exception:
                    pass
            self._proc = None

    def _cb(self, msg: ImageMsg):
        with self._lock:
            try:
                frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().warn(f'imgmsg_to_cv2 실패: {e}')
                return

            h, w = frame.shape[:2]
            self._ensure_proc(w, h)

            if self._proc is None or self._proc.poll() is not None:
                return
            try:
                self._proc.stdin.write(frame.tobytes())
                self._proc.stdin.flush()
            except BrokenPipeError:
                self.get_logger().warn('gst 파이프 끊김 — 재시작')
                self._proc = None

    def destroy_node(self):
        self._kill_proc()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = StreamNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
