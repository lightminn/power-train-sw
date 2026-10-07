#!/usr/bin/env python3
"""align 생략 최적화 도해 — 전체 정렬이 하는 일 vs 실제로 필요한 것 (자료용, 2026-08-10).

`perception_node` 는 `rs.align`(depth 전체를 color 프레임에 정렬)을 **쓰지 않는다.**
대신 검출된 마스크에서 뽑은 소수의 점만 color→depth 픽셀로 투영한다
(`_deproject_mask`). 이 도구는 그 차이를 그림과 **이 기기에서 직접 잰 시간**으로 보여준다.

    ① full align       depth 프레임 전 픽셀을 color 로 변환. 30만 픽셀.
    ② actually needed  그중 우리가 쓰는 건 마스크 안쪽뿐.
    ③ production       마스크에서 최대 25점만 투영. 나머지는 아예 계산하지 않는다.

코드 주석의 기존 실측(Orin Nano: 5.5fps→30fps, 좌표오차 평균 11mm)은 **다른 기기**
기준이라, 이 도구는 실행하는 기기에서 다시 잰다.

## 쓰는 법

    ros2 run robot_arm_perception align_cost_explain
    ros2 run robot_arm_perception align_cost_explain --ros-args -p autosave:=true -p show_window:=false

`s` 저장, `q`/ESC 종료.
"""
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from robot_arm_perception.model_presets import get_preset
from robot_arm_perception.perception_node import DepthCal

try:
    import pyrealsense2 as rs
except ImportError:
    sys.stderr.write("pyrealsense2 를 import 할 수 없습니다.\n")
    raise

TITLES = ("(1) full align: every pixel",
          "(2) actually needed: mask only",
          "(3) production: sample N points")
COLORS = ((80, 80, 255), (0, 200, 255), (80, 230, 80))
#: production `_deproject_mask` 와 같은 값 — 거기서 바뀌면 여기도 같이 바꿔야 한다.
MAX_SAMPLES = 25


class AlignCostExplain(Node):
    def __init__(self):
        super().__init__("align_cost_explain")
        self.declare_parameter("model_name", "box")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("save_path", "align_cost_explain.png")
        self.declare_parameter("autosave", False)
        self.declare_parameter("show_window", True)
        self.declare_parameter("max_area_frac", 0.30)
        self.declare_parameter("bench_frames", 20)

        preset = get_preset(self.get_parameter("model_name").value, self.get_logger())
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.save_path = self.get_parameter("save_path").value

        from ultralytics import YOLO
        self.model = YOLO(preset["model_path"], task=preset["task"])

        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, self._w, self._h, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, self._w, self._h, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        self.cal = DepthCal(rs, profile)
        self.align = rs.align(rs.stream.color)
        self.total_px = self._w * self._h
        self.get_logger().info(
            f"RealSense started ({self._w}x{self._h}), 전체 픽셀 {self.total_px}")

        self.t_align_ms = None
        self.t_proj_ms = None

    def _pick(self, result, shape):
        if not len(result.boxes):
            return None
        cap = float(self.get_parameter("max_area_frac").value)
        total = shape[0] * shape[1]
        confs = result.boxes.conf.cpu().numpy()
        best, best_conf = None, -1.0
        for i, box in enumerate(result.boxes.xyxy):
            x1, y1, x2, y2 = (int(v) for v in box)
            if max(x2 - x1, 0) * max(y2 - y1, 0) / total <= cap and confs[i] > best_conf:
                best, best_conf = i, float(confs[i])
        return best

    @staticmethod
    def _sample_points(binmask):
        """production `_deproject_mask` 와 동일한 표본 추출 (5x5 침식 → 균등 25점)."""
        eroded = cv2.erode(binmask.astype(np.uint8), np.ones((5, 5), np.uint8), iterations=1)
        ys, xs = np.nonzero(eroded)
        if xs.size == 0:
            return []
        idx = np.linspace(0, xs.size - 1, num=min(MAX_SAMPLES, xs.size), dtype=np.int64)
        return [(float(xs[i]), float(ys[i])) for i in idx]

    def _bench(self, frames, depth_frame, points):
        """이 기기에서 두 경로의 실제 소요 시간 측정 (median)."""
        n = int(self.get_parameter("bench_frames").value)

        align_times = []
        for _ in range(n):
            t0 = time.perf_counter()
            self.align.process(frames)
            align_times.append((time.perf_counter() - t0) * 1000.0)

        proj_times = []
        data = depth_frame.get_data()
        for _ in range(n):
            t0 = time.perf_counter()
            for cx, cy in points:
                rs.rs2_project_color_pixel_to_depth_pixel(
                    data, self.cal.scale, self.cal.dmin, self.cal.dmax,
                    self.cal.di, self.cal.ci, self.cal.c2d, self.cal.d2c, [cx, cy])
            proj_times.append((time.perf_counter() - t0) * 1000.0)

        self.t_align_ms = float(np.median(align_times))
        self.t_proj_ms = float(np.median(proj_times))
        self.get_logger().info(
            f"측정({n}회 median): full align {self.t_align_ms:.2f}ms, "
            f"{len(points)}점 투영 {self.t_proj_ms:.3f}ms "
            f"→ {self.t_align_ms / max(self.t_proj_ms, 1e-6):.0f}x")

    def _panel(self, index, color, depth, binmask, points):
        color_ = COLORS[index]
        if index == 0:
            vis = cv2.applyColorMap(
                cv2.convertScaleAbs(depth, alpha=0.05), cv2.COLORMAP_JET)
            canvas = cv2.addWeighted(color, 0.35, vis, 0.65, 0)
            lines = [f"transformed: {self.total_px:,} px  (every depth pixel)",
                     f"measured: {self.t_align_ms:.2f} ms / frame"
                     if self.t_align_ms is not None else "measuring..."]
        elif index == 1:
            canvas = (color.astype(np.float32) * 0.28).astype(np.uint8)
            canvas[binmask] = color[binmask]
            overlay = canvas.copy()
            overlay[binmask] = color_
            canvas = cv2.addWeighted(overlay, 0.30, canvas, 0.70, 0)
            used = int(binmask.sum())
            lines = [f"mask: {used:,} px  ({used / self.total_px * 100:.1f}% of frame)",
                     "the rest of the align result is discarded"]
        else:
            canvas = (color.astype(np.float32) * 0.28).astype(np.uint8)
            canvas[binmask] = color[binmask]
            contours, _ = cv2.findContours(binmask.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(canvas, contours, -1, (120, 120, 120), 1)
            for cx, cy in points:
                cv2.circle(canvas, (int(cx), int(cy)), 5, color_, -1)
                cv2.circle(canvas, (int(cx), int(cy)), 5, (255, 255, 255), 1)
            speed = (self.t_align_ms / max(self.t_proj_ms, 1e-6)
                     if self.t_align_ms and self.t_proj_ms else None)
            lines = [f"transformed: {len(points)} px  (sampled from eroded mask)",
                     f"measured: {self.t_proj_ms:.3f} ms"
                     + (f"   -> {speed:,.0f}x faster" if speed else "")
                     if self.t_proj_ms is not None else "measuring..."]

        h, w = canvas.shape[:2]
        cv2.rectangle(canvas, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(canvas, TITLES[index], (10, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.58, color_, 2, cv2.LINE_AA)
        cv2.rectangle(canvas, (0, h - 20 - 20 * len(lines)), (w, h), (0, 0, 0), -1)
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (10, h - 14 - 20 * (len(lines) - 1 - i)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        return canvas

    def run(self):
        autosave = bool(self.get_parameter("autosave").value)
        show = bool(self.get_parameter("show_window").value)
        if show:
            cv2.namedWindow("align cost", cv2.WINDOW_NORMAL)
        benched = False
        while rclpy.ok():
            frames = self.pipe.wait_for_frames()
            cframe, dframe = frames.get_color_frame(), frames.get_depth_frame()
            if not cframe or not dframe:
                continue
            color = np.asanyarray(cframe.get_data())
            depth = np.asanyarray(dframe.get_data())

            result = self.model.predict(color, conf=self.conf, verbose=False)[0]
            best = self._pick(result, color.shape)
            binmask = np.zeros(color.shape[:2], dtype=bool)
            if best is not None and result.masks is not None and best < len(result.masks.xy):
                poly = np.asarray(result.masks.xy[best], dtype=np.float32)
                if poly.ndim == 2 and poly.shape[0] >= 3:
                    filled = np.zeros(color.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(filled, [np.rint(poly).astype(np.int32)], 1)
                    binmask = filled.astype(bool)

            points = self._sample_points(binmask) if binmask.any() else []
            if not points:
                canvas = color.copy()
                cv2.putText(canvas, "no mask", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                if not benched:
                    self._bench(frames, dframe, points)
                    benched = True
                canvas = np.hstack(
                    [self._panel(i, color, depth, binmask, points) for i in range(3)])

                if autosave:
                    cv2.imwrite(self.save_path, canvas)
                    self.get_logger().info(f"자동 저장 후 종료: {self.save_path}")
                    break

            if not show:
                continue
            cv2.imshow("align cost", canvas)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("s"):
                cv2.imwrite(self.save_path, canvas)
                self.get_logger().info(f"저장: {self.save_path}")

    def destroy_node(self):
        self.pipe.stop()
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AlignCostExplain()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
