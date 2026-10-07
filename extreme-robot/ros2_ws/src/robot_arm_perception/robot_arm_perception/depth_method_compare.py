#!/usr/bin/env python3
"""depth 추출 방식 3가지를 한 화면에 나란히 보여주는 비교 뷰어 (자료용, 2026-08-10).

## 왜 만들었나

`perception_node` 의 markerless pose 는 **마스크 내부 median** 으로 거리를 잡는다.
왜 그 방식인지를 설명하려면 대안과 나란히 놓고 보여주는 게 제일 빠르다 — 특히
`estimate_position` 주석에 남아 있는 실패 이력(단일 centroid 패치가 배경 depth 를
섞어 D435 근접 타겟을 오측정, 2026-07-28 수정)을 눈으로 보여줄 수 있다.

    ① bbox 중심 1픽셀   — 표본 1개. 그 픽셀이 구멍(0)이거나 배경이면 그대로 틀린다.
    ② bbox 내부 평균    — 표본은 많지만 bbox 는 사각형이라 **배경이 섞인다.**
                          평균이라 이상치 한 방에 끌려간다.
    ③ 마스크 내부 median — 물체 픽셀만 고르고, median 이라 이상치에 안 끌린다.
                          (production 은 여기에 더해 마스크 침식 + MAD 이상치 제거를 한다)

## 쓰는 법

    ros2 run robot_arm_perception depth_method_compare
    ros2 run robot_arm_perception depth_method_compare --ros-args -p save_path:=/root/out.png

`s` 키로 저장, `q`/ESC 로 종료. 창이 필요하므로 DISPLAY 가 있어야 한다.

⚠️ 이 도구는 **depth 를 color 에 align** 해서 쓴다(`rs.align`). production 인
`perception_node` 는 align 없이 검출별로 color→depth 픽셀 투영을 한다 — 그쪽이 더
정확하지만 코드가 길다. 여기서 비교하려는 건 **어느 영역을 표본으로 쓰는가** 이고
그 결론은 align 여부와 무관하므로, 그림이 단순해지는 쪽을 택했다.
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from robot_arm_perception.model_presets import get_preset

try:
    import pyrealsense2 as rs
except ImportError:
    sys.stderr.write("pyrealsense2 를 import 할 수 없습니다.\n")
    raise

PANEL_TITLES = (
    "(1) bbox center 1px",
    "(2) bbox interior mean",
    "(3) mask interior median",
)
# BGR. 방식별로 색을 다르게 해 캡션과 오버레이가 짝지어 보이게 한다.
PANEL_COLORS = ((60, 60, 255), (0, 190, 255), (80, 220, 80))


class DepthMethodCompare(Node):
    def __init__(self):
        super().__init__("depth_method_compare")
        self.declare_parameter("model_name", "box")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("save_path", "depth_method_compare.png")
        self.declare_parameter("patch", 5)   # ①의 단일 픽셀 주변 표시용 십자 크기
        # 검출이 잡힌 첫 프레임을 자동 저장하고 종료한다. 창을 못 띄우는 환경(SSH 등)에서
        # 그림만 뽑거나, 렌더 결과를 확인할 때 쓴다.
        self.declare_parameter("autosave", False)
        self.declare_parameter("show_window", True)
        # 이 비율보다 큰 검출은 후보에서 뺀다.
        #
        # ⚠️ 2026-08-10 실측: 이 모델은 실제 박스(conf 0.93, 화면의 17%) 말고도 **화면의
        #    28~39% 를 덮는 허위 검출**을 같이 내놓고, 프레임에 따라 그쪽 confidence 가
        #    더 높아지기도 한다. confidence 최고만 고르면 자료용 그림에 그 허위 검출이
        #    잡혀 세 방식 비교가 무의미해진다(실제로 그렇게 나왔다).
        #    50cm 안팎 거리의 타겟이 화면 1/3 을 넘게 채울 일은 없으므로 면적으로 거른다.
        self.declare_parameter("max_area_frac", 0.30)

        preset = get_preset(self.get_parameter("model_name").value, self.get_logger())
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.save_path = self.get_parameter("save_path").value

        from ultralytics import YOLO
        self.model = YOLO(preset["model_path"], task=preset["task"])
        self.get_logger().info(f"YOLO loaded: {preset['model_path']} task={preset['task']}")

        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        cfg.enable_stream(rs.stream.depth, w, h, rs.format.z16, fps)
        profile = self.pipe.start(cfg)
        self.scale = profile.get_device().first_depth_sensor().get_depth_scale()
        self.align = rs.align(rs.stream.color)
        self.get_logger().info(f"RealSense started ({w}x{h} @ {fps}fps), depth scale={self.scale}")

    # ── 방식별 계산 ────────────────────────────────────────────
    def _center_pixel(self, depth, box):
        x1, y1, x2, y2 = box
        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
        raw = depth[cy, cx]
        # 0 은 "측정 실패(구멍)" 이다. 그걸 0m 로 쓰면 안 되므로 실패로 표시한다.
        value = raw * self.scale if raw > 0 else None
        return value, (cx, cy), (1 if raw > 0 else 0)

    def _bbox_mean(self, depth, box):
        x1, y1, x2, y2 = box
        roi = depth[y1:y2, x1:x2]
        valid = roi[roi > 0]
        if valid.size == 0:
            return None, 0
        return float(valid.mean()) * self.scale, int(valid.size)

    def _mask_median(self, depth, binmask):
        valid = depth[binmask & (depth > 0)]
        if valid.size == 0:
            return None, 0
        return float(np.median(valid)) * self.scale, int(valid.size)

    # ── 렌더 ──────────────────────────────────────────────────
    def _panel(self, base, index, box, binmask, value, n, extra=None):
        img = base.copy()
        color = PANEL_COLORS[index]
        x1, y1, x2, y2 = box

        if index == 0:
            cx, cy = extra
            k = int(self.get_parameter("patch").value)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            cv2.line(img, (cx - k * 3, cy), (cx + k * 3, cy), color, 2)
            cv2.line(img, (cx, cy - k * 3), (cx, cy + k * 3), color, 2)
            cv2.circle(img, (cx, cy), 3, (255, 255, 255), -1)
        elif index == 1:
            overlay = img.copy()
            cv2.rectangle(overlay, (x1, y1), (x2, y2), color, -1)
            img = cv2.addWeighted(overlay, 0.35, img, 0.65, 0)
            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        else:
            overlay = img.copy()
            overlay[binmask] = color
            img = cv2.addWeighted(overlay, 0.45, img, 0.55, 0)
            contours, _ = cv2.findContours(binmask.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(img, contours, -1, color, 2)

        h, w = img.shape[:2]
        cv2.rectangle(img, (0, 0), (w, 34), (0, 0, 0), -1)
        cv2.putText(img, PANEL_TITLES[index], (10, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)

        cv2.rectangle(img, (0, h - 78), (w, h), (0, 0, 0), -1)
        text = "N/A (no depth)" if value is None else f"{value * 100:.1f} cm"
        cv2.putText(img, text, (12, h - 30), cv2.FONT_HERSHEY_SIMPLEX,
                    1.25, color, 3, cv2.LINE_AA)
        cv2.putText(img, f"samples: {n}", (12, h - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA)
        return img

    def _pick_detection(self, result, shape):
        """면적 상한을 통과한 검출 중 confidence 최고. 없으면 None.

        고른 이유를 로그로 남긴다 — 자료용 그림에 엉뚱한 검출이 잡혔을 때
        "왜 저게 잡혔지" 를 사후에 알 수 없으면 곤란하다.
        """
        if not len(result.boxes):
            return None
        total = shape[0] * shape[1]
        cap = float(self.get_parameter("max_area_frac").value)
        confs = result.boxes.conf.cpu().numpy()

        rows, best, best_conf = [], None, -1.0
        for i, box in enumerate(result.boxes.xyxy):
            x1, y1, x2, y2 = (int(v) for v in box)
            frac = max(x2 - x1, 0) * max(y2 - y1, 0) / total
            ok = frac <= cap
            rows.append(f"[{i}] conf={confs[i]:.3f} area={frac * 100:.1f}%"
                        f"{'' if ok else ' (면적초과-제외)'}")
            if ok and confs[i] > best_conf:
                best, best_conf = i, float(confs[i])

        if best is None:
            self.get_logger().warn(
                f"면적 {cap * 100:.0f}% 이하 검출이 없습니다 — " + ", ".join(rows))
        elif len(rows) > 1:
            self.get_logger().info(f"검출 {len(rows)}개 → [{best}] 선택. " + ", ".join(rows))
        return best

    def run(self):
        autosave = bool(self.get_parameter("autosave").value)
        show = bool(self.get_parameter("show_window").value)
        if show:
            cv2.namedWindow("depth method compare", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            frames = self.align.process(self.pipe.wait_for_frames())
            cframe, dframe = frames.get_color_frame(), frames.get_depth_frame()
            if not cframe or not dframe:
                continue
            color = np.asanyarray(cframe.get_data())
            depth = np.asanyarray(dframe.get_data())

            result = self.model.predict(color, conf=self.conf, verbose=False)[0]
            panels = None

            best = self._pick_detection(result, color.shape)
            if best is not None:
                x1, y1, x2, y2 = (int(v) for v in result.boxes.xyxy[best])
                x1, y1 = max(x1, 0), max(y1, 0)
                x2, y2 = min(x2, color.shape[1]), min(y2, color.shape[0])
                box = (x1, y1, x2, y2)

                binmask = np.zeros(color.shape[:2], dtype=bool)
                if result.masks is not None and best < len(result.masks.xy):
                    poly = np.asarray(result.masks.xy[best], dtype=np.float32)
                    if poly.ndim == 2 and poly.shape[0] >= 3:
                        filled = np.zeros(color.shape[:2], dtype=np.uint8)
                        cv2.fillPoly(filled, [np.rint(poly).astype(np.int32)], 1)
                        binmask = filled.astype(bool)

                v1, center, n1 = self._center_pixel(depth, box)
                v2, n2 = self._bbox_mean(depth, box)
                v3, n3 = self._mask_median(depth, binmask)

                panels = [
                    self._panel(color, 0, box, binmask, v1, n1, extra=center),
                    self._panel(color, 1, box, binmask, v2, n2),
                    self._panel(color, 2, box, binmask, v3, n3),
                ]

            if panels is None:
                canvas = color.copy()
                cv2.putText(canvas, "no detection", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                canvas = np.hstack(panels)

            if autosave and panels is not None:
                cv2.imwrite(self.save_path, canvas)
                self.get_logger().info(f"자동 저장 후 종료: {self.save_path}")
                break

            if not show:
                continue

            cv2.imshow("depth method compare", canvas)
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
    node = DepthMethodCompare()
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
