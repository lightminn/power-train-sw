#!/usr/bin/env python3
"""마스크에서 위치·각도를 뽑는 과정 도해 (자료용, 2026-08-10).

`perception_node._mask_pca_yaw_quat` 가 하는 일을 4단계로 그린다:

    ① 마스크      세그멘테이션 픽셀 (u,v) 집합. 여기서 모든 게 나온다.
    ② 공분산      centroid(=위치) 로 중심이동한 뒤 2x2 공분산. 퍼진 모양을 담는다.
    ③ 주축        고유분해 → 최대 고유값 방향이 객체의 긴 축.
    ④ yaw         주축 각도 θ=atan2(dy,dx) → optical Z 회전 quaternion.

이미지 좌표(x=오른쪽, y=아래)가 optical frame 의 X,Y 와 같은 방향이라 주축 각도가
곧 광축(Z) 기준 회전이 된다. 주축은 ±180° 모호성이 있으나(긴 축은 양방향 동일)
그리퍼 접근각에는 영향이 없다.

## 쓰는 법

    ros2 run robot_arm_perception mask_pca_explain
    ros2 run robot_arm_perception mask_pca_explain --ros-args -p autosave:=true -p show_window:=false

`s` 저장, `q`/ESC 종료.

⚠️ 그림용 중간값(cov/고유값/고유벡터)은 여기서 다시 계산하지만, **최종 각도는
   production 함수 `_mask_pca_yaw_quat` 를 그대로 호출해 얻는다.** 둘이 어긋나면
   경고를 찍는다 — 자료가 코드와 다른 걸 설명하는 사고를 막기 위해서다.
"""
import math
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node

from robot_arm_perception.model_presets import get_preset
from robot_arm_perception.perception_node import _mask_pca_yaw_quat

try:
    import pyrealsense2 as rs
except ImportError:
    sys.stderr.write("pyrealsense2 를 import 할 수 없습니다.\n")
    raise

TITLES = ("(1) mask", "(2) covariance", "(3) principal axes", "(4) yaw")
CYAN, YELLOW, GREEN, MAGENTA = (255, 220, 0), (0, 200, 255), (80, 230, 80), (255, 0, 220)
COLORS = (CYAN, YELLOW, GREEN, MAGENTA)


class MaskPcaExplain(Node):
    def __init__(self):
        super().__init__("mask_pca_explain")
        self.declare_parameter("model_name", "box")
        self.declare_parameter("conf_threshold", 0.5)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 15)
        self.declare_parameter("save_path", "mask_pca_explain.png")
        self.declare_parameter("autosave", False)
        self.declare_parameter("show_window", True)
        # depth_method_compare 와 같은 이유 — 이 모델은 화면의 28~64% 를 덮는 허위 검출을
        # 높은 confidence 로 같이 내놓는다(2026-08-10 실측). 자료용 그림엔 그게 잡히면 안 된다.
        self.declare_parameter("max_area_frac", 0.30)

        preset = get_preset(self.get_parameter("model_name").value, self.get_logger())
        self.conf = float(self.get_parameter("conf_threshold").value)
        self.save_path = self.get_parameter("save_path").value

        from ultralytics import YOLO
        self.model = YOLO(preset["model_path"], task=preset["task"])

        w = int(self.get_parameter("width").value)
        h = int(self.get_parameter("height").value)
        fps = int(self.get_parameter("fps").value)
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.color, w, h, rs.format.bgr8, fps)
        self.pipe.start(cfg)
        self.get_logger().info(f"RealSense started ({w}x{h} @ {fps}fps)")

    def _pick(self, result, shape):
        if not len(result.boxes):
            return None
        total = shape[0] * shape[1]
        cap = float(self.get_parameter("max_area_frac").value)
        confs = result.boxes.conf.cpu().numpy()
        best, best_conf = None, -1.0
        for i, box in enumerate(result.boxes.xyxy):
            x1, y1, x2, y2 = (int(v) for v in box)
            frac = max(x2 - x1, 0) * max(y2 - y1, 0) / total
            if frac <= cap and confs[i] > best_conf:
                best, best_conf = i, float(confs[i])
        return best

    @staticmethod
    def _base(img, binmask, dim=0.72):
        """마스크 밖을 어둡게 깔아 마스크가 주인공이 되게 한다."""
        out = (img.astype(np.float32) * (1.0 - dim)).astype(np.uint8)
        out[binmask] = img[binmask]
        return out

    def _panel(self, index, img, binmask, stats):
        cu, cv_, cov, evals, evecs, theta, quat = stats
        color = COLORS[index]
        canvas = self._base(img, binmask)

        if index == 0:
            overlay = canvas.copy()
            overlay[binmask] = color
            canvas = cv2.addWeighted(overlay, 0.45, canvas, 0.55, 0)
            contours, _ = cv2.findContours(binmask.astype(np.uint8),
                                           cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(canvas, contours, -1, color, 2)
            lines = [f"mask pixels (u,v): {int(binmask.sum())}"]

        elif index == 1:
            # 공분산 = 퍼진 모양. 2σ 타원으로 그리면 한눈에 보인다.
            ang = math.degrees(math.atan2(evecs[1, 1], evecs[0, 1]))
            axes = (int(2 * math.sqrt(max(evals[1], 1e-6))),
                    int(2 * math.sqrt(max(evals[0], 1e-6))))
            cv2.ellipse(canvas, (int(cu), int(cv_)), axes, ang, 0, 360, color, 2)
            cv2.drawMarker(canvas, (int(cu), int(cv_)), (255, 255, 255),
                           cv2.MARKER_CROSS, 18, 2)
            lines = [f"centroid = ({cu:.1f}, {cv_:.1f}) px   <- position",
                     f"cov = [[{cov[0,0]:7.1f} {cov[0,1]:7.1f}]",
                     f"       [{cov[1,0]:7.1f} {cov[1,1]:7.1f}]]"]

        elif index == 2:
            for k, (col, scale, label) in enumerate(
                    ((color, 2.0, "major"), ((200, 200, 200), 2.0, "minor"))):
                idx = int(np.argmax(evals)) if k == 0 else int(np.argmin(evals))
                vec = evecs[:, idx] * scale * math.sqrt(max(evals[idx], 1e-6))
                p0 = (int(cu - vec[0]), int(cv_ - vec[1]))
                p1 = (int(cu + vec[0]), int(cv_ + vec[1]))
                cv2.arrowedLine(canvas, p0, p1, col, 3, tipLength=0.12)
                cv2.putText(canvas, label, (p1[0] + 6, p1[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 1, cv2.LINE_AA)
            cv2.drawMarker(canvas, (int(cu), int(cv_)), (255, 255, 255),
                           cv2.MARKER_CROSS, 18, 2)
            # 고유값 비율이 1 에 가까우면 긴 축이 없다 = 주축 방향이 결정되지 않는다.
            # 이 비율을 안 보여주면 그림이 "각도가 잘 나온다" 는 잘못된 인상을 준다.
            ratio = math.sqrt(max(evals[1], 1e-9) / max(evals[0], 1e-9))
            lines = [f"eigenvalues: {evals[1]:.1f} (major) / {evals[0]:.1f} (minor)",
                     f"elongation = sqrt(ratio) = {ratio:.2f}"
                     + ("  <- near-circular: axis ill-defined" if ratio < 1.15 else "")]

        else:
            idx = int(np.argmax(evals))
            vec = evecs[:, idx] * 2.0 * math.sqrt(max(evals[idx], 1e-6))
            p1 = (int(cu + vec[0]), int(cv_ + vec[1]))
            ref = (int(cu + abs(vec[0]) + 40), int(cv_))
            cv2.line(canvas, (int(cu), int(cv_)), ref, (180, 180, 180), 2, cv2.LINE_AA)
            cv2.arrowedLine(canvas, (int(cu), int(cv_)), p1, color, 3, tipLength=0.12)
            radius = 46
            a0, a1 = 0.0, math.degrees(theta)
            cv2.ellipse(canvas, (int(cu), int(cv_)), (radius, radius), 0,
                        min(a0, a1), max(a0, a1), color, 2)
            cv2.putText(canvas, f"{math.degrees(theta):+.1f}",
                        (int(cu) + radius + 6, int(cv_) - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
            cv2.drawMarker(canvas, (int(cu), int(cv_)), (255, 255, 255),
                           cv2.MARKER_CROSS, 18, 2)
            ratio = math.sqrt(max(evals[1], 1e-9) / max(evals[0], 1e-9))
            lines = [f"theta = atan2(dy, dx) = {math.degrees(theta):+.2f} deg",
                     f"quat(z,w) = ({quat[2]:+.4f}, {quat[3]:+.4f})"
                     + ("   [UNRELIABLE: near-circular mask]" if ratio < 1.15 else "")]
            if ratio < 1.15:
                cv2.putText(canvas, "axis ill-defined", (int(cu) - 70, int(cv_) + 74),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 255), 2, cv2.LINE_AA)

        h, w = canvas.shape[:2]
        cv2.rectangle(canvas, (0, 0), (w, 32), (0, 0, 0), -1)
        cv2.putText(canvas, TITLES[index], (10, 23),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.62, color, 2, cv2.LINE_AA)
        cv2.rectangle(canvas, (0, h - 20 - 20 * len(lines)), (w, h), (0, 0, 0), -1)
        for i, line in enumerate(lines):
            cv2.putText(canvas, line, (10, h - 14 - 20 * (len(lines) - 1 - i)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (230, 230, 230), 1, cv2.LINE_AA)
        return canvas

    def _stats(self, binmask):
        ys, xs = np.nonzero(binmask)
        if xs.size < 10:
            return None
        pts = np.column_stack([xs.astype(np.float64), ys.astype(np.float64)])
        mean = pts.mean(axis=0)
        cov = np.cov((pts - mean).T)
        evals, evecs = np.linalg.eigh(cov)
        major = evecs[:, int(np.argmax(evals))]
        theta = math.atan2(float(major[1]), float(major[0]))

        # production 함수와 대조 — 그림이 코드와 어긋나면 자료로 못 쓴다.
        quat = _mask_pca_yaw_quat(xs, ys)
        if quat is None:
            return None
        theta_prod = 2.0 * math.atan2(quat[2], quat[3])
        if abs(math.sin(theta - theta_prod)) > 1e-6:
            self.get_logger().warn(
                f"도해 각도({math.degrees(theta):+.3f})와 production 각도"
                f"({math.degrees(theta_prod):+.3f})가 다릅니다 — 그림을 신뢰하지 마세요")
        return mean[0], mean[1], cov, evals, evecs, theta, quat

    def run(self):
        autosave = bool(self.get_parameter("autosave").value)
        show = bool(self.get_parameter("show_window").value)
        if show:
            cv2.namedWindow("mask -> pca -> yaw", cv2.WINDOW_NORMAL)
        while rclpy.ok():
            color = np.asanyarray(self.pipe.wait_for_frames().get_color_frame().get_data())
            result = self.model.predict(color, conf=self.conf, verbose=False)[0]
            best = self._pick(result, color.shape)

            stats = None
            binmask = np.zeros(color.shape[:2], dtype=bool)
            if best is not None and result.masks is not None and best < len(result.masks.xy):
                poly = np.asarray(result.masks.xy[best], dtype=np.float32)
                if poly.ndim == 2 and poly.shape[0] >= 3:
                    filled = np.zeros(color.shape[:2], dtype=np.uint8)
                    cv2.fillPoly(filled, [np.rint(poly).astype(np.int32)], 1)
                    binmask = filled.astype(bool)
                    stats = self._stats(binmask)

            if stats is None:
                canvas = color.copy()
                cv2.putText(canvas, "no mask", (20, 40), cv2.FONT_HERSHEY_SIMPLEX,
                            1.0, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                canvas = np.hstack([self._panel(i, color, binmask, stats) for i in range(4)])

            if autosave and stats is not None:
                cv2.imwrite(self.save_path, canvas)
                self.get_logger().info(f"자동 저장 후 종료: {self.save_path}")
                break
            if not show:
                continue

            cv2.imshow("mask -> pca -> yaw", canvas)
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
    node = MaskPcaExplain()
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
