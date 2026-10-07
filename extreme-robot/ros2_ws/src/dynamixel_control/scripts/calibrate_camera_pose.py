#!/usr/bin/env python3
"""박스를 아는 위치 여러 곳에 놓고 카메라 TF(base_link→camera_link)를 푼다 (2026-08-07).

## 왜 필요한가

`camera_tf.launch.py` 의 오프셋을 줄자로 재고 "카메라가 그리퍼를 본다" 같은 가정으로
방위를 추정하면 오차가 크다 — 2026-08-07 실측에서 단일 대응점으로 맞췄더니 전후/높이는
1cm 안쪽으로 들어왔지만 **좌우가 4~6cm 틀렸다.**

이 팔은 `arm_joint_1`(유일한 요축)에 모터가 없어 **평면 로봇**이다. 즉 좌우로 못
비킨다 — 좌우 오차는 IK 가 흡수할 수 없고 그대로 파지 실패가 된다(analytic IK 의
`ik_accept_tol` 기본 3cm 를 넘으면 아예 실패로 떨어진다). 그래서 좌우 정확도가
특히 중요하다.

대응점 3개 이상을 모아 최소자승으로 카메라 자세 6-DOF 를 한 번에 푸는 게 정답이다.

## 쓰는 법

`perception_node` 가 박스를 검출하고 있어야 한다. **카메라 TF 가 안 떠 있어도 된다**
— 이 도구는 optical frame 원본 좌표만 쓴다.

    bash src/dynamixel_control/scripts/run_calib.sh campose

박스를 서로 다른 위치 3곳 이상(가능하면 4~5곳, **한 직선 위에 두지 말 것**)에 놓고,
매번 base_link 기준 실제 좌표를 cm 단위로 입력한다. 좌표계는 REP-103:

    x = 앞(+) / 뒤(-),  y = 왼쪽(+) / 오른쪽(-),  z = 위(+)

## 원리

optical frame 관측점 A 와 base_link 실제점 B 의 대응으로 강체변환을 Kabsch 로 푼다
(closed-form, 반복 없음) → `T_base_optical`. launch 는 `base_link→camera_link` 를
받고 optical 회전은 자기가 붙이므로, 고정 회전을 되돌려 `T_base_camlink` 로 바꿔서
출력한다.

⚠️ 점이 한 직선 위에 몰리면 회전이 결정되지 않는다 — 앞뒤·좌우·높이를 골고루 섞을 것.
⚠️ depth 는 박스 **앞면**을 보므로 중심보다 약간 가깝게 나온다. 실제 좌표도 같은
   기준(박스 중심이면 중심)으로 일관되게 입력해야 계통오차가 안 생긴다.
"""
import math
import os
import sys

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from tf2_ros import Buffer, TransformListener

try:
    from robot_arm_msgs.msg import DetectedObject
except ImportError:
    sys.stderr.write(
        "robot_arm_msgs 를 import 할 수 없습니다 — 오버레이를 소싱하세요:\n"
        "    source /root/ros2_ws/install/setup.bash\n")
    sys.exit(1)

LATCHED = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                     history=HistoryPolicy.KEEP_LAST, depth=1,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

# camera_link → camera_color_optical_frame (camera_tf.launch.py 고정값)
OPTICAL_ROLL, OPTICAL_PITCH, OPTICAL_YAW = -math.pi / 2.0, 0.0, -math.pi / 2.0


def rot_x(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def rot_y(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def rot_z(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def rpy_from_matrix(R):
    """ZYX(yaw-pitch-roll) 분해 — static_transform_publisher 의 규약과 같다."""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:      # 짐벌락
        return math.atan2(-R[1, 2], R[1, 1]), pitch, 0.0
    return (math.atan2(R[2, 1], R[2, 2]), pitch, math.atan2(R[1, 0], R[0, 0]))


def kabsch(A, B):
    """A(카메라 관측) → B(실제) 강체변환 R,t 를 최소자승으로. 반사(det<0) 방지 포함."""
    ca, cb = A.mean(axis=0), B.mean(axis=0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    d = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ np.diag([1.0, 1.0, d]) @ U.T
    return R, cb - R @ ca


class Collector(Node):
    def __init__(self):
        super().__init__("calibrate_camera_pose")
        self.latest = None
        self.buf = Buffer()
        self.tf_listener = TransformListener(self.buf, self)
        self.create_subscription(DetectedObject, "/pick_target", self._cb, LATCHED)

    def gripper_xyz(self, base_frame, tip_link, timeout_s=5.0,
                    settle_s=2.0, drift_tol=0.005):
        """base_link 기준 그리퍼 위치 — 줄자 대신 쓰는 '진짜 좌표'.

        팔의 FK 는 이미 기어비·영점을 실측해 맞춰둔 상태라, 손으로 재는 것보다
        정확하고 반복성도 좋다. 그리퍼를 박스에 맞대고 이 값을 읽으면 그 지점의
        base_link 좌표를 얻는다.

        ⚠️ **드리프트 검사가 붙어 있다.** 토크를 끄면 직결축(arm_joint_4/5)이 중력으로
           흘러내리므로, 손을 뗀 채 찍으면 '떨어지는 도중'의 좌표가 기록된다. 2026-08-09
           영점 캘리브에서 실제로 이 때문에 1차 측정을 통째로 버렸다(joint_4 가 18° 어긋난
           값이 나왔다). 그래서 `settle_s` 동안 관측해 흔들림이 `drift_tol` 을 넘으면
           경고하고 그 점을 버릴 수 있게 한다.

        반환: (xyz, drift) — 드리프트[m] 를 같이 돌려주므로 호출부가 판단한다.
        """
        end = self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.buf.can_transform(base_frame, tip_link, rclpy.time.Time()):
                break
            if self.get_clock().now().nanoseconds * 1e-9 > end:
                return None, None

        got = []
        end = self.get_clock().now().nanoseconds * 1e-9 + settle_s
        while rclpy.ok() and self.get_clock().now().nanoseconds * 1e-9 < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            tr = self.buf.lookup_transform(
                base_frame, tip_link, rclpy.time.Time()).transform.translation
            got.append([tr.x, tr.y, tr.z])
        arr = np.array(got)
        mean = arr.mean(axis=0)
        drift = float(np.max(np.linalg.norm(arr - mean, axis=1)))
        return mean, drift

    def _cb(self, msg):
        p = msg.pose.position
        self.latest = np.array([p.x, p.y, p.z])

    def average(self, samples=30, timeout_s=15.0):
        """검출 지터를 줄이려 여러 프레임 평균. 프레임마다 마스크 중심이 흔들린다."""
        got, end = [], self.get_clock().now().nanoseconds * 1e-9 + timeout_s
        last = None
        while rclpy.ok() and len(got) < samples:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest is not None and (last is None
                                            or not np.array_equal(self.latest, last)):
                got.append(self.latest.copy())
                last = self.latest.copy()
            if self.get_clock().now().nanoseconds * 1e-9 > end:
                break
        if not got:
            return None, None
        arr = np.array(got)
        return arr.mean(axis=0), arr.std(axis=0)


def main():
    import argparse
    ap = argparse.ArgumentParser(description="카메라 TF 다점 캘리브레이션")
    ap.add_argument("--from-gripper", action="store_true",
                    help="줄자 대신 그리퍼 TF 를 실제 좌표로 쓴다(그리퍼를 박스에 맞대고 Enter)")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--tip-link", default="link_043",
                    help="그리퍼 링크 (arm_fsm 의 tip_link 파라미터와 같아야 함)")
    ap.add_argument("--drift-tol", type=float, default=0.005,
                    help="--from-gripper 에서 허용할 자세 흔들림 [m]. "
                         "토크가 꺼져 있으면 직결축이 처지므로 이걸 넘으면 경고한다")
    args = ap.parse_args()

    rclpy.init()
    node = Collector()
    try:
        domain = os.environ.get("ROS_DOMAIN_ID", "0(미설정)")
        print(f"/pick_target 수신 대기 중... (ROS_DOMAIN_ID={domain})")
        mean, _ = node.average(samples=3, timeout_s=10.0)
        if mean is None:
            print("박스 검출을 못 받았습니다 — perception_node 가 떠 있고 박스가 "
                  "화면에 보이는지 확인하세요.", file=sys.stderr)
            return 1

        if args.from_gripper:
            print(f"\n[그리퍼 모드] 실제 좌표를 {args.base_frame}→{args.tip_link} TF 에서 읽습니다.")
            print("박스를 놓고 → 팔을 움직여 그리퍼를 박스에 맞댄 뒤 → Enter.")
            print("(그리퍼가 박스를 완전히 가리지 않게 옆면에 대세요 — 카메라가 박스를 봐야 합니다)")
            print("박스를 옮겨가며 3회 이상 반복하고, 빈 줄 대신 'q' 로 계산합니다.\n")
        else:
            print("\n박스를 서로 다른 위치에 놓아가며 실제 좌표를 입력하세요.")
            print("좌표는 base_link 기준 cm: x=앞(+), y=왼쪽(+)/오른쪽(-), z=위(+)")
            print("예: '25 0 0'  (앞 25cm, 좌우 중앙, 바닥)")
            print("3점 이상 모은 뒤 빈 줄을 입력하면 계산합니다. 한 직선 위에 몰리지 않게 하세요.\n")

        obs, truth = [], []
        while True:
            prompt = (f"[{len(obs)}점] 그리퍼를 박스에 맞대고 Enter (q=계산): "
                      if args.from_gripper else
                      f"[{len(obs)}점] 박스 실제 좌표 (cm, 공백구분) 또는 빈 줄=계산: ")
            raw = input(prompt).strip()

            if args.from_gripper:
                if raw.lower() == "q":
                    if len(obs) >= 3:
                        break
                    print(f"  아직 {len(obs)}점입니다 — 최소 3점 필요.")
                    continue
                print("  자세 안정 확인 중(2초) — 계속 붙잡고 계세요...")
                xyz_arr, drift = node.gripper_xyz(args.base_frame, args.tip_link,
                                                  drift_tol=args.drift_tol)
                if xyz_arr is None:
                    print(f"  {args.base_frame}→{args.tip_link} TF 를 못 받았습니다 "
                          "(robot_state_publisher 와 브릿지가 떠 있나요?)")
                    continue
                print(f"  그리퍼 위치 = ({xyz_arr[0]:+.4f}, {xyz_arr[1]:+.4f}, "
                      f"{xyz_arr[2]:+.4f}) m   흔들림 {drift * 1000:.1f}mm")
                if drift > args.drift_tol:
                    print(f"  ⚠️ 흔들림이 허용치({args.drift_tol * 1000:.0f}mm)를 넘습니다 — "
                          "팔이 처지는 중일 수 있습니다(직결축 j4/j5).")
                    if input("     그래도 이 점을 쓸까요? (y/그 외=버림): ").strip().lower() != "y":
                        print("     버렸습니다. 다시 잡고 시도하세요.")
                        continue
            else:
                if not raw:
                    if len(obs) >= 3:
                        break
                    print(f"  아직 {len(obs)}점입니다 — 최소 3점 필요.")
                    continue
                try:
                    vals = [float(v) / 100.0 for v in raw.split()]
                    if len(vals) != 3:
                        raise ValueError
                except ValueError:
                    print("  숫자 3개를 공백으로 구분해 입력하세요 (예: 25 0 0)")
                    continue
                xyz_arr = np.array(vals)

            mean, std = node.average()
            if mean is None:
                print("  검출 수신 실패 — 박스가 화면에 보이나요?")
                continue
            print(f"  관측(optical) = ({mean[0]:+.4f}, {mean[1]:+.4f}, {mean[2]:+.4f}) m"
                  f"   지터 σ=({std[0]*100:.1f}, {std[1]*100:.1f}, {std[2]*100:.1f}) cm")
            obs.append(mean)
            truth.append(xyz_arr)

        A, B = np.array(obs), np.array(truth)
        R_bo, t_bo = kabsch(A, B)          # base_link ← optical

        resid = np.linalg.norm((R_bo @ A.T).T + t_bo - B, axis=1)
        print("\n" + "=" * 70)
        print("점별 잔차:")
        for i, r in enumerate(resid):
            print(f"  {i + 1}번: {r * 100:5.1f} cm")
        print(f"  RMS {np.sqrt((resid ** 2).mean()) * 100:.1f} cm, "
              f"최대 {resid.max() * 100:.1f} cm")

        # launch 는 base_link→camera_link 를 받고 optical 회전을 자기가 붙인다.
        R_co = rot_z(OPTICAL_YAW) @ rot_y(OPTICAL_PITCH) @ rot_x(OPTICAL_ROLL)
        R_bc = R_bo @ R_co.T
        roll, pitch, yaw = rpy_from_matrix(R_bc)

        print("=" * 70)
        print("\ncamera_tf.launch.py 인자:\n")
        print(f"  cam_x:={t_bo[0]:.4f} cam_y:={t_bo[1]:.4f} cam_z:={t_bo[2]:.4f} \\")
        print(f"  cam_roll:={roll:.4f} cam_pitch:={pitch:.4f} cam_yaw:={yaw:.4f}")
        print(f"\n  (카메라 위치 {t_bo[0]*100:.1f}, {t_bo[1]*100:.1f}, {t_bo[2]*100:.1f} cm / "
              f"roll {math.degrees(roll):.1f}° pitch {math.degrees(pitch):.1f}° "
              f"yaw {math.degrees(yaw):.1f}°)")

        if resid.max() > 0.03:
            print("\n⚠️ 최대 잔차가 3cm 를 넘습니다 — analytic IK 의 수용 오차와 같은 크기라")
            print("   이대로면 파지가 불안정합니다. 점을 더 넓게 분포시켜 다시 재세요.")
        return 0
    except (KeyboardInterrupt, EOFError):
        return 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
