"""카메라 TF 다점 캘리브 계산 + 품질 가드 (ROS 비의존, 2026-08-07 추가).

`camera_tf_tuner`의 RViz 메뉴가 이 모듈을 호출한다. ROS를 안 쓰는 순수 함수라
단독으로 테스트할 수 있고, `dynamixel_control/scripts/calibrate_camera_pose.py`
(터미널 입력 방식)와 **같은 수식**을 쓴다 — 두 경로의 결과가 달라지면 안 된다.

## 가드가 왜 필요한가 (2026-08-07 실측에서 겪은 것)

터미널 도구는 점을 다 모은 **뒤에야** 잔차를 보여준다. 그런데 그날 나온 문제는
전부 점을 찍는 순간 알 수 있는 것들이었고, 심지어 일부는 잔차에 아예 안 나타났다:

1. **측정 원점 불일치(19cm)** — 모든 점이 같이 밀리므로 잔차는 그대로다. 원리적으로
   최소자승이 못 잡는다. → 사람이 3D로 봐야 한다(그래서 RViz 시각화가 짝이다).
2. **너무 가까운 점(16cm)** — D435i 최소 측정거리 아래라 depth 자체가 틀린다.
   잔차 3.4cm로 "애매하게 나쁨"으로만 보여서 원인 파악이 안 됐다. → `check_range`
3. **한 평면 위 배치** — 높이를 섞었는데도 바닥 위치가 2곳뿐이라 평면이 안 깨졌다.
   회전/높이가 결정되지 않는데 잔차는 멀쩡해 보인다. → `check_spread`
4. **관측 간 거리와 실측 간 거리의 불일치(최대 4.4cm)** — 박스를 8cm 올렸는데 관측은
   3.6cm만 움직인 경우. 오타·측정 실수·계통 오차가 여기서 드러난다. → `check_pair`
"""
import math

import numpy as np

#: camera_link → camera_color_optical_frame 고정 회전 (REP-103).
#: camera_tf.launch.py / calibrate_camera_pose.py 와 같은 값이어야 한다.
OPTICAL_RPY = (-math.pi / 2.0, 0.0, -math.pi / 2.0)

#: D435i 권장 동작 범위 하한. 이보다 가까우면 depth 를 믿을 수 없다(해상도에 따라
#: 최소 측정거리가 17~28cm, 권장은 30cm 이상 — 2026-08-07 실측에서 16.5cm 점이
#: 최악의 잔차를 냈다).
DEFAULT_MIN_RANGE = 0.30

#: 관측 간 거리 vs 실측 간 거리 허용차.
DEFAULT_PAIR_TOL = 0.02

#: 최소자승이 회전을 결정하려면 점 배치가 3차원으로 퍼져야 한다. 세 번째 특이값이
#: 이보다 작으면 사실상 평면(또는 직선) 배치다.
DEFAULT_SPREAD_MIN = 0.03


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
    """ZYX(yaw-pitch-roll) 분해 — static_transform_publisher 규약."""
    pitch = math.atan2(-R[2, 0], math.hypot(R[0, 0], R[1, 0]))
    if abs(math.cos(pitch)) < 1e-8:          # 짐벌락
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


# ── 가드 ───────────────────────────────────────

def check_range(observation, min_range=DEFAULT_MIN_RANGE):
    """관측점이 depth 신뢰 범위 안인가. 측정하는 순간 판정할 수 있다."""
    distance = float(np.linalg.norm(observation))
    if distance < min_range:
        return (f"only {distance * 100:.0f}cm from camera (min {min_range * 100:.0f}cm) "
                "- depth is unreliable this close. Move the box farther away.")
    return None


def check_pair(new_obs, new_truth, observations, truths, tol=DEFAULT_PAIR_TOL):
    """이미 찍은 점들과의 '관측 거리 vs 실측 거리'를 대조한다.

    두 점 사이 거리는 카메라 자세와 **무관한 불변량**이라, 여기서 어긋나면 회전을
    맞춰봐야 소용없다 — 측정 자체가 틀린 것이다.
    """
    warnings = []
    for i, (obs, truth) in enumerate(zip(observations, truths)):
        d_obs = float(np.linalg.norm(new_obs - obs))
        d_truth = float(np.linalg.norm(new_truth - truth))
        if abs(d_obs - d_truth) > tol:
            warnings.append(
                f"distance to point {i + 1}: measured {d_truth * 100:.1f}cm vs observed "
                f"{d_obs * 100:.1f}cm ({(d_obs - d_truth) * 100:+.1f}cm) - tape-measure "
                "mistake, or the box face seen by the camera changed.")
    return warnings


def check_spread(truths, minimum=DEFAULT_SPREAD_MIN):
    """점 배치가 3차원으로 퍼졌는지 — 특이값 3개로 판정."""
    if len(truths) < 3:
        return [f"only {len(truths)} point(s) - need at least 3."], None
    centered = np.array(truths) - np.array(truths).mean(axis=0)
    singular = np.linalg.svd(centered, compute_uv=False)
    if len(truths) == 3:
        # 점 3개는 **항상** 한 평면 위다(수학적으로). 해는 유일하게 결정되지만
        # 노이즈에 그대로 노출되므로, 4점 이상과는 다른 안내를 해야 한다.
        return (["3 points are always coplanar: the solution is unique but fully "
                 "exposed to noise. Add a 4th point off their plane."], singular)
    if singular[2] < minimum:
        return ([f"points are coplanar (or collinear) - spread "
                 f"{singular[0] * 100:.0f}/{singular[1] * 100:.0f}/{singular[2] * 100:.1f}cm. "
                 "Rotation and height are not determined. Use 3+ FLOOR positions forming "
                 "a triangle, and vary height as well."], singular)
    return [], singular


# ── 해 계산 ────────────────────────────────────

def solve(observations, truths, *, min_range=DEFAULT_MIN_RANGE,
          pair_tol=DEFAULT_PAIR_TOL, spread_min=DEFAULT_SPREAD_MIN):
    """관측/실측 대응점 → base_link→camera_link 자세 + 품질 리포트.

    반환 dict: xyz, rpy, residuals, rms, max_residual, warnings, singular
    점이 3개 미만이면 `None` 을 돌려준다(호출부가 안내를 띄운다).
    """
    if len(observations) < 3:
        return None

    A = np.array(observations, dtype=float)
    B = np.array(truths, dtype=float)
    R_bo, t_bo = kabsch(A, B)
    residuals = np.linalg.norm((R_bo @ A.T).T + t_bo - B, axis=1)

    # launch 는 base_link→camera_link 를 받고 optical 회전은 자기가 붙인다.
    # camera_link 와 optical frame 은 원점이 같고 회전만 다르므로 translation 은 그대로다.
    R_co = rot_z(OPTICAL_RPY[2]) @ rot_y(OPTICAL_RPY[1]) @ rot_x(OPTICAL_RPY[0])
    roll, pitch, yaw = rpy_from_matrix(R_bo @ R_co.T)

    warnings, singular = check_spread(truths, spread_min)
    for i, obs in enumerate(observations):
        message = check_range(obs, min_range)
        if message:
            warnings.append(f"point {i + 1}: {message}")
    for i in range(len(observations)):
        warnings.extend(
            f"point {i + 1} <-> " + w
            for w in check_pair(A[i], B[i], A[:i], B[:i], pair_tol))

    max_residual = float(residuals.max())
    if max_residual > 0.03:
        warnings.append(
            f"max residual {max_residual * 100:.1f}cm - same size as the analytic IK "
            "acceptance tolerance (3cm), so grasping will be unreliable as-is.")

    return {
        "xyz": tuple(float(v) for v in t_bo),
        "rpy": (roll, pitch, yaw),
        "residuals": [float(v) for v in residuals],
        "rms": float(np.sqrt((residuals ** 2).mean())),
        "max_residual": max_residual,
        "warnings": warnings,
        "singular": None if singular is None else [float(v) for v in singular],
    }


def format_report(result) -> str:
    """로그/RViz 화면에 그대로 찍는 리포트.

    RViz(Ogre 기본 폰트)는 한글을 두부(□)로 그리므로 **화면에 뜨는 문자열은 영어**로
    쓴다. 주석/문서만 한국어(프로젝트 규칙).
    """
    if result is None:
        return "fewer than 3 points - need at least 3."
    x, y, z = result["xyz"]
    roll, pitch, yaw = result["rpy"]
    lines = [
        "residuals: " + " / ".join(f"{r * 100:.1f}" for r in result["residuals"]) + " cm",
        f"RMS {result['rms'] * 100:.2f}cm, max {result['max_residual'] * 100:.2f}cm",
        f"cam_x:={x:.4f} cam_y:={y:.4f} cam_z:={z:.4f} "
        f"cam_roll:={roll:.4f} cam_pitch:={pitch:.4f} cam_yaw:={yaw:.4f}",
        f"(fwd {x * 100:.1f}, side {y * 100:+.1f}, up {z * 100:.1f} cm / "
        f"roll {math.degrees(roll):+.1f}° pitch {math.degrees(pitch):+.1f}° "
        f"yaw {math.degrees(yaw):+.1f}°)",
    ]
    lines.extend("⚠️ " + w for w in result["warnings"])
    return "\n".join(lines)
