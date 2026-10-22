# tmap_live.py — 광민 지형 지도(terrain_map, 브랜치 feat/l515-terrain-map)를 노트북 L515로 실시간 돌려 보는 시험대(10-22 저녁)
#   광민 코드는 별도 작업트리(--tmap-root, detached·읽기 전용)에서 import만 하고 고치지 않는다. 노드(terrain_map_node) 대신 이 파일이
#   L515 깊이(640×480 원본 uint16)·자이로·가속도(보정 적용)를 MapRuntime에 넣고, 바퀴는 '정지한 로버'(6바퀴 0 속도, 50 Hz)로 넣는다
#   → 이동은 0, 요는 자이로(정지 중 영점 학습). 지도 패킷은 콘솔 수신기(LatestMapReceiver.ingest)·렌더(map_rgba)로 콘솔과 똑같이 그린다.
#   - 장착 회전(camera_pitch_down_deg·roll)은 시작 1 s 가속도(보정 적용)로 잰다 — 놓아 둔 실제 자세를 '가상 로버' 수평으로 본다(--pitch로 고정 가능)
#   - 광민 검출기는 카메라 높이 0.25–0.80 m만 바닥으로 받는다 → 의자 위(약 0.45 m, 로버 0.47 m와 비슷)에서 바닥을 보게 둔다
#   - --criteria-thresholds: 지도 문턱을 기준값 JSON으로(돌출 = A 턱 주의 시작, 도랑 = A 도랑 불가 아래 cm) — 기본은 광민 MapConfig 그대로
#   - 시각: 스트림(깊이·가속도·자이로)마다 처음 2 s 동안 '도착 monotonic − 장치 시각'의 최솟값을 잡아 고정하고, 그 뒤부터 런타임에 넣는다
#     (스트림마다 시각 기준(domain)이 다를 수 있음 — 하나의 벽시계 차이로 옮겼더니 바퀴(호스트 시각)와 어긋나 NO_POSE·시대 반복, 10-22 첫 실행)
# 사용: .venv_l515/bin/python tmap_live.py [--tmap-root DIR] [--calib JSON] [--pitch 19.5] [--criteria-thresholds] [--cell 0.05] [--seconds N]
#   키: s = 화면 저장, r = 지도 새로 시작(런타임 다시 만듦), q·Esc = 끝
import argparse, collections, glob, json, math, os, sys, threading, time

if os.path.isdir("/usr/share/fonts/truetype/dejavu"):
    os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
import cv2
import numpy as np
import pyrealsense2 as rs

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
# 기본 경로(환경 변수로 바꿈): 광민 terrain_map·terrain_overlay가 있는 저장소 = 이 저장소(feat/l515-terrain-map 위) 또는 L515_TMAP_ROOT 작업트리,
#   기록·보정 파일 = L515_OUT_DIR(없으면 지금 폴더의 recordings), 기준값 = L515_CRITERIA(없으면 lidar_dev/config의 v0.2)
TMAP_DEFAULT = os.environ.get("L515_TMAP_ROOT") or REPO
REC = os.environ.get("L515_OUT_DIR") or os.path.join(os.getcwd(), "recordings")
CRIT_DEFAULT = os.environ.get("L515_CRITERIA") or os.path.join(HERE, "config", "obstacle_criteria_v0.2.json")
WHITE, YEL, GRAY, CYAN = (255, 255, 255), (40, 220, 240), (150, 150, 150), (230, 200, 40)


def put(img, s, org, col=WHITE, sc=0.5, th=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, col, th, cv2.LINE_AA)


def load_tmap(root):
    for p in (root, os.path.join(root, "motor_control")):
        if p not in sys.path:
            sys.path.insert(0, p)
    from terrain_overlay.camera import CameraModel
    from terrain_overlay.runtime import SenderSession
    from terrain_map import contract as C
    from terrain_map.config import MapConfig
    from terrain_map.odometry import WheelLayout, WheelReading
    from terrain_map.runtime import MapRuntime
    from operator_console.terrain_map import DisplayConfig, LatestMapReceiver, MapView, map_rgba
    from chassis.kinematics import default_geometry
    return dict(CameraModel=CameraModel, SenderSession=SenderSession, C=C, MapConfig=MapConfig, WheelLayout=WheelLayout,
                WheelReading=WheelReading, MapRuntime=MapRuntime, DisplayConfig=DisplayConfig, LatestMapReceiver=LatestMapReceiver,
                MapView=MapView, map_rgba=map_rgba, default_geometry=default_geometry)


def git_head(root):
    try:
        import subprocess
        return subprocess.run(["git", "-C", root, "log", "-1", "--format=%h %cs %s"], capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:  # noqa: BLE001
        return "?"


def apply_cal(a, cal):
    if cal is None:
        return np.asarray(a, float)
    return (np.asarray(a, float) - np.asarray(cal["accel_bias_m_s2"])) / np.asarray(cal["accel_scale"])


def mount_from_gravity(up_opt):
    """가속도(보정 뒤, 깊이 광학 좌표, 정지 = 위쪽 비력) → 광민 base_to_optical(pitch_down, roll, 0)의 (pitch_down, roll) °.
    수평 로버라면 위 방향(광학) = base_to_optical(p, r, 0)·(0, 0, 1) = (−cos p·sin r, −cos p·cos r, −sin p) 꼴(광학 x 오른쪽, y 아래, z 앞)."""
    u = np.asarray(up_opt, float) / np.linalg.norm(up_opt)
    pitch_down = math.degrees(math.asin(max(-1.0, min(1.0, -u[2]))))     # 아래로 숙이면 위 방향이 광축 반대(−z)로 기움
    roll = math.degrees(math.atan2(-u[0], -u[1]))
    return pitch_down, roll


def map_panel(T, snap, fwd, size, disp, now):
    """콘솔 렌더(map_rgba)를 앞이 위로 오게 돌려 size×size 창(카메라 x = 0 기준 −0.5…fwd m, 옆 ±(fwd + 0.5)/2 m)에 붙인다. 거리 눈금·자세 화살표 포함."""
    status = snap.status
    view = T["MapView"](x0=-0.5, y0=-(fwd + 0.5) / 2, width_m=fwd + 0.5, height_m=fwd + 0.5, width_px=size, height_px=size)
    rgba, meta = T["map_rgba"](snap, view, disp, now)
    mimg = np.full((size, size, 3), 30, np.uint8)
    sx = size / (fwd + 0.5)
    if meta.get("drawn") and rgba.shape[0] > 1:
        # rgba: 행 0 = 가장 큰 y(왼쪽), 열 = x(앞) → 앞이 위: 시계 반대로 90° 돌리면 행 = x 감소, 열 = y 감소
        rgb = cv2.rotate(rgba[..., :3], cv2.ROTATE_90_COUNTERCLOCKWISE)
        al = cv2.rotate(rgba[..., 3], cv2.ROTATE_90_COUNTERCLOCKWISE).astype(np.float32)[..., None] / 255.0
        img = (rgb[..., ::-1].astype(np.float32) * al + 30 * (1 - al)).astype(np.uint8)
        mpp, ox, ty = meta["m_per_px"], meta["origin_x_m"], meta["top_y_m"]
        h_, w_ = img.shape[:2]
        x_top = ox + h_ * mpp                                  # 돌린 영상 맨 위 행의 x
        r0 = int(round((view.x0 + view.width_m - x_top) * sx))
        c0 = int(round((view.y0 + view.height_m - ty) * sx))
        scl = mpp * sx
        img = cv2.resize(img, (max(1, int(round(w_ * scl))), max(1, int(round(h_ * scl)))), interpolation=cv2.INTER_NEAREST)
        h2, w2 = img.shape[:2]
        rr0, cc0, rr1, cc1 = max(0, r0), max(0, c0), min(size, r0 + h2), min(size, c0 + w2)
        if rr1 > rr0 and cc1 > cc0:
            mimg[rr0:rr1, cc0:cc1] = img[rr0 - r0:rr1 - r0, cc0 - c0:cc1 - c0]
    for r_ in np.arange(0.5, fwd + 1e-6, 0.5):              # 거리 눈금(월드 x)
        y = int(round((view.x0 + view.width_m - r_) * sx))
        cv2.line(mimg, (0, y), (size, y), (70, 70, 70), 1)
        put(mimg, f"{r_:.1f}m", (4, y - 3), GRAY, 0.4)
    cx_, cy_ = (view.y0 + view.height_m) * sx, (view.x0 + view.width_m) * sx
    if status is not None and status.pose is not None:
        p = status.pose
        px, py = int(round(cx_ - p.y_m * sx)), int(round(cy_ - p.x_m * sx))
        cv2.arrowedLine(mimg, (px, py), (int(px - 18 * math.sin(p.yaw_rad)), int(py - 18 * math.cos(p.yaw_rad))), CYAN, 2, tipLength=0.4)
    return mimg


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tmap-root", default=TMAP_DEFAULT)
    ap.add_argument("--calib", default=None, help="가속도 보정 JSON(기본 = recordings의 가장 최근 유효 imu_calib_live_*.json)")
    ap.add_argument("--pitch", type=float, default=None, help="장착 숙임각 고정(°). 없으면 시작 1 s 가속도로")
    ap.add_argument("--roll", type=float, default=None)
    ap.add_argument("--cell", type=float, default=None, help="칸 크기(m, 기본 MapConfig 0.05)")
    ap.add_argument("--criteria-thresholds", action="store_true", help="지도 문턱을 기준값 JSON으로")
    ap.add_argument("--criteria", default=CRIT_DEFAULT)
    ap.add_argument("--fwd", type=float, default=3.0, help="지도 보기 앞쪽 끝(m)")
    ap.add_argument("--out", default=REC)
    ap.add_argument("--seconds", type=float, default=0)
    a = ap.parse_args()
    T = load_tmap(a.tmap_root)
    head = git_head(a.tmap_root)

    cal_path = a.calib
    if cal_path is None:
        for p in sorted(glob.glob(os.path.join(REC, "imu_calib_live_*.json")), reverse=True):
            if json.load(open(p)).get("valid"):
                cal_path = p
                break
    cal = json.load(open(cal_path)) if cal_path else None

    cfg_kw = {}
    if a.cell:
        cfg_kw["cell_m"] = a.cell
    if a.criteria_thresholds:
        sys.path.insert(0, HERE)
        from criteria import load_criteria
        R = load_criteria(a.criteria)["rules"]
        cfg_kw["obstacle_height_m"] = float(R["step_up_m"]["caution_from"])
        cfg_kw["ditch_depth_m"] = math.floor(float((R.get("trench_depth_m") or R["step_up_m"])["block_from"]) * 100 - 1e-9) / 100.0   # trench 없는 옛 형식은 턱 기준

    rcfg = rs.config()
    rcfg.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    rcfg.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    rcfg.enable_stream(rs.stream.accel)
    rcfg.enable_stream(rs.stream.gyro)
    lock = threading.Lock()
    st = {"rt": None, "off": {}, "frozen": False, "domain": {}, "color": None, "depth": None, "acc": collections.deque(maxlen=2000),
          "n_depth": 0, "n_gyro": 0}

    def to_mono(kind, ts_ms, now, frame):
        """장치 시각 → monotonic(live_overlay TmapBridge.stamp와 같은 규칙: 전역 시각이면 바로, 하드웨어 시계면 최소 지연선을 초당 0.6 %까지 따라감)."""
        t = ts_ms / 1000.0
        dom = frame.get_frame_timestamp_domain()
        st["domain"].setdefault(kind, str(dom))
        if dom == rs.timestamp_domain.global_time:
            return t - (time.time() - time.monotonic())
        o = now - t
        prev, last = st["off"].get(kind), st.setdefault("last_now", {}).get(kind, now)
        st["off"][kind] = o if prev is None else min(prev + 0.006 * max(0.0, now - last), o)
        st["last_now"][kind] = now
        return t + st["off"][kind]

    pipe = rs.pipeline()

    def cb(f):
        try:
            now = time.monotonic()
            if f.is_frameset():
                fs = f.as_frameset()
                d, c = fs.get_depth_frame(), fs.get_color_frame()
                if c:
                    with lock:
                        st["color"] = np.asanyarray(c.get_data()).copy()
                if d:
                    arr = np.asanyarray(d.get_data()).copy()
                    with lock:
                        st["depth"] = arr
                        rt = st["rt"]
                    td = to_mono("depth", d.get_timestamp(), now, d)
                    if rt is not None and st["frozen"]:
                        rt.on_depth(arr, td, now)
                        st["n_depth"] += 1
            elif f.is_motion_frame():
                m = f.as_motion_frame()
                v = m.get_motion_data()
                kind = "accel" if m.get_profile().stream_type() == rs.stream.accel else "gyro"
                t = to_mono(kind, m.get_timestamp(), now, m)
                if kind == "accel":
                    ac = apply_cal((v.x, v.y, v.z), cal)
                    with lock:
                        st["acc"].append((t, *ac))
                        rt = st["rt"]
                    if rt is not None and st["frozen"]:
                        rt.on_accel(t, now, tuple(R_ad @ ac))
                else:
                    with lock:
                        rt = st["rt"]
                    if rt is not None and st["frozen"]:
                        rt.on_gyro(t, now, tuple(R_gd @ np.array([v.x, v.y, v.z])))
                        st["n_gyro"] += 1
        except Exception as e:  # noqa: BLE001 — 콜백 예외는 SDK가 삼키므로 직접 남긴다
            st["cb_error"] = repr(e)

    prof = pipe.start(rcfg, cb)
    _OPEN.append(pipe)                                # 아래 try 전에 예외가 나도 장치를 닫게(__main__)
    for sen_ in prof.get_device().query_sensors():           # 전역 시각 켬(10-22 밤): L515 하드웨어 시계는 호스트보다 약 0.4 % 느려 몇 분이면 0.5 s 넘게 어긋남
        try:                                                  # → 광민 지도 스트림 시각 어긋남(시대 반복, 지도 0칸). 전역 시각이면 SDK가 호스트 시계에 맞춰 줌
            if sen_.supports(rs.option.global_time_enabled):
                sen_.set_option(rs.option.global_time_enabled, 1.0)
        except RuntimeError:
            pass
    ds = prof.get_device().first_depth_sensor()
    scale = ds.get_depth_scale()
    dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile()
    K = dprof.get_intrinsics()
    R_ad = np.array(prof.get_stream(rs.stream.accel).get_extrinsics_to(prof.get_stream(rs.stream.depth)).rotation, float).reshape(3, 3).T
    R_gd = np.array(prof.get_stream(rs.stream.gyro).get_extrinsics_to(prof.get_stream(rs.stream.depth)).rotation, float).reshape(3, 3).T
    if abs(scale - 0.00025) > 1e-9:
        print(f"⚠ depth scale {scale} ≠ 광민 검출기 기본 0.00025 — 높이가 틀어짐", flush=True)
    camera = T["CameraModel"](fx=K.fx, fy=K.fy, cx=K.ppx, cy=K.ppy, width=K.width, height=K.height)
    geo = T["default_geometry"]()
    layout = T["WheelLayout"]({w.name: (w.x, w.y) for w in geo.wheels}, geo.wheel_radius_m)
    names = [w.name for w in geo.wheels]

    # 시작 1 s: 가속도(보정 뒤)로 장착 자세
    t0 = time.monotonic()
    while time.monotonic() - t0 < 2.0:            # 스트림 시각 차이 최솟값·장착 자세를 잡는 동안
        time.sleep(0.05)
    mount = {"pitch": 15.0, "roll": 0.0, "warn": ""}

    def measure_mount():
        """최근 1 s 가속도(보정 뒤)로 장착 자세. MapConfig 범위(숙임 0–35°, 옆 ±45°로 가정) 밖이면 이전 값 유지·경고."""
        with lock:
            A = np.array(st["acc"], float)
        if a.pitch is not None:
            mount.update(pitch=a.pitch, roll=a.roll if a.roll is not None else 0.0, warn="")
            return
        A = A[A[:, 0] > A[-1, 0] - 1.0] if len(A) else A
        if len(A) < 20:
            mount["warn"] = "no accel - mount 15 deg"
            return
        p_meas, r_meas = mount_from_gravity(R_ad @ A[:, 1:4].mean(0))
        if os.environ.get("TMAP_DEBUG"):
            print("[debug] 가속도 표본", A.shape, "평균", np.round(A[:, 1:4].mean(0), 3), "R_ad 대각", np.round(np.diag(R_ad), 4), "→", round(p_meas, 2), round(r_meas, 2), flush=True)
        try:                                                     # 범위는 광민 MapConfig가 정한다(숙임 0–35°, 옆 ±20°)
            T["MapConfig"](camera_pitch_down_deg=round(p_meas, 2), camera_roll_deg=round(r_meas, 2), **cfg_kw)
            in_range = True
        except ValueError:
            in_range = False
        if in_range:
            mount.update(pitch=round(p_meas, 2), roll=round(r_meas, 2) if a.roll is None else a.roll, warn="")
        else:
            mount["warn"] = f"measured pitch {p_meas:.0f} roll {r_meas:.0f} out of range - aim at floor, press r"
            print(f"⚠ 잰 장착 자세(숙임 {p_meas:.1f}°·옆 {r_meas:.1f}°)가 범위(숙임 0–35°·옆 ±20°) 밖 — {mount['pitch']}°로 시작. 바닥을 보게 두고 r", flush=True)

    measure_mount()

    def new_runtime():
        cfg = T["MapConfig"](camera_pitch_down_deg=mount["pitch"], camera_roll_deg=mount["roll"], **cfg_kw)
        sess = T["SenderSession"](boot_id=os.urandom(16).hex(), start_mono_ns=time.monotonic_ns(), instance=os.urandom(8).hex())
        rt = T["MapRuntime"](cfg, sess, layout, log_error=lambda m: print("[map]", m, flush=True))
        rt.on_camera_info(camera)
        return cfg, rt

    cfg, rt = new_runtime()
    rx = T["LatestMapReceiver"](0)
    disp = T["DisplayConfig"]()
    with lock:
        st["rt"] = rt
        st["frozen"] = True
    print("스트림 시각 기준", st["domain"], "· 도착−장치 최소(s)", {k: round(v, 4) for k, v in st["off"].items()}, flush=True)
    print(f"광민 terrain_map {head} · 깊이 {K.width}x{K.height} fx {K.fx:.1f} · 보정 {os.path.basename(cal_path) if cal_path else '없음'} · "
          f"장착 숙임 {mount['pitch']}°·옆 {mount['roll']}° ({'고정' if a.pitch is not None else '가속도로 잼'}) · 칸 {cfg.cell_m} m · 문턱 돌출 {cfg.obstacle_height_m} / 도랑 {cfg.ditch_depth_m} m"
          f"{' (기준값)' if a.criteria_thresholds else ' (광민 기본)'}", flush=True)
    print("키: s 저장, r 지도 새로, q 끝 | 카메라 높이 0.25–0.80 m(의자 위)에서 바닥을 보게", flush=True)

    win = "terrain_map live (gwangmin branch)"
    MAPW = 540
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 960 + MAPW, 540 + 150)
    t_start, t_proc, t_send, t_log = time.monotonic(), 0.0, 0.0, 0.0
    status, packets_n, frame = None, 0, None
    zero = tuple(T["WheelReading"](n, 0.0, 0.0, 0.0, 0.0, True) for n in names)
    stop_wheels = threading.Event()

    def wheel_feeder():
        """50 Hz '정지한 로버' 바퀴 — 그리기 루프와 떼어 둠(루프가 60 ms 넘게 걸리면 바퀴 간격이 pose_max_gap_s를 넘어 POSE_GAP·프레임 잃음, 10-22 첫 실기)."""
        nxt = time.monotonic()
        while not stop_wheels.is_set():
            nxt += 0.02
            now_ = time.monotonic()
            with lock:
                rt_ = st["rt"]
            rt_.on_wheels(now_, now_, zero)
            time.sleep(max(0.0, nxt - time.monotonic()))
    threading.Thread(target=wheel_feeder, name="wheels", daemon=True).start()
    try:
        while True:
            now = time.monotonic()
            if now - t_proc >= 1.0 / (2 * cfg.process_hz):
                rt.process_tick()
                t_proc = now
            if now - t_send >= 1.0 / cfg.status_hz:
                for raw in rt.send_tick():
                    rx.ingest(raw, now)
                    packets_n += 1
                t_send = now
            snap = rx.snapshot(now)
            status = snap.status
            mimg = map_panel(T, snap, a.fwd, MAPW, disp, now)
            put(mimg, "gwangmin terrain_map (console render)", (6, MAPW - 10), GRAY, 0.42)
            with lock:
                color = st["color"]
            view_img = color.copy() if color is not None else np.zeros((540, 960, 3), np.uint8)
            hud = np.zeros((150, 960 + MAPW, 3), np.uint8)
            L = []
            if status is not None:
                th = status.thresholds
                L.append(f"status {status.status}" + (f" ({status.reason})" if status.reason else "") + f"   tiles {status.tile_count}  pose gaps {status.pose_gaps}"
                         f"  not-support {status.not_support}  late {status.late_dropped}  proc {status.proc_ms if status.proc_ms is None else round(status.proc_ms)} ms")
                L.append(f"thresholds {th}")
                if status.plane_tilt is not None:
                    L.append(f"plane tilt {status.plane_tilt}" + (f"   gravity {status.gravity}" if status.gravity is not None else ""))
            else:
                L.append("status: waiting for first packet")
            L.append(f"mount pitch {cfg.camera_pitch_down_deg} deg roll {cfg.camera_roll_deg} deg ({'fixed' if a.pitch is not None else 'from IMU at start/r'})"
                     + (f" !! {mount['warn']}" if mount["warn"] else "") + f"  cell {cfg.cell_m} m  depth frames {st['n_depth']}  gyro {st['n_gyro']}  packets {packets_n}"
                     + (f"  CB ERROR {st['cb_error']}" if st.get("cb_error") else ""))
            L.append(f"branch {head[:60]}   keys: s save  r restart map  q quit")
            for i, s_ in enumerate(L[:6]):
                put(hud, s_, (8, 22 + 24 * i), WHITE, 0.48)
            frame = np.vstack([np.hstack([view_img, mimg]), hud])
            cv2.imshow(win, frame)
            if now - t_log > 2.0:
                t_log = now
                print(f"[{time.strftime('%H:%M:%S')}] " + " | ".join(L[:3]) + f" | {rt.processing_summary()}", flush=True)
            if a.seconds and now - t_start > a.seconds:
                fn = os.path.join(a.out, time.strftime("tmap_auto_%Y%m%d_%H%M%S.png"))
                cv2.imwrite(fn, frame)
                print("시험 시간 끝 — 마지막 화면 저장:", fn, flush=True)
                break
            k = cv2.waitKey(15) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s") and frame is not None:
                fn = os.path.join(a.out, time.strftime("tmap_%Y%m%d_%H%M%S.png"))
                cv2.imwrite(fn, frame)
                print("저장:", fn, flush=True)
            if k == ord("r"):
                measure_mount()
                cfg, rt_new = new_runtime()
                rx.close()
                rx = T["LatestMapReceiver"](0)
                with lock:
                    st["rt"] = rt_new
                rt = rt_new
                print("지도 새로 시작", flush=True)
    finally:
        stop_wheels.set()
        _OPEN.remove(pipe)
        pipe.stop()
        rx.close()
        cv2.destroyAllWindows()


_OPEN = []

if __name__ == "__main__":
    try:
        main()
    except BaseException:
        for _p in _OPEN:
            _p.stop()
        raise
