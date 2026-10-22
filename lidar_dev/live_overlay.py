# live_overlay.py — L515 실시간 창(7판 끝, 10-22 밤 — 광민 두 런타임을 자식 프로세스로(gm_worker.py, GIL 다툼 없앰 → 지도 켜져도 23–25 fps),
#   화면 녹화 w(VideoSink 스레드 → H.264), 비스듬히 보기 시점 [ ] (휘기 없이 줄마다 사다리꼴), 새 프레임 대기·IMU 고리 버퍼·칠하기 마스크 복사,
#   자이로 영점 대기 중 '가만히 · 흔들림 °/s' 안내)
#   (7판, 10-22 밤 — 장착 캘리브레이션 k(mount_calib.py, 다음 실행 자동 적용·바닥 사전값), 바닥 검출 numpy 새로
#   (ground_est.py: 중력 사전 높이 히스토그램 + 거리 비례 허용폭 가중 재맞춤, 화면 전체, 천장 오인 고침), 광민 지도 범위 4 m, Jetson 대비 open3d 없이)
#   (6판, 10-22 저녁 — 광민 레포 화면 기능 전부: 오버레이 런타임 배지·윤곽, 콘솔식 지도 보기·조작·상태줄·기울기·범례, 기록 r — gm_features.py)
#   (5판, 10-22 저녁 — 4판 '내 버전 UI·지도 표시 + 광민 버전 실시간성' 통합에 튐 거르기·속도·통로 띠)
#   5판: 광민 코드 문턱 = 기준값 측정값 문턱(돌출 = 턱 주의 − 1 cm, 도랑 = 도랑 주의 − 1 cm), 주의·불가는 광민 다수결 코드가 맞을 때만(한 프레임 튐 제거),
#   카메라 화면 바닥 찾기 --stride 2(480×270), 지도 그림은 지도가 바뀔 때만, 화면에 통로 띠·거리선, 장착 각 범위 밖이면 가만히 둘 때 자동 재측정,
#   L515 --confidence·--noise-filter
#   - 카메라 화면(3판 그대로): 깊이를 컬러에 정렬해 GroundDetector(ground_est.py, 7판 — 6판까지 광민 terrain.py)로 바닥 초록·장애물 빨강, 덩어리마다 높이·거리·폭, 클릭 점 거리·높이·두 점 거리
#   - 지도 원천(--map gwangmin, 기본): 광민 terrain_map MapRuntime(브랜치 feat/l515-terrain-map, 별도 작업트리 --tmap-root에서 import만) — 세계 고정 칸,
#     자이로 요(정지 중 영점 학습), 칸 분류 이력 다수결, 5 Hz 솎은 처리를 자식 프로세스에서. 바퀴는 '정지한 로버'(0 속도 50 Hz) → 이동 0, 제자리 회전은 따라감
#     --map own: 자체 LiveHeightMap(카메라가 가만히 있을 때만 쌓음, 3판) — 비교용
#   - 지도 표시(3판 방식): 지금 카메라 기준 '앞이 위' 격자로 잘라(live_map.tmap_local_grid) 기준값 JSON 단계 색(통과·주의·불가·꺼짐, 측정 여유 +1 cm),
#     음영, 비스듬히 본 2.5D. 광민 칸 높이(1 cm 정수)를 그대로 쓰고 광민의 이진 코드(돌출·도랑)는 '관측됨' 판정에만 씀. 지금 안 보이는 칸(--dim s 넘음)은 어둡게
#   - 바닥 찾기 중력 확인(--grav-tol°), 바닥 높이 범위 --dmin·--dmax(광민 지도는 검출기 고정 0.25–0.80 m), 가속도 보정 자동(가장 최근 유효 imu_calib_live_*.json)
#   - IMU 장착 각(키 1·2, 3·4 = 180°)·가속도 보정(키 c)은 3판 그대로. 광민 지도 '가상 로버' 장착 숙임·옆은 시작 2 s·키 m 때 가속도로
# 사용: python live_overlay.py [--map gwangmin|own] [--cell 0.03] [--criteria-thresholds] [--dmin 0.25 --dmax 0.80] [--calib JSON|none]
#       python live_overlay.py --recalc imu_mount_*.json   (카메라 없이 장착 각 다시 계산)
#   마우스: 왼쪽 클릭 = 그 점 거리·높이(두 번째 클릭부터 직전 점과의 3D 거리) / 지도 칸 높이·단계·나이, 오른쪽 클릭 = 지움
#   키: 1/2 IMU 자세(3·4 = 180°), c 보정 자세(x 지우기) | v 지도 보기(기준값 단계 → 높이 → 광민 콘솔), f 따라가기, e 지도 화면 지우기, m 지도 새로(장착 다시 잼)
#       k 장착 캘리브레이션, p 바닥 사전값, w 화면 녹화 켜기/끄기, [ ] 비스듬히 보기 시점 낮게/높게
#       o 광민 윤곽 켜기/끄기, b 우리 덩어리 상자 켜기/끄기, r 기록(광민 lidar_dev.record 형식 40쌍), g 중력 확인, s 화면·지도 저장, q·Esc 끝
#   콘솔 보기: 지도를 끌면 옮김(따라가기 꺼짐), 휠 = 확대·축소
import argparse, collections, glob, json, math, os, sys, threading, time, types

for _v in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS"):   # numpy 불러오기 전에: BLAS 스레드 1개(7판 끝 실측 — 다른 부하가 있으면 16스레드 풀이
    os.environ.setdefault(_v, "1")                                          # 서로 헛돌아 바닥 검출 중앙 28–32 ms·최대 75 ms, 1스레드 12 ms·최대 15–23 ms. fork 자식도 물려받음)

if os.path.isdir("/usr/share/fonts/truetype/dejavu"):
    os.environ.setdefault("QT_QPA_FONTDIR", "/usr/share/fonts/truetype/dejavu")
import cv2
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from ground_est import GroundDetector            # noqa: E402
import mount_calib                               # noqa: E402
import imu_calib                                 # noqa: E402
from criteria import load_criteria               # noqa: E402
from live_map import LiveHeightMap, cell_colors, classify_heights, filter_hazards, nearest_in_band, oblique_image, tidy_low, tmap_local_grid, top_image   # noqa: E402
from tmap_live import CRIT_DEFAULT, REC, TMAP_DEFAULT, git_head, load_tmap, mount_from_gravity   # noqa: E402
from gm_features import ConsoleMap, KText, OverlayBridge, Recorder, draw_overlay, intr_dict, load_constants   # noqa: E402
from gm_worker import GmProc, hybrid_cores       # noqa: E402


WHITE, GREEN, RED, YEL, CYAN, GRAY = (255, 255, 255), (60, 200, 60), (40, 40, 230), (40, 220, 240), (230, 200, 40), (130, 130, 130)


def cam_angles(up):
    u = up / np.linalg.norm(up)
    return math.degrees(math.asin(max(-1.0, min(1.0, u[2])))), math.degrees(math.atan2(u[0], -u[1]))


def put(img, s, org, col=WHITE, sc=0.5, th=1):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, (0, 0, 0), th + 2, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, sc, col, th, cv2.LINE_AA)


def blobs(det, P, min_px):
    """장애물 마스크 덩어리 → [(x, y, w, h, 높이 m, 앞 거리 m, 폭 m, 옆 위치 m)]"""
    out = []
    if det.obst_mask is None or det.n is None:
        return out
    om = cv2.morphologyEx(det.obst_mask.astype(np.uint8), cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))   # 6판: 잘게 쪼개진 조각 합침(상자 수 줄임)
    nlab, lab, st, _ = cv2.connectedComponentsWithStats(om, 8)
    H = P @ det.n + det.d
    for i in range(1, nlab):
        x, y, w, h, area = st[i]
        if area < min_px:
            continue
        sel = (lab == i) & det.obst_mask                                 # 통계는 원래 장애물 픽셀만(닫힘으로 메운 틈 제외)
        if not sel.any():
            continue
        hh, ff, ll = H[sel], det.fwd[sel], det.lat[sel]
        out.append((int(x), int(y), int(w), int(h), float(np.percentile(hh, 95)), float(np.percentile(ff, 5)),
                    float(np.percentile(ll, 95) - np.percentile(ll, 5)), float(np.median(ll))))
    return out


def birdseye(det, size=360, fwd_max=2.5, lat_max=1.25):
    """위에서 본 지도: 바닥(초록)·장애물(빨강) 점, 0.5 m 눈금, 구역 띠(반폭)."""
    img = np.full((size, size, 3), 25, np.uint8)
    sx, sy = size / (2 * lat_max), size / fwd_max
    for r in np.arange(0.5, fwd_max + 1e-6, 0.5):
        y = int(size - r * sy)
        cv2.line(img, (0, y), (size, y), (60, 60, 60), 1)
        put(img, f"{r:.1f}m", (4, y - 3), GRAY, 0.4)
    for lat in (-det.half_width, det.half_width):
        x = int(size / 2 + lat * sx)
        cv2.line(img, (x, 0), (x, size), (90, 90, 40), 1)
    if det.n is not None and det.floor_mask is not None:
        step = 3
        F, L = det.fwd[::step, ::step], det.lat[::step, ::step]
        for mask, col in ((det.floor_mask, (40, 110, 40)), (det.obst_mask, RED)):
            mm = mask[::step, ::step] & (F > 0) & (F < fwd_max) & (np.abs(L) < lat_max)
            xs = (size / 2 + L[mm] * sx).astype(int)
            ys = (size - F[mm] * sy).astype(int)
            img[np.clip(ys, 0, size - 1), np.clip(xs, 0, size - 1)] = col
    cv2.drawMarker(img, (size // 2, size - 4), CYAN, cv2.MARKER_TRIANGLE_UP, 14, 2)
    put(img, "top view (lines = half width)", (4, 14), GRAY, 0.4)
    return img


def mount_result(snaps, R_ad, cal=None):
    """장착면 아래·렌즈 면 아래 자세의 위 방향 → 광축·렌즈 면이 장착면 법선보다 숙인 각. 180° 돌린 짝이 있으면 평균(바닥 기울기 상쇄).
    자세 이름은 위 방향 부호로 다시 정한다(키를 바꿔 눌러도 됨): 장착면 아래 = 광축(z) 성분 +, 렌즈 아래 = −. cal이 있으면 가속도 보정 뒤 계산."""
    ups, swapped = {}, []
    for k, s in snaps.items():
        if not isinstance(s, dict) or "accel_mean_raw" not in s:
            continue
        a_ = np.array(s["accel_mean_raw"], float)
        if cal is not None:
            a_ = imu_calib.apply(a_, cal)
        u = R_ad @ a_
        u = u / np.linalg.norm(u)
        name = ("mount_down" if u[2] > 0 else "lens_down") + ("_r180" if k.endswith("_r180") else "")
        if name != k:
            swapped.append(f"{k}->{name}")
        ups[name] = u

    def avg(a, b):
        if a not in ups:
            return None, False
        if b in ups:
            m = ups[a] + ups[b]              # 제자리 180° 회전: 바닥 기울기 성분은 장치 좌표에서 부호가 바뀜 → 평균이 바닥 법선에 가까움
            return m / np.linalg.norm(m), True
        return ups[a], False
    f, r1 = avg("mount_down", "mount_down_r180")
    if f is None:
        return None
    out = {"axis_down_deg": math.degrees(math.atan2(-f[1], f[2])), "axis_yaw_deg": math.degrees(math.atan2(f[0], f[2])), "reversal": r1,
           "relabeled": swapped, "calibrated": cal is not None}
    g, r2 = avg("lens_down", "lens_down_r180")
    if g is not None:
        nrm = -g
        out.update(lens_face_down_deg=out["axis_down_deg"] - math.degrees(math.atan2(-nrm[1], nrm[2])),
                   lens_face_yaw_deg=out["axis_yaw_deg"] - math.degrees(math.atan2(nrm[0], nrm[2])),
                   lens_face_vs_axis_deg=math.degrees(math.acos(max(-1.0, min(1.0, float(nrm[2]))))), reversal=r1 and r2)
    if r1:
        out["mount_r180_diff_deg"] = math.degrees(math.acos(max(-1.0, min(1.0, float(ups["mount_down"] @ ups["mount_down_r180"])))))
    out["note"] = "axis = IMU→카메라 SDK 외부 파라미터를 믿은 광축 숙임각, lens_face = 렌즈 면이 광축에 수직이라 보고 두 자세로 구한 숙임각(IMU 정렬과 무관)"
    return out


def fmt_mount(R, tag):
    if not R:
        return f"MOUNT{tag}: -"
    s = f"MOUNT{tag}: optical axis {R['axis_down_deg']:+.2f} deg down"
    if "lens_face_down_deg" in R:
        s += f", lens face {R['lens_face_down_deg']:+.2f} deg down"
    return s + (" [180-avg]" if R.get("reversal") else "") + (" [keys relabeled]" if R.get("relabeled") else "")


def load_cal(path):
    cal = json.load(open(path))
    if "accel_bias_m_s2" not in cal:
        raise SystemExit(f"{path}: 보정 결과(accel_bias_m_s2) 없음")
    if not cal.get("valid"):
        print(f"⚠ {path}: valid=false 보정 — 참고로만 쓴다", flush=True)
    return cal


def cover_str(poses):
    if not poses:
        return "-"
    a = np.array([p["accel_mean"] for p in poses])
    d = a / np.linalg.norm(a, axis=1)[:, None]
    c = imu_calib.Q["cover_cos"]
    return " ".join((f"+{ax}" if d[:, k].max() >= c else f"({'+' + ax})") + " " + (f"-{ax}" if d[:, k].min() <= -c else f"({'-' + ax})")
                    for k, ax in enumerate("XYZ"))


def draw_corridor(view, det, K, half_width, fwd_max=3.0):
    """바닥 평면 위 통로 띠(±반폭)와 1·2·3 m 거리선을 컬러 영상에 그린다. 평면은 카메라 좌표(m) — 해상도 솎기와 무관, 투영은 원 해상도 K."""
    if det.n is None:
        return
    n = det.n / np.linalg.norm(det.n)
    foot = -det.d * n
    f = np.array([0.0, 0.0, 1.0]) - n[2] * n
    r = np.array([1.0, 0.0, 0.0]) - n[0] * n
    f, r = f / np.linalg.norm(f), r / np.linalg.norm(r)

    def proj(F, Lt):
        p = foot[None, :] + np.outer(F, f) + np.outer(Lt, r)
        ok = p[:, 2] > 0.1
        uv = np.c_[K.fx * p[:, 0] / np.maximum(p[:, 2], 1e-6) + K.ppx, K.fy * p[:, 1] / np.maximum(p[:, 2], 1e-6) + K.ppy]
        return uv, ok
    F = np.arange(0.3, fwd_max + 1e-6, 0.05)
    for lat in (-half_width, half_width):
        uv, ok = proj(F, np.full_like(F, lat))
        pts = uv[ok & (np.abs(uv[:, 0]) < 5000) & (np.abs(uv[:, 1]) < 5000)].astype(np.int32)
        if len(pts) > 1:
            cv2.polylines(view, [pts], False, (60, 200, 220), 1, cv2.LINE_AA)
    for dist in (1.0, 2.0, 3.0):
        if dist > fwd_max:
            continue
        Lt = np.linspace(-half_width, half_width, 12)
        uv, ok = proj(np.full_like(Lt, dist), Lt)
        pts = uv[ok].astype(np.int32)
        if len(pts) > 1 and (0 <= pts[:, 1]).any() and (pts[:, 1] < view.shape[0]).any():
            cv2.polylines(view, [pts], False, (60, 200, 220), 1, cv2.LINE_AA)
            put(view, f"{dist:.0f}m", (int(pts[-1, 0]) + 4, int(pts[-1, 1]) + 4), (60, 200, 220), 0.45)


class ImuRing:
    """IMU 표본 고리 버퍼(시각, x, y, z) — 콜백이 한 줄씩 쓰고 화면 루프가 시간 순 배열로 읽음(7판 끝: 3000개 덱을 매 루프 np.array로 바꾸던 약 3 ms 없앰).
    잠금은 부르는 쪽(콜백·화면 루프가 같은 lock)."""

    def __init__(self, n):
        self.a, self.i, self.n = np.zeros((n, 4)), 0, 0

    def append(self, row):
        self.a[self.i] = row
        self.i = (self.i + 1) % len(self.a)
        self.n = min(self.n + 1, len(self.a))

    def __len__(self):
        return self.n

    def array(self):
        if self.n < len(self.a):
            return self.a[:self.n].copy()
        return np.concatenate([self.a[self.i:], self.a[:self.i]])


class VideoSink:
    """키 w 화면 녹화 쓰기 스레드(7판 끝): 1422×1030 mp4v 쓰기가 프레임당 수 ms라 화면 루프에서 뺌. (화면, 반복 수)를 큐로 받아 순서대로 씀.
    큐가 차면(디스크가 느리면) 반복 수만 늘려 실시간 길이는 지킴."""

    def __init__(self, path, fps, size):
        self.w = cv2.VideoWriter(path, cv2.VideoWriter_fourcc(*"mp4v"), fps, size)
        self.q = collections.deque()
        self.cv = threading.Condition()
        self.done = False
        self.t = threading.Thread(target=self._run, name="video", daemon=True)
        self.t.start()

    def put(self, frame, n):
        with self.cv:
            if len(self.q) >= 30:
                f0, n0 = self.q[-1]
                self.q[-1] = (f0, n0 + n)
            else:
                self.q.append((frame, n))
            self.cv.notify()

    def _run(self):
        while True:
            with self.cv:
                self.cv.wait_for(lambda: self.q or self.done)
                if not self.q and self.done:
                    break
                frame, n = self.q.popleft()
            for _ in range(n):
                self.w.write(frame)
        self.w.release()

    def close(self):
        with self.cv:
            self.done = True
            self.cv.notify()
        self.t.join(10.0)


class TmapBridge:
    """광민 MapRuntime(terrain_map, ROS 없는 틱 논리) 다리 — 부모 쪽(7판 끝: 런타임 자체는 gm_worker 자식 프로세스에서, GIL 다툼 없이).
    여기서는 스트림 시각 맞춤(stamp), 깊이·IMU 보내기, 콘솔 수신기(LatestMapReceiver.ingest)·설정값(MapConfig, 부모에서도 만들어 범위 검사·표시)만.
    광민 코드는 별도 작업트리에서 import만 하고 고치지 않는다."""

    def __init__(self, T, camera, cfg_kw, gm):
        self.T, self.camera, self.cfg_kw, self.gm = T, camera, dict(cfg_kw), gm
        self.off, self.domain, self.ready, self.last_now = {}, {}, False, {}
        self.lock = threading.Lock()
        self.rx = self.cfg = None
        self.gen = 0
        self.n_depth = 0
        self.wheel_gap_max = 0.0
        self.summary = "terrain map: (자식 프로세스 시작 중)"
        self._rx_ms, self._rx_n = 0.0, 0
        gm.handlers["map"] = self._on_map
        gm.handlers["stats"] = self._on_stats
        gm.send("map_init", dict(fx=camera.fx, fy=camera.fy, cx=camera.cx, cy=camera.cy, width=camera.width, height=camera.height), self.cfg_kw)

    def stamp(self, kind, ts_ms, now, frame):
        """장치 프레임 시각 → monotonic. 전역 시각(SDK가 호스트에 맞춤)이면 바로 옮기고, 하드웨어 시계면 최소 지연선을 따라가되 초당 0.6 %까지 올라가게
        (장치 시계가 호스트보다 약 0.4 % 느림 — 시작 2 s에 고정하던 7판 첫 방식은 몇 분 뒤 0.5 s 넘게 어긋나 광민 지도가 시대 반복·0칸, 10-22 밤)."""
        t = ts_ms / 1000.0
        dom = frame.get_frame_timestamp_domain()
        self.domain.setdefault(kind, str(dom))
        if dom == rs.timestamp_domain.global_time:
            return t - (time.time() - time.monotonic())
        o = now - t
        prev, last = self.off.get(kind), self.last_now.get(kind, now)
        self.off[kind] = o if prev is None else min(prev + 0.006 * max(0.0, now - last), o)
        self.last_now[kind] = now
        return t + self.off[kind]

    def start(self, pitch, roll):
        cfg = self.T["MapConfig"](camera_pitch_down_deg=pitch, camera_roll_deg=roll, **self.cfg_kw)   # 범위 밖이면 여기서 ValueError(자식에 안 보냄)
        rx = self.T["LatestMapReceiver"](0)
        with self.lock:
            old = self.rx
            self.gen += 1
            self.cfg, self.rx = cfg, rx
            self.ready = True
            gen = self.gen
        if old is not None:
            old.close()
        self.gm.send("map_start", pitch, roll, gen)

    def _on_map(self, gen, raws, now):
        t0 = time.perf_counter()
        with self.lock:
            rx = self.rx if gen == self.gen else None                # 새로 시작하기 전 런타임의 패킷은 버림
        if rx is not None:
            for raw in raws:
                rx.ingest(raw, now)
        self._rx_ms += (time.perf_counter() - t0) * 1000.0
        self._rx_n += len(raws)

    def rx_stat(self):
        """마지막 호출 뒤 수신 스레드가 패킷을 푼 시간(ms)·패킷 수 — 2 s 기록용."""
        out = (self._rx_ms, self._rx_n)
        self._rx_ms, self._rx_n = 0.0, 0
        return out

    def _on_stats(self, d):
        self.summary, self.wheel_gap_max = d["summary"], d["wheel_gap_max"]

    def on_depth(self, arr, ts_ms, now, frame, mono_ns):
        """깊이 한 장을 두 런타임(지도·오버레이)에 같이 보낸다 — 지도 시각은 시작(ready) 뒤에만."""
        t = self.stamp("depth", ts_ms, now, frame)
        self.gm.depth(arr, t if self.ready else None, now, mono_ns)
        self.n_depth += 1

    def on_imu(self, kind, ts_ms, now, frame, v):
        t = self.stamp(kind, ts_ms, now, frame)
        if self.ready:
            self.gm.imu(kind, t, now, v)

    def snapshot(self, now):
        with self.lock:
            rx = self.rx
        return rx.snapshot(now) if rx is not None else None

    def close(self):
        if self.rx is not None:
            self.rx.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dmin", type=float, default=0.12, help="카메라 화면 바닥 찾기: 카메라 아래 최소 높이(m)")
    ap.add_argument("--dmax", type=float, default=1.6, help="카메라 아래 최대 높이(m) — 광민 지도 검출기는 따로 0.25–0.80 고정")
    ap.add_argument("--max-dist", type=float, default=5.0, help="바닥·장애물에 쓰는 최대 거리(m)")
    ap.add_argument("--map-range", type=float, default=4.0, help="광민 지도 range_max_m(1–5 m, 광민 기본 3)")
    ap.add_argument("--tilt-max", type=float, default=40.0)
    ap.add_argument("--grav-tol", type=float, default=12.0)
    ap.add_argument("--min-dist", type=float, default=None, help="L515 최소 거리(mm, 기본 장치값 490)")
    ap.add_argument("--obst-h", type=float, default=0.05, help="장애물로 칠할 바닥 위 높이(m, terrain.py 기본 0.05)")
    ap.add_argument("--min-blob", type=int, default=150, help="장애물 덩어리 최소 픽셀")
    ap.add_argument("--out", default=REC, help="저장 화면·지도·녹화·보정 파일 폴더(기본 L515_OUT_DIR 또는 지금 폴더의 recordings)")
    ap.add_argument("--snap-sec", type=float, default=3.0)
    ap.add_argument("--calib-sec", type=float, default=5.0, help="보정 자세 하나 기록 시간(imu_calib 기록 절차와 같은 5 s)")
    ap.add_argument("--calib", default=None, help="가속도 보정 JSON(기본 = recordings의 가장 최근 유효 imu_calib_live_*.json, 'none'이면 안 씀)")
    ap.add_argument("--recalc", default=None, help="imu_mount_*.json·imu_snaps_*.json → 장착 각만 다시 계산하고 끝(카메라 안 씀)")
    ap.add_argument("--criteria", default=CRIT_DEFAULT if os.path.exists(CRIT_DEFAULT) else None)
    ap.add_argument("--profile", default=None, help="기준값 프로필(기본 = JSON의 default_profile)")
    ap.add_argument("--map", choices=("gwangmin", "own"), default="gwangmin", help="지도 원천: 광민 MapRuntime(기본, 세계 고정 칸·자이로 회전) 또는 자체 LiveHeightMap")
    ap.add_argument("--tmap-root", default=TMAP_DEFAULT, help="광민 terrain_map·terrain_overlay가 있는 저장소(기본 L515_TMAP_ROOT 또는 이 저장소, 읽기 전용)")
    ap.add_argument("--cell", type=float, default=0.03, help="지도 칸(m) — 광민 MapConfig cell_m(0.02–0.20) 또는 자체 지도 칸")
    ap.add_argument("--gwangmin-thresholds", action="store_true", help="광민 코드 문턱을 광민 기본(8 cm)으로 — 기본은 기준값 측정값 문턱(돌출 = 턱 주의 − 1 cm, 도랑 = 도랑 주의 − 1 cm)")
    ap.add_argument("--no-code-gate", action="store_true", help="광민 다수결 코드로 거르지 않음(마지막 관측 높이만으로 분류 — 한 프레임 튐이 그대로 보임)")
    ap.add_argument("--min-neighbors", type=int, default=None, help="주의·불가 칸은 8이웃 중 이만큼 주의·불가여야 표시(기본: 광민 다수결 쓰면 0, 자체 지도 1)")
    ap.add_argument("--stride", type=int, default=2, help="카메라 화면 바닥 찾기 해상도 솎기(1 = 960×540, 2 = 480×270)")
    ap.add_argument("--confidence", type=int, default=2, help="L515 confidence_threshold(0–3, 장치 기본 1). 7판 고침: 2 — 10-22 잡음 실측(tools/l515/noise_bench.py): "
                                                              "0은 화면 99 %를 가짜 깊이로 채워 잡음·가짜 장애물(사용자 '노이즈 너무 심함'), 2는 가짜 장애물 0 %·바닥 95 % 2.85 m(VGA 문턱 1은 2.48 m)")
    ap.add_argument("--depth-res", default="320x240", help="깊이 해상도(320x240·640x480·1024x768). 7판 기본 QVGA — 실측: 같은 문턱에서 VGA보다 바닥이 멀리(2.48 → 2.85 m)")
    ap.add_argument("--mask-vote", type=int, default=3, help="카메라 화면 바닥·장애물 색을 최근 N프레임 다수결로(깜빡임 줄임, 1 = 끔)")
    ap.add_argument("--preset", type=int, default=4, help="L515 visual_preset(2 No Ambient·3 Low Ambient·4 Max Range·5 Short Range, -1 = 안 바꿈). 실측: 밝은 실내에서 수신 이득 18 프리셋은 나빠짐")
    ap.add_argument("--headless", action="store_true", help="창 없이(Jetson·opencv-headless) — 브라우저 보기 --serve-host:--serve-port")
    ap.add_argument("--serve-host", default="127.0.0.1", help="브라우저 보기 주소(다른 기기에서 보려면 테일넷 IP나 0.0.0.0 — 인증 없음, 신뢰하는 네트워크만)")
    ap.add_argument("--serve-port", type=int, default=8090)
    ap.add_argument("--noise-filter", type=int, default=None, help="L515 noise_filtering(0–6, 장치 기본 4)")
    ap.add_argument("--map-fwd", type=float, default=3.5)
    ap.add_argument("--map-lat", type=float, default=1.5)
    ap.add_argument("--keep", type=float, default=None, help="이 시간 넘게 안 보인 칸은 모름(s, 기본 광민 30·자체 2)")
    ap.add_argument("--dim", type=float, default=3.0, help="광민 지도: 이보다 오래된 칸은 어둡게(s)")
    ap.add_argument("--map-hz", type=float, default=2.0, help="지도·비스듬히·깊이 그림 다시 그리기 횟수(초당)")
    ap.add_argument("--move-dps", type=float, default=3.0, help="자체 지도: 자이로가 이보다 빠르면 쌓기 멈춤(°/s)")
    ap.add_argument("--view", choices=("class", "height", "console"), default="class", help="처음 지도 보기(v로 바꿈): 기준값 단계·높이·광민 콘솔")
    ap.add_argument("--oblique-tilt", type=float, default=0.35, help="2.5D 비스듬히 보기 내려다보는 정도(0 = 6판 시점 … 1, 키 [ ] 로 0.1씩)")
    ap.add_argument("--max-fps", type=float, default=0.0, help="처리 속도 상한(0 = 끔). 얇은 노트북은 계속 30 fps로 돌면 80 °C대에서 발열 제한으로 15–20 s씩 "
                                                                 "0.5–1 GHz까지 떨어짐(10-22 밤 실측) — 20–24로 두면 고르게")
    ap.add_argument("--video-fps", type=float, default=20.0, help="키 w 화면 녹화의 출력 fps(실시간 속도로 맞춰 씀)")
    ap.add_argument("--seconds", type=float, default=0, help="이 시간 뒤 자동 종료(시험용, 마지막 화면 저장)")
    a = ap.parse_args()
    if a.calib is None:                                  # 기록 폴더 + 기본 recordings 폴더에서 가장 최근 유효 보정(7판: --out을 바꿔도 찾게)
        cands = glob.glob(os.path.join(a.out, "imu_calib_live_*.json")) + glob.glob(os.path.join(REC, "imu_calib_live_*.json"))
        for p_ in sorted(set(cands), key=os.path.basename, reverse=True):
            if json.load(open(p_)).get("valid"):
                a.calib = p_
                break
    cal = load_cal(a.calib) if a.calib and a.calib != "none" else None
    if a.recalc:
        d = json.load(open(a.recalc))
        R_ad = np.array(d.get("R_accel_to_depth", np.eye(3).tolist()), float)
        for c_ in ([None, cal] if cal else [None]):
            r_ = mount_result(d, R_ad, c_)
            print("보정 적용" if c_ else "보정 없음", json.dumps({k_: (round(v_, 2) if isinstance(v_, float) else v_) for k_, v_ in (r_ or {}).items() if k_ != "note"},
                                                              ensure_ascii=False))
        return
    os.makedirs(a.out, exist_ok=True)
    crit = load_criteria(a.criteria, a.profile) if a.criteria else None
    rules = crit["rules"] if crit else None
    keep = a.keep if a.keep is not None else (30.0 if a.map == "gwangmin" else 2.0)
    tm = ob = None
    T = load_tmap(a.tmap_root)                      # 광민 코드(읽기 전용 작업트리) — 오버레이 런타임은 지도 원천과 상관없이 늘
    gm = GmProc()                                   # 광민 두 런타임 자식 프로세스 — 카메라·스레드를 열기 전에 fork(자식은 E코어로 스스로 옮김)
    hc = hybrid_cores()
    if hc and os.environ.get("OV_PIN", "1") == "1":  # P/E 혼합 CPU(노트북 i7-1260P): 화면 루프·카메라는 P코어에. 메인 스레드가 E코어로 옮겨지면 모든 구간이
        os.sched_setaffinity(0, hc[0])                # 고르게 2–3배 느려지는 저하(10-22 밤 2 s 기록)를 막음. 이후 만드는 스레드가 물려받음. OV_PIN=0이면 끔
    gm.start_rx()
    tm_head = git_head(a.tmap_root)
    consts = load_constants(a.tmap_root)
    kt = KText()
    if a.map == "gwangmin":
        cfg_kw = {"cell_m": a.cell, "range_max_m": float(min(5.0, max(1.0, a.map_range)))}
        if not a.gwangmin_thresholds and rules:      # 광민 코드 경계 = 표시 단계(주의) 경계 → 다수결 거르기가 같은 기준
            cfg_kw["obstacle_height_m"] = round(float(rules["step_up_m"]["caution_from"]) - 0.01, 4)
            cfg_kw["ditch_depth_m"] = round(float(rules["trench_depth_m"]["caution_from"]) - 0.01, 4)
    min_nb = a.min_neighbors if a.min_neighbors is not None else (0 if (a.map == "gwangmin" and not a.no_code_gate) else 1)

    cfg = rs.config()
    dw, dh = (int(v) for v in a.depth_res.lower().split("x"))
    cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, 30)
    cfg.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    cfg.enable_stream(rs.stream.accel)
    cfg.enable_stream(rs.stream.gyro)
    lock = threading.Lock()
    latest = {"fs": None, "n": 0}
    imu = {"accel": ImuRing(3000), "gyro": ImuRing(3000)}        # 가장 긴 창 = 보정 자세 5 s(+0.5). numpy 고리 버퍼(매 루프 리스트 → 배열 변환 없앰)
    fcv = threading.Condition(lock)                               # 새 프레임 알림(화면 루프가 같은 화면을 다시 칠하지 않게)
    R = {"ad": np.eye(3), "gd": np.eye(3)}
    errs = {}

    def cb(f):
        try:
            now = time.monotonic()
            if f.is_frameset():
                f.keep()
                fs = f.as_frameset()
                with lock:
                    latest["fs"] = fs
                    latest["n"] += 1
                    fcv.notify_all()
                d = fs.get_depth_frame()
                if d and (tm is not None or ob is not None):
                    arr = np.asanyarray(d.get_data()).copy()                 # 원본 640×480 — 광민 두 런타임(자식 프로세스)이 같이 씀
                    if tm is not None:
                        tm.on_depth(arr, d.get_timestamp(), now, d, time.monotonic_ns())
                    else:
                        gm.depth(arr, None, now, time.monotonic_ns())
            elif f.is_motion_frame():
                m = f.as_motion_frame()
                v = m.get_motion_data()
                k = "accel" if m.get_profile().stream_type() == rs.stream.accel else "gyro"
                with lock:
                    imu[k].append((m.get_timestamp() / 1000.0, v.x, v.y, v.z))
                if tm is not None:
                    raw = np.array([v.x, v.y, v.z])
                    vv_ = R["ad"] @ (imu_calib.apply(raw, cal) if cal is not None else raw) if k == "accel" else R["gd"] @ raw
                    tm.on_imu(k, m.get_timestamp(), now, m, tuple(float(x) for x in vv_))
        except Exception as e:  # noqa: BLE001 — SDK가 콜백 예외를 삼키므로 화면에 띄운다
            errs["cb"] = repr(e)

    pipe = rs.pipeline()
    prof = pipe.start(cfg, cb)
    _OPEN.append(pipe)
    for sen_ in prof.get_device().query_sensors():           # 전역 시각 켬(10-22 밤): L515 하드웨어 시계는 호스트보다 약 0.4 % 느려 몇 분이면 0.5 s 넘게 어긋남
        try:                                                  # → 광민 지도 스트림 시각 어긋남(시대 반복, 지도 0칸). 전역 시각이면 SDK가 호스트 시계에 맞춰 줌
            if sen_.supports(rs.option.global_time_enabled):
                sen_.set_option(rs.option.global_time_enabled, 1.0)
        except RuntimeError:
            pass
    dev = prof.get_device()
    ds = dev.first_depth_sensor()
    if a.min_dist is not None and ds.supports(rs.option.min_distance):
        ds.set_option(rs.option.min_distance, float(a.min_dist))
    if a.preset is not None and a.preset >= 0 and ds.supports(rs.option.visual_preset):   # 프리셋 먼저(다른 설정을 덮어씀) → 그 뒤 개별 설정
        ds.set_option(rs.option.visual_preset, float(a.preset))
    for opt_, val_ in ((rs.option.confidence_threshold, a.confidence), (rs.option.noise_filtering, a.noise_filter)):
        if val_ is not None and ds.supports(opt_):
            ds.set_option(opt_, float(val_))
    sens = {nm: (int(ds.get_option(o_)) if ds.supports(o_) else None) for nm, o_ in (("conf", rs.option.confidence_threshold), ("noise", rs.option.noise_filtering))}
    scale = ds.get_depth_scale()
    cprof = prof.get_stream(rs.stream.color).as_video_stream_profile()
    K = cprof.get_intrinsics()                    # 깊이를 컬러에 정렬하므로 컬러 내부 파라미터로 계산
    e = prof.get_stream(rs.stream.accel).get_extrinsics_to(prof.get_stream(rs.stream.color))
    R_ac = np.array(e.rotation, float).reshape(3, 3).T     # accel → color(SDK 회전은 열 우선) — 바닥 찾기(컬러 좌표) 중력 확인용
    e2 = prof.get_stream(rs.stream.accel).get_extrinsics_to(prof.get_stream(rs.stream.depth))
    R_ad = np.array(e2.rotation, float).reshape(3, 3).T    # accel → depth — 장착 각·자세는 라이다(깊이) 광축 기준(컬러 축과 약 1.7° 다름)
    R["ad"] = R_ad
    R["gd"] = np.array(prof.get_stream(rs.stream.gyro).get_extrinsics_to(prof.get_stream(rs.stream.depth)).rotation, float).reshape(3, 3).T
    align = rs.align(rs.stream.color)
    S_ = max(1, int(a.stride))
    Hs, Ws = len(range(0, K.height, S_)), len(range(0, K.width, S_))
    det = GroundDetector(K.fx / S_, K.fy / S_, K.ppx / S_, K.ppy / S_, scale, shape=(Hs, Ws), obst_h=a.obst_h, half_width=0.4445, max_h=2.0,
                         max_dist=a.max_dist, hmin=a.dmin, hmax=a.dmax, grav_tol=a.grav_tol)   # 솎은 정렬 깊이(컬러 좌표, u' = u/s → 내부 파라미터 1/s)
    e_dc = prof.get_stream(rs.stream.depth).get_extrinsics_to(prof.get_stream(rs.stream.color))
    R_dtoc, t_dtoc = np.array(e_dc.rotation, float).reshape(3, 3).T, np.array(e_dc.translation, float)   # p_color = R p_depth + t

    def plane_to_depth(n_c, d_c):                                # 컬러 좌표 평면(n·p + d = 0) → 깊이 좌표(광민 지도·캘리브레이션 저장 기준)
        return R_dtoc.T @ n_c, float(d_c + n_c @ t_dtoc)

    def plane_to_color(n_d, d_d):
        n_c = R_dtoc @ np.asarray(n_d, float)
        return n_c, float(d_d - n_c @ t_dtoc)
    mcal = mount_calib.load(None)
    calib_run, mcal_msg, use_prior = None, "", True
    hm = LiveHeightMap(cell=a.cell, fwd_max=a.map_fwd, lat_max=a.map_lat, keep_s=keep, rules=rules) if a.map == "own" else None
    uu, vv = np.meshgrid(np.arange(K.width), np.arange(K.height))
    xn, yn = (uu - K.ppx) / K.fx, (vv - K.ppy) / K.fy                       # 원 해상도(클릭 점만)
    serial = dev.get_info(rs.camera_info.serial_number)
    print(f"L515 {serial} · 정렬 깊이 {K.width}x{K.height}(컬러 좌표) · 최소 거리 {ds.get_option(rs.option.min_distance):.0f} mm · "
          f"바닥 범위 {a.dmin}–{a.dmax} m · 중력 확인 ±{a.grav_tol}° · accel→depth 회전 대각 {np.round(np.diag(R_ad), 5)} · color↔depth 축 차 "
          f"{math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R_ac @ R_ad.T) - 1) / 2)))):.2f}°")

    mount = {"pitch": 15.0, "roll": 0.0, "warn": ""}

    def measure_mount():
        """최근 1 s 가속도(보정 뒤)로 '가상 로버' 장착 자세 — 광민 MapConfig 범위(숙임 0–35°·옆 ±20°) 밖이면 이전 값 유지·경고."""
        with lock:
            A = imu["accel"].array()
        A = A[A[:, 0] > A[-1, 0] - 1.0] if len(A) else A
        if len(A) < 20:
            mount["warn"] = "no accel"
            return False
        am = A[:, 1:4].mean(0)
        p_, r_ = mount_from_gravity(R_ad @ (imu_calib.apply(am, cal) if cal is not None else am))
        if os.environ.get("OV_DEBUG"):
            print("[debug] 표본", A.shape, "시각 범위", round(float(A[0, 0]), 3), round(float(A[-1, 0]), 3), "평균", np.round(am, 3),
                  "전체 마지막 5", np.round(imu["accel"].array()[-5:, 0], 3), "→", round(p_, 1), round(r_, 1), flush=True)
        ok_ = True
        if tm is not None:                                       # 범위는 광민 MapConfig가 정한다(숙임 0–35°, 옆 ±20° — 직접 만들어 보고 거부되면 경고)
            try:
                T["MapConfig"](camera_pitch_down_deg=round(p_, 2), camera_roll_deg=round(r_, 2), **cfg_kw)
            except ValueError:
                ok_ = False
        if ok_:
            mount.update(pitch=round(p_, 2), roll=round(r_, 2), warn="")
            return True
        if True:
            mount["warn"] = f"measured pitch {p_:.0f} roll {r_:.0f} out of range - aim at floor, hold still, press m"
            print(f"⚠ 잰 장착 자세(숙임 {p_:.1f}°·옆 {r_:.1f}°)가 광민 범위(숙임 0–35°·옆 ±20°) 밖 — {mount['pitch']}°로. 바닥을 보게 가만히 두면 자동으로 다시 잼(m도 됨)", flush=True)
            return False

    if mcal is not None and mcal.get("serial") not in (None, serial):
        print(f"장착 캘리브레이션 파일이 다른 L515({mcal.get('serial')}) 것 — 쓰지 않음", flush=True)
        mcal = None
    if a.map == "gwangmin":
        dprof = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        tm = TmapBridge(T, T["CameraModel"](fx=dprof.fx, fy=dprof.fy, cx=dprof.ppx, cy=dprof.ppy, width=dprof.width, height=dprof.height), cfg_kw, gm)
        time.sleep(2.0)                                         # 스트림 시각 차이 최솟값·장착 자세
        if mcal is not None:                                    # 7판: 장착 캘리브레이션이 있으면 그 값(바닥 법선 기준)
            mount.update(pitch=float(mcal["pitch_down_deg"]), roll=float(mcal["roll_deg"]), warn="")
            print(f"장착 캘리브레이션 불러옴({mcal['time']}): 숙임 {mount['pitch']}°·옆 {mount['roll']}°·높이 {mcal['camera_height_m']} m", flush=True)
        else:
            measure_mount()
        tm.start(mount["pitch"], mount["roll"])
        print(f"지도: 광민 terrain_map {tm_head} · 시각 기준 {tm.domain} · 칸 {tm.cfg.cell_m} m · 광민 코드 문턱 돌출 {tm.cfg.obstacle_height_m}/도랑 {tm.cfg.ditch_depth_m} m"
              f" · 장착 숙임 {mount['pitch']}°·옆 {mount['roll']}° · 표시 = 기준값 {os.path.basename(a.criteria) if crit else '없음'} {crit['criteria_profile'] if crit else ''}"
              f" · 튐 거르기: 다수결 {'끔' if a.no_code_gate else '켬'}, 이웃 {min_nb}", flush=True)
    else:
        print(f"지도: 자체 LiveHeightMap · 칸 {a.cell} m · 기준값 {os.path.basename(a.criteria) if crit else '없음'}", flush=True)
    dprof_i = prof.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
    ob = OverlayBridge(T["CameraModel"](fx=dprof_i.fx, fy=dprof_i.fy, cx=dprof_i.ppx, cy=dprof_i.ppy, width=dprof_i.width, height=dprof_i.height),
                       T["CameraModel"](fx=K.fx, fy=K.fy, cx=K.ppx, cy=K.ppy, width=K.width, height=K.height), gm)
    cmap = ConsoleMap(T, consts, kt) if tm is not None else None
    rec_meta = {"depth_scale_m": scale, "depth_intrinsics": intr_dict(dprof_i), "color_intrinsics": intr_dict(K),
                "depth_to_color": {"rotation": list(prof.get_stream(rs.stream.depth).get_extrinsics_to(prof.get_stream(rs.stream.color)).rotation),
                                   "translation_m": list(prof.get_stream(rs.stream.depth).get_extrinsics_to(prof.get_stream(rs.stream.color)).translation)}}
    print(f"광민 오버레이 런타임(terrain_overlay) · 경로 반폭 {ob.cfg.path_half_width_m} m · 글꼴 {kt.path}", flush=True)
    print(f"보정 {a.calib or '없음'} · L515 confidence_threshold {sens['conf']}·noise_filtering {sens['noise']} · 카메라 화면 바닥 찾기 {Ws}x{Hs}", flush=True)
    print("마우스: 왼쪽 클릭 = 점 거리·높이 / 지도 칸 높이, 오른쪽 = 지움 | 키: 1·2 IMU 자세(3·4 = 180°), c 보정 자세(x 지우기), v 지도 색, m 지도 새로(장착 다시 잼), g, s, q")

    clicks, snaps, pending, cposes, cal_live, map_click = [], {}, None, [], None, None
    msg = ""
    win = "L515 live overlay v5"
    MAP = K.height
    mapW = int(round(MAP * 2 * a.map_lat / a.map_fwd))
    OBL = 300
    web = None
    if a.headless:
        from webview import WebView
        web = WebView(a.serve_host, a.serve_port)
        print(f"화면 없이 실행 — 브라우저: {web.url}", flush=True)
    else:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, K.width + mapW, K.height + OBL + 190)
    grid = {"nf": 1, "nl": 1, "cell": a.cell}

    drag = {"p": None}

    def on_mouse(ev, x, y, flags, param):
        nonlocal map_click
        in_map = K.width <= x < K.width + mapW and y < MAP
        if ev == cv2.EVENT_LBUTTONDOWN and x < K.width and y < K.height:
            clicks.append((x, y))
            del clicks[:-2]
        elif ev == cv2.EVENT_LBUTTONDOWN and in_map:
            if map_mode == "console":
                drag["p"] = (x, y)                                          # 콘솔 지도: 끌면 옮김(따라가기 꺼짐)
            else:
                map_click = ((MAP - 1 - y) / MAP * a.map_fwd, (x - K.width) / mapW * 2 * a.map_lat - a.map_lat)   # (앞, 오른쪽) m
        elif ev == cv2.EVENT_MOUSEMOVE and drag["p"] is not None and (flags & cv2.EVENT_FLAG_LBUTTON) and cmap is not None:
            cmap.drag_to(x - drag["p"][0], y - drag["p"][1])
            drag["p"] = (x, y)
        elif ev == cv2.EVENT_LBUTTONUP:
            drag["p"] = None
        elif ev == cv2.EVENT_MOUSEWHEEL and in_map and cmap is not None and map_mode == "console":
            cmap.zoom(cv2.getMouseWheelDelta(flags) > 0)
        elif ev == cv2.EVENT_RBUTTONDOWN:
            clicks.clear()
            map_click = None
    if web is None:
        cv2.setMouseCallback(win, on_mouse)

    t_start, t_log, n_seen, fps, t_fps, n_fps = time.time(), 0.0, -1, 0.0, time.time(), 0
    res, P, bl, color, depth = None, None, [], None, None
    show_gm, show_blobs, rec = True, True, None
    vid = None                                                            # 키 w 화면 녹화: {"w": VideoWriter, "path", "t0", "n"}

    def video_stop():
        nonlocal vid
        if vid is None:
            return
        if vid["w"] is not None:
            vid["w"].close()                                               # 남은 프레임을 다 쓰고 닫음
        src, dst = vid["path"], vid["path"].replace("_raw.mp4", ".mp4")
        dur = vid["n"] / a.video_fps
        vid = None
        import shutil, subprocess
        if shutil.which("ffmpeg"):                                         # 공유하기 쉬운 H.264(yuv420p)로 — 백그라운드
            pr_ = subprocess.Popen(["nice", "-n", "15", "ffmpeg", "-y", "-loglevel", "error", "-i", src, "-c:v", "libx264", "-preset", "veryfast",
                                    "-threads", "2", "-pix_fmt", "yuv420p", "-crf", "22",       # 낮은 우선순위·2코어: 바꾸는 동안 화면 루프를 방해하지 않게
                                    "-vf", "scale=trunc(iw/2)*2:trunc(ih/2)*2", "-movflags", "+faststart", dst],
                                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            threading.Thread(target=pr_.wait, name="ffmpeg-wait", daemon=True).start()   # 끝나면 회수(좀비로 남지 않게)
            print(f"화면 녹화 끝({dur:.1f} s) → {dst} (H.264로 바꾸는 중, 원본 {src})", flush=True)
        else:
            print(f"화면 녹화 끝({dur:.1f} s) → {src}", flush=True)
    mask_hist = collections.deque(maxlen=max(1, a.mask_vote))
    map_mode, t_gyr, gbias, moving = (a.view if (a.view != "console" or a.map == "gwangmin") else "class"), None, np.zeros(3), False
    oimg, mimg_c, map_key, frame, depth_s, valid_s, t_still_try = None, None, None, None, None, None, 0.0
    lv = dip = Z = known = age = None
    obl_tilt = float(np.clip(a.oblique_tilt, 0.0, 1.0))
    t_loop = time.perf_counter()
    solid = {}                                       # 칠하기 단색 화면(크기별 한 번)
    n_filtered = 0
    prof_t = collections.defaultdict(float)
    prof_n = [0]
    PROF = bool(os.environ.get("OV_PROFILE"))
    log_t, log_n, cam_n0 = collections.defaultdict(float), [0], [0]          # 2 s 기록용 구간 시간(늘 켬 — 주기적으로 느려지는 원인을 기록에 남김)

    def main_cpu():
        try:
            return int(open(f"/proc/self/task/{threading.get_native_id()}/stat").read().rsplit(")", 1)[1].split()[36])   # 필드 39 = 마지막에 돈 CPU
        except (OSError, ValueError, IndexError):
            return -1

    def tick(name, t0):
        t1 = time.perf_counter()
        log_t[name] += t1 - t0
        if PROF:
            prof_t[name] += t1 - t0
        return t1
    try:
        while True:
            if a.max_fps > 0:                                                  # 처리 속도 상한(발열 줄이기): 이전 루프 시작에서 1/max_fps 지날 때까지 쉼
                time.sleep(max(0.0, t_loop + 1.0 / a.max_fps - time.perf_counter()))
            t_loop = time.perf_counter()
            with lock:
                fcv.wait_for(lambda: latest["n"] != n_seen, timeout=0.03)    # 새 프레임을 30 ms까지 기다림(없으면 그대로 진행 — 키·지도는 계속)
                n_now, fs = latest["n"], latest["fs"]
                acc = imu["accel"].array() if len(imu["accel"]) else None
                gyr = imu["gyro"].array() if len(imu["gyro"]) else None
            use_cal = cal_live if (cal_live is not None and cal_live.get("valid")) else cal
            up = None
            if acc is not None and len(acc) > 20:
                w = acc[acc[:, 0] > acc[-1, 0] - 0.5, 1:4].mean(0)
                if use_cal is not None:
                    w = imu_calib.apply(w, use_cal)
                up_d = R_ad @ w                                                  # 깊이 좌표 위(보정 회전 전)
                mounted = use_prior and mount_calib.mount_matches(mcal, up_d)
                if mounted:                                                       # 7판: 장착 자세면 IMU 위를 바닥 법선 쪽으로 보정, 바닥 사전값 켬
                    up_d = np.asarray(mcal["R_imu_correction"], float) @ up_d
                    det.prior = plane_to_color(mcal["plane_normal_depth"], mcal["camera_height_m"])
                else:
                    det.prior = None
                up = R_dtoc @ up_d                                                # 컬러 좌표(검출기)
                det.up = up / np.linalg.norm(up)
            if hm is not None and gyr is not None and len(gyr) > 10 and up is not None:   # 자체 지도: 세로축 회전 누적·움직임
                ua = w / np.linalg.norm(w)
                new = gyr[gyr[:, 0] > t_gyr] if t_gyr is not None else gyr[-1:]
                rec = gyr[gyr[:, 0] > gyr[-1, 0] - 0.3, 1:4] - gbias
                moving = bool(np.linalg.norm(rec, axis=1).max() > math.radians(a.move_dps))
                if len(new) and t_gyr is not None:
                    dt = np.diff(np.r_[t_gyr, new[:, 0]])
                    hm.yaw += math.degrees(float(np.sum(((new[:, 1:4] - gbias) @ ua) * np.clip(dt, 0, 0.05))))
                    if not moving:
                        gbias = 0.98 * gbias + 0.02 * new[:, 1:4].mean(0)
                t_gyr = gyr[-1, 0]
            tp = time.perf_counter()
            if fs is not None and n_now != n_seen:
                n_seen = n_now
                al = align.process(fs)
                tp = tick("align", tp)
                d_f, c_f = al.get_depth_frame(), al.get_color_frame()
                if rec is not None:
                    rd, rc = fs.get_depth_frame(), fs.get_color_frame()
                    if rd and rc and rec.add(np.asanyarray(rd.get_data()), np.asanyarray(rc.get_data())):
                        print(f"기록 끝 — {rec.folder} (광민 도구: python -m lidar_dev.overlay {rec.folder} 등)", flush=True)
                        rec = None
                if d_f and c_f:
                    depth = np.asanyarray(d_f.get_data())                   # 원 해상도(클릭 점만 씀) — 복사하지 않음
                    color = np.asanyarray(c_f.get_data()).copy()
                    depth_s = np.ascontiguousarray(depth[::S_, ::S_])
                    res = det.process(depth_s)
                    tp = tick("detect", tp)
                    if res["status"] == "OK" and det.floor_mask is not None:
                        if mask_hist and mask_hist[-1][0].shape != det.floor_mask.shape:
                            mask_hist.clear()
                        mask_hist.append((det.floor_mask.copy(), det.obst_mask.copy()))
                    if calib_run is not None:                                     # 장착 캘리브레이션 수집(깊이 좌표로)
                        pd = types.SimpleNamespace(n=None, d=None, reject=det.reject, stats=dict(det.stats))
                        if det.n is not None and det.reject == "":
                            pd.n, pd.d = plane_to_depth(det.n, det.d)
                        g3 = gyr[gyr[:, 0] > gyr[-1, 0] - 0.1, 1:4] if gyr is not None and len(gyr) else None
                        calib_run.add(pd, (R_ad @ w) if up is not None else None, g3)        # IMU 위(가속도 보정 적용, 장착 보정 회전 전)
                        if calib_run.done(time.monotonic()):
                            resm, why_ = calib_run.solve(serial, os.path.basename(a.calib) if cal is not None else None)
                            calib_run = None
                            if resm is None or why_:
                                mcal_msg = "mount calib FAILED: " + "; ".join(why_)
                                print("장착 캘리브레이션 실패:", *why_, flush=True)
                                if resm is not None:
                                    print("  (참고 값)", json.dumps({k_: resm[k_] for k_ in ("pitch_down_deg", "roll_deg", "camera_height_m", "imu_vs_plane_deg",
                                                                                         "height_std_mm", "normal_jitter_deg")}, ensure_ascii=False), flush=True)
                            else:
                                paths = mount_calib.save(resm, a.out)
                                mcal = resm
                                mcal_msg = f"mount calib OK: pitch {resm['pitch_down_deg']:.2f} roll {resm['roll_deg']:+.2f} h {resm['camera_height_m']:.3f} m"
                                print("장착 캘리브레이션 저장:", *paths, json.dumps({k_: resm[k_] for k_ in ("pitch_down_deg", "roll_deg", "camera_height_m",
                                      "imu_vs_plane_deg", "floor_rms_mm", "height_std_mm", "normal_jitter_deg", "floor_far_m")}, ensure_ascii=False), flush=True)
                                if tm is not None:
                                    mount.update(pitch=resm["pitch_down_deg"], roll=resm["roll_deg"], warn="")
                                    tm.start(mount["pitch"], mount["roll"])
                                    print("광민 지도를 캘리브레이션 장착 각으로 새로 시작", flush=True)
                    valid_s, P = det.valid, det.P                                 # 검출기가 만든 점(같은 솎은 격자·내부 파라미터)
                    bl = [(x * S_, y * S_, w_ * S_, h_ * S_, hh, ff, ww, ll) for (x, y, w_, h_, hh, ff, ww, ll) in blobs(det, P, max(10, a.min_blob // (S_ * S_)))] \
                        if res["status"] == "OK" else []
                    tp = tick("blobs", tp)
                    if hm is not None and res["status"] == "OK" and det.fwd is not None:
                        hm.check_motion(det.n, det.d)
                        if not moving:
                            hm.update(P @ det.n + det.d, det.fwd, det.lat, valid_s, time.time(), step=max(1, 2 // S_))
                    n_fps += 1
            if time.time() - t_fps > 1.0:
                fps, n_fps, t_fps = n_fps / (time.time() - t_fps), 0, time.time()
            if color is None:
                kk = web.poll_key() if web is not None else (cv2.waitKey(30) & 0xFF)
                if web is not None:
                    time.sleep(0.03)
                if kk in (ord("q"), 27):
                    break
                continue
            # 1) 컬러 + 바닥·장애물 칠, 덩어리 상자
            tp = time.perf_counter()
            view = color.copy()
            ok = res is not None and res["status"] == "OK"
            if ok and det.floor_mask is not None:
                need = min(a.mask_vote // 2 + 1, len(mask_hist)) if mask_hist else 1   # 최근 N프레임 다수결(한 프레임 깜빡임은 안 칠함)
                fv = np.sum([m[0] for m in mask_hist], 0) >= need if mask_hist else det.floor_mask
                ov = np.sum([m[1] for m in mask_hist], 0) >= need if mask_hist else det.obst_mask
                lab = fv.view(np.uint8) | (ov.view(np.uint8) << 1)          # 1 = 바닥, 2·3 = 장애물(장애물이 위)
                if S_ > 1:
                    lab = cv2.resize(lab, (K.width, K.height), interpolation=cv2.INTER_NEAREST)
                for col_, m_ in ((GREEN, lab == 1), (RED, lab >= 2)):        # 단색과 섞은 화면을 마스크로만 복사(불리언 인덱싱 6 ms → 1.4 ms, 같은 결과)
                    m8 = m_.view(np.uint8)
                    if not cv2.countNonZero(m8):
                        continue
                    sol = solid.get(col_)
                    if sol is None or sol.shape != view.shape:
                        sol = solid[col_] = np.full(view.shape, col_, np.uint8)
                    cv2.copyTo(cv2.addWeighted(view, 0.55, sol, 0.45, 0), m8, view)
                draw_corridor(view, det, K, det.half_width, a.map_fwd)
                for (x, y, w_, h_, hh, ff, ww, ll) in (bl if show_blobs else []):
                    cv2.rectangle(view, (x, y), (x + w_, y + h_), YEL, 2)
                    put(view, f"h {hh * 100:.1f}cm  d {ff:.2f}m  w {ww * 100:.0f}cm", (x, max(44, y - 6)), YEL, 0.5)
            tv, _ = ob.view(time.monotonic())                                # 광민 오버레이: 윤곽(경로 안 빨강·밖 주황)·배지·주의 문구
            draw_overlay(view, tv, kt, show_gm)
            cv2.rectangle(view, (0, 0), (K.width, 28), (0, 110, 0) if ok else (0, 0, 170), -1)
            f2 = lambda q: "-" if q is None else f"{q:.2f}m"
            zs = res["zones"] if res else {"left": None, "center": None, "right": None}
            put(view, f"{res['status'] if res else 'WAIT'}  L {f2(zs['left'])}  C {f2(zs['center'])}  R {f2(zs['right'])}   {fps:.0f} fps",
                (8, 20), WHITE, 0.6, 2)
            pts3 = []
            for (cx, cy) in clicks:
                zc = float(depth[cy, cx]) * scale if depth is not None else 0.0
                if zc > 0:
                    p3 = np.array([xn[cy, cx] * zc, yn[cy, cx] * zc, zc])
                    pts3.append(p3)
                    hh = float(p3 @ det.n + det.d) if det.n is not None else None
                    cv2.drawMarker(view, (cx, cy), CYAN, cv2.MARKER_CROSS, 18, 2)
                    put(view, f"r {np.linalg.norm(p3):.3f}m" + (f"  h {hh * 100:.1f}cm" if hh is not None else ""), (cx + 8, cy - 8), CYAN, 0.5)
                else:
                    cv2.drawMarker(view, (cx, cy), GRAY, cv2.MARKER_TILTED_CROSS, 14, 2)
                    put(view, "no depth", (cx + 8, cy - 8), GRAY, 0.5)
            if len(pts3) == 2:
                cv2.line(view, clicks[0], clicks[1], CYAN, 1)
                mx, my = (clicks[0][0] + clicks[1][0]) // 2, (clicks[0][1] + clicks[1][1]) // 2
                put(view, f"{np.linalg.norm(pts3[0] - pts3[1]) * 100:.1f} cm", (mx + 6, my + 18), CYAN, 0.7, 2)
            tp = tick("paint", tp)
            # 2) 2.5D 지도 — 원천(광민·자체)과 상관없이 같은 표시. 지도가 바뀌거나(광민 상태 sequence·자체 프레임 수) 0.5 s 지나거나 모드·클릭이 바뀔 때만 다시 그림
            t_now = time.time()
            mono = time.monotonic()
            snap = tm.snapshot(mono) if tm is not None else None
            mstat = snap.status if snap is not None else None
            tp = tick("snap", tp)
            key = ((None if tm is not None else hm.frames), map_mode, map_click, int(mono * a.map_hz),   # 7판: 지도 그림은 초당 map_hz번(광민 상태마다 다시 그리면 타일이 쌓일수록 느려짐)
                   (cmap.mpp, cmap.follow, cmap.center) if cmap is not None and map_mode == "console" else None)
            if key != map_key:
                map_key = key
                codes = None
                if tm is not None:
                    g = tmap_local_grid(snap, mono, (tm.cfg.camera_x_m, tm.cfg.camera_y_m), a.map_fwd, a.map_lat, keep, T["C"]) if snap is not None else None
                    tp = tick("grid", tp)
                    if g is None:
                        nf_, nl_ = int(round(a.map_fwd / tm.cfg.cell_m)), int(round(2 * a.map_lat / tm.cfg.cell_m))
                        g = {"Z": np.full((nf_, nl_), np.nan), "known": np.zeros((nf_, nl_), bool), "weak": np.zeros((nf_, nl_), bool), "codes": np.full((nf_, nl_), 255, np.uint8),
                             "age_s": np.full((nf_, nl_), np.inf), "cell": tm.cfg.cell_m, "nf": nf_, "nl": nl_}
                    Z, known, age = g["Z"], g["known"], g["age_s"]
                    lv, dip = classify_heights(Z, known, rules)
                    known = tidy_low(Z, known, lv)                                 # 광민 콘솔 규칙: 낮은 외톨이 칸 숨김
                    lv[~known] = 0
                    codes = None if a.no_code_gate else g["codes"]
                    grid = g
                else:
                    Z, known, lv, dip = hm.levels(t_now)
                    age = None
                    grid = {"nf": hm.nf, "nl": hm.nl, "cell": hm.cell}
                lv, dip, n_filtered = filter_hazards(lv, dip, codes, T["C"] if T is not None else None, min_nb)
                C_ = cell_colors(Z, known, lv, dip, map_mode, grid["cell"])
                if age is not None:
                    old = known & (age > a.dim)
                    C_[old] = (C_[old] * 0.55).astype(np.uint8)            # 지금 안 보이는(오래된) 칸
                if tm is not None:
                    wk = grid["weak"] & ~known
                    C_[wk] = (0.45 * np.array([128, 128, 128]) + 0.55 * C_[wk]).astype(np.uint8)   # 관측 부족(광민: 반투명 회색)
                Mv = types.SimpleNamespace(nf=grid["nf"], nl=grid["nl"], fwd_max=a.map_fwd, lat_max=a.map_lat)
                if map_mode == "console" and cmap is not None:
                    mimg_c = cmap.render(snap, mono, mapW, MAP)                  # 광민 콘솔 지도 그대로(세계 고정·따라가기·끌기·확대)
                else:
                    mimg_c = top_image(Mv, C_, MAP, det.half_width, map_mode)
                if map_click is not None and map_mode != "console":
                    fq, lq = map_click
                    i_, j_ = int(fq / grid["cell"]), int((lq + a.map_lat) / grid["cell"])
                    if 0 <= i_ < grid["nf"] and 0 <= j_ < grid["nl"]:
                        px_ = (int((j_ + 0.5) * mapW / grid["nl"]), int(MAP - (i_ + 0.5) * MAP / grid["nf"]))
                        cv2.drawMarker(mimg_c, px_, CYAN, cv2.MARKER_CROSS, 14, 2)
                        s_ = f"fwd {fq:.2f} lat {lq:+.2f} " + ("unknown" if not known[i_, j_] else
                                                           f"h {Z[i_, j_] * 100:+.1f}cm {['-', 'pass', 'caution', 'block'][lv[i_, j_]]}{' dip' if dip[i_, j_] else ''}"
                                                           + (f" age {age[i_, j_]:.0f}s" if age is not None else ""))
                        put(mimg_c, s_, (min(px_[0] + 8, mapW - 280), max(px_[1] - 8, 50)), CYAN, 0.45)
                oimg = oblique_image(Z, C_, known, W=K.width, Hh=OBL, fwd_max=a.map_fwd, tilt=obl_tilt)
                if depth_s is not None:                                     # 깊이 색(0.3–3 m)도 같은 때만
                    dsm = cv2.resize(depth_s, (mapW, OBL), interpolation=cv2.INTER_NEAREST)
                    dz = np.clip((dsm.astype(np.float32) * scale - 0.3) / 2.7, 0, 1)
                    dsmall = cv2.applyColorMap((dz * 255).astype(np.uint8), cv2.COLORMAP_TURBO)
                    dsmall[dsm == 0] = 0
                    put(dsmall, "depth 0.3-3 m", (6, 16), WHITE, 0.45)
            tp = tick("map", tp)
            mimg = mimg_c
            oimg_d = oimg.copy()
            if mstat is not None and mstat.reason == "BIAS_PENDING":       # 광민 지도가 자이로 영점을 배우는 중(1 s 정지): 지도·비스듬히 판이 빈 이유를 바로 보이게
                shake = None                                                 # 지금 흔들림 = 최근 1 s 세로축 회전 속도 표준편차(광민 판정과 같은 양, 문턱 0.01 rad/s)
                if gyr is not None and up is not None:
                    g1 = gyr[gyr[:, 0] > gyr[-1, 0] - 1.0, 1:4]
                    if len(g1) > 20:
                        shake = math.degrees(float(np.std(g1 @ R["gd"].T @ (up_d / np.linalg.norm(up_d)))))
                t_ = "로버(카메라)를 1초 가만히 두세요 — 자이로 영점 배우는 중" + (f" · 흔들림 {shake:.1f}°/s (문턱 {math.degrees(tm.cfg.bias_max_std_rad_s):.1f})" if shake is not None else "")
                tw_, _ = kt.size(t_, 18)
                kt.draw(oimg_d, t_, (max(4, (K.width - tw_) // 2), OBL // 2 - 12), YEL, 18)
            if hm is not None and moving:
                put(oimg_d, "MOVING - map paused", (K.width - 230, 18), YEL, 0.55, 2)
            # 4) 아래 정보 줄
            hud = np.zeros((190, K.width + mapW, 3), np.uint8)
            L = [f"ground: h {res['camera_height_m'] if res else None} m  tilt {res['camera_tilt_deg'] if res else None} deg  fresh {res['ground_plane_ok'] if res else None}"
                 + (f"   [{det.reject}]" if det.reject else "") + f"   gravity check {'ON' if det.use_grav else 'OFF'}"]
            if up is not None:
                upd = R_ad @ w
                p, r = cam_angles(upd)
                L.append(f"IMU up(depth{', calibrated' if use_cal is not None else ''}) {np.round(upd / 9.80665, 3)} g  |a| {np.linalg.norm(upd):.3f}"
                         f"  lidar pitch {p:+.1f}  roll {r:+.1f} deg")
            if calib_run is not None:
                L.append(f"MOUNT CALIBRATION: hold still {max(0.0, calib_run.SECONDS - (time.monotonic() - calib_run.t0)):.1f} s  (frames {calib_run.n_frames}, floor {len(calib_run.planes)})")
            else:
                L.append(("mount: calibrated " + mcal["time"][5:16] + f"  pitch {mcal['pitch_down_deg']:.1f} roll {mcal['roll_deg']:+.1f} h {mcal['camera_height_m']:.2f} m"
                          if mcal else "mount: NOT calibrated - put rover on flat floor, press k")
                         + f"  | floor prior {'ON' if det.prior is not None else ('off (handheld?)' if (mcal and use_prior) else 'off')}"
                         + f"  | floor {det.stats.get('rms_mm', float('nan')):.0f} mm rms, far {det.stats.get('floor_far_m') or 0:.1f} m" + (f"  | {mcal_msg}" if mcal_msg else ""))
            nc = nearest_in_band(lv, grid["cell"], a.map_lat, det.half_width, 2)
            nb = nearest_in_band(lv, grid["cell"], a.map_lat, det.half_width, 3)
            if tm is not None:
                yaw_s = f"{math.degrees(mstat.pose.yaw_rad):+.0f}" if (mstat is not None and mstat.pose is not None) else "-"
                reason_kr = consts.get("REASON_TEXT", {}).get(mstat.reason, mstat.reason) if mstat and mstat.reason else ""
                L.append(f"광민 지도: {mstat.status if mstat else 'WAIT'}{' · ' + reason_kr if reason_kr else ''}  tiles {mstat.tile_count if mstat else 0}"
                         f"  proc {mstat.proc_ms if mstat and mstat.proc_ms is not None else '-'} ms  yaw {yaw_s}  cell {grid['cell']:.2f}  | band: caution {f2(nc)} block {f2(nb)}"
                         f"  | spikes filtered {n_filtered}  codes {tm.cfg.obstacle_height_m}/{tm.cfg.ditch_depth_m}  wheel gap max {tm.wheel_gap_max * 1000:.0f} ms"
                         f"  | mount {tm.cfg.camera_pitch_down_deg}/{tm.cfg.camera_roll_deg}" + (f" !! {mount['warn']}" if mount["warn"] else ""))
            else:
                L.append(f"map own {a.cell * 100:.0f}cm: known {known.mean() * 100:.0f}%  frames {hm.frames}  band: caution {f2(nc)} block {f2(nb)}"
                         f"  | reset #{hm.n_reset}: {hm.why}")
            if pending is not None and acc is not None:
                name, t0 = pending
                dur = a.calib_sec if name == "calib" else a.snap_sec
                sel = acc[acc[:, 0] >= t0]
                el = sel[-1, 0] - t0 if len(sel) else 0.0
                if el >= dur:
                    gsel = gyr[gyr[:, 0] >= t0] if gyr is not None else np.zeros((0, 4))
                    pending = None
                    if name == "calib":
                        info, why = imu_calib.pose_from_arrays(sel, gsel, f"live_{len(cposes) + 1}")
                        if why:
                            msg = "calib pose REJECTED (moving?) - see terminal, retry"
                            print("보정 자세 거부:", *why, flush=True)
                        else:
                            cposes.append(info)
                            am = np.array(info["accel_mean"])
                            print(f"[보정 자세 {len(cposes)}] 위 방향(IMU) {np.round(am / np.linalg.norm(am), 3)} · |a| {np.linalg.norm(am):.3f} · 덮은 방향 {cover_str(cposes)}", flush=True)
                            msg = f"calib pose {len(cposes)} ok"
                            if len(cposes) >= 6:
                                cal_live = imu_calib.calibrate_poses(cposes, [])
                                if "accel_bias_m_s2" in cal_live:
                                    print(f"  편향 {np.round(np.array(cal_live['accel_bias_m_s2']) / 9.80665 * 1000, 1)} mg · 배율 {np.round(cal_live['accel_scale'], 4)} · "
                                          f"보정 후 잔차 최대 {np.abs(cal_live['resid_after']).max() / 9.80665 * 1000:.1f} mg · 유효 {cal_live['valid']}", flush=True)
                                for why_ in cal_live["reasons"]:
                                    print("   -", why_, flush=True)
                                if cal_live["valid"]:
                                    cal_live["source"] = "live_overlay"
                                    cal_live["serial"] = serial
                                    fn = os.path.join(a.out, time.strftime("imu_calib_live_%Y%m%d_%H%M%S.json"))
                                    json.dump(cal_live, open(fn, "w"), ensure_ascii=False, indent=1)
                                    print("  유효한 보정 저장:", fn, flush=True)
                    else:
                        am = sel[:, 1:4].mean(0)
                        snaps[name] = {"accel_mean_raw": am.tolist(), "accel_std": sel[:, 1:4].std(0).tolist(), "n": int(len(sel)),
                                       "up_cam": (R_ad @ am).tolist(), "up_color": (R_ac @ am).tolist(), "frame": "up_cam = 깊이(라이다) 좌표, 보정 전",
                                       "gyro_mean": gsel[:, 1:4].mean(0).tolist() if len(gsel) else None}
                        print(f"[{name}] {len(sel)}표본 · 위 방향(깊이 좌표) {np.round((R_ad @ am) / np.linalg.norm(am), 4)} · |a| {np.linalg.norm(am):.3f} · "
                              f"표준편차 {np.round(sel[:, 1:4].std(0), 4)}", flush=True)
                        res_m = mount_result(snaps, R_ad)
                        if res_m:
                            out_ = {"serial": serial, "R_accel_to_depth": R_ad.tolist(), "R_accel_to_color": R_ac.tolist(), **snaps, "result": res_m}
                            if use_cal is not None:
                                out_["result_calibrated"] = mount_result(snaps, R_ad, use_cal)
                            fn = os.path.join(a.out, time.strftime("imu_mount_%Y%m%d_%H%M%S.json"))
                            json.dump(out_, open(fn, "w"), ensure_ascii=False, indent=1)
                            print("→", json.dumps({k_: (round(v_, 2) if isinstance(v_, float) else v_) for k_, v_ in res_m.items() if k_ != "note"},
                                                  ensure_ascii=False), "· 저장", fn, flush=True)
                else:
                    L.append(f"measuring [{name}] {el:.1f}/{dur:.0f} s - hold still")
            res_raw = mount_result(snaps, R_ad)
            if res_raw:
                L.append(fmt_mount(res_raw, ""))
            if use_cal is not None and res_raw:
                L.append(fmt_mount(mount_result(snaps, R_ad, use_cal), " (calibrated)"))
            if cposes or cal_live is not None:
                cs = f"calib: {len(cposes)} poses (need 9+)  cover {cover_str(cposes)}"
                if cal_live is not None and "accel_bias_m_s2" in cal_live:
                    cs += f"  -> {'VALID' if cal_live['valid'] else 'not valid (' + str(len(cal_live['reasons'])) + ' reasons, terminal)'}"
                L.append(cs + (f"   {msg}" if msg else ""))
            L.append(("CALLBACK ERROR " + errs["cb"] + "  | " if errs.get("cb") else "")
                     + f"calib {os.path.basename(a.calib) if cal is not None else 'none'}  L515 conf {sens['conf']} noise {sens['noise']}   keys k MOUNT CALIB p prior | w VIDEO | 1/2(3/4) c x | v view [ ] tilt f follow e erase m new map | o b r g s q")
            for i, s_ in enumerate(L[:2] + L[-6:] if len(L) > 8 else L):           # 바닥·IMU 두 줄은 늘 보이게
                if s_.isascii():
                    put(hud, s_, (8, 20 + 22 * i), WHITE, 0.48)
                else:                                                           # 한글 섞인 줄(광민 사유 문구 등)
                    kt.draw(hud, s_, (8, 6 + 22 * i), WHITE, 14)
            if tm is not None and mcal is None and mount["warn"] and gyr is not None and len(gyr) > 50 and mono - t_still_try > 2.0:
                g1 = gyr[gyr[:, 0] > gyr[-1, 0] - 1.0, 1:4]
                if len(g1) > 50 and np.linalg.norm(g1, axis=1).max() < 0.05:     # 1 s 동안 3 °/s 아래 = 놓아 둠
                    t_still_try = mono
                    if measure_mount():
                        tm.start(mount["pitch"], mount["roll"])
                        print(f"자동 재측정 — 장착 숙임 {mount['pitch']}°·옆 {mount['roll']}°로 광민 지도 새로", flush=True)
            if vid is not None:                                                # 녹화 중 표시(빨간 점·경과 시간)
                el = time.monotonic() - vid["t0"]
                cv2.circle(view, (K.width - 120, 14), 7, (40, 40, 230), -1)
                put(view, f"REC {int(el // 60):02d}:{int(el % 60):02d}", (K.width - 108, 20), (40, 40, 230), 0.55, 2)
            frame = np.vstack([np.hstack([view, mimg]), np.hstack([oimg_d, dsmall]), hud])
            if vid is not None:                                                # 실시간 속도: 경과 시간만큼 출력 프레임을 채움(느리면 같은 화면 반복) — 쓰기는 스레드에서
                want = int((time.monotonic() - vid["t0"]) * a.video_fps) + 1
                if vid["w"] is None:
                    vid["w"] = VideoSink(vid["path"], a.video_fps, (frame.shape[1], frame.shape[0]))
                if want > vid["n"]:
                    vid["w"].put(frame, want - vid["n"])
                    vid["n"] = want
            tp = tick("hud", tp)
            if web is not None:
                web.publish(frame)
            else:
                cv2.imshow(win, frame)
            tp = tick("imshow", tp)
            prof_n[0] += 1
            if PROF and prof_n[0] % 60 == 0:
                print("[profile ms/frame]", {k_: round(1000 * v_ / 60, 1) for k_, v_ in prof_t.items() if not k_.startswith("_")}, f"루프 {prof_n[0]} 새 프레임 {n_seen}", flush=True)
                prof_t.clear()
            log_n[0] += 1
            if time.time() - t_log > 2.0:
                dt_log = time.time() - t_log if t_log else 2.0
                t_log = time.time()
                with lock:
                    cam_now = latest["n"]
                cam_fps, cam_n0[0] = (cam_now - cam_n0[0]) / max(dt_log, 1e-3), cam_now
                slow = sorted(((k_, 1000 * v_ / max(log_n[0], 1)) for k_, v_ in log_t.items()), key=lambda x: -x[1])[:4]
                rx_ms, rx_n = (tm.rx_stat() if tm is not None else (0.0, 0))
                print(f"[{time.strftime('%H:%M:%S')}] 카메라 {cam_fps:.1f} fps · 루프 {log_n[0] / max(dt_log, 1e-3):.1f}/s · ms/루프 "
                      + " ".join(f"{k_} {v_:.0f}" for k_, v_ in slow)
                      + f" · 지도 수신 {rx_ms / max(dt_log, 1e-3) / 10:.0f}%({rx_n}개) 타일 {len(snap.tiles) if snap is not None else 0}"
                      + f" · 코어 {main_cpu()}", flush=True)
                log_t.clear()
                log_n[0] = 0
                print(f"[{time.strftime('%H:%M:%S')}] {fps:.1f} fps {res['status'] if res else '-'} 구역 {zs} 덩어리 [" +
                      ", ".join(f"h{b[4] * 100:.1f}cm d{b[5]:.2f}m w{b[6] * 100:.0f}cm" for b in bl[:4]) + f"] | 지도 알려진 칸 {known.mean() * 100:.0f}% "
                      f"주의 {f2(nc)} 불가 {f2(nb)} | " + (L[2] if len(L) > 2 else "") + (f" | {tm.summary}" if tm is not None else ""), flush=True)
            if os.environ.get("OV_AUTOVIDEO") and vid is None and not prof_t.get("_vid_auto") and time.time() - t_start > 2.0:   # 시험: 2 s 뒤 녹화 자동 시작
                prof_t["_vid_auto"] = 1.0
                vid = {"w": None, "path": os.path.join(a.out, time.strftime("overlay_video_%Y%m%d_%H%M%S_raw.mp4")), "t0": time.monotonic(), "n": 0}
            if os.environ.get("OV_AUTOCAL") and calib_run is None and not mcal_msg and time.time() - t_start > 3.0:   # 시험: 3 s 뒤 캘리브레이션 자동 시작
                calib_run = mount_calib.MountCalibrator(time.monotonic())
                mcal_msg = "mount calib (auto test)"
            if a.seconds and time.time() - t_start > a.seconds:
                fn = os.path.join(a.out, time.strftime("overlay_auto_%Y%m%d_%H%M%S.png"))
                cv2.imwrite(fn, frame)
                print("시험 시간 끝 — 마지막 화면 저장:", fn, flush=True)
                break
            tp = time.perf_counter()
            if web is not None:                                              # 브라우저 키·클릭
                k = web.poll_key()
                ck = web.poll_click()
                if ck is not None:
                    on_mouse(cv2.EVENT_LBUTTONDOWN, ck[0], ck[1], 0, None)
                time.sleep(0.005)
            else:
                k = cv2.waitKey(1) & 0xFF
            tick("waitkey", tp)
            if k in (ord("q"), 27):
                break
            if k in (ord("1"), ord("2"), ord("3"), ord("4"), ord("c")) and acc is not None and pending is None:
                name = {ord("1"): "mount_down", ord("2"): "lens_down", ord("3"): "mount_down_r180", ord("4"): "lens_down_r180", ord("c"): "calib"}[k]
                pending = (name, acc[-1, 0] + 0.5)
                msg = ""
                print(f"[{name}] {a.calib_sec if name == 'calib' else a.snap_sec:.0f} s 측정 — 가만히", flush=True)
            if k == ord("x") and cposes:
                cposes.pop()
                cal_live = imu_calib.calibrate_poses(cposes, []) if len(cposes) >= 6 else None
                msg = f"removed last calib pose ({len(cposes)} left)"
                print(f"마지막 보정 자세 지움 — {len(cposes)}개 남음", flush=True)
            if k == ord("v"):                                                  # 지도 보기: 단계 → 높이 → 광민 콘솔
                modes = ["class", "height"] + (["console"] if cmap is not None else [])
                map_mode = modes[(modes.index(map_mode) + 1) % len(modes)]
            if k in (ord("["), ord("]")):                                    # 2.5D 비스듬히 보기 시점 낮게/높게
                obl_tilt = float(np.clip(round(obl_tilt + (0.1 if k == ord("]") else -0.1), 2), 0.0, 1.0))
                map_key = None
            if k == ord("f") and cmap is not None:
                cmap.follow = not cmap.follow
            if k == ord("e") and tm is not None:
                tm.rx.clear_display()                                          # 광민 '화면에서 지우기'(칸·지나온 길)
                print("지도 화면 지움(광민 런타임 기록은 그대로)", flush=True)
            if k == ord("w"):                                                  # 화면 녹화 켜기/끄기
                if vid is None:
                    vid = {"w": None, "path": os.path.join(a.out, time.strftime("overlay_video_%Y%m%d_%H%M%S_raw.mp4")), "t0": time.monotonic(), "n": 0}
                    print("화면 녹화 시작 →", vid["path"], flush=True)
                else:
                    video_stop()
            if k == ord("o"):
                show_gm = not show_gm
            if k == ord("b"):
                show_blobs = not show_blobs
            if k == ord("r") and rec is None:
                rec = Recorder(a.out, rec_meta)
                print(f"기록 시작(40쌍, 0.1 s 간격) → {rec.folder}", flush=True)
            if k == ord("m"):
                if tm is not None:
                    measure_mount()
                    tm.start(mount["pitch"], mount["roll"])
                    print(f"광민 지도 새로 — 장착 숙임 {mount['pitch']}°·옆 {mount['roll']}°", flush=True)
                else:
                    hm.reset("manual")
            if k == ord("k") and calib_run is None:                            # 장착 캘리브레이션(평지에 세운 로버, 3 s 가만히)
                calib_run = mount_calib.MountCalibrator(time.monotonic())
                mcal_msg = "mount calib: hold still 3 s on flat floor"
                print("장착 캘리브레이션 시작 — 평평한 바닥, 3 s 가만히", flush=True)
            if k == ord("p"):
                use_prior = not use_prior
            if k == ord("g"):
                det.use_grav = not det.use_grav
            if k == ord("s"):
                stamp = time.strftime("%Y%m%d_%H%M%S")
                fn = os.path.join(a.out, f"overlay_{stamp}.png")
                cv2.imwrite(fn, frame)
                np.savez_compressed(os.path.join(a.out, f"heightmap_{stamp}.npz"), Z=Z, known=known, level=lv, dip=dip, cell=grid["cell"],
                                    fwd_max=a.map_fwd, lat_max=a.map_lat, source=a.map, criteria=str(a.criteria),
                                    profile=str(crit["criteria_profile"] if crit else ""), age_s=age if age is not None else np.zeros(0))
                print("저장:", fn, f"+ heightmap_{stamp}.npz", json.dumps(res, ensure_ascii=False), [tuple(round(x, 3) if isinstance(x, float) else x for x in b) for b in bl], flush=True)
    finally:
        video_stop()
        if tm is not None:
            tm.close()
        if ob is not None:
            ob.close()
        gm.close()
        _OPEN.remove(pipe)
        pipe.stop()
        if web is not None:
            web.close()
        else:
            cv2.destroyAllWindows()
        if snaps:
            fn = os.path.join(a.out, time.strftime("imu_snaps_%Y%m%d_%H%M%S.json"))
            json.dump({"serial": serial, "R_accel_to_depth": R_ad.tolist(), "R_accel_to_color": R_ac.tolist(), **snaps}, open(fn, "w"), ensure_ascii=False, indent=1)
            print("IMU 측정 저장:", fn)
        if cposes:
            fn = os.path.join(a.out, time.strftime("imu_calib_poses_%Y%m%d_%H%M%S.json"))
            json.dump({"serial": serial, "poses": cposes, "result": cal_live}, open(fn, "w"), ensure_ascii=False, indent=1)
            print("보정 자세 저장:", fn)


_OPEN = []

if __name__ == "__main__":
    try:
        main()
    except BaseException:
        for _p in _OPEN:
            _p.stop()
        raise
