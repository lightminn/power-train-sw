# gm_features.py — 광민 브랜치(feat/l515-terrain-map) 사용자 기능을 노트북 창(live_overlay 6판)에 옮긴 모음(10-22 저녁, 사용자: '광민 레포 기능은 모두 구현')
#   광민 코드는 tmap 작업트리에서 import만 한다 — 카메라 오버레이 런타임(TerrainOverlayRuntime → 패킷 → 표시 결정 terrain_view), 콘솔 지도 렌더(map_rgba).
#   GTK가 필요한 콘솔 화면 부분(오버레이 윤곽·배지 그리기, 지도 상태줄·기울기 줄·범례·고정 문구·축척 막대·1 m 격자·지나온 길·로버 화살표·따라가기·끌기·확대·지우기)은
#   광민과 같은 문구·색·수치로 cv2에 다시 그린다. 문구·범례 상수는 광민 파일에서 ast로 읽는다(손으로 옮기지 않음 — 광민이 바꾸면 따라감).
#   한글은 cv2 글꼴로 못 그려 PIL(맑은 고딕 → Noto CJK → 은돋움)로 작은 조각을 그려 섞는다(같은 문구는 조각 캐시).
#   제외: 표시 지연(SRT 영상 맞추기 — 노트북 직결이라 필요 없음), ROS bag 기록·재생(rosbag2 필요)
import ast, math, os, threading, time

import cv2
import numpy as np
try:                                             # Pillow 없으면(Jetson ROS 컨테이너 등) 한글 대신 cv2 영문 글꼴로(한글 글자는 ?)
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    Image = ImageDraw = ImageFont = None

FONT_CANDIDATES = [os.path.expanduser("~/.local/share/fonts/MalgunGothic/malgun.ttf"),
                   "/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc",
                   "/usr/share/fonts/truetype/unfonts-core/UnDotum.ttf"]
TONE_BGR = {"ok": (255, 213, 0), "warn": (0, 184, 255), "error": (82, 82, 255), "muted": (209, 199, 191)}   # 광민 (0,.835,1)·(1,.72,0)·(1,.32,.32)·(.75,.78,.82)
SCALE_1280 = 1280.0 / 960.0          # 광민 오버레이는 컬러 1280×720 고정 — 우리 960×540 내부 파라미터를 4/3배 해서 주고 윤곽을 0.75배로 그린다


class KText:
    """한글 문구를 PIL로 그려 cv2 영상에 섞는다. (문구, 크기, 색)별 조각 캐시."""

    def __init__(self):
        self.path = next((p for p in FONT_CANDIDATES if os.path.exists(p)), None) if Image is not None else None
        self.fonts, self.cache = {}, {}

    def font(self, size):
        if size not in self.fonts:
            self.fonts[size] = ImageFont.truetype(self.path, size) if self.path else ImageFont.load_default()
        return self.fonts[size]

    def patch(self, text, size, color_bgr):
        key = (text, size, color_bgr)
        p = self.cache.get(key)
        if p is None and Image is None:                  # Pillow 없음: cv2 영문 글꼴(한글은 ?)
            t = "".join(ch if ord(ch) < 128 else "?" for ch in text)
            sc = size / 30.0
            (w, h), b = cv2.getTextSize(t, cv2.FONT_HERSHEY_SIMPLEX, sc, 1)
            im = np.zeros((h + b + 4, w + 4, 3), np.uint8)
            al = np.zeros((h + b + 4, w + 4), np.uint8)
            cv2.putText(im, t, (2, h + 2), cv2.FONT_HERSHEY_SIMPLEX, sc, color_bgr, 1, cv2.LINE_AA)
            cv2.putText(al, t, (2, h + 2), cv2.FONT_HERSHEY_SIMPLEX, sc, 255, 1, cv2.LINE_AA)
            p = (im.astype(np.float32), (al.astype(np.float32) / 255.0)[..., None])
            self.cache[key] = p
        if p is None:
            f = self.font(size)
            x0, y0, x1, y1 = f.getbbox(text)
            w, h = max(1, x1 - min(0, x0) + 2), max(1, y1 - min(0, y0) + 2)
            im = Image.new("RGBA", (w, h), (0, 0, 0, 0))
            ImageDraw.Draw(im).text((-min(0, x0) + 1, -min(0, y0) + 1), text, font=f, fill=(color_bgr[2], color_bgr[1], color_bgr[0], 255))
            a = np.asarray(im)
            p = (a[..., [2, 1, 0]].astype(np.float32), a[..., 3:4].astype(np.float32) / 255.0)
            if len(self.cache) > 400:
                self.cache.clear()
            self.cache[key] = p
        return p

    def size(self, text, size):
        rgb, _ = self.patch(text, size, (255, 255, 255))
        return rgb.shape[1], rgb.shape[0]

    def draw(self, img, text, org, color_bgr=(255, 255, 255), size=16, shadow=True):
        """org = 왼쪽 위. 반환 = 글자 폭."""
        if not text:
            return 0
        rgb, al = self.patch(text, size, color_bgr)
        if shadow:
            self._blend(img, np.zeros_like(rgb), al * 0.8, (org[0] + 1, org[1] + 1))
        self._blend(img, rgb, al, org)
        return rgb.shape[1]

    @staticmethod
    def _blend(img, rgb, al, org):
        x, y = int(org[0]), int(org[1])
        h, w = rgb.shape[:2]
        x0, y0, x1, y1 = max(0, x), max(0, y), min(img.shape[1], x + w), min(img.shape[0], y + h)
        if x1 <= x0 or y1 <= y0:
            return
        r, a = rgb[y0 - y:y1 - y, x0 - x:x1 - x], al[y0 - y:y1 - y, x0 - x:x1 - x]
        roi = img[y0:y1, x0:x1].astype(np.float32)
        img[y0:y1, x0:x1] = (roi * (1 - a) + r * a).astype(np.uint8)


def box(img, x0, y0, x1, y1, bgr=(0, 0, 0), alpha=0.6):
    x0, y0, x1, y1 = max(0, int(x0)), max(0, int(y0)), min(img.shape[1], int(x1)), min(img.shape[0], int(y1))
    if x1 > x0 and y1 > y0:
        roi = img[y0:y1, x0:x1]
        img[y0:y1, x0:x1] = cv2.addWeighted(roi, 1 - alpha, np.full_like(roi, bgr), alpha, 0)


def load_constants(root):
    """광민 콘솔 파일(GTK import 때문에 직접 import 못 함)에서 문구·범례 상수를 ast로."""
    out = {}
    for rel, names in (("operator_console/terrain_map_panel.py", ("REASON_TEXT", "LEGEND", "FIXED_NOTE", "BACKGROUND", "LINK_LOST_S", "TILT_MAX_AGE_S")),
                       ("operator_console/terrain.py", ("REFERENCE_NOTE",))):
        tree = ast.parse(open(os.path.join(root, rel), encoding="utf-8").read())
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name) and node.targets[0].id in names:
                try:
                    out[node.targets[0].id] = ast.literal_eval(node.value)
                except ValueError:
                    pass
    out.setdefault("LINK_LOST_S", 0.5)
    out.setdefault("TILT_MAX_AGE_S", 1.0)
    return out


# --- A. 카메라 오버레이(광민 TerrainOverlayRuntime) -------------------------------------------------------
class OverlayBridge:
    """광민 지형 오버레이 송신 런타임 다리 — 부모 쪽(7판 끝: 런타임은 gm_worker 자식 프로세스에서 publish_hz로). 깊이·CameraInfo는 자식에, 받은 패킷 → TerrainFrame.
    표시 결정은 콘솔과 같은 terrain_view(배지·색조·주의 문구·장애물 윤곽, 1280×720 기준 좌표)."""

    def __init__(self, depth_cam, color_cam_960, gm, half_width_m=0.4445, publish_hz=5.0):
        from terrain_overlay.runtime import RuntimeConfig
        from terrain_overlay.contract import parse_packet
        from operator_console.terrain import terrain_view
        self.parse, self.view_fn = parse_packet, terrain_view
        c = color_cam_960
        self.cfg = RuntimeConfig(path_half_width_m=half_width_m, publish_hz=publish_hz)   # 5 Hz = 광민 허용 범위(5–10) 아래 끝(자식에서 같은 값으로 만든다)
        cam = lambda m: dict(fx=m.fx, fy=m.fy, cx=m.cx, cy=m.cy, width=m.width, height=m.height)
        cam1280 = dict(fx=c.fx * SCALE_1280, fy=c.fy * SCALE_1280, cx=c.cx * SCALE_1280, cy=c.cy * SCALE_1280, width=1280, height=720)
        self.latest, self.last_rx, self.n = None, None, 0
        self.lock = threading.Lock()
        gm.handlers["ov"] = self._on_packet
        gm.send("ov_init", cam(depth_cam), cam1280, half_width_m, publish_hz)

    def _on_packet(self, raw, now):
        try:
            fr = self.parse(raw, received_monotonic_s=now)
        except ValueError as e:
            print("[overlay] 패킷 오류", e, flush=True)
            return
        with self.lock:
            self.latest, self.last_rx = fr, now
            self.n += 1

    def view(self, now):
        with self.lock:
            fr, rx = self.latest, self.last_rx
        return self.view_fn(fr, fr, rx, now, 0.0, (1280, 720), video_live=True), fr     # 표시 지연 0 → 표시 프레임 = 최신(None이면 윤곽을 안 냄)

    def close(self):
        pass


def draw_overlay(img, tv, kt, show_outlines=True):
    """광민 TerrainCanvas._on_draw와 같은 모양: 경로 안 = 빨강(채움 α .22·선 3 px), 밖 = 주황 선 2 px, 꼭대기 22 px 위 'd.dd m'. 왼쪽 아래 배지·문구 상자."""
    s = img.shape[1] / 1280.0
    if show_outlines:
        for ob in tv.obstacles:
            pts = (np.array(ob.poly, np.float32) * s).astype(np.int32)
            if len(pts) < 3:
                continue
            if ob.in_path:
                x0, y0 = np.maximum(pts.min(0), 0)
                x1, y1 = np.minimum(pts.max(0) + 1, [img.shape[1], img.shape[0]])
                if x1 > x0 and y1 > y0:                                      # 윤곽 둘레 영역만 합성(화면 전체 합성 3 ms → 작게)
                    roi = img[y0:y1, x0:x1]
                    fill = roi.copy()
                    cv2.fillPoly(fill, [pts - [x0, y0]], (64, 64, 255))
                    img[y0:y1, x0:x1] = cv2.addWeighted(roi, 0.78, fill, 0.22, 0)
                cv2.polylines(img, [pts], True, (64, 64, 255), 3, cv2.LINE_AA)
            else:
                cv2.polylines(img, [pts], True, (0, 153, 255), 2, cv2.LINE_AA)
            top = pts[np.argmin(pts[:, 1])]
            kt.draw(img, f"{ob.dist_m:.2f} m", (int(top[0]) - 20, max(0, int(top[1]) - 22 - 6)), (64, 64, 255) if ob.in_path else (0, 153, 255), 17)
    bw = max(kt.size(tv.badge, 18)[0], kt.size(tv.note, 13)[0]) + 20
    y0 = img.shape[0] - 58
    box(img, 8, y0, 8 + bw, img.shape[0] - 8, (24, 15, 0), 0.82)              # 광민 rgba(0, .059, .094, .82)
    kt.draw(img, tv.badge, (18, y0 + 6), TONE_BGR.get(tv.tone, (255, 255, 255)), 18, shadow=False)
    kt.draw(img, tv.note, (18, y0 + 30), TONE_BGR["muted"], 13, shadow=False)


# --- B. 콘솔 지도(광민 map_rgba + 화면 요소) -------------------------------------------------------------
class ConsoleMap:
    """광민 '지형 지도' 페이지를 한 판에: 높이 음영·분류 색·30 s 흐림·외톨이 숨김(map_rgba 그대로) + 1 m 격자·지나온 길(노랑)·로버 화살표(흰색, 0.5 s 넘게 끊기면
    회색 '마지막 자세')·축척 막대(1 m, 판 1/3 넘으면 0.5 m)·상태줄·기울기 줄·'로버를 1초 정지하세요'·'수신 끊김'·범례·고정 문구.
    조작: 따라가기(기본, 진행 방향 위) — 끌면 옮기고 따라가기 꺼짐, 휠 ×1.2 확대(0.002–0.08 m/px), 지우기(화면에서 칸·길)."""

    def __init__(self, T, consts, kt):
        self.T, self.k, self.kt = T, consts, kt
        self.disp = T["DisplayConfig"]()
        self.mpp, self.follow, self.center, self.cache = 0.01, True, (0.0, 0.0), {}
        self.drag, self.theta = None, 0.0

    def zoom(self, up):
        self.mpp = float(np.clip(self.mpp / 1.2 if up else self.mpp * 1.2, 0.002, 0.08))

    def drag_to(self, dx_px, dy_px):
        """화면 끌기(픽셀) → 세계 이동(돌린 각 되돌림), 따라가기 끔."""
        self.follow = False
        c, s = math.cos(math.radians(self.theta)), math.sin(math.radians(self.theta))
        ex, ey = c * dx_px - s * dy_px, s * dx_px + c * dy_px        # 화면 → 북쪽 위 래스터 픽셀
        self.center = (self.center[0] - ex * self.mpp, self.center[1] + ey * self.mpp)

    def render(self, snap, now, w, h, link_rx=None):
        T, k, kt = self.T, self.k, self.kt
        bg = tuple(int(v * 255) for v in k.get("BACKGROUND", (0.07, 0.09, 0.12)))[::-1]
        out = np.full((h, w, 3), bg, np.uint8)
        st = snap.status if snap is not None else None
        pose = st.pose if st is not None else None
        if self.follow and pose is not None:
            self.center = (pose.x_m, pose.y_m)
            self.theta = 90.0 - math.degrees(pose.yaw_rad)                     # 진행 방향이 위로(광민 φ = yaw − π/2)
        elif self.follow:
            self.theta = 0.0
        R = int(math.hypot(w, h)) + 4
        cx, cy, mpp = self.center[0], self.center[1], self.mpp
        view = T["MapView"](x0=cx - R * mpp / 2, y0=cy - R * mpp / 2, width_m=R * mpp, height_m=R * mpp, width_px=R, height_px=R)
        canvas = np.full((R, R, 3), bg, np.uint8)
        if snap is not None and st is not None:
            rgba, meta = T["map_rgba"](snap, view, self.disp, now, self.cache)
            if meta.get("drawn") and rgba.shape[0] > 1:
                sc = meta["m_per_px"] / mpp
                img = cv2.resize(rgba, (max(1, int(round(rgba.shape[1] * sc))), max(1, int(round(rgba.shape[0] * sc)))), interpolation=cv2.INTER_LINEAR)
                c0 = int(round((meta["origin_x_m"] - view.x0) / mpp))
                r0 = int(round((view.y0 + view.height_m - meta["top_y_m"]) / mpp))
                h2, w2 = img.shape[:2]
                rr0, cc0, rr1, cc1 = max(0, r0), max(0, c0), min(R, r0 + h2), min(R, c0 + w2)
                if rr1 > rr0 and cc1 > cc0:
                    sub = img[rr0 - r0:rr1 - r0, cc0 - c0:cc1 - c0]
                    al = sub[..., 3:4].astype(np.float32) / 255.0
                    roi = canvas[rr0:rr1, cc0:cc1].astype(np.float32)
                    canvas[rr0:rr1, cc0:cc1] = (roi * (1 - al) + sub[..., [2, 1, 0]].astype(np.float32) * al).astype(np.uint8)

        def w2p(x, y):                                                         # 세계 → 캔버스(북쪽 위) 픽셀
            return (np.asarray(x) - view.x0) / mpp, (view.y0 + view.height_m - np.asarray(y)) / mpp
        grid = canvas.copy()                                                   # 1 m 격자(흰색 α .10)
        for gx in np.arange(math.floor(view.x0), view.x0 + view.width_m + 1):
            px, _ = w2p(gx, 0)
            cv2.line(grid, (int(px), 0), (int(px), R), (255, 255, 255), 1)
        for gy in np.arange(math.floor(view.y0), view.y0 + view.height_m + 1):
            _, py = w2p(0, gy)
            cv2.line(grid, (0, int(py)), (R, int(py)), (255, 255, 255), 1)
        canvas = cv2.addWeighted(canvas, 0.9, grid, 0.1, 0)
        M = cv2.getRotationMatrix2D((R / 2, R / 2), self.theta, 1.0)
        rot = cv2.warpAffine(canvas, M, (R, R), flags=cv2.INTER_LINEAR, borderValue=bg)
        x0c, y0c = (R - w) // 2, (R - h) // 2
        out[:] = rot[y0c:y0c + h, x0c:x0c + w]

        def to_screen(x, y):
            px, py = w2p(x, y)
            p = M @ np.vstack([np.atleast_1d(px), np.atleast_1d(py), np.ones(np.size(px))])
            return p[0] - x0c, p[1] - y0c
        if snap is not None and len(snap.trajectory) > 1:                     # 지나온 길(노랑 2 px)
            tr = np.asarray(snap.trajectory, float)
            sx, sy = to_screen(tr[:, 0], tr[:, 1])
            pts = np.c_[sx, sy].astype(np.int32)
            cv2.polylines(out, [pts], False, (51, 217, 255), 2, cv2.LINE_AA)
        stale_pose = st is None or st.received_s is None or now - st.received_s > k["LINK_LOST_S"]
        if pose is not None:                                                    # 로버 화살표 (16,0),(−10,−9),(−10,9)
            ca, sa = math.cos(pose.yaw_rad), math.sin(pose.yaw_rad)
            tri = np.array([[16, 0], [-10, -9], [-10, 9]], float) * mpp       # 차체 좌표(앞 x·왼쪽 y) 픽셀 → m
            wx = pose.x_m + tri[:, 0] * ca - tri[:, 1] * sa
            wy = pose.y_m + tri[:, 0] * sa + tri[:, 1] * ca
            sx, sy = to_screen(wx, wy)
            col = (150, 150, 150) if stale_pose else (255, 255, 255)
            cv2.fillPoly(out, [np.c_[sx, sy].astype(np.int32)], col, cv2.LINE_AA)
            if stale_pose:
                kt.draw(out, "마지막 자세", (int(sx[0]) + 8, int(sy[0]) - 8), col, 13)
        # 축척 막대: 1 m(판 1/3 넘으면 0.5 m)
        L = 1.0 if 1.0 / mpp <= w / 3 else 0.5
        lp = int(L / mpp)
        cv2.line(out, (12, h - 70), (12 + lp, h - 70), (255, 255, 255), 2)
        for xx in (12, 12 + lp):
            cv2.line(out, (xx, h - 75), (xx, h - 65), (255, 255, 255), 2)
        kt.draw(out, f"{L:g} m", (16 + lp, h - 80), (255, 255, 255), 13)
        # 상태줄(광민 형식)
        if st is None:
            line = "지형 지도 수신 대기 · 노트북 런타임"
        else:
            reason = k["REASON_TEXT"].get(st.reason, st.reason or "") if "REASON_TEXT" in k else (st.reason or "")
            age = "-" if st.received_s is None else f"{now - st.received_s:.1f}s"
            line = (f"{st.status}{(' · ' + reason) if reason else ''} · epoch {st.map_epoch} · 거리 {st.distance_m:.1f} m · 자세 끊김 {st.pose_gaps}"
                    f" · 타일 {len(snap.tiles)}/{st.tile_count} · 수신 {age} 전 · 보정 {st.corrector.name}{' 적용' if st.corrector.applied else ''}"
                    f" · 경계 +{st.thresholds.obstacle_height_m * 100:.0f}/-{st.thresholds.ditch_depth_m * 100:.0f} cm")
        box(out, 0, 0, w, 26, (0, 0, 0), 0.65)
        kt.draw(out, line[:140], (8, 4), (250, 245, 242), 14, shadow=False)
        yy = 28
        if st is not None and st.plane_tilt is not None and st.received_s is not None and \
                st.plane_tilt.age_ms / 1000.0 + (now - st.received_s) <= k["TILT_MAX_AGE_S"]:
            lines = [f"기준면 기울기(차체 기준) 앞 {st.plane_tilt.pitch_deg:+.1f}° 옆 {st.plane_tilt.roll_deg:+.1f}° · 기준면 = 판정기가 고른 가까운 바닥"]
            g = st.gravity
            if g is not None:
                lines.append(f"정지 구간 측정 · 기준면 경사(중력 기준) 앞 {g.slope_pitch_deg:+.1f}° 옆 {g.slope_roll_deg:+.1f}° (전체 {g.slope_total_deg:.1f}°)"
                             f" · 차체 앞 들림 {g.body_pitch_deg:+.1f}° 왼쪽 들림 {g.body_roll_deg:+.1f}°")
            box(out, 0, yy, w, yy + 20 * len(lines) + 4, (0, 0, 0), 0.55)
            for i, s_ in enumerate(lines):
                kt.draw(out, s_[:120], (8, yy + 2 + 20 * i), (255, 230, 204), 12, shadow=False)
            yy += 20 * len(lines) + 6
        if st is not None and st.reason == "BIAS_PENDING":
            t_ = "로버를 1초 정지하세요"
            tw, _ = kt.size(t_, 26)
            kt.draw(out, t_, ((w - tw) // 2, yy + 20), (51, 204, 255), 26)
        if snap is not None and ((snap.last_rx_s is not None and now - snap.last_rx_s > k["LINK_LOST_S"]) or
                                 (st is not None and st.received_s is not None and now - st.received_s > k["LINK_LOST_S"])):
            t_ = "수신 끊김"
            tw, th = kt.size(t_, 36)
            box(out, (w - tw) // 2 - 16, h // 2 - th, (w + tw) // 2 + 16, h // 2 + 16, (0, 0, 0), 0.6)
            kt.draw(out, t_, ((w - tw) // 2, h // 2 - th + 8), (77, 89, 255), 36, shadow=False)
        # 범례·고정 문구(아래)
        box(out, 0, h - 56, w, h, (0, 0, 0), 0.6)
        x = 8
        for name, rgb in k.get("LEGEND", ()):
            col = tuple(int(v * 255) for v in rgb)[::-1]
            cv2.rectangle(out, (x, h - 50), (x + 12, h - 38), col, -1)
            x += 16 + kt.draw(out, name, (x + 16, h - 53), (240, 240, 240), 12, shadow=False) + 10
            if x > w - 60:
                break
        kt.draw(out, k.get("FIXED_NOTE", "")[:90], (8, h - 30), (200, 200, 200), 11, shadow=False)
        kt.draw(out, ("따라가기 켬" if self.follow else "따라가기 끔(f)") + f" · {mpp * 100:.1f} cm/px · 끌기 = 옮기기, 휠 = 확대, e = 지우기",
                (8, h - 15), (170, 170, 170), 11, shadow=False)
        return out


# --- C. 기록(광민 lidar_dev.record 형식 — 광민 오프라인 도구가 그대로 읽음) ---------------------------------
def intr_dict(i):
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy, "model": str(i.model), "coeffs": list(i.coeffs)}


class Recorder:
    """키 r: 원본 깊이(640×480 PNG)·컬러(JPG) 40쌍, 3장마다 1장(0.1 s) — meta.json(depth_scale_m·frame_period_s·depth/color_intrinsics·depth_to_color)."""
    N, SKIP = 40, 3

    def __init__(self, out_root, meta):
        self.folder = os.path.join(out_root, time.strftime("gm_rec_%Y%m%d_%H%M%S"))
        os.makedirs(os.path.join(self.folder, "depth"), exist_ok=True)
        os.makedirs(os.path.join(self.folder, "color"), exist_ok=True)
        import json
        json.dump({**meta, "frame_period_s": self.SKIP / 30.0, "tool": "live_overlay 6판 키 r(광민 lidar_dev.record 형식, 컬러 960×540)"},
                  open(os.path.join(self.folder, "meta.json"), "w"), ensure_ascii=False, indent=2)
        self.saved, self.k = 0, 0

    def add(self, depth_u16, color_bgr):
        """새 프레임마다 부름 → 끝나면 True."""
        self.k += 1
        if (self.k - 1) % self.SKIP:
            return False
        cv2.imwrite(os.path.join(self.folder, "depth", f"{self.saved:06d}.png"), depth_u16)
        cv2.imwrite(os.path.join(self.folder, "color", f"{self.saved:06d}.jpg"), color_bgr)
        self.saved += 1
        return self.saved >= self.N
