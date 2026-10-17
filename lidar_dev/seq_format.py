# seq_format.py — L515 기록 폴더 형식(l515-seq v1) 쓰기·점검. record_seq.py·check_seq.py·시험이 같이 쓴다.
# 형식: 폴더/meta.json + depth/%06d.png(uint16 원본) — ground.py·terrain.py 등 기존 스크립트가 그대로 읽는다(640×480).
# 설계: isaacsim/2026_10_17/docs/specs/2026-10-17-l515-recorder-and-mapping-method.md (2판)
import csv, glob, json, os, queue, threading
import numpy as np
import cv2

FORMAT = "l515-seq"
FORMAT_VERSION = 1
L515_DEPTH_SCALE = 0.00025  # 0.25 mm
# 스트림 키 → (폴더, 확장자, 기대 dtype, 채널 수)
STREAMS = {
    "depth": ("depth", ".png", np.uint16, 1),
    "ir": ("infrared", ".png", np.uint8, 1),
    "color": ("color", ".jpg", np.uint8, 3),
}
STREAM_TS = ("depth", "ir", "color")
FRAME_COLS = ["save_idx", "capture_idx", "host_mono_s", "host_wall"] + [
    f"{s}_{k}" for s in STREAM_TS for k in ("ts_ms", "ts_domain", "fn")]
CAPTURE_COLS = ["capture_idx", "status", "host_mono_s"] + [f"{s}_fn" for s in STREAM_TS]
IMU_COLS = ["host_mono_s", "kind", "ts_ms", "ts_domain", "fn", "x", "y", "z"]


def extrinsics_dict(rot_sdk, trans):
    """rs2_extrinsics(rotation = 열 우선 9개, translation m) → p_to = R·p_from + t 를 행 우선으로 직렬화."""
    rot_sdk = [float(v) for v in rot_sdk]
    R = np.asarray(rot_sdk).reshape(3, 3, order="F")
    return {"R_row_major": R.reshape(-1).tolist(), "t_m": [float(v) for v in trans], "R_sdk_col_major": rot_sdk}


def extrinsics_matrix(d):
    """extrinsics_dict → 4×4 (p_to = T @ [p_from, 1])."""
    T = np.eye(4)
    T[:3, :3] = np.asarray(d["R_row_major"], float).reshape(3, 3)
    T[:3, 3] = d["t_m"]
    return T


def write_json_atomic(path, obj):
    tmp = path + ".tmp"
    with open(tmp, "w", encoding="utf-8") as f:
        json.dump(obj, f, ensure_ascii=False, indent=1)
    os.replace(tmp, path)


class SeqWriter:
    """저장 큐 + 저장 스레드. submit()은 막지 않는다(큐가 차면 그 frameset을 버리고 센다).
    save_idx는 큐에 넣을 때 연속 발급, frames.csv는 close()에서 저장 성공분만 save_idx 순으로 쓴다.
    쓰기 실패는 error에 남기고 이후 submit()을 거부한다(호출 측이 기록을 멈춘다)."""

    def __init__(self, folder, streams, workers=4, queue_size=60, jpeg_quality=95, png_level=1):
        self.folder, self.streams = folder, list(streams)
        for s in self.streams:
            os.makedirs(os.path.join(folder, STREAMS[s][0]), exist_ok=True)
        self.params = {".png": [cv2.IMWRITE_PNG_COMPRESSION, png_level], ".jpg": [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality]}
        self.q = queue.Queue(maxsize=queue_size)
        self.lock = threading.Lock()
        self.done, self.imu, self.error = [], [], None
        self.next_save, self.dropped_queue = 0, 0
        # 줄 단위 버퍼 — 기록 중 죽어도 받은 frameset 목록이 남는다
        self._cap_f = open(os.path.join(folder, "capture_log.csv"), "w", newline="", buffering=1)
        self._cap = csv.DictWriter(self._cap_f, CAPTURE_COLS, extrasaction="ignore")
        self._cap.writeheader()
        self.threads = [threading.Thread(target=self._work, daemon=True) for _ in range(workers)]
        for t in self.threads:
            t.start()

    def log_capture(self, capture_idx, status, row):
        with self.lock:
            self._cap.writerow({"capture_idx": capture_idx, "status": status, **row})

    def submit(self, capture_idx, images, row):
        """images: {스트림: 독립 numpy 배열}. 반환: 큐에 넣었으면 True."""
        with self.lock:
            if self.error:
                return False
            idx = self.next_save
            try:
                self.q.put_nowait((idx, images, {"save_idx": idx, "capture_idx": capture_idx, **row}))
            except queue.Full:
                self.dropped_queue += 1
                self._cap.writerow({"capture_idx": capture_idx, "status": "drop_queue", **row})
                return False
            self.next_save += 1
            self._cap.writerow({"capture_idx": capture_idx, "status": "queued", **row})
            return True

    def add_imu(self, row):
        with self.lock:
            self.imu.append(row)

    def _work(self):
        while True:
            item = self.q.get()
            if item is None:
                self.q.task_done()
                return
            idx, images, row = item
            try:
                for s, arr in images.items():
                    sub, ext = STREAMS[s][0], STREAMS[s][1]
                    path = os.path.join(self.folder, sub, f"{idx:06d}{ext}")
                    tmp = os.path.join(self.folder, sub, f".{idx:06d}.tmp{ext}")
                    if not cv2.imwrite(tmp, arr, self.params[ext]):
                        raise IOError(f"cv2.imwrite 실패: {tmp}")
                    os.replace(tmp, path)
                with self.lock:
                    self.done.append((idx, row))
            except Exception as e:
                with self.lock:
                    self.error = self.error or f"저장 실패(save_idx {idx}): {e}"
            finally:
                self.q.task_done()

    @property
    def saved(self):
        with self.lock:
            return len(self.done)

    def close(self):
        """큐를 비우고 스레드를 끝낸 뒤 frames.csv·imu.csv를 쓴다. 반환: 통계 dict."""
        self.q.join()
        for _ in self.threads:
            self.q.put(None)
        for t in self.threads:
            t.join()
        self._cap_f.close()
        done = sorted(self.done, key=lambda x: x[0])
        with open(os.path.join(self.folder, "frames.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, FRAME_COLS, extrasaction="ignore")
            w.writeheader()
            for _, row in done:
                w.writerow(row)
        imu = sorted(self.imu, key=lambda r: r["host_mono_s"])
        if imu:
            with open(os.path.join(self.folder, "imu.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, IMU_COLS, extrasaction="ignore")
                w.writeheader()
                w.writerows(imu)
        n_acc = sum(r["kind"] == "accel" for r in imu)
        return {"saved": len(done), "queued": self.next_save, "dropped_app_queue": self.dropped_queue,
                "write_errors": 1 if self.error else 0, "error": self.error,
                "imu_samples": {"accel": n_acc, "gyro": len(imu) - n_acc}}


# ------------------------------------------------------------------ 점검

def _read_csv(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def _monotonic_issues(vals):
    """(역행 수, 중복 수)."""
    d = np.diff(np.asarray(vals, float))
    return int((d < 0).sum()), int((d == 0).sum())


def check_sequence(folder, sample=5):
    """기록 폴더 점검. 반환: (errors, warnings, summary)."""
    E, W, S = [], [], {}
    mp = os.path.join(folder, "meta.json")
    if not os.path.exists(mp):
        return [f"meta.json 없음: {folder}"], W, S
    meta = json.load(open(mp, encoding="utf-8"))
    if meta.get("format") != FORMAT or meta.get("format_version") != FORMAT_VERSION:
        E.append(f"형식 불일치: {meta.get('format')} v{meta.get('format_version')}")
    di = meta.get("depth_intrinsics") or {}
    for k in ("width", "height", "fx", "fy", "ppx", "ppy"):
        if k not in di:
            E.append(f"depth_intrinsics.{k} 없음")
    scale = meta.get("depth_scale_m")
    if not isinstance(scale, (int, float)) or not 0 < scale <= 0.01:
        E.append(f"depth_scale_m 이상: {scale}")
    elif abs(scale - L515_DEPTH_SCALE) > 1e-9:
        W.append(f"depth_scale_m {scale} ≠ L515 0.00025")
    if "stats" not in meta or meta["stats"] is None:
        W.append("stats 없음 — 기록이 정상 종료되지 않았을 수 있음")
    if E:
        return E, W, S
    streams = [s for s in STREAMS if (meta.get("streams") or {}).get(s)]
    if "depth" not in streams:
        E.append("depth 스트림이 꺼져 있음")
        return E, W, S
    fp = os.path.join(folder, "frames.csv")
    if not os.path.exists(fp):
        return E + ["frames.csv 없음"], W, S
    rows = _read_csv(fp)
    n = len(rows)
    S["frames"] = n
    idx = [int(r["save_idx"]) for r in rows]
    if idx != list(range(n)):
        E.append("frames.csv save_idx가 0부터 연속이 아님")
    # 파일 집합
    shape = {"depth": (di["height"], di["width"]), "ir": (di["height"], di["width"])}
    ci = meta.get("color_intrinsics")
    if ci:
        shape["color"] = (ci["height"], ci["width"], 3)
    for s in streams:
        sub, ext, dt, ch = STREAMS[s]
        files = sorted(os.path.basename(p) for p in glob.glob(os.path.join(folder, sub, "*" + ext)) if not os.path.basename(p).startswith("."))
        if files != [f"{i:06d}{ext}" for i in range(n)]:
            E.append(f"{sub}/ 파일 {len(files)}개가 frames.csv {n}행과 맞지 않음")
        tmp = glob.glob(os.path.join(folder, sub, ".*.tmp*"))
        if tmp:
            E.append(f"{sub}/ 임시 파일 {len(tmp)}개 남음(쓰다 만 프레임)")
        for i in sorted(set(np.linspace(0, max(n - 1, 0), min(sample, n)).astype(int))) if n else []:
            p = os.path.join(folder, sub, f"{i:06d}{ext}")
            if not os.path.exists(p):
                continue
            a = cv2.imread(p, cv2.IMREAD_UNCHANGED)
            if a is None:
                E.append(f"읽기 실패: {sub}/{i:06d}{ext}")
                continue
            if a.dtype != dt:
                E.append(f"{sub}/{i:06d}{ext} dtype {a.dtype} ≠ {np.dtype(dt)}")
            if s in shape and a.shape != tuple(shape[s]):
                E.append(f"{sub}/{i:06d}{ext} 크기 {a.shape} ≠ meta {tuple(shape[s])}")
    # 시각·번호
    for s in streams:
        ts = [float(r[f"{s}_ts_ms"]) for r in rows if r.get(f"{s}_ts_ms")]
        fn = [int(r[f"{s}_fn"]) for r in rows if r.get(f"{s}_fn")]
        back, dup = _monotonic_issues(ts) if len(ts) > 1 else (0, 0)
        if back or dup:
            E.append(f"{s} 타임스탬프 역행 {back}·중복 {dup}")
        fb, fd = _monotonic_issues(fn) if len(fn) > 1 else (0, 0)
        if fb or fd:
            E.append(f"{s} 프레임 번호 역행 {fb}·중복 {fd}")
        if len(ts) > 1:
            dts = np.diff(ts)
            S[f"{s}_dt_ms"] = {"median": round(float(np.median(dts)), 2), "max": round(float(dts.max()), 2)}
        doms = {r.get(f"{s}_ts_domain") for r in rows}
        if len(doms) > 1:
            W.append(f"{s} 타임스탬프 도메인이 섞임: {sorted(doms)}")
    # 같은 frameset 안 시간차(같은 도메인일 때만)
    for a, b in (("depth", "ir"), ("depth", "color")):
        if a in streams and b in streams and rows and rows[0].get(f"{a}_ts_domain") == rows[0].get(f"{b}_ts_domain"):
            d = [abs(float(r[f"{a}_ts_ms"]) - float(r[f"{b}_ts_ms"])) for r in rows]
            S[f"{a}_{b}_skew_ms"] = {"median": round(float(np.median(d)), 2), "max": round(float(np.max(d)), 2)}
    # IMU
    ip = os.path.join(folder, "imu.csv")
    if os.path.exists(ip):
        imu = _read_csv(ip)
        for kind in ("accel", "gyro"):
            ts = [float(r["ts_ms"]) for r in imu if r["kind"] == kind]
            if len(ts) > 1:
                back, dup = _monotonic_issues(ts)
                if back or dup:
                    E.append(f"IMU {kind} 타임스탬프 역행 {back}·중복 {dup}")
                S[f"imu_{kind}_hz"] = round((len(ts) - 1) / ((ts[-1] - ts[0]) / 1000.0), 1) if ts[-1] > ts[0] else None
    elif (meta.get("imu") or {}).get("available"):
        E.append("meta는 IMU 있음인데 imu.csv 없음")
    # 깊이 표본
    vr, cz = [], []
    for i in sorted(set(np.linspace(0, max(n - 1, 0), min(sample, n)).astype(int))) if n else []:
        a = cv2.imread(os.path.join(folder, "depth", f"{i:06d}.png"), cv2.IMREAD_UNCHANGED)
        if a is None or a.dtype != np.uint16:
            continue
        vr.append(float((a > 0).mean()))
        h, w = a.shape
        c = a[h // 2 - 20:h // 2 + 20, w // 2 - 20:w // 2 + 20]
        c = c[c > 0]
        if c.size:
            cz.append(float(np.median(c)) * scale)
    if vr:
        S["depth_valid_ratio"] = round(float(np.mean(vr)), 3)
    if cz:
        S["center_depth_m"] = round(float(np.median(cz)), 3)
    S["stats"] = meta.get("stats")
    return E, W, S
