# record_seq.py — 노트북에 직접 꽂은 L515로 깊이·IR·컬러·IMU를 기록한다(l515-seq v1 폴더 형식).
# 사용법: python record_seq.py --out ~/data --label wall_1m --seconds 5 --note "벽 정면 1.00 m"
#        Enter 또는 Ctrl+C로 멈춘다. 결과 폴더는 ground.py·terrain.py·check_seq.py가 그대로 읽는다.
# ⚠️ 로봇에서는 쓰지 않는다 — 로봇의 L515는 Gateway만 소유한다(AGENTS.md §7). 로봇 데이터는 rosbag으로.
# 설계: isaacsim/2026_10_17/docs/specs/2026-10-17-l515-recorder-and-mapping-method.md (2판)
import argparse, fcntl, glob, importlib.metadata, os, select, stat, sys, threading, time
from datetime import datetime
import numpy as np
import pyrealsense2 as rs

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from seq_format import (FORMAT, FORMAT_VERSION, SeqWriter, extrinsics_dict, write_json_atomic)

TOOL_VERSION = "record_seq 1.0"
GATEWAY_LOCK = "/run/powertrain/l515-gateway.lock"
WARMUP = 30  # 자동 노출 안정화용으로 버리는 frameset 수


def gateway_guard(maintenance):
    """Gateway 단일 소유권 보호. 반환: (진행 가능 여부, 이유, 정비 모드 잠금 fd 또는 None)."""
    running = []
    for p in glob.glob("/proc/[0-9]*/cmdline"):
        try:
            cmd = open(p, "rb").read().replace(b"\0", b" ").decode(errors="ignore")
        except OSError:
            continue
        if "gateway_main" in cmd and "l515_dashboard" in cmd:
            running.append(p.split("/")[2])
    if running:
        return False, f"L515 Gateway 실행 중(pid {', '.join(running)}) — 카메라를 열지 않는다", None
    jetson = os.path.exists("/etc/nv_tegra_release")
    if not maintenance:
        if os.path.lexists(GATEWAY_LOCK):
            return False, f"{GATEWAY_LOCK} 있음 — 로봇 환경으로 보고 멈춘다(로봇은 rosbag으로 기록)", None
        if jetson:
            return False, "젯슨에서는 Gateway만 L515를 소유한다 — 정비 목적이면 Gateway를 멈춘 뒤 --maintenance", None
        return True, "노트북 직접 연결", None
    # 정비 모드: 사람이 Gateway·감독 재시작을 멈춘 상태가 전제. 잠금 파일은 생성·절단 없이 열어 기록 끝까지 쥔다.
    try:
        fd = os.open(GATEWAY_LOCK, os.O_RDWR | os.O_NOFOLLOW)
    except FileNotFoundError:
        if jetson:
            return False, f"{GATEWAY_LOCK} 없음 — 런타임 디렉터리 준비 상태를 먼저 확인할 것(직접 만들지 않는다)", None
        return True, "정비 모드(잠금 파일 없음, 젯슨 아님)", None
    except OSError as e:
        return False, f"잠금 파일 열기 실패: {e}", None
    st = os.fstat(fd)
    if not stat.S_ISREG(st.st_mode) or st.st_uid not in (0, os.getuid()):
        os.close(fd)
        return False, "잠금 파일이 일반 파일이 아니거나 소유자가 다름", None
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        return False, "Gateway 잠금이 잡혀 있음 — Gateway를 먼저 멈출 것", None
    return True, "정비 모드(Gateway 잠금 획득, 종료까지 유지)", fd


def intr_dict(prof):
    i = prof.as_video_stream_profile().get_intrinsics()
    return {"width": i.width, "height": i.height, "fx": i.fx, "fy": i.fy, "ppx": i.ppx, "ppy": i.ppy,
            "model": str(i.model).split(".")[-1], "coeffs": [float(c) for c in i.coeffs]}


def prof_dict(prof):
    d = {"stream": prof.stream_name(), "format": str(prof.format()).split(".")[-1], "fps": prof.fps()}
    if prof.is_video_stream_profile():
        v = prof.as_video_stream_profile()
        d.update(width=v.width(), height=v.height())
    return d


def options_snapshot(sensor):
    out = {}
    for o in sensor.get_supported_options():
        try:
            out[str(o).split(".")[-1]] = float(sensor.get_option(o))
        except Exception:
            pass
    return out


def parse_res(s):
    w, h = s.lower().split("x")
    return int(w), int(h)


class Capture:
    """pipeline 콜백 하나로 영상 frameset과 IMU를 받는다(레포 Gateway와 같은 방식)."""

    def __init__(self, writer, every, want):
        self.writer, self.every, self.want = writer, every, want
        self.lock = threading.Lock()
        self.capture_idx = 0
        self.skipped_warmup = self.skipped_every = self.incomplete = 0
        self.last_fn, self.fn_gaps = {}, {k: 0 for k in want}
        self.latest_depth = None
        self.imu_n = {"accel": 0, "gyro": 0}
        self.imu_first = None
        self.error = None
        self.stopping = False

    def on_frame(self, f):
        try:
            host_mono, host_wall = time.monotonic(), time.time()
            if f.is_frameset():
                frames = list(f.as_frameset())
            else:
                frames = [f]
            video = {}
            for fr in frames:
                st = fr.get_profile().stream_type()
                if st in (rs.stream.accel, rs.stream.gyro):
                    self._imu(fr, st, host_mono)
                elif st == rs.stream.depth:
                    video["depth"] = fr
                elif st == rs.stream.infrared:
                    video["ir"] = fr
                elif st == rs.stream.color:
                    video["color"] = fr
            if video and not self.stopping:
                self._video(video, host_mono, host_wall)
        except Exception as e:  # 콜백 예외는 SDK가 삼키므로 직접 남긴다
            self.error = self.error or f"콜백 예외: {e!r}"

    def _imu(self, fr, st, host_mono):
        m = fr.as_motion_frame().get_motion_data()
        kind = "accel" if st == rs.stream.accel else "gyro"
        self.writer.add_imu({"host_mono_s": f"{host_mono:.6f}", "kind": kind, "ts_ms": f"{fr.get_timestamp():.3f}",
                             "ts_domain": str(fr.get_frame_timestamp_domain()).split(".")[-1],
                             "fn": fr.get_frame_number(), "x": m.x, "y": m.y, "z": m.z})
        with self.lock:
            self.imu_n[kind] += 1
            if self.imu_first is None:
                self.imu_first = datetime.now().isoformat(timespec="milliseconds")

    def _video(self, video, host_mono, host_wall):
        if set(video) != set(self.want):  # 일부 스트림이 빠진 frameset은 저장하지 않고 센다
            with self.lock:
                self.incomplete += 1
            return
        row = {"host_mono_s": f"{host_mono:.6f}", "host_wall": datetime.fromtimestamp(host_wall).isoformat(timespec="milliseconds")}
        for k, fr in video.items():
            fn = fr.get_frame_number()
            row[f"{k}_ts_ms"] = f"{fr.get_timestamp():.3f}"
            row[f"{k}_ts_domain"] = str(fr.get_frame_timestamp_domain()).split(".")[-1]
            row[f"{k}_fn"] = fn
            prev = self.last_fn.get(k)
            if prev is not None and fn > prev + 1:
                self.fn_gaps[k] += fn - prev - 1
            self.last_fn[k] = fn
        with self.lock:
            ci = self.capture_idx
            self.capture_idx += 1
        if ci < WARMUP:
            self.skipped_warmup += 1
            self.writer.log_capture(ci, "warmup", row)
            return
        if (ci - WARMUP) % self.every:
            self.skipped_every += 1
            self.writer.log_capture(ci, "every", row)
            return
        images = {k: np.ascontiguousarray(np.asanyarray(fr.get_data())).copy() for k, fr in video.items()}
        self.latest_depth = images["depth"]
        self.writer.submit(ci, images, row)


def main():
    ap = argparse.ArgumentParser(description="L515 기록(l515-seq v1)")
    ap.add_argument("--out", default="data", help="기록 폴더를 만들 상위 디렉터리(기본: ./data)")
    ap.add_argument("--label", default="")
    ap.add_argument("--note", default="", help="줄자 거리·장면 설명 등")
    ap.add_argument("--seconds", type=float, default=0, help="저장 시작 후 이 시간이 지나면 멈춤(0 = 수동)")
    ap.add_argument("--frames", type=int, default=0, help="저장 성공 수가 이만큼이면 멈춤(0 = 수동)")
    ap.add_argument("--every", type=int, default=2, help="N개 frameset마다 하나 저장(기본 2 = 15 Hz)")
    ap.add_argument("--depth-res", default="640x480", help="640x480(기존 스크립트 호환) | 1024x768 | 320x240")
    ap.add_argument("--color-res", default="1280x720")
    ap.add_argument("--no-color", action="store_true")
    ap.add_argument("--no-ir", action="store_true")
    ap.add_argument("--no-imu", action="store_true")
    ap.add_argument("--serial", default="", help="장치 시리얼(여러 대일 때 필수)")
    ap.add_argument("--queue", type=int, default=60)
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--maintenance", action="store_true", help="젯슨 정비 모드(Gateway를 사람이 먼저 멈춘 경우만)")
    a = ap.parse_args()

    ok, why, lock_fd = gateway_guard(a.maintenance)
    print("guard:", why)
    if not ok:
        sys.exit(2)
    devs = rs.context().query_devices()
    serials = [d.get_info(rs.camera_info.serial_number) for d in devs]
    if not serials:
        print("L515를 찾지 못함")
        sys.exit(3)
    serial = a.serial or (serials[0] if len(serials) == 1 else "")
    if serial not in serials:
        print(f"장치를 고를 수 없음(--serial 필요): {serials}")
        sys.exit(3)
    dev = devs[serials.index(serial)]

    dw, dh = parse_res(a.depth_res)
    cw, ch = parse_res(a.color_res)
    want = ["depth"] + ([] if a.no_ir else ["ir"]) + ([] if a.no_color else ["color"])
    if (dw, dh) != (640, 480):
        print(f"⚠️ 깊이 {dw}x{dh}: terrain.py·overlay.py·scan_terrain.py는 640x480만 가정한다(호환 안 됨)")

    def make_cfg(imu):
        cfg = rs.config()
        cfg.enable_device(serial)
        cfg.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, 30)
        if "ir" in want:
            cfg.enable_stream(rs.stream.infrared, 0, dw, dh, rs.format.y8, 30)
        if "color" in want:
            cfg.enable_stream(rs.stream.color, cw, ch, rs.format.bgr8, 30)
        if imu:
            cfg.enable_stream(rs.stream.accel)
            cfg.enable_stream(rs.stream.gyro)
        return cfg

    name = datetime.now().strftime("%Y%m%d_%H%M%S") + (f"_{a.label}" if a.label else "")
    folder = os.path.join(os.path.expanduser(a.out), name)
    os.makedirs(folder, exist_ok=False)
    writer = SeqWriter(folder, want, workers=a.workers, queue_size=a.queue)
    cap = Capture(writer, a.every, want)
    pipe = rs.pipeline()
    imu_reason = "요청 안 함(--no-imu)" if a.no_imu else ""
    try:
        prof = pipe.start(make_cfg(not a.no_imu), cap.on_frame)
    except Exception as e:
        if a.no_imu:
            raise
        imu_reason = f"IMU 포함 시작 실패 → 영상만: {e}"
        print("⚠️", imu_reason)
        pipe = rs.pipeline()
        prof = pipe.start(make_cfg(False), cap.on_frame)

    # 메타(시작 시점) — 장치·협상 프로필·보정값
    d_prof = prof.get_stream(rs.stream.depth)
    streams, ext = {}, {}
    for key, st, idx in (("depth", rs.stream.depth, -1), ("infrared", rs.stream.infrared, 0), ("color", rs.stream.color, -1),
                         ("accel", rs.stream.accel, -1), ("gyro", rs.stream.gyro, -1)):
        try:
            p = prof.get_stream(st, idx) if idx >= 0 else prof.get_stream(st)
        except Exception:
            streams[key] = None
            continue
        streams[key] = prof_dict(p)
        if key != "depth":
            e = d_prof.get_extrinsics_to(p)
            ext[f"depth_to_{key}"] = extrinsics_dict(e.rotation, e.translation)
    streams["ir"] = streams.pop("infrared")
    depth_sensor = dev.first_depth_sensor()
    meta = {
        "format": FORMAT, "format_version": FORMAT_VERSION, "tool_version": TOOL_VERSION,
        "created_wall": datetime.now().isoformat(timespec="seconds"), "label": a.label, "note": a.note,
        "device": {k: dev.get_info(getattr(rs.camera_info, k)) for k in
                   ("name", "serial_number", "firmware_version", "usb_type_descriptor", "product_line")
                   if dev.supports(getattr(rs.camera_info, k))},
        "sdk_version": importlib.metadata.version("pyrealsense2"),
        "guard": why, "every": a.every, "warmup_framesets": WARMUP,
        "streams": streams,
        "depth_scale_m": depth_sensor.get_depth_scale(),
        "depth_intrinsics": intr_dict(d_prof),
        "color_intrinsics": intr_dict(prof.get_stream(rs.stream.color)) if streams.get("color") else None,
        "infrared_intrinsics": intr_dict(prof.get_stream(rs.stream.infrared, 0)) if streams.get("ir") else None,
        "extrinsics": ext,
        "frames_convention": "optical: x 오른쪽, y 아래, z 앞, m. extrinsics: p_to = R·p_from + t (R_row_major 행 우선; R_sdk_col_major는 SDK 원본 열 우선)",
        "compat": {"terrain_family": (dw, dh) == (640, 480),
                   "reason": "terrain.py·overlay.py·scan_terrain.py는 (480, 640) 가정" if (dw, dh) != (640, 480) else ""},
        "sensor_options": {s.get_info(rs.camera_info.name): options_snapshot(s) for s in dev.query_sensors()},
        "imu": {"requested": not a.no_imu, "available": False, "first_sample_wall": None, "reason": imu_reason,
                "units": {"accel": "m/s^2", "gyro": "rad/s"}},
        "stats": None,
    }
    write_json_atomic(os.path.join(folder, "meta.json"), meta)
    print(f"기록: {folder}")
    print(f"스트림: {', '.join(k for k, v in streams.items() if v)} / 저장 {30 / a.every:.0f} Hz — Enter 또는 Ctrl+C로 멈춤")

    t_start = time.monotonic()
    t_first_save = None
    reason = "사용자 중지"
    try:
        while True:
            time.sleep(0.5)
            saved = writer.saved
            if saved and t_first_save is None:
                t_first_save = time.monotonic()
            if cap.error or writer.error:
                reason = cap.error or writer.error
                break
            if a.frames and saved >= a.frames:
                reason = f"--frames {a.frames} 도달"
                break
            if a.seconds and t_first_save and time.monotonic() - t_first_save >= a.seconds:
                reason = f"--seconds {a.seconds} 도달"
                break
            el = time.monotonic() - (t_first_save or t_start)
            dz = cap.latest_depth
            vr = cz = float("nan")
            if dz is not None:
                vr = float((dz > 0).mean())
                h, w = dz.shape
                c = dz[h // 2 - 20:h // 2 + 20, w // 2 - 20:w // 2 + 20]
                c = c[c > 0]
                cz = float(np.median(c)) * meta["depth_scale_m"] if c.size else float("nan")
            sys.stdout.write(f"\r{el:6.1f}s 저장 {saved:5d} ({saved / el if el > 0 else 0:4.1f} Hz) 큐 {writer.q.qsize():3d} "
                             f"버림 큐 {writer.dropped_queue} SDK {sum(cap.fn_gaps.values())} IMU {sum(cap.imu_n.values()):6d} "
                             f"유효 {vr:4.2f} 가운데 {cz:5.2f} m   ")
            sys.stdout.flush()
            if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                break
    except KeyboardInterrupt:
        pass
    print(f"\n멈춤: {reason}")
    cap.stopping = True
    pipe.stop()
    t_end = time.monotonic()
    st = writer.close()
    meta["imu"]["available"] = sum(cap.imu_n.values()) > 0
    meta["imu"]["first_sample_wall"] = cap.imu_first
    meta["stats"] = {
        "stop_reason": reason,
        "framesets_received": cap.capture_idx, "skipped_warmup": cap.skipped_warmup, "skipped_every": cap.skipped_every,
        "incomplete_framesets": cap.incomplete,
        "dropped_app_queue": st["dropped_app_queue"], "sdk_frame_number_gaps": cap.fn_gaps,
        "saved": st["saved"], "write_errors": st["write_errors"], "error": st["error"] or cap.error,
        "imu_samples": st["imu_samples"],
        "duration_s": round(t_end - t_start, 3), "monotonic_start": t_start, "monotonic_end": t_end,
    }
    write_json_atomic(os.path.join(folder, "meta.json"), meta)
    if lock_fd is not None:
        os.close(lock_fd)
    print(f"저장 {st['saved']}개, 버림(큐) {st['dropped_app_queue']}, SDK 번호 틈 {cap.fn_gaps}, IMU {st['imu_samples']}")
    print(f"점검: python check_seq.py {folder}")
    sys.exit(1 if (st["error"] or cap.error) else 0)


if __name__ == "__main__":
    main()
