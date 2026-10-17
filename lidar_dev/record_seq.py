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
    parent = os.stat(os.path.dirname(GATEWAY_LOCK))
    # l515_dashboard/resource_guard.py와 같은 조건: 부모 디렉터리 group/other 쓰기·실행 금지(0o027),
    # 잠금 파일은 실행 사용자 소유의 일반 파일이고 group/other 쓰기 금지(0o022)
    if parent.st_mode & 0o027 or not stat.S_ISREG(st.st_mode) or st.st_uid != os.geteuid() or st.st_mode & 0o022:
        os.close(fd)
        return False, "잠금 파일·디렉터리 권한이 Gateway 기준과 다름(소유자·권한 확인)", None
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
        self.last_video_mono = None

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
        self.last_video_mono = host_mono
        row = {"host_mono_s": f"{host_mono:.6f}", "host_wall": datetime.fromtimestamp(host_wall).isoformat(timespec="milliseconds")}
        for k, fr in video.items():  # 스트림별 번호 추적은 완전성 검사보다 먼저(받은 스트림의 틈을 잘못 세지 않게)
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
        missing = [k for k in self.want if k not in video]
        if missing:  # 일부 스트림이 빠진 frameset은 저장하지 않고, 빠진 스트림과 함께 남긴다
            self.incomplete += 1
            self.writer.log_capture(ci, "incomplete:" + "+".join(missing), row)
            return
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
    ap.add_argument("--start-timeout", type=float, default=10.0, help="이 시간 안에 한 프레임도 저장 못 하면 오류로 멈춤(s)")
    ap.add_argument("--stall-timeout", type=float, default=3.0, help="영상이 이 시간 넘게 안 들어오면 오류로 멈춤(s)")
    a = ap.parse_args()
    if a.every < 1 or a.workers < 1 or a.queue < 1 or a.seconds < 0 or a.frames < 0 or a.start_timeout <= 0 or a.stall_timeout <= 0:
        ap.error("--every·--workers·--queue는 1 이상, --seconds·--frames는 0 이상, 시간 제한은 양수여야 한다")

    ok, why, lock_fd = gateway_guard(a.maintenance)
    print("guard:", why)
    if not ok:
        sys.exit(2)
    pipe = writer = cap = meta = folder = None
    started = False
    exit_code, reason = 0, "사용자 중지"
    t_start = time.monotonic()
    try:
        devs = rs.context().query_devices()
        serials = [d.get_info(rs.camera_info.serial_number) for d in devs]
        if not serials:
            reason, exit_code = "L515를 찾지 못함", 3
            return
        serial = a.serial or (serials[0] if len(serials) == 1 else "")
        if serial not in serials:
            reason, exit_code = f"장치를 고를 수 없음(--serial 필요): {serials}", 3
            return
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
        started = True

        # 메타(시작 시점) — 장치·협상 프로필·보정값
        d_prof = prof.get_stream(rs.stream.depth)
        streams, ext = {}, {}
        for key, st_, idx in (("depth", rs.stream.depth, -1), ("ir", rs.stream.infrared, 0), ("color", rs.stream.color, -1),
                              ("accel", rs.stream.accel, -1), ("gyro", rs.stream.gyro, -1)):
            try:
                p_ = prof.get_stream(st_, idx) if idx >= 0 else prof.get_stream(st_)
            except Exception:
                streams[key] = None
                continue
            streams[key] = prof_dict(p_)
            if key != "depth":
                e = d_prof.get_extrinsics_to(p_)
                ext[f"depth_to_{key}"] = extrinsics_dict(e.rotation, e.translation)
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
            "sensor_options": {s_.get_info(rs.camera_info.name): options_snapshot(s_) for s_ in dev.query_sensors()},
            "imu": {"requested": not a.no_imu, "available": False, "first_sample_wall": None, "reason": imu_reason,
                    "units": {"accel": "m/s^2", "gyro": "rad/s"}},
            "stats": None,
        }
        write_json_atomic(os.path.join(folder, "meta.json"), meta)
        print(f"기록: {folder}")
        print(f"스트림: {', '.join(k for k, v in streams.items() if v)} / 저장 {30 / a.every:.0f} Hz — Enter 또는 Ctrl+C로 멈춤")

        t_first_save = None
        try:
            while True:
                time.sleep(0.5)
                now = time.monotonic()
                saved = writer.saved
                if saved and t_first_save is None:
                    t_first_save = now
                if cap.error or writer.error:
                    reason, exit_code = cap.error or writer.error, 1
                    break
                if t_first_save is None and now - t_start > a.start_timeout:
                    reason, exit_code = f"{a.start_timeout:.0f} s 안에 저장된 프레임 없음", 1
                    break
                if cap.last_video_mono is not None and now - cap.last_video_mono > a.stall_timeout:
                    reason, exit_code = f"영상이 {a.stall_timeout:.0f} s 넘게 안 들어옴", 1
                    break
                if a.frames and saved >= a.frames:
                    reason = f"--frames {a.frames} 도달"
                    break
                if a.seconds and t_first_save and now - t_first_save >= a.seconds:
                    reason = f"--seconds {a.seconds} 도달"
                    break
                el = now - (t_first_save or t_start)
                dz = cap.latest_depth
                vr = cz = float("nan")
                if dz is not None:
                    vr = float((dz > 0).mean())
                    h, w = dz.shape
                    c = dz[h // 2 - 20:h // 2 + 20, w // 2 - 20:w // 2 + 20]
                    c = c[c > 0]
                    cz = float(np.median(c)) * meta["depth_scale_m"] if c.size else float("nan")
                sys.stdout.write(f"\r{el:6.1f}s 저장 {saved:5d} ({saved / el if el > 0 else 0:4.1f} Hz) 큐 {writer.q.qsize():3d} "
                                 f"버림 큐 {writer.dropped_queue} SDK {sum(cap.fn_gaps.values())} 불완전 {cap.incomplete} "
                                 f"IMU {sum(cap.imu_n.values()):6d} 유효 {vr:4.2f} 가운데 {cz:5.2f} m   ")
                sys.stdout.flush()
                if sys.stdin.isatty() and select.select([sys.stdin], [], [], 0)[0]:
                    sys.stdin.readline()
                    break
        except KeyboardInterrupt:
            pass
    except Exception as e:  # 시작·메타 단계 예외도 아래 정리로 기록을 확정한다
        reason, exit_code = f"예외: {e!r}", 1
    finally:
        print(f"\n멈춤: {reason}")
        cleanup_err = []
        if cap is not None:
            cap.stopping = True
        if pipe is not None and started:
            try:
                pipe.stop()
            except Exception as e:
                cleanup_err.append(f"pipe.stop: {e!r}")
        t_end = time.monotonic()
        if writer is not None:
            st = writer.close()
            if meta is None:  # 메타를 쓰기 전에 실패 — 점검기가 정상 기록으로 오인하지 않게 최소 정보만 남긴다
                meta = {"format": FORMAT, "format_version": FORMAT_VERSION, "tool_version": TOOL_VERSION, "imu": {}}
            meta["imu"]["available"] = bool(cap and sum(cap.imu_n.values()) > 0)
            meta["imu"]["first_sample_wall"] = cap.imu_first if cap else None
            err = st["error"] or (cap.error if cap else None) or (reason if exit_code else None)
            meta["stats"] = {
                "stop_reason": reason,
                "framesets_received": cap.capture_idx if cap else 0,
                "skipped_warmup": cap.skipped_warmup if cap else 0, "skipped_every": cap.skipped_every if cap else 0,
                "incomplete_framesets": cap.incomplete if cap else 0,
                "dropped_app_queue": st["dropped_app_queue"], "sdk_frame_number_gaps": cap.fn_gaps if cap else {},
                "saved": st["saved"], "write_errors": st["write_errors"], "error": err,
                "cleanup_errors": cleanup_err, "imu_samples": st["imu_samples"],
                "duration_s": round(t_end - t_start, 3), "monotonic_start": t_start, "monotonic_end": t_end,
            }
            write_json_atomic(os.path.join(folder, "meta.json"), meta)
            print(f"저장 {st['saved']}개, 버림(큐) {st['dropped_app_queue']}, SDK 번호 틈 {meta['stats']['sdk_frame_number_gaps']}, "
                  f"불완전 {meta['stats']['incomplete_framesets']}, IMU {st['imu_samples']}")
            print(f"점검: python check_seq.py {folder}")
            if err:
                exit_code = exit_code or 1
        if cleanup_err:
            print("정리 중 오류:", cleanup_err)
            exit_code = exit_code or 1
        if lock_fd is not None:
            os.close(lock_fd)
        sys.exit(exit_code)


if __name__ == "__main__":
    main()
