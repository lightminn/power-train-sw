# gm_worker.py — 광민 두 런타임(지형 지도 MapRuntime·카메라 오버레이 TerrainOverlayRuntime)을 자식 프로세스 하나에서 돌린다(live_overlay 7판, 10-22 밤)
#   한 프로세스 안 스레드로 돌리면 지도 처리 틱(100–180 ms, 100 ms 틱 초과)·오버레이 처리 틱(~200 ms)이 GIL을 쥐어 화면 루프가 6–8 fps까지 떨어짐(py-spy 실측)
#   → 자식에 두 런타임 + '정지한 로버' 바퀴 50 Hz·처리/송신·오버레이 송신 스레드. 부모는 깊이(원본 640×480)·IMU를 큐로 보내고,
#   지도/오버레이 패킷 원본 바이트를 받아 LatestMapReceiver.ingest·parse_packet으로 푼다(광민 코드 그대로 — 콘솔과 같은 수신 경로).
#   - fork는 카메라·스레드를 열기 전에 한다(스레드가 있는 프로세스 fork 회피). 광민 모듈은 부모가 import해 둔 것을 물려받음
#   - 깊이 큐는 2칸: 자식이 밀리면 부모가 새 프레임을 버린다(오래된 프레임이 쌓여 지연·메모리가 늘지 않게). 광민 런타임도 원래 프레임을 건너뜀
#   - IMU는 8개씩 묶어 보냄(약 20 ms) — 광민 주행 거리계는 시각 순으로 합치므로 늦게 온 묶음도 순서대로 처리됨(정지 판정은 수신 간격 0.3 s 기준)
import multiprocessing as mp
import os
import queue
import signal
import threading
import time


def _worker(depq, ctlq, outq):
    signal.signal(signal.SIGINT, signal.SIG_IGN)                     # Ctrl+C는 부모가 받아 'stop'으로 끝낸다
    from terrain_map.config import MapConfig
    from terrain_map.odometry import WheelLayout, WheelReading
    from terrain_map.runtime import MapRuntime
    from terrain_overlay.camera import CameraModel
    from terrain_overlay.runtime import RuntimeConfig, SenderSession, TerrainOverlayRuntime
    from chassis.kinematics import default_geometry

    ppid = os.getppid()
    st = {"rt": None, "cfg": None, "gen": 0, "cam": None, "cfg_kw": {}, "ort": None, "ov_hz": 5.0}
    lock = threading.Lock()
    stop = threading.Event()
    geo = default_geometry()
    layout = WheelLayout({w.name: (w.x, w.y) for w in geo.wheels}, geo.wheel_radius_m)
    zero = tuple(WheelReading(w.name, 0.0, 0.0, 0.0, 0.0, True) for w in geo.wheels)
    wheel_gap = [0.0]
    session = lambda: SenderSession(boot_id=os.urandom(16).hex(), start_mono_ns=time.monotonic_ns(), instance=os.urandom(8).hex())

    def wheel_loop():                                                # 바퀴 50 Hz — 처리 틱과 떼어 둠(간격이 0.06 s를 넘으면 영점 학습 창이 지워짐)
        nxt, last = time.monotonic(), None
        while not stop.is_set():
            nxt += 0.02
            now = time.monotonic()
            with lock:
                rt = st["rt"]
            if rt is not None:
                rt.on_wheels(now, now, zero)
                if last is not None:
                    wheel_gap[0] = max(wheel_gap[0], now - last)
                last = now
            time.sleep(max(0.0, nxt - time.monotonic()))

    def map_loop():
        t_p = t_s = t_st = 0.0
        while not stop.is_set():
            now = time.monotonic()
            with lock:
                rt, cfg, gen = st["rt"], st["cfg"], st["gen"]
            if rt is not None:
                if now - t_p >= 1.0 / (2 * cfg.process_hz):
                    rt.process_tick()
                    t_p = now
                if now - t_s >= 1.0 / cfg.status_hz:
                    raws = list(rt.send_tick())
                    if raws:
                        outq.put(("map", gen, raws, time.monotonic()))
                    t_s = now
                if now - t_st >= 1.0:
                    outq.put(("stats", {"summary": rt.processing_summary(), "wheel_gap_max": wheel_gap[0]}))
                    t_st = now
            time.sleep(0.005)

    def ov_loop():
        while not stop.is_set():
            t0 = time.monotonic()
            with lock:
                ort, hz = st["ort"], st["ov_hz"]
            if ort is not None:
                raw = ort.tick()
                if raw is not None:
                    outq.put(("ov", raw, time.monotonic()))
            time.sleep(max(0.0, 1.0 / hz - (time.monotonic() - t0)))

    def depth_loop():
        while not stop.is_set():
            try:
                arr, t_map, now, mono_ns = depq.get(timeout=0.5)
            except queue.Empty:
                continue
            with lock:
                rt, ort = st["rt"], st["ort"]
            if ort is not None:
                ort.on_depth(arr, mono_ns, now)
            if rt is not None and t_map is not None:
                rt.on_depth(arr, t_map, now)

    for fn in (wheel_loop, map_loop, ov_loop, depth_loop):
        threading.Thread(target=fn, name="gm-" + fn.__name__, daemon=True).start()
    try:
        while True:
            try:
                msg = ctlq.get(timeout=0.5)
            except queue.Empty:
                if os.getppid() != ppid:                             # 부모가 죽었으면 끝
                    break
                continue
            kind = msg[0]
            if kind == "stop":
                break
            if kind == "imu":
                with lock:
                    rt = st["rt"]
                if rt is not None:
                    for k, t, now, v in msg[1]:
                        (rt.on_accel if k == "accel" else rt.on_gyro)(t, now, v)
            elif kind == "map_init":
                st["cam"], st["cfg_kw"] = CameraModel(**msg[1]), dict(msg[2])
            elif kind == "map_start":
                _, pitch, roll, gen = msg
                cfg = MapConfig(camera_pitch_down_deg=pitch, camera_roll_deg=roll, **st["cfg_kw"])
                rt = MapRuntime(cfg, session(), layout, log_error=lambda m: print("[map]", m, flush=True))
                rt.on_camera_info(st["cam"])
                with lock:
                    st["rt"], st["cfg"], st["gen"] = rt, cfg, gen
                    wheel_gap[0] = 0.0
            elif kind == "ov_init":
                _, dcam, ccam, half_w, hz = msg
                ort = TerrainOverlayRuntime(RuntimeConfig(path_half_width_m=half_w, publish_hz=hz), session(),
                                            log_error=lambda m: print("[overlay]", m, flush=True))
                ort.set_depth_info(CameraModel(**dcam))
                ort.set_color_info(CameraModel(**ccam))
                with lock:
                    st["ort"], st["ov_hz"] = ort, hz
    finally:
        stop.set()


class GmProc:
    """부모 쪽 손잡이: 자식 시작(fork), 깊이·IMU·제어 보내기, 받은 패킷을 종류별 처리기로 넘기기(수신 스레드)."""

    def __init__(self):
        ctx = mp.get_context("fork")
        self.depq, self.ctlq, self.outq = ctx.Queue(maxsize=2), ctx.Queue(), ctx.Queue()
        self.p = ctx.Process(target=_worker, args=(self.depq, self.ctlq, self.outq), name="gm-worker", daemon=True)
        self.p.start()
        for q in (self.depq, self.ctlq):                             # 종료 때 남은 전송을 기다리지 않음: 자식이 먼저 끝나면 600 KB 깊이를 파이프에 넣다 막혀
            q.cancel_join_thread()                                   # 파이썬 종료가 끝나지 않음(10-22 시험에서 재현)
        self.handlers = {}
        self.depth_sent = self.depth_dropped = 0
        self._imu, self._imu_lock = [], threading.Lock()
        self._rx_thread = threading.Thread(target=self._rx, name="gm-rx", daemon=True)

    def start_rx(self):                                              # fork 뒤 부모에서만(스레드는 fork 전에 만들지 않음)
        if not self._rx_thread.is_alive():
            self._rx_thread.start()

    def send(self, *msg):
        self.ctlq.put(msg)

    def depth(self, arr, t_map, now, mono_ns):
        try:
            self.depq.put_nowait((arr, t_map, now, mono_ns))
            self.depth_sent += 1
        except queue.Full:
            self.depth_dropped += 1

    def imu(self, k, t, now, v):
        with self._imu_lock:
            self._imu.append((k, t, now, v))
            if len(self._imu) < 8:
                return
            batch, self._imu = self._imu, []
        self.ctlq.put(("imu", batch))

    def _rx(self):
        while True:
            try:
                msg = self.outq.get(timeout=1.0)
            except queue.Empty:
                if not self.p.is_alive():
                    print("[gm] 광민 런타임 자식 프로세스가 끝남(exit", self.p.exitcode, ")", flush=True)
                    return
                continue
            except (EOFError, OSError):
                return
            h = self.handlers.get(msg[0])
            if h is not None:
                try:
                    h(*msg[1:])
                except Exception as e:  # noqa: BLE001 — 수신 스레드가 죽지 않게
                    print(f"[gm] {msg[0]} 처리 오류: {e!r}", flush=True)

    def alive(self):
        return self.p.is_alive()

    def close(self):
        try:
            self.ctlq.put(("stop",))
            self.p.join(1.5)
        except (OSError, ValueError):
            pass
        if self.p.is_alive():
            self.p.terminate()
