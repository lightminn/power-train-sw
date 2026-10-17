"""ROS-free tick logic of the terrain map node (spec §4.8, §5.3).

The node converts messages and sends bytes; everything that decides what the
map holds and what a packet says lives here.  Live and deterministic replay use
the same object: replay passes stamps as the clock and, with ``follow``, the
arrival-time decisions recorded live.  All times are seconds.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import threading
import time
from typing import Callable, Iterable, NamedTuple

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import DetectorConfig, TerrainDetector

from . import contract as C
from .classmap import ClassMap
from .config import MapConfig
from .correction import Correction, apply_delta, make_corrector, update_delta
from .geometry import IDENTITY, Pose2D, base_to_optical
from .odometry import FollowPlan, GyroSample, PlanarOdometry, PoseSample, WheelLayout, WheelSample, follow_plan
from .pose import PoseTrack
from .schedule import POSE_EXPIRED, DepthFrame, FrameScheduler
from .scan import build_local_scan, decimate, make_frame, support_check

DEPTH_STALE_S = 0.5
RECENT_S = 1.0          # NO_POSE / NO_GROUND look back this far (spec §4.8)


class Decisions(NamedTuple):
    odometry: FollowPlan
    frames: frozenset    # depth stamps processed live


def decision_sessions(records: Iterable[dict]) -> dict[str | None, list[dict]]:
    """Split a decisions log into runs by their ``session`` header (None: records before any header)."""
    runs: dict[str | None, list[dict]] = {}
    current = None
    for r in records:
        if r.get("k") == "session":
            current = str(r["instance"])
            runs[current] = []
            continue
        runs.setdefault(current, []).append(r)
    return runs


def load_decisions(records: Iterable[dict], session: str | None = None) -> Decisions:
    """Decisions of one live run.  A log appended across restarts holds several runs;
    then ``session`` (the sender instance) must name one (Codex T7 P2-2)."""
    runs = decision_sessions(records)
    if session is not None:
        if session not in runs:
            raise ValueError(f"session {session!r} not in the decisions log (have {sorted(map(str, runs))})")
        chosen = runs[session]
    elif len(runs) == 1:
        chosen = next(iter(runs.values()))
    else:
        raise ValueError(f"decisions log holds {len(runs)} runs {sorted(map(str, runs))}; choose one session")
    frames = frozenset(float(r["t"]) for r in chosen if r.get("k") == "frame" and r.get("r") == "processed")
    return Decisions(follow_plan(chosen), frames)


@dataclass
class _Epoch:
    """Per-epoch counters for the status packet."""
    late_base: int = 0
    pose_gaps: int = 0
    not_support: int = 0
    distance_m: float = 0.0
    last_xy: tuple[float, float] | None = None


def _cap(value: int) -> int:
    return max(0, min(int(value), C.MAX_COUNTER))


class MapRuntime:
    def __init__(
        self, cfg: MapConfig, session, layout: WheelLayout, gyro_rotation: np.ndarray | None = None, *,
        unix_ns: Callable[[], int] = time.time_ns,
        clock: Callable[[], float] = time.monotonic,
        log_error: Callable[[str], None] = lambda _message: None,
        log_overrun: Callable[[str], None] | None = None,
        decisions: Callable[[dict], None] | None = None,
        follow: Decisions | None = None,
    ) -> None:
        self.cfg = cfg
        self.session = session
        self._unix_ns = unix_ns
        self._clock = clock
        self._log_error = log_error
        self._log_overrun = log_error if log_overrun is None else log_overrun
        self._record = decisions if follow is None else None
        if self._record is not None:
            self._record({"k": "session", "boot": session.boot_id, "start": session.start_mono_ns,
                          "instance": session.instance})
        # v_opt = R v_base for the camera mount; the gyro shares the depth optical frame unless told otherwise.
        self._gyro_rotation = (base_to_optical(cfg.camera_pitch_down_deg, cfg.camera_roll_deg, cfg.camera_yaw_deg)
                               if gyro_rotation is None else np.asarray(gyro_rotation, float))
        self._lock = threading.RLock()
        self._following = follow is not None
        self.odometry = PlanarOdometry(cfg, layout, record=self._record,
                                       follow=None if follow is None else follow.odometry)
        self.track = PoseTrack(cfg)
        self.scheduler = FrameScheduler(cfg, record=self._record, follow=None if follow is None else follow.frames)
        self.map = ClassMap(cfg)
        self.corrector = make_corrector(cfg.corrector)
        self.delta = IDENTITY
        self.epoch = 0
        self._epoch = _Epoch()
        self._camera: CameraModel | None = None
        self._detector: TerrainDetector | None = None
        self._latest_depth: tuple[float, float, tuple] | None = None     # (stamp, rx, shape)
        self._bad_depth_rx: float | None = None
        self._last_pose_sample: PoseSample | None = None
        self._pose_ok_at: float | None = None
        self._pose_reason = "ODOM_MISSING"
        self._ground_ok_at: float | None = None
        self._ground_reason = "GROUND_NOT_FOUND"
        self._error_at: float | None = None
        self._correction = Correction(IDENTITY, False, None, "identity")
        self._changed: dict[tuple[int, int], int] = {}
        self._refresh_cursor = 0
        self.sequence = 0
        self.tile_seq = 0
        self.proc_ms: float | None = None
        self.max_processing_s = 0.0
        self.processing_overruns = 0

    # --- inputs --------------------------------------------------------------
    def on_camera_info(self, camera: CameraModel) -> None:
        with self._lock:
            if camera == self._camera:
                return
            self._camera = camera
            small = CameraModel(fx=camera.fx / self.cfg.pixel_stride, fy=camera.fy / self.cfg.pixel_stride,
                                cx=camera.cx / self.cfg.pixel_stride, cy=camera.cy / self.cfg.pixel_stride,
                                width=-(-camera.width // self.cfg.pixel_stride),
                                height=-(-camera.height // self.cfg.pixel_stride))
            self._detector = TerrainDetector(small, DetectorConfig(min_ground_points=self.cfg.ground_min_points))

    def on_depth(self, array: np.ndarray, stamp_s: float, rx_s: float) -> None:
        with self._lock:
            self._latest_depth = (float(stamp_s), float(rx_s), tuple(array.shape))
            camera = self._camera
            if camera is not None and tuple(array.shape) == (camera.height, camera.width):
                self.scheduler.push(DepthFrame(float(stamp_s), float(rx_s), array))

    def on_bad_depth(self, rx_s: float) -> None:
        """A depth message failed the encoding/step check and was dropped."""
        with self._lock:
            self._bad_depth_rx = float(rx_s)

    def on_gyro(self, stamp_s: float, rx_s: float, omega_optical, seq: int = -1) -> None:
        """Angular velocity in the optical frame -> body z (v_base = Rᵀ v_opt, spec §4.8)."""
        wz = float((self._gyro_rotation.T @ np.asarray(omega_optical, float))[2])
        with self._lock:
            self._take(self.odometry.add_gyro(GyroSample(float(stamp_s), float(rx_s), wz, seq)))

    def on_wheels(self, stamp_s: float, rx_s: float, wheels: tuple, seq: int = -1) -> None:
        with self._lock:
            self._take(self.odometry.add_wheels(WheelSample(float(stamp_s), float(rx_s), tuple(wheels), seq)))

    def _take(self, samples: list[PoseSample]) -> None:
        for s in samples:
            if s.epoch != self.epoch:
                self._sync_epoch()
            self.track.add(s)
            if s.epoch != self.epoch:
                continue
            self._last_pose_sample = s
            if s.valid:
                if self._epoch.last_xy is not None:
                    self._epoch.distance_m += math.hypot(s.x - self._epoch.last_xy[0], s.y - self._epoch.last_xy[1])
                self._epoch.last_xy = (s.x, s.y)
        self._sync_epoch()

    def _sync_epoch(self) -> None:
        """Clear everything tied to the old epoch the moment the odometry changes it."""
        if self.odometry.epoch == self.epoch:
            return
        if self._following:
            # Live processed these frames in ticks before the epoch arrived; a replay commits the
            # epoch inside one input call, so it processes them here, against the old track.
            while (job := self._select(self._clock())) is not None:
                self._process(*job)
        self.epoch = self.odometry.epoch
        self.map.clear(self.epoch)
        if self.track.epoch != self.epoch:
            self.track.clear()
            self.track.epoch = self.epoch
        self.scheduler.clear()
        self.delta = IDENTITY
        self._changed.clear()
        self._refresh_cursor = 0
        self.tile_seq = 0
        self._epoch = _Epoch(late_base=self.odometry.late_dropped)
        self._last_pose_sample = None
        # The map was just wiped: not OK again until a frame of the new epoch is processed.
        self._pose_ok_at = self._ground_ok_at = None
        self._pose_reason = "POSE_GAP"

    # --- processing tick (2 x process_hz) ------------------------------------------
    def process_tick(self) -> None:
        now = self._clock()
        with self._lock:
            self._take(self.odometry.poll(now))
            job = self._select(now)
        if job is not None:
            self._process(*job)               # outside the lock: inputs keep flowing live

    def _select(self, now: float):
        if self._detector is None:
            return None
        selected, lost = self.scheduler.select(self.track, now)
        for _, reason in lost:
            self._epoch.pose_gaps += 1
            self._pose_reason = "ODOM_MISSING" if reason == POSE_EXPIRED else reason
        if selected is None:
            return None
        self._pose_ok_at = now
        seed = apply_delta(self.delta, selected.pose)
        return selected.frame, selected.pose, seed, self.epoch, self._detector, self._camera, now

    def finish(self) -> None:
        """End of a replay: commit what is queued and process every frame that can still get a pose."""
        with self._lock:
            self._take(self.odometry.drain())
        while True:
            before = self.scheduler.processed + self.scheduler.lost
            self.process_tick()
            if self.scheduler.processed + self.scheduler.lost == before:
                return

    def _process(self, frame: DepthFrame, odom: Pose2D, seed: Pose2D, epoch: int, detector, camera, now: float) -> None:
        started = self._clock()
        outcome, scan, correction = "processed", None, None
        try:
            small, _ = decimate(frame.array, camera, self.cfg.pixel_stride)
            result = detector.process(small, frame.rx_s)
            if result.status != "OK" or result.ground_state != "FRESH" or result.ground_plane is None:
                outcome = "GROUND_NOT_FOUND"
            else:
                geometry = make_frame(result, self.cfg)
                if not support_check(result, geometry, self.cfg).ok:
                    outcome = "NOT_SUPPORT"
                else:
                    scan = build_local_scan(result, geometry, frame.stamp_s, self.cfg)
        except Exception as exc:  # noqa: BLE001 — any core failure becomes a status
            outcome = "PROCESSING_ERROR"
            self._log_error(f"terrain map processing failed: {type(exc).__name__}: {exc}")
        elapsed = self._clock() - started
        self._record_time(elapsed)
        with self._lock:
            self.proc_ms = round(elapsed * 1000.0, 1)
            if outcome == "PROCESSING_ERROR":
                self._error_at = now
            if epoch != self.epoch:
                outcome = "epoch_changed"           # processed against a map that no longer exists
            elif outcome == "processed":
                correction = self.corrector.correct(scan, seed, self.map)
                self._correction = correction
                if correction.applied:
                    self.delta = update_delta(correction.pose, odom)
                for key in self.map.fuse(scan, correction.pose, frame.stamp_s, frame.rx_s):
                    self._changed[key] = self.map.version_of(*key)
                for key in self.map.evict_if_needed():
                    self._changed.pop(key, None)
                self._ground_ok_at = now
            elif outcome in ("GROUND_NOT_FOUND", "NOT_SUPPORT"):
                self._ground_reason = outcome
                if outcome == "NOT_SUPPORT":
                    self._epoch.not_support += 1
            if self._record is not None:
                self._record({"k": "result", "t": frame.stamp_s, "epoch": epoch, "r": outcome})

    def _record_time(self, elapsed_s: float) -> None:
        self.max_processing_s = max(self.max_processing_s, elapsed_s)
        tick_s = 1.0 / (2.0 * self.cfg.process_hz)
        if elapsed_s > tick_s:
            self.processing_overruns += 1
            self._log_overrun(
                f"terrain map processing {elapsed_s * 1000.0:.2f} ms exceeds the {tick_s * 1000.0:.0f} ms tick "
                f"({self.processing_overruns} overruns, max {self.max_processing_s * 1000.0:.2f} ms)")

    def processing_summary(self) -> str:
        s, o = self.scheduler, self.odometry
        return (f"terrain map: processed {s.processed}, lost {s.lost}, skipped {s.skipped}, epochs {o.epoch_changes}, "
                f"late {o.late_dropped}, max {self.max_processing_s * 1000.0:.1f} ms, overruns {self.processing_overruns}")

    # --- send tick (status_hz) ------------------------------------------------------
    def status(self, now: float) -> tuple[str, str | None]:
        if self._camera is None:
            return "SENSOR_ERROR", "CAMERA_INFO_MISSING"
        latest, bad = self._latest_depth, self._bad_depth_rx
        if bad is not None and now - bad <= DEPTH_STALE_S and (latest is None or bad > latest[1]):
            return "SENSOR_ERROR", "DEPTH_ENCODING"
        if latest is None or now - latest[1] > DEPTH_STALE_S:
            return "SENSOR_ERROR", "DEPTH_STALE"
        if latest[2] != (self._camera.height, self._camera.width):
            return "SENSOR_ERROR", "DEPTH_SHAPE_MISMATCH"
        if self._error_at is not None and now - self._error_at <= RECENT_S:
            return "SENSOR_ERROR", "PROCESSING_ERROR"
        if self._pose_ok_at is None or now - self._pose_ok_at > RECENT_S:
            return "NO_POSE", self._pose_reason
        if self._ground_ok_at is None or now - self._ground_ok_at > RECENT_S:
            return "NO_GROUND", self._ground_reason
        return "OK", None

    def send_tick(self) -> list[bytes]:
        with self._lock:
            self._sync_epoch()
            now = self._clock()
            out = []
            raw = self._encode(self._status_packet(now))
            if raw is not None:
                out.append(raw)
                self.sequence += 1
            sent = set()
            for key in sorted(self._changed, key=self._changed.get)[:self.cfg.max_tiles_per_tick]:
                del self._changed[key]
                if key in self.map.tiles():
                    out += self._tile(key, now)
                    sent.add(key)
            keys = self.map.tiles()
            for _ in range(min(self.cfg.refresh_tiles_per_tick, len(keys))):
                self._refresh_cursor %= len(keys)
                key = keys[self._refresh_cursor]
                self._refresh_cursor += 1
                if key not in sent:
                    out += self._tile(key, now)
                    sent.add(key)
            return out

    def _tile(self, key, now: float) -> list[bytes]:
        body = self.map.encode_tile(*key, now)
        self.tile_seq += 1
        raw = self._encode(C.TilePacket(**self._common(), tx=body["tx"], ty=body["ty"], version=body["version"],
                                        tile_seq=self.tile_seq, cells=body["cells"]))
        return [] if raw is None else [raw]

    def _common(self) -> dict:
        return {
            "sender_boot_id": self.session.boot_id,
            "sender_start_mono_ns": self.session.start_mono_ns,
            "sender_instance": self.session.instance,
            "map_epoch": _cap(self.epoch),
            "sent_unix_ns": int(self._unix_ns()),
        }

    def _status_packet(self, now: float) -> C.StatusPacket:
        status, reason = self.status(now)
        sample = self._last_pose_sample
        pose = None
        if sample is not None and sample.valid:
            p = apply_delta(self.delta, Pose2D(sample.x, sample.y, sample.yaw))
            pose = C.Pose(p.x, p.y, p.yaw)
        cfg, e, corr = self.cfg, self._epoch, self._correction
        tile_count = len(self.map.tiles())
        return C.StatusPacket(
            **self._common(), sequence=_cap(self.sequence), status=status, reason=reason,
            cell_m=cfg.cell_m, tile_cells=cfg.tile_cells, age_step_s=cfg.age_step_s,
            pose=pose, distance_m=min(e.distance_m, 1e7), pose_gaps=_cap(e.pose_gaps),
            not_support=_cap(e.not_support), late_dropped=_cap(self.odometry.late_dropped - e.late_base),
            tile_count=_cap(tile_count),
            refresh_cycle_s=math.ceil(tile_count / cfg.refresh_tiles_per_tick) / cfg.status_hz,
            thresholds=C.Thresholds(cfg.obstacle_height_m, cfg.ditch_depth_m, cfg.min_observations),
            corrector=C.CorrectorInfo(self.corrector.name, corr.applied, corr.score, corr.reason),
            proc_ms=self.proc_ms,
        )

    def _encode(self, packet) -> bytes | None:
        try:
            return C.encode(packet)
        except (ValueError, TypeError) as exc:
            self._log_error(f"terrain map packet failed self-check, not sent: {exc}")
            return None
