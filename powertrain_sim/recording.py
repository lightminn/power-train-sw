"""Value-based stream recording and deterministic production-core replay."""
from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import math
import os
from pathlib import Path
from typing import Any, BinaryIO

import numpy as np

from powertrain_autonomy.terrain.depth_quality import CameraIntrinsics
from powertrain_observability.events import encode_event
from powertrain_observability.journal import recover_records
from powertrain_ros.state_estimation import ImuSample, WheelSample, WheelValue

from .fixtures import DepthFrame, GroundTruthFrame


RECORDING_SCHEMA_VERSION = 1
PRODUCTION_STREAMS = ("wheel", "imu", "depth", "detections")
ALL_STREAMS = PRODUCTION_STREAMS + ("ground_truth",)
STREAM_PRIORITY = {name: index for index, name in enumerate(ALL_STREAMS)}


@dataclass(frozen=True)
class DetectionFrame:
    stamp_s: float
    frame_id: str
    detections: tuple[Mapping[str, Any], ...]
    lead_distance_m: float | None = None
    follow_state: str | None = None


@dataclass(frozen=True)
class ReplayRecord:
    stream: str
    stamp_s: float
    sequence: int
    value: WheelSample | ImuSample | DepthFrame | DetectionFrame | GroundTruthFrame


def _stamp_ns(stamp_s: float) -> int:
    if isinstance(stamp_s, bool) or not isinstance(stamp_s, (int, float)):
        raise ValueError("stamp_s must be finite and nonnegative")
    stamp_s = float(stamp_s)
    if not math.isfinite(stamp_s) or stamp_s < 0.0:
        raise ValueError("stamp_s must be finite and nonnegative")
    return int(round(stamp_s * 1_000_000_000))


def _safe_run_id(run_id: str) -> str:
    if (
        not isinstance(run_id, str)
        or not run_id
        or run_id in {".", ".."}
        or "/" in run_id
        or "\\" in run_id
        or "\x00" in run_id
    ):
        raise ValueError("run_id must be a non-empty filename-safe string")
    return run_id


class RunWriter:
    """Write one run to stream JSONL files and depth frame NPZ files."""

    def __init__(self, run_directory: str | os.PathLike[str], *, run_id: str) -> None:
        self.directory = Path(run_directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.run_id = _safe_run_id(run_id)
        existing = [
            self.directory / f"{stream}.jsonl" for stream in ALL_STREAMS
            if (self.directory / f"{stream}.jsonl").exists()
        ]
        if existing:
            raise FileExistsError(f"recording stream already exists: {existing[0]}")
        self._sequence = 0
        self._files: dict[str, BinaryIO] = {}
        self._closed = False

    def _encoded_event(
        self,
        stream: str,
        stamp_s: float,
        payload: Mapping[str, Any],
    ) -> tuple[dict[str, Any], bytes]:
        if stream not in ALL_STREAMS:
            raise ValueError(f"unknown recording stream: {stream}")
        stamp_ns = _stamp_ns(stamp_s)
        event = {
            "schema_version": RECORDING_SCHEMA_VERSION,
            "run_id": self.run_id,
            "sequence": self._sequence,
            # Replay never reads host time. The value-based sample stamp is the
            # sole clock; wall_time_ns remains explicitly unavailable.
            "wall_time_ns": 0,
            "monotonic_ns": stamp_ns,
            "source": stream,
            "event_type": f"RECORDING_{stream.upper()}",
            "severity": "INFO",
            "payload": {"stamp_ns": stamp_ns, **payload},
        }
        return event, encode_event(event) + b"\n"

    def _write_line(self, stream: str, encoded: bytes) -> None:
        if self._closed:
            raise RuntimeError("recording writer is closed")
        output = self._files.get(stream)
        if output is None:
            output = (self.directory / f"{stream}.jsonl").open("ab")
            self._files[stream] = output
        output.write(encoded)
        self._sequence += 1

    def _append(self, stream: str, stamp_s: float, payload: Mapping[str, Any]) -> None:
        _, encoded = self._encoded_event(stream, stamp_s, payload)
        self._write_line(stream, encoded)

    def write_wheel(self, sample: WheelSample) -> None:
        wheels = [
            {
                "name": str(wheel.name),
                "command_turns_per_s": float(wheel.command_turns_per_s),
                "measured_turns_per_s": float(wheel.measured_turns_per_s),
                "steer_deg": float(wheel.steer_deg),
                "stale": bool(wheel.stale),
            }
            for wheel in sample.wheels
        ]
        self._append("wheel", sample.stamp_s, {"wheels": wheels})

    def write_imu(self, sample: ImuSample) -> None:
        self._append(
            "imu",
            sample.stamp_s,
            {
                "gyro_rad_s": [
                    float(sample.gyro_x_rad_s),
                    float(sample.gyro_y_rad_s),
                    float(sample.gyro_z_rad_s),
                ],
                "accel_m_s2": [
                    float(sample.accel_x_m_s2),
                    float(sample.accel_y_m_s2),
                    float(sample.accel_z_m_s2),
                ],
            },
        )

    def write_depth(self, frame: DepthFrame) -> None:
        if self._closed:
            raise RuntimeError("recording writer is closed")
        depth_roi = np.asarray(frame.depth_roi)
        if depth_roi.ndim != 2 or depth_roi.dtype == object:
            raise ValueError("depth_roi must be a numeric 2D array")
        sequence = self._sequence
        relative_path = Path("depth") / f"{sequence:012d}.npz"
        payload = {
            "npz_path": relative_path.as_posix(),
            "npz_member": "depth_roi",
            "shape": [int(value) for value in depth_roi.shape],
            "dtype": depth_roi.dtype.str,
            "depth_scale_m": float(frame.depth_scale_m),
            "intrinsics_px": {
                "fx": float(frame.intrinsics.fx),
                "fy": float(frame.intrinsics.fy),
                "cx": float(frame.intrinsics.cx),
                "cy": float(frame.intrinsics.cy),
            },
            "frame_id": str(frame.frame_id),
        }
        _, encoded = self._encoded_event("depth", frame.stamp_s, payload)

        depth_directory = self.directory / "depth"
        depth_directory.mkdir(exist_ok=True)
        final_path = self.directory / relative_path
        temporary_path = final_path.with_suffix(".npz.tmp")
        try:
            with temporary_path.open("xb") as output:
                np.savez_compressed(output, depth_roi=np.ascontiguousarray(depth_roi))
                output.flush()
                os.fsync(output.fileno())
            os.replace(temporary_path, final_path)
        except BaseException:
            temporary_path.unlink(missing_ok=True)
            raise
        self._write_line("depth", encoded)

    def write_detections(
        self,
        *,
        stamp_s: float,
        frame_id: str,
        detections: Sequence[Mapping[str, Any]],
        lead_distance_m: float | None = None,
        follow_state: str | None = None,
    ) -> None:
        if not isinstance(frame_id, str) or not frame_id:
            raise ValueError("frame_id must be a non-empty string")
        normalized = []
        for detection in detections:
            if not isinstance(detection, Mapping):
                raise ValueError("each detection must be a mapping")
            normalized.append(dict(detection))
        payload = {"frame_id": frame_id, "detections": normalized}
        if lead_distance_m is not None:
            lead_distance_m = float(lead_distance_m)
            if not math.isfinite(lead_distance_m) or lead_distance_m < 0.0:
                raise ValueError("lead_distance_m must be finite and nonnegative")
            payload["lead_distance_m"] = lead_distance_m
        if follow_state is not None:
            if not isinstance(follow_state, str) or not follow_state:
                raise ValueError("follow_state must be a non-empty string")
            payload["follow_state"] = follow_state
        self._append("detections", stamp_s, payload)

    def write_ground_truth(self, frame: GroundTruthFrame) -> None:
        self._append(
            "ground_truth",
            frame.stamp_s,
            {
                "x_m": float(frame.x_m),
                "y_m": float(frame.y_m),
                "z_m": float(frame.z_m),
                "yaw_rad": float(frame.yaw_rad),
                "bank_rad": float(frame.bank_rad),
                "linear_speed_m_s": float(frame.linear_speed_m_s),
                "yaw_rate_rad_s": float(frame.yaw_rate_rad_s),
            },
        )

    def flush(self) -> None:
        for output in self._files.values():
            output.flush()
            os.fsync(output.fileno())

    def close(self) -> None:
        if self._closed:
            return
        try:
            self.flush()
        finally:
            for output in self._files.values():
                output.close()
            self._files.clear()
            self._closed = True

    def __enter__(self) -> RunWriter:
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()


def _stamp_s(event: Mapping[str, Any]) -> float:
    stamp_ns = event["payload"].get("stamp_ns")
    if (
        isinstance(stamp_ns, bool)
        or not isinstance(stamp_ns, int)
        or stamp_ns < 0
        or event["monotonic_ns"] != stamp_ns
    ):
        raise ValueError("recording stamp_ns is invalid or inconsistent")
    return stamp_ns / 1_000_000_000.0


class RecordedRun:
    """Read and merge complete stream records from one run directory."""

    def __init__(self, run_directory: str | os.PathLike[str]) -> None:
        self.directory = Path(run_directory)
        if not self.directory.is_dir():
            raise ValueError("recorded run directory does not exist")

    def _events(self, streams: Sequence[str]) -> list[dict[str, Any]]:
        events = []
        run_ids = set()
        sequences = set()
        for stream in streams:
            path = self.directory / f"{stream}.jsonl"
            if not path.exists():
                continue
            for event in recover_records(path):
                if event["schema_version"] != RECORDING_SCHEMA_VERSION:
                    raise ValueError("unsupported recording schema_version")
                if event["source"] != stream:
                    raise ValueError("record source does not match its stream file")
                if event["event_type"] != f"RECORDING_{stream.upper()}":
                    raise ValueError("record event_type does not match its stream file")
                if event["sequence"] in sequences:
                    raise ValueError("duplicate global recording sequence")
                sequences.add(event["sequence"])
                run_ids.add(event["run_id"])
                events.append(event)
        if len(run_ids) > 1:
            raise ValueError("stream files contain more than one run_id")
        return events

    def _value(self, stream: str, event: Mapping[str, Any]):
        payload = event["payload"]
        stamp_s = _stamp_s(event)
        if stream == "wheel":
            wheels = tuple(
                WheelValue(
                    name=str(wheel["name"]),
                    command_turns_per_s=float(wheel["command_turns_per_s"]),
                    measured_turns_per_s=float(wheel["measured_turns_per_s"]),
                    steer_deg=float(wheel["steer_deg"]),
                    stale=bool(wheel["stale"]),
                )
                for wheel in payload["wheels"]
            )
            return WheelSample(stamp_s=stamp_s, wheels=wheels)
        if stream == "imu":
            gyro = payload["gyro_rad_s"]
            accel = payload["accel_m_s2"]
            if len(gyro) != 3 or len(accel) != 3:
                raise ValueError("recorded IMU vectors must contain three values")
            return ImuSample(
                stamp_s=stamp_s,
                gyro_x_rad_s=float(gyro[0]),
                gyro_y_rad_s=float(gyro[1]),
                gyro_z_rad_s=float(gyro[2]),
                accel_x_m_s2=float(accel[0]),
                accel_y_m_s2=float(accel[1]),
                accel_z_m_s2=float(accel[2]),
            )
        if stream == "depth":
            relative_path = Path(payload["npz_path"])
            if relative_path.is_absolute() or ".." in relative_path.parts:
                raise ValueError("depth npz_path must stay within the run directory")
            with np.load(self.directory / relative_path, allow_pickle=False) as archive:
                depth_roi = np.array(archive[payload["npz_member"]], copy=True)
            if list(depth_roi.shape) != payload["shape"] or depth_roi.dtype.str != payload["dtype"]:
                raise ValueError("depth NPZ does not match its JSONL metadata")
            intrinsics = payload["intrinsics_px"]
            depth_roi.setflags(write=False)
            return DepthFrame(
                stamp_s=stamp_s,
                depth_roi=depth_roi,
                depth_scale_m=float(payload["depth_scale_m"]),
                intrinsics=CameraIntrinsics(
                    fx=float(intrinsics["fx"]),
                    fy=float(intrinsics["fy"]),
                    cx=float(intrinsics["cx"]),
                    cy=float(intrinsics["cy"]),
                ),
                frame_id=str(payload["frame_id"]),
            )
        if stream == "detections":
            return DetectionFrame(
                stamp_s=stamp_s,
                frame_id=str(payload["frame_id"]),
                detections=tuple(dict(item) for item in payload["detections"]),
                lead_distance_m=(
                    None
                    if payload.get("lead_distance_m") is None
                    else float(payload["lead_distance_m"])
                ),
                follow_state=(
                    None
                    if payload.get("follow_state") is None
                    else str(payload["follow_state"])
                ),
            )
        return GroundTruthFrame(
            stamp_s=stamp_s,
            x_m=float(payload["x_m"]),
            y_m=float(payload["y_m"]),
            z_m=float(payload["z_m"]),
            yaw_rad=float(payload["yaw_rad"]),
            bank_rad=float(payload["bank_rad"]),
            linear_speed_m_s=float(payload["linear_speed_m_s"]),
            yaw_rate_rad_s=float(payload["yaw_rate_rad_s"]),
        )

    def _records(self, streams: Sequence[str]):
        events = self._events(streams)
        events.sort(
            key=lambda event: (
                event["monotonic_ns"],
                event["sequence"],
                STREAM_PRIORITY[event["source"]],
            )
        )
        for event in events:
            stream = event["source"]
            yield ReplayRecord(
                stream=stream,
                stamp_s=_stamp_s(event),
                sequence=event["sequence"],
                value=self._value(stream, event),
            )

    def iter_records(self):
        """Yield only production-consumer streams in deterministic time order."""
        yield from self._records(PRODUCTION_STREAMS)

    def iter_ground_truth(self):
        """Yield isolated ``/sim/*`` truth; the replayer never injects it."""
        for record in self._records(("ground_truth",)):
            yield record.value


class Replayer:
    """Inject recorded value types into production-consumer callbacks."""

    def __init__(self, run: RecordedRun | str | os.PathLike[str]) -> None:
        self.run = run if isinstance(run, RecordedRun) else RecordedRun(run)

    def replay(
        self,
        *,
        wheel: Callable[[WheelSample], Any] | None = None,
        imu: Callable[[ImuSample], Any] | None = None,
        depth: Callable[[DepthFrame], Any] | None = None,
        detections: Callable[[DetectionFrame], Any] | None = None,
    ) -> int:
        callbacks = {
            "wheel": wheel,
            "imu": imu,
            "depth": depth,
            "detections": detections,
        }
        delivered = 0
        for record in self.run.iter_records():
            callback = callbacks[record.stream]
            if callback is not None:
                callback(record.value)
                delivered += 1
        return delivered
