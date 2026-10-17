"""Map-only planar dead reckoning (spec §4.4, r10).

Yaw comes from the body-z gyro (bias learnt only while every valid wheel is
commanded and measured at rest), the body velocity from a least-squares fit of
the valid wheels given that yaw rate.  Samples are committed strictly in stamp
order up to a watermark; anything that arrives behind it is dropped and
counted.  Losing continuity (gyro gap, long wheel gap, stalled stream, stamp
regression) starts a new epoch instead of joining the old map with a wrong pose.
Standard library only.
"""
from __future__ import annotations

import bisect
from dataclasses import dataclass
import math
from typing import NamedTuple
import zlib

from .config import MapConfig


def wrap(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


class GyroSample(NamedTuple):
    stamp_s: float
    rx_s: float
    wz: float                 # body z yaw rate, rad/s (raw, bias not removed)
    seq: int = -1             # n-th copy of this stamp in its clock segment (normally 0); -1 = assign on intake
    seg: int = -1             # clock segment of this stream (+1 when its stamps run back); -1 = assign
    digest: int = -1          # content fingerprint (sample_digest), set on intake


class WheelReading(NamedTuple):
    name: str
    speed_m_s: float          # ground speed along the wheel heading, + = forward
    steer_rad: float
    cmd_turns_per_s: float
    meas_turns_per_s: float
    valid: bool               # not drive_stale and not steer_stale


class WheelSample(NamedTuple):
    stamp_s: float
    rx_s: float
    wheels: tuple
    seq: int = -1             # n-th copy of this stamp in its clock segment (normally 0); -1 = assign on intake
    seg: int = -1             # clock segment of this stream (+1 when its stamps run back); -1 = assign
    digest: int = -1          # content fingerprint (sample_digest), set on intake


class PoseSample(NamedTuple):
    stamp_s: float
    x: float
    y: float
    yaw: float
    valid: bool
    reason: str | None
    epoch: int


class BodyVelocity(NamedTuple):
    vx: float
    vy: float
    ok: bool
    reason: str | None


@dataclass(frozen=True)
class WheelLayout:
    positions: dict          # name -> (x, y) in base_link, metres
    wheel_radius_m: float


def solve_body_velocity(readings, layout: WheelLayout, omega: float, cfg: MapConfig) -> BodyVelocity:
    """Least squares (vx, vy) from s_i = (vx − ω y_i) cos δ_i + (vy + ω x_i) sin δ_i (spec §4.4)."""
    rows = []
    for r in readings:
        if not r.valid or r.name not in layout.positions:
            continue
        x_i, y_i = layout.positions[r.name]
        c, s = math.cos(r.steer_rad), math.sin(r.steer_rad)
        rows.append((c, s, r.speed_m_s + omega * y_i * c - omega * x_i * s))
    if len(rows) < cfg.min_valid_wheels:
        return BodyVelocity(0.0, 0.0, False, "WHEELS_STALE")
    scc = sum(c * c for c, _, _ in rows)
    sss = sum(s * s for _, s, _ in rows)
    scs = sum(c * s for c, s, _ in rows)
    scb = sum(c * b for c, _, b in rows)
    ssb = sum(s * b for _, s, b in rows)
    if scc < 1e-6:
        return BodyVelocity(0.0, 0.0, False, "WHEELS_STALE")
    det = scc * sss - scs * scs
    trace = scc + sss
    # condition number of the 2x2 normal matrix is (cond of A)^2
    disc = math.sqrt(max(0.0, (trace / 2.0) ** 2 - det))
    lam_max, lam_min = trace / 2.0 + disc, trace / 2.0 - disc
    cond = math.inf if lam_min <= 1e-12 else math.sqrt(lam_max / lam_min)
    if cond > cfg.ls_max_condition:
        return BodyVelocity(scb / scc, 0.0, True, None)
    vx = (sss * scb - scs * ssb) / det
    vy = (scc * ssb - scs * scb) / det
    if abs(vy) > cfg.max_lateral_speed_m_s:
        return BodyVelocity(vx, vy, False, "WHEELS_STALE")
    return BodyVelocity(vx, vy, True, None)


def is_stationary(readings, cfg: MapConfig) -> bool:
    """Every valid wheel commanded and measured at rest (and enough valid wheels) — no bias state."""
    return _stationary(tuple(_finite_reading(r) for r in readings), cfg)


def _stationary(readings, cfg: MapConfig) -> bool:
    valid = [r for r in readings if r.valid]
    if len(valid) < cfg.min_valid_wheels:
        return False
    return all(abs(r.cmd_turns_per_s) < cfg.stationary_command_turns_per_s
               and abs(r.meas_turns_per_s) < cfg.stationary_measured_turns_per_s for r in valid)


class FollowPlan(NamedTuple):
    """Live decisions replayed instead of the arrival-time rules (spec §4.4 equivalence scope).

    Samples are identified by ``(source, segment, header stamp, seq)`` (``SampleIds``):
    set by the publisher's stamps, so a message lost by only the recorder or only the
    node shifts nothing (Codex T7 P2-1); ``seq`` keeps a duplicated message apart from
    its twin (Codex T4 re-review P2-3) and ``segment`` keeps a clock restart that repeats
    old stamps apart from the first run (Codex T7 re-review P2-1).
    """
    received: frozenset | None # samples the live node got (None: no restriction)
    drops: frozenset           # samples never committed live
    before: dict               # ident -> [mode, ...] applied before that sample is processed
    after: dict                # ident | None -> [mode, ...] applied after it (None: before the first)


def _ident(value):
    return None if value is None else (value[0], int(value[1]), float(value[2]), int(value[3]), int(value[4]))


def sample_digest(sample) -> int:
    """Content fingerprint of a gyro/wheel sample: copies of one stamp with different content stay
    distinct even when only one receiver lost one of them (branch review)."""
    if isinstance(sample, GyroSample):
        data = repr(float(sample.wz))
    else:
        data = repr(tuple((r.name, float(r.speed_m_s), float(r.steer_rad), float(r.cmd_turns_per_s),
                           float(r.meas_turns_per_s), bool(r.valid)) for r in sample.wheels))
    return zlib.crc32(data.encode())


class SampleIds:
    """Assign ``(segment, copy)`` per stream in input order — the node and a replay run the same rule.

    A stamp more than ``regression_s`` behind the newest stamp of its stream's segment opens a
    new segment; ``copy`` counts repeats of one stamp inside a segment (recent stamps only).
    """

    def __init__(self, regression_s: float) -> None:
        self.regression_s = regression_s
        self._segment: dict[str, int] = {}
        self._newest: dict[str, float] = {}
        self._copies: dict[str, dict[float, int]] = {}

    def assign(self, src: str, stamp: float, digest: int = 0) -> tuple[int, int]:
        """(segment, copy) — ``copy`` counts earlier samples with this stamp *and* content."""
        newest = self._newest.get(src)
        if newest is not None and stamp < newest - self.regression_s:
            self._segment[src] = self._segment.get(src, 0) + 1
            self._copies[src] = {}
            newest = None
        self._newest[src] = stamp if newest is None else max(newest, stamp)
        copies = self._copies.setdefault(src, {})
        k = copies.get((stamp, digest), 0)
        copies[(stamp, digest)] = k + 1
        if len(copies) > 512:
            floor = self._newest[src] - 5.0
            for old in [key for key in copies if key[0] < floor]:
                del copies[old]
        return self._segment.get(src, 0), k


def follow_plan(records) -> FollowPlan:
    """Build a FollowPlan from the ``record`` hook's dicts (e.g. read back from JSONL)."""
    received, drops, before, after = set(), set(), {}, {}
    for r in records:
        kind = r.get("k")
        if kind == "rx":
            received.add((r["src"], int(r["seg"]), float(r["t"]), int(r["d"]), int(r["seq"])))
        elif kind == "drop":
            drops.add((r["src"], int(r["seg"]), float(r["t"]), int(r["d"]), int(r["seq"])))
        elif kind == "epoch":
            if "before" in r:
                before.setdefault(_ident(r["before"]), []).append(r["mode"])
            else:
                after.setdefault(_ident(r["after"]), []).append(r["mode"])
    return FollowPlan(frozenset(received) if received else None, frozenset(drops), before, after)


def _finite_reading(r: WheelReading) -> WheelReading:
    fields = (r.speed_m_s, r.steer_rad, r.cmd_turns_per_s, r.meas_turns_per_s)
    return r if all(math.isfinite(v) for v in fields) else r._replace(valid=False)


class PlanarOdometry:
    """Watermark-ordered planar dead reckoning.

    ``record`` receives every decision that depends on arrival time (dropped
    samples, epoch changes).  With ``follow`` those arrival-time rules are off
    and the recorded decisions are applied instead, so a stamp-ordered replay of
    the same inputs reproduces the live pose samples.
    """

    def __init__(self, cfg: MapConfig, layout: WheelLayout, *, record=None, follow: FollowPlan | None = None) -> None:
        self.cfg = cfg
        self.layout = layout
        self.epoch = 0
        self.bias = 0.0
        self.bias_ready = False
        self.late_dropped = 0
        self.rejected = 0             # non-finite stamps or rates
        self.trimmed = 0              # dropped while the other stream was absent
        self.flushed = 0              # queued samples discarded by a flush epoch
        self.epoch_changes = 0
        self.invalid_samples = 0
        self.processed = 0            # samples committed so far
        self._record = record
        self._follow = follow
        self._ids = SampleIds(cfg.stamp_regression_epoch_s)
        self._processing = None       # ident being processed
        self._last_id = None          # ident processed last
        self._gyro: list[GyroSample] = []
        self._wheels: list[WheelSample] = []
        self._committed = None
        self._clear_intake()
        self._reset_motion()

    # --- public -------------------------------------------------------------
    def add_gyro(self, sample: GyroSample) -> list[PoseSample]:
        return self._add(sample, "gyro")

    def add_wheels(self, sample: WheelSample) -> list[PoseSample]:
        return self._add(sample, "wheel")

    def poll(self, rx_now_s: float) -> list[PoseSample]:
        """Receive-time stall check: a silent stream breaks continuity."""
        if self._follow is not None:
            return []
        limit = self.cfg.wheel_gap_epoch_s
        if any(last is not None and rx_now_s - last > limit for last in (self._last_rx_gyro, self._last_rx_wheel)):
            self._new_epoch("flush", "stall")
        return []

    def drain(self) -> list[PoseSample]:
        """Commit everything still queued (end of a replay)."""
        return self._advance(math.inf)

    def queue_span_s(self) -> float:
        stamps = [q[0].stamp_s for q in (self._gyro, self._wheels) if q]
        latest = [s for s in (self._latest_gyro, self._latest_wheel) if s is not None]
        if not stamps or not latest:
            return 0.0
        return max(latest) - min(stamps)

    # --- intake -------------------------------------------------------------
    def _note(self, entry: dict) -> None:
        if self._record is not None and self._follow is None:
            self._record(entry)

    def _drop(self, src: str, sample, why: str) -> None:
        self._note({"k": "drop", "src": src, "seg": sample.seg, "t": sample.stamp_s, "d": sample.digest,
                    "seq": sample.seq, "why": why})

    def _add(self, sample, src: str) -> list[PoseSample]:
        gyro = src == "gyro"
        if not (math.isfinite(sample.stamp_s) and math.isfinite(sample.rx_s)) or (gyro and not math.isfinite(sample.wz)):
            self.rejected += 1
            return []                                    # no usable stamp: deterministic, nothing to record
        sample = sample._replace(digest=sample_digest(sample))
        if sample.seq < 0 or sample.seg < 0:
            seg, seq = self._ids.assign(src, sample.stamp_s, sample.digest)
            sample = sample._replace(seg=seg, seq=seq)
        ident = (src, sample.seg, sample.stamp_s, sample.digest, sample.seq)
        self._note({"k": "rx", "src": src, "seg": sample.seg, "t": sample.stamp_s, "d": sample.digest,
                    "seq": sample.seq})
        if not gyro:
            sample = sample._replace(wheels=tuple(_finite_reading(r) for r in sample.wheels))
        # The committed floor survives a stall flush, so it also detects a new time base (Codex T4 re-review P2-1).
        reference = max(v for v in (self._latest_gyro if gyro else self._latest_wheel, self._committed, -math.inf)
                        if v is not None)
        regressed = sample.stamp_s < reference - self.cfg.stamp_regression_epoch_s
        if self._follow is not None:
            received = self._follow.received
            if (received is not None and ident not in received) or ident in self._follow.drops:
                return []                                # the live node never got it, or dropped it
            if regressed:
                # Finish the old time segment (and its anchored epochs) before the new one is queued
                # in front of it (Codex T4 re-review 2).
                done = self._advance(math.inf)
                self._gyro.clear()
                self._wheels.clear()
                self._latest_gyro = self._latest_wheel = None
                self._committed = None
                return done + self._add(sample, src)
        else:
            if regressed:
                self._new_epoch("flush", "regression", reset_floor=True)
            elif self._committed is not None and sample.stamp_s <= self._committed:
                self.late_dropped += 1
                self._drop(src, sample, "late")
                return []
        queue = self._gyro if gyro else self._wheels
        keys = [s.stamp_s for s in queue]
        index = bisect.bisect_right(keys, sample.stamp_s)
        if index and keys[index - 1] == sample.stamp_s:
            self._drop(src, sample, "duplicate")
            return []
        queue.insert(index, sample)
        if gyro:
            self._latest_gyro = sample.stamp_s if self._latest_gyro is None else max(self._latest_gyro, sample.stamp_s)
            self._last_rx_gyro = sample.rx_s
        else:
            self._latest_wheel = sample.stamp_s if self._latest_wheel is None else max(self._latest_wheel, sample.stamp_s)
            self._last_rx_wheel = sample.rx_s
        if self._follow is None:
            own, other = (self._latest_gyro, self._latest_wheel) if gyro else (self._latest_wheel, self._latest_gyro)
            if other is None:
                # Nothing can commit without the other stream: keep only max_stream_skew_s of this one (Codex T4 P2-2).
                while queue and own - queue[0].stamp_s > self.cfg.max_stream_skew_s:
                    self.trimmed += 1
                    self._drop(src, queue.pop(0), "trim")
            elif abs(own - other) > self.cfg.max_stream_skew_s:
                self._new_epoch("flush", "skew")
                return []
        return self._advance()

    def _advance(self, watermark: float | None = None) -> list[PoseSample]:
        if watermark is None:
            if self._latest_gyro is None or self._latest_wheel is None:
                return []
            watermark = min(self._latest_gyro, self._latest_wheel) - self.cfg.lateness_margin_s
        out: list[PoseSample] = []
        while True:
            g = self._gyro[0] if self._gyro and self._gyro[0].stamp_s <= watermark else None
            w = self._wheels[0] if self._wheels and self._wheels[0].stamp_s <= watermark else None
            if g is None and w is None:
                return out
            if w is not None and (g is None or w.stamp_s <= g.stamp_s):
                if g is None and self._prev_gyro is not None:
                    # The gyro right of this wheel is not committed yet (Codex T4 P2-1); at the end
                    # of a replay it never will be, so the unsupported tail stays out (re-review P2-2).
                    return out
                sample, src = self._wheels.pop(0), "wheel"
            else:
                sample, src = self._gyro.pop(0), "gyro"
            self._committed = sample.stamp_s
            ident = (src, sample.seg, sample.stamp_s, sample.digest, sample.seq)
            self._followed("before", ident)
            self._processing = ident
            if src == "wheel":
                out.extend(self._on_wheel(sample))
            else:
                self._on_gyro(sample)
            self._processing = None
            self._last_id = ident
            self.processed += 1
            self._followed("after", ident)

    # --- epochs -------------------------------------------------------------
    def _clear_intake(self) -> None:
        self._gyro.clear()
        self._wheels.clear()
        self._latest_gyro = self._latest_wheel = None
        self._last_rx_gyro = self._last_rx_wheel = None

    def _reset_motion(self, *, keep_gyro: bool = False) -> None:
        self.x = self.y = self.yaw = 0.0
        if not keep_gyro:
            self._t = None            # time of the last yaw knot
            self._omega_t = 0.0       # bias-corrected yaw rate at self._t
            self._prev_gyro = None    # (stamp, raw wz) of the last processed gyro
        self._knots: list[tuple[float, float]] = []   # (t, yaw) since the last wheel sample
        self._last_ok_t = None        # stamp of the last wheel sample with a velocity solution
        self._last_v = None           # (vx, vy) at that sample
        self._window = None           # (start_t, sum, sumsq, n) stationary gyro window

    def _new_epoch(self, mode: str, cause: str, *, reset_floor: bool = False) -> None:
        """Pose back to the origin; the caller's map clears on the epoch change. Bias survives.

        ``reset``: motion state; ``keep_gyro``: the yaw-rate state survives (wheel gap);
        ``flush``: also discards the queues (live only — a follower never queued those samples).
        """
        self.epoch += 1
        self.epoch_changes += 1
        anchor = {"before": self._processing} if self._processing is not None else {"after": self._last_id}
        self._note({"k": "epoch", **anchor, "mode": mode, "cause": cause, "epoch": self.epoch})
        if mode == "flush" and self._follow is None:
            for src, queue in (("gyro", self._gyro), ("wheel", self._wheels)):
                for queued in queue:
                    self.flushed += 1
                    self._drop(src, queued, "flush")
            self._clear_intake()
            if reset_floor:
                self._committed = None
        self._reset_motion(keep_gyro=mode == "keep_gyro")

    def _followed(self, when: str, ident) -> None:
        if self._follow is None:
            return
        table = self._follow.before if when == "before" else self._follow.after
        if when == "before" and self._last_id is None:
            for mode in self._follow.after.get(None, ()):
                self._new_epoch(mode, "followed")
        for mode in table.get(ident, ()):
            self._new_epoch(mode, "followed")

    # --- processing ---------------------------------------------------------
    def _rate_at(self, t: float) -> float:
        """Bias-corrected yaw rate at t, linear between the last processed and next queued gyro."""
        t0, raw0 = self._prev_gyro
        nxt = self._gyro[0] if self._gyro else None
        if nxt is None or nxt.stamp_s <= t0 or t <= t0:
            return raw0 - self.bias
        a = (t - t0) / (nxt.stamp_s - t0)
        return (raw0 + a * (nxt.wz - raw0)) - self.bias

    def _yaw_to(self, t: float, omega_t: float) -> None:
        if self._t is not None and t > self._t:
            self.yaw = wrap(self.yaw + 0.5 * (self._omega_t + omega_t) * (t - self._t))
        self._t, self._omega_t = t, omega_t
        self._knots.append((t, self.yaw))

    def _on_gyro(self, g: GyroSample) -> None:
        if (self._follow is None and self._prev_gyro is not None
                and g.stamp_s - self._prev_gyro[0] > self.cfg.gyro_gap_epoch_s):
            self._new_epoch("reset", "gyro_gap")
        if self._window is not None:
            start, total, squares, n = self._window
            self._window = (start, total + g.wz, squares + g.wz * g.wz, n + 1)
        omega = g.wz - self.bias
        if self._t is None:
            self._t, self._omega_t = g.stamp_s, omega
            self._prev_gyro = (g.stamp_s, g.wz)
            return
        self._yaw_to(g.stamp_s, omega)
        self._prev_gyro = (g.stamp_s, g.wz)

    def _emit(self, t: float, valid: bool, reason: str | None) -> list[PoseSample]:
        if not valid:
            self.invalid_samples += 1
        return [PoseSample(t, self.x, self.y, self.yaw, valid, reason, self.epoch)]

    def _on_wheel(self, w: WheelSample) -> list[PoseSample]:
        t1 = w.stamp_s
        if self._follow is None and self._prev_gyro is not None:
            nxt = self._gyro[0] if self._gyro else None
            if nxt is not None and t1 > self._prev_gyro[0] and nxt.stamp_s - self._prev_gyro[0] > self.cfg.gyro_gap_epoch_s:
                # The gyro gap straddles this wheel sample: rotation inside it is unknowable.
                self._new_epoch("reset", "gyro_gap")
        if self._prev_gyro is None:
            self._update_window(w, ok=False)
            return self._emit(t1, False, "POSE_GAP" if self.bias_ready else "BIAS_PENDING")
        # A wheel gap and a run of too few valid wheels are the same thing: no speed since _last_ok_t.
        gap_invalid = False
        if self._last_ok_t is not None:
            gap = t1 - self._last_ok_t
            if gap > self.cfg.wheel_gap_epoch_s:
                if self._follow is None:
                    self._new_epoch("keep_gyro", "wheel_gap")
            elif gap > self.cfg.wheel_gap_invalid_s:
                gap_invalid = True
        omega = self._rate_at(t1)
        if self._t is None or t1 >= self._t:
            self._yaw_to(t1, omega)
        sol = solve_body_velocity(w.wheels, self.layout, omega, self.cfg)
        v1 = (sol.vx, sol.vy) if sol.ok else None
        self._translate(v1, gap_invalid)
        if v1 is not None:
            self._last_ok_t, self._last_v = t1, v1
        self._knots = [(t1, self.yaw)]
        self._update_window(w, ok=sol.ok and not gap_invalid)
        if not self.bias_ready:
            return self._emit(t1, False, "BIAS_PENDING")
        if not sol.ok:
            return self._emit(t1, False, sol.reason)
        if gap_invalid:
            return self._emit(t1, False, "POSE_GAP")
        return self._emit(t1, True, None)

    def _translate(self, v1, gap_invalid: bool) -> None:
        if len(self._knots) < 2 or self._last_v is None:
            return
        v0 = self._last_v
        v_end = v0 if (v1 is None or gap_invalid) else v1   # a gap extrapolates the last speed
        t_start, t_end = self._knots[0][0], self._knots[-1][0]
        span = t_end - t_start
        if span <= 0:
            return
        for (ta, ya), (tb, yb) in zip(self._knots, self._knots[1:]):
            dt = tb - ta
            if dt <= 0:
                continue
            m = ((ta + tb) / 2.0 - t_start) / span
            vx = v0[0] + m * (v_end[0] - v0[0])
            vy = v0[1] + m * (v_end[1] - v0[1])
            yaw_mid = ya + 0.5 * wrap(yb - ya)
            c, s = math.cos(yaw_mid), math.sin(yaw_mid)
            self.x += (c * vx - s * vy) * dt
            self.y += (s * vx + c * vy) * dt

    def _update_window(self, w: WheelSample, *, ok: bool) -> None:
        if not ok or not _stationary(w.wheels, self.cfg):
            self._window = None
            return
        if self._window is None:
            self._window = (w.stamp_s, 0.0, 0.0, 0)
            return
        start, total, squares, n = self._window
        if w.stamp_s - start < self.cfg.bias_stationary_s or n < 5:
            return
        mean = total / n
        std = math.sqrt(max(0.0, squares / n - mean * mean))
        if not (math.isfinite(mean) and math.isfinite(std)):
            self._window = None
            return
        if std < self.cfg.bias_max_std_rad_s:
            if self.bias_ready:
                self.bias += self.cfg.bias_alpha * (mean - self.bias)
            else:
                self.bias = mean
                self.bias_ready = True
                # Yaw integrated so far carries the unknown bias; no valid sample used it, so start clean.
                self.x = self.y = self.yaw = 0.0
                self._knots = [(w.stamp_s, 0.0)]
        self._window = (w.stamp_s, 0.0, 0.0, 0)
