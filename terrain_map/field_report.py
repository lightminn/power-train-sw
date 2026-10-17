"""Field-check summaries (Task 12, spec §7.3) — pure helpers, no ROS.

The ROS side (``powertrain_ros.terrain_map_field_report``) streams a bag through
these; ``summarize_decisions`` reads the live node's ``processed_log_path``, which
carries the processing time measured *on the robot* for every frame.
"""
from __future__ import annotations

from collections import Counter
from typing import Iterable, Sequence

import numpy as np

from .config import MapConfig
from .runtime import decision_sessions


def _pct(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q))


def distribution(values: Sequence[float]) -> dict:
    v = np.asarray([x for x in values if np.isfinite(x)], float)
    if v.size == 0:
        return {"n": 0}
    return {"n": int(v.size), "p50": _pct(v, 50), "p95": _pct(v, 95), "p99": _pct(v, 99), "max": float(v.max())}


def clock_segments(stamps: Sequence[float], regression_s: float) -> list[list[float]]:
    """Split stamps (arrival order) where one runs more than ``regression_s`` behind its segment's
    newest — the odometry's clock-reset rule — so two clocks are never measured as one."""
    segments: list[list[float]] = []
    newest = None
    for t in stamps:
        if newest is None or t < newest - regression_s:
            segments.append([])
            newest = t
        segments[-1].append(t)
        newest = max(newest, t)
    return segments


def interval_stats(stamps: Sequence[float], gaps_s: Iterable[float] = (0.06, 0.30),
                   regression_s: float | None = None, sort: bool = False) -> dict:
    """Gaps between consecutive stamps: percentiles and how many exceed each limit.

    With ``regression_s`` the stamps are first split at clock resets (counted separately);
    ``sort`` measures each segment's own rhythm (publisher) instead of arrival order.
    """
    parts = clock_segments(stamps, regression_s) if regression_s is not None else [list(stamps)]
    diffs = [np.diff(np.sort(p) if sort else np.asarray(p, float)) for p in parts if len(p) > 1]
    d = np.concatenate(diffs) if diffs else np.zeros(0)
    out = {"count": int(d.size), "clock_resets": max(0, len(parts) - 1)}
    if d.size == 0:
        return out
    out.update(p50_s=_pct(d, 50), p95_s=_pct(d, 95), p99_s=_pct(d, 99), max_s=float(d.max()),
               backwards=int((d < 0).sum()),
               over_s={f"{g:g}": int((d > g + 1e-9).sum()) for g in gaps_s})
    return out


def support_suggestion(base_heights: Sequence[float], cfg: MapConfig, reference_m: float | None = None,
                       margin_m: float = 0.0) -> dict:
    """Spec §4.2/§7.3-1: the check may be switched on only if variation ≤ tol < min(obstacle, ditch) − variation.

    ``variation`` is the largest deviation of **every** sample from the reference height (no
    percentile trimming — a few low poses are real variation), plus ``margin_m``.  Measure the
    reference on flat ground first, then pool every rough-ground pose against that fixed
    reference (``reference_m``); a per-recording median hides the variation between poses.
    """
    v = np.asarray([x for x in base_heights if np.isfinite(x)], float)
    if v.size == 0:
        return {"n": 0, "can_enable": False, "approval": "no supported FRESH frame"}
    ref = float(np.median(v)) if reference_m is None else float(reference_m)
    variation = float(np.max(np.abs(v - ref))) + margin_m
    variation = float(np.ceil(variation * 1000.0 - 1e-9) / 1000.0)        # round up to the millimetre
    tol = max(variation, cfg.support_height_tol_m)
    limit = min(cfg.obstacle_height_m, cfg.ditch_depth_m) - variation
    if reference_m is None:
        # Re-centred on these samples: says nothing about their offset from the flat height.
        can_enable, approval = None, "measurement only — pool the rough poses with the flat reference (--reference-height)"
    else:
        can_enable = bool(tol < limit)
        approval = "variation ≤ tol < min(obstacle, ditch) − variation" + (" holds" if can_enable else " fails")
    return {"n": int(v.size), "reference": "given" if reference_m is not None else "median of these samples",
            "support_base_height_m": round(ref, 4), "support_base_variation_m": variation,
            "support_height_tol_m": round(tol, 4), "limit_m": round(limit, 4), "can_enable": can_enable,
            "approval": approval,
            "min": float(v.min()), "max": float(v.max()), "p5": _pct(v, 5), "p95": _pct(v, 95)}


def summarize_decisions(records: Iterable[dict], session: str | None = None) -> dict:
    """Loss rate, processing time, outcomes, epochs and dropped samples of one live run."""
    runs = decision_sessions(records)
    if session is not None:
        if session not in runs:
            raise ValueError(f"session {session!r} not in the log (have {sorted(map(str, runs))})")
        chosen = runs[session]
    elif len(runs) == 1:
        chosen = next(iter(runs.values()))
    else:
        raise ValueError(f"log holds {len(runs)} runs {sorted(map(str, runs))}; choose one session")
    frames = Counter(r["r"] for r in chosen if r.get("k") == "frame")
    lost_why = Counter(r.get("why") for r in chosen if r.get("k") == "frame" and r.get("r") == "lost")
    results = [r for r in chosen if r.get("k") == "result"]
    epochs = [r for r in chosen if r.get("k") == "epoch"]
    drops = Counter(r.get("why") for r in chosen if r.get("k") == "drop")
    key = lambda r: (r.get("seg"), r.get("t"), r.get("d"), r.get("seq"))  # noqa: E731
    handed = {key(r) for r in chosen if r.get("k") == "frame" and r.get("r") == "processed"}
    finished = {key(r): r["r"] for r in results}
    errors = sum(1 for k in handed if finished.get(k) == "PROCESSING_ERROR")
    incomplete = sum(1 for k in handed if k not in finished)
    attempted = frames["processed"] + frames["lost"]
    failed = frames["lost"] + errors + incomplete          # intentional rate skips are not in the log at all
    return {
        "session": chosen[0].get("instance") if chosen and chosen[0].get("k") == "session" else None,
        "frames": {"processed": frames["processed"], "lost": frames["lost"], "lost_why": dict(lost_why),
                   "errors": errors, "incomplete": incomplete,
                   "loss_rate": (failed / attempted) if attempted else None},
        "results": dict(Counter(r["r"] for r in results)),
        # total_ms = the whole frame (detection, correction, fusion). Logs written before it existed
        # carried "ms" = detection only, which must not pass the load criterion (re-review).
        "proc_ms": distribution([float(r["total_ms"]) for r in results if "total_ms" in r]),
        "proc_ms_note": (None if any("total_ms" in r for r in results) or not any("ms" in r for r in results)
                         else "old log: its ms covered detection only — re-measure with this version"),
        "epochs": {"count": len(epochs), "by_cause": dict(Counter(r.get("cause") for r in epochs))},
        "samples": {"received": sum(1 for r in chosen if r.get("k") == "rx"), "late": drops.get("late", 0),
                    "drops": dict(drops)},
    }
