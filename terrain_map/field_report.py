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


def interval_stats(stamps: Sequence[float], gaps_s: Iterable[float] = (0.06, 0.30)) -> dict:
    """Gaps between consecutive stamps (in arrival order): percentiles and how many exceed each limit."""
    s = np.asarray(stamps, float)
    d = np.diff(s) if s.size > 1 else np.zeros(0)
    out = {"count": int(d.size)}
    if d.size == 0:
        return out
    out.update(p50_s=_pct(d, 50), p95_s=_pct(d, 95), p99_s=_pct(d, 99), max_s=float(d.max()),
               backwards=int((d < 0).sum()),
               over_s={f"{g:g}": int((d > g + 1e-9).sum()) for g in gaps_s})
    return out


def support_suggestion(base_heights: Sequence[float], cfg: MapConfig) -> dict:
    """Spec §4.2/§7.3-1: height = median of n·o+d, variation = spread around it; the check may be
    switched on only if variation ≤ tol < min(obstacle, ditch) − variation."""
    v = np.asarray([x for x in base_heights if np.isfinite(x)], float)
    if v.size == 0:
        return {"n": 0, "can_enable": False}
    median = float(np.median(v))
    variation = max(median - _pct(v, 5), _pct(v, 95) - median)
    variation = float(np.ceil(variation * 1000.0) / 1000.0)               # round up to the millimetre
    tol = max(variation, cfg.support_height_tol_m)
    limit = min(cfg.obstacle_height_m, cfg.ditch_depth_m) - variation
    return {"n": int(v.size), "support_base_height_m": round(median, 4), "support_base_variation_m": variation,
            "support_height_tol_m": round(tol, 4), "limit_m": round(limit, 4), "can_enable": bool(tol < limit),
            "p5": _pct(v, 5), "p95": _pct(v, 95)}


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
    attempted = frames["processed"] + frames["lost"]
    return {
        "session": chosen[0].get("instance") if chosen and chosen[0].get("k") == "session" else None,
        "frames": {"processed": frames["processed"], "lost": frames["lost"], "lost_why": dict(lost_why),
                   "loss_rate": (frames["lost"] / attempted) if attempted else None},
        "results": dict(Counter(r["r"] for r in results)),
        "proc_ms": distribution([float(r["ms"]) for r in results if "ms" in r]),
        "epochs": {"count": len(epochs), "by_cause": dict(Counter(r.get("cause") for r in epochs))},
        "samples": {"received": sum(1 for r in chosen if r.get("k") == "rx"), "late": drops.get("late", 0),
                    "drops": dict(drops)},
    }
