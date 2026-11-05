"""Display criteria shared by both console entrypoints (no GTK or camera SDK)."""
from __future__ import annotations

from dataclasses import dataclass
import math
import os
from pathlib import Path

from lidar_dev.criteria import load_criteria

DEFAULT_CRITERIA = Path(__file__).resolve().parents[1] / "lidar_dev/config/obstacle_criteria_v0.2.json"


@dataclass(frozen=True)
class VisionCriteria:
    path: str
    version: str
    profile: str
    sha256: str
    rules: dict


def resolve_criteria(path=None, profile=None, *, config=None, config_dir=None,
                     environ=None) -> VisionCriteria:
    """CLI > JSON config > environment > repository default; JSON paths relative to config."""
    config = config or {}
    environ = os.environ if environ is None else environ
    configured = config.get("l515_criteria")
    value = path if path is not None else configured if configured is not None else environ.get("L515_CRITERIA", str(DEFAULT_CRITERIA))
    configured_profile = config.get("l515_criteria_profile")
    selected = (profile if profile is not None else configured_profile
                if configured_profile is not None else environ.get("L515_CRITERIA_PROFILE"))
    if not isinstance(value, (str, Path)) or not str(value).strip():
        raise ValueError("l515_criteria: 기준값 파일 경로가 필요합니다")
    if selected is not None and (not isinstance(selected, str) or not selected.strip()):
        raise ValueError("l515_criteria_profile: 프로필 이름이 필요합니다")
    file = Path(value).expanduser()
    if path is None and configured is not None and config_dir is not None and not file.is_absolute():
        file = Path(config_dir) / file
    try:
        data = load_criteria(file, selected)
        rules = data["rules"]
        for name in ("step_up_m", "trench_depth_m"):
            rule = rules.get(name)
            if name == "trench_depth_m" and rule is None:
                continue
            if not isinstance(rule, dict) or rule.get("kind") != "higher_is_worse":
                raise ValueError(f"{name}: higher_is_worse 규칙이 필요합니다")
            caution, block = rule["caution_from"], rule["block_from"]
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v)
                   for v in (caution, block)) or not 0 <= caution <= block:
                raise ValueError(f"{name}: 유한한 0 ≤ 주의 ≤ 불가 기준이 필요합니다")
        return VisionCriteria(str(file.resolve()), str(data.get("criteria_version", "미표기")),
                              str(data.get("criteria_profile") or "단일 규칙"), data["criteria_sha256"], rules)
    except (OSError, ValueError, TypeError, KeyError, AttributeError, OverflowError, SystemExit) as exc:
        raise ValueError(f"L515 기준값 {file}: {exc}") from None
