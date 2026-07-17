from dataclasses import dataclass
from math import isfinite


STOP_MM_MIN = 50.0
STOP_MM_MAX = 2000.0

_PROVENANCES = {"BENCH", "COMMISSIONED"}
_OPS_TOKEN_NAMES = {"ops_console.token", "ops_controller.token"}


@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    failures: tuple[str, ...]
    warnings: tuple[str, ...]
    stop_mm: float | None
    provenance: str | None


def check_preflight(env: dict, token_dir_lister) -> PreflightResult:
    failures = []
    warnings = []

    stop_mm = None
    raw_stop_mm = env.get("STOP_MM")
    if raw_stop_mm is None:
        failures.append("STOP_MM is required")
    else:
        try:
            stop_mm = float(raw_stop_mm)
        except (TypeError, ValueError):
            failures.append("STOP_MM must be numeric")
        else:
            if not isfinite(stop_mm) or not STOP_MM_MIN <= stop_mm <= STOP_MM_MAX:
                failures.append(
                    f"STOP_MM must be between {STOP_MM_MIN:g} and {STOP_MM_MAX:g} mm"
                )

    raw_provenance = env.get("STOP_MM_PROVENANCE")
    provenance = raw_provenance if raw_provenance in _PROVENANCES else None
    if provenance is None:
        failures.append("STOP_MM_PROVENANCE must be BENCH or COMMISSIONED")
    elif provenance == "BENCH":
        warnings.append("STOP_MM provenance is BENCH")

    token_names = token_dir_lister()
    if _OPS_TOKEN_NAMES.isdisjoint(token_names):
        failures.append("ops_console.token or ops_controller.token is required")

    return PreflightResult(
        ok=not failures,
        failures=tuple(failures),
        warnings=tuple(warnings),
        stop_mm=stop_mm,
        provenance=provenance,
    )
