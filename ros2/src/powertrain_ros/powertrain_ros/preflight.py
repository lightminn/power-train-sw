from dataclasses import dataclass
from math import isfinite
from pathlib import Path
import re
import shlex
import sys


STOP_MM_MIN = 50.0
STOP_MM_MAX = 2000.0

_PROVENANCES = {"BENCH", "COMMISSIONED"}
_OPS_TOKEN_NAMES = {"ops_console.token", "ops_controller.token"}
_ENV_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")

ENV_FILE = Path("/etc/powertrain/powertrain.env")
TOKEN_DIR = Path("/etc/powertrain")


@dataclass(frozen=True)
class PreflightResult:
    ok: bool
    failures: tuple[str, ...]
    warnings: tuple[str, ...]
    stop_mm: float | None
    provenance: str | None


def load_env_file(path: str | Path) -> dict[str, str]:
    """Parse the assignment subset used by a systemd EnvironmentFile."""
    source = Path(path)
    env = {}
    for line_number, raw_line in enumerate(source.read_text(encoding="utf-8").splitlines(), 1):
        line = raw_line.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{source}:{line_number}: expected NAME=VALUE")
        name, raw_value = line.split("=", 1)
        name = name.strip()
        if _ENV_NAME.fullmatch(name) is None:
            raise ValueError(f"{source}:{line_number}: invalid environment name")
        try:
            values = shlex.split(raw_value.strip(), comments=True, posix=True)
        except ValueError as exc:
            raise ValueError(f"{source}:{line_number}: invalid environment value: {exc}") from exc
        if len(values) > 1:
            raise ValueError(f"{source}:{line_number}: environment value must be quoted")
        env[name] = values[0] if values else ""
    return env


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


def _token_dir_lister(path: Path = TOKEN_DIR) -> set[str]:
    return {entry.name for entry in path.iterdir() if entry.is_file()}


def main() -> int:
    try:
        env = load_env_file(ENV_FILE)
        result = check_preflight(env, _token_dir_lister)
    except (OSError, ValueError) as exc:
        print(f"powertrain preflight failed: {exc}", file=sys.stderr)
        return 1

    for warning in result.warnings:
        print(f"powertrain preflight warning: {warning}", file=sys.stderr)
    if not result.ok:
        for failure in result.failures:
            print(f"powertrain preflight failed: {failure}", file=sys.stderr)
        return 1

    print(
        f"powertrain preflight ok: STOP_MM={result.stop_mm:g} "
        f"provenance={result.provenance}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
