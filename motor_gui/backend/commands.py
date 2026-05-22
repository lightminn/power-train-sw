from __future__ import annotations


class CommandError(Exception):
    """잘못된/미지원 command envelope."""


def normalize(cmd: dict, caps: dict) -> dict:
    """envelope 검증 + 인자 클램프. 실패 시 CommandError. 정규화된 dict 반환."""
    if not isinstance(cmd, dict):
        raise CommandError("command must be an object")
    target = cmd.get("target")
    op = cmd.get("op")
    args = dict(cmd.get("args") or {})

    allowed = caps.get("commands", {})
    if target not in allowed:
        raise CommandError(f"unknown target: {target!r}")
    if op not in allowed[target]:
        raise CommandError(f"op {op!r} not supported for target {target!r}")

    args = _clamp(target, op, args, caps)
    return {"target": target, "op": op, "args": args}


def _clamp(target: str, op: str, args: dict, caps: dict) -> dict:
    limits = caps.get("limits", {}).get(target, {})
    if op == "set_input":
        for key in ("pos", "vel", "torque", "pos_deg", "rpm", "brake_cur", "duty"):
            if key in args and key in limits:
                hi = abs(float(limits[key]))
                args[key] = max(-hi, min(hi, float(args[key])))
    elif op == "set_limit":
        for key in ("vel_limit", "current_lim"):
            if key in args:
                args[key] = max(0.0, float(args[key]))
    return args
