from safety_us100.verdict import SAFE, WARN, STOP

_DANGER = {SAFE: 0, WARN: 1, STOP: 2}


def evaluate(distance_mm, cfg, prev_level):
    if distance_mm is None:
        return STOP

    if distance_mm <= cfg.stop_mm:
        raw = STOP
    elif distance_mm <= cfg.warn_mm:
        raw = WARN
    else:
        raw = SAFE

    if prev_level is None:
        return raw

    if _DANGER[raw] >= _DANGER[prev_level]:
        return raw

    if prev_level == STOP:
        if distance_mm <= cfg.stop_mm + cfg.hysteresis_mm:
            return STOP
        if distance_mm <= cfg.warn_mm + cfg.hysteresis_mm:
            return WARN
        return SAFE

    if prev_level == WARN:
        if distance_mm <= cfg.warn_mm + cfg.hysteresis_mm:
            return WARN
        return SAFE

    return raw
