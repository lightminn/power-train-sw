from safety_us100.verdict import VALID


def requires_estop(status, distance_mm, cfg):
    return (
        status == VALID
        and distance_mm is not None
        and distance_mm < cfg.stop_mm
    )
