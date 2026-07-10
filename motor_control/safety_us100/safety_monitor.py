from safety_us100.evaluator import requires_estop
from safety_us100.verdict import CHECKING, NO_RESPONSE, Verdict


class SafetyMonitor:
    def __init__(self, sensor, cfg):
        self._sensor = sensor
        self._cfg = cfg
        self._fail_count = 0
        self._verdict = Verdict(CHECKING, None, False, 0, "startup")

    def tick(self):
        reading = self._sensor.read()
        if reading.status == NO_RESPONSE:
            self._fail_count += 1
            confirmed = self._fail_count >= self._cfg.fail_stop_count
            self._verdict = Verdict(
                NO_RESPONSE if confirmed else CHECKING,
                None,
                confirmed,
                self._fail_count,
                reading.detail,
            )
            return

        self._fail_count = 0
        too_close = requires_estop(reading.status, reading.distance_mm, self._cfg)
        self._verdict = Verdict(
            reading.status,
            reading.distance_mm,
            too_close,
            0,
            "too_close" if too_close else reading.detail,
        )

    def verdict(self):
        return self._verdict
