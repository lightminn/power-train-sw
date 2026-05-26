from safety_us100.verdict import Verdict, STOP
from safety_us100.evaluator import evaluate


class SafetyMonitor:
    def __init__(self, sensor, cfg):
        self._sensor = sensor
        self._cfg = cfg
        self._fail_count = 0
        self._verdict = Verdict(level=STOP, distance_mm=None)

    def tick(self):
        distance = self._sensor.read()

        if distance is None:
            self._fail_count += 1
            if self._fail_count >= self._cfg.fail_stop_count:
                self._verdict = Verdict(level=STOP, distance_mm=None)
            return

        self._fail_count = 0
        level = evaluate(distance, self._cfg, self._verdict.level)
        self._verdict = Verdict(level=level, distance_mm=distance)

    def verdict(self):
        return self._verdict
