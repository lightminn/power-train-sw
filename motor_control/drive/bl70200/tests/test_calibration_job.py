import importlib
import struct
from types import SimpleNamespace


def _can_calibration_module():
    return importlib.import_module("drive.bl70200.can_calibrate_all")


def _job_module():
    return importlib.import_module("powertrain_ros.calibration_job")


class FakeClock:
    def __init__(self) -> None:
        self.now = 0.0

    def time(self) -> float:
        return self.now

    def sleep(self, seconds: float) -> None:
        self.now += seconds

    def __call__(self) -> float:
        return self.now


def _heartbeat(node: int, error: int, state: int):
    return SimpleNamespace(
        arbitration_id=(node << 5) | 0x01,
        data=bytearray(struct.pack("<I", error) + bytes((state, 0, 0, 0))),
        is_extended_id=False,
    )


class ScriptedBus:
    """Fake CAN bus whose state commands trigger heartbeat scripts."""

    def __init__(self, node: int, scenario: str, clock: FakeClock) -> None:
        self.node = node
        self.scenario = scenario
        self.clock = clock
        self.pending = []
        self.sent_states = []
        self.last_state = None

    def send(self, message) -> None:
        command = message.arbitration_id & 0x1F
        if command != 0x07:
            return
        state = struct.unpack("<I", bytes(message.data[:4]))[0]
        self.sent_states.append(state)
        self.last_state = state

        if state == 3 and self.scenario == "full-success":
            self.pending.extend(
                (
                    _heartbeat(self.node, 0, 4),
                    _heartbeat(self.node, 0, 7),
                    _heartbeat(self.node, 0, 1),
                )
            )
        elif state == 4 and self.scenario == "fallback-success":
            self.pending.extend(
                (_heartbeat(self.node, 0, 4), _heartbeat(self.node, 0, 1))
            )
        elif state == 7 and self.scenario == "fallback-success":
            self.pending.extend(
                (_heartbeat(self.node, 0, 7), _heartbeat(self.node, 0, 1))
            )

    def recv(self, timeout=0.0):
        if timeout == 0.0:
            return None
        if self.pending:
            return self.pending.pop(0)
        if self.last_state is None:
            return _heartbeat(self.node, 0, 1)
        self.clock.sleep(timeout)
        return _heartbeat(self.node, 1, 1)


def test_calibrate_nodes_observes_full_sequence_success(monkeypatch) -> None:
    module = _can_calibration_module()
    clock = FakeClock()
    monkeypatch.setattr(module, "time", clock)
    bus = ScriptedBus(11, "full-success", clock)
    observed = []

    result = module.calibrate_nodes(
        bus, [11], observe=lambda node, ok: observed.append((node, ok))
    )

    assert result == {11: True}
    assert observed == [(11, True)]
    assert bus.sent_states == [3]


def test_calibrate_nodes_falls_back_after_full_cal_rejection(monkeypatch) -> None:
    module = _can_calibration_module()
    clock = FakeClock()
    monkeypatch.setattr(module, "time", clock)
    bus = ScriptedBus(12, "fallback-success", clock)

    result = module.calibrate_nodes(bus, [12])

    assert result == {12: True}
    assert bus.sent_states == [3, 4, 7]


def test_calibrate_nodes_reports_rejected_fallback(monkeypatch) -> None:
    module = _can_calibration_module()
    clock = FakeClock()
    monkeypatch.setattr(module, "time", clock)
    bus = ScriptedBus(13, "fallback-rejected", clock)

    result = module.calibrate_nodes(bus, [13])

    assert result == {13: False}
    assert bus.sent_states == [3, 4]


def test_calibration_job_starts_idle() -> None:
    module = _job_module()
    status = module.CalibrationJob(FakeClock()).status()

    assert status.state == module.CalibrationState.IDLE
    assert status.nodes == ()
    assert status.results == ()
    assert status.started_at is None
    assert status.finished_at is None


def test_calibration_job_start_enters_running_with_timestamp() -> None:
    module = _job_module()
    clock = FakeClock()
    clock.now = 10.0
    job = module.CalibrationJob(clock)

    assert job.start([11, 12]) is True
    status = job.status()
    assert status.state == module.CalibrationState.RUNNING
    assert status.nodes == (11, 12)
    assert status.started_at == 10.0
    assert status.finished_at is None


def test_calibration_job_rejects_concurrent_start() -> None:
    module = _job_module()
    job = module.CalibrationJob(FakeClock())

    assert job.start([11]) is True
    assert job.start([12]) is False
    assert job.status().nodes == (11,)


def test_calibration_job_finishes_done_after_all_axes_succeed() -> None:
    module = _job_module()
    clock = FakeClock()
    job = module.CalibrationJob(clock)
    job.start([11, 12])

    assert job.on_axis_result(11, True) is True
    assert job.status().state == module.CalibrationState.RUNNING
    clock.now = 55.0
    assert job.on_axis_result(12, True) is True

    status = job.status()
    assert status.state == module.CalibrationState.DONE
    assert status.results == ((11, True), (12, True))
    assert status.finished_at == 55.0


def test_calibration_job_finishes_failed_if_any_axis_fails() -> None:
    module = _job_module()
    job = module.CalibrationJob(FakeClock())
    job.start([11, 12])

    job.on_axis_result(11, False)
    job.on_axis_result(12, True)

    assert job.status().state == module.CalibrationState.FAILED


def test_calibration_job_cancel_is_terminal_and_ignores_late_results() -> None:
    module = _job_module()
    clock = FakeClock()
    job = module.CalibrationJob(clock)
    job.start([11])
    clock.now = 2.0

    assert job.cancel() is True
    assert job.on_axis_result(11, True) is False
    assert job.cancel() is False
    status = job.status()
    assert status.state == module.CalibrationState.CANCELLED
    assert status.results == ()
    assert status.finished_at == 2.0
