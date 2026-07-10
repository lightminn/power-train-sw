from safety_us100.config import SafetyConfig
from safety_us100.fake_sensor import FakeUs100
from safety_us100.safety_monitor import SafetyMonitor
from safety_us100.us100 import Us100Sensor
from safety_us100.verdict import (
    CHECKING,
    INVALID_READING,
    NO_RESPONSE,
    VALID,
    SensorReading,
)


def test_config_default_values():
    cfg = SafetyConfig()
    assert cfg.stop_mm == 200.0
    assert cfg.fail_stop_count == 3
    assert cfg.port == "/dev/ttyTHS1"
    assert cfg.baud == 9600


def test_empty_fake_reports_no_response():
    reading = FakeUs100([]).read()
    assert reading == SensorReading(NO_RESPONSE, None, "no_fake_reading")


def test_initial_verdict_is_checking():
    mon = SafetyMonitor(
        FakeUs100([SensorReading(VALID, 500.0, "distance")]),
        SafetyConfig(),
    )
    verdict = mon.verdict()
    assert verdict.status == CHECKING
    assert verdict.estop_required is False


def test_far_valid_distance_does_not_request_estop():
    mon = SafetyMonitor(
        FakeUs100([SensorReading(VALID, 500.0, "distance")]),
        SafetyConfig(),
    )
    mon.tick()
    verdict = mon.verdict()
    assert verdict.status == VALID
    assert verdict.estop_required is False


def test_near_valid_distance_requests_estop():
    mon = SafetyMonitor(
        FakeUs100([SensorReading(VALID, 150.0, "distance")]),
        SafetyConfig(),
    )
    mon.tick()
    assert mon.verdict().estop_required is True


def test_distance_at_stop_threshold_does_not_request_estop():
    cfg = SafetyConfig(stop_mm=200.0)
    mon = SafetyMonitor(
        FakeUs100([SensorReading(VALID, 200.0, "distance")]),
        cfg,
    )
    mon.tick()
    assert mon.verdict().estop_required is False


def test_invalid_reading_is_normal_when_liveness_responds():
    mon = SafetyMonitor(
        FakeUs100([SensorReading(INVALID_READING, None, "temperature_alive")]),
        SafetyConfig(),
    )
    mon.tick()
    verdict = mon.verdict()
    assert verdict.status == INVALID_READING
    assert verdict.estop_required is False


def test_first_two_misses_are_checking_and_third_is_no_response():
    miss = SensorReading(NO_RESPONSE, None, "liveness_timeout")
    mon = SafetyMonitor(
        FakeUs100([miss, miss, miss]),
        SafetyConfig(fail_stop_count=3),
    )
    states = []
    for _ in range(3):
        mon.tick()
        states.append((mon.verdict().status, mon.verdict().estop_required))
    assert states == [(CHECKING, False), (CHECKING, False), (NO_RESPONSE, True)]


def test_alive_response_resets_consecutive_failures():
    miss = SensorReading(NO_RESPONSE, None, "timeout")
    alive = SensorReading(INVALID_READING, None, "temperature_alive")
    mon = SafetyMonitor(
        FakeUs100([miss, miss, alive, miss]),
        SafetyConfig(),
    )
    for _ in range(4):
        mon.tick()
    assert mon.verdict().status == CHECKING
    assert mon.verdict().consecutive_failures == 1


class FakeSerial:
    def __init__(self, responses):
        self.responses = list(responses)
        self.writes = []

    def reset_input_buffer(self):
        pass

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        pass

    def read(self, size):
        return self.responses.pop(0) if self.responses else b""


def test_sensor_returns_valid_distance_without_liveness_probe():
    ser = FakeSerial([bytes([0x01, 0xF4])])  # 500 mm
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading == SensorReading(VALID, 500.0, "distance")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]


def test_sensor_marks_out_of_range_distance_response_invalid():
    ser = FakeSerial([bytes([0x00, 0x00])])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading == SensorReading(INVALID_READING, None, "out_of_range")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]


def test_sensor_uses_temperature_as_liveness_after_distance_timeout():
    ser = FakeSerial([b"", bytes([70])])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading.status == INVALID_READING
    assert reading.detail == "temperature_alive"
    assert ser.writes == [
        b"\xff" * 8 + b"\x55",
        b"\xff" * 8 + b"\x50",
    ]


def test_sensor_reports_no_response_when_distance_and_liveness_timeout():
    ser = FakeSerial([b"", b""])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading.status == NO_RESPONSE
    assert reading.detail == "liveness_timeout"


def test_sensor_reports_no_response_when_port_is_closed():
    sensor = Us100Sensor(sleeper=lambda _: None)
    assert sensor.read() == SensorReading(NO_RESPONSE, None, "port_closed")
