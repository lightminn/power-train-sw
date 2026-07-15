import serial

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
        self.read_sizes = []
        self.reset_count = 0
        self.flush_count = 0
        self.close_count = 0

    def reset_input_buffer(self):
        self.reset_count += 1

    def write(self, data):
        self.writes.append(bytes(data))

    def flush(self):
        self.flush_count += 1

    def read(self, size):
        self.read_sizes.append(size)
        return self.responses.pop(0) if self.responses else b""

    def close(self):
        self.close_count += 1


def test_sensor_returns_valid_distance_without_liveness_probe():
    ser = FakeSerial([bytes([0x01, 0xF4])])  # 500 mm
    sleeps = []
    sensor = Us100Sensor(serial_port=ser, sleeper=sleeps.append)
    reading = sensor.read()
    assert reading == SensorReading(VALID, 500.0, "distance")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]
    assert ser.read_sizes == [2]
    assert ser.reset_count == 1
    assert ser.flush_count == 1
    assert sleeps == [0.1]


def test_sensor_marks_out_of_range_distance_response_invalid():
    ser = FakeSerial([bytes([0x00, 0x00])])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)
    reading = sensor.read()
    assert reading == SensorReading(INVALID_READING, None, "out_of_range")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]


def test_sensor_uses_temperature_as_liveness_after_distance_timeout():
    ser = FakeSerial([b"", bytes([70])])
    sleeps = []
    sensor = Us100Sensor(serial_port=ser, sleeper=sleeps.append)
    reading = sensor.read()
    assert reading.status == INVALID_READING
    assert reading.detail == "temperature_alive"
    assert ser.writes == [
        b"\xff" * 8 + b"\x55",
        b"\xff" * 8 + b"\x50",
    ]
    assert ser.read_sizes == [2, 1]
    assert ser.reset_count == 2
    assert ser.flush_count == 2
    assert sleeps == [0.1, 0.1]


def test_sensor_probes_liveness_after_partial_distance_response():
    ser = FakeSerial([bytes([0x01]), bytes([70])])
    sleeps = []
    sensor = Us100Sensor(serial_port=ser, sleeper=sleeps.append)

    reading = sensor.read()

    assert reading == SensorReading(
        INVALID_READING,
        None,
        "temperature_alive",
    )
    assert ser.writes == [
        b"\xff" * 8 + b"\x55",
        b"\xff" * 8 + b"\x50",
    ]
    assert ser.read_sizes == [2, 1]
    assert ser.reset_count == 2
    assert ser.flush_count == 2
    assert sleeps == [0.1, 0.1]


def test_sensor_reports_no_response_when_distance_and_liveness_timeout():
    ser = FakeSerial([b"", b""])
    sleeps = []
    sensor = Us100Sensor(serial_port=ser, sleeper=sleeps.append)
    reading = sensor.read()
    assert reading.status == NO_RESPONSE
    assert reading.detail == "liveness_timeout"
    assert ser.read_sizes == [2, 1]
    assert ser.reset_count == 2
    assert ser.flush_count == 2
    assert sleeps == [0.1, 0.1]


def test_sensor_maps_serial_exception_to_no_response():
    class FailingReadSerial(FakeSerial):
        def read(self, size):
            self.read_sizes.append(size)
            raise serial.SerialException("read failed")

    ser = FailingReadSerial([])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)

    reading = sensor.read()

    assert reading == SensorReading(NO_RESPONSE, None, "serial_error")
    assert ser.read_sizes == [2]


def test_sensor_maps_serial_write_timeout_to_no_response():
    class FailingWriteSerial(FakeSerial):
        def write(self, data):
            self.writes.append(bytes(data))
            raise serial.SerialTimeoutException("write timed out")

    ser = FailingWriteSerial([])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)

    reading = sensor.read()

    assert reading == SensorReading(NO_RESPONSE, None, "serial_error")
    assert ser.writes == [b"\xff" * 8 + b"\x55"]


def test_sensor_reports_no_response_when_port_is_closed():
    sensor = Us100Sensor(sleeper=lambda _: None)
    assert sensor.read() == SensorReading(NO_RESPONSE, None, "port_closed")


def test_close_detaches_injected_port_without_closing_caller_resource():
    ser = FakeSerial([])
    sensor = Us100Sensor(serial_port=ser, sleeper=lambda _: None)

    sensor.close()

    assert ser.close_count == 0
    assert sensor.read() == SensorReading(NO_RESPONSE, None, "port_closed")


def test_close_closes_sensor_opened_port(monkeypatch):
    ser = FakeSerial([])
    opened_with = []

    def open_serial(port, baud, timeout, write_timeout):
        opened_with.append((port, baud, timeout, write_timeout))
        return ser

    monkeypatch.setattr("safety_us100.us100.serial.Serial", open_serial)
    sensor = Us100Sensor(port="/dev/test-us100", baud=19200, timeout=0.25)
    sensor.open()

    sensor.close()

    assert opened_with == [("/dev/test-us100", 19200, 0.25, 0.1)]
    assert ser.close_count == 1
    assert sensor.read() == SensorReading(NO_RESPONSE, None, "port_closed")
