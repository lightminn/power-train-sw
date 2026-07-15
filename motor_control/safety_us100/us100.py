import time
import serial

from safety_us100.verdict import (
    INVALID_READING,
    NO_RESPONSE,
    VALID,
    SensorReading,
)


class Us100Sensor:
    """US-100 UART adapter with explicit serial-port ownership.

    A ``serial_port`` supplied by the caller remains caller-owned. ``close()``
    only detaches it. A port created by ``open()`` is sensor-owned and is closed
    by ``close()``.
    """

    def __init__(
        self,
        port="/dev/ttyTHS1",
        baud=9600,
        timeout=0.1,
        serial_port=None,
        sleeper=time.sleep,
    ):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser = serial_port
        self._owns_serial_port = False
        self._sleeper = sleeper
        self._response_wait = 0.1

    def open(self):
        try:
            self._ser = serial.Serial(
                self._port,
                self._baud,
                timeout=self._timeout,
                write_timeout=0.1,
            )
            self._owns_serial_port = True
        except serial.SerialException as e:
            raise RuntimeError(
                f"센서 포트({self._port})를 열 수 없습니다. 연결과 권한을 확인하세요. ({e})"
            ) from e

    def _request(self, command, expected):
        self._ser.reset_input_buffer()
        self._ser.write(b"\xff" * 8 + bytes([command]))
        self._ser.flush()
        self._sleeper(self._response_wait)
        return self._ser.read(expected)

    def read(self):
        if self._ser is None:
            return SensorReading(NO_RESPONSE, None, "port_closed")
        try:
            data = self._request(0x55, 2)
            if len(data) >= 2:
                mm = data[-2] * 256 + data[-1]
                if 20 <= mm <= 4000:
                    return SensorReading(VALID, float(mm), "distance")
                return SensorReading(INVALID_READING, None, "out_of_range")

            alive = self._request(0x50, 1)
            if len(alive) >= 1:
                return SensorReading(
                    INVALID_READING,
                    None,
                    "temperature_alive",
                )
            return SensorReading(NO_RESPONSE, None, "liveness_timeout")
        except (serial.SerialTimeoutException, serial.SerialException):
            return SensorReading(NO_RESPONSE, None, "serial_error")

    def close(self):
        serial_port = self._ser
        owns_serial_port = self._owns_serial_port
        self._ser = None
        self._owns_serial_port = False
        if serial_port is not None and owns_serial_port:
            serial_port.close()
