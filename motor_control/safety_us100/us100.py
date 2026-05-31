import time
import serial


class Us100Sensor:
    def __init__(self, port="/dev/ttyTHS1", baud=9600, timeout=0.1):
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._ser = None

    def open(self):
        try:
            self._ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
        except serial.SerialException as e:
            raise RuntimeError(
                f"센서 포트({self._port})를 열 수 없습니다. 연결과 권한을 확인하세요. ({e})"
            ) from e

    def read(self):
        if self._ser is None:
            return None
        try:
            self._ser.reset_input_buffer()
            # 0xFF 더미 8개 + 0x55 — Jetson 첫 글자 깨짐 버그 우회
            self._ser.write(b"\xff" * 8 + bytes([0x55]))
            time.sleep(0.1)
            data = self._ser.read(64)
            if len(data) < 2:
                return None
            high, low = data[-2], data[-1]
            mm = high * 256 + low
            if 20 <= mm <= 4000:
                return float(mm)
            return None
        except serial.SerialException:
            return None

    def close(self):
        if self._ser is not None:
            self._ser.close()
            self._ser = None
