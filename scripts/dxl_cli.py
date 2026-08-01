#!/usr/bin/env python3
"""XL430 다이나믹셀 인터랙티브 조그 CLI (벤치 점검용).

설정값은 로봇팔팀 레포(ksp118/extreme-robot, `dynamixel_control`)를 그대로 따른다:
  `/dev/ttyUSB0` @ 1 Mbps, protocol 2.0, XL430-W250(model 1060),
  Profile Acceleration 25 / Velocity 80.

⚠️ 같은 버스를 팔팀 노드(`position_node` / `moveit_dynamixel_bridge`)가 이미 잡고 있으면
   절대 같이 쓰지 말 것 — 한 서보를 두 주인이 만지면 명령이 서로를 덮어쓴다.
   `ps aux | grep -E "position_node|moveit_dynamixel_bridge"` 로 먼저 확인하고 내릴 것.

⚠️ Profile Acceleration(108)/Velocity(112)를 0으로 두면 명령마다 최고속으로 튀어
   **순간 과전류로 토크가 풀린다**(팔팀 HW-8 실기 검증, 재현율 100%). 이 CLI 는 토크를
   켤 때마다 25/80(기본값)을 다시 써 넣는다.

사용 예 (젯슨 호스트):
    pip3 install --user dynamixel-sdk       # 최초 1회
    python3 scripts/dxl_cli.py --scan       # 읽기 전용 버스 스캔
    python3 scripts/dxl_cli.py              # 인터랙티브 조그
"""
from __future__ import annotations

import argparse
from collections import deque
import contextlib
from dataclasses import dataclass
import select
import sys
import time

# 다이나믹셀 기본 설정 (팔팀 dynamixel_position_node.py 와 동일)
DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 1_000_000
PROTOCOL_VERSION = 2.0

# XL430 컨트롤 테이블
ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11          # EEPROM — 이 CLI 는 절대 쓰지 않는다
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70
ADDR_PROFILE_ACCELERATION = 108   # RAM — 토크 상태와 무관하게 쓸 수 있다
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_LOAD = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

# 70~135 는 X-시리즈에서 연속이다 → hwerr/load/velocity/position 을 SyncRead 한 번에 읽는다.
SYNC_READ_ADDR = ADDR_HARDWARE_ERROR_STATUS
SYNC_READ_LEN = ADDR_PRESENT_POSITION + 4 - ADDR_HARDWARE_ERROR_STATUS  # = 66

TORQUE_ON = 1
TORQUE_OFF = 0
POSITION_MODE = 3
EXTENDED_POSITION_MODE = 4
JOGGABLE_MODES = (POSITION_MODE, EXTENDED_POSITION_MODE)

MODEL_XL430_W250 = 1060
TICKS_PER_REV = 4096
CENTER_TICK = 2048
TICKS_PER_DEG = TICKS_PER_REV / 360.0

DEFAULT_PROFILE_ACCELERATION = 25
DEFAULT_PROFILE_VELOCITY = 80

# 팔팀 실기 버스 스캔값(2026-07-29) + 랙피니언 그리퍼 2모터.
DEFAULT_IDS = (2, 3, 4, 15, 21, 22, 23, 24)
# 그리퍼는 두 모터가 항상 **같은 goal tick** 으로 움직여야 한다(좌우 조 대칭).
DEFAULT_PAIR = (3, 4)

TEMP_WARN_C = 55
TEMP_TRIP_C = 65                  # 이 온도를 넘으면 해당 모터 토크를 스스로 내린다
DEFAULT_STEP_DEG = 5.0
DEFAULT_FINE_DEG = 1.0
REFRESH_S = 0.1
LOG_LINES = 6

HW_ERROR_LABELS = {
    0: "입력전압",
    2: "과열",
    3: "엔코더",
    4: "전기충격",
    5: "과부하",
}


# --------------------------------------------------------------------- 순수 헬퍼
def signed(value: int, byte_count: int) -> int:
    """dynamixel_sdk 는 unsigned 로 준다 → 2의 보수로 되돌린다(load/velocity 는 음수가 정상)."""
    bits = byte_count * 8
    value &= (1 << bits) - 1
    return value - (1 << bits) if value >= (1 << (bits - 1)) else value


def clamp(value: int, low: int, high: int) -> int:
    return max(low, min(high, value))


def deg_to_ticks(deg: float) -> int:
    """각도 증분[deg] → tick 증분. 반올림은 0 스텝이 되지 않도록 부호를 지킨다."""
    ticks = int(round(deg * TICKS_PER_DEG))
    if ticks == 0 and deg != 0.0:
        ticks = 1 if deg > 0 else -1
    return ticks


def tick_to_deg(tick: int) -> float:
    """2048 을 0°로 보는 절대 각도."""
    return (tick - CENTER_TICK) / TICKS_PER_DEG


def hardware_error_labels(bits: int) -> list[str]:
    return [label for bit, label in sorted(HW_ERROR_LABELS.items()) if bits & (1 << bit)]


def parse_ids(text: str) -> list[int]:
    """"21,22,23" 또는 "1-6" 또는 둘의 조합을 ID 목록으로. 중복은 순서를 지켜 제거."""
    ids: list[int] = []
    for chunk in text.replace(" ", "").split(","):
        if not chunk:
            continue
        if "-" in chunk[1:]:  # 음수 ID 는 없으니 첫 글자 뒤의 '-' 만 범위로 본다
            start, _, end = chunk.partition("-")
            span = range(int(start), int(end) + 1)
        else:
            span = [int(chunk)]
        for value in span:
            if not 0 <= value <= 252:
                raise ValueError(f"다이나믹셀 ID 범위(0~252)를 벗어났습니다: {value}")
            if value not in ids:
                ids.append(value)
    if not ids:
        raise ValueError("ID 를 하나도 파싱하지 못했습니다")
    return ids


def decode_keys(buffer: bytes) -> tuple[list[str], bytes]:
    """raw 바이트를 논리 키 이름으로. 덜 들어온 이스케이프 시퀀스는 버퍼에 남긴다."""
    arrows = {b"A": "up", b"B": "down", b"C": "right", b"D": "left"}
    keys: list[str] = []
    while buffer:
        if buffer[:1] == b"\x1b":
            if len(buffer) == 1:
                break                       # ESC 하나만 도착 — 다음 바이트를 기다린다
            if buffer[1:2] != b"[":
                keys.append("esc")
                buffer = buffer[1:]
                continue
            if len(buffer) < 3:
                break
            keys.append(arrows.get(buffer[2:3], "unknown"))
            buffer = buffer[3:]
            continue
        char = buffer[:1]
        buffer = buffer[1:]
        if char == b"\x03":
            keys.append("ctrl-c")
        else:
            keys.append(char.decode("utf-8", "replace"))
    return keys, buffer


# --------------------------------------------------------------------- 모터/타깃
@dataclass
class Motor:
    dxl_id: int
    model: int = 0
    mode: int = -1
    torque: bool = False
    position: int = 0
    load: int = 0
    velocity: int = 0
    temperature: int = 0
    hw_error: int = 0
    stale: bool = False               # 직전 SyncRead 에서 데이터가 빠졌는가

    @property
    def joggable(self) -> bool:
        return self.mode in JOGGABLE_MODES


@dataclass
class Target:
    """조그 단위. 그리퍼처럼 항상 같이 움직여야 하는 모터는 한 타깃으로 묶는다."""

    label: str
    motors: list[Motor]
    goal: int | None = None           # 이 CLI 가 명령한 tick(토크를 켤 때 현재값으로 초기화)
    limits: tuple[int, int] = (0, TICKS_PER_REV - 1)

    @property
    def ids(self) -> list[int]:
        return [motor.dxl_id for motor in self.motors]

    @property
    def lead(self) -> Motor:
        return self.motors[0]

    @property
    def torque(self) -> bool:
        return all(motor.torque for motor in self.motors)

    @property
    def joggable(self) -> bool:
        return all(motor.joggable for motor in self.motors)


def build_targets(motors: list[Motor], pair: tuple[int, ...] | None) -> list[Target]:
    """응답한 모터를 타깃으로 묶는다. pair 는 **둘 다 응답할 때만** 하나로 묶인다."""
    by_id = {motor.dxl_id: motor for motor in motors}
    paired: list[int] = []
    targets: list[Target] = []
    if pair and all(dxl_id in by_id for dxl_id in pair):
        paired = list(pair)
        targets.append(
            Target(
                label="GRIP " + "+".join(str(i) for i in pair),
                motors=[by_id[i] for i in pair],
            )
        )
    for motor in motors:
        if motor.dxl_id in paired:
            continue
        targets.append(Target(label=f"ID {motor.dxl_id:>3}", motors=[motor]))
    return targets


def motor_limits(motor: Motor, explicit: tuple[int | None, int | None]) -> tuple[int, int]:
    """조그 범위. 확장위치모드(4)는 0~4095 밖에 있을 수 있어 현재 위치 ±1회전으로 잡는다."""
    low, high = explicit
    if low is None:
        low = motor.position - TICKS_PER_REV if motor.mode == EXTENDED_POSITION_MODE else 0
    if high is None:
        high = (
            motor.position + TICKS_PER_REV
            if motor.mode == EXTENDED_POSITION_MODE
            else TICKS_PER_REV - 1
        )
    return low, high


# --------------------------------------------------------------------- 버스
def _import_sdk():
    try:
        from dynamixel_sdk import GroupSyncRead, PacketHandler, PortHandler
    except ImportError as exc:  # pragma: no cover - 설치 환경 의존
        raise SystemExit(
            "dynamixel_sdk 를 찾을 수 없습니다.\n"
            "  젯슨 호스트: pip3 install --user dynamixel-sdk\n"
            "  (팔팀 ros2_humble 컨테이너 안에서는 이미 설치돼 있습니다)"
        ) from exc
    return PortHandler, PacketHandler, GroupSyncRead


def set_ftdi_latency(port: str, value: int) -> str | None:
    """FTDI latency_timer[ms] 를 낮춘다. 기본 16 ms 는 왕복마다 그대로 붙는다.

    실패해도 치명적이지 않다(느려질 뿐) → 사유 문자열만 돌려주고 계속 진행한다.
    """
    name = port.rsplit("/", 1)[-1]
    path = f"/sys/bus/usb-serial/devices/{name}/latency_timer"
    try:
        with open(path) as handle:
            if handle.read().strip() == str(value):
                return None
        with open(path, "w") as handle:
            handle.write(str(value))
        return None
    except OSError as exc:
        return f"latency_timer 조정 실패({path}): {exc} — 통신이 느리면 " \
               f"sudo sh -c 'echo {value} > {path}'"


class Bus:
    """다이나믹셀 시리얼 버스. EEPROM 은 절대 쓰지 않는다(읽기만)."""

    def __init__(self, port: str, baud: int):
        PortHandler, PacketHandler, GroupSyncRead = _import_sdk()
        self._group_sync_read_cls = GroupSyncRead
        self.port = PortHandler(port)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise SystemExit(f"포트를 열지 못했습니다: {port} (팔팀 노드가 잡고 있진 않은지 확인)")
        if not self.port.setBaudRate(baud):
            self.port.closePort()
            raise SystemExit(f"baudrate 설정 실패: {baud}")
        self._sync_read = None

    def close(self) -> None:
        with contextlib.suppress(Exception):
            self.port.closePort()

    def ping(self, dxl_id: int) -> int | None:
        model, result, _error = self.packet.ping(self.port, dxl_id)
        return model if result == 0 else None

    def read1(self, dxl_id: int, addr: int) -> int | None:
        value, result, error = self.packet.read1ByteTxRx(self.port, dxl_id, addr)
        return value if result == 0 and error == 0 else None

    def write1(self, dxl_id: int, addr: int, value: int) -> bool:
        result, error = self.packet.write1ByteTxRx(self.port, dxl_id, addr, value)
        return result == 0 and error == 0

    def write4(self, dxl_id: int, addr: int, value: int) -> bool:
        result, error = self.packet.write4ByteTxRx(
            self.port, dxl_id, addr, value & 0xFFFFFFFF)
        return result == 0 and error == 0

    def read_position(self, dxl_id: int) -> int | None:
        value, result, error = self.packet.read4ByteTxRx(
            self.port, dxl_id, ADDR_PRESENT_POSITION)
        return signed(value, 4) if result == 0 and error == 0 else None

    def prepare_sync_read(self, ids: list[int]) -> None:
        self._sync_read = self._group_sync_read_cls(
            self.port, self.packet, SYNC_READ_ADDR, SYNC_READ_LEN)
        for dxl_id in ids:
            self._sync_read.addParam(dxl_id)

    def sync_read(self, motors: list[Motor]) -> bool:
        """전 모터의 hwerr/load/velocity/position 을 한 번에 읽어 Motor 를 갱신."""
        if self._sync_read is None or self._sync_read.txRxPacket() != 0:
            for motor in motors:
                motor.stale = True
            return False
        for motor in motors:
            if not self._sync_read.isAvailable(motor.dxl_id, SYNC_READ_ADDR, SYNC_READ_LEN):
                motor.stale = True
                continue
            motor.stale = False
            motor.hw_error = self._sync_read.getData(
                motor.dxl_id, ADDR_HARDWARE_ERROR_STATUS, 1)
            motor.load = signed(
                self._sync_read.getData(motor.dxl_id, ADDR_PRESENT_LOAD, 2), 2)
            motor.velocity = signed(
                self._sync_read.getData(motor.dxl_id, ADDR_PRESENT_VELOCITY, 4), 4)
            motor.position = signed(
                self._sync_read.getData(motor.dxl_id, ADDR_PRESENT_POSITION, 4), 4)
        return True


def discover(bus: Bus, ids: list[int]) -> list[Motor]:
    """ping → model/mode/torque/position 읽기. 쓰기는 전혀 하지 않는다."""
    motors: list[Motor] = []
    for dxl_id in ids:
        model = bus.ping(dxl_id)
        if model is None:
            continue
        motor = Motor(dxl_id=dxl_id, model=model)
        mode = bus.read1(dxl_id, ADDR_OPERATING_MODE)
        motor.mode = -1 if mode is None else mode
        motor.torque = bus.read1(dxl_id, ADDR_TORQUE_ENABLE) == TORQUE_ON
        position = bus.read_position(dxl_id)
        motor.position = 0 if position is None else position
        motor.temperature = bus.read1(dxl_id, ADDR_PRESENT_TEMPERATURE) or 0
        motor.hw_error = bus.read1(dxl_id, ADDR_HARDWARE_ERROR_STATUS) or 0
        motors.append(motor)
    return motors


# --------------------------------------------------------------------- 표시
def mode_text(mode: int) -> str:
    return {
        POSITION_MODE: "위치",
        EXTENDED_POSITION_MODE: "확장위치",
        1: "속도",
        0: "전류",
        16: "PWM",
    }.get(mode, f"모드{mode}")


def scan_lines(motors: list[Motor]) -> list[str]:
    lines = [f"{'ID':>4}  {'모델':>6}  {'모드':<8}  {'토크':<4}  "
             f"{'위치(tick)':>10}  {'각도':>8}  {'온도':>5}  에러"]
    for motor in motors:
        model = f"{motor.model}"
        if motor.model == MODEL_XL430_W250:
            model = "1060"
        errors = hardware_error_labels(motor.hw_error)
        lines.append(
            f"{motor.dxl_id:>4}  {model:>6}  {mode_text(motor.mode):<8}  "
            f"{'ON' if motor.torque else 'off':<4}  {motor.position:>10}  "
            f"{tick_to_deg(motor.position):>7.1f}°  {motor.temperature:>3}°C  "
            + (",".join(errors) if errors else "-")
        )
    return lines


def target_line(target: Target, selected: bool) -> str:
    motor = target.lead
    marker = "▶" if selected else " "
    goal = "   -" if target.goal is None else f"{target.goal:>4}"
    errors = sorted({bit for m in target.motors for bit in hardware_error_labels(m.hw_error)})
    if not target.joggable:
        flag = f"[{mode_text(motor.mode)} — 조그 불가]"
    elif any(m.stale for m in target.motors):
        flag = "[응답 없음]"
    elif errors:
        flag = "[" + ",".join(errors) + "]"
    elif max(m.temperature for m in target.motors) >= TEMP_WARN_C:
        flag = "[고온]"
    else:
        flag = ""
    return (
        f" {marker} {target.label:<10} {'토크 ON ' if target.torque else '토크 off'}  "
        f"goal {goal}  pos {motor.position:>5} ({tick_to_deg(motor.position):>7.1f}°)  "
        f"load {max((m.load for m in target.motors), key=abs):>5}  "
        f"{max(m.temperature for m in target.motors):>3}°C  {flag}"
    )


# --------------------------------------------------------------------- 터미널
@contextlib.contextmanager
def raw_terminal():
    """방향키를 한 글자씩 받기 위한 raw 모드. 어떤 경로로 빠져나가도 원복한다."""
    import termios
    import tty

    fd = sys.stdin.fileno()
    saved = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        yield
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, saved)


# --------------------------------------------------------------------- 조그 루프
class Jog:
    # 한글은 터미널에서 2칸이라 열 맞춤이 어긋난다 → 줄마다 독립적으로 묶는다.
    HELP = [
        "  ←/→ : ∓{step}°   ·   [ ] : ∓{fine}°   ·   ↑/↓ : 관절 선택",
        "  t : 토크 토글(선택)   ·   c : goal→현재 위치   ·   s : 상태 스냅샷",
        "  e : 전 모터 토크 OFF(비상)   ·   q : 종료(토크 유지)",
    ]

    def __init__(self, bus: Bus, targets: list[Target], args):
        self.bus = bus
        self.targets = targets
        self.args = args
        self.selected = 0
        self.log = deque(maxlen=LOG_LINES)
        self.motors = [motor for target in targets for motor in target.motors]
        self._temp_cursor = 0
        self._rendered_lines = 0

    # -------------------------------------------------------------- 하드웨어
    def _poll(self) -> None:
        self.bus.sync_read(self.motors)
        # 온도(146)는 SyncRead 묶음(70~135) 밖이라 따로 읽는다 → 매 주기 한 개씩만.
        if self.motors:
            self._temp_cursor %= len(self.motors)
            motor = self.motors[self._temp_cursor]
            self._temp_cursor += 1
            value = self.bus.read1(motor.dxl_id, ADDR_PRESENT_TEMPERATURE)
            if value is None:
                return
            motor.temperature = value
            if value >= TEMP_TRIP_C and motor.torque:
                self._torque(self._target_of(motor), False)
                self.say(f"⚠️ ID {motor.dxl_id} {value}°C — 토크를 내렸습니다")

    def _target_of(self, motor: Motor) -> Target:
        return next(t for t in self.targets if motor in t.motors)

    def _torque(self, target: Target, enable: bool) -> None:
        if enable:
            if not target.joggable:
                self.say(f"{target.label}: {mode_text(target.lead.mode)} 모드라 조그할 수 없습니다")
                return
            # 토크를 켜기 전에 goal 을 **현재 위치**로 맞춘다 — 낡은 goal 이 남아 있으면
            # 토크가 들어오는 순간 그리로 튄다.
            for motor in target.motors:
                position = self.bus.read_position(motor.dxl_id)
                if position is None:
                    self.say(f"⚠️ ID {motor.dxl_id} 위치를 못 읽어 토크를 켜지 않습니다")
                    return
                motor.position = position
            goal = target.lead.position
            for motor in target.motors:
                self.bus.write4(motor.dxl_id, ADDR_GOAL_POSITION, goal)
                # 0(=최고속 즉시 이동)이면 명령마다 과전류로 토크가 풀린다.
                self.bus.write4(
                    motor.dxl_id, ADDR_PROFILE_ACCELERATION, self.args.profile_acc)
                self.bus.write4(
                    motor.dxl_id, ADDR_PROFILE_VELOCITY, self.args.profile_vel)
            target.goal = goal
            target.limits = motor_limits(
                target.lead, (self.args.min_tick, self.args.max_tick))
        for motor in target.motors:
            if self.bus.write1(
                    motor.dxl_id, ADDR_TORQUE_ENABLE,
                    TORQUE_ON if enable else TORQUE_OFF):
                motor.torque = enable
            else:
                self.say(f"⚠️ ID {motor.dxl_id} 토크 {'ON' if enable else 'OFF'} 실패")
        if not enable:
            target.goal = None
        self.say(
            f"{target.label}: 토크 {'ON' if enable else 'OFF'}"
            + (f" (goal {target.goal}, 범위 {target.limits[0]}~{target.limits[1]})"
               if enable else "")
        )

    def _step(self, degrees: float) -> None:
        target = self.targets[self.selected]
        if not target.torque:
            self.say(f"{target.label}: 토크가 꺼져 있습니다 — t 로 먼저 켜세요")
            return
        errors = [bit for m in target.motors for bit in hardware_error_labels(m.hw_error)]
        if errors:
            self.say(f"{target.label}: 하드웨어 에러({','.join(errors)}) — "
                     f"t 로 토크를 껐다 켜서 해소하세요")
            return
        goal = clamp(
            (target.goal or target.lead.position) + deg_to_ticks(degrees), *target.limits)
        if goal == target.goal:
            self.say(f"{target.label}: 범위 끝({target.limits[0]}~{target.limits[1]})입니다")
            return
        for motor in target.motors:
            if not self.bus.write4(motor.dxl_id, ADDR_GOAL_POSITION, goal):
                self.say(f"⚠️ ID {motor.dxl_id} goal 쓰기 실패")
                return
        target.goal = goal

    def _resync(self) -> None:
        target = self.targets[self.selected]
        position = self.bus.read_position(target.lead.dxl_id)
        if position is None:
            self.say(f"{target.label}: 위치를 읽지 못했습니다")
            return
        target.goal = position
        if target.torque:
            for motor in target.motors:
                self.bus.write4(motor.dxl_id, ADDR_GOAL_POSITION, position)
        self.say(f"{target.label}: goal 을 현재 위치({position})로 맞췄습니다")

    def _snapshot(self) -> None:
        target = self.targets[self.selected]
        for motor in target.motors:
            errors = hardware_error_labels(motor.hw_error)
            self.say(
                f"ID {motor.dxl_id} model {motor.model} {mode_text(motor.mode)} "
                f"pos {motor.position} ({tick_to_deg(motor.position):.1f}°) "
                f"goal {target.goal} load {motor.load} vel {motor.velocity} "
                f"{motor.temperature}°C err {','.join(errors) if errors else '-'}"
            )

    # ------------------------------------------------------------------ 표시
    def say(self, message: str) -> None:
        self.log.append(f"{time.strftime('%H:%M:%S')} {message}")

    def _render(self) -> None:
        lines = [f"[jog] {self.args.port} @ {self.args.baud} — 타깃 {len(self.targets)}개", ""]
        lines += [target_line(t, i == self.selected) for i, t in enumerate(self.targets)]
        lines.append("")
        lines += [
            line.format(step=f"{self.args.step_deg:g}", fine=f"{self.args.fine_deg:g}")
            for line in self.HELP
        ]
        lines.append("")
        lines += list(self.log) + [""] * (LOG_LINES - len(self.log))

        out = []
        if self._rendered_lines:
            out.append(f"\x1b[{self._rendered_lines}A")  # 직전 블록 위로 올라가 덮어쓴다
        for line in lines:
            out.append("\x1b[2K" + line + "\n")
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self._rendered_lines = len(lines)

    # ------------------------------------------------------------------ 루프
    def handle(self, key: str) -> bool:
        """키 하나를 처리한다. False 를 돌려주면 종료."""
        if key in ("q", "ctrl-c"):
            return False
        if key == "up":
            self.selected = (self.selected - 1) % len(self.targets)
        elif key == "down":
            self.selected = (self.selected + 1) % len(self.targets)
        elif key == "right":
            self._step(self.args.step_deg)
        elif key == "left":
            self._step(-self.args.step_deg)
        elif key == "]":
            self._step(self.args.fine_deg)
        elif key == "[":
            self._step(-self.args.fine_deg)
        elif key == "t":
            target = self.targets[self.selected]
            self._torque(target, not target.torque)
        elif key == "c":
            self._resync()
        elif key == "s":
            self._snapshot()
        elif key == "e":
            for target in self.targets:
                if target.torque:
                    self._torque(target, False)
            self.say("비상: 전 모터 토크 OFF")
        return True

    def run(self) -> None:
        self.say("t 로 토크를 켜고 ←/→ 로 움직이세요. q 로 종료(토크 유지).")
        buffer = b""
        # cbreak 는 ISIG 를 남겨 두므로 Ctrl-C 는 b"\x03" 이 아니라 SIGINT 로 온다.
        with raw_terminal(), contextlib.suppress(KeyboardInterrupt):
            while True:
                self._poll()
                self._render()
                if select.select([sys.stdin], [], [], REFRESH_S)[0]:
                    chunk = sys.stdin.buffer.raw.read(64) or b""
                    keys, buffer = decode_keys(buffer + chunk)
                    for key in keys:
                        if not self.handle(key):
                            return


# --------------------------------------------------------------------- main
def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="XL430 다이나믹셀 인터랙티브 조그 CLI (벤치 점검용)",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="⚠️ 팔팀 노드(position_node/moveit_dynamixel_bridge)와 버스를 같이 쓰지 말 것.",
    )
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"기본 {DEFAULT_PORT}")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"기본 {DEFAULT_BAUD}")
    parser.add_argument(
        "--ids", default=",".join(str(i) for i in DEFAULT_IDS),
        help="스캔할 ID. 'a,b' / 'a-b' 조합 (기본: 팔팀 실기 스캔값 + 그리퍼)")
    parser.add_argument("--scan", action="store_true", help="읽기 전용 스캔만 하고 종료")
    parser.add_argument("--step-deg", type=float, default=DEFAULT_STEP_DEG)
    parser.add_argument("--fine-deg", type=float, default=DEFAULT_FINE_DEG)
    parser.add_argument("--min-tick", type=int, default=None, help="조그 하한(기본 0)")
    parser.add_argument("--max-tick", type=int, default=None, help="조그 상한(기본 4095)")
    parser.add_argument("--profile-acc", type=int, default=DEFAULT_PROFILE_ACCELERATION)
    parser.add_argument("--profile-vel", type=int, default=DEFAULT_PROFILE_VELOCITY)
    parser.add_argument(
        "--no-pair", action="store_true",
        help=f"그리퍼 {DEFAULT_PAIR[0]}+{DEFAULT_PAIR[1]} 동시 구동 묶음을 쓰지 않는다")
    parser.add_argument("--latency-timer", type=int, default=1, help="FTDI latency[ms], 0=건드리지 않음")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        ids = parse_ids(args.ids)
    except ValueError as exc:
        print(f"--ids 파싱 실패: {exc}", file=sys.stderr)
        return 2
    if args.profile_acc <= 0 or args.profile_vel <= 0:
        print("⚠️ profile acc/vel 이 0 이면 명령마다 과전류로 토크가 풀립니다 "
              "(팔팀 권장 25/80)", file=sys.stderr)
        return 2

    if args.latency_timer > 0:
        warning = set_ftdi_latency(args.port, args.latency_timer)
        if warning:
            print(warning, file=sys.stderr)

    bus = Bus(args.port, args.baud)
    try:
        print(f"버스 스캔 중… {args.port} @ {args.baud} ids={ids}")
        motors = discover(bus, ids)
        if not motors:
            print("응답하는 서보가 없습니다 — 전원/배선/포트, 그리고 팔팀 노드가 "
                  "버스를 잡고 있진 않은지 확인하세요", file=sys.stderr)
            return 1
        print("\n".join(scan_lines(motors)))
        for motor in motors:
            if motor.model != MODEL_XL430_W250:
                print(f"⚠️ ID {motor.dxl_id} 는 model {motor.model} 입니다 — 이 CLI 는 "
                      f"XL430(1060) 컨트롤 테이블을 가정합니다", file=sys.stderr)
        if args.scan:
            return 0
        if not sys.stdin.isatty():
            print("조그 모드는 터미널에서만 동작합니다 (--scan 을 쓰세요)", file=sys.stderr)
            return 2

        targets = build_targets(motors, None if args.no_pair else DEFAULT_PAIR)
        bus.prepare_sync_read([motor.dxl_id for motor in motors])
        print()
        Jog(bus, targets, args).run()
        print("\n종료했습니다 (토크는 그대로 유지 — 내리려면 다시 실행해 e 를 누르세요).")
        return 0
    finally:
        bus.close()


if __name__ == "__main__":
    raise SystemExit(main())
