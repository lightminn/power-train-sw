"""Instrument three source versions without opening any CAN socket.

Run in a fresh interpreter per source tree. This establishes application send
patterns, not firmware behavior or real bus timing. No physical driver connect
method is allowed to open a socket: drives receive a sink and steer connect is
overridden. The real ChassisManager, CornerModule and send encoders are used.
"""
import argparse
from collections import Counter
import json
from pathlib import Path
import struct
import sys

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("source_root")
parser.add_argument("--output", required=True)
args = parser.parse_args()
source = Path(args.source_root).resolve()
sys.path.insert(0, str(source / "motor_control"))

from chassis.chassis_manager import ChassisManager, DEFAULT_WHEEL_MAP
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.null_steer import NullSteer
from corner_module.steer_ak40 import AK40, SteerAk40

now = [0.0]
frames = []


class Sink:
    def send(self, message, **kwargs):
        frames.append(dict(t=now[0], id=message.arbitration_id,
                           extended=message.is_extended_id,
                           rtr=message.is_remote_frame, dlc=message.dlc,
                           data=bytes(message.data).hex()))

    def recv(self, timeout=0.0):
        return None

    def shutdown(self):
        pass


class SinkSteer(SteerAk40):
    def connect(self):
        assert isinstance(self._bus, Sink)


corners = {}
for mapping in DEFAULT_WHEEL_MAP:
    drive = DriveOdriveCan(mapping.drive_node_id, bus=Sink(), clock=lambda: now[0])
    steer = NullSteer()
    if mapping.steer_can_id is not None:
        steer = SinkSteer(mapping.steer_can_id, clock=lambda: now[0])
        steer._bus = Sink()
        steer._ak = AK40(steer._bus, mapping.steer_can_id)
    corners[mapping.wheel] = CornerModule(steer, drive, CornerConfig(), clock=lambda: now[0])

manager = ChassisManager(corners, clock=lambda: now[0])
manager.connect()
for tick in range(500):
    now[0] = tick * 0.02
    if tick == 5:
        manager.estop("us100", "liveness_timeout")
    manager.tick()

counts = Counter()
queries = Counter()
unexpected = []
for frame in frames:
    node = frame["id"] >> 5
    command = frame["id"] & 31
    counts[f'{"ext" if frame["extended"] else "std"}:0x{frame["id"]:x}:rtr={frame["rtr"]}'] += 1
    data = bytes.fromhex(frame["data"])
    if not frame["extended"] and frame["rtr"]:
        queries[f"0x{command:02x}"] += 1
        if node not in range(11, 17) or command not in (9, 20):
            unexpected.append(frame)
    elif not frame["extended"]:
        if not (node in range(11, 17) and (
                (command == 13 and data == struct.pack("<ff", 0., 0.)) or
                (command == 7 and data == struct.pack("<I", 1) + bytes(4)))):
            unexpected.append(frame)
    elif frame["id"] not in range(0x301, 0x305) or any(data):
        unexpected.append(frame)

result = dict(source_root=str(source), method="instrumented send boundary; no CAN sockets",
              seconds=10, chassis_mode=manager.mode, frames=len(frames),
              rtr_total=sum(queries.values()), rtr_per_second=sum(queries.values()) / 10,
              queries=dict(queries), unexpected_control=unexpected, counts=dict(counts))
Path(args.output).write_text(json.dumps(result, indent=2) + "\n")
print(json.dumps({k: v for k, v in result.items() if k != "counts"}))
assert manager.mode == "ESTOP"
assert not unexpected, unexpected
