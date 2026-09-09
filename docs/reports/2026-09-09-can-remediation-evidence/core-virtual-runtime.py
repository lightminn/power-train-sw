"""In-process python-can virtual transport; never opens SocketCAN or hardware."""
import json
import struct
import threading
import time
from collections import Counter

import can
from can.interfaces.virtual import VirtualBus
from chassis.chassis_manager import ChassisManager, DEFAULT_WHEEL_MAP
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.null_steer import NullSteer
from corner_module.steer_ak40 import AK40, SteerAk40

class FilteredVirtualBus(VirtualBus):
    # VirtualBus applies filters in BusABC.recv, whose timeout=0 returns None
    # after an unmatched frame. Model kernel SocketCAN filters before delivery.
    def _recv_internal(self, timeout):
        while True:
            message, _ = super()._recv_internal(timeout)
            if message is None or self._matches_filters(message):
                return message, True
            timeout = 0


channel = 'can-core-remediation-virtual'
buses = []
emulator = can.Bus(interface='virtual', channel=channel)
buses.append(emulator)
axis = {node: [1, 0.0] for node in range(11, 17)}
queries = Counter()
shutdown = threading.Event()
drop_encoder = threading.Event()


def emit(node, cmd, data):
    emulator.send(can.Message(arbitration_id=(node << 5) | cmd, data=data, is_extended_id=False))


def serve():
    next_feedback = 0.0
    while not shutdown.is_set():
        message = emulator.recv(timeout=.001)
        if message is not None and not message.is_extended_id:
            node, cmd = message.arbitration_id >> 5, message.arbitration_id & 31
            if node in axis:
                if cmd == 7:
                    axis[node][0] = struct.unpack('<I', message.data[:4])[0]
                elif cmd == 13:
                    axis[node][1] = struct.unpack('<ff', message.data)[0]
                elif message.is_remote_frame:
                    queries[cmd] += 1
                    if cmd == 9 and not drop_encoder.is_set():
                        emit(node, cmd, struct.pack('<ff', 0, axis[node][1]))
                    elif cmd == 20:
                        emit(node, cmd, struct.pack('<ff', 0, 0))
        now = time.monotonic()
        if now >= next_feedback:
            next_feedback = now + .02
            for node in axis:
                emit(node, 1, struct.pack('<IB3x', 0, axis[node][0]))
            for steer_id in range(1, 5):
                emulator.send(can.Message(arbitration_id=(41 << 8) | steer_id,
                    data=struct.pack('>hhhbb', 0, 0, 0, 20, 0), is_extended_id=True))


class VirtualSteer(SteerAk40):
    def connect(self):
        return  # The virtual socket was explicitly injected below.


corners = {}
for mapping in DEFAULT_WHEEL_MAP:
    drive_bus = FilteredVirtualBus(channel=channel,
        can_filters=[{'can_id': mapping.drive_node_id << 5, 'can_mask': 0x7E0, 'extended': False}])
    buses.append(drive_bus)
    drive = DriveOdriveCan(mapping.drive_node_id, bus=drive_bus)
    steer = NullSteer()
    if mapping.steer_can_id is not None:
        steer = VirtualSteer(mapping.steer_can_id)
        steer._bus = FilteredVirtualBus(channel=channel,
            can_filters=[{'can_id': (41 << 8) | mapping.steer_can_id, 'can_mask': 0xFFFF, 'extended': True}])
        buses.append(steer._bus)
        steer._ak = AK40(steer._bus, mapping.steer_can_id)
    corners[mapping.wheel] = CornerModule(steer, drive, CornerConfig())
cm = ChassisManager(corners)
worker = threading.Thread(target=serve)
worker.start()
try:
    cm.connect()
    time.sleep(.04)
    started = time.monotonic()
    assert cm.arm(), cm.state()
    arm_ms = (time.monotonic() - started) * 1000
    assert arm_ms < 150
    max_tick_ms = 0
    for _ in range(20):
        started = time.monotonic()
        cm.set(.5, 0)
        cm.tick()
        max_tick_ms = max(max_tick_ms, (time.monotonic() - started) * 1000)
        time.sleep(.02)
    moving = cm.hardware_stop_proof('can')
    assert moving['valid'] and not moving['stopped'], moving
    started = time.monotonic()
    cm.disarm()
    disarm_ms = (time.monotonic() - started) * 1000
    queries.clear()
    for _ in range(50):
        cm.tick()
        time.sleep(.02)
    stopped = cm.hardware_stop_proof('can')
    assert stopped['valid'] and stopped['stopped'], stopped
    idle_queries = dict(queries)
    assert sum(idle_queries.values()) <= 156, idle_queries
    assert cm.arm()
    drop_encoder.set()
    for _ in range(20):
        cm.set(.5, 0)
        cm.tick()
        time.sleep(.02)
        if cm.mode == 'ESTOP':
            break
    assert cm.mode == 'ESTOP'
    assert all(corner.mode == 'FAULT' for corner in corners.values())
    print(json.dumps({'transport':'python-can virtual, no real hardware', 'axes':10,
        'arm_ms':arm_ms, 'disarm_ms':disarm_ms, 'max_running_tick_ms':max_tick_ms,
        'idle_queries_one_second':idle_queries,'stopped_proof':stopped,
        'encoder_loss_result':cm.mode}, indent=2))
finally:
    cm.estop('runtime_cleanup')
    shutdown.set()
    worker.join(timeout=2)
    for bus in buses:
        bus.shutdown()
