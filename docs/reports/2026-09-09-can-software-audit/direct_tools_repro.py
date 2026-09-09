"""Host-only GUI/setup probes: injected buses and fake ODrive board only.

Assertions preserve observed defects, not desired behavior. Never connects USB
or SocketCAN and never writes real NVM.
"""
import collections
import contextlib
import io
import json
import struct
import threading
from types import SimpleNamespace
import can
from motor_gui.backend.worker import HardwareWorker
from motor_gui.backend.transport.can_device import CanTransport
from motor_gui.backend.transport.odrive_can_device import OdriveCanDevice
from motor_gui.backend.transport.can_bus import CanBackend, AK
from drive.bl70200 import bl70200_setup as setup
from drive.bl70200.tests.test_bl70200_setup import _board

class FakeBus:
    def __init__(self, packets=()):
        self.packets = collections.deque(packets)
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)
    def recv(self, timeout=None):
        return self.packets.popleft() if self.packets else None
    def shutdown(self):
        pass

bus = FakeBus()
transport = CanTransport([OdriveCanDevice(node_id=11)], bus=bus)
transport.connect()
worker = HardwareWorker(transport)
worker._set_armed('odrive', True)
done, result = threading.Event(), {}
worker._reconnect_q.put((done, result, {'odrive': 13}))
worker._handle_reconnect()
idle_nodes = [msg.arbitration_id >> 5 for msg in bus.sent
              if msg.arbitration_id & 0x1f == 7 and struct.unpack('<I', msg.data)[0] == 1]
assert result['result']['ok'] is True
assert idle_nodes == [13, 13] and 11 not in idle_nodes
print(json.dumps({'case': 'gui_id_switch_11_to_13', 'idle_targets': idle_nodes,
                  'old_node_11_stopped': 11 in idle_nodes}))

ak_msg = can.Message(arbitration_id=(41 << 8) | 1,
                     data=struct.pack('>hhhbb', 123, 0, 0, 25, 0), is_extended_id=True)
bus = FakeBus([ak_msg, None, None])
backend = CanBackend()
backend._bus = bus
backend._ak = AK(bus, 1)
sample = backend.sample()
assert sample['ak.pos_deg'] == 0 and not bus.packets
print(json.dumps({'case': 'combined_can_drops_ak_frame', 'injected_ak_deg': 12.3,
                  'displayed_ak_deg': sample['ak.pos_deg']}))

od_msg = can.Message(arbitration_id=(11 << 5) | 9,
                     data=struct.pack('<ff', 7.5, 10.0), is_extended_id=False)
bus = FakeBus([None, od_msg, None])
backend = CanBackend()
backend._bus = bus
backend._ak = AK(bus, 1)
sample = backend.sample()
assert sample['odrive.pos'] == 0 and not bus.packets
print(json.dumps({'case': 'combined_can_drops_odrive_frame', 'injected_odrive_pos': 7.5,
                  'displayed_odrive_pos': sample['odrive.pos']}))

board = _board()
board.axis0.config.can_node_id = 13
board.axis1.config.can_node_id = 14
find_calls, saved = [], []
board.save_configuration = lambda: saved.append((board.axis0.config.can_node_id,
                                                board.axis1.config.can_node_id))
module = SimpleNamespace(find_any=lambda **kwargs: (find_calls.append(kwargs), board)[1])
setup._load_enums = lambda: SimpleNamespace(MOTOR_TYPE_HIGH_CURRENT=0, ENCODER_MODE_HALL=1,
                                           CONTROL_MODE_VELOCITY_CONTROL=2, INPUT_MODE_VEL_RAMP=2)
with contextlib.redirect_stdout(io.StringIO()):
    setup.run(setup.parse_args(['--apply']), odrive_module=module, sleep_fn=lambda _: None)
assert saved == [(13, 11)]
assert all('serial_number' not in call for call in find_calls)
print(json.dumps({'case': 'setup_default_apply', 'original_node_pair': [13, 14],
                  'saved_node_pairs': saved, 'usb_find_calls': find_calls}))
from motor_gui.backend.transport.ak_device import AkDevice
class FailingBus(FakeBus):
    def send(self, msg, timeout=None):
        self.sent.append(msg)
        raise can.CanOperationError('synthetic TX queue failure')
bus = FailingBus()
ak_device = AkDevice(motor_id=1)
ak_device.attach(bus)
ak_device.apply(bus, 'arm', {})
ack = ak_device.apply(bus, 'set_input', {'pos_deg': 10.0})
assert ack['ok'] is True and len(bus.sent) == 1
print(json.dumps({'case': 'ak_gui_success_ack_on_tx_failure', 'send_attempts': len(bus.sent), 'ack': ack}))

bus = FakeBus()
backend = CanBackend()
backend._bus = bus
backend._ak = AK(bus, 1)
for _ in range(100):
    backend.sample()
assert len(bus.sent) == 300
print(json.dumps({'case': 'combined_can_poll_budget', 'sample_calls': 100,
                  'rtr_requests': len(bus.sent), 'explicit_poll_throttle': False}))

# fw-v0.5.6 CANSimple 0x15 is sensorless estimates (position, velocity).
device = OdriveCanDevice(node_id=11)
device.on_rx(can.Message(arbitration_id=(11 << 5) | 0x15,
                        is_extended_id=False, data=struct.pack('<ff', 123.5, 4.0)))
assert device.sample()['odrive.temp_fet'] == 123.5
print(json.dumps({'case': 'sensorless_estimate_mislabeled_temperature',
                  'sensorless_position': 123.5,
                  'displayed_fet_temperature': device.sample()['odrive.temp_fet']}))
