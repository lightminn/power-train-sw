"""An ID registered after this tick's SyncRead must not crash the bridge."""

import threading
from types import SimpleNamespace

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge


def test_an_id_added_after_the_sync_read_reads_as_no_sample():
    def get_data(*_args):
        raise IndexError('list index out of range')

    bridge = SimpleNamespace(
        _bus_lock=threading.RLock(),
        group_sync_read=SimpleNamespace(
            data_dict={3: []}, isAvailable=lambda *_a: True, getData=get_data))
    assert MoveItDynamixelBridge._read_sample(bridge, 3) is None
