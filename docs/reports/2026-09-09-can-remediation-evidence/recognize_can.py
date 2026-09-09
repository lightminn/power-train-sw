"""Receive-only 10-motor inventory. No CAN transmit API is called."""
import argparse
from collections import Counter
from datetime import datetime, timezone
import json
import pathlib
import struct
import subprocess
import time

import can

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--seconds', type=float, default=6.)
parser.add_argument('--output', required=True)
args = parser.parse_args()
assert 0 < args.seconds <= 30

def link():
    return json.loads(subprocess.check_output(['ip', '-details', '-statistics', '-json', 'link', 'show', 'can0']))[0]

result = dict(started_at=datetime.now(timezone.utc).isoformat(), interface='can0',
              method='receive_only_external_frames', before=link())
counts, states = Counter(), {}
feedback_counts, encoders = Counter(), {}
bus = can.Bus(interface='socketcan', channel='can0', receive_own_messages=False)
end = time.monotonic() + args.seconds
try:
    while time.monotonic() < end:
        frame = bus.recv(min(.1, max(0., end - time.monotonic())))
        if frame is None or frame.is_error_frame or frame.is_remote_frame or not frame.is_rx:
            continue
        if (not frame.is_extended_id and frame.arbitration_id >> 5 in range(11, 17)
                and frame.arbitration_id & 31 in (9, 20) and len(frame.data) == 8):
            node, command = frame.arbitration_id >> 5, frame.arbitration_id & 31
            feedback_counts[f'{node}:{command}'] += 1
            if command == 9:
                position, velocity = struct.unpack('<ff', frame.data)
                encoders[str(node)] = dict(motor_position_turns=position, motor_velocity_turns_s=velocity)
            continue
        if frame.is_extended_id and frame.arbitration_id >> 8 == 41:
            node = frame.arbitration_id & 255
            if node not in range(1, 5) or len(frame.data) != 8:
                continue
            pos, speed, current, temperature, fault = struct.unpack('>hhhBB', frame.data)
            key = f'AK{node}'
            states[key] = dict(position_deg=pos/10, speed_raw=speed, current_raw=current,
                               temperature_c=temperature, fault=fault)
        elif not frame.is_extended_id and frame.arbitration_id & 31 == 1:
            node = frame.arbitration_id >> 5
            if node not in range(11, 17) or len(frame.data) < 5:
                continue
            key = f'ODrive{node}'
            states[key] = dict(axis_error=struct.unpack('<I', frame.data[:4])[0], axis_state=frame.data[4])
        else:
            continue
        counts[key] += 1
finally:
    bus.shutdown()
expected = [f'AK{n}' for n in range(1, 5)] + [f'ODrive{n}' for n in range(11, 17)]
result.update(feedback_counts=dict(feedback_counts), encoders=encoders, counts=dict(counts), states=states, missing=[key for key in expected if not counts[key]],
              after=link(), seconds=args.seconds)
result['recognized_all_ten'] = not result['missing']
result['all_motor_errors_zero'] = all(s.get('axis_error', s.get('fault')) == 0 for s in states.values())
pathlib.Path(args.output).write_text(json.dumps(result, indent=2))
print(json.dumps({key: result[key] for key in ('recognized_all_ten', 'all_motor_errors_zero', 'missing', 'counts', 'states')}, indent=2))
raise SystemExit(0 if result['recognized_all_ten'] and result['all_motor_errors_zero'] else 1)
