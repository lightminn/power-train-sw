"""Installed ``teleop_command`` over real TCP: stall recovery + E-stop ack smoke.

Start the node in field mode first, e.g. (ROS sourced, isolated domain)::

    ROS_DOMAIN_ID=77 ros2 run powertrain_ros teleop_command --ros-args \
        -p port:=19500 -p input_timeout_s:=0.5 \
        -p latch_stale_input:=false -p partial_close_violation:=false

then run ``python3 scripts/teleop_stall_smoke.py 19500``. ``--legacy`` is the
negative control for pre-fix nodes (no ``G`` lines): it must FAIL at the
"late pre-stall/schema-2 forward" stage. Exit 0 = PASS.
"""
from pathlib import Path
import socket
import sys
import time
import uuid

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from motor_control.laptop.remote_operation_client import ClientInput, encode_frame  # noqa: E402

legacy = "--legacy" in sys.argv
port = int(sys.argv[1])
sock = socket.create_connection(("127.0.0.1", port), timeout=5)
sock.setblocking(False)
session = str(uuid.uuid4())
seq = 0
buf = b""
last = {"S": None, "G": None}
stage_speeds = []  # every S linear seen since the current stage began

def pump(seconds):
    global buf
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        try:
            chunk = sock.recv(4096)
            if chunk == b"":
                raise SystemExit("FAIL: server closed")
            buf += chunk
        except BlockingIOError:
            time.sleep(0.005)
        *lines, buf = buf.split(b"\n")
        for line in lines:
            f = line.decode().split()
            if f and f[0] in last:
                last[f[0]] = f
                if f[0] == "S":
                    stage_speeds.append(abs(float(f[2])))

def send(sample, epoch=None, estop_id=0):
    global seq
    sock.sendall(encode_frame(sample, session_id=session, sequence=seq,
                              client_monotonic_ns=time.monotonic_ns(),
                              input_epoch=epoch, estop_id=estop_id))
    seq += 1

def stream(sample, seconds, epoch=None):
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        send(sample, epoch)
        pump(0.02)

def linear():
    return float(last["S"][2]) if last["S"] else None


def stage():
    stage_speeds.clear()


def never_moved():
    """No nonzero speed in any status line of this stage (not just the last)."""
    return bool(stage_speeds) and max(stage_speeds) == 0.0

def check(cond, label):
    print(("ok   " if cond else "FAIL ") + label, last["S"], last["G"], flush=True)
    if not cond:
        raise SystemExit(1)

neutral = ClientInput()
forward = ClientInput(deadman=True, right_trigger=0.7)
stream(neutral, 0.3)
if not legacy:
    check(last["G"] is not None, "server sends G line")
    e0 = int(last["G"][1])
else:
    e0 = None
stream(forward, 0.4, e0)
check(linear() and linear() > 0.5, "drives with current input")
pump(0.9)  # stall: nothing sent for longer than input_timeout_s
check(linear() == 0.0, "stall stops the wheels")
check(last["S"][1] != "MOTION_HOLD", "field mode does not latch on stall")
# A forward record from before the stall arrives late (schema 2, no proof).
stage()
stream(forward, 0.3, None)
check(never_moved(), "late pre-stall/schema-2 forward does not restart driving")
if legacy:
    print("PASS (unexpected for legacy)"); raise SystemExit(0)
check(int(last["G"][1]) > e0, "server issued a new epoch")
stage()
stream(forward, 0.3, e0)
check(never_moved(), "pre-stall epoch forward does not restart driving")
stream(forward, 0.3, int(last["G"][1]))
check(linear() > 0.5, "post-stall epoch input resumes driving")
send(ClientInput(estop_edge=True), int(last["G"][1]), 424242)
pump(1.0)  # status cache refreshes on the ROS tick; periodic replies ~0.4 s
check(last["G"][2] == "424242", "E-stop acknowledged by id")
check(last["S"][1] == "MOTION_HOLD", "E-stop latches MOTION_HOLD")
print("PASS")
