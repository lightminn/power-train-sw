"""Field stopgap: clear ODrive MOTOR_FAILED errors on idle drive axes.

node15 repeatedly trips CURRENT_LIMIT_VIOLATION (axis 0x40, motor 0x1000).
While the error stays latched the chassis stop proof is invalid, so disarm
stays PENDING and the operator cannot start driving again. This listener
watches CAN heartbeats of nodes 11-16 and sends Clear_Errors (0x18) only when

* the axis error has no bits outside ``--mask`` (default MOTOR_FAILED 0x40),
* the axis is IDLE (the chassis estop already ran; nothing is re-armed here),
* the patch3/4 CAN recovery flags are clear (ESTOP_REQUESTED 0x4000 and the
  recovery latch keep their own guarded resume path), and
* the per-node cooldown has passed.

Driving still resumes only through the operator's normal start. Before each
clear the motor error is requested (Get_Motor_Error 0x03) and logged.

With ``--motor-allow`` (demo-mode service) the clear is two-phase: query the
motor error first, and send Clear_Errors only after an allowed reply arrived
within ``QUERY_TIMEOUT_S`` and the next heartbeat still shows the same axis
error in IDLE. Clears per axis are limited to ``DEMO_MAX_CLEARS`` per
``DEMO_WINDOW_S``. Residual race: Clear_Errors has no expected-value guard in
firmware, so an error raised between that heartbeat and the clear is cleared
too (accepted for demo mode, logged on every clear).
"""
from __future__ import annotations

import argparse
import logging
import struct
import time

import can

_HEARTBEAT = 0x01
_GET_MOTOR_ERROR = 0x03
_CLEAR_ERRORS = 0x18
_AXIS_IDLE = 1
_MOTOR_FAILED = 0x40
_RECOVERY_MARKER = 0xA3
QUERY_TIMEOUT_S = 0.3
DEMO_MAX_CLEARS = 3
DEMO_WINDOW_S = 60.0

logger = logging.getLogger("odrive_auto_clear")


class AutoClear:
    """Decide which frames to send for one received frame (no I/O)."""

    def __init__(self, nodes=range(11, 17), mask: int = _MOTOR_FAILED,
                 cooldown_s: float = 1.0, motor_allow: int | None = None):
        self.nodes = frozenset(nodes)
        self.mask = int(mask)
        self.cooldown_s = float(cooldown_s)
        self.motor_allow = None if motor_allow is None else int(motor_allow)
        self.last_clear_s: dict[int, float] = {}
        self.clear_count: dict[int, int] = {}
        # Two-phase (motor_allow) state: one query bound to one heartbeat.
        self._pending: dict[int, dict] = {}
        self._clear_times: dict[int, list] = {}

    def on_frame(self, msg, now_s: float) -> list:
        if msg.is_extended_id or msg.is_error_frame or msg.is_remote_frame:
            return []
        node, cmd = msg.arbitration_id >> 5, msg.arbitration_id & 0x1F
        if node not in self.nodes:
            return []
        if cmd == _GET_MOTOR_ERROR and len(msg.data) >= 4:
            motor_error = int.from_bytes(bytes(msg.data[:8]), "little")
            logger.warning("node%d motor.error=0x%X", node, motor_error)
            pending = self._pending.get(node)
            if pending is not None and pending["motor"] is None:
                if now_s - pending["queried_s"] > QUERY_TIMEOUT_S:
                    self._pending.pop(node, None)
                elif motor_error & ~self.motor_allow:
                    logger.warning("node%d motor.error=0x%X outside allowlist 0x%X: not clearing",
                                   node, motor_error, self.motor_allow)
                    self._pending.pop(node, None)
                else:
                    pending["motor"] = motor_error
            return []
        if cmd != _HEARTBEAT or len(msg.data) < 5:
            return []
        axis_error = struct.unpack("<I", bytes(msg.data[:4]))[0]
        axis_state = msg.data[4]
        eligible = (axis_error != 0 and not axis_error & ~self.mask
                    and axis_state == _AXIS_IDLE
                    and not (len(msg.data) == 8 and msg.data[5] == _RECOVERY_MARKER
                             and msg.data[6]))
        if self.motor_allow is not None:
            return self._two_phase(node, axis_error, eligible, now_s)
        if not eligible:
            return []
        last = self.last_clear_s.get(node)
        if last is not None and now_s - last < self.cooldown_s:
            return []
        self.last_clear_s[node] = now_s
        self.clear_count[node] = self.clear_count.get(node, 0) + 1
        logger.warning("node%d axis.error=0x%X state=%d -> Clear_Errors (#%d)",
                       node, axis_error, axis_state, self.clear_count[node])
        base = node << 5
        return [
            can.Message(arbitration_id=base | _GET_MOTOR_ERROR, is_extended_id=False,
                        is_remote_frame=True, dlc=8),
            can.Message(arbitration_id=base | _CLEAR_ERRORS, is_extended_id=False,
                        data=bytes(8)),
        ]


    def _two_phase(self, node, axis_error, eligible, now_s) -> list:
        pending = self._pending.get(node)
        if pending is not None and (not eligible or pending["error"] != axis_error
                                    or now_s - pending["queried_s"] > QUERY_TIMEOUT_S * 2):
            self._pending.pop(node, None)
            pending = None
        if not eligible:
            return []
        base = node << 5
        if pending is None:
            last = self.last_clear_s.get(node)
            if last is not None and now_s - last < self.cooldown_s:
                return []
            self._pending[node] = {"error": axis_error, "queried_s": now_s, "motor": None}
            return [can.Message(arbitration_id=base | _GET_MOTOR_ERROR, is_extended_id=False,
                                is_remote_frame=True, dlc=8)]
        if pending["motor"] is None:
            return []
        self._pending.pop(node, None)
        recent = [t for t in self._clear_times.get(node, []) if now_s - t < DEMO_WINDOW_S]
        if len(recent) >= DEMO_MAX_CLEARS:
            self._clear_times[node] = recent
            logger.warning("node%d clear budget %d/%.0fs used: not clearing axis.error=0x%X",
                           node, DEMO_MAX_CLEARS, DEMO_WINDOW_S, axis_error)
            return []
        recent.append(now_s)
        self._clear_times[node] = recent
        self.last_clear_s[node] = now_s
        self.clear_count[node] = self.clear_count.get(node, 0) + 1
        logger.warning("node%d axis.error=0x%X motor.error=0x%X IDLE -> Clear_Errors (#%d)",
                       node, axis_error, pending["motor"], self.clear_count[node])
        return [can.Message(arbitration_id=base | _CLEAR_ERRORS, is_extended_id=False,
                            data=bytes(8))]


def run(channel: str, policy: AutoClear, reopen_s: float = 2.0) -> None:
    filters = [{"can_id": node << 5, "can_mask": 0x7E0, "extended": False}
               for node in sorted(policy.nodes)]
    while True:
        try:
            with can.Bus(channel=channel, interface="socketcan", can_filters=filters) as bus:
                logger.info("listening on %s nodes=%s mask=0x%X",
                            channel, sorted(policy.nodes), policy.mask)
                while True:
                    msg = bus.recv(timeout=1.0)
                    if msg is None:
                        continue
                    for out in policy.on_frame(msg, time.monotonic()):
                        try:
                            bus.send(out, timeout=0.05)
                        except can.CanError as exc:
                            logger.warning("send 0x%03X failed: %s", out.arbitration_id, exc)
        except (OSError, can.CanError) as exc:
            logger.warning("%s unavailable (%s); retry in %.0fs", channel, exc, reopen_s)
            time.sleep(reopen_s)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--channel", default="can0")
    parser.add_argument("--nodes", default="11,12,13,14,15,16")
    parser.add_argument("--mask", type=lambda text: int(text, 0), default=_MOTOR_FAILED,
                        help="axis error bits allowed to be cleared (default 0x40)")
    parser.add_argument("--cooldown-s", type=float, default=1.0)
    parser.add_argument("--motor-allow", type=lambda text: int(text, 0), default=None,
                        help="demo mode: query motor.error first and clear only if it has "
                             "no bits outside this mask (e.g. 0x1000 CURRENT_LIMIT_VIOLATION)")
    args = parser.parse_args(argv)
    if args.mask & 0x4000:
        parser.error("--mask must not include ESTOP_REQUESTED 0x4000")
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    nodes = [int(part) for part in args.nodes.split(",") if part]
    try:
        run(args.channel, AutoClear(nodes, args.mask, args.cooldown_s, args.motor_allow))
    except KeyboardInterrupt:
        logging.info("stopped")


if __name__ == "__main__":
    main()
