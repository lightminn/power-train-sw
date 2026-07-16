"""WP8 five-section progress adapter using a temporary fake event contract.

Subscriptions:
  ``/section_events`` is ``std_msgs/String`` JSON used only as a fake bridge
  until the cross-team perception topics and vocabulary are confirmed.
  ``/detected_objects`` is converted to ``MARKER_DETECTED`` only for the
  MARKERS profile after the same timestamp/frame/``base_link`` TF gates used
  by ``lead_follower_node``.  ``/odom`` is auxiliary distance evidence only;
  ``/follow/active`` and ``/odom_diagnostics`` become FOLLOW/ICE progress
  events.

Output:
  ``/section/state`` contains hints and progress as JSON.  This node creates
  no chassis command and owns no command authority.

TODO after the cross-team contract is confirmed: an upper adapter may consume
``work_request`` and hold hints and invoke the existing chassis-owned
``MissionSupervisor``/authority gates.  This node deliberately does neither;
``MISSION_STOP`` unlock order, real recognition topics, and recovery actions
remain unconfirmed.
"""

import json
import math
import os
import sys
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformListener

from robot_arm_msgs.msg import DetectedObjectArray

sys.path.insert(
    0,
    os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
)

from chassis.section_profiles import (  # noqa: E402
    FOLLOW,
    ICE,
    LEAD_FOUND,
    LEAD_LOST,
    MARKERS,
    MARKER_DETECTED,
    SECTION_PROFILES,
    STUCK_DETECTED,
    SectionEvent,
    SectionSupervisor,
)
from powertrain_observability.client import EventClient  # noqa: E402


_TF_STALE_S = 0.5


def _stamp_s(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _apply_tf(position, transform):
    """Transform one xyz point with the same quaternion rule as WP7."""
    q = transform.transform.rotation
    px = float(position.x)
    py = float(position.y)
    pz = float(position.z)
    tx = 2.0 * (q.y * pz - q.z * py)
    ty = 2.0 * (q.z * px - q.x * pz)
    tz = 2.0 * (q.x * py - q.y * px)
    rx = px + q.w * tx + (q.y * tz - q.z * ty)
    ry = py + q.w * ty + (q.z * tx - q.x * tz)
    rz = pz + q.w * tz + (q.x * ty - q.y * tx)
    translation = transform.transform.translation
    return (
        rx + translation.x,
        ry + translation.y,
        rz + translation.z,
    )


class SectionSupervisorNode(Node):
    def __init__(self, *, event_client=None, **kwargs):
        super().__init__("section_supervisor", **kwargs)
        self.declare_parameter("section", "")
        self.declare_parameter("enabled", False)

        section = str(self.get_parameter("section").value).strip()
        if section not in SECTION_PROFILES:
            self.get_logger().fatal(
                "section parameter must be one of "
                "SMOG|RELIEF|MARKERS|ICE|FOLLOW"
            )
            self.destroy_node()
            raise ValueError(
                "invalid required section parameter: %r" % section
            )

        self.section = section
        self.enabled = bool(self.get_parameter("enabled").value)
        self.supervisor = SectionSupervisor(SECTION_PROFILES[section])
        self._event_client = (
            event_client if event_client is not None else EventClient()
        )
        self._previous_odom = None
        self._odom_distance_m = 0.0
        self._last_follow_active = None
        self._last_stuck = False

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)
        self.pub_state = self.create_publisher(String, "/section/state", 10)
        self.create_subscription(
            String,
            "/section_events",
            self._on_section_event,
            10,
        )
        self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self._on_detections,
            10,
        )
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(
            Bool,
            "/follow/active",
            self._on_follow_active,
            10,
        )
        self.create_subscription(
            String,
            "/odom_diagnostics",
            self._on_odom_diagnostics,
            10,
        )
        self.create_timer(0.1, self._tick)

        mode = "enabled" if self.enabled else "disabled (state only)"
        self.get_logger().info(
            "section_supervisor section=%s %s; fake event contract"
            % (self.section, mode)
        )

    def _now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_section_event(self, message):
        try:
            decoded = json.loads(message.data)
            if not isinstance(decoded, dict):
                raise ValueError("root must be a JSON object")
            payload = decoded.get("payload", {})
            if not isinstance(payload, dict):
                raise ValueError("payload must be a JSON object")
            event = SectionEvent(
                type=decoded["type"],
                stamp_s=decoded["stamp_s"],
                payload=payload,
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid fake /section_events JSON ignored: %s" % exc
            )
            return
        self.supervisor.submit(event)

    def _on_detections(self, message):
        if self.section != MARKERS:
            return
        frame_id = str(message.header.frame_id).strip()
        if not frame_id:
            self.get_logger().warning("marker frame_id empty; frame ignored")
            return

        stamp_s = _stamp_s(message.header.stamp)
        now_s = self._now_s()
        age_s = now_s - stamp_s
        if stamp_s <= 0.0 or age_s < 0.0 or age_s > _TF_STALE_S:
            self.get_logger().warning(
                "marker stamp invalid or stale; frame ignored"
            )
            return
        try:
            transform = self.tf_buf.lookup_transform(
                "base_link",
                frame_id,
                Time.from_msg(message.header.stamp),
                timeout=Duration(seconds=0.0),
            )
        except Exception:
            self.get_logger().warning(
                "marker TF unavailable (%s -> base_link); frame ignored"
                % frame_id
            )
            return

        for detected in message.objects:
            try:
                position = _apply_tf(detected.pose.position, transform)
                confidence = float(detected.confidence)
                class_id = int(detected.class_id)
                class_name = str(detected.class_name)
            except (TypeError, ValueError):
                continue
            # Fake bridge convention only: non-positive int32 means that the
            # arm team did not provide a stable unique marker ID.
            unique_id = class_id if class_id > 0 else None
            self.supervisor.submit(
                SectionEvent(
                    type=MARKER_DETECTED,
                    stamp_s=stamp_s,
                    payload={
                        "class_id": unique_id,
                        "class_name": class_name,
                        "position": position,
                        "confidence": confidence,
                    },
                )
            )

    def _on_odom(self, message):
        try:
            position = message.pose.pose.position
            current = (float(position.x), float(position.y))
            if not all(math.isfinite(value) for value in current):
                return
        except (AttributeError, TypeError, ValueError):
            return
        if self._previous_odom is not None:
            self._odom_distance_m += math.hypot(
                current[0] - self._previous_odom[0],
                current[1] - self._previous_odom[1],
            )
        self._previous_odom = current
        self.supervisor.update_odom(self._odom_distance_m)

    def _on_follow_active(self, message):
        if self.section != FOLLOW:
            return
        active = bool(message.data)
        if active == self._last_follow_active:
            return
        self._last_follow_active = active
        self.supervisor.submit(
            SectionEvent(
                type=LEAD_FOUND if active else LEAD_LOST,
                stamp_s=self._now_s(),
                payload={"source": "/follow/active"},
            )
        )

    def _on_odom_diagnostics(self, message):
        if self.section != ICE:
            return
        try:
            decoded = json.loads(message.data)
            stuck = bool(decoded["stuck_candidate"])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid /odom_diagnostics ignored: %s" % exc
            )
            return
        rising = stuck and not self._last_stuck
        self._last_stuck = stuck
        if rising:
            self.supervisor.submit(
                SectionEvent(
                    type=STUCK_DETECTED,
                    stamp_s=self._now_s(),
                    payload={"source": "/odom_diagnostics"},
                )
            )

    def _tick(self):
        state = self.supervisor.tick(self._now_s())
        self.pub_state.publish(
            String(
                data=json.dumps(
                    {
                        "section": state.section,
                        "phase": state.phase,
                        "hold_hint": state.drive_hold_hint,
                        "speed_hint": state.speed_hint,
                        "work_request": state.work_request,
                        "notices": list(state.notices),
                        "unique_markers": self.supervisor.unique_markers,
                        "complete": state.complete,
                        "enabled": self.enabled,
                    },
                    allow_nan=False,
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )
        )
        if not self.enabled:
            return
        for entry in state.journal_events:
            self._emit_journal(entry)

    def _emit_journal(self, entry):
        monotonic_ns = time.monotonic_ns()
        event = {
            "schema_version": 1,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": monotonic_ns,
            "source": "section_supervisor_node",
            "event_type": "SECTION_EVENT",
            "severity": "INFO" if entry.accepted else "WARN",
            "payload": {
                "section": self.section,
                "section_event_type": entry.event_type,
                "stamp_s": entry.stamp_s,
                "accepted": entry.accepted,
                "reason": entry.reason,
                "event_payload": dict(entry.payload),
            },
        }
        try:
            self._event_client.emit(event)
        except Exception:
            # Observability is best effort and cannot block section progress.
            pass


def main():
    rclpy.init()
    node = None
    try:
        node = SectionSupervisorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
