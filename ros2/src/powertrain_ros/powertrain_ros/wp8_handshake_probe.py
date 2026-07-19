"""WP8 handshake observer, stimulus driver, and contract-faithful fake arm."""

from __future__ import annotations

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time
from typing import Iterable

from powertrain_ros import contract
from powertrain_ros.wp8_scenario import (
    Event,
    FAULT_SCENARIOS,
    Finding,
    PICKUP_BRANCHES,
    judge_baseline,
    judge_fault,
    judge_full_cycle,
    judge_pickup_conjunction,
    judge_resume,
    pickup_branch,
    summarize,
)


DEFAULT_TIMEOUT_S = 8.0
DEFAULT_NODE_NS = "/chassis_node"
MARKER_TOPIC = "marker"
PICK_TARGET_TOPIC = "/pick_target"
OBSERVE_ONLY_ENV = "WP8_PROBE_OBSERVE_ONLY"


def _positive_float(value: str) -> float:
    parsed = float(value)
    if not math.isfinite(parsed) or parsed <= 0.0:
        raise argparse.ArgumentTypeError("must be a finite positive number")
    return parsed


def _add_common_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--timeout-s", type=_positive_float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--node-ns", default=DEFAULT_NODE_NS)
    parser.add_argument("--json", default=None)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="wp8_handshake_probe",
        description="WP8 chassis-to-arm handshake scenario probe",
    )
    subparsers = parser.add_subparsers(dest="subcommand", required=True)

    for name in ("baseline", "pickup", "full-cycle"):
        _add_common_arguments(subparsers.add_parser(name))

    resume = subparsers.add_parser("resume")
    _add_common_arguments(resume)
    resume.add_argument("--resume-t-file", required=True)

    fault = subparsers.add_parser("fault")
    _add_common_arguments(fault)
    fault.add_argument("--scenario", required=True, choices=sorted(FAULT_SCENARIOS))
    return parser


def parse_args(argv=None) -> argparse.Namespace:
    return build_parser().parse_args(argv)


def observe_only_enabled(environ=None) -> bool:
    environment = os.environ if environ is None else environ
    return environment.get(OBSERVE_ONLY_ENV) == "1"


def events_from_records(records: Iterable[tuple[float, str, object]]) -> list[Event]:
    """Convert receive-time/message records into the pure scenario format."""
    events = []
    for received_t, topic, message in records:
        received_t = float(received_t)
        if not math.isfinite(received_t):
            raise ValueError("record receive time must be finite")
        if topic == contract.TOPIC_CHASSIS_MODE:
            event = Event(received_t, "chassis_mode", str(message.mode))
        elif topic == contract.TOPIC_ARRIVAL:
            event = Event(
                received_t,
                "arrival",
                str(message.status),
                int(message.mission_id),
            )
        elif topic == contract.TOPIC_ARM_STATUS:
            event = Event(
                received_t,
                "arm_status",
                str(message.status),
                int(message.mission_id),
            )
        elif topic == MARKER_TOPIC:
            event = Event(received_t, "marker", str(message))
        else:
            raise ValueError("unsupported record topic: %s" % topic)
        events.append(event)
    return events


def serialize_summary(
    subcommand: str,
    findings: Iterable[Finding],
    *,
    branch: str | None,
) -> str:
    rows = list(findings)
    passed, _table = summarize(rows)
    payload = {
        "subcommand": str(subcommand),
        "pass": passed,
        "findings": [asdict(finding) for finding in rows],
    }
    if subcommand == "pickup":
        if branch not in PICKUP_BRANCHES:
            raise ValueError("pickup summary requires a valid branch")
        payload["branch"] = branch
    elif branch is not None:
        raise ValueError("branch is only valid for pickup summaries")
    return json.dumps(
        payload,
        ensure_ascii=False,
        separators=(",", ":"),
    )


def _service_name(node_ns: str, suffix: str) -> str:
    normalized = "/" + str(node_ns).strip("/")
    return normalized + "/" + suffix.lstrip("/")


def _read_resume_t(path: str) -> float:
    try:
        value = float(Path(path).read_text(encoding="utf-8").strip())
    except (OSError, ValueError) as exc:
        raise RuntimeError("resume timestamp read failed: %s" % exc) from exc
    if not math.isfinite(value):
        raise RuntimeError("resume timestamp must be finite")
    return value


def _judge(args: argparse.Namespace, events: list[Event]) -> list[Finding]:
    if args.subcommand == "baseline":
        return judge_baseline(events, window_s=args.timeout_s)
    if args.subcommand == "pickup":
        return judge_pickup_conjunction(events)
    if args.subcommand == "resume":
        return judge_resume(events, resume_t=_read_resume_t(args.resume_t_file))
    if args.subcommand == "full-cycle":
        return judge_full_cycle(events)
    return judge_fault(events, scenario=args.scenario)


def _runtime_classes(
    *,
    Node,
    QoSProfile,
    HistoryPolicy,
    ReliabilityPolicy,
    DurabilityPolicy,
    ArmStatus,
    ArrivalStatus,
    ChassisMode,
    Trigger,
):
    wire_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )

    class ProbeNode(Node):
        def __init__(self, node_ns: str):
            super().__init__("wp8_handshake_probe")
            self.records = []
            self.node_ns = node_ns
            self.create_subscription(
                ChassisMode,
                contract.TOPIC_CHASSIS_MODE,
                lambda message: self._record(contract.TOPIC_CHASSIS_MODE, message),
                wire_qos,
            )
            self.create_subscription(
                ArrivalStatus,
                contract.TOPIC_ARRIVAL,
                lambda message: self._record(contract.TOPIC_ARRIVAL, message),
                wire_qos,
            )
            self.create_subscription(
                ArmStatus,
                contract.TOPIC_ARM_STATUS,
                lambda message: self._record(contract.TOPIC_ARM_STATUS, message),
                wire_qos,
            )

        def _record(self, topic, message):
            self.records.append((time.monotonic(), topic, message))

        def events(self):
            return events_from_records(self.records)

        def trigger_client(self, suffix):
            return self.create_client(
                Trigger,
                _service_name(self.node_ns, suffix),
            )

    class FakeArmNode(Node):
        def __init__(self, scenario: str):
            super().__init__("wp8_handshake_fake_arm")
            self.scenario = scenario
            self._mode = None
            self._arrival = None
            self._active = None
            self._active_started_t = None
            self._completed_ids = set()
            self._current_status = contract.ARM_STOWED_LOCKED
            self._current_mission_id = 0
            self._work_ready_sent = False
            self._terminal_sent = False
            self._silent = False
            self._publisher = self.create_publisher(
                ArmStatus,
                contract.TOPIC_ARM_STATUS,
                wire_qos,
            )
            self.create_subscription(
                ChassisMode,
                contract.TOPIC_CHASSIS_MODE,
                self._on_mode,
                wire_qos,
            )
            self.create_subscription(
                ArrivalStatus,
                contract.TOPIC_ARRIVAL,
                self._on_arrival,
                wire_qos,
            )
            self.create_timer(0.1, self._tick)

        def _on_mode(self, message):
            self._mode = str(message.mode)
            self._maybe_start()

        def _on_arrival(self, message):
            self._arrival = (int(message.mission_id), str(message.status))
            self._maybe_start()

        def _maybe_start(self):
            if (
                self._active is not None
                or self._arrival is None
                or self._mode != contract.MODE_MISSION_STOP
                or self._arrival[0] in self._completed_ids
            ):
                return
            self._active = self._arrival
            self._active_started_t = time.monotonic()
            self._work_ready_sent = False
            self._terminal_sent = False

        def _completion_status(self):
            if self._active[1] == contract.ARRIVED_PICKUP:
                return contract.ARM_CARRYING_LOCKED
            return contract.ARM_STOWED_LOCKED

        def _publish(self, status, mission_id):
            message = ArmStatus()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = "base_link"
            message.mission_id = int(mission_id)
            message.status = str(status)
            self._publisher.publish(message)

        def _tick(self):
            if self._silent:
                return
            if self._active is None:
                self._publish(self._current_status, self._current_mission_id)
                return

            mission_id, _arrival_status = self._active
            elapsed_s = time.monotonic() - self._active_started_t
            if self.scenario == "no_response":
                self._publish(self._current_status, self._current_mission_id)
                return

            if elapsed_s >= 0.5 and not self._work_ready_sent:
                if self.scenario != "late_done":
                    self._current_status = contract.ARM_WORK_READY
                    self._current_mission_id = mission_id
                self._work_ready_sent = True

            if elapsed_s >= 1.0 and not self._terminal_sent:
                self._terminal_sent = True
                if self.scenario == "failed_latch":
                    self._publish(contract.ARM_FAILED, mission_id)
                    self._silent = True
                    return
                completion_id = (
                    mission_id - 1 if self.scenario == "late_done" else mission_id
                )
                self._current_status = self._completion_status()
                self._current_mission_id = completion_id
                if self.scenario != "late_done":
                    self._completed_ids.add(mission_id)
                    self._active = None

            self._publish(self._current_status, self._current_mission_id)

    return ProbeNode, FakeArmNode


def _spin_until(executor, predicate, *, deadline: float) -> bool:
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=min(0.05, max(0.0, deadline - time.monotonic())))
        if predicate():
            return True
    return bool(predicate())


def _call_trigger(probe, executor, suffix: str, *, deadline: float):
    client = probe.trigger_client(suffix)
    try:
        remaining = max(0.0, deadline - time.monotonic())
        if not client.wait_for_service(timeout_sec=min(2.0, remaining)):
            raise RuntimeError("service unavailable: %s" % client.srv_name)
        future = client.call_async(client.srv_type.Request())
        if not _spin_until(executor, future.done, deadline=deadline):
            raise RuntimeError("service timeout: %s" % client.srv_name)
        exception = future.exception()
        if exception is not None:
            raise RuntimeError("service failed: %s" % exception)
        return future.result()
    finally:
        probe.destroy_client(client)


def _call_until_success(probe, executor, suffix: str, *, deadline: float):
    last_message = "not called"
    while time.monotonic() < deadline:
        response = _call_trigger(probe, executor, suffix, deadline=deadline)
        last_message = str(response.message)
        if response.success:
            return response
        _spin_until(executor, lambda: False, deadline=min(deadline, time.monotonic() + 0.1))
    raise RuntimeError("service rejected: %s: %s" % (suffix, last_message))


def _pickup_has_resumed(events: list[Event]) -> bool:
    arrival = next(
        (
            event
            for event in events
            if event.topic == "arrival" and event.value == contract.ARRIVED_PICKUP
        ),
        None,
    )
    if arrival is None:
        return False
    completion = next(
        (
            event
            for event in events
            if event.t > arrival.t
            and event.topic == "arm_status"
            and event.value == contract.ARM_CARRYING_LOCKED
            and event.mission_id == arrival.mission_id
        ),
        None,
    )
    return completion is not None and any(
        event.t > completion.t
        and event.topic == "chassis_mode"
        and event.value in contract.LOCK_MODES
        for event in events
    )


def _run_ros(args, *, executor, probe, fake_arm_class=None) -> list[Event]:
    deadline = time.monotonic() + args.timeout_s

    if args.subcommand == "baseline":
        _spin_until(executor, lambda: False, deadline=deadline)
        return probe.events()

    if args.subcommand == "resume":
        _spin_until(executor, lambda: False, deadline=deadline)
        return probe.events()

    if args.subcommand == "pickup":
        if observe_only_enabled():
            _spin_until(executor, lambda: False, deadline=deadline)
            return probe.events()
        _spin_until(executor, lambda: bool(probe.events()), deadline=min(deadline, time.monotonic() + 0.5))
        arm_response = _call_trigger(probe, executor, "arm", deadline=deadline)
        arm_marker = (
            "arm_ok"
            if arm_response.success
            else "arm_rejected:%s" % arm_response.message
        )
        probe._record(MARKER_TOPIC, arm_marker)
        if arm_response.success:
            arrival_response = _call_trigger(
                probe,
                executor,
                "mission_arrive_pickup",
                deadline=deadline,
            )
            arrival_marker = (
                "arrival_ok"
                if arrival_response.success
                else "arrival_rejected:%s" % arrival_response.message
            )
            probe._record(MARKER_TOPIC, arrival_marker)
        _spin_until(executor, lambda: False, deadline=deadline)
        return probe.events()

    if probe.count_publishers(contract.TOPIC_ARM_STATUS) >= 1:
        raise RuntimeError("fake-arm guard: /arm_status already has a publisher")
    fake_arm = fake_arm_class(
        "full-cycle" if args.subcommand == "full-cycle" else args.scenario
    )
    executor.add_node(fake_arm)
    try:
        ready_deadline = min(deadline, time.monotonic() + 2.0)
        if not _spin_until(
            executor,
            lambda: any(
                event.topic == "arm_status"
                and event.value == contract.ARM_STOWED_LOCKED
                for event in probe.events()
            ),
            deadline=ready_deadline,
        ):
            raise RuntimeError("fake-arm heartbeat was not observed")
        arm_response = _call_trigger(probe, executor, "arm", deadline=deadline)
        if not arm_response.success:
            raise RuntimeError("arm service rejected: %s" % arm_response.message)
        _call_until_success(probe, executor, "mission_arrive_pickup", deadline=deadline)

        if args.subcommand == "full-cycle":
            if not _spin_until(
                executor,
                lambda: _pickup_has_resumed(probe.events()),
                deadline=deadline,
            ):
                return probe.events()
            _spin_until(
                executor,
                lambda: False,
                deadline=min(deadline, time.monotonic() + 0.2),
            )
            _call_until_success(
                probe,
                executor,
                "mission_arrive_drop",
                deadline=deadline,
            )
            _spin_until(
                executor,
                lambda: all(
                    finding.ok for finding in judge_full_cycle(probe.events())
                ),
                deadline=deadline,
            )
        else:
            _spin_until(
                executor,
                lambda: all(
                    finding.ok
                    for finding in judge_fault(
                        probe.events(),
                        scenario=args.scenario,
                    )
                ),
                deadline=deadline,
            )
        return probe.events()
    finally:
        executor.remove_node(fake_arm)
        fake_arm.destroy_node()


def _write_result(
    args,
    findings: list[Finding],
    *,
    branch: str | None,
) -> int:
    passed, table = summarize(findings)
    encoded = serialize_summary(args.subcommand, findings, branch=branch)
    print(table)
    print(encoded)
    if args.json is not None:
        path = Path(args.json)
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(encoded + "\n", encoding="utf-8")
    return 0 if passed else 1


def main(argv=None):
    args = parse_args(argv)
    try:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode
        from std_srvs.srv import Trigger
    except Exception as exc:
        print("실행 오류: ROS import 실패: %s" % exc)
        return 2

    rclpy.init(args=[])
    executor = SingleThreadedExecutor()
    probe = None
    try:
        ProbeNode, FakeArmNode = _runtime_classes(
            Node=Node,
            QoSProfile=QoSProfile,
            HistoryPolicy=HistoryPolicy,
            ReliabilityPolicy=ReliabilityPolicy,
            DurabilityPolicy=DurabilityPolicy,
            ArmStatus=ArmStatus,
            ArrivalStatus=ArrivalStatus,
            ChassisMode=ChassisMode,
            Trigger=Trigger,
        )
        probe = ProbeNode(args.node_ns)
        executor.add_node(probe)
        discovery_deadline = time.monotonic() + min(1.0, args.timeout_s / 4.0)
        _spin_until(executor, lambda: False, deadline=discovery_deadline)
        events = _run_ros(
            args,
            executor=executor,
            probe=probe,
            fake_arm_class=FakeArmNode,
        )
        branch = pickup_branch(events) if args.subcommand == "pickup" else None
        return _write_result(
            args,
            _judge(args, events),
            branch=branch,
        )
    except Exception as exc:
        print("실행 오류: %s" % exc)
        return 2
    finally:
        if probe is not None:
            executor.remove_node(probe)
            probe.destroy_node()
        executor.shutdown()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
