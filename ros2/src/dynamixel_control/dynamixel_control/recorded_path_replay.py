#!/usr/bin/env python3
"""fold_path.json에서 보호된 역방향 경로를 만들고 선택적으로 실행한다."""

import argparse
import json
import math


ARM_IDS = (14, 13, 12)
NOISE_TICKS = 2
MAX_WAYPOINT_STEP = 50


def significant_reversals(values, expected_sign):
    """잘못된 방향에서 인코더 노이즈보다 큰 샘플 변화량을 센다."""
    deltas = [b - a for a, b in zip(values, values[1:])]
    return sum(delta * expected_sign < -NOISE_TICKS for delta in deltas)


def interpolate_signed(start, end, max_step=MAX_WAYPOINT_STEP):
    """max_step보다 큰 구간 없이 signed 끝점을 반환한다."""
    distance = end - start
    if distance == 0:
        return [start]
    steps = int(math.ceil(abs(distance) / max_step))
    # 정수 보간으로 끝점과 단조로운 signed 방향을 보존한다.
    values = [start + int(round(distance * index / steps))
              for index in range(steps + 1)]
    return [value for index, value in enumerate(values)
            if index == 0 or value != values[index - 1]]


def build_reverse_paths(payload):
    """수동 접기 기록을 검증하고 14/13/12 역방향 경로를 반환한다."""
    samples = payload.get("samples") or []
    if len(samples) < 2 or payload.get("error"):
        raise ValueError("fold recording is empty or contains a communication error")

    id14 = [int(row["id14"]) for row in samples]
    id13_all = [int(row["id13"]) for row in samples]
    id12 = [int(row["id12"]) for row in samples]
    id13_minimum = min(id13_all)
    id13_minimum_index = id13_all.index(id13_minimum)
    id13 = id13_all[:id13_minimum_index + 1]

    fold_paths = {14: id14, 13: id13, 12: id12}
    expected_fold_sign = {14: 1, 13: -1, 12: 1}
    for dxl_id, values in fold_paths.items():
        reversals = significant_reversals(
            values, expected_fold_sign[dxl_id])
        if reversals:
            raise ValueError(
                f"ID {dxl_id} fold path has {reversals} significant reversals")
    if any(not 0 <= value <= 4095 for value in id12):
        raise ValueError("ID 12 fold path leaves Position Mode range [0, 4095]")

    reverse = {
        dxl_id: interpolate_signed(values[-1], values[0])
        for dxl_id, values in fold_paths.items()
    }
    return reverse, {
        "id13_minimum": id13_minimum,
        "id13_minimum_index": id13_minimum_index,
        "id13_excluded_rebound": id13_all[-1] - id13_minimum,
    }


def flatten_paths(paths):
    """액션의 고정 ID 순서로 경로를 평탄화한다."""
    counts = [len(paths[dxl_id]) for dxl_id in ARM_IDS]
    flat = [value for dxl_id in ARM_IDS for value in paths[dxl_id]]
    return counts, flat


def arm_result_allows_rotation(result):
    """안전 측으로 실패하며, 팔 전체 성공 후에만 엔드이펙터를 실행한다."""
    return bool(result is not None and result.success)


def parse_args(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--path", default="/tmp/fold_path.json")
    parser.add_argument("--max-abs-current", type=int, default=300)
    parser.add_argument("--stall-timeout", type=float, default=2.0)
    parser.add_argument("--step-timeout", type=float, default=10.0)
    parser.add_argument("--goal-tolerance", type=int, default=10)
    parser.add_argument("--rotate-ticks", type=int, default=300)
    parser.add_argument("--execute", action="store_true")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    with open(args.path, "r", encoding="utf-8") as stream:
        payload = json.load(stream)
    paths, metadata = build_reverse_paths(payload)
    counts, flat = flatten_paths(paths)
    print(json.dumps({
        "motor_ids": list(ARM_IDS),
        "waypoint_counts": counts,
        "signed_waypoints": flat,
        "metadata": metadata,
    }))
    if not args.execute:
        print("DRY_RUN: no action goals sent")
        return

    import rclpy
    from rclpy.action import ActionClient
    from robot_arm_msgs.action import ArmRecordedPath, EndEffectorRotate

    rclpy.init(args=None)
    node = rclpy.create_node("recorded_path_replay")
    arm_client = ActionClient(node, ArmRecordedPath, "/arm/recorded_path")
    rotate_client = ActionClient(
        node, EndEffectorRotate, "/end_effector/rotate")
    try:
        if not arm_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/arm/recorded_path action server unavailable")
        goal = ArmRecordedPath.Goal()
        goal.motor_ids = list(ARM_IDS)
        goal.waypoint_counts = counts
        goal.signed_waypoints = flat
        goal.max_abs_current = args.max_abs_current
        goal.stall_timeout = args.stall_timeout
        goal.step_timeout = args.step_timeout
        goal.goal_tolerance = args.goal_tolerance

        def arm_feedback(message):
            feedback = message.feedback
            print(
                f"ARM id={feedback.motor_id} index={feedback.waypoint_index} "
                f"goal={feedback.goal_position} present={feedback.present_position} "
                f"velocity={feedback.present_velocity} "
                f"current={feedback.present_current} "
                f"error={feedback.hardware_error}")

        send_future = arm_client.send_goal_async(
            goal, feedback_callback=arm_feedback)
        rclpy.spin_until_future_complete(node, send_future)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError("recorded path goal rejected")
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(node, result_future)
        arm_result = result_future.result().result
        print(
            f"ARM_RESULT success={arm_result.success} "
            f"completed={arm_result.completed_waypoints} "
            f"reason={arm_result.reason}")
        if not arm_result_allows_rotation(arm_result):
            return

        if not rotate_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError("/end_effector/rotate action server unavailable")
        rotate = EndEffectorRotate.Goal()
        rotate.relative = True
        rotate.ticks = args.rotate_ticks
        rotate.max_abs_current = 100
        rotate.timeout = 10.0
        rotate_send = rotate_client.send_goal_async(rotate)
        rclpy.spin_until_future_complete(node, rotate_send)
        rotate_handle = rotate_send.result()
        if rotate_handle is None or not rotate_handle.accepted:
            raise RuntimeError("end-effector rotate goal rejected")
        rotate_result_future = rotate_handle.get_result_async()
        rclpy.spin_until_future_complete(node, rotate_result_future)
        rotate_result = rotate_result_future.result().result
        print(
            f"ROTATE_RESULT success={rotate_result.success} "
            f"delta={rotate_result.actual_delta} reason={rotate_result.reason}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
