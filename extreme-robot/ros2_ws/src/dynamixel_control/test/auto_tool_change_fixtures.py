"""실측값이 아닌 검증 전용 데이터. 생산 config에는 설치하지 않는다."""

from copy import deepcopy


def dock_fixture():
    names = [f'arm_joint_{i}' for i in range(1, 6)]
    pose = dict(positions=[0.0] * len(names), tolerance_rad=0.01,
                timeout_s=2.0, captured_at=1700000000.0)
    return dict(version=1, joint_names=names, exchange=dict(
        feedback_timeout_s=0.5, home_timeout_s=2.0, home_tolerance_rad=0.01,
        unlock_direction=-1, lock_direction=1), slots={
        str(i): dict(tool_type=('spur_1motor_gripper', 'dual_motor_gripper', 'cleaner')[i - 1],
                     poses={key: deepcopy(pose) for key in ('approach', 'seat', 'retreat')})
        for i in range(1, 4)})
