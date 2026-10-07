from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch


def _launch_setup(context):
    preset = LaunchConfiguration("end_effector_preset").perform(context)
    if preset not in (
            "dual_motor_gripper", "single_motor_gripper", "rotary_id5"):
        raise RuntimeError(f"unsupported MoveIt end_effector_preset {preset!r}")
    single = preset == "single_motor_gripper"
    model_preset = preset if single else "dual_motor_gripper"
    builder = MoveItConfigsBuilder(
        "robot_arm", package_name="robot_arm_moveit_config")
    builder.robot_description(
        file_path="config/robot_arm.urdf.xacro",
        mappings={"end_effector_preset": model_preset})
    builder.robot_description_semantic(
        file_path="config/robot_arm.srdf.xacro",
        mappings={"end_effector_preset": model_preset})
    builder.joint_limits(file_path=(
        "config/joint_limits.single_motor_gripper.yaml" if single
        else "config/joint_limits.yaml"))
    builder.trajectory_execution(file_path=(
        "config/moveit_controllers.single_motor_gripper.yaml" if single
        else "config/moveit_controllers.yaml"))
    return list(generate_move_group_launch(builder.to_moveit_configs()).entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "end_effector_preset", default_value="dual_motor_gripper"),
        OpaqueFunction(function=_launch_setup),
    ])
