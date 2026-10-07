from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_rsp_launch


def _launch_setup(context):
    preset = LaunchConfiguration("end_effector_preset").perform(context)
    if preset not in (
            "dual_motor_gripper", "single_motor_gripper", "rotary_id5"):
        raise RuntimeError(f"unsupported RSP end_effector_preset {preset!r}")
    model_preset = (
        preset if preset == "single_motor_gripper" else "dual_motor_gripper")
    builder = MoveItConfigsBuilder(
        "robot_arm", package_name="robot_arm_moveit_config")
    builder.robot_description(
        file_path="config/robot_arm.urdf.xacro",
        mappings={"end_effector_preset": model_preset})
    return list(generate_rsp_launch(builder.to_moveit_configs()).entities)


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "end_effector_preset", default_value="dual_motor_gripper"),
        OpaqueFunction(function=_launch_setup),
    ])
