from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stop_mm",
            description="HIL-approved US-100 emergency-stop distance (mm)",
        ),
        Node(
            package="powertrain_ros",
            executable="us100_safety",
            output="screen",
            parameters=[{
                "stop_mm": ParameterValue(
                    LaunchConfiguration("stop_mm"),
                    value_type=float,
                ),
            }],
        ),
        GroupAction(actions=[
            DeclareLaunchArgument(
                "contract_v2_verified",
                default_value="false",
                choices=["true", "false"],
                description=(
                    "Enable contract-v2 chassis intent only after joint "
                    "arm/chassis verification"
                ),
            ),
            DeclareLaunchArgument(
                "arm_gate_mode",
                default_value="production",
                choices=["production", "arm_absent_field"],
                description=(
                    "production requires ArmStatus; arm_absent_field is an "
                    "operator-confirmed powertrain-only field profile"
                ),
            ),
            DeclareLaunchArgument(
                "arm_override_ttl_s",
                default_value="30.0",
                description=(
                    "Maximum REMOTE_ARM_OVERRIDE drive-permission lifetime "
                    "before explicit service reactivation is required"
                ),
            ),
            SetParameter(
                name="contract_v2_verified",
                value=ParameterValue(
                    LaunchConfiguration("contract_v2_verified"),
                    value_type=bool,
                ),
            ),
            SetParameter(
                name="arm_gate_mode",
                value=LaunchConfiguration("arm_gate_mode"),
            ),
            SetParameter(
                name="arm_override_ttl_s",
                value=ParameterValue(
                    LaunchConfiguration("arm_override_ttl_s"),
                    value_type=float,
                ),
            ),
            Node(
                package="powertrain_ros",
                executable="chassis",
                output="screen",
            ),
        ]),
    ])
