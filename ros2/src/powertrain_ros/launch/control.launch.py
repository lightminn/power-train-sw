"""powertrain_control 서비스의 PID 1 — teleop_command + ops_broker (D5).

두 노드 중 하나라도 죽으면 런치 전체를 내린다(`on_exit=Shutdown()`).
그러면 PID 1 이 종료되고 compose 의 `restart: unless-stopped` 가 컨테이너를
다시 세워 오버레이를 재빌드·재소싱한다.

이유(2026-07-28 실기): 기동 시점의 install space 가 stale 해
`ops_broker` 가 `ModuleNotFoundError: powertrain_msgs` 로 즉사했는데,
`teleop_command` 는 그 메시지를 import 하지 않아 살아남았다.  런치가 계속
돌았으므로 컨테이너는 죽지 않았고, ops 채널 :9001 만 없는 채로 약 1시간을
버텼다.  그동안 운용 콘솔의 조작·비상정지는 전부 불통이었고 조용했다.
healthcheck 는 unhealthy 를 표시했지만 compose 는 unhealthy 로 재시작하지
않는다 — 그래서 자기 종료가 필요하다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("input_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("input_port", default_value="9000"),
        DeclareLaunchArgument(
            "manual_command_format", default_value="steering", choices=["steering", "twist"],
            description="Manual steering command by default; twist selects the legacy yaw-rate topic",
        ),
        DeclareLaunchArgument("ops_host", default_value="0.0.0.0"),
        DeclareLaunchArgument("ops_port", default_value="9001"),
        DeclareLaunchArgument(
            "ops_token_dir",
            default_value="/etc/powertrain",
        ),
        Node(
            package="powertrain_ros",
            executable="teleop_command",
            name="teleop_command",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("input_host"),
                "manual_command_format": LaunchConfiguration("manual_command_format"),
                "port": ParameterValue(
                    LaunchConfiguration("input_port"),
                    value_type=int,
                ),
            }],
            on_exit=Shutdown(),
        ),
        Node(
            package="powertrain_ros",
            executable="ops_broker",
            name="ops_broker",
            output="screen",
            parameters=[{
                "host": LaunchConfiguration("ops_host"),
                "port": ParameterValue(
                    LaunchConfiguration("ops_port"),
                    value_type=int,
                ),
                "token_dir": LaunchConfiguration("ops_token_dir"),
            }],
            on_exit=Shutdown(),
        ),
    ])
