from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg=FindPackageShare('jetin_rover_description')
    model=PathJoinSubstitution([pkg,'urdf','jetin_rover.xacro'])
    rviz=PathJoinSubstitution([pkg,'config','display.rviz'])
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        DeclareLaunchArgument('use_nominal_sensor_extrinsics',default_value='true'),
        Node(package='robot_state_publisher',executable='robot_state_publisher',
             parameters=[{'robot_description':ParameterValue(Command(['xacro ',model,' use_nominal_sensor_extrinsics:=',LaunchConfiguration('use_nominal_sensor_extrinsics')]),value_type=str),
                          'use_sim_time':LaunchConfiguration('use_sim_time')}]),
        Node(package='joint_state_publisher_gui',executable='joint_state_publisher_gui'),
        Node(package='rviz2',executable='rviz2',arguments=['-d',rviz]),
    ])
