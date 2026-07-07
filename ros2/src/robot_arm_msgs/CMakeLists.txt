cmake_minimum_required(VERSION 3.8)
project(robot_arm_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjectArray.msg"
  "msg/ArrivalStatus.msg"
  "msg/ChassisMode.msg"
  "msg/ArmStatus.msg"
  DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

ament_package()
