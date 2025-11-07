cmake_minimum_required(VERSION 3.5)
project(omnihand_pro_node_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

set(msg_files
  "msg/ControlMode.msg"
  "msg/CurrentPeriod.msg"
  "msg/CurrentReport.msg"
  "msg/CurrentThreshold.msg"
  "msg/ErrorPeriod.msg"
  "msg/MotorErrorReport.msg"
  "msg/MotorPos.msg"
  "msg/MotorVel.msg"
  "msg/TactileSensorData.msg"
  "msg/TactileSensor.msg"
  "msg/TemperaturePeriod.msg"
  "msg/TemperatureReport.msg"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES builtin_interfaces std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()