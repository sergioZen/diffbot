cmake_minimum_required(VERSION 3.8)
project(diffbot_msgs)

# Recommended for ROS 2
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Add only the packages whose types your .msg use
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# List all your .msg files here
set(msg_files
  msg/WheelsCmd.msg
  msg/WheelsCmdStamped.msg
  msg/Encoders.msg
  msg/EncodersStamped.msg
  msg/AngularVelocities.msg
  msg/AngularVelocitiesStamped.msg
  msg/PID.msg
  msg/PIDStamped.msg
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()