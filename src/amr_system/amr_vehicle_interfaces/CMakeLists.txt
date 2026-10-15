cmake_minimum_required(VERSION 3.10)
project(amr_vehicle_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DataStatus.msg" "msg/MotorStatus.msg" "msg/VehicleStatus.msg"
  DEPENDENCIES builtin_interfaces std_msgs geometry_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
