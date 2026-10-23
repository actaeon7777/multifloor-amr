cmake_minimum_required(VERSION 3.8)
project(amr_elevator_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ElevatorBeacon.msg" "msg/ElevatorCommand.msg" "msg/ElevatorCommandStatus.msg"
  "msg/ElevatorDoorState.msg"
  "msg/FloorState.msg" "msg/FloorEvent.msg"
  "srv/SetCurrentFloor.srv"
  "action/NavigateToFloorPose.action"
  DEPENDENCIES std_msgs geometry_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
