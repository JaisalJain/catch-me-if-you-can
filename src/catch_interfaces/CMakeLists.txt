cmake_minimum_required(VERSION 3.16)
project(catch_interfaces)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ThrowPlan.msg" "msg/CubeDetection.msg" "msg/CubeDetectionArray.msg"
  "srv/SpawnCube.srv" "srv/ResetScene.srv" "srv/GetColor.srv" "action/Throw.action"
  DEPENDENCIES std_msgs geometry_msgs builtin_interfaces)
ament_package()
