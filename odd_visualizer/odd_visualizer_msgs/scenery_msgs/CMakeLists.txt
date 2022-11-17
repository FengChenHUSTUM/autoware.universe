# All rights reserved.
cmake_minimum_required(VERSION 3.14)

### Export headers
project(scenery_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED
geometry_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/laneSequenceWithID.idl"
  "msg/speedLimitDisplay.idl"
  "msg/edge.idl"
  "msg/nonDrivableLane.idl"
  "msg/ODDElements.idl"
  "msg/laneletODD.idl"
  "msg/attributePrim.idl"
  "srv/Teleoperation.idl"
  DEPENDENCIES
    "std_msgs"
    "builtin_interfaces"
    "geometry_msgs"
)

ament_auto_package()