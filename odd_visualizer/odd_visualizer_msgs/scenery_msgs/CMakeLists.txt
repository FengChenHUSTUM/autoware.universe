# All rights reserved.
cmake_minimum_required(VERSION 3.14)

### Export headers
project(scenery_msgs)

# Generate messages
find_package(ament_cmake_auto REQUIRED
geometry_msgs REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  # "msg/edgeData.idl"
  # "msg/edgePrimitive.idl"
  # "msg/laneData.idl"
  # "msg/lanePrimitive.idl"
  # "msg/regulatoryEleData.idl"
  # "msg/regulatoryElePrimitive.idl"
  "msg/laneSequenceWithID.idl"
  DEPENDENCIES
    "std_msgs"
    "builtin_interfaces"
    "geometry_msgs"
  ADD_LINTER_TESTS
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()