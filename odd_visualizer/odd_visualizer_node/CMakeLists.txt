cmake_minimum_required(VERSION 3.14)
project(odd_visualizer_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(odd_visualizer_node
src/odd_visualizer_node.cpp
src/utils.cpp
src/main.cpp)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)