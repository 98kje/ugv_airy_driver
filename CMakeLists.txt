cmake_minimum_required(VERSION 3.14)
project(ugv_airy_driver)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_library(airy_converter SHARED
  src/airy_converter.cpp
)

rclcpp_components_register_node(airy_converter
  PLUGIN "ugv_airy_driver::AiryConverter"
  EXECUTABLE airy_converter_node
)

ament_auto_package(INSTALL_TO_SHARE
  config
  launch
)
