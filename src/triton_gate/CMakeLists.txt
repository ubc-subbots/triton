cmake_minimum_required(VERSION 3.5)
project(triton_gate)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(triton_vision_utils REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(tf2 REQUIRED)

include_directories(include)

add_library(triton_gate SHARED
  src/gate_detector.cpp
)

rclcpp_components_register_nodes(triton_gate "triton_gate::GateDetector")

target_compile_definitions(triton_gate
  PRIVATE "triton_gate_CPP_BUILDING_DLL"
)
target_compile_options(triton_gate
  PRIVATE -Wno-vla
)

ament_target_dependencies(triton_gate
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "OpenCV"
  "triton_vision_utils"
  "sensor_msgs"
  "image_transport"
  "tf2"
  "triton_interfaces"
)

install(TARGETS
  triton_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
endif()

ament_package()
