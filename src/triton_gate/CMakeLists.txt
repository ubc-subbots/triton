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

include_directories(include)

add_library(triton_gate SHARED
  src/gate_detector.cpp
)

rclcpp_components_register_nodes(triton_gate "triton_gate::GateDetector")

target_compile_definitions(triton_gate
  PRIVATE "triton_gate_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_gate
  "rclcpp"
  "rclcpp_components"
  "cv_bridge"
  "OpenCV"
  "triton_vision_utils"
)

install(TARGETS
  triton_gate
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

add_executable(gate_detector
  src/gate_detector1.cpp
)

target_include_directories(gate_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
  
ament_target_dependencies(gate_detector
  cv_bridge
  OpenCV
  triton_vision_utils
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS gate_detector
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
