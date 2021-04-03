cmake_minimum_required(VERSION 3.5)
project(triton_sphere)

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
find_package(geometry_msgs REQUIRED)
find_package(triton_interfaces REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_executable(sphere_detector
  src/sphere_detector.cpp
)

target_include_directories(sphere_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
  
ament_target_dependencies(sphere_detector
  cv_bridge
  OpenCV
  triton_vision_utils
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "triton_interfaces"
)

install(TARGETS sphere_detector
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
endif()

ament_package()
