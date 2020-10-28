cmake_minimum_required(VERSION 3.5)
project(triton_controls)

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
find_package(Eigen3 REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(thrust_allocator 
  src/thrust_allocator.cpp
)
target_include_directories(thrust_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  thrust_allocator
  "rclcpp"
  "rclcpp_components"
  "Eigen3"
  "triton_interfaces"
  "geometry_msgs"
  "sensor_msgs"
)
install(TARGETS thrust_allocator
  DESTINATION lib/${PROJECT_NAME}
)

add_library(triton_controls SHARED
  src/thrust_allocator.cpp
)

rclcpp_components_register_nodes(triton_controls "triton_controls::ThrustAllocator")

target_compile_definitions(triton_controls
  PRIVATE "triton_controls_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_controls
  "rclcpp"
  "rclcpp_components"
  "Eigen3"
  "triton_interfaces"
  "geometry_msgs"
  "sensor_msgs"
)

install(TARGETS
  triton_controls
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_thrust_allocator_launch.py
  )
endif()

ament_package()
