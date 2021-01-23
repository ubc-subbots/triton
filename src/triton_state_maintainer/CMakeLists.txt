cmake_minimum_required(VERSION 3.5)
project(triton_state_maintainer)

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
find_package(std_msgs REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
)

add_executable(component_one
  src/component_one.cpp
)
target_include_directories(component_one PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(
  component_one
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "triton_interfaces"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)
install(TARGETS component_one
  DESTINATION lib/${PROJECT_NAME}
)

add_library(triton_state_maintainer SHARED
  src/component_one.cpp
)

rclcpp_components_register_nodes(triton_state_maintainer "triton_state_maintainer::ComponentOne")

target_compile_definitions(triton_state_maintainer
  PRIVATE "triton_state_maintainer_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_state_maintainer
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "triton_interfaces"
  "geometry_msgs"
  "tf2"
  "tf2_ros"
)

install(TARGETS
  triton_state_maintainer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

#install(DIRECTORY
#  test/config
#  DESTINATION share/${PROJECT_NAME}/test
#)

#if(BUILD_TESTING)
#  find_package(launch_testing_ament_cmake REQUIRED)
#  add_launch_test(
#    test/test_triton_state_maintainer_launch.py //what name goes here?
#  )
#endif()

ament_package()
