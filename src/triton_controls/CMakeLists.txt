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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

include_directories(include)

add_executable(thrust_allocator 
  src/thrust_allocator.cpp
)
target_include_directories(thrust_allocator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(waypoint_marker
  src/waypoint_marker.cpp
)
target_include_directories(waypoint_marker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_generator
  src/trajectory_generator.cpp
)
target_include_directories(trajectory_generator PUBLIC
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
  "OpenCV"
  "tf2"
  "tf2_geometry_msgs"
)
install(TARGETS thrust_allocator
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  waypoint_marker
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "triton_interfaces"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)
install(TARGETS waypoint_marker
  DESTINATION lib/${PROJECT_NAME}
)

ament_target_dependencies(
  trajectory_generator
  "rclcpp"
  "rclcpp_components"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "triton_interfaces"
  "geometry_msgs"
  "sensor_msgs"
  "nav_msgs"
)
install(TARGETS trajectory_generator
  DESTINATION lib/${PROJECT_NAME}
)

add_library(triton_controls SHARED
  src/thrust_allocator.cpp
  # src/waypoint_marker.cpp
  src/serial_subscriber
)

rclcpp_components_register_nodes(triton_controls "triton_controls::ThrustAllocator")
rclcpp_components_register_nodes(triton_controls "triton_controls::WaypointMarker")
rclcpp_components_register_nodes(triton_controls "triton_controls::SerialSubscriber")
rclcpp_components_register_nodes(triton_controls "triton_controls::TrajectoryGenerator")

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
  "OpenCV"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "nav_msgs"
)

install(TARGETS
  triton_controls
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  test/config
  DESTINATION share/${PROJECT_NAME}/test
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  triton_controls/auv_transform_publisher.py
  triton_controls/waypoint_marker_tester.py
  triton_controls/predetermined_route.py
  triton_controls/predetermined_thrust.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_thrust_allocator_launch.py
  )
endif()

ament_package()


