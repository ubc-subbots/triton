cmake_minimum_required(VERSION 3.5)
project(triton_gazebo)

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
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)

include_directories(include)

add_executable(underwater_camera 
  src/underwater_camera.cpp
)
target_include_directories(underwater_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(underwater_camera
  "image_transport"
  "sensor_msgs"
  "message_filters"
  "rclcpp"
)

install(TARGETS 
  underwater_camera
  DESTINATION lib/${PROJECT_NAME}
)

add_library(triton_gazebo SHARED
  src/underwater_camera.cpp
)

rclcpp_components_register_nodes(triton_gazebo "triton_gazebo::UnderwaterCamera")

target_compile_definitions(triton_gazebo
  PRIVATE "triton_gazebo_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_gazebo
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "sensor_msgs"
  "message_filters"
)

install(TARGETS
  triton_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)




install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}/gazebo
)

install(DIRECTORY
  models
  DESTINATION share/${PROJECT_NAME}/gazebo
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)

  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_default_world
    ARGS "headless:=true"
  )
    
  add_launch_test(
    test/test_gazebo_launch.py
    TARGET test_gazebo_launch_cube_world
    ARGS "headless:=true" "world:=cube.world"
  )

  add_launch_test(
    test/test_underwater_camera_launch.py
    TARGET test_underwater_camera_launch
  )

endif()

ament_package()
