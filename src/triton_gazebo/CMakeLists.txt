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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS} -O3 -DNDEBUG")

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(image_transport REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(OpenCV 4.2 REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(triton_interfaces REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})

link_directories(${gazebo_dev_LIBRARY_DIRS})

add_library(ground_truth_sensor SHARED
  src/ground_truth_sensor.cpp
)
target_include_directories(ground_truth_sensor PUBLIC
  include
)
ament_target_dependencies(ground_truth_sensor
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(ground_truth_sensor)
target_link_libraries(
  ground_truth_sensor
  ${geometry_msgs_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  ground_truth_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(thruster_driver_plugin SHARED
  src/thruster_driver_plugin.cpp
)
target_include_directories(thruster_driver_plugin PUBLIC
  include
)
ament_target_dependencies(thruster_driver_plugin
  "gazebo_dev"
  "gazebo_ros"
)
ament_export_libraries(thruster_driver_plugin)
target_link_libraries(
  thruster_driver_plugin 
  ${GAZEBO_LIBRARIES}
)

install(TARGETS
  thruster_driver_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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
  "OpenCV"
  "cv_bridge"
)

install(TARGETS 
  underwater_camera
  DESTINATION lib/${PROJECT_NAME}
)

add_library(triton_gazebo SHARED
  src/underwater_camera.cpp
)

add_library(bounding_box_plugin SHARED src/bounding_box_plugin.cpp)
target_include_directories(bounding_box_plugin PUBLIC
  include
)
target_link_libraries(bounding_box_plugin ${GAZEBO_LIBRARIES})
ament_target_dependencies(bounding_box_plugin
  "triton_interfaces"
)
ament_export_libraries(bounding_box_plugin)
target_link_libraries(bounding_box_plugin ${GAZEBO_LIBRARIES})


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
  "geometry_msgs"
)

install(TARGETS
  triton_gazebo
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bounding_box_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
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

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  triton_gazebo/bounding_box_image_saver.py
  DESTINATION lib/${PROJECT_NAME}
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
