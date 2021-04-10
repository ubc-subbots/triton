cmake_minimum_required(VERSION 3.5)
project(triton_sphere_detection)

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
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} )

#add_executable(triton_sphere_detection
#  src/sphere_detector.cpp
#)

add_library(triton_sphere_detection SHARED
  src/sphere_detector.cpp
)

#target_include_directories(triton_sphere_detection PUBLIC
#  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#  $<INSTALL_INTERFACE:include>
#)

target_link_libraries(triton_sphere_detection ${OpenCV_LIBRARIES} )

rclcpp_components_register_nodes(triton_sphere_detection
  "triton_sphere_detection::SphereDetector"
)

target_compile_definitions(triton_sphere_detection
  PRIVATE "triton_sphere_detection_CPP_BUILDING_DLL"
)
  
ament_target_dependencies(triton_sphere_detection
  "cv_bridge"
  "OpenCV"
  "triton_vision_utils"
  "rclcpp"
  "rclcpp_components"
  "geometry_msgs"
  "triton_interfaces"
  "std_msgs"
  "sensor_msgs"
  "image_transport"
)

install(TARGETS 
  triton_sphere_detection
  #EXPORT export_${PROJECT_NAME}
  #DESTINATION lib/${PROJECT_NAME}
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
    test/test_sphere_detection_launch.py
  )
endif()

ament_package()
