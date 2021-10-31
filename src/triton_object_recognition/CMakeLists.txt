cmake_minimum_required(VERSION 3.5)
project(triton_object_recognition)

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
find_package(geometry_msgs REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV 4.5.3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem) 

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(triton_object_recognition SHARED
  src/object_recognizer.cpp
  src/bounding_box_pose_estimation.cpp
)

target_link_libraries(triton_object_recognition ${OpenCV_LIBRARIES} ${Boost_LIBRARIES})

rclcpp_components_register_nodes(triton_object_recognition 
  "triton_object_recognition::ObjectRecognizer"
  "triton_object_recognition::BoundingBoxPoseEstimation"
)

target_compile_definitions(triton_object_recognition
  PRIVATE "triton_object_recognition_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_object_recognition
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs"
  "triton_interfaces"
  "cv_bridge"
  "sensor_msgs"
  "image_transport"
  "OpenCV"
  "Boost"
)

install(TARGETS
  triton_object_recognition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_object_recognition_launch.py
  )
endif()

ament_package()
