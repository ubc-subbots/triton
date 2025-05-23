cmake_minimum_required(VERSION 3.5)
project(triton_object_recognition)

# Default to C99 for C and C++14 for C++
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(triton_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Include directories
include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

# Create a shared library
add_library(triton_object_recognition SHARED
  src/object_recognizer.cpp
)

# Link libraries
target_link_libraries(triton_object_recognition
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
)

# Register component nodes
rclcpp_components_register_nodes(triton_object_recognition 
  "triton_object_recognition::ObjectRecognizer"
)

# Define target dependencies explicitly
ament_target_dependencies(triton_object_recognition
  rclcpp
  rclcpp_components
  std_msgs
  triton_interfaces
  cv_bridge
  sensor_msgs
  image_transport
  OpenCV
  Boost
)

# Install the library and other directories
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

# Testing
if(BUILD_TESTING)
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(
    test/test_object_recognition_launch.py
  )
endif()

# Package configuration
ament_package()
