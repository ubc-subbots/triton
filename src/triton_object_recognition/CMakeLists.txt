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
find_package(triton_interfaces REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV 4.2 REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

add_library(triton_object_recognition SHARED
  src/object_recognizer.cpp
)

target_link_libraries(triton_object_recognition ${OpenCV_LIBRARIES})

rclcpp_components_register_nodes(triton_object_recognition "object_recognition::ObjectRecognizer")

target_compile_definitions(triton_object_recognition
  PRIVATE "triton_object_recognition_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_object_recognition
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "triton_interfaces"
  "cv_bridge"
  "sensor_msgs"
  "OpenCV"
)

install(TARGETS
  triton_object_recognition
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()
endif()

ament_package()