cmake_minimum_required(VERSION 3.5)
project(triton_example)

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

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(triton_example SHARED
  src/component_one.cpp
  src/component_two.cpp
)

rclcpp_components_register_nodes(triton_example "example::ComponentOne")
rclcpp_components_register_nodes(triton_example "example::ComponentTwo")

target_compile_definitions(triton_example
  PRIVATE "triton_example_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_example
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "triton_interfaces"
)

install(TARGETS
  triton_example
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
