cmake_minimum_required(VERSION 3.5)
project(triton_mission_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
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
find_package(behaviortree_cpp REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(triton_mission_planner SHARED
  src/mission_planner_root.cpp
  src/gate_tree.cpp
  src/path_tree.cpp
)

target_link_libraries(triton_mission_planner behaviortree_cpp)

target_compile_definitions(triton_mission_planner
  PRIVATE "triton_mission_planner_CPP_BUILDING_DLL"
)

ament_target_dependencies(triton_mission_planner
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "triton_interfaces"
)

install(TARGETS
  triton_mission_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
endif()

ament_package()
