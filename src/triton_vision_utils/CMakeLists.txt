cmake_minimum_required(VERSION 3.5)
project(triton_vision_utils)

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
find_package(OpenCV 4 REQUIRED)

include_directories(include)

add_library(triton_vision_utils SHARED
  src/object_detector.cpp
  src/contour_features.cpp
)

target_include_directories(triton_vision_utils
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(triton_vision_utils
  "OpenCV"
)

ament_export_targets(export_triton_vision_utils HAS_LIBRARY_TARGET)
ament_export_dependencies(
  "OpenCV"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS triton_vision_utils
  EXPORT export_triton_vision_utils
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

if(BUILD_TESTING)
endif()

ament_package()
