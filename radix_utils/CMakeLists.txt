cmake_minimum_required(VERSION 3.10)
project(radix_utils)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ROS2 dependencies
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(radix_msgs REQUIRED)


# Include headers
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/3rdparty
)

# Build library
add_library(${PROJECT_NAME}
  src/logger.cpp
  src/config.cpp
  src/helpers.cpp
)

set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)


ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  sensor_msgs
  radix_msgs
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp::yaml-cpp
)
