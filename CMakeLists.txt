cmake_minimum_required(VERSION 3.10)
project(radix)

# add_compile_definitions(CLASSHISTORY)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)

  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")

  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

set(CMAKE_CXX_STANDARD 17)

#---- Add the subdirectory cmake ----
set(CMAKE_CONFIG_PATH ${CMAKE_MODULE_PATH}  "${CMAKE_CURRENT_LIST_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CONFIG_PATH}")

find_package(PCL QUIET COMPONENTS common io)

# find_package(benchmark QUIET)
# find_package(octomap QUIET)
find_package(LZ4 QUIET)
find_package(Eigen3 QUIET)

# custom srv/msg
find_package(radix_msgs REQUIRED)

#---- Use Address sanitizer if compiled in Debug mode ----
option(BONXAI_SANITIZE "Add address sanitizer when compiling in Debug mode" OFF)

if(BONXAI_SANITIZE)
    set (CMAKE_CXX_DEBUG_FLAGS "${CMAKE_CXX_DEBUG_FLAGS}  -fsanitize=address")
    set (CMAKE_LINKER_DEBUG_FLAGS "${CMAKE_LINKER_DEBUG_FLAGS}  -fsanitize=address")
endif()


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -fno-omit-frame-pointer)
endif()


####################################################
add_subdirectory(bonxai_core)
add_subdirectory(radix_utils)

if(Eigen_FOUND AND PCL_FOUND )
    add_subdirectory(radix_map)
endif()

# Default behavior
find_package(ament_cmake QUIET)

if ( ament_cmake_FOUND )

    project(radix_ros)

    # Add colcon-specific configuration here
    message(STATUS "Building with colcon")

    find_package(ament_cmake_auto REQUIRED)
    ament_auto_find_build_dependencies()

    include_directories(radix_ros/include 3rdparty)

    ament_auto_add_library(radix_server SHARED
        radix_ros/src/server.cpp
    )

    target_link_libraries(radix_server
        radix_map
        radix_utils
        ${PCL_LIBRARIES}
    )

    rclcpp_components_register_node(radix_server
    PLUGIN "Server"
    EXECUTABLE radix_server_node)

    ament_auto_package(
      INSTALL_TO_SHARE
      radix_ros/launch
      radix_ros/params
    )
else()

    message(STATUS "Building with cmake")

endif()
