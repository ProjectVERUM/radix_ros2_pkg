add_library(radix_map
    src/radix_map/pcl_utils.cpp
)

target_compile_definitions(radix_map PRIVATE
    ${PCL_DEFINITIONS})

target_include_directories(radix_map PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(radix_map PUBLIC
    bonxai_core
    radix_utils
    ${PCL_LIBRARIES})


####################################################

if(benchmark_FOUND AND octomap_FOUND)
    add_subdirectory(benchmark)
endif()
