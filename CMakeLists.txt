cmake_minimum_required(VERSION 3.16)
project(thor_deploy VERSION 0.1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(LCM REQUIRED IMPORTED_TARGET lcm)
find_package(yaml-cpp REQUIRED)
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Unitree SDK examples")
add_subdirectory(third_party/unitree_sdk2)

foreach(target IN ITEMS g1_control hand_control)
    add_executable(${target} cpp/${target}.cpp)
    target_include_directories(${target} PRIVATE
        cpp/lcm_types
        third_party/unitree_sdk2
    )
    target_link_libraries(${target} PRIVATE unitree_sdk2 PkgConfig::LCM)
    set_target_properties(${target} PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    )
endforeach()
target_link_libraries(g1_control PRIVATE yaml-cpp)
