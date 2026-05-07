cmake_minimum_required(VERSION 3.10)

project(tricubic_interpolation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Avoid Windows headers defining min/max macros that conflict with std::min/std::max
if (WIN32)
    add_compile_definitions(NOMINMAX)
endif()

# Add compiler warnings for GCC and Clang
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    set(CMAKE_CXX_FLAGS_RELEASE "-O3")
    add_compile_options(
        -Wall        # Enable most warning flags
        -Wextra      # Enable extra warning flags
        -Wpedantic   # Enable pedantic mode
        -Werror      # Treat warnings as errors
    )
endif()

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(nanoflann)
find_package(PDAL 2.3.0 REQUIRED)
find_package(cxxopts 3.0.0 REQUIRED)

# GoogleTest
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip
)
FetchContent_MakeAvailable(googletest)
include(GoogleTest)

include_directories(${PDAL_INCLUDE_DIRS})
link_directories(${PDAL_LIBRARY_DIRS})
add_definitions(${PDAL_DEFINITIONS})

set(LIB_EIGEN Eigen3::Eigen)
set(LIB_NANOFLANN nanoflann::nanoflann)
set(LIB_CXXOPTS cxxopts::cxxopts)

set(BIN_DIR "${CMAKE_CURRENT_LIST_DIR}/bin") # binaries will be copied to this directory
#message("BIN_DIR=" ${BIN_DIR})

find_package(Threads REQUIRED)

# libnonrigid_icp library
add_library(libnonrigid_icp STATIC
    src/lib/io_utils.cpp
    src/lib/io_utils.hpp
    src/lib/pt_cloud.cpp
    src/lib/pt_cloud.hpp
    src/lib/translation_grid.cpp
    src/lib/translation_grid.hpp
    src/lib/correspondences.cpp
    src/lib/correspondences.hpp
    src/lib/optimization.cpp
    src/lib/optimization.hpp)

target_link_libraries(libnonrigid_icp PUBLIC ${LIB_EIGEN} ${LIB_NANOFLANN} ${PDAL_LIBRARIES})
target_include_directories(libnonrigid_icp PUBLIC ${CMAKE_CURRENT_LIST_DIR}/src/lib)
target_include_directories(libnonrigid_icp PRIVATE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(libnonrigid_icp PROPERTIES DEBUG_POSTFIX _d)

# nonrigid-icp executable
add_executable(nonrigid-icp src/prog/nonrigid_icp.cpp)
target_link_libraries(nonrigid-icp libnonrigid_icp ${LIB_CXXOPTS})
target_include_directories(nonrigid-icp PRIVATE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(nonrigid-icp PROPERTIES DEBUG_POSTFIX _d)

# nonrigid-icp-transform executable
add_executable(nonrigid-icp-transform src/prog/nonrigid_icp_transform.cpp)
target_link_libraries(nonrigid-icp-transform libnonrigid_icp ${LIB_CXXOPTS})
target_include_directories(nonrigid-icp-transform PRIVATE ${CMAKE_CURRENT_LIST_DIR})
set_target_properties(nonrigid-icp-transform PROPERTIES DEBUG_POSTFIX _d)

# Copy final executables to bin directory
file(MAKE_DIRECTORY ${BIN_DIR})
add_custom_command(TARGET nonrigid-icp           POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nonrigid-icp>           ${BIN_DIR})
add_custom_command(TARGET nonrigid-icp-transform POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:nonrigid-icp-transform> ${BIN_DIR})

# Enable testing
enable_testing()

# Add tests for the shell scripts
add_test(
    NAME nonrigid-icp.EagleScript
    COMMAND /bin/bash ${CMAKE_SOURCE_DIR}/test/test-eagle.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)
add_test(
    NAME nonrigid-icp.NordbahnScript
    COMMAND /bin/bash ${CMAKE_SOURCE_DIR}/test/test-nordbahn.sh
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test)
