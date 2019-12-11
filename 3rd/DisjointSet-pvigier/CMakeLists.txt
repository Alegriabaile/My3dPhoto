# Configuration

cmake_minimum_required(VERSION 3.8.2)

project(DisjointSet LANGUAGES CXX VERSION 1.0)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DisjointSet

add_library(disjoint_set INTERFACE)
target_include_directories(disjoint_set INTERFACE
    $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

# Tests

option (BUILD_TESTING "Build the testing tree." ON)
if (BUILD_TESTING)
    enable_testing()
    add_subdirectory(tests)
endif()

# Install

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    ${CMAKE_BINARY_DIR}/DisjointSetConfigVersion.cmake
    VERSION
    COMPATIBILITY AnyNewerVersion)
configure_package_config_file(
    ${CMAKE_SOURCE_DIR}/cmake/DisjointSetConfig.cmake.in
    ${CMAKE_BINARY_DIR}/DisjointSetConfig.cmake
    INSTALL_DESTINATION lib/cmake/DisjointSet)

install(TARGETS disjoint_set EXPORT DisjointSetTargets
    INCLUDES DESTINATION include)
install(EXPORT DisjointSetTargets
    FILE DisjointSetTargets.cmake
    DESTINATION lib/cmake/DisjointSet)
install(FILES
    ${CMAKE_BINARY_DIR}/DisjointSetConfigVersion.cmake
    ${CMAKE_BINARY_DIR}/DisjointSetConfig.cmake
    DESTINATION lib/cmake/DisjointSet)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)