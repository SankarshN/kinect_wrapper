cmake_minimum_required(VERSION 2.8)
project(kinect_wrapper)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")
set(CMAKE_BUILD_TYPE DEBUG)

add_subdirectory (share)
add_subdirectory (libkinectwrapper)
add_subdirectory (applications)