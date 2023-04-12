cmake_minimum_required(VERSION 2.8)
project(kinect_wrapper)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

add_subdirectory (libkinectwrapper)
add_subdirectory (applications)