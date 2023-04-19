cmake_minimum_required(VERSION 2.8)
project(pointCloudTest)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

#find_package(libkinectwrapper REQUIRED)

message("pointCloudTest.libkinectwrapper_INCLUDE_DIRS:  " ${libkinectwrapper_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Ceres REQUIRED)
find_package(Boost REQUIRED COMPONENTS program_options)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
# pkg_check_modules(GTK3 REQUIRED gtkglext-1.0)

include_directories(
    ${libkinectwrapper_INCLUDE_DIRS}
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${APRIL_TAG_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
)

SET(MULTIKINECT_SRC
    pointCloudTest.cpp
)

add_executable(pointCloudTest  ${MULTIKINECT_SRC})

target_link_libraries(pointCloudTest kinectwrapper
    k4a
    #k4abt
    k4arecord
    pthread
    apriltag
    #vision_geometry
    GL
    GLU
    glfw
    gthread-2.0
    ${OpenCV_LIBRARIES}
    ${GTK3_LIBRARIES}
    ${CAIRO_LIBRARIES}
    ${CERES_LIBRARIES}
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)