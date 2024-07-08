cmake_minimum_required(VERSION 2.8)
project(libkinectwrapper)

set(CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

set(${PROJECT_NAME}_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
    CACHE INTERNAL "${PROJECT_NAME}: Include Directories" FORCE)

message("PROJECT_NAME:  " ${PROJECT_NAME})
message("PROJECT_NAME_INCLUDE_DIRS:  " ${PROJECT_NAME}_INCLUDE_DIRS)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Cairo REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(k4a REQUIRED)
find_package(VISP REQUIRED)

pkg_check_modules(GTK3 REQUIRED gtk+-3.0)
pkg_check_modules(APRILTAG REQUIRED apriltag)  # Add this line to find the AprilTag package

include_directories(
    include
    ${GTK3_INCLUDE_DIRS}
    ${CAIRO_INCLUDE_DIRS}
    ${VISION_GEOMETRY_INCLUDE_DIRS}
    ${APRILTAG_INCLUDE_DIRS}  # Add this line to include the AprilTag headers
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${VISP_INCLUDE_DIRS}
)

SET(LIB_KINECT_WRAPPER_SRC
    src/SKComputeXYZImage.cpp
    src/SKConfig.cpp
    src/SKPRDepthRegister.cpp
    src/SKDepthViewer.cpp
    src/SKPacket.cpp
    src/SKPRAprilTag.cpp
    src/SKPVideoDisplay.cpp
    src/SKWrapper.cpp
)

SET(LIB_KINECT_WRAPPER_INCLUDES
)

# Specify the path to the ViSP libraries
SET(VISP_LIB_DIR "/usr/lib/x86_64-linux-gnu/libvisp_detection.so")

add_library(kinectwrapper SHARED ${LIB_KINECT_WRAPPER_SRC})

# Link the AprilTag library
# target_link_libraries(kinectwrapper PRIVATE ${APRILTAG_LIBRARIES} ${VISP_LIBRARIES})
target_link_libraries(kinectwrapper PRIVATE ${APRILTAG_LIBRARIES} ${VISP_LIBRARIES} ${OpenCV_LIBS} ${GTK3_LIBRARIES} ${CAIRO_LIBRARIES} ${VISP_LIB_DIR} -lz)


install(TARGETS kinectwrapper DESTINATION lib)
install(DIRECTORY include/ DESTINATION include/kinectwrapper)
