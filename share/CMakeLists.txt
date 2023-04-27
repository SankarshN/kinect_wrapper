cmake_minimum_required(VERSION 2.8)
project(kinect_wrapper_share)

set(APRIL_TAG_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(VISION_GEOMETRY_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/vision_geometry/include PARENT_SCOPE)

# add_subdirectory (apriltag apriltag/build)
#add_subdirectory (vision_geometry vision_geometry/build)

# message("APRIL_TAG_INCLUDE_DIRS" ${APRIL_TAG_INCLUDE_DIRS})
message("VISION_GEOMETRY_INCLUDE_DIRS" ${VISION_GEOMETRY_INCLUDE_DIRS})
