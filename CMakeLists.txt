cmake_minimum_required(VERSION 2.8)
project(ecto_corrector)
message(STATUS "~~ ${PROJECT_NAME}")

find_package(ecto REQUIRED)
#find_package(Boost COMPONENTS signals thread REQUIRED)

include(rosbuild_lite.cmake)
find_ros_package(pose_corrector)
if(NOT pose_corrector_FOUND)
  message("**\n** Disabling build of ${PROJECT_NAME} due to missing dependency pose_corrector\n**")
  return()
endif()
find_ros_package(sensor_msgs REQUIRED)
find_ros_package(geometry_msgs REQUIRED)
find_ros_package(tabletop_object_detector REQUIRED)  #for TODServiceCaller. can be removed once there's an ecto_tod

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
ecto_python_env_gen(${CMAKE_BINARY_DIR}/lib)

include_directories(${sensor_msgs_INCLUDES})
include_directories(${geometry_msgs_INCLUDES})
include_directories(${pose_corrector_INCLUDES})
include_directories(${tabletop_object_detector_INCLUDES})

#TODO: need some way to lookup where ecto_pcl is located
include_directories(/wg/stor2a/mkrainin/workspace/ecto_kitchen/pcl/include)

add_subdirectory(src)
