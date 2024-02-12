cmake_minimum_required(VERSION 3.0.2)
project(radar_pointcloud)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  radar_msgs
  sensor_msgs
  nav_msgs
  datafitting
  tf2
  tf2_ros
  tf2_geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs radar_msgs sensor_msgs tf2_geometry_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Detections
add_executable(detection_accumulation src/detection_accumulation/main.cpp src/detection_accumulation/node.cpp)
target_link_libraries(detection_accumulation ${catkin_LIBRARIES})
