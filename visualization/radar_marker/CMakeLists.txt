cmake_minimum_required(VERSION 3.0.2)
project(radar_marker)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  visualization_msgs
  radar_msgs
  nav_msgs
  datafitting
  tf2
  tf2_ros
)

catkin_package(
  CATKIN_DEPENDS std_msgs radar_msgs visualization_msgs
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


## Detections
add_executable(detections src/detections/main.cpp src/detections/node.cpp)
target_link_libraries(detections ${catkin_LIBRARIES})
