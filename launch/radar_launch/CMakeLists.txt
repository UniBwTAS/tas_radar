cmake_minimum_required(VERSION 3.5)
project(radar_launch)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED)
  catkin_package()
  install(DIRECTORY launch/ros1/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  find_package(ament_cmake REQUIRED)
  install(DIRECTORY launch/ros2/
    DESTINATION share/${PROJECT_NAME}/launch)
  ament_package()

else()
  message(FATAL_ERROR "radar_launch: ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
