cmake_minimum_required(VERSION 3.5)
project(radar_marker)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  # ========================= ROS 1 (catkin) =========================
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
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
    src/detections
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(detections
    src/detections/detections_node_ros1.cpp
    src/detections/ros1/node.cpp
    src/detections/core/marker_core.cpp)
  add_dependencies(detections ${catkin_EXPORTED_TARGETS})
  target_link_libraries(detections ${catkin_LIBRARIES})

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS 2 (ament / rclcpp) =====================
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(visualization_msgs REQUIRED)
  find_package(radar_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(datafitting REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(detections
    src/detections/detections_node_ros2.cpp
    src/detections/ros2/node.cpp
    src/detections/core/marker_core.cpp)
  target_include_directories(detections PRIVATE src/detections)
  ament_target_dependencies(detections
    rclcpp std_msgs geometry_msgs visualization_msgs radar_msgs nav_msgs datafitting tf2 tf2_ros)
  install(TARGETS detections DESTINATION lib/${PROJECT_NAME})

  ament_package()

else()
  message(FATAL_ERROR "radar_marker: ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
