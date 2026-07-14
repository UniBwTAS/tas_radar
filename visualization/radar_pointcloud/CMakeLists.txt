cmake_minimum_required(VERSION 3.5)
project(radar_pointcloud)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  # ========================= ROS 1 (catkin) =========================
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    radar_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    datafitting
    tf2
    tf2_ros
  )

  catkin_package(
    CATKIN_DEPENDS std_msgs radar_msgs sensor_msgs
  )

  include_directories(
    src/detection_accumulation
    ${catkin_INCLUDE_DIRS}
  )

  add_executable(detection_accumulation
    src/detection_accumulation/detection_accumulation_node_ros1.cpp
    src/detection_accumulation/ros1/node.cpp
    src/detection_accumulation/core/accumulate.cpp)
  add_dependencies(detection_accumulation ${catkin_EXPORTED_TARGETS})
  target_link_libraries(detection_accumulation ${catkin_LIBRARIES})

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS 2 (ament / rclcpp) =====================
  find_package(ament_cmake REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(radar_msgs REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(nav_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(datafitting REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)

  add_executable(detection_accumulation
    src/detection_accumulation/detection_accumulation_node_ros2.cpp
    src/detection_accumulation/ros2/node.cpp
    src/detection_accumulation/core/accumulate.cpp)
  target_include_directories(detection_accumulation PRIVATE src/detection_accumulation)
  ament_target_dependencies(detection_accumulation
    rclcpp std_msgs radar_msgs sensor_msgs nav_msgs geometry_msgs datafitting tf2 tf2_ros)
  install(TARGETS detection_accumulation DESTINATION lib/${PROJECT_NAME})

  ament_package()

else()
  message(FATAL_ERROR "radar_pointcloud: ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
