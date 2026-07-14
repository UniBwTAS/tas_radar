cmake_minimum_required(VERSION 3.5)
project(radar_msgs)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # ========================= ROS 1 (catkin) =========================
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
  )

  add_message_files(
    FILES
    Coverage.msg
    Detection.msg
    DetectionRecord.msg
    Measurement.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS std_msgs
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS 2 (ament / rosidl) =====================
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Coverage.msg
    msg/Detection.msg
    msg/DetectionRecord.msg
    msg/Measurement.msg
    DEPENDENCIES std_msgs builtin_interfaces
  )

  ament_package()

else()
  message(FATAL_ERROR
    "radar_msgs: ROS_VERSION not set or unsupported: '$ENV{ROS_VERSION}'")
endif()
