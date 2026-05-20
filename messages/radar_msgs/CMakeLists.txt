cmake_minimum_required(VERSION 3.8)
project(radar_msgs)


if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")

  find_package(catkin REQUIRED COMPONENTS
    std_msgs
    message_generation
  )

  add_message_files(DIRECTORY msg FILES
    Coverage.msg
    Detection.msg
    DetectionRecord.msg
    Measurement.msg
  )

  generate_messages(DEPENDENCIES
    std_msgs
  )

  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")

  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Coverage.msg"
    "msg/Detection.msg"
    "msg/DetectionRecord.msg"
    "msg/Measurement.msg"
    DEPENDENCIES std_msgs builtin_interfaces
  )

  ament_package()

else()
  message(FATAL_ERROR "ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
