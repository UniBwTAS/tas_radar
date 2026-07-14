cmake_minimum_required(VERSION 3.5)
project(smartmicro_driver)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  # ========================= ROS 1 (catkin) =========================
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

  find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    can_msgs
    ethernet_msgs
    radar_msgs
    message_generation
  )

  add_message_files(
    FILES
    Instruction.msg
    Instructions.msg
    Sensor.msg
    Sensors.msg
  )

  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS std_msgs radar_msgs message_runtime
  )

  include_directories(
    src
    ${catkin_INCLUDE_DIRS}
  )

  ## Ethernet
  add_executable(ethernet
    src/ethernet/ethernet_node_ros1.cpp
    src/ethernet/ros1/node.cpp
    src/ethernet/core/sms.cpp)
  add_dependencies(ethernet smartmicro_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(ethernet ${catkin_LIBRARIES})

  ## CAN
  add_executable(can
    src/can/can_node_ros1.cpp
    src/can/ros1/node.cpp
    src/can/core/can.cpp
    src/can/core/dbcc/umrr.c)
  add_dependencies(can smartmicro_driver_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
  target_link_libraries(can ${catkin_LIBRARIES})

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS 2 (ament / rclcpp) =====================
  find_package(ament_cmake REQUIRED)
  find_package(rosidl_default_generators REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(std_msgs REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(ethernet_msgs REQUIRED)
  find_package(radar_msgs REQUIRED)
  find_package(can_msgs REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME}
    msg/Instruction.msg
    msg/Instructions.msg
    msg/Sensor.msg
    msg/Sensors.msg
    DEPENDENCIES std_msgs builtin_interfaces
  )

  ## Ethernet
  add_executable(ethernet
    src/ethernet/ethernet_node_ros2.cpp
    src/ethernet/ros2/node.cpp
    src/ethernet/core/sms.cpp)
  target_include_directories(ethernet PRIVATE src)
  ament_target_dependencies(ethernet rclcpp ethernet_msgs radar_msgs)

  # link the message types generated in THIS package into the node
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
  target_link_libraries(ethernet ${cpp_typesupport_target})

  install(TARGETS ethernet DESTINATION lib/${PROJECT_NAME})

  ## CAN
  add_executable(can
    src/can/can_node_ros2.cpp
    src/can/ros2/node.cpp
    src/can/core/can.cpp
    src/can/core/dbcc/umrr.c)
  target_include_directories(can PRIVATE src)
  ament_target_dependencies(can rclcpp can_msgs radar_msgs)
  install(TARGETS can DESTINATION lib/${PROJECT_NAME})

  ament_package()

else()
  message(FATAL_ERROR "smartmicro_driver: ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
