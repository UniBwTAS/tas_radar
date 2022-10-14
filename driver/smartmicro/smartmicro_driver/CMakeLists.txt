cmake_minimum_required(VERSION 3.8)
project(smartmicro_driver)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(can_msgs REQUIRED)
find_package(ethernet_msgs REQUIRED)
find_package(radar_msgs REQUIRED)

# include_directories(include)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/Instruction.msg"
    "msg/Instructions.msg"
    DEPENDENCIES std_msgs)

rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

## CAN
# add_executable(can src/can/main.cpp src/can/node.cpp src/can/dbcc/umrr.c)
# ament_target_dependencies(can rclcpp std_msgs can_msgs radar_msgs)

# Ethernet
add_executable(ethernet src/ethernet/main.cpp src/ethernet/node.cpp)
add_dependencies(ethernet ${PROJECT_NAME})
target_link_libraries(ethernet ${cpp_typesupport_target})
ament_target_dependencies(ethernet rclcpp std_msgs ethernet_msgs radar_msgs)

install(TARGETS
    # can
    ethernet
    DESTINATION lib/${PROJECT_NAME})

ament_package()
