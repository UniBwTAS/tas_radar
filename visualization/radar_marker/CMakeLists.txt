cmake_minimum_required(VERSION 3.8)
project(radar_marker)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(radar_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(datafitting REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(detections src/detections/main.cpp src/detections/node.cpp)
ament_target_dependencies(detections rclcpp std_msgs visualization_msgs radar_msgs nav_msgs datafitting tf2 tf2_ros)

install(TARGETS
    detections
    DESTINATION lib/${PROJECT_NAME})

ament_package()
