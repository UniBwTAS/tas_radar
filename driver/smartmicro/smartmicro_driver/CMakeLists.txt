cmake_minimum_required(VERSION 3.0.2)
project(smartmicro_driver)

#add_compile_options(-std=c++11)
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
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  CATKIN_DEPENDS std_msgs radar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## CAN
add_executable(can src/can/main.cpp src/can/node.cpp src/can/dbcc/umrr.c)
target_link_libraries(can ${catkin_LIBRARIES})

## Ethernet
add_executable(ethernet src/ethernet/main.cpp src/ethernet/node.cpp)
add_dependencies(ethernet smartmicro_driver_generate_messages_cpp)
target_link_libraries(ethernet ${catkin_LIBRARIES})
