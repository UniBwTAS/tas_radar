cmake_minimum_required(VERSION 3.0.2)
project(smartmicro_configurator)

#add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  smartmicro_driver
  librosqt
  radar_msgs
)

find_package(Qt5 5.9 COMPONENTS
    Core
    Gui
    Widgets
    REQUIRED
)

catkin_package(
  CATKIN_DEPENDS std_msgs smartmicro_driver radar_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTORCC ON)

## Configurator
add_executable(configurator src/main.cpp src/node.cpp src/node.ui)
target_link_libraries(configurator ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)
