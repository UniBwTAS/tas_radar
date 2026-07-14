cmake_minimum_required(VERSION 3.5)
project(smartmicro_configurator)

if(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "1")
  # ===================== ROS 1 only (Qt5 + librosqt GUI) =====================
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
  add_executable(configurator src/ros1/main.cpp src/ros1/node.cpp src/ros1/node.ui)
  add_dependencies(configurator ${catkin_EXPORTED_TARGETS})
  target_link_libraries(configurator ${catkin_LIBRARIES} Qt5::Core Qt5::Gui Qt5::Widgets)

elseif(DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" STREQUAL "2")
  # The configurator is a ROS 1-only Qt/librosqt tool (see src/ros1/); there is
  # nothing to build under ROS 2.
  find_package(ament_cmake REQUIRED)
  ament_package()

else()
  message(FATAL_ERROR "smartmicro_configurator: ROS_VERSION not set to 1 or 2. Please source a ROS environment first.")
endif()
