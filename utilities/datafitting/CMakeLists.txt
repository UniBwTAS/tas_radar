cmake_minimum_required(VERSION 3.5)
project(datafitting)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  # ========================= ROS 1 (catkin) =========================
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

  find_package(catkin REQUIRED)

  catkin_package(
    INCLUDE_DIRS include
  )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
  )

  install(DIRECTORY include/${PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
      PATTERN ".svn" EXCLUDE)

  ## Example
  add_executable(example src/example.cpp)
  target_link_libraries(example ${catkin_LIBRARIES})

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  # ===================== ROS 2 (ament) =====================
  find_package(ament_cmake REQUIRED)

  # Header-only library. Consumers include <DataFitting/Service.h>, so install the
  # include/ tree at the prefix root and export it (keeps the ROS 1 include path).
  install(DIRECTORY include/
    DESTINATION include
    FILES_MATCHING PATTERN "*.h")
  ament_export_include_directories(include)

  ## Example
  add_executable(example src/example.cpp)
  target_include_directories(example PRIVATE include)
  install(TARGETS example DESTINATION lib/${PROJECT_NAME})

  ament_package()

else()
  message(FATAL_ERROR
    "datafitting: ROS_VERSION not set or unsupported: '$ENV{ROS_VERSION}'")
endif()
