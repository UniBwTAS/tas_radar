cmake_minimum_required(VERSION 3.0.2)
project(datafitting)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11" CACHE STRING "" FORCE)

find_package(catkin REQUIRED COMPONENTS
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

install (DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
    PATTERN ".svn" EXCLUDE)

## Example
add_executable(example src/example.cpp)
target_link_libraries(example ${catkin_LIBRARIES})
