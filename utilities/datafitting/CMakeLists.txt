cmake_minimum_required(VERSION 3.8)
project(datafitting)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)

include_directories(include)

add_executable(example src/example.cpp)

# header-only library
add_library(${PROJECT_NAME}_library INTERFACE)
target_include_directories(${PROJECT_NAME}_library INTERFACE
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# install header-only libraries
install(TARGETS
    ${PROJECT_NAME}_library
    EXPORT export_${PROJECT_NAME})

# install executables
install(TARGETS
    example
    DESTINATION lib/${PROJECT_NAME})

# install include directory
install(DIRECTORY
    include/
    DESTINATION include/${PROJECT_NAME})

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")

# Export modern CMake targets
ament_export_targets(export_${PROJECT_NAME})

ament_export_dependencies(
    rclcpp)

ament_package()
