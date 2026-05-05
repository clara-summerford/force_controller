cmake_minimum_required(VERSION 3.8)
project(force_controller)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 Standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Library: force_controller
add_library(force_controller SHARED
  src/force_controller.cpp
  src/controller_utils.cpp
  src/wrench_filters.cpp
)
ament_target_dependencies(force_controller
  rclcpp
  geometry_msgs
  Eigen3
  std_msgs
  tf2_geometry_msgs
)

# Executable: fake_wrench_publisher
add_executable(fake_wrench_publisher controller_example/fake_wrench_pub.cpp)
ament_target_dependencies(fake_wrench_publisher
  rclcpp
  geometry_msgs
)

# Install targets
install(TARGETS
  force_controller
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Install directory contents
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(force_controller)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  Eigen3
  std_msgs
  tf2_geometry_msgs
)

ament_package()