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
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)


find_package(Eigen3 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(action_msgs REQUIRED)        
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)

# ROS-related depends to link to the library
set(
  THIS_PACKAGE_INCLUDE_DEPENDS 
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  sensor_msgs
  tf2_geometry_msgs
  moveit_core
  moveit_ros_planning
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/HybridController.action"
  DEPENDENCIES geometry_msgs sensor_msgs
)

rosidl_get_typesupport_target(
  cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Executables
add_executable(fake_wrench_publisher controller_example/fake_wrench_pub.cpp)
add_executable(hybrid_controller_server src/hybrid_controller_server_main.cpp)

# Library: force_controller
add_library(force_controller_lib SHARED
  src/force_controller.cpp
  src/utils/param_handler.cpp
  src/utils/controller_utils.cpp
  src/utils/utils.cpp
  src/utils/wrench_filters.cpp
  src/hybrid_controller_action_server.cpp
  # controller_example/fake_wrench_pub.cpp
)

# Link executable to corresponding library 
target_link_libraries(hybrid_controller_server force_controller_lib)
target_link_libraries(fake_wrench_publisher force_controller_lib)
target_link_libraries(force_controller_lib 
  yaml-cpp
  "${cpp_typesupport_target}")

# Specify header search path 
target_include_directories(force_controller_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(force_controller_lib ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Install targets
install(TARGETS
  force_controller_lib
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  INCLUDES DESTINATION include
)

# Install executables
install(TARGETS
  hybrid_controller_server
  fake_wrench_publisher
  DESTINATION lib/${PROJECT_NAME})

# Install directory contents
install(
  DIRECTORY include/ 
  DESTINATION include)

install(
  DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

# EXPORTS
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(force_controller_lib)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
  
ament_package()