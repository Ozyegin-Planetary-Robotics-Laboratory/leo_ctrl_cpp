cmake_minimum_required(VERSION 3.8)
project(leo_ctrl_cpp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Add compiler warnings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable
add_executable(leo_ctrl_cpp_node src/main.cpp)

# Link libraries
ament_target_dependencies(leo_ctrl_cpp_node rclcpp std_msgs sensor_msgs geometry_msgs)

# Install targets
install(TARGETS
  leo_ctrl_cpp_node
  DESTINATION lib/${PROJECT_NAME})

# Install include files
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

# Testing (optional)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize
ament_package()
