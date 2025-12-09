cmake_minimum_required(VERSION 3.8)
project(sim_detection_bridge)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(vision_interface REQUIRED)

# Create executable
add_executable(sim_detection_bridge_node src/sim_detection_bridge.cpp)

# Link libraries
ament_target_dependencies(sim_detection_bridge_node
  rclcpp
  visualization_msgs
  vision_interface)

target_include_directories(sim_detection_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Install executable
install(TARGETS sim_detection_bridge_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
