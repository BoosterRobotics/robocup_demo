cmake_minimum_required(VERSION 3.16...3.27)
project(brain)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unknown-pragmas)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(booster_interface REQUIRED)
find_package(game_controller_interface REQUIRED)
find_package(vision_interface REQUIRED)
find_package(booster_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(OpenCV REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)


# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Kick.msg"
  DEPENDENCIES std_msgs # Add packages that above messages depend on
)
ament_export_dependencies(rosidl_default_runtime)

# source files
file(GLOB SOURCE_FILES src/*.cpp)
add_executable(brain_node ${SOURCE_FILES})

# otherwise connot find msg in same pkg 
# Find the typesupport target
rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
# Link the typesupport target to your project
target_link_libraries(brain_node ${typesupport_target})


set_property(TARGET brain_node PROPERTY CXX_STANDARD 17)

target_link_libraries(brain_node ${OpenCV_LIBS} yaml-cpp) 
target_link_libraries(brain_node ${backward_ros_LIBRARIES})
include_directories(${geometry_msgs_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(${sensor_msgs_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-narrowing -Wno-unused-parameter -Wno-sign-compare -Wno-missing-field-initializers -Wno-unused-variable")

ament_target_dependencies(brain_node rclcpp rosidl_typesupport_cpp std_msgs sensor_msgs nav_msgs diagnostic_msgs behaviortree_cpp booster_interface vision_interface game_controller_interface booster_msgs tf2 tf2_ros tf2_geometry_msgs geometry_msgs visualization_msgs)

target_include_directories(brain_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(brain_node PUBLIC c_std_99 cxx_std_17) # Require C99 and C++17
install(TARGETS
  brain_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY behavior_trees
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
