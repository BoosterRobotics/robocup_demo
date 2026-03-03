cmake_minimum_required(VERSION 3.8)
project(booster_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

# generate message files
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoosterApiReqMsg.msg"
  "msg/BoosterApiRespMsg.msg"
  "msg/ButtonEventMsg.msg"
  "msg/MotorState.msg"
  "msg/MotorCmd.msg"
  "msg/LowCmd.msg"
  "msg/ImuState.msg"
  "msg/LowState.msg"
  "msg/RawBytesMsg.msg"
  "msg/RawBytesStamped.msg"
  "msg/Odometer.msg"
  "msg/FallDownState.msg"
  "msg/RemoteControllerState.msg"
  "msg/HandCommand.msg"
  "msg/HandDdsMsg.msg"
  "msg/HandParam.msg"
  "srv/RpcService.srv"
  "srv/AgentService.srv"
  DEPENDENCIES std_msgs
)

# install header files
install(DIRECTORY include/
  DESTINATION include/
)

# export dependencies
ament_export_dependencies(rosidl_default_runtime)
ament_export_include_directories(include)

ament_package()
