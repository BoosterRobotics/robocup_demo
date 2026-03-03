add_subdirectory(base)
add_subdirectory(model)
add_subdirectory(pose_estimator)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  add_subdirectory(calibration)
endif()

set(BOOSTER_ROBOTICS_SDK_DEPENDENCIES booster_robotics_sdk.a fastrtps fastcdr)

add_executable(vision_node main.cpp 
                           img_bridge.cpp
                           vision_node.cpp)


target_link_libraries(vision_node ${BOOSTER_ROBOTICS_SDK_DEPENDENCIES}
                                     booster_vision_base
                                     booster_vision_pose_estimator
                                     yolov8_model
                                     ${typesupport_target}
                                     )

ament_target_dependencies(vision_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  tf2
  builtin_interfaces
  vision_interface
)


install(TARGETS vision_node 
  DESTINATION lib/${PROJECT_NAME})


