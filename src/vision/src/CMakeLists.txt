add_subdirectory(base)
add_subdirectory(model)
add_subdirectory(pose_estimator)
add_subdirectory(calibration)

set(BOOSTER_ROBOTICS_SDK_DEPENDENCIES booster_robotics_sdk.a fastrtps fastcdr)


add_executable(vision_node main.cpp 
                           img_bridge.cpp
                           vision_node.cpp)
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

target_link_libraries(vision_node ${BOOSTER_ROBOTICS_SDK_DEPENDENCIES}
                                     booster_vision_base
                                     booster_vision_pose_estimator
                                     yolov8_model
                                     ${typesupport_target}
                                    #  booster_inferface_dds
                                     )


install(TARGETS
  vision_node
  DESTINATION lib/${PROJECT_NAME})
