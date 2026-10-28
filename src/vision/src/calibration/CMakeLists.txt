message(STATUS "Builing Calibration")
if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  list(APPEND CMAKE_PREFIX_PATH ${PROJECT_SOURCE_DIR}/thirdparty)
  find_package(Ceres REQUIRED)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/lib/
    DESTINATION lib
    FILES_MATCHING PATTERN "*.so*")
else()
  find_package(Ceres REQUIRED)
endif()

add_library(booster_vision_calibration SHARED calibration.cpp
                                              board_detector
                                              )
target_link_libraries(booster_vision_calibration PUBLIC ${OpenCV_LIBRARIES}
                                                        ${YAML_CPP_LIBRARIES}
                                                        booster_vision_base
                                                        Ceres::ceres
                                                        )

add_executable(calibration_node calibration_node.cpp
                                ../img_bridge.cpp)

target_link_libraries(calibration_node booster_vision_base
                                       booster_vision_pose_estimator
                                       yolov8_model
                                       booster_vision_calibration
                                       ${typesupport_target}
                                     )

ament_target_dependencies(calibration_node
  rclcpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  builtin_interfaces
)

install(TARGETS calibration_node
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS booster_vision_calibration
        DESTINATION lib)