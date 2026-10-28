enable_testing()

add_executable(test_yolov8 model/test_yolov8.cpp)
target_link_libraries(test_yolov8 yolov8_model pthread
                                  gtest
                                  gtest_main
) 

add_executable(validate model/validator.cpp)
target_link_libraries(validate yolov8_model
) 

add_executable(test_utils base/test_utils.cpp)
target_link_libraries(test_utils booster_vision_base pthread
                                  gtest
                                  gtest_main
)

add_executable(test_intrin base/test_intrin.cpp)
target_link_libraries(test_intrin booster_vision_base pthread
                                  gtest
                                  gtest_main
)

add_executable(test_ball_pose_estimator pose_estimator/test_ball_pose_estimator.cpp)
target_link_libraries(test_ball_pose_estimator booster_vision_pose_estimator pthread
                                               gtest
                                               gtest_main
)

add_executable(test_color_classifier test_color_classifier.cpp)
target_link_libraries(test_color_classifier booster_vision_base pthread yolov8_model
)

add_test(NAME utest_utils COMMAND test_utils)
add_test(NAME utest_intrin COMMAND test_intrin)
add_test(NAME utest_ball_pose_estimator COMMAND test_ball_pose_estimator)
add_test(NAME utest_yolov8 COMMAND test_yolov8 ${PROJECT_SOURCE_DIR}/config/vision.yaml ${PROJECT_SOURCE_DIR}/test/imgs/detection/raw.jpg)

install(TARGETS test_yolov8
                test_utils
                test_intrin
                test_ball_pose_estimator
        DESTINATION lib/${PROJECT_NAME})
