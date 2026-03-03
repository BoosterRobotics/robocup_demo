add_library(yolov8_onnx SHARED detection_impl.cpp
                               segmentation_impl.cpp)
target_link_libraries(yolov8_onnx PUBLIC ${OpenCV_LIBRARIES}
                                  PRIVATE ${ONNXRUNTIME_LIBRARIES})
target_include_directories(yolov8_onnx PRIVATE ${ONNXRUNTIME_INCLUDE_DIRS})
install(TARGETS yolov8_onnx DESTINATION lib)