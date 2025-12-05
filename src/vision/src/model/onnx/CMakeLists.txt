add_library(yolov8_onnx SHARED detection_impl.cpp)
target_link_libraries(yolov8_onnx PUBLIC ${OpenCV_LIBRARIES}
                                  PRIVATE onnxruntime)
install(TARGETS yolov8_onnx DESTINATION lib)