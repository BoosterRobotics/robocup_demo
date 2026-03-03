# include and link dirs of cuda and tensorrt, you need adapt them if yours are different
# Only include CUDA/TensorRT when NO_CUDA is OFF
if(NO_CUDA)
  # Build with ONNX
  add_subdirectory(onnx)
else()
  # Build with CUDA/TensorRT
  if (CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    include_directories(/usr/local/cuda/targets/aarch64-linux/include)
    link_directories(/usr/local/cuda/targets/aarch64-linux/lib)
  else()
    # cuda
    include_directories(/usr/local/cuda/include)
    link_directories(/usr/local/cuda/lib64)

    # tensorrt
    include_directories(/usr/local/TensorRT-8.6.1.6/include)
    link_directories(/usr/local/TensorRT-8.6.1.6/lib)
  endif()

  add_subdirectory(trt)
endif()

add_library(yolov8_model SHARED detector.cc
                                segmentor.cc)

if(NO_CUDA)
  # Link only ONNX when NO_CUDA is ON
  target_link_libraries(yolov8_model PUBLIC yolov8_onnx
                                          ${YAML_CPP_LIBRARIES})
else()
  # Link TensorRT when CUDA is available
  target_link_libraries(yolov8_model PUBLIC yolov8_trt
                                          ${YAML_CPP_LIBRARIES})
endif()
target_compile_options(yolov8_model PRIVATE -w)

install(TARGETS yolov8_model DESTINATION lib)