# add_definitions(-DAPI_EXPORTS)

set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
enable_language(CUDA)

set(SRCS block.cpp
         calibrator.cpp
         impl.cpp
         model.cpp
         postprocess.cpp
         postprocess.cu
         preprocess.cu
         yololayer.cu
         )

add_library(yolov8_trt SHARED ${SRCS})
target_link_libraries(yolov8_trt PRIVATE nvinfer
                                         cudart
                                 PUBLIC ${OpenCV_LIBS})
target_compile_options(yolov8_trt PRIVATE -w)
                              
add_executable(yolov8_det yolov8_det.cpp)
target_link_libraries(yolov8_det yolov8_trt nvinfer cudart)
target_compile_options(yolov8_det PRIVATE -w)

install(TARGETS yolov8_det
        DESTINATION lib/${PROJECT_NAME})