cmake_minimum_required(VERSION 3.10)
project(vision)

set(CMAKE_CXX_STANDARD 17)

option(BUILD_TEST "Build test" OFF)
option(NO_CUDA "Build without CUDA/TensorRT, use ONNX instead" OFF)

# If NO_CUDA is ON, use ONNX and skip CUDA/TensorRT
if(NO_CUDA)
  add_definitions(-DNO_CUDA)
  message(STATUS "Building without CUDA - using ONNX for inference")
  
  # Find ONNX Runtime when NO_CUDA is ON
  find_path(ONNXRUNTIME_INCLUDE_DIRS onnxruntime_cxx_api.h
    HINTS 
    /usr/include/onnxruntime
    /usr/local/include/onnxruntime
    /opt/onnxruntime/include
    PATH_SUFFIXES onnxruntime core/session)
  
  find_library(ONNXRUNTIME_LIBRARIES
    NAMES onnxruntime
    HINTS
    /usr/lib
    /usr/local/lib
    /opt/onnxruntime/lib)
  
  if(ONNXRUNTIME_INCLUDE_DIRS AND ONNXRUNTIME_LIBRARIES)
    message(STATUS "Found ONNX Runtime: ${ONNXRUNTIME_LIBRARIES}")
    include_directories(${ONNXRUNTIME_INCLUDE_DIRS})
  else()
    message(FATAL_ERROR "ONNX Runtime not found! Please install ONNX Runtime or set NO_CUDA=OFF to use CUDA/TensorRT")
  endif()
else()
  message(STATUS "Building with CUDA/TensorRT support")
endif()

# find ros2 package
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(vision_interface REQUIRED)

find_package(OpenCV 4.5 REQUIRED)
find_package(YAML-CPP REQUIRED)
find_package(PCL REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
#   "msg/DetectedObject.msg"
#   "msg/Detections.msg"
#   "msg/LineSegments.msg"
#   "msg/CalParam.msg"
#   DEPENDENCIES std_msgs builtin_interfaces
# )
# ament_export_dependencies(rosidl_default_runtime)
# rosidl_get_typesupport_target(typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/thirdparty/include
    ${OpenCV_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
)

add_subdirectory(src)
if(BUILD_TEST)
  add_subdirectory(test)
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY model
  DESTINATION share/${PROJECT_NAME})

ament_export_include_directories(include)
ament_package()
