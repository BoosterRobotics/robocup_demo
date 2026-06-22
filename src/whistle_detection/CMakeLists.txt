cmake_minimum_required(VERSION 3.14)
project(whistle_detection LANGUAGES C CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(ALSA REQUIRED alsa)
find_library(FFTW3F_LIB NAMES fftw3f)
if(FFTW3F_LIB)
  set(FFTW_LIBRARIES ${FFTW3F_LIB})
  find_path(FFTW_INCLUDE_DIRS NAMES fftw3.h PATHS /usr/include)
endif()
if(NOT FFTW3F_LIB)
  pkg_check_modules(FFTW fftw3)
  if(FFTW_FOUND)
    set(FFTW_LIBRARIES ${FFTW_LIBRARIES} fftw3f)
    if(NOT FFTW_INCLUDE_DIRS)
      set(FFTW_INCLUDE_DIRS "/usr/include")
    endif()
  else()
    message(FATAL_ERROR "FFTW3 float required: install libfftw3-dev and ensure libfftw3f is available")
  endif()
endif()

set(DUERWEN_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/third_party")
set(DUERWEN_INCLUDE_DIR "${DUERWEN_ROOT}/include")
set(DUERWEN_LIB_DIR "${DUERWEN_ROOT}/lib")
set(DUERWEN_SRC_DIR "${DUERWEN_ROOT}/src")

# Select the Duerwen DSP library by target architecture:
#   aarch64 (robot)   -> libduerwen_wakeup_DOA_v1.1.0_20251113.so
#   x86_64  (dev/sim) -> libduerwen_wakeup_DOA.so
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
  set(DUERWEN_LIB "${DUERWEN_LIB_DIR}/libduerwen_wakeup_DOA_v1.1.0_20251113.so")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
  set(DUERWEN_LIB "${DUERWEN_LIB_DIR}/libduerwen_wakeup_DOA.so")
else()
  message(FATAL_ERROR "whistle_detection: no Duerwen library for arch '${CMAKE_SYSTEM_PROCESSOR}'")
endif()

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${DUERWEN_INCLUDE_DIR}
)

add_executable(whistle
  src/main.cpp
  src/whistle_detection.cpp
  third_party/src/duerwen_alsa.c
  third_party/src/RecvDataCache.c
)

target_include_directories(whistle PRIVATE
  ${ALSA_INCLUDE_DIRS}
  ${DUERWEN_INCLUDE_DIR}
  ${FFTW_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

target_link_libraries(whistle
  ${DUERWEN_LIB}
  ${ALSA_LIBRARIES}
  ${FFTW_LIBRARIES}
  pthread
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
)

set_target_properties(whistle PROPERTIES
  INSTALL_RPATH "${DUERWEN_LIB_DIR}"
  BUILD_RPATH "${DUERWEN_LIB_DIR}"
)

install(TARGETS whistle
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
