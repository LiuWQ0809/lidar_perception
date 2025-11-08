cmake_minimum_required(VERSION 3.8)
project(fusion_cpp)

# C++ standard
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# System dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

# CUDA (optional, for TensorRT)
find_package(CUDA)
if(CUDA_FOUND)
  message(STATUS "CUDA found: ${CUDA_VERSION}")
  enable_language(CUDA)
  set(CMAKE_CUDA_STANDARD 14)
  add_definitions(-DUSE_CUDA)
endif()

# TensorRT (optional)
set(TENSORRT_ROOT "/usr/local/tensorrt" CACHE PATH "TensorRT root directory")
find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES include)
find_library(TENSORRT_LIBRARY_INFER nvinfer
  HINTS ${TENSORRT_ROOT} ${CUDA_TOOLKIT_ROOT_DIR}
  PATH_SUFFIXES lib lib64 lib/x64)

if(TENSORRT_INCLUDE_DIR AND TENSORRT_LIBRARY_INFER)
  message(STATUS "TensorRT found")
  set(TENSORRT_FOUND TRUE)
  add_definitions(-DUSE_TENSORRT)
else()
  message(WARNING "TensorRT not found, detector will be disabled")
  set(TENSORRT_FOUND FALSE)
endif()

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIR}
)

if(TENSORRT_FOUND)
  include_directories(${TENSORRT_INCLUDE_DIR})
endif()

# Library: fusion_perception_core
add_library(fusion_perception_core SHARED
  src/livox_parser.cpp
  src/sensor_fusion.cpp
  src/tracker.cpp
)

ament_target_dependencies(fusion_perception_core
  rclcpp
  sensor_msgs
  std_msgs
  livox_ros_driver2
)

target_link_libraries(fusion_perception_core
  ${OpenCV_LIBRARIES}
  Eigen3::Eigen
  yaml-cpp
)

# Library: detector (optional, if TensorRT is available)
if(TENSORRT_FOUND)
  add_library(detector SHARED
    src/detector_tensorrt.cpp
  )
  
  ament_target_dependencies(detector
    rclcpp
    sensor_msgs
    cv_bridge
  )
  
  target_link_libraries(detector
    ${OpenCV_LIBRARIES}
    ${TENSORRT_LIBRARY_INFER}
    ${CUDA_LIBRARIES}
  )
  
  if(CUDA_FOUND)
    target_include_directories(detector PUBLIC ${CUDA_INCLUDE_DIRS})
  endif()
endif()

# Executable: fusion_perception_node
add_executable(fusion_perception_node
  src/fusion_perception_node.cpp
)

ament_target_dependencies(fusion_perception_node
  rclcpp
  sensor_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  image_transport
  livox_ros_driver2
)

target_link_libraries(fusion_perception_node
  fusion_perception_core
  ${OpenCV_LIBRARIES}
  yaml-cpp
)

if(TENSORRT_FOUND)
  target_link_libraries(fusion_perception_node detector)
endif()

# Install
install(TARGETS
  fusion_perception_core
  fusion_perception_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

if(TENSORRT_FOUND)
  install(TARGETS
    detector
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
  )
endif()

install(DIRECTORY
  include/
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  calibration
  DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
