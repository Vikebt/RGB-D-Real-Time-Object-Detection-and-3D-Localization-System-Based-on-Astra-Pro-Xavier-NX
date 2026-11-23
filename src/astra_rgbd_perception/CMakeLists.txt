# 中文：astra_rgbd_perception 包构建脚本。
cmake_minimum_required(VERSION 3.5.1)
project(astra_rgbd_perception)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# catkin_make does not set CMAKE_BUILD_TYPE by default; without optimization the
# per-pixel preprocessing and FP16<->FP32 conversions are an order of magnitude
# slower on the Xavier NX.
# 中文：catkin_make 默认不开优化；必须加 -O3，否则预处理/FP16 转换会慢一个量级。
add_compile_options(-O3)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  message_filters
  message_generation
  pcl_conversions
  tf2
  tf2_ros
)

find_package(PCL REQUIRED COMPONENTS common)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  PATHS /usr/include/aarch64-linux-gnu /usr/include
)
find_library(TENSORRT_LIBRARY nvinfer
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib
)
find_library(TENSORRT_PLUGIN_LIBRARY nvinfer_plugin
  PATHS /usr/lib/aarch64-linux-gnu /usr/lib
)
if(NOT TENSORRT_INCLUDE_DIR OR NOT TENSORRT_LIBRARY)
  message(FATAL_ERROR "TensorRT headers/library not found")
endif()

add_message_files(
  FILES
  Object3D.msg
  Object3DArray.msg
  RgbdSyncStats.msg
  Detection2D.msg
  Detection2DArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    roslib
    std_msgs
    geometry_msgs
    sensor_msgs
    visualization_msgs
    message_filters
    message_runtime
    pcl_conversions
    tf2
    tf2_ros
  DEPENDS PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})

add_executable(rgbd_sync_node src/rgbd_sync_node.cpp)
add_dependencies(rgbd_sync_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(rgbd_sync_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_library(astra_trt_engine src/trt_engine.cpp)
target_include_directories(astra_trt_engine PUBLIC
  include
  ${TENSORRT_INCLUDE_DIR}
  ${CUDA_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
target_link_libraries(astra_trt_engine
  ${TENSORRT_LIBRARY}
  ${TENSORRT_PLUGIN_LIBRARY}
  ${CUDA_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(detector_node src/detector_node.cpp)
add_dependencies(detector_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
  astra_trt_engine
)
target_link_libraries(detector_node
  astra_trt_engine
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${CUDA_LIBRARIES}
  ${TENSORRT_LIBRARY}
)

add_executable(depth_localizer_node src/depth_localizer_node.cpp)
add_dependencies(depth_localizer_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(depth_localizer_node
  ${catkin_LIBRARIES}
)

install(TARGETS rgbd_sync_node detector_node depth_localizer_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)