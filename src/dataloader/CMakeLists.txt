cmake_minimum_required(VERSION 3.10)
project(dataloader)

include_directories (
  include
  ${ASSIMP_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
  /usr/local/include/
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)

set (directories 
  .
)

set (headers
  include/dataloader.h
  include/bfm_loader.h
  include/rgbd.h
)

set (sources
  src/dataloader.cpp
  src/bfm_loader.cpp
  src/rgbd.cpp
  )

add_library(${PROJECT_NAME} ${sources} ${headers})

target_link_libraries (${PROJECT_NAME}
  ${CMAKE_DL_LIBS}
  ${ASSIMP_LIBRARIES}
  ${HDF5_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenCV_LIBS}
  ${OpenPose_LIBS}
  ${GFLAGS_LIBRARY}
  ${GLOG_LIBRARY}
  nlohmann_json::nlohmann_json
  Eigen3::Eigen
  common
  ${realsense2_LIBRARY}
  spdlog::spdlog
)

target_link_directories(${PROJECT_NAME} PUBLIC 
 ${PCL_LIBRARY_DIRS}
)


target_include_directories(${PROJECT_NAME}  PUBLIC 
  include
  ${ASSIMP_INCLUDE_DIRS}
  ${HDF5_INCLUDE_DIR}
  /usr/local/include/
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
  ${GFLAGS_INCLUDE_DIR}
  ${GLOG_INCLUDE_DIR}
  ${realsense2_INCLUDE_DIR}
)
