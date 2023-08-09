cmake_minimum_required(VERSION 3.10)
project(tracker)

include_directories (
  include
  /usr/local/include/
  ${OpenCV_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
)

# source code directories
set (directories 
  .
)

set (headers
  include/face_detector.h
)

set (sources
  src/face_detector.cpp
  )

add_library(${PROJECT_NAME} ${sources} ${headers})

target_link_libraries (${PROJECT_NAME}
  ${CMAKE_DL_LIBS}
  ${OpenCV_LIBS}
  ${OpenPose_LIBS}
  Eigen3::Eigen
  common
)

target_include_directories(${PROJECT_NAME}  PUBLIC 
  include
  /usr/local/include/
  ${OpenCV_INCLUDE_DIRS}
  ${OpenPose_INCLUDE_DIRS}
)
