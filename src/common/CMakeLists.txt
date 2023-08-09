cmake_minimum_required(VERSION 3.10)
project(common)

include_directories (
  include  
)

set (headers
  include/data_types.h
  include/constants.h
  include/shaders.h
  include/glad/glad.h
  include/KHR/khrplatform.h
)

set (sources
 src/constants.cpp
 src/shaders.cpp
 src/glad.c
)

add_library(${PROJECT_NAME} ${sources} ${headers} )
target_include_directories(${PROJECT_NAME} PUBLIC 
include)
target_link_libraries (${PROJECT_NAME}
Eigen3::Eigen
)
