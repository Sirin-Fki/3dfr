cmake_minimum_required(VERSION 3.10)
project(renderer)

include_directories(
 include
 /usr/local/include
 ${OPENGL_INCLUDE_DIRS} 
 ${GLUT_INCLUDE_DIRS}
 ${GLFW3_INCLUDE_DIR}
)
set(directories
.
)

set(headers
 include/renderer.h
 include/camera.h
)

set (sources
 src/main.cpp
 src/renderer.cpp
)

add_executable(${PROJECT_NAME}_exe ${sources} ${headers})
target_link_libraries(${PROJECT_NAME}_exe
 common
 spdlog::spdlog
 ${CMAKE_DL_LIBS}
 ${OPENGL_LIBRARIES}
 ${GLUT_LIBRARY}
 ${GLFW3_LIBRARY}
 dataloader
 tracker
)