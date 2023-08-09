cmake_minimum_required(VERSION 3.10)

project(3DFaceReconstruction)

cmake_policy(SET CMP0072 NEW)
cmake_policy(SET CMP0100 NEW)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_PREFIX_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_MODULE_PATH ${CMAKE_BINARY_DIR})

set(nlohmann_json_DIR "/home/sfkiditk/Documents/Studium/Master/4.Semester/IDPs/3rdparty/json/build")

list (APPEND CMAKE_MODULE_PATH "cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Debug CACHE STRING "Choose the type of build." FORCE)
  message(STATUS "Setting build type to '${CMAKE_BUILD_TYPE}' as none was specified.")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

find_package(OpenGL REQUIRED)

find_package(GLFW3 REQUIRED)

find_package(GLUT REQUIRED)

find_package(assimp REQUIRED)

find_package(HDF5 REQUIRED COMPONENTS CXX HL)

find_package(PCL 1.3 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

find_package(GFlags)

find_package(Glog)

find_package(Threads)

find_package(OpenPose REQUIRED)

find_package(OpenCV REQUIRED)

find_package(nlohmann_json 3.2.0 REQUIRED)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

find_package(Ceres REQUIRED)

find_package(libfreenect REQUIRED)

find_package(Boost REQUIRED)

find_package(FLANN REQUIRED)

find_package(realsense2 2.50.0)

find_package(spdlog REQUIRED)

add_subdirectory(src)

macro(print_all_variables)
    message(STATUS "print_all_variables------------------------------------------{")
    get_cmake_property(_variableNames VARIABLES)
    foreach (_variableName ${_variableNames})
        message(STATUS "${_variableName}=${${_variableName}}")
    endforeach()
    message(STATUS "print_all_variables------------------------------------------}")
endmacro()

#print_all_variables()
