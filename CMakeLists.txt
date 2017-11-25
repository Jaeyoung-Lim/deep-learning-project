cmake_minimum_required(VERSION 3.5)
project(raiApp)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_COMPILER "/usr/bin/g++-6")
set(CMAKE_CXX_FLAGS "-Wl,--no-as-needed -fopenmp -O3 -w -funroll-loops")

find_package(RAI REQUIRED)
include_directories(${TENSORFLOW_EIGEN_DIR})
include_directories(${RAI_INCLUDE_DIR})

# Search for raiCommon
find_package(raiCommon CONFIG REQUIRED)
include_directories(${RAI_COMMON_INCLUDE_DIR})

# Search for raiGraphics
find_package(raiGraphicsOpengl CONFIG REQUIRED)
include_directories(${RAI_GRAPHICS_OPENGL_INCLUDE_DIRS})

include_directories(Task/include)
add_subdirectory(Task/src/quadrotor)

add_subdirectory(applications/quadrotorwithTRPO)
#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/applications/${RAI_APP})