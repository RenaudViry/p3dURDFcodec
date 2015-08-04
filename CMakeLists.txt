cmake_minimum_required(VERSION 2.6)
project(p3dURDFcodec CXX)

set(CMAKE_CXX_FLAGS "-Wall -std=c++0x")
set(CMAKE_BUILD_TYPE "debug")
set(EXECUTABLE_OUTPUT_PATH ..)

find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
#pkg_check_modules(tinyxml2 REQUIRED tinyxml2)
pkg_check_modules(assimp REQUIRED assimp>=3.1)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${assimp_INCLUDE_DIRS})

message(STATUS "Path to Assimp lib : ${assimp_LIBRARY_DIRS}")
message(STATUS "Path to Assimp headers : ${assimp_INCLUDE_DIRS}")
message(STATUS "Path to Eigen headers :${EIGEN3_INCLUDE_DIR}")

#file(GLOB p3dURDFcodec_SOURCES *.c)
set(p3dURDFcodec_SOURCES main.cpp
data.cpp
p3d-codec.cpp
urdf-codec.cpp
tinyXML2/tinyxml2.cpp)

link_directories(${assimp_LIBRARY_DIRS})
add_executable(p3dURDFcodec ${p3dURDFcodec_SOURCES})
target_link_libraries(p3dURDFcodec ${assimp_LIBRARIES}) #${tinyxml2_LIBRARIES} 
