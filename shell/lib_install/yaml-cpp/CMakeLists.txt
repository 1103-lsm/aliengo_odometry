cmake_minimum_required(VERSION 3.12)
project(test)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})
message(STATUS "Found yaml-cpp: ${YAML_CPP_FOUND}")
message(STATUS "yaml-cpp include directory: ${YAML_CPP_INCLUDE_DIR}")
message(STATUS "yaml-cpp libraries: ${YAML_CPP_LIBRARIES}")

add_executable(test test.cpp)
target_link_libraries(test yaml-cpp)
