cmake_minimum_required(VERSION 3.8.0)
 
project(test)
 
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
 
find_package(Ceres REQUIRED)
 
include_directories(
  ${CERES_INCLUDE_DIRS}
)
 
add_executable(test
test.cpp)
 
target_link_libraries(test
    ${CERES_LIBRARIES}
)