cmake_minimum_required(VERSION 3.0.0)
project(test)

# 设置构建类型为Release
set(CMAKE_BUILD_TYPE "Release")

# 设置C++标准为C++14
set(CMAKE_CXX_STANDARD 14)

# 设置Release编译选项
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

# 查找casadi库
find_package(casadi REQUIRED)

# 添加casadi库的头文件目录
include_directories(${CASADI_INCLUDE_DIR})

# 编译可执行文件test
add_executable(test test.cpp)

# 链接casadi库
target_link_libraries(test casadi)
