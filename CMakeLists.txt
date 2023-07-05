cmake_minimum_required(VERSION 3.0.2)
project(ilo)

set(CMAKE_BUILD_TYPE "Release")
#set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_FLAGS "-std=c++14")
#-DEIGEN_USE_MKL_ALL")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    geometry_msgs
    nav_msgs
    tf
)

# yaml-cpp
find_package(yaml-cpp REQUIRED)
include_directories(${YAML_CPP_INCLUDE_DIR})

find_package(Eigen3)
include_directories(
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# search for casadi in the environment
find_package(casadi REQUIRED) 

# Find the savitzky golay filter package and all its dependencies 
find_package(gram_savitzky_golay REQUIRED)

# find_package(Ceres REQUIRED)
#message(WARNING "CERES_LIBRARIES: ${CERES_LIBRARIES}")
# include_directories(${catkin_INCLUDE_DIRS} ${CERES_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
 
include_directories(
  ${CERES_INCLUDE_DIRS}
)



catkin_package()

include_directories(
${catkin_INCLUDE_DIRS}
)

add_library(ilo_lib
  src/utils/parameters.cpp
  src/utils/utility.cpp
  src/utils/visualization.cpp

  src/legKinematics/A1Kinematics.cpp
  src/estimator/estimator.cpp

  # src/factor/imu_leg_factor.cpp 
  # src/factor/imu_leg_factor.h 
  src/factor/imu_leg_integration_base.cpp 
  src/factor/imu_leg_integration_base.h
)

# kf subdirectory
add_subdirectory(src/kalmanFilter)

target_link_libraries(ilo_lib yaml-cpp ${catkin_LIBRARIES}   ${CERES_LIBRARIES})

add_executable(ilo
  src/main.cpp
)
target_link_libraries(ilo
  ilo_lib
  kf_lib
  casadi
  gram_savitzky_golay::gram_savitzky_golay

)
