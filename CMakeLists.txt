cmake_minimum_required(VERSION 2.8.3)
project(kalman_filter)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
SET(CMAKE_CXX_FLAGS "-std=c++11 -O3 -g -Wall ${CMAKE_CXX_FLAGS}")

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR})

find_package(Eigen3 REQUIRED)


include_directories(

  ${Eigen3_INCLUDE_DIR}

)


add_executable(kalman_filter main.cpp kalman_filter.hpp)




 
