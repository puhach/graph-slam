cmake_minimum_required(VERSION 3.10)

project(GraphSlamConsole VERSION 1.0)

set(CMAKE_CXX_STANDARD 17)
#set(CMAKE_CXX_STANDARD_REQUIRED True)

#list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/eigen/cmake")

#message("${CMAKE_MODULE_PATH}")

set(EIGEN3_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/eigen")
#set(Eigen3_DIR "${CMAKE_CURRENT_LIST_DIR}/eigen/cmake")

#message("${Eigen3_DIR}")

#find_package(Eigen3 3.3 REQUIRED NO_MODULE)
#find_package(Eigen3 3.3 REQUIRED)
#include_directories(${EIGEN3_INCLUDE_DIR})



#message("${CMAKE_MODULE_PATH}")
#message("${EIGEN3_INCLUDE_DIR}")

add_executable(GraphSlamConsole main.cpp world.h world.cpp robot.h robot.cpp graphslam.h graphslam.cpp lkdistance.h measurement.h position.h)

target_include_directories(GraphSlamConsole PUBLIC "${EIGEN3_INCLUDE_DIR}")
                           
#target_link_libraries(GraphSlamConsole  PUBLIC  Eigen)
#target_link_libraries(GraphSlamConsole Eigen3::Eigen)