cmake_minimum_required(VERSION 3.15)
project(synthetic_data)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../)
include_directories("D:/ClionProjects/libs/eigen-3.3.9")

add_executable(traj_example traj_2d_example.cpp)