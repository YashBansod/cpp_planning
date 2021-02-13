cmake_minimum_required(VERSION 3.15)
project(a_star)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})
include_directories("D:/ClionProjects/libs/boost_1_75_0")

add_executable(a_star main.cpp)