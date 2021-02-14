cmake_minimum_required(VERSION 3.15)
project(a_star)

set(CMAKE_CXX_STANDARD 17)

set(BOOST_ROOT "/home/yashb/cpp_libs/boost_1_75_0")

find_package(Boost 1.75.0 COMPONENTS graph REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(a_star a_star.cpp)