cmake_minimum_required(VERSION 3.15)
project(rrt_template_exp)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

set(BOOST_ROOT "/home/yashb/cpp_libs/boost_1_75_0")

find_package(Boost 1.75.0 COMPONENTS program_options graph REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

set(Eigen3_DIR /home/yashb/cpp_libs/eigen-3.3.9)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(rrt src/rrt.cpp)
target_link_libraries(rrt Boost::program_options)