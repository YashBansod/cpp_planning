cmake_minimum_required(VERSION 3.15)
project(program_options_sandbox)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

set(BOOST_ROOT "/home/yashb/cpp_libs/boost_1_75_0")

find_package(Boost 1.75.0 COMPONENTS program_options REQUIRED)
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

add_executable(first first.cpp)
target_link_libraries(first Boost::program_options)

add_executable(options_description options_description.cpp)
target_link_libraries(options_description Boost::program_options)