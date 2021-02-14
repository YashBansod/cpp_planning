cmake_minimum_required(VERSION 3.15)
project(random_data)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

set(Eigen3_DIR /home/yashb/cpp_libs/eigen-3.3.9)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(random_example rand_mat_example.cpp)
target_link_libraries(random_example Eigen3::Eigen)