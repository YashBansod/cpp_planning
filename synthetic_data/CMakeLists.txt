cmake_minimum_required(VERSION 3.15)
project(synthetic_data)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/../)

set(Eigen3_DIR /home/yashb/cpp_libs/eigen-3.3.9)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

add_executable(traj_example traj_2d_example.cpp)
target_link_libraries(traj_example Eigen3::Eigen)