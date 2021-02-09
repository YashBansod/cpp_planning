cmake_minimum_required(VERSION 3.15)
project(cpp_robotics)

set(CMAKE_CXX_STANDARD 17)

include_directories(${CMAKE_SOURCE_DIR})

add_executable(cpp_robotics main.cpp)
add_executable(kf_example kalman_filter/kf_example.cpp kalman_filter/KalmanFilter.h kalman_filter/KalmanFilter.cpp
        synthetic_data/traj_2d_monte_carlo.cpp synthetic_data/traj_2d_monte_carlo.h)