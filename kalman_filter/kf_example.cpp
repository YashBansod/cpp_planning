#include <iostream>
#include "Eigen/Dense"
#include "KalmanFilter.h"

using namespace Eigen;

int main() {
    std::cout << "Hello, World! This is the Kalman Filter Example." << std::endl;

    KalmanFilter2D kf_2d;
    std::cout << "\nState Transition Function:\n" << kf_2d.get_state_fn() << std::endl;
    Matrix<float, 4, 1> state = kf_2d.get_state();
    Matrix<float, 4, 4> covariance = kf_2d.get_covariance();
    std::cout << "Step 0, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    kf_2d.predict();
    state = kf_2d.get_state();
    covariance = kf_2d.get_covariance();
    std::cout << "Step 0p, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    Matrix<float, 2, 1> measurement;
    measurement << 4, 2;
    kf_2d.update(measurement);
    state = kf_2d.get_state();
    covariance = kf_2d.get_covariance();
    std::cout << "Step 1, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    return 0;
}
