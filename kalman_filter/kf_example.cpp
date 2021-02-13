/*!
 * @brief
 *
 * @file
 *
 * @ingroup     kalman_filter
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include "Eigen/Dense"
#include "kalman_filter.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main() {
    std::cout << "Hello, World! This is the Kalman Filter Example." << std::endl;

    kf::kalman_filter_2d kf_2d;
    std::cout << "\nState Transition Function:\n" << kf_2d.get_state_fn() << std::endl;
    Eigen::Matrix<float, 4, 1> state = kf_2d.get_state();
    Eigen::Matrix<float, 4, 4> covariance = kf_2d.get_covariance();
    std::cout << "Step 0, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    kf_2d.predict();
    state = kf_2d.get_state();
    covariance = kf_2d.get_covariance();
    std::cout << "Step 0p, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    Eigen::Matrix<float, 2, 1> measurement;
    measurement << 4, 2;
    kf_2d.update(measurement);
    state = kf_2d.get_state();
    covariance = kf_2d.get_covariance();
    std::cout << "Step 1, \nState:\n" << state << "\nCovariance:\n" << covariance << std::endl;

    return 0;
}
