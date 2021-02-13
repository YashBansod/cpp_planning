/*!
 * @brief
 *
 * @file
 *
 * @ingroup     synthetic_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include "traj_2d_monte_carlo.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main() {
    Eigen::MatrixXf traj_2d = s_data::traj_2d_monte_carlo(10);
    std::cout << "Random Trajectory: \n" << traj_2d << std::endl;
    return 0;
}
