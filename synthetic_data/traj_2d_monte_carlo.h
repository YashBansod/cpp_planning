/*!
 * @brief
 *
 * @file
 *n
 * @igroup     synthetic_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"
#include "random_data/random_matrix.h"

/*------------------------------------------        Using Statements      `------------------------------------------*/
using namespace std;
using Eigen::Matrix;
using Eigen::MatrixXf;

/*------------------------------------------        Func Definition         ------------------------------------------*/
#ifndef CPP_ROBOTICS_TRAJ_2D_MONTE_CARLO
#define CPP_ROBOTICS_TRAJ_2D_MONTE_CARLO

MatrixXf traj_2d_monte_carlo(const int traj_length){
    MatrixXf traj = random_uniform_mat(2, traj_length, 0, 100);
    return traj;
}

#endif //CPP_ROBOTICS_TRAJ_2D_MONTE_CARLO