/*!
 * @brief
 *
 * @file
 *
 * @igroup     synthetic_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"
#include "random_data/random_mat.h"

/*------------------------------------------        Func Definition         ------------------------------------------*/
#ifndef SYNTHETIC_DATA_TRAJ_2D_MONTE_CARLO_H
#define SYNTHETIC_DATA_TRAJ_2D_MONTE_CARLO_H

namespace s_data{
    Eigen::MatrixXf traj_2d_monte_carlo(const int traj_length){
        Eigen::MatrixXf traj = r_data::random_uniform_mat(2, traj_length, 0, 100);
        return traj;
    }
}

#endif //SYNTHETIC_DATA_TRAJ_2D_MONTE_CARLO_H
