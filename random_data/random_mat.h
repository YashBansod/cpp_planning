/*!
 * @brief
 *
 * @file
 *
 * @ingroup     random_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"

/*------------------------------------------        Func Definition         ------------------------------------------*/
#ifndef RANDOM_DATA_RANDOM_MAT_H
#define RANDOM_DATA_RANDOM_MAT_H

namespace r_data {
    Eigen::MatrixXf random_uniform_mat(const int row, const int col, const float low, const float high) {
        float range_div_2 = (high - low) / 2;
        Eigen::MatrixXf mat = ((Eigen::MatrixXf::Random(row, col).array() + 1) * range_div_2).array() + low;
        return mat;
    }
}

#endif //RANDOM_DATA_RANDOM_MAT_H
