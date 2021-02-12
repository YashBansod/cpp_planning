/*!
 * @brief
 *
 * @file
 *
 * @ingroup     random_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include "Eigen/Dense"

/*------------------------------------------        Using Statements        ------------------------------------------*/
using namespace std;
using Eigen::Matrix, Eigen::MatrixXf, Eigen::Array, Eigen::ArrayXXf;

/*------------------------------------------        Func Definition         ------------------------------------------*/
#ifndef CPP_ROBOTICS_RANDOM_MATRIX
#define CPP_ROBOTICS_RANDOM_MATRIX

MatrixXf random_uniform_mat(const int row, const int col, const float low, const float high) {
    float range_div_2 = (high - low) / 2;
    MatrixXf mat = ((MatrixXf::Random(row, col).array() + 1) * range_div_2).array() + low;
    return mat;
}

//Matrix<float, 100, 100 random_uniform_mat(const int row, const int col, const float low, const float high) {
//    float range_div_2 = (high - low) / 2;
//    Matrix<float, 100, 100> mat = ((Matrix<float, 100, 100>::Random().array() + 1) * range_div_2).array() + low;
//    return mat;
//}

#endif //CPP_ROBOTICS_RANDOM_MATRIX
