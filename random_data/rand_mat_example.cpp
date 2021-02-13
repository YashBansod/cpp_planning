/*!
 * @brief
 *
 * @file
 *
 * @ingroup     random_data
 */

/*------------------------------------------        Include Files           ------------------------------------------*/
#include <iostream>
#include "random_mat.h"

/*------------------------------------------        Main Function           ------------------------------------------*/
int main() {
    Eigen::MatrixXf rand_mat = r_data::random_uniform_mat(6, 6, -10, 10);
    std::cout << "Random Matrix: \n" << rand_mat << std::endl;
    return 0;
}
