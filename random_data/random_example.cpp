#include <iostream>
#include "random_matrix.h"

int main() {
    MatrixXf rand_mat = random_uniform_mat(6, 6, -10, 10);
    std::cout << "Random Matrix: \n" << rand_mat << std::endl;
    return 0;
}
