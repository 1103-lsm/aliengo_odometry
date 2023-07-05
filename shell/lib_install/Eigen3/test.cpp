#include <iostream>
#include <Eigen/Dense>

int main() {
    Eigen::MatrixXd m(2, 2);
    m(0, 0) = 1.0;
    m(0, 1) = 2.0;
    m(1, 0) = 3.0;
    m(1, 1) = 4.0;

    std::cout << "Matrix m:" << std::endl;
    std::cout << m << std::endl;

    return 0;
}