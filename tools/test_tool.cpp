
#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <unsupported/Eigen/AutoDiff>


int main()
{
    Eigen::Matrix<float, -1,-1> m = Eigen::MatrixXf::Random(5,5);

    std::cout <<"MAT \n" << m <<std::endl;

    Eigen::VectorXf v = m;

    std::cout <<"VEC \n" << v <<std::endl;
}
