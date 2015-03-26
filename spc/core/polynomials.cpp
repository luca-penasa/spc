#include "polynomials.hpp"
#include <vector>
#include <iostream>
#include <spc/core/macros.h>
namespace spc
{

////////////////////////////////////// INSTANTIATION OF COMMON TYPES
template void pow_vector<float>(const std::vector<float> &x, const float &power,
                                std::vector<float> &out_vector);
template void pow_vector
    <double>(const std::vector<double> &x, const double &power,
             std::vector<double> &out_vector);

template int vander(const std::vector<float> &x, const int &n_cols,
                    Eigen::Matrix
                    <float, Eigen::Dynamic, Eigen::Dynamic> &vandermat);

template SPC_LIB_API 
void lstsq(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &A,
           const Eigen::Matrix<float, Eigen::Dynamic, 1> &b,
           Eigen::Matrix<float, Eigen::Dynamic, 1> &x);


template  int  polyfit(const std::vector<float> &x, const std::vector<float> &y, const int &deg, std::vector<float> &poly);




template Eigen::Matrix<float, -1, 1> getPolynomialVariables(const Eigen::Matrix<float, -1, 1> &vars,
                                                        const size_t &degree);


template Eigen::Matrix<double, -1, 1> getPolynomialVariables(const Eigen::Matrix<double, -1, 1> &vars,
                                                        const size_t &degree);

} //end nspace
