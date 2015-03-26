#ifndef SPC_POLYNOMIALS_H
#define SPC_POLYNOMIALS_H

#include <spc/core/spc_eigen.h>
#include <spc/core/logging.h>
#include <vector>

namespace spc
{

template <typename ScalarT>
int polyfit(const typename std::vector<ScalarT> &x,
            const typename std::vector<ScalarT> &y, const int &deg,
            std::vector<ScalarT> &poly);

template <typename ScalarT>
int vander(const std::vector<ScalarT> &x, const int &order,
           Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &vandermat);

template <typename ScalarT>
void pow_vector(const std::vector<ScalarT> &x, const ScalarT &power,
                std::vector<ScalarT> &out_vector);

template <typename ScalarT> 
void lstsq(const Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &A,
           const Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &b,
           Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &x);

//! vars is a vector of type [x,y,...]^T with all the variables
//! it returns a vector of type (e.g. for a second order with two variables x,y):
//! [x^2 * y^2,
//!  x^2 * y^1,
//!  x^2 * y^0,
//!  x^1 * y^2,
//!  x^1 * y^1,
//!  x^1 * y^0,
//!  x^0 * y^2,
//!  x^0 * y^1,
//!  x^0 * y^0]


template <typename T>
Eigen::Matrix<T, -1, 1> getPolynomialVariables(const Eigen::Matrix<T, -1, 1> &vars, const size_t &degree);
}
#endif // POLYNOMIALS_H
