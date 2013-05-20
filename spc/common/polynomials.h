#ifndef SPC_POLYNOMIALS_H
#define SPC_POLYNOMIALS_H

#include <Eigen/Dense>

namespace ll
{

    template<typename ScalarT>
    int
    polyfit(const std::vector<ScalarT> &x, const std::vector<ScalarT> &y, const int &deg, std::vector<ScalarT> &poly);




    template<typename ScalarT>
    int
    vander(const std::vector<ScalarT> &x, const int &order, Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &vandermat);



    template<typename ScalarT>
    void
    pow_vector(const std::vector<ScalarT> &x, const ScalarT &power, std::vector<ScalarT> &out_vector);


    template<typename ScalarT>
    void
    lstsq(const Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &A,
          const Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &b,
          Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &x);

}
#endif // POLYNOMIALS_H
