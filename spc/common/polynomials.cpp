#include "polynomials.h"
#include <vector>
#include <iostream>
namespace spc
{

template<typename ScalarT>
int
polyfit(const std::vector<ScalarT> &x, const std::vector<ScalarT> &y, const int &deg, std::vector<ScalarT> &poly)
{
    //TODO introduce a scaling as in numpy
    const int order = deg + 1;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> vandermat;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> solution;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> y_eigen = Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>::Map(y.data(), y.size());


    vander(x, order, vandermat);
    std::cout << vandermat << std::endl;

    lstsq<float>(vandermat, y_eigen, solution);


    //remap solution to std::vector
    poly.resize(solution.rows());

    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>::Map(poly.data(), poly.size()) = solution;

    return 1;
}


template<typename ScalarT>
int
vander(const std::vector<ScalarT> &x, const int &n_cols, Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &vandermat)
{
    const int n_rows = x.size();

    vandermat.resize(n_rows, n_cols);


    for (int i = 0; i < n_cols -1 ; ++i ) //for each column in out matrix
    {
        std::vector<ScalarT> powered;
        pow_vector(x, (ScalarT) n_cols - 1 - i, powered);

        //write in matrix
        for (int j = 0; j < n_rows; ++j)
        {
            vandermat(j, i) = powered[j];
        }

    }

    vandermat.col(n_cols-1).fill(1);

    return 1;

}

template<typename ScalarT>
void
pow_vector(const std::vector<ScalarT> &x, const ScalarT &power, std::vector<ScalarT> &out_vector)
{
    out_vector.resize(x.size());
    for (int i = 0; i < x.size(); ++i)
    {
        out_vector[i] = pow(x[i], power);
    }

}


template<typename ScalarT>
void
lstsq(const Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &A,
      const Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &b,
      Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &x)
{


    if ( A.rows() == A.cols() )
    {
        //solve via pivoting
        x = A.colPivHouseholderQr().solve(b);
    }
    else if ( A.rows() > A.cols() )
    {
        //solving via SVD
        x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) ;
    }
    else
    {
        x.fill(std::numeric_limits<ScalarT>::quiet_NaN());
        std::cout << "Error solving linear system" << std::endl;
        return;
    }

}
////////////////////////////////////// INSTANTIATION OF COMMON TYPES
template void pow_vector<float>(const std::vector<float> &x, const float &power, std::vector<float> &out_vector);
template void pow_vector<double>(const std::vector<double> &x, const double &power, std::vector<double> &out_vector);

template
int
vander(const std::vector<float> &x, const int &n_cols, Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &vandermat);

template<typename ScalarT>
void
lstsq(const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> &A,
      const Eigen::Matrix<float, Eigen::Dynamic, 1> &b,
      Eigen::Matrix<float, Eigen::Dynamic, 1> &x);

template
int
polyfit(const std::vector<float> &x, const std::vector<float> &y, const int &deg, std::vector<float> &poly);


} //end namespca
