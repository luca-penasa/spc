#ifndef SPC_POLYNOMIALS_HPP
#define SPC_POLYNOMIALS_HPP

#include <spc/methods/polynomials.h>
#include <iostream>

namespace spc
{

template <typename ScalarT>
int polyfit(const std::vector<ScalarT> &x, const std::vector<ScalarT> &y,
            const int &deg, std::vector<ScalarT> &poly)
{
    // TODO introduce a scaling as in numpy
    const int order = deg + 1;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> vandermat;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> solution;
    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> y_eigen = Eigen::Matrix
        <ScalarT, Eigen::Dynamic, 1>::Map(y.data(), y.size());

    vander(x, order, vandermat);

    lstsq<float>(vandermat, y_eigen, solution);

    // remap solution to std::vector
    poly.resize(solution.rows());

    Eigen::Matrix<ScalarT, Eigen::Dynamic, 1>::Map(poly.data(), poly.size())
        = solution;

    return 1;
}

template <typename ScalarT>
int vander(const std::vector<ScalarT> &x, const int &n_cols,
           Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &vandermat)
{
    const int n_rows = x.size();

    vandermat.resize(n_rows, n_cols);

    for (int i = 0; i < n_cols - 1; ++i) // for each column in out matrix
    {
        std::vector<ScalarT> powered;
        pow_vector(x, (ScalarT)n_cols - 1 - i, powered);

        // write in matrix
        for (int j = 0; j < n_rows; ++j) {
            vandermat(j, i) = powered[j];
        }
    }

    vandermat.col(n_cols - 1).fill(1);

    return 1;
}

template <typename ScalarT>
void pow_vector(const std::vector<ScalarT> &x, const ScalarT &power,
                std::vector<ScalarT> &out_vector)
{
    out_vector.resize(x.size());
    for (int i = 0; i < x.size(); ++i) {
        out_vector[i] = pow(x[i], power);
    }
}

template <typename ScalarT>
void lstsq(const Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &A,
           const Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &b,
           Eigen::Matrix<ScalarT, Eigen::Dynamic, 1> &x)
{

    if (A.rows() == A.cols()) {
        // solve via pivoting
        x = A.colPivHouseholderQr().solve(b);
    } else if (A.rows() > A.cols()) {
        // solving via SVD
        x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
    } else {
        x.fill(std::numeric_limits<ScalarT>::quiet_NaN());
        std::cout << "Error solving linear system" << std::endl;
        return;
    }
}


template <typename T>
Eigen::Matrix<T, -1, 1> getPolynomialVariables(const Eigen::Matrix<T, -1, 1> &vars,
                                               const size_t & degree)
{


    typedef Eigen::Matrix<T, -1, 1> Vector;
    typedef Eigen::Matrix<T, -1, -1, Eigen::RowMajor> Matrix;

    auto expand_to_degree = [](const float &x, const int &degree)
    {
        Vector out(degree+1);

        for (int i = 0; i <= degree ; ++i)
            out(i)  = pow(x, i);

        return out;
    };


    Vector current (1);
    current << 1;

    for (int i = 0; i < vars.rows() ; ++i)
    {
        // we expand to the given degree the variable
        Vector expanded = expand_to_degree(vars(i), degree);

        Matrix mul = current * expanded.transpose();


        current.resize(mul.size());

        current = Eigen::Map<Vector> (mul.data(), mul.size());

    }

    return current;

}
}

#endif // POLYNOMIALS_HPP
