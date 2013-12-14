#ifndef EIGEN_EXTENSIONS_H
#define EIGEN_EXTENSIONS_H

#include <vector>
#include <iostream>
#include <Eigen/Dense>


namespace spc
{


template<typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
vander(const Eigen::MatrixBase<Derived> &x, int n_cols = 0)
{
    if (n_cols == 0)
        n_cols = x.size();

    const int n_rows = x.size();

    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> vandermat;

    vandermat.resize(n_rows, n_cols);

    for (int i =0; i < n_cols ; ++i ) //for each column in out matrix
    {
        vandermat.col(n_cols - i - 1).array() = x.array().pow(i);

    }

    vandermat.col(n_cols-1).fill(1);
    return vandermat;
}

template<typename Derived, typename Derived2>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
lstsq(const Eigen::MatrixBase<Derived> &A,
      const Eigen::MatrixBase<Derived2> &b)
{

    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> x;


    if ( A.rows() == A.cols() )
    {
        //solve via pivoting
        x = A.fullPivLu().solve(b);
        return x;

    }
     else if ( A.rows() >= A.cols() )
    {
        //solving via SVD
        x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b) ;
        return x;
    }
    else
    {
        x.fill(std::numeric_limits<typename Derived::Scalar>::quiet_NaN());
        std::cout << "Error solving linear system" << std::endl;
        return x;
    }

}

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
    polyfit(const Eigen::MatrixBase<Derived> &x, const Eigen::MatrixBase<Derived> &y, const int &deg)
    {
        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> x_tmp = x;

        typename Derived::Scalar scale = x.array().abs().maxCoeff(); //to improve condition number
        x_tmp /= scale;

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> van = vander(x_tmp, deg+1);

        //scale back, reconstruct a scaling vector
        Eigen::Matrix<typename Derived::Scalar, 1, 1> tmp_s;
        tmp_s(0) = scale;

        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> scalevan = vander(tmp_s, deg+1);

        //solve the system
        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> sol = lstsq(van, y);

        //effective scaling back.
        sol.array() /= scalevan.transpose().array();
        return  sol;
    }

    template<typename Derived>
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
    polyval(const Eigen::MatrixBase<Derived> &pol, const Eigen::MatrixBase<Derived> &x)
    {
        Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> y;
        y.resize(x.size());
        y.fill(0);


        for (int i = 0; i < pol.array().size();  ++i)
            y = x.array() * y.array() + pol.array().coeff(i);

        return y;


    }








//    template<typename ScalarT>
//    std::vector<ScalarT>
//    eigenToStdVector(Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic> &mat)
//    {
//        std::vector<ScalarT> vec;
//        vec.resize(mat.size());
//        Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>::Map(vec.data(), vec.size()) = mat;
//        return vec;
//    }

    template<typename Derived>
    std::vector<typename Derived::Scalar>
    eigenToStdVector(Eigen::MatrixBase<Derived> &mat)
    {
        std::vector<typename Derived::Scalar> vec;
        vec.resize(mat.array().size());
        for (int i = 0 ; i < mat.array().size(); ++i)
            vec[i] = mat.array().coeff(i);

        return vec;
    }


    Eigen::Matrix<float, Eigen::Dynamic, 1>
    stdVectorToEigen(const std::vector<float> &vec)
    {

        return Eigen::Matrix< float, Eigen::Dynamic, 1>::Map(vec.data(), vec.size()) ;

    }

    Eigen::Matrix<int, Eigen::Dynamic, 1>
    stdVectorToEigen(const std::vector<int> &vec)
    {

        return Eigen::Matrix< int, Eigen::Dynamic, 1>::Map(vec.data(), vec.size()) ;

    }

    Eigen::Matrix<double, Eigen::Dynamic, 1>
    stdVectorToEigen(const std::vector<double> &vec)
    {

        return Eigen::Matrix< double, Eigen::Dynamic, 1>::Map(vec.data(), vec.size()) ;

    }


}//end namespace ll

#endif // EIGEN_EXTENSIONS_H
