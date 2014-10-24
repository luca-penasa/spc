#ifndef EIGEN_EXTENSIONS_H
#define EIGEN_EXTENSIONS_H


#include <spc/methods/spc_eigen.h>


namespace spc
{

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic>
vander(const Eigen::MatrixBase<Derived> &x, int n_cols = 0)
{
    if (n_cols == 0)
        n_cols = x.size();

    const int n_rows = x.size();

    Eigen::Matrix
        <typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> vandermat;

    vandermat.resize(n_rows, n_cols);

    for (int i = 0; i < n_cols; ++i) // for each column in out matrix
    {
        vandermat.col(n_cols - i - 1).array() = x.array().pow(i);
    }

    vandermat.col(n_cols - 1).fill(1);
    return vandermat;
}

template <typename Derived, typename Derived2>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
lstsq(const Eigen::MatrixBase<Derived> &A, const Eigen::MatrixBase<Derived2> &b)
{

    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> x;

    if (A.rows() == A.cols()) {
        // solve via pivoting
        x = A.fullPivLu().solve(b);
        return x;

    } else if (A.rows() >= A.cols()) {
        // solving via SVD
        x = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);
        return x;
    } else {
        x.fill(std::numeric_limits<typename Derived::Scalar>::quiet_NaN());
        std::cout << "Error solving linear system" << std::endl;
        return x;
    }
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
polyfit(const Eigen::MatrixBase<Derived> &x,
        const Eigen::MatrixBase<Derived> &y, const int &deg)
{
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> x_tmp = x;

    typename Derived::Scalar scale
        = x.array().abs().maxCoeff(); // to improve condition number
    x_tmp /= scale;

    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> van
        = vander(x_tmp, deg + 1);

    // scale back, reconstruct a scaling vector
    Eigen::Matrix<typename Derived::Scalar, 1, 1> tmp_s;
    tmp_s(0) = scale;

    Eigen::Matrix
        <typename Derived::Scalar, Eigen::Dynamic, Eigen::Dynamic> scalevan
        = vander(tmp_s, deg + 1);

    // solve the system
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> sol
        = lstsq(van, y);

    // effective scaling back.
    sol.array() /= scalevan.transpose().array();
    return sol;
}

template <typename Derived>
Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1>
polyval(const Eigen::MatrixBase<Derived> &pol,
        const Eigen::MatrixBase<Derived> &x)
{
    Eigen::Matrix<typename Derived::Scalar, Eigen::Dynamic, 1> y;
    y.resize(x.size());
    y.fill(0);

    for (int i = 0; i < pol.array().size(); ++i)
        y = x.array() * y.array() + pol.array().coeff(i);

    return y;
}

//    template<typename ScalarT>
//    std::vector<ScalarT>
//    eigenToStdVector(Eigen::Matrix<ScalarT, Eigen::Dynamic, Eigen::Dynamic>
// &mat)
//    {
//        std::vector<ScalarT> vec;
//        vec.resize(mat.size());
//        Eigen::Matrix<ScalarT, Eigen::Dynamic,
// Eigen::Dynamic>::Map(vec.data(), vec.size()) = mat;
//        return vec;
//    }

template <typename Derived>
std::vector<typename Derived::Scalar> eigenToStdVector(Eigen::MatrixBase
                                                       <Derived> &mat)
{
    std::vector<typename Derived::Scalar> vec;
    vec.resize(mat.array().size());
    for (int i = 0; i < mat.array().size(); ++i)
        vec[i] = mat.array().coeff(i);

    return vec;
}

//Eigen::Matrix<float, Eigen::Dynamic, 1> stdVectorToEigen(const std::vector
//                                                         <float> &vec)
//{

//    return Eigen::Matrix<float, Eigen::Dynamic, 1>::Map(vec.data(), vec.size());
//}

//Eigen::Matrix<int, Eigen::Dynamic, 1> stdVectorToEigen(const std::vector
//                                                       <int> &vec)
//{

//    return Eigen::Matrix<int, Eigen::Dynamic, 1>::Map(vec.data(), vec.size());
//}

//Eigen::Matrix<double, Eigen::Dynamic, 1> stdVectorToEigen(const std::vector
//                                                          <double> &vec)
//{

//    return Eigen::Matrix
//        <double, Eigen::Dynamic, 1>::Map(vec.data(), vec.size());
//}


/**
 * \brief this is a special version of replicate() which includes a multiciplity factor for each
 * elements. See examples... This method is useful for constructing meshgrid-like methods
 *
 * see also meshgrid
 *
 * Given a vector v = [1,2,3]^T
 * multiciplity 3 and total size 9. output will be:
 * out = [1,1,1,2,2,2,3,3,3]^T
 * if multiplicity is 2 and total size 12:
 * out = [1,1,2,2,3,3,1,1,2,2,3,3]^T
 * routine checks that the total_size / v.rows() * multiplicity is zero;
 */
template <typename T>
Eigen::Matrix<T, -1, 1> replicate(const Eigen::Matrix<T, -1, 1> &v, /**< [in] input vector */
                                  const size_t &multiplicity, /**< [in] repetition of the single elements */
                                  const size_t &total_size) /**< [in] until this total size is obtained */
{
    CHECK(total_size % (multiplicity * v.rows()) == 0) << "total size must be multiple of multiplicity * number of elements in vector";
    DLOG(INFO) << total_size / multiplicity << " fragments";

    Eigen::Matrix<T, -1, -1> m = v.replicate(1, multiplicity);

    Eigen::Matrix<T, -1, -1> m2 = m.replicate(total_size/ (multiplicity *v.rows()), 1).transpose();

    Eigen::Matrix<T, -1, 1> out = Eigen::Map<Eigen::Matrix<T, -1, 1>>(m2.data(), m2.size());

//    DLOG(INFO) << "replicated " << out;

    return out;

}


/**
 * Emulates the meshgrid function found in matlab or python
 * Given a set of vectors it expand the coordinates in a grid.
 *
 * It works for any arbitraty number of input vectors (each will represent a dimension)
 * in the n-dimensional space on whih the grid will be constructed
 *
 * e.g. xyz = {[1,2,3]^T, [4,5]^T}
 *
 * output will be:
 * out = [[1,4].
 *        [1,5],
 *        [2,4],
 *        [2,5],
 *        [3,4],
 *        [3,5]]
 *
 * see also replicate()
 */
template<typename T>
Eigen::Matrix<T, -1, -1> meshgrid(const std::vector<Eigen::Matrix<T, -1, 1>> &xyz)
{
    DLOG(INFO) << "meshgridding";

    Eigen::VectorXi sizes_of_vectors(xyz.size());
    for (int i = 0 ; i < xyz.size(); ++i)
    {
        sizes_of_vectors(i) = xyz.at(i).rows();
    }

    size_t out_rows = sizes_of_vectors.array().prod();

    DLOG(INFO) << "init out matrix";

    Eigen::Matrix<T, -1, -1> out(out_rows, xyz.size());

    DLOG(INFO) << "done";

    size_t multiplicity = 1;
    for (int col_id = xyz.size() - 1; col_id >= 0 ; --col_id)
    {
        DLOG(INFO) << "saving replica in column " << col_id;

        Eigen::Matrix<T, -1, 1> rep = replicate<T>(xyz.at(col_id), multiplicity , out_rows);

        DLOG(INFO) << "replica is " << rep.rows() << " in size ";
        DLOG(INFO) << "out is is " << out.rows() << " in size ";


        out.col(col_id) = rep;

        multiplicity *= sizes_of_vectors(col_id);
    }

    DLOG(INFO) << "Done";
    return out;

}





} // end namespace ll

#endif // EIGEN_EXTENSIONS_H
