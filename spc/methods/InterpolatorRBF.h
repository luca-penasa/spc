#ifndef SPC_RBF_INTERPOLATOR_H
#define SPC_RBF_INTERPOLATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>


namespace spc
{


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
template<typename T>
Eigen::Matrix<T, -1, 1> getPolynomialVariables(const Eigen::Matrix<T, -1, 1> &vars, const size_t order)
{
    size_t n_vars = vars.rows();
    Eigen::Matrix<T, -1, -1> matrix_of_powers;
    matrix_of_powers.resize(order + 1, n_vars);

    matrix_of_powers.row(0) = Eigen::Matrix<T, -1, 1>::Zeros(n_vars);
    matrix_of_powers.row(1) = Eigen::Matrix<T, -1, 1>::Ones(n_vars);

    for (int i  = 2; i <order ; ++i)
    {
        matrix_of_powers.row(i) = vars.array().pow(i);
    }


}


//! D is the number of dimensions
//! on which interpolation is performed
template <typename T, size_t DIM = 1>
class InterpolatorRBF
{

    typedef Eigen::Matrix<T, -1, DIM> PointsT;
    typedef Eigen::Matrix<T, -1, 1> VectorT;
    typedef Eigen::Matrix<T, DIM, 1> PointT;

public:
    InterpolatorRBF() {}

    T evaluateRbf(PointT &point)
    {

    }

    void setPoints(PointsT &points)
    {
        points_ = points;
    }

    //! if nodes are not provided all the input points will be used
    //! this means we will get a RB node for each point
    //! if you set the nodes you will get a smoothing rbf on those nodes
    void setNodes(const PointsT &nodes)
    {
        nodes_ = nodes;
    }

    void setInputValues(VectorT &values)
    {
        values_ = values;
    }


    VectorT getSqDistanceFromNodes(const PointT &point) const
    {
        // point-to-node difference
        PointsT diff  = nodes_.rowwise() - point.transpose();
        // squared norm of that diff
        VectorT sq_dist =  diff.rowwise().squaredNorm();

        return sq_dist;
    }

    VectorT getRbfPart(const PointT &point) const
    {
        VectorT w = getSqDistanceFromNodes(point);
        for (int i = 0; i < w.rows(); ++i)
            w(i) = thinPlate(w(i)); //!TODO or whatever kernel you want!

        return w;
    }

private:

    inline T thinPlate(T &r) const
    {
        if (r == 0)
            return 0.0;
        else
            return pow(r, 2) * log(r);
    }


    void updateAMatrix()
    {
        // allocate the A matrix
        // the A matrix is n samples x n nodes.
        // if nodes_ is not set we will simply use samples
        if (nodes_.size() == 0)
            A_.resize(points_.rows(), points_.rows());

        else
            A_.resize(points_.rows(), nodes_.rows());

    }


    void solve()
    {

    }


    PointsT points_;
    VectorT values_;

    //! the positions of the nodes
    PointsT nodes_;

    //! Matrix of coefficients
    PointsT A_;

    //! Vector of nodes coefficients
    VectorT coeffs_;


};
//#include <spc/methods/rbf_interpolator.hpp>
} // end namespace
#endif
