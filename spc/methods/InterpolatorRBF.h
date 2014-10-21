#ifndef SPC_RBF_INTERPOLATOR_H
#define SPC_RBF_INTERPOLATOR_H

#include <Eigen/Core>
#include <Eigen/Dense>

#include <vector>

#include <spc/methods/polynomials.hpp>

namespace spc
{





//! D is the number of dimensions
//! on which interpolation is performed
template <typename T, size_t DIM = 1>
class InterpolatorRBF
{

    typedef Eigen::Matrix<T, -1, DIM> PointsT;
    typedef Eigen::Matrix<T, -1, 1> VectorT;
    typedef Eigen::Matrix<T, DIM, 1> PointT;
    typedef Eigen::Matrix<T, -1, -1> MatrixT;

public:
    //! default constructor
    InterpolatorRBF(): classical_rbf_(true), nodes_(NULL), points_(NULL)
    {

    }

    T evaluateRbf(PointT &point)
    {

    }

    void setPoints(PointsT &points)
    {
        points_ = &points;

    }

    //! if nodes are not provided all the input points will be used
    //! this means we will get a RB node for each point
    void setNodes(const PointsT &nodes)
    {
        nodes_ = &nodes;
        classical_rbf_ = false; // we will use the nodes as rbf kernels and not the points themselves
    }

    void setInputValues(VectorT &values)
    {
        values_ = &values;
    }


    VectorT getSqDistanceFromNodes(const PointT &point) const
    {
        PointsT diff;
        // point to nodes difference
        diff = nodes_->rowwise() - point.transpose();

        // squared norm of that difference.
        // i.e. the squared distance from nodes or points
        // for the input point
        VectorT sq_dist =  diff.rowwise().squaredNorm();

        return sq_dist;
    }

    VectorT getRbfPart(const PointT &point) const
    {
        VectorT w = getSqDistanceFromNodes(point);
        for (int i = 0; i < w.rows(); ++i)
            w(i) = gaussian(w(i)); //!\todo or whatever kernel you want

        return w;
    }

    VectorT getPolyPart(const PointT &point) const
    {
        return getPolynomialVariables<T>(point, poly_order_);
    }

    VectorT getALine(const PointT &point) const
    {
        VectorT o(getNumberOfNodes() + n_polys_);
        o << getRbfPart(point), getPolyPart(point);
        return o;

    }


    void setPolyOrder(const size_t & order)
    {
        poly_order_ = order;
        n_polys_ = pow(poly_order_+1, DIM); // precomputed number of poly-terms
    }

    T evaluate(const PointT &p) const
    {

        VectorT v = getALine(p);

        std::cout << "\n A line \n" << v << std::endl;
        std::cout << "\n coeffs \n" << coeffs_ << std::endl;
        return v.dot(coeffs_);
    }

    void setLambda(const T& lambda)
    {
        lambda_ = lambda;
    }

    T getLambda() const
    {
        return lambda_;
    }

    void setSigma(const T& sigma)
    {
        sigma_ = sigma;
    }

    T getSigma () const
    {
        return sigma_;
    }

    //! the number of input points
    //! must correspont to the number of values_
    size_t getNumberOfPoints() const
    {
        return points_->rows();
    }

    //! a size of 0 nodes means that a node will be placed
    //! for each input point. This corresponds to the classical formulation of RBF
    size_t getNumberOfNodes() const
    {
        return nodes_->rows();
    }


private:

    size_t getNumberOfpolynomialTerms() const
    {
        return n_polys_;
    }

    //! \todo move these kernels in an appropriate place.
    //! and make so that the user can select the kernel he wants
    inline T thinPlate(T &squared_r) const
    {
        if (squared_r == 0)
            return 0.0;
        else
            return squared_r * log(sqrt(squared_r));
    }

    inline T gaussian(const T& squared_r) const
    {
        return exp(-(squared_r / sigma_));
    }





public:

    //! this method takes care of initializing A_ matrix and b_ vector for
    //! solving the problem
    void initProblem()
    {
        if (!nodes_)
        {
            std::cout << "Nodes not selected. The classic RBF will be estimated..." << std::endl;
            nodes_ = points_; // make nodes_ point to points_
//            classical_rbf_ = true; // just to be sure
        }
        else
            std::cout << "using user-selected nodes" << std::endl;

        // number of columns is the same with both methods
        // notice that in the classical rbf nodes_ corresponds to points_
        size_t n_cols = getNumberOfNodes() + getNumberOfpolynomialTerms();

        // number of rows changes depending on the method
        // in the classical implementation a square matrix will be formed
        // while in the method with user-selected nodes the matrix will be rectangular
        // with size n_points +
        // and solution is provided by least squares
        size_t n_rows;

        if (classical_rbf_)
            n_rows = getNumberOfPoints() + getNumberOfpolynomialTerms();
        else // non classical
        {
            // then we have two cases
            if (lambda_ == 0 ) // no regularization
                n_rows = getNumberOfPoints();
            else // with regularization
                n_rows = getNumberOfPoints() + getNumberOfNodes();
        }

        // resize A
        A_.resize(n_rows, n_cols);


        // b_ MUST have same to A_.rows();
        b_.resize(n_rows);

        // init them with zeros.
        A_.fill(0);
        b_.fill(0);


        // fill b vector with the values. the remaining part will be zeros...
        b_.head(values_->rows()) = *values_;


        // now fill A with the observations
        for (int i = 0; i < getNumberOfPoints(); ++i) // a row for each point
        {
            A_.row(i) = getALine(points_->row(i));
        }

        if (classical_rbf_)
        {
            // copy the transposed part of polynomials at the bottom
            // we use an intermediate matrix for clarity
            MatrixT tmp = A_.topRightCorner(getNumberOfPoints(), getNumberOfpolynomialTerms());
            A_.bottomLeftCorner(getNumberOfpolynomialTerms(), getNumberOfPoints()) = tmp.transpose();
        }
//        else
//        {
//            // nothing to do
//        }


        // now add regularization
        if (lambda_ != 0)
        {
            if (classical_rbf_) // add an identinty * lambda_ to the upper left corner
            {
                A_.topLeftCorner(getNumberOfPoints(), getNumberOfPoints()) += MatrixT::Identity(getNumberOfPoints(), getNumberOfPoints()) * lambda_;
            }
            else
            {
                MatrixT tikhonov = MatrixT::Identity(getNumberOfNodes(), getNumberOfNodes()) * sqrt(lambda_);
                A_.bottomLeftCorner(getNumberOfNodes(), getNumberOfNodes()) = tikhonov;
            }
        }

    }

    //! solve the linear system and retrieve cofficients
    //! once coefficients are computed the interpolator is compeltely defined.
    int solveProblem()
    {
        // do the linear system solving
        if (A_.rows() == A_.cols()) {
            // solve via pivoting
            coeffs_ = A_.colPivHouseholderQr().solve(b_);
        } else if (A_.rows() > A_.cols()) {
            // solving via SVD
            coeffs_ = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_);
        } else
            return -1; // unsuccessfull;
    }


//    void solve()
//    {
//        if (nodes_.size() == 0) // we will set up a radial basis function
//                                // centered on each data point.

//        {

//            //! \todo add check on the number of input points vs total number of parameters
//            MatrixT A;
//            A.resize(points_.rows() + n_polys_, points_.rows() + n_polys_);
//            A.fill(0); // a clean matrix to start with

//            // populate with the observations
//            for (int i = 0 ; i < points_.rows(); ++i)
//            {
//                A.row(i) = getALine(points_.row(i));
//            }

//            // regularization is needed
//            if (lambda_ != 0.0)
//            {
//                MatrixT tikhonov = MatrixT::Identity(points_.rows(), points_.rows()).array() * lambda_;
//                if (false)
//                {
//                    A.block(0,0, points_.rows(), points_.rows()) += tikhonov;
//                }
//                else
//                {
//                    // add the tikhonov matrix at the end
//                    A.conservativeResize(A.rows() + tikhonov.rows(), A.cols() );
//                    A.block(points_.rows() + n_polys_, 0, tikhonov.rows(), tikhonov.cols()) = tikhonov;
//                }
//            }

//            // get the part of the observations matrix of the polynomials
//            // and stack it under the observation matrix A
//            MatrixT tmp = A.block(0, points_.rows(), points_.rows(), n_polys_);
//            A.block(points_.rows(), 0, n_polys_, points_.rows()) = tmp.transpose();

//            // we also need to pad with zeros the values_ vector
//            VectorT zeros = VectorT::Zero(A.rows() - values_.rows());
//            VectorT b = values_; // copy the values
//            b.conservativeResize(A.rows());
//            b.tail(A.rows() - values_.rows()) = zeros;


//            // do the linear system solving
//            if (A.rows() == A.cols()) {
//                // solve via pivoting
//                coeffs_ = A.colPivHouseholderQr().solve(b);
//            } else if (A.rows() > A.cols()) {
//                // solving via SVD
//                coeffs_ = A.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b);

//            }


//            std::cout <<"A: \n"<< A << std::endl;
//            std::cout << "b: \n " << b << std::endl;

//            std::cout << "\n solution: \n" << coeffs_ <<std::endl;
//        }
//        else // we will use a radial basis centered on each node.
//        {

//        }
//    }


    //! the points used to estimate the RBF
    PointsT * points_;
    size_t n_points_;

    //! the vector of values
    VectorT * values_;

    //! the positions of the nodes
    //! if no nodes are provided a classical RBF
    //! with one node for each point will be used
    const PointsT * nodes_;
    size_t n_nodes_ = 0;

    //! regularization factor
    T lambda_ = 0;

    //! user options
    //! set the polynomial order using setPolyOrder()
    //! n_polys_ will be automatically updated
    size_t poly_order_ = 0;
    size_t n_polys_ = 1;

    //! Vector of nodes coefficients - almost compeltely defines the RBF
    VectorT coeffs_;

    //! A_ and b_ define the linear system Ax=b where x are placed in coeffs_
    MatrixT A_;
    VectorT b_;

    //! say it the inerpolator is a classica RBF or not
    //! it is automatically switched to false when setNodes() is called
    bool classical_rbf_ ;

    //! some radial basis kernel may require a sigma parameter.
    //! set it depending on the basis you are using.
    T sigma_ = 1.0;
};
//#include <spc/methods/rbf_interpolator.hpp>
} // end namespace
#endif
