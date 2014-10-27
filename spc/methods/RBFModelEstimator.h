#ifndef SPC_RBF_INTERPOLATOR_H
#define SPC_RBF_INTERPOLATOR_H

#include <spc/elements/RBFModel.h>
#include <spc/methods/spc_eigen.h>
#include <spc/methods/eigen_extensions.h>


namespace spc
{





//! D is the number of dimensions
//! on which interpolation is performed
template <typename T>
class RBFModelEstimator
{
    typedef Eigen::Matrix<T, -1, -1> PointsT;
    typedef Eigen::Matrix<T, -1, 1> VectorT;
    typedef Eigen::Matrix<T, -1, 1> PointT;
    typedef Eigen::Matrix<T, -1, -1> MatrixT;


public:
    //! default constructor
    RBFModelEstimator(): classical_rbf_(true), model_(new RBFModel<T>)

    {
    }

    void setPoints(PointsT &points)
    {
        points_ = points;
    }

    void setInputValues(VectorT values)
    {
        values_ = values;
    }

    void setWeights(VectorT weights)
    {
        weights_ = weights;
    }

    typename RBFModel<T>::Ptr getModel() const
    {
        return model_;
    }

    void setModel(typename RBFModel<T>::Ptr model)
    {
        model_ = model;
    }


    /**
     * \brief autosetNodes
     * \param n_nodes the number of nodes in each dimension
     */
    void autosetNodes(const Eigen::VectorXi &n_nodes)
    {
        CHECK (points_.size() != 0) << "set the points before calling this method";
        CHECK (n_nodes.rows() == points_.cols()) << "dimensions and number of splits does not match";

        DLOG(INFO) << "autosetting nodes";

        size_t total_nodes_ = n_nodes.array().prod();
        DLOG(INFO) << "total number of nodes " << total_nodes_ << std::endl;

        std::vector<VectorT> nod_coords;
        for (int i =0; i < points_.cols(); ++i)
        {
            T min = points_.col(i).minCoeff();
            T max = points_.col(i).maxCoeff();


            VectorT coords = VectorT::LinSpaced(n_nodes(i), min, max);
            nod_coords.push_back(coords);
        }

        MatrixT  nodes = meshgrid<T>(nod_coords);
        model_->setNodes( nodes );

        DLOG(INFO) << "nodes automatically set";


//        autosetScales();

    }

    void autosetScales(const size_t fixed_dimension = 0)
    {
        CHECK(points_.size() != 0) <<"set the input points before to call this method";
        CHECK(fixed_dimension <= points_.cols()) << "you have to set a fixed dimension index lower or equal than the dimensions";

        DLOG(INFO) << "autodefining scales" ;

        VectorT m, std;
        points_.meanAndStd(m, std);
        DLOG(INFO) << "mean: \n" << m ;
        DLOG(INFO) << "std: \n" << std ;

        VectorT scales(points_.cols());

        T fixed_std = std(fixed_dimension);

        for (int i = 0; i < points_.cols(); ++i)
        {
            if (i == fixed_dimension)
                scales(i) = 1;
            else
            {
                T ratio = fixed_std / std(i);
                scales(i) = ratio;
            }
        }

        model_->setScales(scales);

        DLOG(INFO) << "scales defined";
    }

    /**
     * @brief autosetSigma sets the sigma value of the kernel (the kernel size)
     * to the average distance between a node and the nearest one.
     */
    void autosetSigma()
    {
        CHECK(model_->getNodes().size() != 0) << "nodes must be set before calling this function";
        VectorT nn_distances(model_->getNodes().rows()); // nearest neighbor distance for each node

        for (int i = 0 ; i < model_->getNodes().rows(); ++i)
        {
            PointT n = model_->getNodes().row(i);
            VectorT distances = model_->getSqDistanceFromNodes(n);


            distances = distances.nonZeroCoeffs();
            T min_dist = distances.minCoeff();
            nn_distances(i) = sqrt(min_dist);
        }

        T avg =  nn_distances.mean();
        LOG(INFO) << "Sigma autoset to " << avg;

        this->model_->setSigma(avg);




        DLOG(INFO) << "nearest neighbor distances " << nn_distances.T();
    }





    //! lambda is the regularization term
    //! its meaning may differ depending if RBF is classsic or not
    //! classic RBF uses a node for each input point. This result in a peculiar matrix
    //! and the regularization is added as described in many papers.
    //! when the nodes are user-chosen the system is overdetermined and a tikhonov regularization is used
    //! and the square root of lambda is used
    void setLambda(const T& lambda)
    {
        lambda_ = lambda;
    }

    T getLambda() const
    {
        return lambda_;
    }

    //! the number of input points
    //! must correspont to the number of values_
    size_t getNumberOfPoints() const
    {
        return points_.rows();
    }

    bool hasWeights() const
    {
        if (weights_.size() != 0)
            return true;
        else
            return false;
    }

public:

    //! this method takes care of initializing A_ matrix and b_ vector for
    //! solving the problem
    int initProblem()
    {
        // a lot of safety checks here!
        if (points_.size() == 0)
        {
            LOG(WARNING) << "poits are empty. Please provide them";
            return -1;
        }


        if (values_.size() == 0)
        {
            LOG(WARNING) << "values empty. Set them";
            return -1;
        }

        if (model_->getNodes().size() == 0 )
        {
            LOG(INFO) << "Nodes not selected. The classic RBF will be estimated..." ;
            classical_rbf_ = true;
            model_->setNodes(points_);
        }
        else
        {
            classical_rbf_ = false;
            LOG(INFO) << "using user-selected nodes" ;
        }


        if (hasWeights())
        {
            LOG(INFO) << "Using weights" ;

            if(weights_.rows() != points_.rows())
            {
                LOG(WARNING) << "you are trying to use weighted least squares for solving the problem."
                               "but it looks like your weights size is different than number of points";
                return -1;
            }
        }

        //////////// checks are ok... go ahead!
        DLOG(INFO) << "Checks passed!";

        size_t n_nodes = model_->getNumberOfNodes();
        size_t n_polys = model_->getNumberOfpolynomialTerms();

        // number of columns is the same with both methods
        // notice that in the classical rbf nodes_ corresponds to points_
        size_t n_cols = n_nodes + n_polys;

        // number of rows changes depending on the method
        // in the classical implementation a square matrix will be formed
        // while in the method with user-selected nodes the matrix will be rectangular
        // with size n_points +
        // and solution is provided by least squares
        size_t n_rows;

        if (classical_rbf_)
            n_rows = getNumberOfPoints() + n_polys;
        else // non classical
        {
            // then we have two cases
            if (lambda_ == 0 ) // no regularization
                n_rows = getNumberOfPoints();
            else // with regularization
                n_rows = getNumberOfPoints() + n_nodes;
        }

        // resize A
        try {
            A_.resize(n_rows, n_cols);
        }
        catch (const std::bad_alloc&)
        {
            LOG(WARNING) << "out of memory allocating A";
            return -1;
        }





        // b_ MUST have same to A_.rows();
        b_.resize(n_rows);

        // init them with zeros.
        A_.fill(0);
        b_.fill(0);


        DLOG(INFO) << "Going to populate A";


        // fill b vector with the values. the remaining part will be zeros...
        b_.head(values_.rows()) = values_;

        if (hasWeights())
        {
            b_.head(values_.rows()).array() *= weights_.array();
            DLOG(INFO) << "Wightin done";
        }
        else
        {
            DLOG(INFO) << "NOT weighted";
        }





        // now fill A with the preditors
#ifdef USE_OPENMP
#pragma omp parallel for
#endif
        for (int i = 0; i < getNumberOfPoints(); ++i) // a row for each point
        {
            VectorT pre  = model_->getPredictorVector(points_.row(i));
            A_.row(i) = pre;


            if (hasWeights())
            {
                A_.row(i).array() *= weights_(i);
            }
        }

        DLOG(INFO) << "A populated";


        if (classical_rbf_)
        {
            DLOG(INFO) << "copy transposed matrix";

            // copy the transposed part of polynomials at the bottom
            // we use an intermediate matrix for clarity
            MatrixT tmp = A_.topRightCorner(getNumberOfPoints(), n_polys);
            A_.bottomLeftCorner(n_polys, getNumberOfPoints()) = tmp.transpose();
        }




        // now add regularization
        if (lambda_ != 0)
        {
            DLOG(INFO) << "Add regularization";

            if (classical_rbf_) // add an identinty * lambda_ to the upper left corner
            {
                A_.topLeftCorner(getNumberOfPoints(), getNumberOfPoints()) += MatrixT::Identity(getNumberOfPoints(), getNumberOfPoints()) * lambda_;
            }
            else
            {
                MatrixT tikhonov = MatrixT::Identity(n_nodes, n_nodes) * sqrt(lambda_);
                A_.bottomLeftCorner(n_nodes, n_nodes) = tikhonov;
            }
        }

        DLOG(INFO) << "Init done";


        return 1;

    }

    //! solve the linear system and retrieve cofficients
    //! once coefficients are computed the interpolator is compeltely defined.
    int solveProblem()
    {
        int status = initProblem();
        if (status != 1)
        {
            LOG(WARNING) << "initialization of linar system failed" ;
            return -1;
        }

        if (A_.cols() > A_.rows())
        {
            LOG(WARNING)     << "looks like your problem is underdetermined. Consider reducing the number of polynomials terms \n"
                               "or the number of nodes you are using, or add more data points for the fitting \n"
                               "you can also overcome this problem using regularization (if you are not using it yet) but it is not suggested.";

            return -1;
        }

        if(A_.size() == 0)
        {
            LOG(WARNING)<< "A matrix is empty";
            return -1;
        }

        if(b_.size() == 0)
        {
            LOG(WARNING)<< "b vector is empty";
            return -1;
        }

        DLOG(INFO) << "going to solve the system";

        VectorT coeffs;
        // do the linear system solving
        if (A_.rows() == A_.cols()) {
            // solve via pivoting
            coeffs = A_.colPivHouseholderQr().solve(b_);
            model_->setCoefficients(coeffs);
            DLOG(INFO) << "system correctly solved via Housolder QR";


        } else if (A_.rows() > A_.cols()) {
            // solving via SVD
            coeffs = A_.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(b_);
            model_->setCoefficients(coeffs);
            DLOG(INFO) << "system correctly solved via JacobiSVD";
        } else
        {
            DLOG(INFO) << "Cannot solve the system cause is missing constraints in the A matrix (rows)";
            return -1; // unsuccessfull;
        }
    }


    //! the points used to estimate the RBF
    PointsT points_;
    size_t n_points_;

    //! the vector of values
    VectorT values_;

    //! at each input point we can associate a different weight for the solution.
    VectorT weights_;

    //! regularization factor
    T lambda_ = 0;

    //! A_ and b_ define the linear system Ax=b where x are placed in coeffs_
    MatrixT A_;
    VectorT b_;

    //! say it the inerpolator is a classica RBF or not
    //! it is automatically switched to false when setNodes() is called
    bool classical_rbf_ ;

    typename RBFModel<T>::Ptr model_;





};
//#include <spc/methods/rbf_interpolator.hpp>
} // end namespace
#endif
