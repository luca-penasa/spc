#ifndef SPC_RBF_INTERPOLATOR_H
#define SPC_RBF_INTERPOLATOR_H

#include <spc/elements/RBFModel.h>
#include <spc/core/spc_eigen.h>
#include <spc/core/eigen_extensions.h>
#include <spc/elements/RBFKernelFactory.h>


namespace spc
{






template <typename T>
class RBFModelEstimator
{
public:
    typedef Eigen::Matrix<T, -1, -1> PointsT;
    typedef Eigen::Matrix<T, -1, 1> VectorT;
    typedef Eigen::Matrix<T, -1, 1> PointT;
    typedef Eigen::Matrix<T, -1, -1> MatrixT;
    typedef Eigen::Matrix<size_t, -1, 1> IDVectorT;


    friend class IntensityCalibratorRBF;

    //! default constructor
    RBFModelEstimator(): classical_rbf_(true), model_(new RBFModel<T>)

    {

		model_->setKernel(RBFKernelFactory<T>::RBF_GAUSSIAN_APPROX);
    }

	spcSetMacro(Points, points_, PointsT)
	spcGetMacro(Points, points_, PointsT)


	void setInputValues(const VectorT &values)
    {
        values_ = values;
    }

    VectorT getInputValues() const
    {
        return values_;
    }

	void setWeights(const VectorT &weights)
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

    //! ths actually states that all the points must have the same value
    //! see also appendEqualityConstrain(); method
    //! it is used when calibrating models for intensities. Not sure if it can be used in other contexts
    //! so if you want to use it please verify what it is doing and if it is good for you.
    //! if you have points A, B and C this method will add constrains which will state that:
    //! A = B, A = C and B = C
	void appendEqualityConstrainForPoints(const PointsT & points,
										  const VectorT & values,
										  const VectorT & weights);

    //! this add to the Ax = b system additional constrains about equality between two points
    //! it is used when calibrating  intensities
    //! equals to say that \f$I_1/f(a_1, d_1) = I_2/f(a_2, d_2)\f$. or equally that the corrected intensity
    //! for the point a must be equal to the corrected intensity for point b
    //! \todo better explanations here
	//! optionally also a weight for this observation can be applied
	int appendEqualityConstrain(const PointT &point1, const T &value1,
								 const PointT &point2, const T &value2,
								const T &weight = 1);


    /**
     * \brief autosetNodes
     * \param n_nodes the number of nodes in each dimension
     */
	void autosetNodes(const Eigen::VectorXi &n_nodes, PointsT input_points = PointsT());


    void setNodes(const Eigen::MatrixXf & nodes);


    void autosetScales(const int fixed_dimension = 0, PointsT input_points = PointsT());

    /**
     * @brief autosetSigma sets the sigma value of the kernel (the kernel size)
     * to the average distance between a node and the nearest one.
     */
	void autosetSigma();


//    //! this allows to inject additional stuff
//    //! into the A (and b -just use getb() method-) matrix (and vector)
//    //! befoe perfoming the actual solution
//    Eigen::Ref<MatrixT> getA()
//    {
//        return Eigen::Ref<MatrixT> (A_);
//    }

//    Eigen::Ref<VectorT> getb()
//    {
//        return Eigen::Ref<VectorT> (A_);
//    }


    //! lambda is the regularization term
    //! its meaning may differ depending if RBF is classsic or not
    //! classic RBF uses a node for each input point. This result in a peculiar matrix
    //! and the regularization is added as described in many papers.
    //! when the nodes are user-chosen the system is overdetermined and a tikhonov regularization is used
    //! and the square root of lambda is used as often described for tikhonov reg.
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

    int doChecksBeforeInit()
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


//        if (getCorrespondenceIds().size() == 0)
//        {
//            LOG(INFO) << "no correspondences provided. no additional constrains for correspondent points will be used "
//                         "caibrating the model";
//        }
//        else
//        {
//            if (getCorrespondenceIds().size() != points_.rows())
//            {
//                LOG(ERROR) << "the provided correspondences for points have a different number of entries in"
//                              "respect to the total number of points";
//                return -1;
//            }
//        }

    return 1;
    }

public:

    //! this method takes care of initializing A_ matrix and b_ vector for
    //! solving the problem
	int initProblem();

    //! solve the linear system and retrieve cofficients
    //! once coefficients are computed the interpolator is compeltely defined.
	int solveProblem();




private:

    //! the points used to estimate the RBF
    //! points are in the variable space.
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

    //! the output model we are calibrating
    typename RBFModel<T>::Ptr model_;





};
//#include <spc/methods/rbf_interpolator.hpp>
} // end namespace
#endif
