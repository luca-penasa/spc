#ifndef RBFPARAMETERS_H
#define RBFPARAMETERS_H
#include <spc/elements/ElementBase.h>
#include <spc/calibration/RBFKernels.h>


namespace spc
{

template <typename ScalarT = double>
class RBFParameters: public ElementBase
{
public:

    //    typedef double ScalarT;
    typedef Eigen::Matrix<ScalarT,-1,1> VectorT;
    typedef Eigen::Matrix<ScalarT,-1,-1> MatrixT;

    SPC_OBJECT(RBFParameters)
//    EXPOSE_TYPE

    RBFParameters ()
    {
    }

    RBFParameters (const MatrixT &knots,
                   const VectorT &coeffs,
                   const typename RBFKernel<ScalarT>::Ptr kernel)
    {
        nodes_ = knots;
        coeffs_ = coeffs;
        kernel_ = kernel;
    }



public:
    virtual bool isValid() const
    {
        if (nodes_.rows() != coeffs_.rows())
        {
            std::cout << "different size of nodes and coefficients!\n" <<std::endl;
            return false;
        }
        else if (!kernel_->isValid())
        {
            std::cout << "Kernel is invalid. sigma value not setted maybe?!\n" <<std::endl;
            return false;
        }
        else
            return true;
    }

    void initUnityCoefficients()
    {
        coeffs_ = VectorT::Ones(nodes_.rows());
    }

    void setNodes(const MatrixT & nodes)
    {
        nodes_ = nodes;
    }

    MatrixT getNodes() const
    {
        return nodes_;
    }

    void setCoefficients(const VectorT &coeffs)
    {
        coeffs_ = coeffs;
    }

    MatrixT getCoefficients() const
    {
        return coeffs_;
    }

    size_t getDimensions( ) const
    {
        return nodes_.cols();
    }

    size_t getNumberOfNodes() const
    {
        return nodes_.rows();
    }

    typename RBFKernel<ScalarT>::Ptr getKernel() const
    {
        return kernel_;
    }

    void setKernel(typename RBFKernel<ScalarT>::Ptr kernel)
    {
        kernel_ = kernel;
    }

protected:
    MatrixT nodes_;
    VectorT coeffs_;

    typename RBFKernel< ScalarT>::Ptr kernel_ = typename RBFKernel< ScalarT>::Ptr(new RBFKernelGaussian<ScalarT>());


};


}//end nspace

#endif // RBFPARAMETERS_H
