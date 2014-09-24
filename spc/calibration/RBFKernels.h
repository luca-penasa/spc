#ifndef RBFKERNELS_H
#define RBFKERNELS_H

#include <spc/elements/ElementBase.h>

namespace spc
{

template <typename ScalarT = double>
class RBFKernel: public ElementBase
{
public:


    SPC_OBJECT(RBFKernel)
//    EXPOSE_TYPE



    virtual const ScalarT operator()(const ScalarT& sq_dist) const = 0;


    //! some kernels may have additional parameters that need
    //! to b filled for the kernel to be valid
    virtual bool isValid() const = 0;

    //! by def we consider the kernel as not-compactly supported
    //! fitters may use this information for setting up dense or sparse matrix and solvers
    virtual bool isCompactlySupported() const
    {
        return false;
    }

};


template <typename ScalarT = double>
class RBFKernelGaussian: public RBFKernel<ScalarT>
{
public:

    SPC_OBJECT(RBFKernelGaussian)
//    EXPOSE_TYPE

    RBFKernelGaussian()
    {

    }

    RBFKernelGaussian(const ScalarT& sigma) : sq_sigma_(sigma*sigma) {}

    virtual const ScalarT operator()(const ScalarT& sq_dist) const
    {
        return exp(- sq_sigma_ * sq_dist );
    }


    virtual bool isValid() const
    {
        // if sigma is ok we are fine
//        return std::isfinite(sigma_);
        return true;
    }


    void setSigma(const ScalarT & sigma)
    {
        sq_sigma_ = sigma * sigma;
    }

    ScalarT getSigma() const
    {
        return sqrt(sq_sigma_);
    }


    spcGetMacro(SquaredSigma, sq_sigma_, ScalarT)
    spcSetMacro(SquaredSigma, sq_sigma_, ScalarT)

protected:
    ScalarT sq_sigma_ = std::numeric_limits<ScalarT>::quiet_NaN(); // is not valid by default


};

}//end nspace

#endif // RBFKERNELS_H
