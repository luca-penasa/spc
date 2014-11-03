#ifndef RBFKERNELFACTORY_H
#define RBFKERNELFACTORY_H
#include <spc/elements/Kernels.hpp>
namespace spc
{


template<typename ScalarT>
class RBFKernelFactory
{
public:
    enum RBF_FUNCTION {RBF_GAUSSIAN, RBF_GAUSSIAN_APPROX, RBF_MULTIQUADRIC, RBF_EPANECHNIKOV};

    static
    typename RBFBase<ScalarT>::Ptr create(const RBF_FUNCTION &kernel,
                                          const ScalarT & scale = 1)
    {
        if (kernel == RBF_GAUSSIAN)
            return typename RBFBase<ScalarT>::Ptr(new GaussianRBF<ScalarT>(scale));

        else if (kernel == RBF_GAUSSIAN_APPROX)
            return typename RBFBase<ScalarT>::Ptr(new GaussianApproxRBF<ScalarT>(scale));

        else if (kernel == RBF_MULTIQUADRIC)
            return typename RBFBase<ScalarT>::Ptr(new MultiquadricRBF<ScalarT>(scale));

        else if (kernel == RBF_EPANECHNIKOV)
            return typename RBFBase<ScalarT>::Ptr(new EpanechnikovRBF<ScalarT>(scale));

        else
        {
            LOG(ERROR) << "Requested kernel not found";
            return NULL;
        }
    }


};


}//end nspace
#endif // RBFKERNELFACTORY_H
