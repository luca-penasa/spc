#ifndef RBFKERNELFACTORY_H
#define RBFKERNELFACTORY_H
#include <spc/elements/Kernels.hpp>
namespace spc
{


template<typename ScalarT>
class RBFKernelFactory
{
public:
    enum RBF_FUNCTION {RBF_GAUSSIAN, RBF_MULTIQUADRIC};

    static
    typename RBFBase<ScalarT>::Ptr create(const RBF_FUNCTION &kernel,
                                          const ScalarT & scale = 1)
    {
        if (kernel == RBF_GAUSSIAN)
            return RBFBase<ScalarT>::Ptr(new GaussianRBF<ScalarT>(scale));

        else if (kernel == RBF_MULTIQUADRIC)
            return RBFBase<ScalarT>::Ptr(new MultiquadricRBF<ScalarT>(scale));
        else
        {
            LOG(WARNING) << "Requested kernel not found";
            return NULL;
        }
    }


};


}//end nspace
#endif // RBFKERNELFACTORY_H
