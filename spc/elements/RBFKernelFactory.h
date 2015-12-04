#ifndef RBFKERNELFACTORY_H
#define RBFKERNELFACTORY_H
#include <spc/elements/Kernels.hpp>
namespace spc
{


template<typename ScalarT>
class RBFKernelFactory
{
public:
    enum RBF_FUNCTION {RBF_GAUSSIAN,
                       RBF_GAUSSIAN_APPROX,
                       RBF_MULTIQUADRIC,
                       RBF_EPANECHNIKOV,
                      RBF_POLYHARMONIC_1,
                      RBF_POLYHARMONIC_2,
                      RBF_POLYHARMONIC_3,
                      RBF_POLYHARMONIC_4,
                      RBF_POLYHARMONIC_5,
                      RBF_POLYHARMONIC_6};

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

        else if (kernel == RBF_POLYHARMONIC_1)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(1));

        else if (kernel == RBF_POLYHARMONIC_2)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(2));

        else if (kernel == RBF_POLYHARMONIC_3)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(3));

        else if (kernel == RBF_POLYHARMONIC_4)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(4));

        else if (kernel == RBF_POLYHARMONIC_5)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(5));

        else if (kernel == RBF_POLYHARMONIC_6)
            return typename RBFBase<ScalarT>::Ptr(new PolyharmonicRBF<ScalarT>(6));

        else
        {
            LOG(ERROR) << "Requested kernel not found";
            return NULL;
        }
    }


};


}//end nspace
#endif // RBFKERNELFACTORY_H
