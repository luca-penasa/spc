#include "RBFKernels.h"


namespace spc
{

//template <>
//DtiClassType RBFKernel<double>::Type ("RBFKernel", &ElementBase::Type);

//template <>
//DtiClassType RBFKernelGaussian<double>::Type ("RBFKernelGaussian", &RBFKernel<double>::Type);


template class RBFKernel<double>;
template class RBFKernelGaussian<double>;






}//end nspace
