#include "Kernels.hpp"


#include <spc/core/spc_cereal.hpp>
namespace spc
{


//template<> RBFBase<float>;


template<> DtiClassType RBFBase<float>::Type ("RBFBaseF", &spc::ElementBase::Type);
template<> DtiClassType GaussianRBF<float>::Type ("GaussianRBFF", &spc::RBFBase<float>::Type);
template<> DtiClassType GaussianApproxRBF<float>::Type ("GaussianApproxRBFF", &spc::RBFBase<float>::Type);
template<> DtiClassType MultiquadricRBF<float>::Type ("MultiquadricRBFF", &spc::RBFBase<float>::Type);
template<> DtiClassType EpanechnikovRBF<float>::Type ("EpanechnikovRBFF", &spc::RBFBase<float>::Type);
template<> DtiClassType PolyharmonicRBF<float>::Type ("PolyharmonicRBFF", &spc::RBFBase<float>::Type);



}



// cereal rgistrations to serialize kernels
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::RBFBase<float>, "RBFBaseF")
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::GaussianRBF<float>, "GaussianRBFF")
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::GaussianApproxRBF<float>, "GaussianApproxRBFF")
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::MultiquadricRBF<float>, "MultiquadricRBFF")
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::EpanechnikovRBF<float>, "EpanechnikovRBFF")
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::PolyharmonicRBF<float>, "PolyharmonicRBFF")
