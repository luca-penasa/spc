#include "Kernels.hpp"


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::BasicRadialBasisFunction<float>, "spc::BasicRadialBasisFucntionFloat")

SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::GaussianRBF<float>, "spc::GaussianRBF")
