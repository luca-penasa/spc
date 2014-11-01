#include "Kernels.hpp"


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::RBFBase<float>, "spc::RBFBaseF")

SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::GaussianRBF<float>, "spc::GaussianRBF")

SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::MultiquadricRBF<float>, "spc::MultiquadricRBF")
