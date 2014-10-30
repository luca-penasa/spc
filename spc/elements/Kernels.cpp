#include "Kernels.hpp"


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::BasicKernel<float>, "spc::BasicKernelFloat")

SPC_CEREAL_REGISTER_TYPE_WITH_NAME(spc::GaussianKernel<float>, "spc::GaussianKernelFloat")
