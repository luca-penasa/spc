#include "EigenLinearFunctionBase.h"
namespace spc
{
DtiClassType EigenLinearFunctionBase::Type = DtiClassType("EigenLinearFunctionBase", &EigenFunctionBase::Type);

}//end nspace
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::EigenLinearFunctionBase)
