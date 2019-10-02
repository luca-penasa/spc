#include "EigenFunctionBase.h"
namespace spc
{
DtiClassType EigenFunctionBase::Type = DtiClassType("EigenFunctionBase", &ElementBase::Type);



}//end nspace
#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::EigenFunctionBase)
