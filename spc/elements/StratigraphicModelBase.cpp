#include "StratigraphicModelBase.h"

namespace spc
{
DtiClassType StratigraphicModelBase::Type ("StratigraphicModelBase", &VariableScalarFieldBase::Type);

}


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicModelBase)
