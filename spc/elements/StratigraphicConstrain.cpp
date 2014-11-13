#include "StratigraphicConstrain.h"

namespace spc
{

DtiClassType StratigraphicConstrain::Type ("StratigraphicConstrain", &ElementBase::Type);


} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicConstrain)
