#include "VirtualOutcrop.h"


namespace spc
{

DtiClassType VirtualOutcrop::Type ("VirtualOutcrop", &ElementBase::Type);





}//end nspace



#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::VirtualOutcrop)
