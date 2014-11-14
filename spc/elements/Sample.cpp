#include "Sample.h"
 namespace spc
{

 DtiClassType Sample::Type ("Sample", &StratigraphicPositionableElement::Type);


}//end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Sample);
