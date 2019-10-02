#include "GeologicalElement.h"

namespace spc
{


DtiClassType GeologicalElement::Type ("GeologicalElement", &GeometricElement3DBase::Type);


GeologicalElement::GeologicalElement()
{

}




}// end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::GeologicalElement)
