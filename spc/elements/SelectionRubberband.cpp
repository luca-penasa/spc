#include "SelectionRubberband.h"
namespace spc
{

DtiClassType SelectionRubberband::Type ("SelectionRubberband", &ElementBase::Type);

//SelectionRubberband::SelectionRubberband() : max_distance_(1.0)
//{
//}




} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::SelectionRubberband)
