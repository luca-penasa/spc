#include "SelectionRubberband.h"

#include <cereal/access.hpp>
#include <spc/core/spc_cereal.hpp>


namespace spc
{

DtiClassType SelectionRubberband::Type ("SelectionRubberband", &ElementBase::Type);

//SelectionRubberband::SelectionRubberband() : max_distance_(1.0)
//{
//}




} // end nspace


//spc::SelectionRubberband test;
//CEREAL_REGISTER_DYNAMIC_INIT(spc)




SPC_CEREAL_REGISTER_TYPE(spc::SelectionRubberband)

CEREAL_CLASS_VERSION( spc::SelectionRubberband, 1 )


