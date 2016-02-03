#include "StratigraphicPositionableElement.h"
namespace  spc {

DtiClassType StratigraphicPositionableElement::Type ("StratigraphicPositionableElement", &GeologicalElement::Type);




}// end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicPositionableElement)
