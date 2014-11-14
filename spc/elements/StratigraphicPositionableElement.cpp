#include "StratigraphicPositionableElement.h"
namespace  spc {

DtiClassType StratigraphicPositionableElement::Type ("StratigraphicPositionableElement", &Point3D::Type);




}// end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicPositionableElement);
