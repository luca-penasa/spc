#include "StratigraphicPositionableElement.h"
namespace  spc {

DtiClassType StratigraphicPositionableElement::Type ("StratigraphicPositionableElement", &GeologicalElement::Type);

bool StratigraphicPositionableElement::isAsciiSerializable() const
{
    return true;
}

int StratigraphicPositionableElement::toAsciiStream(std::ostream &stream) const
{
    stream << this->getStratigraphicPosition();
    stream << " " << this->getElementName() << "\n";
    return 1;
}




}// end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicPositionableElement)
