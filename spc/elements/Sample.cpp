#include "Sample.h"
namespace spc {

DtiClassType Sample::Type("Sample", &StratigraphicPositionableElement::Type);

void Sample::applyTransform(const GeometricElement3DBase::TransformT &transform)
{
    point_.applyTransform(transform);
}

} //end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Sample);
