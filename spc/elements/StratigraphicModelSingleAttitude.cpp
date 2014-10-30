#include "StratigraphicModelSingleAttitude.h"
namespace spc
{
DtiClassType StratigraphicModelSingleAttitude::Type ("StratigraphicModelSingleAttitude", &StratigraphicModelBase::Type);

float StratigraphicModelSingleAttitude::getScalarFieldValue(const Vector3f &point) const
{
    return attitude_.distanceTo(point) + additional_shift_;
}

Vector3f StratigraphicModelSingleAttitude::getScalarFieldGradient(const Vector3f
                                                     &point) const
{
    return attitude_.getUnitNormal();
}

} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicModelSingleAttitude)
