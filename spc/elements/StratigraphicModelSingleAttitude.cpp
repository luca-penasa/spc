#include "StratigraphicModelSingleAttitude.h"
namespace spc
{

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


