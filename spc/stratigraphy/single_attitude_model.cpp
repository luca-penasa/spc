#include "single_attitude_model.h"
namespace spc{


float spcSingleAttitudeModel::getStratigraphicPosition(const Vector3f &point)
{
    return attitude_->distanceTo(point) + additional_shift_;
}

Vector3f spcSingleAttitudeModel::getStratigraphicNormal(const Vector3f &point)
{
    return attitude_->getUnitNormal();
}












}//end nspace
