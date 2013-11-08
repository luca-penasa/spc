#include "single_attitude_model.h"
namespace spc{


float SingleAttitudeModel::getStratigraphicPosition(const Vector3f &point)
{
    return distanceTo(point);
}

Vector3f SingleAttitudeModel::getStratigraphicNormal(const Vector3f &point)
{
    return getUnitNormal();
}






}//end nspace
