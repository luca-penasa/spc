#include "single_attitude_model.h"
namespace spc{


float SingleAttitudeModel::getStratigraphicPosition(const Vector3f &point)
{
    return distanceTo(point) + additional_shift_;
}

Vector3f SingleAttitudeModel::getStratigraphicNormal(const Vector3f &point)
{
    return getUnitNormal();
}





SingleAttitudeModel::SingleAttitudeModel() : additional_shift_(0.0)
{

}






}//end nspace
