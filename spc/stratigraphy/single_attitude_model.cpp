#include "single_attitude_model.h"
namespace spc{


float spcSingleAttitudeModel::getStratigraphicPosition(const Vector3f &point) const
{
    return attitude_->distanceTo(point) + additional_shift_;
}

Vector3f spcSingleAttitudeModel::getStratigraphicNormal(const Vector3f &point) const
{
    return attitude_->getUnitNormal();
}












}//end nspace


//BOOST_CLASS_EXPORT_GUID(spc::spcSingleAttitudeModel, "spcSingleAttitudeModel")
