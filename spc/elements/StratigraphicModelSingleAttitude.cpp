#include "StratigraphicModelSingleAttitude.h"
namespace spc{


float SingleAttitudeModel::getScalarFieldValue(const Vector3f &point) const
{
    return attitude_.distanceTo(point) + additional_shift_;
}

Vector3f SingleAttitudeModel::getScalarFieldGradient(const Vector3f &point) const
{
    return attitude_.getUnitNormal();
}












}//end nspace


//BOOST_CLASS_EXPORT_GUID(spc::spcSingleAttitudeModel, "spcSingleAttitudeModel")
