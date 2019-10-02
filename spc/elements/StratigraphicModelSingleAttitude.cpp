#include "StratigraphicModelSingleAttitude.h"

namespace spc
{
DtiClassType StratigraphicModelSingleAttitude::Type ("StratigraphicModelSingleAttitude", &StratigraphicModelBase::Type);

StratigraphicModelSingleAttitude::StratigraphicModelSingleAttitude()
{
    attitude_ = new Attitude();

}

StratigraphicModelSingleAttitude::StratigraphicModelSingleAttitude(const StratigraphicModelSingleAttitude &model): StratigraphicModelBase(model)
{
    attitude_ = new Attitude( model.getAttitude() );
}

StratigraphicModelSingleAttitude::StratigraphicModelSingleAttitude(const Attitude &attitude)
{

    attitude_ = new  Attitude(attitude);
}


float StratigraphicModelSingleAttitude::getScalarFieldValue(const Vector3f &point) const
{

    DLOG(INFO) << "evaluating scalar field at " << point.transpose();

    if (this->getIsElastic())
    {
        DLOG(INFO) << "model is elastic";
        return (attitude_->distanceTo(point) * elastic_parameter_ + getStratigraphicShift()) ;
    }
    else
    {
        DLOG(INFO) << "model is NOT elastic";
        float d = attitude_->distanceTo(point);
        float value =  d + getStratigraphicShift();
        DLOG(INFO) << "d is " << d << " value is "<< value;
        return value;
    }


}

Vector3f StratigraphicModelSingleAttitude::getScalarFieldGradient(const Vector3f
                                                     &point) const
{
    return attitude_->getUnitNormal();
}

Vector3f StratigraphicModelSingleAttitude::getPointAtStratigraphicPosition(float sp) const
{
    return attitude_->getPosition() + attitude_->getUnitNormal()
            * (sp - getStratigraphicShift());
}

Attitude StratigraphicModelSingleAttitude::getAttitude() const
{
    return *attitude_;
}

void StratigraphicModelSingleAttitude::setNormal(Vector3f n)
{
    attitude_->setNormal(n);
}

Vector3f StratigraphicModelSingleAttitude::getNormal() const
{
    return attitude_->getNormal();
}

void StratigraphicModelSingleAttitude::applyTransform(const GeometricElement3DBase::TransformT &transform)
{
    attitude_->applyTransform(transform);
}

} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::StratigraphicModelSingleAttitude)
