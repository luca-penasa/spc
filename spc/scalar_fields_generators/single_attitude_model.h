#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/attitude.h>
#include <spc/scalar_fields_generators/DynamicScalarFieldGenerator.h>
//#include <boost/serialization/shared_ptr.hpp>

namespace spc
{

///
/// \brief The SingleAttitudeModel class represent a stratigraphic "meter" or model.
///
///
class SingleAttitudeModel: public DynamicScalarFieldGenerator, public spcElementBase
{
public:
    spcTypedefSmartPointersMacro(SingleAttitudeModel)

    /// def const
    SingleAttitudeModel() : additional_shift_(0.0)
    {
        attitude_ = Attitude::Ptr(new Attitude);
    }

    /// copy const
    SingleAttitudeModel(const SingleAttitudeModel & model)
    {
        additional_shift_ =  model.getAdditionalShift();
        attitude_ = model.attitude_;
    }

    /// copy from an attitude
    SingleAttitudeModel(const Attitude & attitude)
    {
        additional_shift_ = 0.0;

        attitude_ = spcMakeSharedPtrMacro<Attitude>(attitude);
    }

    SingleAttitudeModel(const Attitude::Ptr attitude)
    {
        additional_shift_ = 0.0;

        attitude_ = attitude;
    }




    /// inherited from StratigraphicModelBase
    virtual float getScalarFieldValue(const Vector3f &point) const;

    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const;

    Vector3f getPointAtStratigraphicPosition(float sp) const
    {
        return attitude_->getPosition() + attitude_->getUnitNormal() * (sp - additional_shift_ );
    }


    float getAdditionalShift() const
    {
        return additional_shift_;
    }

    void setAdditionalShift(float additional_shift)
    {
        additional_shift_ = additional_shift;
    }


    void setAttitude(Attitude::Ptr &attitude)
    {
        attitude_ = attitude;
    }


    Attitude::Ptr getAttitude() const
    {
        return attitude_;
    }

    void setNormal (Vector3f n)
    {
        attitude_->setNormal(n);
    }




protected:
    float additional_shift_;

    Attitude::Ptr attitude_;

};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
