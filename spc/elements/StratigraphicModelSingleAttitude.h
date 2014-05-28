#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/Attitude.h>
#include <spc/elements/StratigraphicModelBase.h>

namespace spc
{


///
/// \brief The SingleAttitudeModel class represent a stratigraphic "meter" or model.
///
///
class SingleAttitudeModel: public StratigraphicModelBase, public spcObject
{
public:
    SPC_OBJECT(SingleAttitudeModel)

    /// def const
    SingleAttitudeModel() : additional_shift_(0.0)
    {

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

        attitude_ = attitude;
    }

    SingleAttitudeModel(const Attitude::Ptr attitude)
    {
        additional_shift_ = 0.0;

        attitude_ = *attitude;
    }




    /// inherited from StratigraphicModelBase
    virtual float getScalarFieldValue(const Vector3f &point) const;

    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const;

    Vector3f getPointAtStratigraphicPosition(float sp) const
    {
        return attitude_.getPosition() + attitude_.getUnitNormal() * (sp - additional_shift_ );
    }


    float getAdditionalShift() const
    {
        return additional_shift_;
    }

    void setAdditionalShift(float additional_shift)
    {
        additional_shift_ = additional_shift;
    }


    void setAttitude(const Attitude &attitude)
    {
        attitude_ = attitude;
    }


    Attitude getAttitude() const
    {
        return attitude_;
    }

    void setNormal (Vector3f n)
    {
        attitude_.setNormal(n);
    }

private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar )
    {
        ar( cereal::base_class<spc::StratigraphicModelBase>( this ),
            cereal::base_class<spc::spcObject>( this ),
            additional_shift_,
            attitude_);
    }


protected:
    float additional_shift_;

    Attitude attitude_;

};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
