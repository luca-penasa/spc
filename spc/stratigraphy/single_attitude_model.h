#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/attitude.h>
#include <spc/stratigraphy/stratigraphic_model_base.h>

namespace spc
{

///
/// \brief The SingleAttitudeModel class represent a stratigraphic "meter" or model.
///
///
class spcSingleAttitudeModel: public spcStratigraphicModelBase, public spcElementBase
{
public:


    typedef boost::shared_ptr<spcSingleAttitudeModel> Ptr;
    typedef boost::shared_ptr<const spcSingleAttitudeModel> ConstPtr;

    /// def const
    spcSingleAttitudeModel() : additional_shift_(0.0), spcElementBase("spcSingleAttitudeModel")
    {
        attitude_ = spcAttitude::Ptr(new spcAttitude);
    }

    /// copy const
    spcSingleAttitudeModel(const spcSingleAttitudeModel & model):  spcElementBase("spcSingleAttitudeModel")
    {
        additional_shift_ =  model.getAdditionalShift();
        attitude_ = model.attitude_;
    }

    /// copy from an attitude
    spcSingleAttitudeModel(const spcAttitude & attitude):  spcElementBase("spcSingleAttitudeModel")
    {
        additional_shift_ = 0.0;

        attitude_ = boost::make_shared<spcAttitude>(attitude);
    }




    /// inherited from StratigraphicModelBase
    virtual float getStratigraphicPosition(const Vector3f &point);

    virtual Vector3f getStratigraphicNormal(const Vector3f &point);

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


    void setAttitude(spcAttitude::Ptr &attitude)
    {
        attitude_ = attitude;
    }


    spcAttitude::Ptr getAttitude() const
    {
        return attitude_;
    }

    void setNormal (Vector3f n)
    {
        attitude_->setNormal(n);
    }




protected:
    float additional_shift_;

    spcAttitude::Ptr attitude_;

protected:
    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {

        /// also the attitude must be serialized
        ar & boost::serialization::base_object<spcElementBase>(*this);
        ar & BOOST_SERIALIZATION_NVP(additional_shift_);
        ar & BOOST_SERIALIZATION_NVP(*attitude_);

    }

};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
