#pragma once
#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/StratigraphicModelBase.h>
#include <spc/elements/Attitude.h>




namespace spc
{

///
/// \brief The SingleAttitudeModel class represent a stratigraphic "meter" or
/// model.
///
///
class StratigraphicModelSingleAttitude : public StratigraphicModelBase
{
public:
    SPC_ELEMENT(StratigraphicModelSingleAttitude)
    EXPOSE_TYPE
    /// def const
    StratigraphicModelSingleAttitude();

    /// copy const
    StratigraphicModelSingleAttitude(const StratigraphicModelSingleAttitude &model);

    /// copy from an attitude
    StratigraphicModelSingleAttitude(const Attitude &attitude);

    //    StratigraphicModelSingleAttitude(const Attitude::Ptr attitude);

    float predictStratigraphicPosition(const Eigen::Vector3f & point) const override
    {
        return this->getScalarFieldValue(point);
    }

    /// inherited from StratigraphicModelBase
    virtual float getScalarFieldValue(const Vector3f &point) const override;

    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const override;

    Vector3f getPointAtStratigraphicPosition(float sp) const;





    Attitude getAttitude() const;

    void setNormal(Vector3f n);

    Vector3f getNormal() const;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar, std::uint32_t const version)
    {
        ar(cereal::base_class<spc::StratigraphicModelBase>(this),
           *attitude_);
    }

protected:

    Attitude * attitude_;

    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override;
};

} // end nspace

CEREAL_CLASS_VERSION(spc::StratigraphicModelSingleAttitude, 1)

#endif // SINGLE_PLANE_NORMAL_MODEL_H
