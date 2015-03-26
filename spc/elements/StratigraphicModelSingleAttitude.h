#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/Attitude.h>
#include <spc/elements/StratigraphicModelBase.h>

namespace spc
{

///
/// \brief The SingleAttitudeModel class represent a stratigraphic "meter" or
/// model.
///
///
	class SPC_LIB_API StratigraphicModelSingleAttitude : public StratigraphicModelBase
{
public:
    SPC_ELEMENT(StratigraphicModelSingleAttitude)
EXPOSE_TYPE
    /// def const
    StratigraphicModelSingleAttitude()
    {
    }

    /// copy const
    StratigraphicModelSingleAttitude(const StratigraphicModelSingleAttitude &model): StratigraphicModelBase(model)
    {
        attitude_ = model.attitude_;
    }

    /// copy from an attitude
    StratigraphicModelSingleAttitude(const Attitude &attitude)
    {

        attitude_ = attitude;
    }

    StratigraphicModelSingleAttitude(const Attitude::Ptr attitude)
    {

        attitude_ = *attitude;
    }

    float predictStratigraphicPosition(const Eigen::Vector3f & point) const override
    {
        return this->getScalarFieldValue(point);
    }

    /// inherited from StratigraphicModelBase
    virtual float getScalarFieldValue(const Vector3f &point) const;

    virtual Vector3f getScalarFieldGradient(const Vector3f &point) const;

    Vector3f getPointAtStratigraphicPosition(float sp) const
    {
        return attitude_.getPosition() + attitude_.getUnitNormal()
                                         * (sp - getStratigraphicShift());
    }




    void setAttitude(const Attitude &attitude)
    {
        attitude_ = attitude;
    }

    Attitude getAttitude() const
    {
        return attitude_;
    }

    void setNormal(Vector3f n)
    {
        attitude_.setNormal(n);
    }

    Vector3f getNormal() const
    {
        return attitude_.getNormal();
    }

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<spc::StratigraphicModelBase>(this),
           attitude_);
    }

protected:

    Attitude attitude_;
};

} // end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
