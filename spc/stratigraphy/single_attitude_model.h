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
class SingleAttitudeModel: public StratigraphicModelBase, public Attitude
{
public:


    typedef boost::shared_ptr<SingleAttitudeModel> Ptr;

    /// def const
    SingleAttitudeModel();

    /// inherited from StratigraphicModelBase
    virtual float getStratigraphicPosition(const Vector3f &point);

    virtual Vector3f getStratigraphicNormal(const Vector3f &point);

    Vector3f getPointAtStratigraphicPosition(float sp)
    {
        return getPosition() + getUnitNormal() * (sp - additional_shift_ );
    }


    float getAdditionalShift() const
    {
        return additional_shift_;
    }

    void setAdditionalShift(float additional_shift)
    {
        additional_shift_ = additional_shift;
    }

protected:
    float additional_shift_;

};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
