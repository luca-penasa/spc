#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/attitude.h>
#include <spc/stratigraphy/stratigraphic_model_base.h>

namespace spc
{


class SingleAttitudeModel: public StratigraphicModelBase, public Attitude
{
public:


    typedef boost::shared_ptr<SingleAttitudeModel> Ptr;

    /// def const
    SingleAttitudeModel() {}

    /// inherited from StratigraphicModelBase
    virtual float getStratigraphicPosition(const Vector3f &point);

    virtual Vector3f getStratigraphicNormal(const Vector3f &point);

};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
