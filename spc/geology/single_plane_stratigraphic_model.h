#ifndef SINGLE_PLANE_NORMAL_MODEL_H
#define SINGLE_PLANE_NORMAL_MODEL_H

#include <spc/elements/plane.h>
#include <spc/geology/stratigraphic_model_base.h>

namespace spc
{

///
/// \brief The SinglePlaneStratigraphicModel class is an implementation of the virtual StratigraphicModelBase
/// this model is simply defined as STRATIRAPHIC POSITION OF A POINT = SIGNED DISTANCE OF THE POINT FROM THE PLANE
///
class SinglePlaneStratigraphicModel: public StratigraphicModelBase, public Plane
{
public:


    typedef boost::shared_ptr<SinglePlaneStratigraphicModel> Ptr;

    /// def const
    SinglePlaneStratigraphicModel();

    // from StratigraphicModelBase
    virtual float getStratigraphicPosition(const Vector3f &point)
    {
        return this->distanceTo(point);
    }

    virtual Vector3f getStratigraphicNormal(const Vector3f &point)
    {
        return this->getUnitNormal();
    }

    virtual void setParameters(const VectorXf &parameters)
    {
        setPlaneParameters(parameters);
    }


};

}//end nspace

#endif // SINGLE_PLANE_NORMAL_MODEL_H
