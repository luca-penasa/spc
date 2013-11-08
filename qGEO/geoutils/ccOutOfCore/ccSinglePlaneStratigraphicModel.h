#ifndef CC_SINGLE_PLANE_NORMAL_MODEL_H
#define CC_SINGLE_PLANE_NORMAL_MODEL_H

#include "ccHObject.h"

#include <spc/stratigraphy/single_attitude_model.h>

//#include "ccMyCCHObject.h"

class ccSingleAttitudeModel:  public ccHObject, public spc::SingleAttitudeModel
{
public:
    ccSingleAttitudeModel();

//    ccSingleAttitudeModel(const Vector3f normal, const float dist);

    ccSingleAttitudeModel(const spc::SingleAttitudeModel model);

    virtual QString getName() {return QString("SinglePlaneStratigraphicModel");}
};

#endif // CCSINGLEPLANENORMALMODEL_H
