#ifndef CC_SINGLE_PLANE_NORMAL_MODEL_H
#define CC_SINGLE_PLANE_NORMAL_MODEL_H

#include "ccHObject.h"

#include <spc/geology/single_plane_stratigraphic_model.h>

#include "ccMyCCHObject.h"

class ccSinglePlaneStratigraphicModel:  public ccHObject, public spc::SinglePlaneStratigraphicModel
{
public:
    ccSinglePlaneStratigraphicModel();

    ccSinglePlaneStratigraphicModel(const Vector3f normal, const float dist);

    ccSinglePlaneStratigraphicModel(const spc::SinglePlaneStratigraphicModel model);


    //! Returns class ID
    virtual CC_CLASS_ENUM getClassID() const {return static_cast<CC_CLASS_ENUM> (MY_CC_SINGLE_PLANE_MODEL);}
};

#endif // CCSINGLEPLANENORMALMODEL_H
