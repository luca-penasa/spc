#ifndef CC_SINGLE_PLANE_NORMAL_MODEL_H
#define CC_SINGLE_PLANE_NORMAL_MODEL_H

#include "ccGenericPrimitive.h"

#include <spc/geology/single_plane_stratigraphic_model.h>

class ccSinglePlaneStratigraphicModel:  public ccGenericPrimitive, public spc::SinglePlaneStratigraphicModel
{
public:
    ccSinglePlaneStratigraphicModel();
};

#endif // CCSINGLEPLANENORMALMODEL_H
