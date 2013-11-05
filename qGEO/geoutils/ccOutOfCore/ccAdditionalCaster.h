#ifndef CCADDITIONALCASTER_H
#define CCADDITIONALCASTER_H

#include "ccSinglePlaneStratigraphicModel.h"
#include <spc/geology/stratigraphic_model_base.h>


class ccAdditionalCaster
{
 public:
    static ccSinglePlaneStratigraphicModel * ToCCSinglePlaneStatigraphicModel(ccHObject * obj)
    {
        return obj && obj->isA(static_cast<CC_CLASS_ENUM> (MY_CC_SINGLE_PLANE_MODEL)) ? static_cast<ccSinglePlaneStratigraphicModel*>(obj) : 0;
    }
};

#endif // CCADDITIONALCASTER_H
