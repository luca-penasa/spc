#include "ccSinglePlaneStratigraphicModel.h"

ccSinglePlaneStratigraphicModel::ccSinglePlaneStratigraphicModel()
{
}

ccSinglePlaneStratigraphicModel::ccSinglePlaneStratigraphicModel(const Vector3f normal, const float dist)
{
    setUnitNormal(normal);
    setD(dist);
}

ccSinglePlaneStratigraphicModel::ccSinglePlaneStratigraphicModel(const spc::SinglePlaneStratigraphicModel model)
{
    setUnitNormal(model.getUnitNormal());
    setD(model.getD());
}



