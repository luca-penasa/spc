#include "ccSinglePlaneStratigraphicModel.h"

ccSingleAttitudeModel::ccSingleAttitudeModel()
{
    QVariant var(QString("A stratigrahic model"));

    setMetaData(QString("[qGEO][ccSinglePlaneStratigraphicModel]"), var);

}

//ccSingleAttitudeModel::ccSingleAttitudeModel(const Vector3f normal, const float dist)
//{
//    QVariant var(QString("A stratigrahic model"));

//    setMetaData(QString("[qGEO][ccSinglePlaneStratigraphicModel]"), var);
//    setNormal(normal);
//    setD(dist);
//}

ccSingleAttitudeModel::ccSingleAttitudeModel(const spc::SingleAttitudeModel model)
{
    QVariant var(QString("A stratigrahic model"));

    setMetaData(QString("[qGEO][ccSinglePlaneStratigraphicModel]"), var);
    setNormal(model.getUnitNormal());
//    setD(model.getD());
}



