#include "ccMyBaseObject.h"

ccMyBaseObject::ccMyBaseObject()
{
    QVariant var(QString("QGEO plugin object"));
    setMetaData(QString("[qGEO]"), var);
}


Q_DECLARE_METATYPE(ccPointCloud)
