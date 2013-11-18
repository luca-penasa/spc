#ifndef CCTIMESERIES_H
#define CCTIMESERIES_H
#include "ccMyBaseObject.h"
#include <spc/time_series/equally_spaced_time_series.h>
#include <QIcon>

class ccTimeSeries: public ccMyBaseObject, public spc::EquallySpacedTimeSeries<float>
{
public:
    ccTimeSeries();

    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/tseries.png"));
    }
};

#endif // CCTIMESERIES_H
