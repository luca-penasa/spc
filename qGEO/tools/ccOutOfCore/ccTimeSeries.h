#ifndef CCTIMESERIES_H
#define CCTIMESERIES_H
#include "ccMyBaseObject.h"
#include <spc/time_series/equally_spaced_time_series.h>
#include <QIcon>

class ccTimeSeries:  public ccMyBaseObject, public spc::EquallySpacedTimeSeries<float>
{
public:

    /// this is goind to create strange things with the ref count system of qt, maybe
    typedef typename boost::shared_ptr<ccTimeSeries> Ptr;
    ccTimeSeries();

    ccTimeSeries(const ccTimeSeries & other);

    ccTimeSeries(const spc::EquallySpacedTimeSeries<float> &other);

    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/tseries.png"));
    }


protected:
    void initMetaData();
};

#endif // CCTIMESERIES_H
