#ifndef CCTIMESERIESGENERATOR_H
#define CCTIMESERIESGENERATOR_H

#include "ccMyBaseObject.h"
#include <spc/methods/time_series_generator.h>
#include <QIcon>

class ccTimeSeriesGenerator: public ccMyBaseObject, public spc::TimeSeriesGenerator<float>
{
public:
    ccTimeSeriesGenerator();

    virtual QIcon * getIcon() const
    {
        return new QIcon(QString::fromUtf8(":/toolbar/icons/tseries_generator.png"));
    }

protected:
    ///
    /// \brief initEditDlg is the only method you MUST reimplement in subclasses
    ///         if you want to hve a dlg
    ///
    virtual void initEditDlg();


    virtual void updateEditDlg();


};


Q_DECLARE_METATYPE(ccTimeSeriesGenerator)

#endif // CCTIMESERIESGENERATOR_H
