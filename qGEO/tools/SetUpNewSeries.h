#ifndef ASSOCIATECLOUDTOMODEL_H
#define ASSOCIATECLOUDTOMODEL_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/ccTimeSeriesGeneratorEditorDlg.h>


class SetUpNewSeries : public BaseFilter
{
public:
    SetUpNewSeries(ccPluginInterface * parent_plugin);

protected:
    virtual int compute();

    virtual int checkSelected();



};

#endif // ASSOCIATECLOUDTOMODEL_H
