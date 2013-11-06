#ifndef ASSOCIATECLOUDTOMODEL_H
#define ASSOCIATECLOUDTOMODEL_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/AddNewSeries.h>


class SetUpNewSeries : public BaseFilter
{
public:
    SetUpNewSeries(ccPluginInterface * parent_plugin);

protected:
    virtual int compute();

    virtual int openInputDialog();

    virtual int checkSelected();

private:
    AddNewSeriesDlg * m_dialog;

};

#endif // ASSOCIATECLOUDTOMODEL_H
