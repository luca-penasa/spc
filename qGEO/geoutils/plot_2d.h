#ifndef PLOT_2D_H
#define PLOT_2D_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
#include <dialogs/ccCurvePlotterDlg.h>


class Plot2D : public BaseFilter
{
public:
    Plot2D(ccPluginInterface * parent_plugin = 0);

    virtual int compute();

    int checkSelected() {return 1;}

    int openOutputDialog() ;

    ccCurvePlotterDlg * getPlot() {return m_dialog;}

private:
    ccCurvePlotterDlg * m_dialog;
};

#endif // PLOT_2D_H
