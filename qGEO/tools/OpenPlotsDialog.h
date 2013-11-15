#ifndef PLOT_2D_H
#define PLOT_2D_H

#include <qPCL/PclUtils/filters/BaseFilter.h>
//#include <dialogs/ccCurvePlotterDlg.h>


class OpenPlotsDialog : public BaseFilter
{
public:
    OpenPlotsDialog(ccPluginInterface * parent_plugin = 0);

    virtual int compute();

    int checkSelected() {return 1;}
};

#endif // PLOT_2D_H
