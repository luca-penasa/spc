#include "OpenPlotsDialog.h"
#include <qGEO/qGEO.h>

OpenPlotsDialog::OpenPlotsDialog(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Show 2d Plots ",
                                                     "Open the 2D plots dialog",
                                                     "Open the 2D plots dialog",
                                                     ":/toolbar/icons/plot_dlg.png" ), parent_plugin)
{
    this->setShowProgressBar(false);
}

int
OpenPlotsDialog::compute()
{

    ccPluginInterface * plugin = getParentPlugin();

    qGEO * qgeo = static_cast<qGEO *> (plugin);

    PlotterDlg * dlg = qgeo->getPlotter();

    assert (dlg); //just to be sure

    if (!dlg->isVisible())
    {
        dlg->show();
        dlg->update();
    }
    else
        dlg->setVisible(false);

    return 1;
}

