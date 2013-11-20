#include "OpenPlotsDialog.h"
#include <qGEO/qGEO.h>
#include <PlotterDlg.h>

OpenPlotsDialog::OpenPlotsDialog(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Show 2d Plots ",
                                                     "Open the 2D plots dialog",
                                                     "Open the 2D plots dialog",
                                                                                                     ":/toolbar/icons/plot_dlg.png" ), parent_plugin), m_plotter_dialog(0)
{
    this->setShowProgressBar(false);
    m_plotter_dialog = new PlotterDlg();
    m_plotter_dialog->setVisible(false);
}

int
OpenPlotsDialog::compute()
{
    return 1;
}

int OpenPlotsDialog::openInputDialog()
{
    if (!m_plotter_dialog->isVisible())
        m_plotter_dialog->show();

    else
        m_plotter_dialog->setVisible(false);

    return 1;
}

