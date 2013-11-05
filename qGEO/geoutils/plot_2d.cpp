#include "plot_2d.h"

Plot2D::Plot2D(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Show 2d Plots ",
                                                     "Open the 2D plots dialog",
                                                     "Open the 2D plots dialog",
                                                     ":/toolbar/icons/plot_dlg.png" ), parent_plugin)
{
    this->setShowProgressBar(false);
    m_dialog = new ccCurvePlotterDlg;
}

int
Plot2D::compute()
{

    return 1;
}

int Plot2D::openOutputDialog()
{
    if (m_dialog->isVisible())
        m_dialog->setVisible(false);

    else
    {
        m_dialog->setVisible(true);
    }
    return 1;
}
