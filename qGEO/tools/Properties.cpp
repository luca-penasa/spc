#include "Properties.h"


Properties::Properties(ccPluginInterface *parent_plugin): BaseFilter(FilterDescription("Open Properties Dialog",
                                                                                       "Opens the properties editor for selected object",
                                                                                       "Properties",
                                                                                       ":/toolbar/icons/plot_dlg.png"), parent_plugin)
{
    this->setShowProgressBar(false);

    m_dialog = new ccProperties();
}


int Properties::openOutputDialog()
{
    if (m_dialog->isVisible())
        m_dialog->setVisible(false);

    else
    {
        m_dialog->setVisible(true);
//        m_dialog->raise();
    }




//    segtool->linkWith(this->)
}
