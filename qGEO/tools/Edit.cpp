#include "Edit.h"
#include <ccOutOfCore/ccAttitude.h>
#include <ccOutOfCore/ccSingleAttitudeModel.h>


#include <iostream> // for debug

#include <qGEO/qGEO.h>

Edit::Edit(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Edit selected element",
                                                                         "Edit selected element",
                                                                         "Edit selected element",
                                                                         ":/toolbar/icons/edit.png"), parent_plugin)
{
    this->setShowProgressBar(false);
}


int Edit::compute()
{
    ccHObject * selected = getSelectedEntityAsCCHObject(); //we are sure it exists!

    ccMyBaseObject * myobj = static_cast<ccMyBaseObject *> (selected);
    if (!myobj)
        return 0;


    myobj->showEditDlg();
    return 1;
}




int Edit::checkSelected()
{
    ccHObject * selected = getSelectedEntityAsCCHObject();
    if (selected && selected->hasMetaData("[qGEO]"))
    {

        ccMyBaseObject * mobj = static_cast<ccMyBaseObject * >(selected);

        if (!mobj)
        {
            std::cout << "problem casting!" << std::endl;
            return 0;
        }

        if (mobj->getHasEditDlg())
        {
            std::cout << "has edit dialog" << std::endl;
            return 1;
        }
        else
        {
            std::cout << "it does not have a edit dialog" << std::endl;
        }


    }
    else
        return 0;
}
