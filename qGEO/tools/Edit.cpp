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
        std::cout << "positive!" << std::endl;
        ccMyBaseObject * mobj = static_cast<ccMyBaseObject * >(selected);

        if (!mobj)
        {
            return 0;
        }

        if (mobj->getHasEditDlg())
        {
            std::cout << "has edit dialog" << std::endl;
            return 1;
        }

    }
    else
        return 0;
}
