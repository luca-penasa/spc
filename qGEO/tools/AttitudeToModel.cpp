#include "AttitudeToModel.h"
#include <ccOutOfCore/ccAttitude.h>
#include <ccOutOfCore/ccSingleAttitudeModel.h>


#include <iostream> // for debug

#include <qGEO/qGEO.h>

AttitudeToModel::AttitudeToModel(ccPluginInterface *parent_plugin) : BaseFilter(FilterDescription(   "Create Stratigraphic Model from one attitude",
                                                                         "Create Stratigraphic Model from one attitude",
                                                                         "Create Stratigraphic Model from one attitude",
                                                                         ":/toolbar/icons/AttitudeToModel.png"), parent_plugin)
{
this->setShowProgressBar(false);
}


int AttitudeToModel::compute()
{


    ccHObject * selected = getSelectedEntityAsCCHObject(); //we are sure it exists!

    ccAttitude * att = static_cast<ccAttitude *> (selected);

    if (!att)
    {
        std::cout << "NOT DEFINED" << std::endl;
        return -1;
    }

    ccSingleAttitudeModel * model = new ccSingleAttitudeModel (*att);

//    selected->addChild(model);

//    qGEO::theInstance()->getMainAppInterface()->dbRootObject()->updateModificationTime();
    newEntity(model);



    return 1;
}




int AttitudeToModel::checkSelected()
{
    ccHObject * selected = getSelectedEntityAsCCHObject();
    if (selected && selected->hasMetaData("[qGEO][ccAttitude]"))
        return 1;
    else
        return 0;
}
