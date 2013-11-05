#include "SetUpNewSeries.h"
#include <ccArrow.h>

#include <dialogs/AddNewSeries.h>

#include <ccOutOfCore/ccMyCCHObject.h>

SetUpNewSeries::SetUpNewSeries(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Add a new time series",
                                                                                                   "Add a new time series",
                                                                                                   "Add a new time series",
                                                                                                   ":/toolbar/icons/new_series.png")
                                                                              , parent_plugin)
{

}

int SetUpNewSeries::compute()
{
    return 1;
}

int SetUpNewSeries::openInputDialog()
{
    m_dialog = new AddNewSeries;

    ccHObject::Container objsa;
    this->getAllEntitiesOfType(CC_2D_RUBBERBAND_LABEL, objsa);
    m_dialog->setInputAreas(objsa);

    ccHObject::Container objsb;
    this->getAllEntitiesOfType(CC_POINT_CLOUD, objsb);
    m_dialog->setInputClouds(objsb);

    ccHObject::Container objsc;
    this->getAllEntitiesOfType((CC_CLASS_ENUM) MY_CC_SINGLE_PLANE_MODEL, objsc);
    m_dialog->setInputModels(objsc);

    m_dialog->exec();


    return 1;
}


int SetUpNewSeries::checkSelected()
{
    return 1;
}
