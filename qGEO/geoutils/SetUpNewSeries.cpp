#include "SetUpNewSeries.h"
#include <ccArrow.h>

#include <dialogs/AddNewSeries.h>

#include <ccOutOfCore/ccMyCCHObject.h>
#include <ccOutOfCore/ccAdditionalCaster.h>

#include <spc/geology/stratigraphic_model_base.h>

#include <ccHObjectCaster.h>

SetUpNewSeries::SetUpNewSeries(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Add a new time series",
                                                                                                   "Add a new time series",
                                                                                                   "Add a new time series",
                                                                                                   ":/toolbar/icons/new_series.png")
                                                                              , parent_plugin)
{

}

int SetUpNewSeries::compute()
{
    //get the objects from the combo
    ccHObject * mod_obj = m_dialog->getSelectedModel();

//    std::cout << mod_obj << std::endl;

//    if (!mod_obj)
//        return -1;

//    ccHObject::Container cont;
//    this->getAllEntitiesOfType((CC_CLASS_ENUM) MY_CC_OBJECT, cont);

//    std::cout << cont.size() << " of " << (CC_CLASS_ENUM) MY_CC_SINGLE_PLANE_MODEL << std::endl;
//    std::cout << cont.size() << " of " <<  MY_CC_SINGLE_PLANE_MODEL << std::endl;
//    std::cout << mod_obj->getClassID() << std::endl;



    spc::StratigraphicModelBase * model;
    std::cout <<"a" << std::endl;

    std::cout << mod_obj->getClassID() << std::endl;
    std::cout <<"b" << std::endl;



    if (mod_obj->isA( static_cast<CC_CLASS_ENUM>(MY_CC_SINGLE_PLANE_MODEL)) )//in a future we will have other models
    {

        std::cout << "here we are" << std::endl;


        ccSinglePlaneStratigraphicModel * mo = ccAdditionalCaster::ToCCSinglePlaneStatigraphicModel( mod_obj );
        model = static_cast<spc::StratigraphicModelBase *> (mo);
    }
    else
        return -1; //just to be sure



    //get also the clou and the area
    ccPointCloud * cloud = ccHObjectCaster::ToPointCloud( m_dialog->getSelectedCloud() );
    cc2DRubberbandLabel * area = ccHObjectCaster::To2DRubberbandLabel(m_dialog->getSelectedArea());

    if (!cloud )
        return -1;

    if (!area)
        ccLog::Warning("No area selected. using the whole cloud!");





    return 1;
}

int SetUpNewSeries::openInputDialog()
{
    m_dialog = new AddNewSeriesDlg;

    ccHObject::Container objsa;
    this->getAllEntitiesOfType(CC_2D_RUBBERBAND_LABEL, objsa);
    m_dialog->setInputAreas(objsa);

    ccHObject::Container objsb;
    this->getAllEntitiesOfType(CC_POINT_CLOUD, objsb);
    m_dialog->setInputClouds(objsb);

    ccHObject::Container objsc;


    this->getAllEntitiesOfType((CC_CLASS_ENUM) MY_CC_SINGLE_PLANE_MODEL, objsc);

    std::cout << "number of models in tree: " << objsc.size() << std::endl;

    m_dialog->setInputModels(objsc);

    return m_dialog->exec();


}


int SetUpNewSeries::checkSelected()
{
    return 1;
}
