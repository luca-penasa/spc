#include "SetUpNewSeries.h"
#include <ccArrow.h>

#include <dialogs/AddNewSeries.h>

//#include <ccOutOfCore/ccMyCCHObject.h>
//#include <ccOutOfCore/ccAdditionalCaster.h>

//#include <spc/geology/stratigraphic_model_base.h>
#include <ccOutOfCore/ccSingleAttitudeModel.h>

#include <ccHObjectCaster.h>
#include <CloudMapper.h>

#include <iostream>
#include <qPCL/PclUtils/utils/cc2sm.h>

#include <spc/elements/generic_cloud.h>

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
    ccHObject * cloud_obj = m_dialog->getSelectedCloud();

    if (!mod_obj || !cloud_obj)
        return -1;


    ccPointCloud * cloud = static_cast<ccPointCloud *> (cloud_obj);
    ccSingleAttitudeModel * model = static_cast<ccSingleAttitudeModel *> (mod_obj);


    std::cout <<  "size  here: " << cloud->size() << std::endl;


    // try a conversion
    cc2smReader reader;
    reader.setInputCloud(cloud);
    sensor_msgs::PointCloud2 sm_cloud = reader.getXYZ();

    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
    pcl::fromROSMsg(sm_cloud, pcl_cloud);

    spc::GenericCloud * mycloud2 = new CloudWrapper<pcl::PointCloud<pcl::PointXYZ> > (&pcl_cloud);


    //try the mapper
    spc::GenericCloud * mycloud = new CloudWrapper<ccPointCloud>(cloud);
    float x, y, z;
    mycloud->getPoint(0, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;

    mycloud2->getPoint(0, x, y, z);
    std::cout << x << " " << y << " " << z << std::endl;

    return 1;
}

int SetUpNewSeries::openInputDialog()
{

    ccHObject::Container clouds, models;
    getAllEntitiesOfType(CC_POINT_CLOUD, clouds);
    getAllEntitiesThatHaveMetaData("[qGEO][ccSingleAttitudeModel]", models);

    m_dialog = new AddNewSeriesDlg;

    m_dialog->setInputClouds(clouds);
    m_dialog->setInputModels(models);

    return m_dialog->exec();


}


int SetUpNewSeries::checkSelected()
{
    return 1;
}
