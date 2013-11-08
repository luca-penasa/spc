#include "FitGeologicalOrientation.h"
#include <qPCL/PclUtils/utils/cc2sm.h>


#include <ccOutOfCore/ccSinglePlaneStratigraphicModel.h>

#include <ccArrow.h>

#include <ccHObjectCaster.h>

#include <ccOutOfCore/ccAttitude.h>

#include <spc/estimators/attitude_estimator.h>



FitGeologicalOrientation::FitGeologicalOrientation(ccPluginInterface * parent_plugin): BaseFilter(FilterDescription(   "Fit A geological orientation",
                                                                                              "Fit a geological orientation",
                                                                                              "Use a set of points to fit a geological orientation",
                                                                                              ":/toolbar/icons/stratigraphic_normal.png")
                                                                                                        , parent_plugin)
{
//    m_dialog = 0;
}

int
FitGeologicalOrientation::compute()
{
    ccHObject::Container ents;



    ccHObject::Container entities;
    this->getSelectedEntitiesThatAreCCPointCloud(entities);

    //convert them to pcl point clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> clouds;
    for (auto ent: entities)
    {
        ccPointCloud * cloud = ccHObjectCaster::ToPointCloud( ent );

        cc2smReader reader;
        reader.setInputCloud(cloud);
        sensor_msgs::PointCloud2 sm_cloud = reader.getXYZ();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(sm_cloud, *pcl_cloud);

        clouds.push_back(pcl_cloud);

    }

    //set up an estimator
    spc::AttitudeEstimator estimator;

    for (auto cloud: clouds) //add the single clouds
    {
        estimator.addInputCloud(cloud);

        std::cout << "adding cloud with points: " << cloud->size() << std::endl;
    }

    std::cout << "starting opimization" << std::endl;

    spc::SingleAttitudeModel model;

    int status = estimator.estimate(model);

    //we got a model return it as a ccObject
    ccSingleAttitudeModel * ccmodel = new ccSingleAttitudeModel(model);
    ccmodel->setName(QString("Single Attitude Stratigraphic Model"));
    newEntity(ccmodel);

    if (status == 0)
    {
        std::cout << "NOT CONVERGED!" << std::endl;
        return 0;
    }

    std::cout << "ended optimizing" << std::endl;

    Vector3f n = model.getUnitNormal();

    std::vector<float> sps = estimator.getStratigraphicPositionsOfClouds();

    //now for each entity we send back a ccOrientation for visualizing the result
    for (int i= 0; i < clouds.size(); ++i)
    {

        //each cloud will have the same normal but different centers

        //get the center
        Vector4f centroid;
        pcl::compute3DCentroid(*(clouds.at(i)), centroid);

        Vector3f c = centroid.segment(0,3);

        CCVector3 cc (c(0), c(1), c(2));
        CCVector3 cn (n(0), n(1), n(2));

        ccAttitude * orientation = new ccAttitude (cc, cn);

        orientation->setName("Orientation");

        orientation->setVisible(true);

        newEntity(orientation);        
    }


//    //we also store the used cloud in a group, child of the orientation.
//    ccHObject * folder = new ccHObject;
//    folder->setName("Fitted clouds");
//    folder->setEnabled(false);
//    for (auto ent: entities)
//        folder->addChild( ccHObjectCaster::ToGenericPointCloud(ent)->clone() );

//    orientation->addChild(folder);

    return 1;
}

int FitGeologicalOrientation::checkSelected()
{
    ccHObject::Container ents, planes, polys;
    getSelectedEntitiesThatAre(CC_POINT_CLOUD, ents);

    if (ents.size() > 0)
        return 1;
    else
        return -1;
}

