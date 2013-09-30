#include "fitGeologicalOrientation.h"
#include <qPCL/PclUtils/utils/cc2sm.h>

#include <spc/geology/normal_estimator.h>

#include <ccPlaneOrientation.h>

#include <ccArrow.h>

#include <ccHObjectCaster.h>





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

    ccHObject::Container entities;
    this->getSelectedEntitiesThatAreCCPointCloud(entities);

    spc::NormalEstimator estimator;

    for (int i = 0 ; i < entities.size(); ++i)
    {


        ccPointCloud * cloud = ccHObjectCaster::ToPointCloud( entities[i] );

        cc2smReader reader;
        reader.setInputCloud(cloud);
        sensor_msgs::PointCloud2 sm_cloud = reader.getXYZ();

        pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        pcl::fromROSMsg(sm_cloud, *pcl_cloud);

        estimator.addPlanarCloud(pcl_cloud);

        std::cout << "adding cloud with points: " << pcl_cloud->size() << std::endl;

    }

    std::cout << "starting opimization" << std::endl;


    int status = estimator.estimateNormalAsNonlinearOpt();

    if (status == 0)
    {
        std::cout << "NOT CONVERGED!" << std::endl;
        return 0;
    }

    std::cout << "ended optimizing" << std::endl;

    spc::StratigraphicNormalModel model = estimator.getNormalModel();


    Eigen::Vector3f n = model.getNormal();
    Eigen::Vector3f c = model.getPosition();

    CCVector3 cn = CCVector3(n(0), n(1), n(2));
    CCVector3 cc = CCVector3(c(0), c(1), c(2));

    ccPlaneOrientation * orientation = new ccPlaneOrientation(cc, cn, 0.05);

    orientation->setVisible(true);


    //we also store the used cloud in a group, child of the orientation.
    ccHObject * folder = new ccHObject;
    folder->setName("Fitted clouds");
    for (auto ent: entities)
        folder->addChild( ccHObjectCaster::ToGenericPointCloud(ent)->clone() );

    orientation->addChild(folder);

    newEntity(orientation);

}

int FitGeologicalOrientation::checkSelected()
{
    ccHObject::Container ents;
    this->getSelectedEntitiesThatAreCCPointCloud(ents);
    if (ents.size() > 0)
        return 1;
    else
        return -1;
}

