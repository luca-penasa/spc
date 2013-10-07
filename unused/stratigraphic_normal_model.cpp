#include <spc/geology/stratigraphic_normal_model.h>
//#include <ccPlane.h>
//#include <ccHObjectCaster.h>

//TMP
//#include <iostream>

#include<pcl/features/normal_3d.h>


namespace spc
{

StratigraphicNormalModel::StratigraphicNormalModel(): GeologicPlane(), stratigraphic_shift_(0.0f)
{
    //nothing for now
}

float StratigraphicNormalModel::getStratigraphicPosition(const Vector3f & point)
{
    //    float dist = this->getDistanceFromOrigin();
    return getPointToPlaneDistance(point) + stratigraphic_shift_ ;
}

Vector3f StratigraphicNormalModel::getStratigraphicNormal(const Vector3f &point)
{
    return getNormal();
}

GeologicPlane::Ptr StratigraphicNormalModel::getGeologicPlaneAtStratigraphicPosition(const float sp, const pcl::PointCloud<pcl::PointXYZ> &cloud /*= 0*/) const
{
    GeologicPlane::Ptr plane (new GeologicPlane); //initialize one
    plane->setNormal(getNormal());

    Vector3f n = getNormal();

    if (cloud.size() != 0)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr proj_cloud (new pcl::PointCloud<pcl::PointXYZ>);

        for (int i = 0 ; i < cloud.size(); ++i)
        {
            pcl::PointXYZ pcl_point = cloud.at(i);

            Vector3f p = pcl_point.getArray3fMap();

            float dist =  getPointToPlaneDistance(p);

            Vector3f np = p - n*(dist + sp - stratigraphic_shift_); //project on plane

            pcl::PointXYZ newpoint = pcl::PointXYZ( np(0), np(1), np(2) );

            proj_cloud->push_back(newpoint);

            //now get the centroid of projected points
            Vector4f centroid;
            pcl::compute3DCentroid(*proj_cloud, centroid);

            plane->setPosition(centroid.head(3));

            std::cout << centroid << std::endl;
            return plane;

        }
    }
    else
    {
        //we know the position of this model is at the "stratigraphic_shift_" position
        Vector3f this_center = (sp - stratigraphic_shift_) * normal_ + position_;

        plane->setPosition(this_center);
        return plane;

    }

}


}//end nspace
