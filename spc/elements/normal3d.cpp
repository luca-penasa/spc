#include "normal3d.h"

namespace spc
{


spcNormal3D::spcNormal3D() : normal_(0,0,1)
{
}

void spcNormal3D::normalize()
{
    normal_.normalize();
}

Eigen::Vector3f spcNormal3D::getUnitNormal() const
{
    Eigen::Vector3f unit = normal_;
    unit.normalize();
    return unit;
}

void spcNormal3D::flipNormal()
{
    normal_ = - normal_;
}

void spcNormal3D::setUnitAxis(const int ax_id)
{
    normal_.setZero();
    normal_(ax_id) = 1.0;
}

void spcNormal3D::normalFromBestFit(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    float curv;
    Eigen::Vector4f params;

    //do the fit
    pcl::computePointNormal(cloud, params, curv);

    Eigen::Vector3f n = params.head(3);
    n.normalize();
    setNormal(n);
}



}//end nspace


