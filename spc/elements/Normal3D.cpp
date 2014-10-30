#include "Normal3D.h"
namespace spc
{

DtiClassType Normal3D::Type ("Normal3D", &ElementBase::Type);


Normal3D::Normal3D() : normal_(0, 0, 1)
{
}

void Normal3D::normalize()
{
    normal_.normalize();
}

Eigen::Vector3f Normal3D::getUnitNormal() const
{
    Eigen::Vector3f unit = normal_;
    unit.normalize();
    return unit;
}

void Normal3D::flipNormal()
{
    normal_ = -normal_;
}

void Normal3D::setUnitAxis(const int ax_id)
{
    normal_.setZero();
    normal_(ax_id) = 1.0;
}

void Normal3D::normalFromBestFit(pcl::PointCloud<pcl::PointXYZ> &cloud)
{
    float curv;
    Eigen::Vector4f params;

    // do the fit
    pcl::computePointNormal(cloud, params, curv);

    Eigen::Vector3f n = params.head(3);
    n.normalize();
    setNormal(n);
}

} // end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Normal3D);
