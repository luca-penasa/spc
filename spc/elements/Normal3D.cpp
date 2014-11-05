#include "Normal3D.h"
#include <spc/core/eigen_extensions.h>
#include <spc/elements/PointCloudBase.h>
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

void Normal3D::normalFromBestFit(const PointCloudXYZBase &cloud)
{
    DLOG(INFO) << "setting normal from best fit of plane.";

    // this will do a copy. I am sorry :-(
//    Eigen::Matrix<float, -1, 3> asmat= cloud.getMatrixXfMap(3,4,0).transpose();

    Eigen::Hyperplane<float, 3> plane = cloud.fitHyperplane();

    setNormal(plane.normal());

    DLOG(INFO) << "normal is now: " << plane.normal().transpose();
}

} // end nspace

#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Normal3D);
