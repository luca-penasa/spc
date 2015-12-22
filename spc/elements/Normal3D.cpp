#include "Normal3D.h"
//#include <spc/core/eigen_extensions.h>
#include <spc/elements/PointCloudBase.h>
//#include <spc/elements/ElementBase.h>
//#include <pcl/features/normal_3d.h>
#include <cereal/types/polymorphic.hpp>
#include <spc/elements/PointCloudBase.h>


namespace spc
{

DtiClassType Vector3D::Type ("Vector3D", &ElementBase::Type);


Vector3D::Vector3D()
{
    normal_.fill(spcNANMacro);
}

void Vector3D::normalize()
{
    normal_.normalize();
}

Eigen::Vector3f Vector3D::getUnitNormal() const
{
    Eigen::Vector3f unit = normal_;
    unit.normalize();
    return unit;
}

void Vector3D::flipNormal()
{
    normal_ = -normal_;
}

void Vector3D::setUnitAxis(const int ax_id)
{
    normal_.setZero();
    normal_(ax_id) = 1.0;
}

void Vector3D::normalFromBestFit(const PointCloudXYZBase &cloud)
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
SPC_CEREAL_REGISTER_TYPE(spc::Vector3D);
