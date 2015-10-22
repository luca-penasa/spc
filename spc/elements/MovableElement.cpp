#include <spc/elements/MovableElement.h>

#include <spc/io/eigen_serialization.hpp>
//#include <pcl/common/centroid.h>
#include <spc/elements/PointCloudBase.h>

namespace spc
{


DtiClassType Point3D::Type ("Point3D", &ElementBase::Type);

Point3D::Point3D()
{
    position_.fill(spcNANMacro);
    position_(3) = spcNANMacro;
}

Point3D::Point3D(const float x,
                 const float y,
                 const float z)
{
    position_ << x,y,z,1;
}

Point3D::Point3D(const Eigen::Vector3f point)
{
    position_.head(3) = point;
    position_(3) = 1;
}

void Point3D::getPosition(float &x, float &y, float &z) const
{
    x = position_(0);
    y = position_(1);
    z = position_(2);
}

void Point3D::setPositionH(const Vector4f &position)
{
    position_ = position;
    position_(3) = 1; // be sure the last element is one
}


void Point3D::setPosition(const Point3D &el)
{
    setPosition(el.getPosition());
}

void Point3D::setPosition(const Eigen::Vector3f & position)
{
    position_.head(3) = position;
}

//void Point3D::positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud)
//{
//    Eigen::Vector4f centroid;
//    pcl::compute3DCentroid(cloud, centroid);
//    setPosition(centroid.head(3));
//}

void Point3D::positionFromCentroid(const PointCloudXYZBase &cloud)
{
    DLOG(INFO) << "setting position from cloud centroid";
    Eigen::Vector3f pos = cloud.getCentroid();
    this->setPosition(pos);
    DLOG(INFO) << "now position is " << pos.transpose();
}


} // end nspace



#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Point3D);
