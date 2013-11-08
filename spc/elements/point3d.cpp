#include "point3d.h"
namespace spc
{


Point3D::Point3D() : position_(0,0,0)
{
}

Point3D::Point3D(const float x, const float y, const float z)
{
    position_ = Vector3f(x, y, z);
}

Point3D::Point3D(const Vector3f point)
{
    position_ = point;
}


void Point3D::setPosition(const Vector3f position)
{
    //compute the new distance from the origin
    position_ = position;
}

void Point3D::positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(cloud, centroid);

    setPosition( centroid.head(3) );
}


} //end nspace
