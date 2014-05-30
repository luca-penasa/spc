#include <spc/elements/MovableElement.h>

#include <spc/io/eigen_serialization.hpp>
#include <pcl/common/centroid.h>
#include <spc/elements/PointCloudBase.h>

namespace spc
{

MovableElement::MovableElement() : position_(0, 0, 0)
{
}

MovableElement::MovableElement(const float x, const float y,
                                         const float z)
{
    position_ = Eigen::Vector3f(x, y, z);
}

MovableElement::MovableElement(const Eigen::Vector3f point)
{
    position_ = point;
}

void MovableElement::getPosition(float &x, float &y, float &z) const
{
    x = position_(0);
    y = position_(1);
    z = position_(2);
}

Eigen::Vector3f &MovableElement::getPosition()
{
    return position_;
}

Eigen::Vector3f MovableElement::getPosition() const
{
    return position_;
}

void MovableElement::setPosition(const MovableElement el)
{
    setPosition(el.getPosition());
}

void MovableElement::setPosition(const Eigen::Vector3f position)
{
    position_ = position;
}

void MovableElement::positionFromCentroid(pcl::PointCloud
                                               <pcl::PointXYZ> &cloud)
{

    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(cloud, centroid);

    setPosition(centroid.head(3));
}

} // end nspace
