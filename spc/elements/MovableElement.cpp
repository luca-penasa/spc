#include <spc/elements/MovableElement.h>

#include <spc/io/eigen_serialization.hpp>
#include <pcl/common/centroid.h>
#include <spc/elements/PointCloudBase.h>


namespace spc
{


PositionableElement::PositionableElement() : position_(0,0,0)
{
}

PositionableElement::PositionableElement(const float x, const float y, const float z)
{
    position_ = Eigen::Vector3f(x, y, z);
}

PositionableElement::PositionableElement(const Eigen::Vector3f point)
{
    position_ = point;
}

void PositionableElement::getPosition(float &x, float &y, float &z) const
{
    x = position_(0);
    y = position_(1);
    z = position_(2);
}



Eigen::Vector3f &PositionableElement::getPosition()
{
    return position_;
}

Eigen::Vector3f PositionableElement::getPosition() const
{
    return position_;
}


void PositionableElement::setPosition(const PositionableElement el)
{    
    setPosition(el.getPosition());
}

void PositionableElement::setPosition(const Eigen::Vector3f position)
{
    position_ = position;
}

void PositionableElement::positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(cloud, centroid);

    setPosition( centroid.head(3) );
}




} //end nspace



