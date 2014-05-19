#include "movable_element.h"





namespace spc
{


PositionableElement::PositionableElement() : position_(0,0,0)
{
}

PositionableElement::PositionableElement(const float x, const float y, const float z)
{
    position_ = Vector3f(x, y, z);
}

PositionableElement::PositionableElement(const Vector3f point)
{
    position_ = point;
}


void PositionableElement::setPosition(const PositionableElement el)
{    
    setPosition(el.getPosition());
}

void PositionableElement::setPosition(const Vector3f position)
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


