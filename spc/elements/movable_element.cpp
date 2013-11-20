#include "movable_element.h"
namespace spc
{


spcMovableElement::spcMovableElement() : position_(0,0,0)
{
}

spcMovableElement::spcMovableElement(const float x, const float y, const float z)
{
    position_ = Vector3f(x, y, z);
}

spcMovableElement::spcMovableElement(const Vector3f point)
{
    position_ = point;
}


void spcMovableElement::setPosition(const spcMovableElement el)
{    
    setPosition(el.getPosition());
}

void spcMovableElement::setPosition(const Vector3f position)
{
    position_ = position;
}

void spcMovableElement::fromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud)
{

    Eigen::Vector4f centroid;

    pcl::compute3DCentroid(cloud, centroid);

    setPosition( centroid.head(3) );
}


} //end nspace
