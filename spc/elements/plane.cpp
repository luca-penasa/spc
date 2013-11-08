#include <spc/elements/plane.h>
using namespace Eigen;
namespace spc
{

Plane::Plane()
{
}



Plane::Plane(const Vector3f normal, const Vector3f point)
{
    setNormal(normal);
    setPosition(point);
}


float Plane::distanceTo(const Vector3f &point) const
{
    return getUnitNormal().dot(point) + getP();
}

float Plane::getP() const
{
    return  - getUnitNormal().dot(position_);
}






}//end nspace
