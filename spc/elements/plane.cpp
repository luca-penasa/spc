#include <spc/elements/plane.h>
using namespace Eigen;
namespace spc
{

spcPlane::spcPlane()
{
}

spcPlane::spcPlane(const Vector3f normal, const Vector3f point): spcMovableElement(point)
{
    normal_.setNormal(normal);
}

string spcPlane::getSPCClassName()
{
    std::string name = "spcPlane3D";
    return name;
}

int spcPlane::toAsciiMeOnly(stringstream &stream)
{
    normal_.toAsciiMeOnly(stream);
    spcMovableElement::toAsciiMeOnly(stream);
}

void spcPlane::setNormal(Vector3f n)
{
    normal_.setNormal(n);
}

Vector3f spcPlane::getUnitNormal() const
{
    return normal_.getUnitNormal();
}


float spcPlane::distanceTo(const Vector3f &point) const
{
    return normal_.getUnitNormal().dot(point) + getP();
}

float spcPlane::getP() const
{
    return  - normal_.getUnitNormal().dot(getPosition());
}

Vector3f spcPlane::getNormal() const
{
    return normal_.getNormal();
}






}//end nspace
