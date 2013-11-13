#include <spc/elements/attitude.h>
#include <math.h>
#include <Eigen/Geometry>
#include <sstream>
#include <string>

namespace spc
{
Attitude::Attitude()
{
}

Attitude::Attitude( const Vector3f direction, const Vector3f position): Plane::Plane(direction, position)
{
}




Vector3f spc::Attitude::getDipDirectionVector() const
{
    //we simply put t 0 the z component
    Vector3f p = getUnitNormal();
    p(2) = 0.0;
    p.normalize();

    return p;
}

Vector3f Attitude::getStrikeVector() const
{
    Vector3f strike = getUnitNormal().cross(getDipDirectionVector());
    strike.normalize();
    return strike;
}

Vector3f Attitude::getDipVector() const
{
    Vector3f plane_n = getUnitNormal();
    Vector3f dip_dir = getDipDirectionVector();

    //project dip_dir on the plane
    Vector3f dipv = dip_dir - (dip_dir.dot(plane_n) * plane_n);

    dipv.normalize();

    return dipv;
}

float Attitude::getDipAngle() const
{
    float angle = acos(getDipVector().dot(getDipDirectionVector())) * 180.0/M_PI;

    if (angle > 90)
        angle -= 90.0;

    return angle ;
}

float Attitude::getDip() const
{

    Vector2f north = Vector2f::UnitX(); // parallel to x axis
    Vector2f dip = getDipDirectionVector().head(2); // he third element is 0 (working on horizontal plane)

    float angle = acos(north.dot(dip)) * 180/M_PI;

    if (dip(1) > 0.0) //if y component is postive we are in the 180 to 360 half-space
        angle = 360 - angle;

    return angle;

}

std::string Attitude::getDipAndDipAngleAsString() const
{
    char d = 0xb0;
//    std::string degree = d;
    std::stringstream s;
    s << std::fixed << setprecision(2) << getDipAngle() << d << "/" << getDip() << d << "N";
    return s.str();
}




}//end nspace
