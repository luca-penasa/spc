#include <spc/elements/Attitude.h>
#include <math.h>
//#include <Eigen/Geometry>
#include <sstream>
#include <string>

#include <iomanip>
namespace spc
{

DtiClassType Attitude::Type ("Attitude", &Plane::Type);

Attitude::Attitude()
{

}

Attitude::Attitude(const Vector3f &direction, const Vector3f &position)
    : Plane(direction, position)
{
   if (getNormal()(2) < 0)
       normal_.flipNormal();
}

Attitude::Attitude(const float dipAngle, const float dip, const Vector3f position)
{

    if ((dipAngle <= 0.0) | (dipAngle >= 90))
        LOG(WARNING) << "dipAngle out of bounds. Are you sure" << std::endl;

    Vector3f n = Vector3f::UnitZ(); // is a vertical vector
    // we rotate a vertical vector on the two axis, of the given angles
    AngleAxis<float> rot_x(dipAngle / 180 * M_PI, -Vector3f::UnitX());
    AngleAxis<float> rot_z(dip / 180 * M_PI, -Vector3f::UnitZ());

    Vector3f out_n = rot_z * rot_x * n;

    this->setNormal(out_n.normalized());
    this->setPosition(position);
}

Vector3f spc::Attitude::getDipDirectionVector() const
{
    // we simply put t 0 the z component
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

    // project dip_dir on the plane
    Vector3f dipv = dip_dir - (dip_dir.dot(plane_n) * plane_n);

    dipv.normalize();

    return dipv;
}

float Attitude::getDipAngle() const
{
    float angle = acos(getDipVector().dot(getDipDirectionVector())) * 180.0
                  / M_PI;

    if (angle > 90)
        angle -= 90.0;

    return angle;
}

float Attitude::getDip() const
{

    Vector2f north = Vector2f::UnitX(); // parallel to x axis
    Vector2f dip = getDipDirectionVector().head(
        2); // he third element is 0 (working on horizontal plane)

    float angle = acos(north.dot(dip)) * 180 / M_PI;

    if (dip(1)
        > 0.0) // if y component is postive we are in the 180 to 360 half-space
        angle = 360 - angle;

    return angle;
}

std::string Attitude::getDipAndDipAngleAsString() const
{
//    char d = 0xb0;
    //    std::string degree = d;
    std::stringstream s;
	s << std::fixed << std::setprecision(2) << getDipAngle() <<"°" << "/"
	  << getDip() << "°" << "N";
    return s.str();
}



} // end nspace


#include <spc/core/spc_cereal.hpp>
SPC_CEREAL_REGISTER_TYPE(spc::Attitude)
