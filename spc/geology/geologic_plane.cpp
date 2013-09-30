#include "geologic_plane.h"

namespace spc
{

GeologicPlane::GeologicPlane()
{
    normal_.setZero();
    normal_(2) = 1.0;
    position_.setZero();
}

GeologicPlane::Ptr GeologicPlane::fromStandardEq(const Vector4f pars)
{
    float d = pars(4);
    Vector3f n = pars.head(3); //first 3 pars.

    //normalize the norm and the distance from origin
    float norm= n.norm();
    d /= norm;
    n /= norm;

    GeologicPlane::Ptr normal (new GeologicPlane);

    normal->setNormal(n);

    Vector3f P = n * d;
    normal->setPosition(P);

    return normal;
}

float GeologicPlane::getDistanceFromOrigin() const
{
//    this->normalize(); //be sure it is normalized
    Eigen::Vector3f c = this->getPosition();
//        std::cout << c << " n: " << this->getNormal() << std::endl;
    return c.dot(this->getNormal()); //project the center on the normal and get the distance
}


GeologicPlane GeologicPlane::operator+(const GeologicPlane other)
{
    GeologicPlane result;
    result.setNormal((this->getNormal() + other.getNormal()));
    result.setPosition((this->getPosition() + other.getPosition()) );
    result.normalize();
    return result;
}

}//end nspace
