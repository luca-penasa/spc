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

Transform<float, 3, Affine, AutoAlign> spcPlane::get2DArbitraryRefSystem() const
{
    Transform<float, 3, Affine, AutoAlign> T;

    Matrix3f new_basis;
    new_basis.setIdentity();

//    T.setIdentity();
    Eigen::Vector3f n = normal_.getUnitNormal();

    float minval = n.array().abs().minCoeff();

    ///seek for a good unit axis to project on plane
    ///and then use it as X direction of the 2d local system
    Vector3f proj_axis;
    int min_id;
    for (int i = 0 ; i < 3; ++i)
    {
        if (minval == n.array().abs()(i))
            min_id = i;
    }

    // we are at the minimum
    if ( min_id == 0 ) // on x
        proj_axis = Vector3f::UnitX();
    else if ( min_id == 1 )
        proj_axis = Vector3f::UnitY();
    else
        proj_axis = Vector3f::UnitZ();


    //project the selected axis on the plane
    Vector3f second_ax = projectOnPlane(proj_axis);
    second_ax.normalize();

    Translation3f translation ( -getPosition());

//    T = T * translation ;
    new_basis.col(0) = n;
    new_basis.col(1) = second_ax;
    new_basis.col(2) = n.cross(second_ax);

    Matrix3f Rot;
      Rot = Quaternionf().setFromTwoVectors(n,second_ax);
//    T.matrix().col(3).head(3) = -getPosition();



//    AngleAxis<float> R = new_basis;
    T = Rot*translation;

    std::cout << "origin:" <<  getPosition() << std::endl;
    //generate a suitable ref frame

    return T;
}

Vector3f spcPlane::getNormal() const
{
    return normal_.getNormal();
}

Vector3f spcPlane::projectOnPlane(const Vector3f &v) const
{
    Vector3f n = getUnitNormal();
    return v - v.dot(n) * n;

}






}//end nspace
