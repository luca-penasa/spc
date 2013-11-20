#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <spc/elements/element_base.h>
#include <spc/elements/movable_element.h>
#include <spc/elements/normal3d.h>

namespace spc
{

///
/// \brief The PlaneModel class is the model of a plane in space
/// we represent the plane as a normal plus a point in space
///
class spcPlane: public spcMovableElement

{
public:


    /// Def const
    spcPlane();

    /// a Plane from direction of the normal and passing for a given point
    spcPlane(const Vector3f normal_, const Vector3f point);

    spcPlane(const spcPlane &plane): spcMovableElement(plane)
    {
        normal_.setNormal(plane.getNormal());
    }


    virtual std::string getSPCClassName();


    virtual int toAsciiMeOnly(std::stringstream &stream);

    void setNormal (Vector3f n);

    Vector3f getUnitNormal() const;
    Vector3f getNormal() const;


    /// distance of a point to this plane
    /// distance is signed (+ if in the same half-space of normal_)
    float distanceTo( const Vector3f &point) const;

    /// get the P parameters (distance of the plane from the origin)
    /// in normal hessian form: n.dot(x) = -P with n unit normal vector
    float getP() const;

protected:
    Normal3D normal_;


};

} //end nspace

#endif // NORMAL_MODEL_H
