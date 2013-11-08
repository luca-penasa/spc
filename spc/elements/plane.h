#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <spc/elements/element_base.h>
#include <spc/elements/point3d.h>
#include <spc/elements/normal3d.h>

namespace spc
{

///
/// \brief The PlaneModel class is the model of a plane in space
/// we represent the plane as a normal plus a point in space
///
class Plane: public Point3D, public Normal3D

{
public:

    typedef boost::shared_ptr<Plane> Ptr;

    /// Def const
    Plane();

    /// a Plane from direction of the normal and passing for a given point
    Plane(const Vector3f normal, const Vector3f point);

    /// distance of a point to this plane
    /// distance is signed (+ if in the same half-space of normal_)
    float distanceTo( const Vector3f &point) const;

    /// get the P parameters (distance of the plane from the origin)
    /// in normal hessian form: n.dot(x) = -P with n unit normal vector
    float getP() const;


};

} //end nspace

#endif // NORMAL_MODEL_H
