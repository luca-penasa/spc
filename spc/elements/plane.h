#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <spc/elements/spcObject.h>
#include <spc/elements/movable_element.h>
#include <spc/elements/normal3d.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <Eigen/Geometry>

//#include <boost/serialization/base_object.hpp>

#include <cereal/cereal.hpp>

#include<cereal/types/polymorphic.hpp>

#include <spc/common/eigen_serialization.hpp>

using namespace Eigen;
namespace spc
{

///
/// \brief The PlaneModel class is the model of a plane in space
/// we represent the plane as a normal plus a point in space
///
class Plane: public PositionableElement

{


    SPC_OBJECT(Plane)
    public:


        /// Def const
        Plane()
    {
    }

    /// copy const
    Plane(const Plane &plane): PositionableElement(plane)
    {
        normal_ = plane.normal_;
    }

    /// a Plane from direction of the normal and passing for a given point
    Plane(const Vector3f normal, const Vector3f point): PositionableElement(point)
    {
        normal_.setNormal(normal);
    }

    void setNormal (Vector3f n)
    {
        normal_.setNormal(n);
    }

    Vector3f getUnitNormal() const
    {
        return normal_.getUnitNormal();
    }

    Vector3f getNormal() const
    {
        return normal_.getNormal();
    }

    Vector3f projectOnPlane(const Vector3f &v) const
    {
        Vector3f n = getUnitNormal();
        return v - v.dot(n) * n;

    }


    /// distance of a point to this plane
    /// distance is signed (+ if in the same half-space of normal_)
    float distanceTo( const Vector3f &point) const
    {
        return normal_.getUnitNormal().dot(point) + getP();
    }

    /// get the P parameters (distance of the plane from the origin)
    /// in normal hessian form: n.dot(x) = -P with n unit normal vector
    float getP() const
    {
        return  - normal_.getUnitNormal().dot(getPosition());
    }

    /// get the matrix that would project a point on a two-d ref system
    /// if you project a point in this ref system you'll get a 3d point
    /// with discardeable z coordinate. and the point wil be mapped in 2D
    /// on the plane. Z coordinate will simply be the distance from the plane
    Transform<float, 3, Affine, AutoAlign> get2DArbitraryRefSystem() const;



protected:

    spcNormal3D normal_;


private:
    friend class cereal::access;

    template <class Archive>
    void serialize( Archive & ar)
    {
        ar( cereal::base_class<spc::PositionableElement>( this ),
            CEREAL_NVP(normal_) );

    }


};

//CEREAL_CLASS_VERSION( Plane, 1)
} //end nspace




#endif // NORMAL_MODEL_H

