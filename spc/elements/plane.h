#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <spc/elements/element_base.h>
#include <spc/elements/movable_element.h>
#include <spc/elements/normal3d.h>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <Eigen/Geometry>

#include <boost/serialization/base_object.hpp>

using namespace Eigen;
namespace spc
{

///
/// \brief The PlaneModel class is the model of a plane in space
/// we represent the plane as a normal plus a point in space
///
class spcPlane: public spcMovableElement

{
public:


    typedef typename boost::shared_ptr<spcPlane> Ptr;
    typedef typename boost::shared_ptr<const spcPlane> ConstPtr;



    /// Def const
    spcPlane();

    /// copy const
    spcPlane(const spcPlane &plane): spcMovableElement(plane)
    {
        normal_ = plane.normal_;
    }

    /// a Plane from direction of the normal and passing for a given point
    spcPlane(const Vector3f normal_, const Vector3f point);



    void setNormal (Vector3f n);

    Vector3f getUnitNormal() const;

    Vector3f getNormal() const;

    Vector3f projectOnPlane(const Vector3f &v) const;


    /// distance of a point to this plane
    /// distance is signed (+ if in the same half-space of normal_)
    float distanceTo( const Vector3f &point) const;

    /// get the P parameters (distance of the plane from the origin)
    /// in normal hessian form: n.dot(x) = -P with n unit normal vector
    float getP() const;


    /// get the matrix that would project a point on a two-d ref system
    /// if you project a point in this ref system you'll get a 3d point
    /// with discardeable z coordinate. and the point wil be mapped in 2D
    /// on the plane. Z coordinate will simply be the distance from the plane
    Transform<float, 3, Affine, AutoAlign> get2DArbitraryRefSystem() const;



protected:

    spcNormal3D normal_;

    friend class boost::serialization::access;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {
//        ar & boost::serialization::make_nvp("position",boost::serialization::base_object<spcMovableElement >(*this));
//        ar & boost::serialization::make_nvp("normal", normal_);


        ar & BOOST_SERIALIZATION_BASE_OBJECT_NVP(spcMovableElement);
        ar & BOOST_SERIALIZATION_NVP(normal_);
    }



};

} //end nspace

#endif // NORMAL_MODEL_H

