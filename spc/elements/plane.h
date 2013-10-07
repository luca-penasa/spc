#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <Eigen/Core>
#include <boost/smart_ptr.hpp>

using namespace Eigen;


namespace spc
{

///
/// \brief The PlaneModel class is the model of a plane in space
/// it is in NORMAL HESSIAN FORM
///
class Plane
{
public:

    typedef boost::shared_ptr<Plane> Ptr;

    /// Def const
    Plane();

    /// Parameters may or may not be in normal hessian form. They will be always reduced to it!
    Plane(const Vector4f parameters);

    /// Plane with unit normal in the direction of n, with a distance from origin of "d"
    Plane(const Vector3f n, const float d);


    /// You can also pass a non-unit vector -> it will be reduced to the unit via normalizeNormal()
    void setUnitNormal(const Vector3f normal)
    {
        normal_ = normal;
        normalizeNormal(); // be sure we are passing a unit normal
    }

    /// set the distance from the origin for this plane, following hessian normal form of the plane
    void setD(const float d)
    {
        distance_ = d;
    }

    /// set all the parameters all at once from a generic plane:
    /// ax+by+cz+d = 0 (generic plane not in normal hessian form)
    /// you can also pass parameters for a plane in normal hessian form, no problem
    void setPlaneParameters (const Vector4f pars)
    {
        float norm = pars.head(3).norm();        
        normal_ = pars.head(3) / norm;
        distance_ = pars(4) / norm;
    }

    /// project a point on this plane
    Vector3f projectOnPlane(const Vector3f &point) const
    {
        return point - normal_ * distanceTo(point);
    }

    /// distance of from a point to this plane
    float distanceTo( const Vector3f &point) const
    {
        return point.dot(normal_) - distance_;
    }


    /// get the d parameter of for the plane
    float getD() const
    {
        return distance_;
    }

    /// get the unit normal vector for this plane
    Vector3f getUnitNormal( ) const
    {
        return normal_;
    }




protected:

    void normalizeNormal()
    {
        normal_ = normal_ / normal_.norm();
    }

    Vector3f normal_;

    /// is the distance of the plane from the origin
    float distance_;

};

} //end nspace

#endif // NORMAL_MODEL_H
