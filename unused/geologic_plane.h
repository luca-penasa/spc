#ifndef STRATIGRAPHIC_NORMAL_H
#define STRATIGRAPHIC_NORMAL_H

//#include <Eigen/Dense>
#include <Eigen/Core>
#include "geologic_element_base.h"

//#include <iostream> //for debug

#include <boost/shared_ptr.hpp>
namespace spc
{
using namespace Eigen;

///
/// \brief The GeologicPlane class represent a plane in space
/// Defined by a normal and a point in space for which the plane passes
/// (classicla point and normal representation)
/// it is not the more eficient but it makes sense in a geologic context.
/// (measure of orientation + location of this measure)
///
class GeologicPlane: public GeologicElementBase
{
public:
    typedef typename boost::shared_ptr<GeologicPlane> Ptr;

    //! constructor
    GeologicPlane(Vector3f normal = Vector3f(0,0,1), Vector3f center = Vector3f(0,0,0));

    //! set the stratigraphic normal
    void setNormal(float x, float y, float z) {normal_(0) = x; normal_(1)=y; normal_(2)=z; normalize();}

    //! set the normal from a vector
    void setNormal(Vector3f n) {normal_ = n; this->normalize();}

    //! set a spatial position for this measure
    void setPosition(float x, float y, float z) {position_(0) = x; position_(1)=y; position_(2)=z;}

    //! set a spatial position for this measure via a single eigen vector
    void setPosition(Vector3f v) { position_ = v; }

    //////////////////////////////////// GETTERS

    //! the distance of this measure from the origin (0.0, 0.0, 0.0) along the normal
    /**
     * Is like to get the "d" parameter in ax + by + cz + d = 0 (in normal hessian form)
     */
    float getDistanceFromOrigin() const ;

    //! get back the normal as vector
    Vector3f getNormal() const {return normal_;}

    //! get the position of this measure
    Vector3f getPosition () const {return position_;}

    //! distance
    float getPointToPlaneDistance(const Vector3f &point) const
    {
        return point.dot(normal_) - getDistanceFromOrigin();
    }

    Vector3f projectOnPlane(const Vector3f &point) const
    {
        return point - getNormal() * getPointToPlaneDistance(point);
    }

    //// OPERATORS AND USEFUL METHODS
    //! normalize the normal of the model
    void normalize() {normal_ = normal_/normal_.norm();}

    //! Operator plus +
    /** it sums the three components of the normal vector. NO normalization
      * the position in space is also summed between the two
      * this is used for making averages!
      * TODO remove this can lead to misunderstandings
     **/
    GeologicPlane operator+(const GeologicPlane other);

    //! a plane starting from the commonly used plane parameters
    /**
      * ax + by + cz + d = 0
      **/
    static GeologicPlane::Ptr fromStandardEq(const Vector4f pars);


protected:
    //! normal of the plane
    Vector3f normal_;

    //! a point part of the plane
    /** maybe it is not the best way of representing a plane, but it is the more
     * convenient for geology (position + orientation)
    **/
    Vector3f position_;  

};

}//end nspace

#endif // STRATIGRAPHIC_NORMAL_H
