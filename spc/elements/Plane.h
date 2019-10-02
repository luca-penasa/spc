#pragma once
#ifndef SPC_PLANE_MODEL_H
#define SPC_PLANE_MODEL_H

#include <spc/elements/MovableElement.h>
#include <spc/elements/Normal3D.h>
#include <spc/core/spc_eigen.h>

using namespace Eigen;
namespace spc {

///
/// \brief The PlaneModel class is the model of a plane in space
/// we represent the plane as a normal plus a point in space
///
class Plane : public Point3D

              {
public:
    SPC_ELEMENT(Plane)
    EXPOSE_TYPE
    /// Def const
    Plane()
    {
    }

    /// copy const
    Plane(const Plane& plane)
        : Point3D(plane)
    {
        normal_ = plane.normal_;
    }

    /// a Plane from direction of the normal and passing for a given point
    Plane(const Vector3f normal, const Vector3f point)
        : Point3D(point)
    {
        normal_.setNormal(normal);
    }

    static Plane fromEigenHyperplane3f(const Eigen::Hyperplane<float, 3>& hyperplane3)
    {
        Plane out;
        out.setNormal(hyperplane3.normal());
        out.normal_.normalize();
        // this is a point which is obvisouly on the plane
        // but we may use any other point.
        Eigen::Vector3f point_on_plane = -hyperplane3.offset() * hyperplane3.normal();
        out.setPosition(point_on_plane);
        return out;
    }

    void setNormal(Vector3f n)
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

    //! get the component of v on the plane.
    Vector3f projectVectorOnPlane(const Vector3f& v) const
    {
        Vector3f n = getUnitNormal();
        return v - v.dot(n) * n;
    }

    //! project a point onto the plane
    Vector3f projectPointOnPlane(const Vector3f& point)
    {
        Vector3f n = getUnitNormal();
        return point - (point - getPosition()).dot(n) * n;
    }

    /// distance of a point to this plane
    /// distance is signed (+ if in the same half-space of normal_)
    float distanceTo(const Vector3f& point) const
    {
        DLOG(INFO) << "position is " << getPosition().transpose();

        DLOG(INFO) << "computing distance to point " << point.transpose();
        float value = getUnitNormal().dot(point) + getP();
        DLOG(INFO) << "value is  "<< value << " P: " << getP();

        return value;
    }

    /// get the P parameters (distance of the plane from the origin)
    /// in normal hessian form: n.dot(x) = -P with n unit normal vector
    float getP() const
    {
        return -getUnitNormal().dot(getPosition());
    }

    /// get the matrix that would project a point on a two-d ref system
    /// if you project a point in this ref system you'll get a 3d point
    /// with discardeable z coordinate (in respect to the plane's space).
    /// and the point wil be mapped in 2D
    /// on the plane. Z coordinate will simply be the distance from the plane
    Transform<float, 3, Affine, AutoAlign> get2DArbitraryRefSystem() const;

protected:
    Vector3D normal_;

private:
    friend class cereal::access;

    template <class Archive>
    void serialize(Archive& ar, const std::uint32_t version)
    {
        ar(cereal::base_class<spc::Point3D>(this), CEREAL_NVP(normal_));
    }

    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override;
};

// CEREAL_CLASS_VERSION( Plane, 1)
} // end nspace

#endif // NORMAL_MODEL_H
