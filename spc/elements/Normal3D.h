#ifndef NORMAL3D_H
#define NORMAL3D_H
#include <spc/elements/ElementBase.h>
//#include <pcl/features/normal_3d.h>
#include <cereal/types/polymorphic.hpp>

#include <spc/elements/PointCloudBase.h>

#include <spc/elements/GeometricElement3DBase.h>

namespace spc
{
class Vector3D : public GeometricElement3DBase
{

    SPC_ELEMENT(Vector3D)
    EXPOSE_TYPE
public:
    Vector3D();

    Vector3D(float x, float y, float z)
    {
        normal_ = Eigen::Vector3f(x, y, z);
    }

    Vector3D(const Eigen::Vector3f v)
    {
        normal_ = v;
    }

    Vector3D(const Vector3D & other): GeometricElement3DBase(other)
    {
        normal_ = other.normal_;
    }



    /// this project a given 3d point onto the normal
    Eigen::Vector3f projectPoint(const Eigen::Vector3f &point) const
    {
        return (this->getUnitNormal() * this->getUnitNormal().dot(point));
    }

    void setNormal(const Eigen::Vector3f n)
    {
        normal_ = n;
    }

    void setNormal(const Vector3D n)
    {
        *this = n;
    }

    Eigen::Vector3f getNormal() const
    {
        return normal_;
    }

    void normalize();

    Eigen::Vector3f getUnitNormal() const;

    /// invert the direction of the normal.
    void flipNormal();

    void setUnitAxis(const int ax_id = 2);

    void normalFromBestFit(const PointCloudXYZBase &cloud);

protected:
    Eigen::Vector3f normal_;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(cereal::base_class<GeometricElement3DBase>(this), CEREAL_NVP(normal_));
    }

    // GeometricElement3DBase interface
public:
    virtual void applyTransform(const TransformT &transform) override
    {
        normal_= transform.linear() * normal_; // no scaling or shearing
    }
};

} // end nspace

#endif // NORMAL3D_H
