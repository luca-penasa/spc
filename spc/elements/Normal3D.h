#ifndef NORMAL3D_H
#define NORMAL3D_H
#include <spc/elements/ElementBase.h>
#include <pcl/features/normal_3d.h>
#include <cereal/types/polymorphic.hpp>

#include <spc/elements/PointCloudBase.h>

namespace spc
{
class Normal3D : public ElementBase
{

    SPC_OBJECT(Normal3D)
EXPOSE_TYPE
public:
    Normal3D();

    Normal3D(float x, float y, float z)
    {
        normal_ = Eigen::Vector3f(x, y, z);
    }

    Normal3D(const Eigen::Vector3f v)
    {
        normal_ = v;
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

    void setNormal(const Normal3D n)
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
        ar(cereal::base_class<ElementBase>(this), CEREAL_NVP(normal_));
    }
};

} // end nspace

#endif // NORMAL3D_H
