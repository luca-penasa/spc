#ifndef NORMAL3D_H
#define NORMAL3D_H
#include <spc/elements/element_base.h>
#include <pcl/features/normal_3d.h>


namespace spc
{


class Normal3D: public ElementBase
{
public:
    Normal3D();

    void setNormal(const Vector3f n)
    {
        normal_ = n;
    }

    Vector3f getNormal() const
    {
        return normal_;
    }

    void normalize()
    {
        normal_.normalize();
    }

    Vector3f getUnitNormal() const
    {
        Vector3f unit = normal_;
        unit.normalize();
        return unit;
    }

    /// invert the direction of the normal.
    void flipNormal()
    {
        normal_ = - normal_;
    }

    void normalFromBestFit(pcl::PointCloud<pcl::PointXYZ> & cloud);

protected:
    Vector3f normal_;
};


} //end nspace

#endif // NORMAL3D_H
