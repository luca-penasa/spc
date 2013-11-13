#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/element_base.h>
#include <pcl/common/centroid.h>
#include <pcl/point_types.h>

namespace spc
{
class Point3D: public ElementBase
{
public:
    Point3D();

    Point3D(const float x, const float y, const float z);

    Point3D ( const Vector3f point);

    /// move this measure to this new position
    /// this imply updating the d parameters of the plane
    void setPosition(const Vector3f position);


    Vector3f getPosition () const
    {
        return position_;
    }

    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);

protected:
    Vector3f position_;

};

}//end nspace

#endif // POINT3D_H
