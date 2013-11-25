#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/element_base.h>
#include <spc/elements/salvable_object.h>
#include <pcl/common/centroid.h>
#include <spc/elements/generic_cloud.h>
#include <pcl/point_types.h>

namespace spc
{
class spcMovableElement: public spcElementBase, public SalvableObject
{
public:
    spcMovableElement();

    spcMovableElement(const float x, const float y, const float z);

    spcMovableElement ( const Vector3f point);

    Vector3f getPosition() const
    {
        return position_;
    }

    /// move this measure to this new position
    /// this imply updating the d parameters of the plane
    void setPosition(const spcMovableElement el);

    void setPosition(const Vector3f position);

    virtual std::string getSPCClassName()
    {
        std::string name = "spcMovableElement";
        return name;
    }


    virtual int toAsciiMeOnly(std::stringstream &stream)
    {
        stream << position_(0) << std::endl;
        stream << position_(1) << std::endl;
        stream << position_(2) << std::endl;
    }


    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);


    void positionFromCentroid(spcGenericCloud &cloud);

protected:
    Vector3f position_;

};

}//end nspace

#endif // POINT3D_H
