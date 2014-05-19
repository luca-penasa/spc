#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/element_base.h>
#include <spc/elements/salvable_object.h>
#include <pcl/common/centroid.h>
#include <spc/elements/generic_cloud.h>
#include <pcl/point_types.h>

//#include <spc/io/element_io.h>


#include <spc/common/eigen_serialization.hpp>

namespace spc
{

class PositionableElement: public spcElementBase
{
public:

    typedef boost::shared_ptr<PositionableElement> Ptr;
    typedef boost::shared_ptr<const PositionableElement> ConstPtr;


    PositionableElement();

    PositionableElement(const float x, const float y, const float z);

    PositionableElement ( const Vector3f point);

    Vector3f getPosition() const
    {
        return position_;
    }

    Vector3f &getPosition()
    {
        return position_;
    }

    /// move this measure to this new position
    /// this imply updating the d parameters of the plane
    void setPosition(const PositionableElement el);

    void setPosition(const Vector3f position);

    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);

    template<class Archive>
    void serialize(Archive & archive)
    {
//        archive( position_ );
    }


protected:

    Vector3f position_;



};

}//end nspace



#endif // END SAFEGFUARD
