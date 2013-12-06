#ifndef POINT3D_H
#define POINT3D_H

#include <spc/elements/element_base.h>
#include <spc/elements/salvable_object.h>
#include <pcl/common/centroid.h>
#include <spc/elements/generic_cloud.h>
#include <pcl/point_types.h>

namespace spc
{
class spcMovableElement: public spcElementBase
{
public:

    typedef typename boost::shared_ptr<spcMovableElement> Ptr;
    typedef typename boost::shared_ptr<const spcMovableElement> ConstPtr;


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



    void positionFromCentroid(pcl::PointCloud<pcl::PointXYZ> &cloud);


    void positionFromCentroid(spcGenericCloud &cloud);

protected:
    Vector3f position_;

    template <class Archive>
    void serialize(Archive &ar, const unsigned int version)
    {

        ar & boost::serialization::base_object<spcElementBase>(*this);
        /// for now we dont need to call base-classes serialization methods
        ar & BOOST_SERIALIZATION_NVP(position_);
    }

};

}//end nspace

#endif // POINT3D_H