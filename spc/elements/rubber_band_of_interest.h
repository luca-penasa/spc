#ifndef RUBBER_BAND_OF_INTEREST_H
#define RUBBER_BAND_OF_INTEREST_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
using namespace Eigen;

namespace spc
{
class RubberBandOfInterest
{
    typedef typename pcl::PointXYZ PointT;
    typedef typename pcl::PointCloud<PointT> CloudT;
    typedef typename CloudT::Ptr CloudPtrT;

public:
    RubberBandOfInterest();

    /// set the (ordered or not) set of points defining the rubber band area selection
    void setInputCloud(CloudPtrT in_cloud)
    {

    }


private:
    ///in cloud defines the input set of points in 3d space defining the rubber band!
    CloudPtrT in_cloud_;

    /// the direction in whih the rubberband selection is "extruded" to form a volume that is intersected with a cloud
    Vector3f direction_;

    /// is the length of extrusion of the rubberband volume
    float length_;


};

}//end nspace

#endif // RUBBER_BAND_OF_INTEREST_H
