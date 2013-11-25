#ifndef CLOUDMAPPER_H
#define CLOUDMAPPER_H
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <sensor_msgs/PointCloud2.h>
#include <ccPointCloud.h>

#include <spc/elements/generic_cloud.h>


template <typename inCloudT>
class CloudWrapper: public spc::spcGenericCloud
{
public:
    CloudWrapper(inCloudT *cloud)
    {
        in_cloud  = cloud;
    }

    virtual void getPoint (const int id, float &x, float &y, float &z);

    virtual void getFieldValue(const int id, const std::string fieldname,  float &val);

    virtual int getSize() const;

    inCloudT * in_cloud;
};




#endif // CLOUDMAPPER_H
