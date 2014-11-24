#ifndef PERCLOUDDATA_H
#define PERCLOUDDATA_H

#include <spc/elements/ElementBase.h>
#include <spc/elements/CloudDataSourceOnDisk.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint;
typedef spcSharedPtrMacro<CalibrationKeyPoint> CalibrationKeyPointPtr;

class PerCloudCalibrationData
{
public:
    spcTypedefSharedPtrs(PerCloudCalibrationData)

    PerCloudCalibrationData(CloudDataSourceOnDisk::Ptr ref_cloud, CalibrationKeyPointPtr parent)
    {
        parent_keypoint = parent;
        cloud = ref_cloud;
    }
    CalibrationKeyPointPtr getParent() const
    {
        return parent_keypoint;
    }

    CloudDataSourceOnDisk::Ptr getCloud() const
    {
        return cloud;
    }

    CloudDataSourceOnDisk::Ptr cloud;

    CalibrationKeyPointPtr parent_keypoint;

    size_t n_neighbors;
    float distance;
    float angle;
    float intensity;
    float intensity_std;

    Eigen::Vector3f sensor_position;


};
}
}




#endif
