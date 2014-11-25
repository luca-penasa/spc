#include "PerCloudData.h"
#include <spc/core/cerea_requested_types.hpp>

calibration::PerCloudCalibrationData::PerCloudCalibrationData(CloudDataSourceOnDisk::Ptr ref_cloud, calibration::CalibrationKeyPointPtr parent)
{
    parent_keypoint = parent;
    cloud = ref_cloud;
    sensor_position.fill(spcNANMacro);
}
