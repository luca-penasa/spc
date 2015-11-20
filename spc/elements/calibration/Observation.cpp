#include "Observation.h"
#include <spc/core/cerea_requested_types.hpp>

calibration::Observation::Observation(CloudDataSourceOnDisk::Ptr ref_cloud, calibration::CalibrationKeyPointPtr parent)
{
    parent_keypoint = parent;
    cloud = ref_cloud;
//    sensor_position.fill(spcNANMacro);
}
