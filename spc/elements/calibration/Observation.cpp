#include "Observation.h"
#include <spc/core/cerea_requested_types.hpp>

#include <spc/core/ElementBase.h>
#include <spc/elements/CloudDataSourceOnDisk.h>


#include <spc/elements/calibration/KeyPoint.h>
namespace spc
{




calibration::Observation::Observation(CloudDataSourceOnDisk::Ptr ref_cloud, calibration::KeyPoint::Ptr parent)
{
    parent_keypoint = parent;
    cloud = ref_cloud;
//    sensor_position.fill(spcNANMacro);
}




}
