#include "CalibrationKeypoint.h"
#include <spc/core/cerea_requested_types.hpp>

namespace spc
{
namespace calibration
{


CalibrationKeyPoint::CalibrationKeyPoint(const Vector3f &pos)
{
    LOG(INFO) << "creating a new calibration keypoint at "<< pos.transpose();
    original_position = pos;

    cumulative_set.addNewField("position", 3);
    lambdas.fill(spcNANMacro);

}

PerCloudCalibrationData::Ptr CalibrationKeyPoint::newPerCloudData(CloudDataSourceOnDisk::Ptr cloud)
{
    PerCloudCalibrationData::Ptr cdata (new PerCloudCalibrationData(cloud, shared_from_this()));
    per_cloud_data.push_back(cdata);
    return cdata;
}

// it is a root

}
}
