#include "CalibrationKeypoint.h"
#include <spc/core/cerea_requested_types.hpp>

namespace spc
{
namespace calibration
{


CalibrationKeyPoint::CalibrationKeyPoint(const Vector3f &pos, const size_t mat_id)
{
    LOG(INFO) << "creating a new calibration keypoint at "<< pos.transpose();
    original_position = pos;
    material_id = mat_id;
    cumulative_set.addNewField("position", 3);
    lambdas.fill(spcNANMacro);

    material_id = std::numeric_limits<size_t>::quiet_NaN();

}

PerCloudCalibrationData::Ptr
CalibrationKeyPoint::newPerCloudData(CloudDataSourceOnDisk::Ptr cloud)
{
    PerCloudCalibrationData::Ptr cdata (new PerCloudCalibrationData(cloud, shared_from_this()));
    per_cloud_data.push_back(cdata);
    return cdata;
}

// it is a root

}
}
