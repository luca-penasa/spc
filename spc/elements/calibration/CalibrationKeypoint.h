
#ifndef CALIBRATION_KEYPOINT_H
#define CALIBRATION_KEYPOINT_H
#include <spc/elements/ElementBase.h>
#include <spc/elements/calibration/PerCloudData.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint
{
public:

    CalibrationKeyPoint(const Eigen::Vector3i &position)
    {
        original_position = position;
    }

    spcTypedefSharedPtrs(CalibrationKeyPoint)

    Eigen::Vector3f normal;
    Eigen::Vector3f position;
    Eigen::Vector3f original_position;
    float eigen_ratio;
    Eigen::Vector3f lambdas;

    std::vector<PerCloudCalibrationData::Ptr> per_cloud_data;

    PointSet<> cumulative_set_;
};


}//end cal

}// end spc


#endif
