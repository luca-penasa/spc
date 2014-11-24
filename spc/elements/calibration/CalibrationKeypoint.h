
#ifndef CALIBRATION_KEYPOINT_H
#define CALIBRATION_KEYPOINT_H
#include <spc/elements/ElementBase.h>
#include <spc/elements/calibration/PerCloudData.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint: public ElementBase
{
public:
    spcTypedefSharedPtrs(CalibrationKeyPoint)


    CalibrationKeyPoint(const Eigen::Vector3f &position)
    {
        original_position = position;
    }


    PerCloudCalibrationData::Ptr newPerCloudData(CloudDataSourceOnDisk::Ptr cloud)
    {
        PerCloudCalibrationData::Ptr cdata (new PerCloudCalibrationData(cloud,
                                                                        std::static_pointer_cast<CalibrationKeyPoint>(this->getPtr())));
        per_cloud_data.push_back(cdata);
        return cdata;
    }


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
