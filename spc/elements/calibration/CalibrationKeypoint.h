
#ifndef CALIBRATION_KEYPOINT_H
#define CALIBRATION_KEYPOINT_H
#include <spc/elements/ElementBase.h>
#include <spc/elements/calibration/PerCloudData.h>
#include <spc/elements/Plane.h>
namespace spc
{
namespace calibration
{

class CalibrationKeyPoint: public std::enable_shared_from_this<CalibrationKeyPoint>
{
public:
    spcTypedefSharedPtrs(CalibrationKeyPoint)

    //! just for cereal to not complain
    CalibrationKeyPoint()
    {
        lambdas.fill(spcNANMacro);
    }

    CalibrationKeyPoint(const Eigen::Vector3f &pos);


    PerCloudCalibrationData::Ptr newPerCloudData(CloudDataSourceOnDisk::Ptr cloud);

    Plane fitting_plane;

    Eigen::Vector3f original_position;
    Eigen::Vector3f post_position;
    float eigen_ratio;
    Eigen::Vector3f lambdas;

    std::vector<PerCloudCalibrationData::Ptr> per_cloud_data;

    NewSpcPointCloud cumulative_set;

private:
    friend class cereal::access;

    template <class Archive> void serialize(Archive &ar)
    {
        ar(
           CEREAL_NVP(fitting_plane),
           CEREAL_NVP(original_position),
           CEREAL_NVP(post_position),
           CEREAL_NVP(eigen_ratio),
           CEREAL_NVP(lambdas),
           CEREAL_NVP(per_cloud_data),
           CEREAL_NVP(cumulative_set));
    }
};


}//end cal

}// end spc


#endif
