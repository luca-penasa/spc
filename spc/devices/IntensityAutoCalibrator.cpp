#include "IntensityAutoCalibrator.h"

namespace spc
{

IntensityAutoCalibrator::IntensityAutoCalibrator(): search_radius_(0.1)
{
}

void spc::IntensityAutoCalibrator::setInputClouds(std::vector<std::string> cloud_names)
{
    input_fnames_ = cloud_names;
}

void spc::IntensityAutoCalibrator::setInputCorePoints(std::string core_points_name)
{
    input_core_points_fname_ = core_points_name;
}

void spc::IntensityAutoCalibrator::setSearchRadius(const float rad)
{
    search_radius_ = rad;
}

spc::CalibrationDataDB spc::IntensityAutoCalibrator::getCalibrationDB()
{
    return db_;
}

}
