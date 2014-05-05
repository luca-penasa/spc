#include "IntensityCalibratrionModels.h"
namespace spc
{


Eigen::VectorXf IntensityCalibrationModelBase::getCorrectedIntensities(CalibrationDataDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    BOOST_FOREACH(CorePointData::ConstPtr point, in_data->getDataDB())
    {
        out(counter++) = point->value<float>("intensity") * this->getOverallCorrectionFactor(point);
    }

    return out;
}

Eigen::VectorXf IntensityCalibrationModelBase::getPredictedIntensities(CalibrationDataDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    BOOST_FOREACH(CorePointData::ConstPtr point, in_data->getDataDB())
    {
        out(counter++) = 1.0f / this->getOverallCorrectionFactor(point);
    }

    return out;
}




}//end nspace
