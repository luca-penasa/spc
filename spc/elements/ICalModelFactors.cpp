#include "ICalModelFactors.h"
namespace spc
{

Eigen::VectorXf IntensityCalibrationModelBase::getCorrectedIntensities(
    DataDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    spcForEachMacro(CorePoint::ConstPtr point, in_data->getDataDB())
    {
        out(counter++) = point->value<float>("intensity")
                         / this->getOverallCorrectionFactor(point);
    }

    return out;
}

Eigen::VectorXf IntensityCalibrationModelBase::getPredictedIntensities(
    DataDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    spcForEachMacro(CorePoint::ConstPtr point, in_data->getDataDB())
    {
        out(counter++) = this->getOverallCorrectionFactor(point);
    }

    return out;
}

} // end nspace
