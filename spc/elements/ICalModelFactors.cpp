#include "ICalModelFactors.h"
namespace spc
{

Eigen::VectorXf IntensityCalibrationModelBase::getCorrectedIntensities(
    SamplesDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    spcForEachMacro(Sample::ConstPtr point, in_data->getSamplesDB())
    {
        out(counter++) = point->variantPropertyValue<float>("intensity")
                         / this->getOverallCorrectionFactor(point);
    }

    return out;
}

Eigen::VectorXf IntensityCalibrationModelBase::getPredictedIntensities(
    SamplesDB::ConstPtr in_data)
{
    Eigen::VectorXf out(in_data->size());

    size_t counter = 0;
    spcForEachMacro(Sample::ConstPtr point, in_data->getSamplesDB())
    {
        out(counter++) = this->getOverallCorrectionFactor(point);
    }

    return out;
}

} // end nspace
