#include "AlignSingleAttitudeStratigraphicModelToSamples.h"
#include <spc/elements/StratigraphicModelSingleAttitude.h>
#include <spc/elements/Sample.h>
#include <spc/elements/SamplesDB.h>

#include <numeric> // for std::accumulate on msvc12

namespace spc
{
AlignSingleAttitudeStratigraphicModelToSamples::
    AlignSingleAttitudeStratigraphicModelToSamples()
{
}

void AlignSingleAttitudeStratigraphicModelToSamples::updateResiduals()
{

    real_sp_.clear();
    model_sp_.clear();
    residuals_.clear();

    for(Sample::Ptr sample: db_->getSamplesDB())
    {
        if (sample->hasProperty(strat_pos_field_)) {
            real_sp_.push_back(sample->variantPropertyValue<float>(strat_pos_field_));
            model_sp_.push_back(
                model_->getScalarFieldValue(sample->getPosition()));
        }

        for (int i = 0; i < real_sp_.size(); ++i)
            residuals_.push_back(real_sp_.at(i) - model_sp_.at(i));
    }
}

void AlignSingleAttitudeStratigraphicModelToSamples::updateShift()
{
    float sum = std::accumulate(residuals_.begin(), residuals_.end(), 0.0);
    float avg = sum / residuals_.size();

    shift_ = avg;

    model_->addShift(shift_);
}

int AlignSingleAttitudeStratigraphicModelToSamples::compute()
{
    updateResiduals();

    if (residuals_.empty())
    {
        pcl::console::print_error("Cannot find any sample with the requested "
                                  "field\n Cannot compute.");
        return -1;
    }

    updateShift();

    updateResiduals(); // do it another time so to recompute the final residuals

    // compute squared sum of residuals


    return 1;

}

} // end nspace
