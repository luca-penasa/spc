#ifndef KERNEL_SMOOTHING_CALIBRATION_MODEL_H
#define KERNEL_SMOOTHING_CALIBRATION_MODEL_H
#include <spc/experimental//ICalModelBase.h>
#include <spc/elements/TimeSeriesBase.h>
#include <spc/methods/InterpolatorSpline.h>

#include <pcl/console/print.h>

namespace spc
{

class ModelDiscretePoints : public CalibrationModelBase
{
public:
    SPC_ELEMENT(ModelDiscretePoints)
EXPOSE_TYPE
    ModelDiscretePoints();

    virtual float getDistanceCorrection(const float &distance) override
    {
        if (!interpolator_)
            setUpInterpolator();

        return interpolator_->getInterpolatedValue(distance);
    }

    void setDiscretePoints(spc::TimeSeriesBase::Ptr points)
    {
        discrete_points_distance_ = points;
    }

    spc::TimeSeriesBase::Ptr getTS()
    {
        return discrete_points_distance_;
    }

protected:
    int setUpInterpolator()
    {
        if (!discrete_points_distance_) {
            pcl::console::print_error("[Error in %s] first you should set up a "
                                      "set of discrete points to be used",
                                      this->getType()->getClassName().c_str());
            return -1;
        }

        // for now only this kind of interpolation is supported
        interpolator_ = spc::InterpolatorSimpleSpline::Ptr(
            new spc::InterpolatorSimpleSpline);

        interpolator_->setInputSeries(discrete_points_distance_);

        return 1;
    }

    ///
    /// \brief discrete_points_distance_ is a TS that set a series of passing
    /// points for
    /// the correction function f(d)= ...
    /// these passing points will be interpolated to get a continous function
    ///
    spc::TimeSeriesBase::Ptr discrete_points_distance_;

    /// we need an interpolator to interpolate between points
    /// any derived interpolator from interpolator base is fine
    spc::InterpolatorBase::Ptr interpolator_;
};

} // end nspace

#endif // KERNEL_SMOOTHING_CALIBRATION_MODEL_H
