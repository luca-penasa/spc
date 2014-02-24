#ifndef KERNEL_SMOOTHING_CALIBRATION_MODEL_H
#define KERNEL_SMOOTHING_CALIBRATION_MODEL_H
#include <spc/devices/calibration_model_base.h>
#include <spc/time_series/base_time_series.h>
#include <spc/methods/interpolator_simple_spline.h>

#include <pcl/console/print.h>

namespace spc
{

class DiscretePointsCalibrationModel: public CalibrationModelBase, public spcElementBase
{
public:

    typedef boost::shared_ptr<DiscretePointsCalibrationModel> Ptr;
    typedef boost::shared_ptr<const DiscretePointsCalibrationModel> ConstPtr;

    DiscretePointsCalibrationModel();

    virtual float getDistanceCorrection(const float &distance) override {
        if (!interpolator_)
            setUpInterpolator();

        return interpolator_->getInterpolatedValue(distance);
    }


    void setDiscretePoints(spc::GenericTimeSeries<float>::Ptr points)
    {
        discrete_points_distance_ = points;
    }

    spc::GenericTimeSeries<float>::Ptr getTS()
    {
        return discrete_points_distance_;
    }


protected:


    int setUpInterpolator()
    {
        if (!discrete_points_distance_)
        {
            pcl::console::print_error("[Error in %s] first you should set up a set of discrete points to be used", getClassName().c_str());
            return -1;
        }

        // for now only this kind of interpolation is supported
        interpolator_ = spc::InterpolatorSimpleSpline::Ptr(new spc::InterpolatorSimpleSpline);

        interpolator_->setInputSeries(discrete_points_distance_);

        return 1;
    }

    ///
    /// \brief discrete_points_distance_ is a TS that set a series of passing points for
    /// the correction function f(d)= ...
    /// these passing points will be interpolated to get a continous function
    ///
    spc::GenericTimeSeries<float>::Ptr discrete_points_distance_;

    /// we need an interpolator to interpolate between points
    /// any derived interpolator from interpolator base is fine
    spc::InterpolatorBase::Ptr interpolator_;
};


}//end nspace

#endif // KERNEL_SMOOTHING_CALIBRATION_MODEL_H
