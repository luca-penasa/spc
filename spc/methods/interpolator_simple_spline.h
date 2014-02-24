#ifndef INTERPOLATOR_SIMPLE_LINEAR_H
#define INTERPOLATOR_SIMPLE_LINEAR_H

#include <spc/methods/interpolator_base.h>
#include <spc/external/spline.h>

namespace spc
{
///
/// \brief The InterpolatorSimpleLinear class
/// \warning we assume the x values in the input time series are yet sorted
///
class InterpolatorSimpleSpline : public InterpolatorBase
{
public:
    InterpolatorSimpleSpline();

    virtual float getInterpolatedValue(const float x_val) override {
        return spline_(x_val);
    }

    virtual int updateInternals() override {
        // clear current
        spline_ = magnet::math::Spline();

        for (int i = 0; i < input_->getX().size(); ++i )
            spline_.addPoint(input_->getX().at(i), input_->getY().at(i));

        return 1;
    }

protected:
    magnet::math::Spline spline_;
};



}//end nspace
#endif // INTERPOLATOR_SIMPLE_LINEAR_H
