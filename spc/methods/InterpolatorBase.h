#ifndef INTERPOLATOR_BASE_H
#define INTERPOLATOR_BASE_H

#include <spc/elements/TimeSeriesBase.h>
#include <pcl/console/print.h>
#include <boost/foreach.hpp>
namespace spc {

class InterpolatorBase {
public:
    spcTypedefSharedPtrs(InterpolatorBase)

		InterpolatorBase();

    virtual float getInterpolatedValue(const float x_val) = 0;

    std::vector<float> getInterpolatedValues(const std::vector<float> new_x)
    {
        std::vector<float> out(new_x.size());

        int c = 0;
		for (const float x : new_x) {
            out.at(c++) = getInterpolatedValue(x);
        }
        return out;
    }

    virtual void setInputSeries(TimeSeriesBase::ConstPtr input)
    {
        if (!input) {
            pcl::console::print_error("[ERROR] You are setting a null pointer");
            return;
        }
        input_ = input;

        this->updateInternals();
    }

	virtual void getValidityRange(float& min, float& max)
    {

        if (!input_) {
			LOG(ERROR) << "You must define a valid input time series";
            min = std::numeric_limits<float>::quiet_NaN();
            max = std::numeric_limits<float>::quiet_NaN();
            return;
        }

        min = input_->getMinX();
        max = input_->getMaxX();
    }

    // is automatically called after setting input
    virtual int updateInternals()
    {
        // nothing in this abstract class
        return 1;
    }

protected:
    TimeSeriesBase::ConstPtr input_;
};

} // end nspace

#endif // INTERPOLATOR_BASE_H
