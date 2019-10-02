#ifndef INTERPOLATOR_BASE_H
#define INTERPOLATOR_BASE_H

#include <spc/elements/TimeSeriesBase.h>
//#include <pcl/console/print.h>
#include <boost/foreach.hpp>
namespace spc {

class InterpolatorBase {
public:
    spcTypedefSharedPtrs(InterpolatorBase)

		InterpolatorBase();

    virtual float getInterpolatedValue(const float x_val) = 0;

	std::vector<float> getInterpolatedValues(const std::vector<float> new_x);

	virtual void setInputSeries(TimeSeriesBase::ConstPtr input);

	virtual void getValidityRange(float& min, float& max);

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
