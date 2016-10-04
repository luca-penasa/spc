#include "InterpolatorBase.h"
#include <spc/core/logging.h>
namespace spc
{
InterpolatorBase::InterpolatorBase()
{
}

std::vector<float> InterpolatorBase::getInterpolatedValues(const std::vector<float> new_x)
{
	std::vector<float> out(new_x.size());

	int c = 0;
	for (const float x : new_x) {
		out.at(c++) = getInterpolatedValue(x);
	}
	return out;
}

void InterpolatorBase::setInputSeries(TimeSeriesBase::ConstPtr input)
{
	if (!input) {
		LOG(ERROR) <<"[ERROR] You are setting a null pointer";
		return;
	}
	input_ = input;

	this->updateInternals();
}

void InterpolatorBase::getValidityRange(float &min, float &max)
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

} // end nspace
