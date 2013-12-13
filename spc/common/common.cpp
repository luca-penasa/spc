#include <spc/common/common.h>
namespace spc {


template<class nType> std::vector<nType> 
subdivideRange(const nType start, const nType end, const nType step)
{
	nType range = std::abs(end-start);
	int n_steps = ceil(range / step) + 1;
	
	std::vector<nType> result;
	result.resize(n_steps);
	
	for (int i = 0; i < n_steps; ++i)
	{
		result[i] = start + i*step;
	}
	return result;
	
	
}



//// INST
template std::vector<float> subdivideRange<float>(const float start, const float end, const float step);
template std::vector<double> subdivideRange<double>(const double start, const double end, const double step);


} //end namespace
