#include <spc/common/common.h>
namespace spc {


template<class nType> std::vector<nType> 
subdivideRange(nType &start, nType &end, nType &step)
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

template std::vector<float> subdivideRange<float>(float &start, float &end, float &step);
template std::vector<double> subdivideRange<double>(double &start, double &end, double &step);


} //end namespace
