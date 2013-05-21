#include <spc/methods/nn_interpolator.h>

namespace spc
{
InterpolatorNN::InterpolatorNN()
	{
		reset();
	}
	
	int
	InterpolatorNN::compute()
	{
		
		if (is_input_ok_ == false)
		{
			std::cout << "You must set up input vectors first! Remember that both must have same size."<< std::endl;
			return -1;
		}
		int n = new_x_.size();
		std::vector<float> output;
		output.resize(n);
		for (int i = 0; i < n; ++i)
		{
			new_y_[i] = getNearestValue(new_x_[i]);
		}
		return 1;
		is_computed_ = true;
	}

	
int
InterpolatorNN::getNearestID(const float &value)
	{
		int n_id = 0;
		float old_diff = std::numeric_limits<float>::infinity();
		for (int i = 0; i < n_in_; ++i)
		{
			float this_x = x_[i];
			float diff = abs(value - this_x);
			
			if (old_diff > diff)
			{
				
				n_id = i;
				old_diff = diff;
			}
		}
		return n_id;
	}
	
float
InterpolatorNN::getNearestValue(const float &value)
	{
		int n_id = getNearestID(value);
		
		return y_[n_id];
	}
	
	
} //end namespace
